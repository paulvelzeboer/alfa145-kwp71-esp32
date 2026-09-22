/*
 * KWP71 reader for Alfa 145 QV (Bosch Motronic M2.10.4)
 * Hardware: ESP32-2432S028 (CYD) + ISO9141 click board
 *
 * Ported 1:1 from a confirmed-working Python/pyserial tool (kwp71_m2_10_4.py)
 * that successfully completed the wakeup + sync + keyword handshake over a
 * KKL/VAG-COM 409.1 cable:
 *
 *   address 0x10, 4800 baud, break-style 5-baud wakeup (200ms/bit),
 *   n_sync=5 bytes read after 0x55, keyword byte = index 1 (0-based),
 *   turnaround 8ms before replying with the complement of the keyword byte.
 *   Confirmed result: keyword 0x86 -> reply 0x79, ECU accepted it.
 *
 * The info-block phase (phase 4, reading ECU ID blocks) was NOT yet
 * confirmed working on the Python side (failed with a desync). This sketch
 * ports the documented KWP71 block echo protocol as best-effort and logs
 * every raw TX/RX byte over Serial so you can diagnose/tune it further on
 * real hardware, same as you were doing with the Python script.
 *
 * Wiring:
 *   GPIO27 -> ISO9141 click TX-in  (K_TX_PIN, ESP32 drives this)
 *   GPIO22 -> ISO9141 click RX-out (K_RX_PIN, ESP32 reads this)
 *   Click K-line -> car diagnostic connector K-line (3-pin Fiat/Alfa: pin K)
 *   Click VS/VBB -> +12V ignition-switched
 *   Click GND    -> chassis ground
 *
 * Needs TFT_eSPI configured for the CYD (ILI9341 240x320).
 */

#include <TFT_eSPI.h>
#include "driver/uart.h"

TFT_eSPI tft = TFT_eSPI();

// ---------- PROTOCOL CONSTANTS (from the confirmed-working Python defaults) ----------
#define K_TX_PIN 27
#define K_RX_PIN 22

#define ECU_ADDRESS   0x10   // Motronic engine ECU - CONFIRMED working
#define COMM_BAUD     4800

#define BIT_MS        200    // 5-baud bit time
#define IDLE_BEFORE_MS 300   // idle-high settle before wakeup
#define IDLE_AFTER_MS  190   // idle-high margin after wakeup

#define N_SYNC        5      // bytes to read after the 0x55 sync byte
#define KW_POS        1      // 0-based index of the keyword byte among those N_SYNC bytes
#define SYNC_TIMEOUT_MS 800  // wait per sync-byte attempt
#define SYNC_TRIES    3
#define BYTE_TIMEOUT_MS 150  // per-byte timeout while reading post-sync bytes
#define TURNAROUND_MS 8      // delay before echoing/replying (lock-step timing)

#define INFO_BLOCKS   2      // number of ECU info blocks expected during init
#define MAX_BLOCK_LEN 64
#define EOB           0x03

#define RETRY_SETTLE_MS 1000 // idle time between failed handshake attempts

// Try RX inversion on/off automatically if plain sync fails a few rounds.
// Some ISO9141 click boards output inverted logic on their RX-out pin
// relative to what a standard idle-high UART framing expects.
bool rxInverted = false;
int consecutiveFails = 0;
#define FAILS_BEFORE_TOGGLE 2

// ---------- STATE ----------
bool ecuConnected = false;
unsigned long lastAttempt = 0;
uint8_t msgCounter = 1;
int lastTx = -1; // last byte WE transmitted, for self-echo discard (matches Python's last_tx)

// ---------- DISPLAY ----------
int logY = 20;
const int logLineHeight = 12;
const int logMaxY = 300;

void logLine(const String &msg, uint16_t color = TFT_GREEN) {
  Serial.println(msg);
  if (logY > logMaxY) {
    tft.fillRect(0, 20, 240, logMaxY - 20 + logLineHeight, TFT_BLACK);
    logY = 20;
  }
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(0, logY);
  tft.print(msg);
  logY += logLineHeight;
}

// Serial-only, fast - used inside timing-critical sections so we never stall
// the lock-step echo with slow SPI/TFT writes.
void dbg(const String &msg) {
  Serial.println(msg);
}

String hexs(const uint8_t *buf, int len) {
  String s;
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10) s += "0";
    s += String(buf[i], HEX);
    if (i != len - 1) s += " ";
  }
  return s;
}

// ---------- LOW-LEVEL K-LINE I/O ----------
uint8_t comp(uint8_t b) {
  return b ^ 0xFF;
}

void kWriteByte(uint8_t b) {
  Serial2.write(b);
  Serial2.flush();
  lastTx = b;
}

// Returns -1 on timeout.
int kReadByte(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      return Serial2.read();
    }
  }
  return -1;
}

void kDrain() {
  while (Serial2.available()) Serial2.read();
}

// ---------- PHASE 1: 5-BAUD BREAK-STYLE WAKEUP ----------
// Bit-bangs the address byte directly on the GPIO (equivalent to the Python
// "break" transport: pull TX low for a '0' bit, let it idle high for a '1'
// bit). Done BEFORE the UART peripheral is attached to K_TX_PIN.
void wakeupBreakBitbang(uint8_t address) {
  pinMode(K_TX_PIN, OUTPUT);
  digitalWrite(K_TX_PIN, HIGH); // idle high
  delay(IDLE_BEFORE_MS);

  digitalWrite(K_TX_PIN, LOW);  // start bit
  delay(BIT_MS);

  for (int i = 0; i < 8; i++) { // data bits, LSB first
    digitalWrite(K_TX_PIN, (address >> i) & 0x01);
    delay(BIT_MS);
  }

  digitalWrite(K_TX_PIN, HIGH); // stop bit
  delay(BIT_MS);
  delay(IDLE_AFTER_MS);
}

// ---------- PHASES 2+3: SYNC BYTE + KEYWORD REPLY ----------
// Returns true on success. Mirrors wakeup_and_sync() from the Python tool.
bool wakeupAndSync(uint8_t address) {
  pinMode(K_RX_PIN, INPUT);
  delay(5);
  int idle = digitalRead(K_RX_PIN);
  logLine("K-line idle level before wakeup: " + String(idle ? "HIGH" : "LOW"),
          idle ? TFT_GREEN : TFT_RED);

  logLine("Wakeup: addr 0x" + String(address, HEX) + " @ 5 baud (invert=" +
          (rxInverted ? "Y" : "N") + ")", TFT_CYAN);
  wakeupBreakBitbang(address);

  // Attach UART at operating baud now for sync/keyword/block phases.
  Serial2.end();
  delay(20);
  Serial2.begin(COMM_BAUD, SERIAL_8N1, K_RX_PIN, K_TX_PIN);
  uart_set_line_inverse(UART_NUM_2, rxInverted ? UART_SIGNAL_RXD_INV : UART_SIGNAL_INV_DISABLE);
  kDrain(); // drop our own wakeup garbage reflected back on the K-line

  logLine("Waiting for 0x55 sync...", TFT_CYAN);
  int sync = -1;
  String seen = "";
  for (int t = 0; t < SYNC_TRIES; t++) {
    int b = kReadByte(SYNC_TIMEOUT_MS);
    if (b >= 0) {
      seen += "0x" + String(b, HEX) + " ";
      if (b == 0x55) { sync = b; break; }
    }
  }
  if (sync != 0x55) {
    if (seen.length() > 0) {
      logLine("FAIL: no 0x55, but saw: " + seen, TFT_ORANGE);
    } else {
      logLine("FAIL: no 0x55 sync byte, nothing received at all", TFT_RED);
    }
    consecutiveFails++;
    if (consecutiveFails >= FAILS_BEFORE_TOGGLE) {
      rxInverted = !rxInverted;
      consecutiveFails = 0;
      logLine("Toggling RX invert to " + String(rxInverted ? "Y" : "N") + " for next attempt", TFT_MAGENTA);
    }
    return false;
  }
  consecutiveFails = 0;
  logLine("0x55 SYNC received", TFT_GREEN);

  uint8_t kw[N_SYNC];
  int kwLen = 0;
  for (int i = 0; i < N_SYNC; i++) {
    int b = kReadByte(BYTE_TIMEOUT_MS);
    if (b < 0) break;
    kw[kwLen++] = (uint8_t)b;
  }
  if (kwLen == 0) {
    logLine("FAIL: no keyword bytes after sync", TFT_RED);
    return false;
  }
  logLine("Bytes after 0x55: " + hexs(kw, kwLen), TFT_YELLOW);

  int kwIdx = (KW_POS < kwLen) ? KW_POS : kwLen - 1;
  uint8_t kw2 = kw[kwIdx];
  if (kw2 == 0) {
    logLine("FAIL: keyword byte is 0x00 - try different KW_POS", TFT_RED);
    return false;
  }
  uint8_t reply = comp(kw2);
  logLine("Keyword[" + String(kwIdx) + "]=0x" + String(kw2, HEX) +
          " -> reply 0x" + String(reply, HEX), TFT_GREEN);

  delay(TURNAROUND_MS);
  kWriteByte(reply);
  delay(100);

  logLine("Handshake OK - addr 0x" + String(address, HEX), TFT_GREEN);
  return true;
}

// ---------- KWP71 BLOCK PROTOCOL ----------
// Every byte received is echoed back inverted by the receiver (except the
// final EOB byte). The bus is single-wire, so our own transmissions loop
// back into our own RX - detect and discard those before treating a byte as
// really coming from the ECU. This mirrors read_block()/send_block()/
// _next_ecu_byte() from the Python tool exactly.

// Read the next ECU byte, discarding a single self-echo if present.
int nextEcuByte(int prevEchoed, unsigned long timeoutMs) {
  int y = kReadByte(timeoutMs);
  if (y < 0 || prevEchoed < 0) return y;
  if (y == comp((uint8_t)prevEchoed)) {
    dbg("  own echo 0x" + String(y, HEX) + " discarded");
    return kReadByte(timeoutMs);
  }
  return y;
}

// Returns block length (>0) on success, 0 on failure. Data (without LEN byte)
// is written into outBuf: [COUNTER, TYPE, data..., EOB].
int readBlock(uint8_t *outBuf, int maxLen, unsigned long timeoutMs = 800) {
  int b = kReadByte(timeoutMs);
  if (b < 0) {
    dbg("  !! no block start (expected LEN byte)");
    return 0;
  }
  int guard = 0;
  while (lastTx >= 0 && b == lastTx && guard < 8) {
    dbg("  own echo 0x" + String(b, HEX) + " discarded");
    b = kReadByte(timeoutMs);
    if (b < 0) {
      dbg("  !! no LEN byte after discarding own echo");
      return 0;
    }
    guard++;
  }
  int length = b;
  if (length == 0 || length > MAX_BLOCK_LEN || length > maxLen) {
    dbg("  !! implausible block length 0x" + String(length, HEX));
    return 0;
  }

  delay(TURNAROUND_MS);
  kWriteByte(comp((uint8_t)b)); // echo complement of LEN

  int prev = b;
  for (int i = 0; i < length; i++) {
    int bb = nextEcuByte(prev, timeoutMs);
    if (bb < 0) {
      dbg("  !! timeout mid-block (wanted byte " + String(i + 1) + "/" + String(length) + ")");
      return 0;
    }
    outBuf[i] = (uint8_t)bb;
    if (i != length - 1) { // last byte = EOB, not echoed
      kWriteByte(comp((uint8_t)bb));
      prev = bb;
    }
  }
  if (outBuf[length - 1] != EOB) {
    dbg("  !! block does not end with 0x03: " + hexs(outBuf, length));
    return 0;
  }
  if (length >= 2) msgCounter = outBuf[0];
  return length;
}

// Sends [len(payload)+2][counter+1][payload...][EOB], expecting the ECU to
// echo the complement of every byte except the final EOB. Returns true if
// every echo matched.
bool sendBlock(const uint8_t *payload, int payloadLen, const char *label) {
  uint8_t block[MAX_BLOCK_LEN];
  int idx = 0;
  block[idx++] = payloadLen + 2;
  block[idx++] = msgCounter + 1;
  for (int i = 0; i < payloadLen; i++) block[idx++] = payload[i];
  block[idx++] = EOB;

  dbg(String("[SEND ") + label + "] " + hexs(block, idx));
  bool ok = true;
  for (int i = 0; i < idx; i++) {
    kWriteByte(block[i]);
    if (i == idx - 1) continue; // no echo expected for final EOB
    int e = kReadByte(600);
    if (e < 0) {
      dbg("  no echo for byte " + String(i));
      ok = false;
      continue;
    }
    if (e == block[i]) { // our own loopback copy
      dbg("  own echo 0x" + String(e, HEX) + " discarded");
      e = kReadByte(600);
      if (e < 0) {
        dbg("  no echo for byte " + String(i));
        ok = false;
        continue;
      }
    }
    uint8_t wanted = comp(block[i]);
    if ((uint8_t)e != wanted) {
      dbg("  echo 0x" + String(e, HEX) + " != 0x" + String(wanted, HEX) + " for byte " + String(i));
      ok = false;
    }
  }
  if (ok) msgCounter++;
  return ok;
}

void describeBlock(uint8_t *blk, int len) {
  if (len < 3) {
    logLine("block: " + hexs(blk, len));
    return;
  }
  uint8_t ctr = blk[0], typ = blk[1];
  String ascii;
  for (int i = 2; i < len - 1; i++) {
    char c = blk[i];
    ascii += (c >= 32 && c < 127) ? c : '.';
  }
  logLine("block ctr=0x" + String(ctr, HEX) + " type=0x" + String(typ, HEX) +
          " data=" + hexs(blk + 2, len - 3) + " '" + ascii + "'", TFT_WHITE);
}

// ---------- PHASE 4: ECU INFO BLOCKS ----------
bool runInit() {
  logLine("Reading " + String(INFO_BLOCKS) + " ECU info blocks...", TFT_CYAN);
  uint8_t buf[MAX_BLOCK_LEN];
  uint8_t nop[] = {0x09};

  for (int i = 0; i < INFO_BLOCKS; i++) {
    int len = readBlock(buf, sizeof(buf));
    if (len == 0) {
      logLine("FAIL: info block " + String(i + 1) + " failed (see Serial log)", TFT_RED);
      return false;
    }
    describeBlock(buf, len);
    sendBlock(nop, 1, "ACK");
  }
  int len = readBlock(buf, sizeof(buf));
  if (len == 0) {
    logLine("FAIL: no final ACK block", TFT_RED);
    return false;
  }
  describeBlock(buf, len);
  return true;
}

// ---------- PARAMETER REQUESTS (from the Python REQ dict - unverified decode) ----------
// Uses output parameters instead of a custom struct return type, since the
// Arduino IDE auto-inserts function prototypes at the top of the file
// (before any struct definitions), which breaks custom struct return types.
bool requestParam(const uint8_t *payload, int payloadLen, const char *label,
                   uint8_t *outBlk, int *outLen) {
  sendBlock(payload, payloadLen, label);
  *outLen = readBlock(outBlk, MAX_BLOCK_LEN);
  return (*outLen > 0);
}

// battery/rpm/coolant/airtemp decode formulas ported from the Python decode()
// function. UNVERIFIED against real hardware (info-block phase wasn't
// confirmed working yet on the PC side either) - treat these as a starting
// point and adjust indices/formulas once you see real block bytes logged
// over Serial.
void pollAndDisplayParams() {
  uint8_t battReq[] = {0x01, 0x01, 0x00, 0x36};
  uint8_t rpmReq[]  = {0x01, 0x02, 0x00, 0x3B};
  uint8_t coolReq[] = {0x08, 0x03};
  uint8_t nop[]     = {0x09};

  uint8_t blk[MAX_BLOCK_LEN];
  int len;

  bool battOk = requestParam(battReq, sizeof(battReq), "BATTERY", blk, &len);
  float battV = -1;
  if (battOk && len >= 4) {
    battV = blk[3] * 0.0681f + 0.0019f;
  }

  bool rpmOk = requestParam(rpmReq, sizeof(rpmReq), "RPM", blk, &len);
  int rpm = -1;
  if (rpmOk && len >= 5) {
    rpm = (int)(0.2f * blk[3] * blk[4]);
  }

  bool coolOk = requestParam(coolReq, sizeof(coolReq), "COOLANT", blk, &len);
  float coolantC = -1000;
  if (coolOk && len >= 5) {
    float x = blk[4];
    coolantC = -0.000014482f * x * x * x + 0.006319247f * x * x - 1.35140625f * x + 144.4095455f;
  }

  sendBlock(nop, 1, "ACK"); // keep-alive

  String line = "Batt: " + (battV > -1 ? String(battV, 2) + "V" : String("--"));
  line += "  RPM: " + (rpm >= 0 ? String(rpm) : String("--"));
  line += "  Cool: " + (coolantC > -999 ? String(coolantC, 1) + "C" : String("--"));
  logLine(line, TFT_WHITE);
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("KWP71 M2.10.4 Reader");

  pinMode(K_TX_PIN, OUTPUT);
  digitalWrite(K_TX_PIN, HIGH);

  lastAttempt = 0;
}

void loop() {
  if (!ecuConnected) {
    if (millis() - lastAttempt > RETRY_SETTLE_MS) {
      lastAttempt = millis();
      msgCounter = 1;
      lastTx = -1;
      bool synced = wakeupAndSync(ECU_ADDRESS);
      if (synced) {
        ecuConnected = runInit();
        if (ecuConnected) {
          logLine("== HANDSHAKE COMPLETE ==", TFT_GREEN);
        }
      }
    }
    return;
  }

  pollAndDisplayParams();
  delay(300);
}

