/*
 * KWP71 reader for Alfa 145 QV (Bosch Motronic M2.10.4, part 0261204478)
 * Hardware: ESP32-2432S028 (CYD) + ISO9141 click board
 *
 * Ported 1:1 from a CONFIRMED FULLY WORKING Python/pyserial tool
 * (kwp71_m2_10_4_fixed.py) that completed the ENTIRE KWP71 init sequence
 * over a KKL/VAG-COM 409.1 cable, including both ECU info blocks and the
 * final session block, then reached a live command REPL:
 *
 *   address 0x10, 4800 baud, break-style 5-baud wakeup (200ms/bit),
 *   n_sync=5 bytes read after 0x55, keyword byte = index 1 (0-based),
 *   turnaround 8ms before replying with the complement of the keyword byte.
 *   Confirmed result: keyword 0x86 -> reply 0x79, ECU accepted it.
 *   Info block 1 payload decoded (ASCII reversed) = "0261204478", the
 *   ECU's own Bosch part number - definitive proof of correct comms.
 *
 * Two real protocol bugs were found and fixed on the Python side, both
 * ported here:
 *   1) LEN/EOB: CORRECTED 22 Sep 2026 from a raw trace of the ESP32 on the
 *      real ECU. LEN counts COUNTER+TYPE+DATA+EOB (standard convention):
 *      block 1 = 0D | 01 F6 "8744021620" 03 (13), block 2 = 04 | 02 0A 01 03.
 *      The earlier "EOB arrives one byte past LEN" finding was an artifact:
 *      the ECU always sends the LEN byte of its first block TWICE (it doesn't
 *      accept the tester's first echo), and the repeat was read as data.
 *      readBlock() now echoes such a repeat without storing it, uses LEN to
 *      find the EOB (a 0x03 before that point is data, e.g. a counter of
 *      0x03), and falls back to scanning for 0x03 up to LEN + BLOCK_SLACK.
 *   2) Self-echo drift bug: since K-line is single-wire, every byte WE
 *      transmit loops back into our own RX and must be discarded before
 *      being treated as ECU data. The old approach deferred/counted this
 *      discard separately from the write, and the count drifted out of
 *      sync across multiple blocks, eventually causing real ECU bytes to be
 *      misidentified and discarded as leftover echo. Fix: kWriteByteSync()
 *      writes a byte and immediately consumes its own physical loopback
 *      echo right there, every time, with no separate counter/state to drift.
 *
 * Also: KWP71 requires continuous bus traffic roughly every 200-300ms or
 * the ECU silently times out and drops the session (looks like a dead bus
 * afterward - zero echo on anything sent). The main loop's parameter-poll
 * exchanges (pollAndDisplayParams(), back to back) satisfy this while
 * connected, so no extra keep-alive task is needed on the ESP32 side.
 *
 * Wiring (mikroBUS pins are named from the click's side, so they CROSS):
 *   GPIO27 (K_TX_PIN) -> click mikroBUS pin labelled "RX" (L9637 TX input)
 *   GPIO22 (K_RX_PIN) <- click mikroBUS pin labelled "TX" (L9637 RX output)
 *   CYD CN1 3.3V/GND  -> click +3.3V/GND, click JP1 set to 3V3 (NOT 5V)
 *   Click K-line -> car diagnostic connector K-line (3-pin Fiat/Alfa: pin K)
 *   Click VS/VBB -> +12V ignition-switched
 *   Click GND    -> chassis ground
 *
 * Needs TFT_eSPI configured for the CYD (ILI9341 240x320).
 */

#include <TFT_eSPI.h>
#include "driver/uart.h"
#include "driver/gpio.h"

TFT_eSPI tft = TFT_eSPI();

// ---------- PROTOCOL CONSTANTS (from the confirmed-working Python defaults) ----------
#define K_TX_PIN 27
#define K_RX_PIN 22
#define TFT_BACKLIGHT_PIN 21   // CYD display backlight, active high

// DEMO_MODE 1: no K-line traffic at all - the dashboard shows simulated
// values (rpm sweeping into the red zone, battery, coolant warming up) so
// the screen can be checked without an ECU. Status bar shows "DEMO".
// Set to 0 for the real ECU.
#define DEMO_MODE 0

#define ECU_ADDRESS   0x10   // Motronic engine ECU - CONFIRMED working
#define COMM_BAUD     4800

#define BIT_MS        200    // 5-baud bit time
#define IDLE_BEFORE_MS 2000  // K-line must be idle high >= 2 s before the start bit

#define N_SYNC        5      // bytes to read after the 0x55 sync byte
#define KW_POS        1      // 0-based index of the keyword byte among those N_SYNC bytes
#define SYNC_TIMEOUT_MS 800  // wait per sync-byte attempt
#define SYNC_TRIES    3
#define BYTE_TIMEOUT_MS 150  // per-byte timeout while reading post-sync bytes
#define TURNAROUND_MS 8      // delay before the keyword reply (0x79) - accepted by the ECU

// Delay before EVERY byte we put on the K-line inside a block exchange: each
// inverted echo in readBlock() and each byte of sendBlock(). A 0 ms echo was
// rejected by the real ECU; 10, 15, 20, 25 and 30 ms all worked (sweep,
// 22 Sep 2026). 10 ms = the fastest tested value (more rpm updates per
// second); go back to 20 ms (the value used for the first full sessions) if
// the ECU starts rejecting echoes.
#define ECHO_DELAY_MS 10
unsigned long echoDelayMs = ECHO_DELAY_MS;
#define SELF_ECHO_TIMEOUT_MS 50 // our own byte loops back through the L9637 within ~1 byte time

#define INFO_BLOCKS   2      // number of ECU info blocks expected during init
#define MAX_BLOCK_LEN 64
#define BLOCK_SLACK   4      // safety cap above LEN while scanning for EOB
#define EOB           0x03
#define ECU_LEN_INCLUDES_EOB 1 // 1 = confirmed on this ECU (raw trace, 22 Sep 2026): LEN counts
                               // COUNTER+TYPE+DATA+EOB. 0 = LEN excludes the 0x03.
#define MAX_LEN_REPEATS 3      // how often the ECU may re-send a block's LEN byte because it
                               // didn't accept our echo (it always does this once for block 1)

#define RETRY_SETTLE_MS 1000 // idle time between the END of one attempt and the next

// ---------- STATE ----------
bool ecuConnected = false;
unsigned long lastAttempt = 0;
uint8_t msgCounter = 0; // counter of the last block on the bus (ECU's first block is 01)
int lastTx = -1; // last byte WE transmitted, for self-echo discard (matches Python's last_tx)
unsigned long selfEchoErrors = 0; // bytes whose loopback was missing or wrong

// ---------- DASHBOARD (TFT, landscape 320 x 240) ----------
// The screen is a fixed dashboard; the detailed log goes to Serial only.
//
//   0..24    status bar: title + connection status
//   left     RPM arc gauge (0-8000, red zone from 6500) with the value inside
//   right    BATTERY and COOLANT boxes
//   214..240 bottom line: ID numbers once connected, else the latest step
//
// Everything is drawn once in dashFrame(); updates only redraw what changed
// (the arc grows/shrinks by the difference, numbers overwrite themselves
// with text padding), so there is no flicker. All drawing happens outside
// the timing-critical K-line exchanges.
#define COL_BG      TFT_BLACK
#define COL_BAR     0x10A2      // status bar background
#define COL_FRAME   0x3186      // box outlines, gauge track
#define COL_LABEL   0x9CD3      // light grey labels
#define COL_REDZONE 0x6000      // dark red track in the red zone

#define G_CX   100              // gauge centre
#define G_CY   124
#define G_R    92               // outer radius
#define G_W    14               // ring thickness
#define G_A0   60               // TFT_eSPI arc angles: 0 = 6 o'clock, clockwise
#define G_A1   300              // 60..300 = 240 degree sweep over the top
#define RPM_MAX   8000
#define RPM_WARN  5000          // yellow from here
#define RPM_RED   6500          // red from here

bool dashReady = false;
int gaugeAngle = G_A0;          // current end of the coloured bar
uint16_t gaugeColor = TFT_GREEN;
String idHw, idSw, idPn;        // from the ECU's ID blocks

// ---- Shared state between the K-line code (loop, core 1) and the display
// task (core 0). ONLY the display task draws on the TFT after setup(); the
// K-line code just hands over values/texts, so SPI drawing can never delay a
// K-line echo. Numbers are single 32-bit words (atomic on the ESP32); the
// Strings are guarded by a mutex.
volatile int   shRpm  = -1;     // target rpm, -1 = no value
volatile float shBatt = -1;     // V, < 0 = no value
volatile float shCool = -1000;  // C, <= -999 = no value
SemaphoreHandle_t dashMutex = NULL;
String   shStatus = "STARTING";
uint16_t shStatusCol = TFT_YELLOW;
bool     shStatusDirty = true;
String   shBottom = "";
uint16_t shBottomCol = COL_LABEL;
bool     shBottomDirty = false;

#define DASH_FRAME_MS   40      // ~25 frames per second
#define RPM_SMOOTHING   0.30f   // fraction of the remaining distance per frame:
                                // ~90% of a new value within ~7 frames (~0.3 s)

int rpmToAngle(int rpm) {
  if (rpm < 0) rpm = 0;
  if (rpm > RPM_MAX) rpm = RPM_MAX;
  return G_A0 + (long)rpm * (G_A1 - G_A0) / RPM_MAX;
}

uint16_t rpmColor(int rpm) {
  return rpm >= RPM_RED ? TFT_RED : rpm >= RPM_WARN ? TFT_YELLOW : TFT_GREEN;
}

// Draws the empty track between two angles (grey, dark red in the red zone).
void gaugeTrack(int a, int b) {
  int aRed = rpmToAngle(RPM_RED);
  if (a < aRed) tft.drawArc(G_CX, G_CY, G_R, G_R - G_W, a, min(b, aRed), COL_FRAME, COL_BG, true);
  if (b > aRed) tft.drawArc(G_CX, G_CY, G_R, G_R - G_W, max(a, aRed), b, COL_REDZONE, COL_BG, true);
}

void gaugeSet(int rpm) {
  int a = rpmToAngle(rpm);
  uint16_t col = rpmColor(rpm);
  if (col != gaugeColor) {                    // zone changed: repaint the whole bar
    if (a > G_A0) tft.drawArc(G_CX, G_CY, G_R, G_R - G_W, G_A0, a, col, COL_BG, true);
    if (a < G_A1) gaugeTrack(a, G_A1);
  } else if (a > gaugeAngle) {                // grow
    tft.drawArc(G_CX, G_CY, G_R, G_R - G_W, gaugeAngle, a, col, COL_BG, true);
  } else if (a < gaugeAngle) {                // shrink
    gaugeTrack(a, gaugeAngle);
  }
  gaugeAngle = a;
  gaugeColor = col;
}

void drawText(const String &txt, int x, int y, uint8_t datum, int font, uint16_t fg,
              uint16_t bg, int padding) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
  tft.setTextPadding(padding);
  tft.drawString(txt, x, y, font);
  tft.setTextPadding(0);
}

// Static parts; called once from setup() before the display task starts.
void dashFrame() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 320, 24, COL_BAR);
  drawText("ALFA 145 QV  M2.10.4", 6, 12, ML_DATUM, 2, TFT_WHITE, COL_BAR, 0);

  // gauge: track, ticks every 1000 rpm, labels 2/4/6 inside, 0 and 8 under the ends
  gaugeTrack(G_A0, G_A1);
  gaugeAngle = G_A0;
  gaugeColor = TFT_GREEN;
  for (int k = 0; k <= 8; k++) {
    float a = (G_A0 + k * (G_A1 - G_A0) / 8) * DEG_TO_RAD;
    float sn = sin(a), cs = cos(a);
    int r1 = G_R - G_W - 3, r2 = r1 - (k % 2 ? 4 : 8);
    tft.drawLine(G_CX - r1 * sn, G_CY + r1 * cs, G_CX - r2 * sn, G_CY + r2 * cs,
                 k * 1000 >= RPM_RED ? TFT_RED : COL_LABEL);
    if (k == 2 || k == 4 || k == 6) {
      int rl = r2 - 9;
      drawText(String(k), G_CX - rl * sn, G_CY + rl * cs, MC_DATUM, 2, COL_LABEL, COL_BG, 0);
    }
  }
  drawText("0", G_CX - 70, G_CY + 58, MC_DATUM, 2, COL_LABEL, COL_BG, 0);
  drawText("8", G_CX + 70, G_CY + 58, MC_DATUM, 2, COL_LABEL, COL_BG, 0);
  drawText("rpm", G_CX, G_CY + 46, MC_DATUM, 2, COL_LABEL, COL_BG, 0);
  drawText("x1000", G_CX, G_CY + 64, MC_DATUM, 1, COL_LABEL, COL_BG, 0);

  // value boxes with static labels and units
  tft.drawRoundRect(204, 30, 112, 86, 6, COL_FRAME);
  drawText("BATTERY", 212, 40, ML_DATUM, 2, COL_LABEL, COL_BG, 0);
  drawText("V", 306, 82, MR_DATUM, 4, COL_LABEL, COL_BG, 0);
  tft.drawRoundRect(204, 124, 112, 86, 6, COL_FRAME);
  drawText("COOLANT", 212, 134, ML_DATUM, 2, COL_LABEL, COL_BG, 0);
  tft.drawCircle(289, 168, 3, COL_LABEL);    // degree sign (built-in fonts have none)
  drawText("C", 308, 176, MR_DATUM, 4, COL_LABEL, COL_BG, 0);

  dashReady = true;
}

// ---- Called from the K-line code: only store, never draw.
void dashStatus(const String &txt, uint16_t color) {
  if (!dashMutex) return;
  xSemaphoreTake(dashMutex, portMAX_DELAY);
  shStatus = txt; shStatusCol = color; shStatusDirty = true;
  xSemaphoreGive(dashMutex);
}

void dashBottom(const String &txt, uint16_t color) {
  if (!dashMutex) return;
  xSemaphoreTake(dashMutex, portMAX_DELAY);
  shBottom = txt; shBottomCol = color; shBottomDirty = true;
  xSemaphoreGive(dashMutex);
}

void dashIds() {
  dashBottom("HW " + idHw + "  SW " + idSw + "  PN " + idPn, COL_LABEL);
}

// battV < 0, rpm < 0, coolantC <= -999 mean "no value" (shown as --).
void dashValues(float battV, int rpm, float coolantC) {
  shBatt = battV;
  shRpm = rpm;
  shCool = coolantC;
}

// ---- Display task (core 0): the only code that draws after setup().
// The rpm gauge and number glide towards the latest reading every frame, so
// the needle moves smoothly although new values arrive only ~3-4x/s.
void dashTask(void *param) {
  float shownRpm = 0;
  int drawnRpm = -2;            // -2 = nothing drawn yet, -1 = "--" drawn
  float drawnBatt = -2, drawnCool = -2;
  for (;;) {
    // status bar and bottom line (copied under the mutex, drawn outside it)
    String st, bt;
    uint16_t sc = 0, bc = 0;
    bool sd = false, bd = false;
    xSemaphoreTake(dashMutex, portMAX_DELAY);
    if (shStatusDirty) { st = shStatus; sc = shStatusCol; sd = true; shStatusDirty = false; }
    if (shBottomDirty) { bt = shBottom; bc = shBottomCol; bd = true; shBottomDirty = false; }
    xSemaphoreGive(dashMutex);
    if (sd) {
      tft.fillRect(170, 0, 150, 24, COL_BAR);
      int w = tft.textWidth(st, 2);
      tft.fillCircle(314 - w - 10, 12, 5, sc);
      drawText(st, 314, 12, MR_DATUM, 2, sc, COL_BAR, 0);
    }
    if (bd) drawText(bt, 6, 227, ML_DATUM, 2, bc, COL_BG, 308);

    // rpm: ease towards the target, redraw only when the shown value changes
    int target = shRpm;
    if (target < 0) {
      shownRpm = 0;
      if (drawnRpm != -1) {
        gaugeSet(0);
        drawText("--", G_CX, G_CY + 8, MC_DATUM, 6, COL_LABEL, COL_BG, tft.textWidth("8888", 6));
        drawnRpm = -1;
      }
    } else {
      shownRpm += (target - shownRpm) * RPM_SMOOTHING;
      if (fabsf(target - shownRpm) < 5) shownRpm = target;
      int r = (int)(shownRpm + 0.5f);
      if (r != drawnRpm) {
        gaugeSet(r);
        int r10 = (r + 5) / 10 * 10;          // number in 10 rpm steps: calmer to read
        drawText(String(r10), G_CX, G_CY + 8, MC_DATUM, 6, TFT_WHITE, COL_BG, tft.textWidth("8888", 6));
        drawnRpm = r;
      }
    }

    float bv = shBatt;
    if (bv != drawnBatt) {
      uint16_t col = bv < 0 ? COL_LABEL : bv < 11.8 ? TFT_RED : bv < 12.2 ? TFT_YELLOW :
                     bv <= 14.8 ? TFT_GREEN : TFT_RED;
      drawText(bv < 0 ? String("--") : String(bv, 2), 288, 82, MR_DATUM, 4, col, COL_BG, 76);
      drawnBatt = bv;
    }
    float cv = shCool;
    if (cv != drawnCool) {
      uint16_t col = cv <= -999 ? COL_LABEL : cv < 60 ? TFT_SKYBLUE : cv <= 105 ? TFT_GREEN : TFT_RED;
      drawText(cv <= -999 ? String("--") : String(cv, 1), 284, 176, MR_DATUM, 4, col, COL_BG, 72);
      drawnCool = cv;
    }

    vTaskDelay(pdMS_TO_TICKS(DASH_FRAME_MS));
  }
}

// Log line: always to Serial; while not connected, also the dashboard's
// bottom line (the ID numbers live there once connected).
void logLine(const String &msg, uint16_t color = TFT_GREEN) {
  Serial.println(msg);
  if (!ecuConnected) dashBottom(msg, color);
}

// ECU ID blocks carry their number as reversed ASCII: "8744021620" -> "0261204478".
String reversedId(const uint8_t *blk, int len) {
  String out;
  for (int i = len - 2; i >= 2; i--) {       // data = blk[2 .. len-2], blk[len-1] = 0x03
    if (blk[i] >= '0' && blk[i] <= '9') out += (char)blk[i];
  }
  return out;
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

// ---------- RAW K-LINE TRACE ----------
// Every byte on the wire, timestamped (us since the end of the wakeup stop
// bit), like the Python tool's --raw-trace. Dumped over Serial whenever a
// block fails, so you can see exactly which byte the ECU stopped after and
// how fast each echo went out. Recording is cheap; printing only happens
// after a failure, outside the timing-critical part.
//   TX = byte we sent   SE = its self-echo (loopback)   RX = byte from the ECU
//   -- = read timeout (nothing arrived)
#define TRACE_MAX 200
uint32_t traceT[TRACE_MAX];
char     traceTag[TRACE_MAX];
uint8_t  traceByte[TRACE_MAX];
int      traceN = 0;
uint32_t traceT0 = 0;
char     readTag = 'R';          // tag for the next kReadByte(): 'R' ECU byte, 'S' self-echo

void traceReset() {
  traceN = 0;
  traceT0 = micros();
}

void traceAdd(char tag, uint8_t b) {
  if (traceN < TRACE_MAX) {
    traceT[traceN] = micros() - traceT0;
    traceTag[traceN] = tag;
    traceByte[traceN] = b;
    traceN++;
  }
}

void traceDump(const char *why) {
  Serial.printf("  ---- raw K-line trace (%s), %d events, t=0 at end of wakeup stop bit ----\n", why, traceN);
  uint32_t prev = 0;
  for (int i = 0; i < traceN; i++) {
    const char *name = traceTag[i] == 'T' ? "TX" : traceTag[i] == 'S' ? "SE" :
                       traceTag[i] == 'R' ? "RX" : "--";
    if (traceTag[i] == '!') {
      Serial.printf("  t=%9.2fms (+%7.2f)  --  timeout\n", traceT[i] / 1000.0, (traceT[i] - prev) / 1000.0);
    } else {
      Serial.printf("  t=%9.2fms (+%7.2f)  %s %02X\n", traceT[i] / 1000.0, (traceT[i] - prev) / 1000.0, name, traceByte[i]);
    }
    prev = traceT[i];
  }
  if (traceN >= TRACE_MAX) Serial.println("  (trace full - later events not recorded)");
  Serial.println("  ---- end of trace ----");
}

// ---------- LOW-LEVEL K-LINE I/O ----------
uint8_t comp(uint8_t b) {
  return b ^ 0xFF;
}

void kWriteByte(uint8_t b) {
  traceAdd('T', b);
  Serial2.write(b);
  Serial2.flush();
  lastTx = b;
}

// Writes one byte and IMMEDIATELY consumes its own physical self-echo
// loopback on the single-wire K-line before returning. Every byte we
// transmit loops back into our own RX exactly once, regardless of whether
// the KWP71 protocol itself expects the ECU to echo it back (e.g. the final
// EOB byte of a sent block is never echoed BY THE ECU, but it still loops
// back to our own RX electrically). Consuming it synchronously - right here,
// right after the write - is the confirmed fix for a real bug found on the
// PC/Python side: deferred/counted echo tracking drifted out of sync across
// multiple blocks and eventually misread genuine ECU bytes as leftover echo.
// The loopback is also the best wiring test there is: if it comes back equal
// to what we sent, GPIO27 -> click -> K-line -> click -> GPIO22 all works.
// Returns false (and logs) when the loopback is missing or different.
bool kWriteByteSync(uint8_t b, unsigned long echoTimeoutMs = SELF_ECHO_TIMEOUT_MS) {
  kWriteByte(b);
  readTag = 'S';
  int echo = kReadByte(echoTimeoutMs); // consume the physical loopback byte
  readTag = 'R';
  if (echo == b) return true;
  selfEchoErrors++;
  if (echo < 0) {
    dbg("  !! no self-echo for 0x" + String(b, HEX) + " (TX not reaching K-line, or click RX -> GPIO22 broken)");
  } else {
    dbg("  !! self-echo 0x" + String(echo, HEX) + " != sent 0x" + String(b, HEX) + " (bus collision or bad K-line edges)");
  }
  return false;
}

// Returns -1 on timeout.
int kReadByte(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      int b = Serial2.read();
      traceAdd(readTag, (uint8_t)b);
      return b;
    }
  }
  traceAdd('!', 0);
  return -1;
}

void kDrain() {
  while (Serial2.available()) Serial2.read();
}

// ---------- PHASE 1: 5-BAUD BREAK-STYLE WAKEUP ----------
// Bit-bangs the address byte directly on the GPIO (equivalent to the Python
// "break" transport: pull TX low for a '0' bit, let it idle high for a '1'
// bit). The UART is detached for the address bits and re-attached at the
// START of the stop bit (its idle-high TX holds the stop bit for us), so it
// is already listening the moment the stop bit ends. The previous version
// only attached it ~210 ms after the stop bit, which can miss or cut the
// ECU's 0x55 (W1 may be anywhere in 20-300 ms).
void wakeupBreakBitbang(uint8_t address) {
  Serial2.end();                // release both pins from UART2 before GPIO use
  pinMode(K_TX_PIN, OUTPUT);
  digitalWrite(K_TX_PIN, HIGH); // idle high
  delay(IDLE_BEFORE_MS);

  // Debug: sniff GPIO22 (RX) during our own TX pulse. Single-wire K-line
  // means our own transmission should loop back and be visible on RX - if
  // we see zero transitions here, the TX side isn't reaching the K-line at
  // all (GPIO27/click-board-TX-in/wiring fault), independent of protocol
  // timing or ECU behavior.
  pinMode(K_RX_PIN, INPUT_PULLUP);
  int lastRxLevel = digitalRead(K_RX_PIN);
  int rxTransitions = 0;

  digitalWrite(K_TX_PIN, LOW);  // start bit
  delay(BIT_MS);

  for (int i = 0; i < 8; i++) { // data bits, LSB first
    digitalWrite(K_TX_PIN, (address >> i) & 0x01);
    delay(BIT_MS);
    int rx = digitalRead(K_RX_PIN);
    if (rx != lastRxLevel) { rxTransitions++; lastRxLevel = rx; }
  }

  // Stop bit: attach the UART now; its TX idles high = the stop bit level.
  // Do NOT call pinMode()/digitalWrite() on either pin after this - that
  // detaches the pin from the UART (real bug seen earlier: zero bytes ever
  // received). gpio_pullup_en only touches the pad pull, not the routing.
  digitalWrite(K_TX_PIN, HIGH);
  Serial2.begin(COMM_BAUD, SERIAL_8N1, K_RX_PIN, K_TX_PIN);
  Serial2.setRxFIFOFull(1);     // make every byte readable at once (default adds ~2 byte times)
  gpio_pullup_en((gpio_num_t)K_RX_PIN);
  delay(BIT_MS);
  kDrain();                     // drop anything the UART saw during the stop bit
  traceReset();                 // trace times are relative to this moment

  // Printed AFTER the stop bit is sent but BEFORE we read: fast Serial-only.
  dbg("[DEBUG] RX transitions seen during our own TX pulse: " + String(rxTransitions) +
      " (expected 3 for 0x10; 0 = TX not reaching K-line or click RX -> GPIO22 broken)");
}

// ---------- PHASES 2+3: SYNC BYTE + KEYWORD REPLY ----------
// Returns true on success. Mirrors wakeup_and_sync() from the Python tool.
bool wakeupAndSync(uint8_t address) {
  Serial2.end();                   // UART may still own the pins from the last attempt
  pinMode(K_RX_PIN, INPUT_PULLUP); // avoid floating-pin noise (was misread as spurious 0x00 bytes)
  delay(5);
  int idle = digitalRead(K_RX_PIN);
  logLine("K-line idle level before wakeup: " + String(idle ? "HIGH" : "LOW"),
          idle ? TFT_GREEN : TFT_RED);

  logLine("Wakeup: addr 0x" + String(address, HEX) + " @ 5 baud", TFT_CYAN);
  // Returns with the UART attached at COMM_BAUD and listening from the end
  // of the stop bit. No TFT writes until the sync bytes are in the buffer.
  wakeupBreakBitbang(address);

  dbg("Waiting for 0x55 sync...");
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
    return false;
  }

  // Read the keyword bytes before any (slow) TFT output.
  uint8_t kw[N_SYNC];
  int kwLen = 0;
  for (int i = 0; i < N_SYNC; i++) {
    int b = kReadByte(BYTE_TIMEOUT_MS);
    if (b < 0) break;
    kw[kwLen++] = (uint8_t)b;
  }
  dbg("0x55 SYNC received");
  if (kwLen == 0) {
    traceDump("no keyword bytes");
    logLine("FAIL: no keyword bytes after sync", TFT_RED);
    return false;
  }
  int kwIdx = (KW_POS < kwLen) ? KW_POS : kwLen - 1;
  uint8_t kw2 = kw[kwIdx];
  if (kw2 == 0) {
    logLine("Bytes after 0x55: " + hexs(kw, kwLen), TFT_YELLOW);
    logLine("FAIL: keyword byte is 0x00 - try different KW_POS", TFT_RED);
    return false;
  }
  uint8_t reply = comp(kw2);

  // Reply first, then do the slow TFT logging inside the 100 ms settle the
  // Python tool also waits here.
  delay(TURNAROUND_MS);
  bool echoOk = kWriteByteSync(reply);
  unsigned long replied = millis();

  logLine("Bytes after 0x55: " + hexs(kw, kwLen), TFT_YELLOW);
  logLine("Keyword[" + String(kwIdx) + "]=0x" + String(kw2, HEX) +
          " -> reply 0x" + String(reply, HEX), TFT_GREEN);
  if (!echoOk) {
    logLine("WARN: no clean self-echo of the reply (see Serial log)", TFT_ORANGE);
  }
  logLine("Handshake OK - addr 0x" + String(address, HEX), TFT_GREEN);
  while (millis() - replied < 100) { }
  return true;
}

// ---------- KWP71 BLOCK PROTOCOL ----------
// Every byte received is echoed back inverted by the receiver (except the
// final EOB byte). The bus is single-wire, so our own transmissions loop
// back into our own RX - detect and discard those before treating a byte as
// really coming from the ECU. This mirrors read_block()/send_block()/
// _next_ecu_byte() from the Python tool exactly.

// Returns block length (>0) on success, 0 on failure. Data (without LEN byte)
// is written into outBuf: [COUNTER, TYPE, data..., EOB].
// LEN decides where the block ends: on this ECU LEN covers COUNTER+TYPE+DATA
// only and the EOB (0x03) comes as one extra byte after them. A 0x03 BEFORE
// that point is data, not the end - counters and values can be 0x03 (the
// ECU's 2nd block has counter 0x03 with the usual 01/02/03 numbering, and the
// "08 03" coolant request carries one), and stopping there used to leave that
// byte un-echoed and kill the session. If the byte at the expected EOB
// position isn't 0x03, it keeps echoing and scanning for a 0x03 up to
// LEN + BLOCK_SLACK bytes as a fallback. All self-echo of bytes WE transmit is
// consumed synchronously inside kWriteByteSync(), so every byte kReadByte()
// returns here is a genuine ECU byte - no separate self-echo filtering needed.
int readBlock(uint8_t *outBuf, int maxLen, unsigned long timeoutMs = 800) {
  int b = kReadByte(timeoutMs);
  if (b < 0) {
    dbg("  !! no block start (expected LEN byte)");
    { traceDump("readBlock failed"); return 0; }
  }
  int length = b;
  if (length == 0 || length > MAX_BLOCK_LEN || length > maxLen) {
    dbg("  !! implausible block length 0x" + String(length, HEX));
    { traceDump("readBlock failed"); return 0; }
  }

  delay(echoDelayMs);
  kWriteByteSync(comp((uint8_t)b)); // echo complement of LEN

  // Number of COUNTER+TYPE+DATA bytes before the EOB.
  const int bodyLen = ECU_LEN_INCLUDES_EOB ? length - 1 : length;

  const uint8_t expectedCtr = (uint8_t)(msgCounter + 1);
  int repeats = 0;
  int n = 0;
  int maxReads = length + BLOCK_SLACK;
  if (maxReads > maxLen) maxReads = maxLen;
  for (int i = 0; i < maxReads; i++) {
    int bb = kReadByte(timeoutMs);
    if (bb < 0) {
      dbg("  !! timeout mid-block (read " + String(n) + " bytes so far, EOB expected after " +
          String(bodyLen) + ")");
      { traceDump("readBlock failed"); return 0; }
    }
    // The ECU re-sends the LEN byte when it didn't accept our echo of it
    // (always once for its first block). Echo the repeat again, don't store
    // it. A LEN value that happens to equal the expected counter is taken as
    // the counter.
    if (n == 0 && bb == length && bb != expectedCtr && repeats < MAX_LEN_REPEATS) {
      repeats++;
      i--;
      delay(echoDelayMs);
      kWriteByteSync(comp((uint8_t)bb));
      continue;
    }
    if (bb == EOB && n >= bodyLen) {
      // EOB reached - protocol says do NOT echo it, stop here.
      outBuf[n++] = (uint8_t)bb;
      break;
    }
    if (n == bodyLen) {
      dbg("  !! expected EOB after " + String(bodyLen) + " bytes (LEN=0x" + String(length, HEX) +
          "), got 0x" + String(bb, HEX) + " - scanning on");
    }
    outBuf[n++] = (uint8_t)bb;
    delay(echoDelayMs);
    kWriteByteSync(comp((uint8_t)bb));
  }
  if (outBuf[n - 1] != EOB) {
    dbg("  !! never saw EOB (0x03) within " + String(maxReads) + " bytes after LEN=0x" +
        String(length, HEX) + ": " + hexs(outBuf, n));
    { traceDump("readBlock failed"); return 0; }
  }
  if (repeats) {
    dbg("  (ECU repeated LEN 0x" + String(length, HEX) + " " + String(repeats) + "x before accepting our echo)");
  }
  if (n >= 2 && outBuf[0] != expectedCtr) {
    dbg("  note: ECU counter 0x" + String(outBuf[0], HEX) + ", expected 0x" + String(expectedCtr, HEX));
  }
  if (n >= 2) msgCounter = outBuf[0];
  return n;
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
    // kWriteByteSync writes the byte and immediately consumes its own
    // physical self-echo loopback - this is ALWAYS present regardless of
    // whether the ECU also sends a protocol-level echo.
    delay(echoDelayMs);
    kWriteByteSync(block[i]);
    if (i == idx - 1) continue; // final byte (EOB): ECU never echoes it
    int e = kReadByte(600);
    if (e < 0) {
      dbg("  no echo for byte " + String(i));
      ok = false;
      continue;
    }
    uint8_t wanted = comp(block[i]);
    if ((uint8_t)e != wanted) {
      dbg("  echo 0x" + String(e, HEX) + " != 0x" + String(wanted, HEX) + " for byte " + String(i));
      ok = false;
    }
  }
  if (ok) msgCounter++;
  else traceDump(label);
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
    if (i == 0) idHw = reversedId(buf, len);
    if (i == 1) idSw = reversedId(buf, len);
    sendBlock(nop, 1, "ACK");
  }
  int len = readBlock(buf, sizeof(buf));
  if (len == 0) {
    logLine("FAIL: no final ACK block", TFT_RED);
    return false;
  }
  describeBlock(buf, len);
  idPn = reversedId(buf, len);
  return true;
}

// ---------- PARAMETER REQUESTS (from the Python REQ dict - unverified decode) ----------
// Uses output parameters instead of a custom struct return type, since the
// Arduino IDE auto-inserts function prototypes at the top of the file
// (before any struct definitions), which breaks custom struct return types.
bool requestParam(const uint8_t *payload, int payloadLen, const char *label,
                   uint8_t *outBlk, int *outLen) {
  bool sent = sendBlock(payload, payloadLen, label);
  *outLen = readBlock(outBlk, MAX_BLOCK_LEN);
  // Log the ECU's reply (Serial only; it's our turn to send next, so a few
  // ms of printing here don't disturb the ECU's timing).
  if (*outLen >= 3) {
    uint8_t typ = outBlk[1];
    String what = typ == 0x0A ? " (NAK - request not accepted)" :
                  typ == 0x09 ? " (ACK)" : "";
    dbg(String("  [REPLY ") + label + "] ctr=" + hexs(outBlk, 1) + " type=" + hexs(outBlk + 1, 1) +
        " data=" + (*outLen > 3 ? hexs(outBlk + 2, *outLen - 3) : String("-")) + what);
  } else if (*outLen > 0) {
    dbg(String("  [REPLY ") + label + "] " + hexs(outBlk, *outLen));
  }
  return sent && (*outLen > 0);
}

// battery/rpm/coolant/airtemp decode formulas ported from the Python decode()
// function. UNVERIFIED against real hardware - the PC-side session got past
// the info blocks and reached the REPL, but the "rpm" test command hit a bus
// timeout there because the REPL wasn't sending keep-alive traffic while
// waiting on keyboard input (fixed on the Python side with a background
// keep-alive thread). Actual parameter READS haven't been confirmed yet on
// either side. Treat these formulas as a starting point and adjust
// indices/formulas once you see real block bytes logged over Serial.
// KWP71 strictly alternates: every block we send is answered by exactly one
// ECU block - including our keep-alive NOP, which the ECU answers with its own
// ACK. That answer MUST be read before we send again, or our next request
// collides with it on the bus (seen against m2104_sim.py: session lost after
// every first poll cycle).
// Returns false - after the FIRST failed exchange, so we don't keep pushing
// requests into a dead bus at 600 ms per unanswered byte - when the session
// looks lost.
// One request/reply exchange per call - no pause between calls: the requests
// themselves keep the session alive. RPM is read in 8 of every 10 exchanges
// (~3-4 new values per second); battery and coolant change slowly and are
// read once per 10 exchanges each. Values that fail to decode (e.g. the
// NAK the ECU sends for the first request after the handshake) keep their
// previous reading. Returns false after the first failed exchange.
unsigned long pollN = 0;
int   lastRpm = -1;
float lastBatt = -1, lastCool = -1000;

bool pollAndDisplayParams() {
  uint8_t battReq[] = {0x01, 0x01, 0x00, 0x36};
  uint8_t rpmReq[]  = {0x01, 0x02, 0x00, 0x3B};
  uint8_t coolReq[] = {0x08, 0x03};

  uint8_t blk[MAX_BLOCK_LEN];
  int len;
  int slot = pollN++ % 10;

  if (slot == 4) {
    if (!requestParam(battReq, sizeof(battReq), "BATTERY", blk, &len)) return false;
    // Reply layout: [counter, type, data..., 03] - first data byte is blk[2].
    // Battery: RAM read -> type 0xFE + 1 byte. Calibrated 22 Sep 2026 on the
    // bench: 0x5E (94) at 12.4 V -> 0.1319 V per step (~0.13 V resolution).
    if (len >= 4 && blk[1] == 0xFE) lastBatt = blk[2] * 0.1319f;
  } else if (slot == 9) {
    if (!requestParam(coolReq, sizeof(coolReq), "COOLANT", blk, &len)) return false;
    // Coolant: ADC read -> type 0xFB + 2 bytes (hi, lo); the formula uses the
    // low byte. 0x00BB gave ~18 C at ~20 C room temperature (22 Sep 2026).
    if (len >= 5 && blk[1] == 0xFB) {
      float x = blk[3];
      lastCool = -0.000014482f * x * x * x + 0.006319247f * x * x - 1.35140625f * x + 144.4095455f;
    }
  } else {
    if (!requestParam(rpmReq, sizeof(rpmReq), "RPM", blk, &len)) return false;
    // RPM: RAM read -> type 0xFE + 2 bytes. Reads 00 00 with the engine off;
    // the formula (from the Python tool) is NOT verified with a running engine.
    if (len >= 5 && blk[1] == 0xFE) lastRpm = (int)(0.2f * blk[2] * blk[3]);
  }
  dashValues(lastBatt, lastRpm, lastCool);

  if (slot == 9) {                   // one summary line per 10 exchanges
    String line = "Batt: " + (lastBatt > -1 ? String(lastBatt, 2) + "V" : String("--"));
    line += "  RPM: " + (lastRpm >= 0 ? String(lastRpm) : String("--"));
    line += "  Cool: " + (lastCool > -999 ? String(lastCool, 1) + "C" : String("--"));
    logLine(line, TFT_WHITE);
  }
  return true;
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);

  tft.init();
  // TFT_eSPI's ST7789 init switches colour inversion ON; this CYD panel
  // needs it OFF, otherwise black shows as white (and every colour inverted).
  tft.invertDisplay(false);
  // Backlight: TFT_eSPI only switches it on when TFT_BACKLIGHT_ON is defined
  // in User_Setup.h, which it isn't here (TFT_BL 21 only) - so do it here.
  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH);
  tft.setRotation(1);
  // Colour order: this panel is BGR, TFT_eSPI's setup defaults to RGB (red
  // and blue swapped). Rewrite the orientation register the way setRotation(1)
  // does for the ST7789 (MX | MV), but with the BGR bit. Must come AFTER
  // setRotation(), which overwrites this register.
  tft.writecommand(TFT_MADCTL);
  tft.writedata(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_BGR);
  tft.setTextSize(1);
  dashMutex = xSemaphoreCreateMutex();
  dashFrame();
  dashStatus("STARTING", TFT_YELLOW);
  // All drawing from here on happens in dashTask on core 0; loop() (K-line)
  // runs on core 1, so screen updates never delay a K-line echo.
  xTaskCreatePinnedToCore(dashTask, "dash", 6144, NULL, 1, NULL, 0);

  pinMode(K_TX_PIN, OUTPUT);
  digitalWrite(K_TX_PIN, HIGH);

#if DEMO_MODE
  dashStatus("DEMO", TFT_CYAN);
  idHw = "0261204478";               // the real ECU's IDs, for the look
  idSw = "1037357941";
  idPn = "46525168";
  dashIds();
  Serial.println("DEMO_MODE 1: simulated values, no K-line traffic");
#endif

  lastAttempt = 0;
}

#if DEMO_MODE
// One demo frame: rpm sweeps 800 -> 7600 -> 800 every 8 s (cosine, so it
// eases at both ends), battery 12.4 V at idle / ~14.1 V charging above
// 1000 rpm, coolant warms from 20 to 90 C over the first minute.
void demoStep() {
  float t = millis() / 1000.0f;
  int rpm = 800 + (int)((7600 - 800) * (0.5f - 0.5f * cos(2 * PI * t / 8.0f)));
  float battV = rpm > 1000 ? 14.1f + 0.05f * sin(t * 3.0f) : 12.4f;
  float warm = t / 60.0f;
  if (warm > 1.0f) warm = 1.0f;
  float coolantC = 20.0f + 70.0f * warm + (warm >= 1.0f ? 1.5f * sin(t / 5.0f) : 0.0f);
  dashValues(battV, rpm, coolantC);
  delay(300);                        // ~ the real rpm update rate; the display
                                     // task animates in between
}
#endif

void loop() {
#if DEMO_MODE
  demoStep();
  return;
#endif

  if (!ecuConnected) {
    if (millis() - lastAttempt > RETRY_SETTLE_MS) {
      msgCounter = 0;
      lastTx = -1;
      dashStatus("CONNECTING", TFT_YELLOW);
      bool synced = wakeupAndSync(ECU_ADDRESS);
      if (synced) {
        logLine("Echo delay " + String(echoDelayMs) + " ms", TFT_CYAN);
        ecuConnected = runInit();
        if (ecuConnected) {
          logLine("== HANDSHAKE COMPLETE (echo delay " + String(echoDelayMs) + " ms) ==", TFT_GREEN);
          dashStatus("CONNECTED", TFT_GREEN);
          dashIds();
        } else {
          dashStatus("HANDSHAKE FAIL", TFT_ORANGE);
        }
      } else {
        dashStatus("NO ECU", TFT_RED);
      }
      // Settle is measured from the END of the attempt; the wakeup itself
      // adds another IDLE_BEFORE_MS of idle-high before the start bit.
      lastAttempt = millis();
    }
    return;
  }

  if (!pollAndDisplayParams()) {
    logLine("Session lost - self-echo errors so far: " + String(selfEchoErrors) +
            " - reconnecting", TFT_RED);
    ecuConnected = false;
    dashStatus("RECONNECTING", TFT_ORANGE);
    lastRpm = -1; lastBatt = -1; lastCool = -1000;
    dashValues(-1, -1, -1000);
    lastAttempt = millis();
    return;
  }
}
