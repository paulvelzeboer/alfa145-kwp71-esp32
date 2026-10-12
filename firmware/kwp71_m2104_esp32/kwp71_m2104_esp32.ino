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
// Badge bitmaps: optional. Generate logo_data.h from your own image with
//   python3 tools/make_logo_header.py logo.png
// It is not in the repository (car maker logos are trademarks). Without it
// the splash is text-only and the status bar has no badge.
#if defined(__has_include)
#  if __has_include("logo_data.h")
#    include "logo_data.h"
#    define HAVE_LOGO 1
#  endif
#endif
#include "driver/uart.h"
#include "driver/gpio.h"

TFT_eSPI tft = TFT_eSPI();

// ---------- PROTOCOL CONSTANTS (from the confirmed-working Python defaults) ----------
#define K_TX_PIN 27
#define K_RX_PIN 22
#define TFT_BACKLIGHT_PIN 21   // CYD display backlight, active high

// Hardware already on the CYD that the dashboard now uses. The RGB LED is
// common anode, so LOW switches a colour ON.
#define RGB_R_PIN 4
#define RGB_G_PIN 16
#define RGB_B_PIN 17
#define LDR_PIN   34           // light sensor, ADC1 channel 6

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

// Battery: the ECU's scaled value, one byte, no offset. 0.073 V per step is a
// two-point fit from the discovery runs of 12 Oct 2026, where the raw byte was
// 164-165 with the ignition on and the engine off, and 197-200 with the engine
// running: 12.0 V and 14.5 V, which is a battery at rest and an alternator
// charging. The ratio between the two states (1.20) matches a plain
// proportional scale, so there is no offset term.
// An earlier single point (raw 94 at a measured 12.4 V, 22 Sep 2026) gave 0.1319
// and read 26 V on the car; raw 94 would be 6.9 V here, so that sample was
// almost certainly taken before the reply byte index was fixed. Still worth
// confirming once against a multimeter - the [BATT] log line prints the raw.
#define BATT_SCALE  0.0730f
#define BATT_OFFSET 0.0f
#define BATT_MIN_PLAUSIBLE 6.0f   // outside this range the log flags the reading
#define BATT_MAX_PLAUSIBLE 16.0f

// ADC channels, identified with tools/kwp71_discover.py on 12 Oct 2026 by
// sweeping all 16 channels in different engine states:
//   00 throttle (23 closed, 209 pressed)   01 battery (same value as RAM 0036)
//   03 coolant                             04 air quantity
//   05 intake air temperature              07 lambda (rises with load)
//   06 changes with load, not identified yet.
//   02, 08 and 0B sit at 254-255 and 09, 0A at 0: unused inputs. 0C-0F do not
//   answer at all.
#define ADC_COOLANT 0x03
#define ADC_AIRQTY  0x04
#define ADC_AIRTEMP 0x05
// Air quantity is a raw count: 6 with the engine off, ~41 at idle, ~70 at
// 3000 rpm. AIRQ_ZERO is the no-flow reading; set AIRQ_SCALE once there is
// something to calibrate kg/h against, and change the cell's label from "raw".
// (For a 2.0 litre engine the idle and 3000 rpm points suggest roughly
// 0.37 kg/h per count, but that is an expectation, not a measurement.)
#define AIRQ_ZERO  6
#define AIRQ_SCALE 1.0f

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
//   right    2 x 2 grid: BATTERY | COOLANT over AIR TEMP | AIR QTY
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

// L-shaped rpm bar, wrapping the value cells: it runs along the bottom edge
// for 0..RPM_MID, turns at the bottom-right corner, and climbs the right edge
// for RPM_MID..RPM_MAX. Each leg carries half the range, so the corner is
// always the halfway mark - the bottom leg is longer, so the lower half of the
// range reads finer than the upper half. The scale labels make the split clear.
#define G_W      14             // bar thickness
#define G_BX0    6              // bottom leg: left end, 0 rpm
#define G_BX1    314            // bottom leg: right end, the corner
#define G_BY     190            // bottom leg: top edge
#define G_VX     300            // side leg: left edge (G_BX1 - G_W)
#define G_VY0    190            // side leg: bottom, the corner (= G_BY)
#define G_VY1    30             // side leg: top, RPM_MAX

// The big rpm number sits left of the cells, in the space the dial used to fill.
// Font 7 is the library's 7-segment face: 32 px per digit, 48 px tall, so
// "8888" is 128 px and lands at x 3-131, clear of the cells at 134. A fifth
// digit would not fit, which is why the number is clamped to RPM_MAX below.
#define RPM_CX   67
#define RPM_CY   108
#define RPM_FONT 7

// 2 x 2 value grid, inside the L: 0 = battery, 1 = coolant, 2 = air temp, 3 = air qty
#define PANEL_X  134
#define PANEL_Y  30
#define CELL_W   74
#define CELL_H   76
#define CELL_GAP 4
#define CELL_BATT 0
#define CELL_COOL 1
#define CELL_AIRT 2
#define CELL_AIRQ 3
#define RPM_MAX   8000
#define RPM_MID   (RPM_MAX / 2) // rpm at the corner, where the bar turns
#define RPM_WARN  5000          // yellow from here
#define RPM_RED   6500          // red from here

bool dashReady = false;
int gaugeBottom = 0;            // drawn length of each leg, in pixels
int gaugeSide = 0;
uint16_t gaugeColor = TFT_GREEN;
bool gaugeFullRepaint = false;  // gaugeSet() repainted both legs, so the
                                // overlays drawn on top of them are gone
int tachoX = 0, tachoY = 0;     // where dashFrame() put the rpm dial
String idHw, idSw, idPn;        // from the ECU's ID blocks

// ---- Shared state between the K-line code (loop, core 1) and the display
// task (core 0). ONLY the display task draws on the TFT after setup(); the
// K-line code just hands over values/texts, so SPI drawing can never delay a
// K-line echo. Numbers are single 32-bit words (atomic on the ESP32); the
// Strings are guarded by a mutex.
volatile int   shRpm  = -1;     // target rpm, -1 = no value
volatile float shBatt = -1;     // V, < 0 = no value
volatile float shCool = -1000;  // C, <= -999 = no value
volatile float shAirT = -1000;  // air temperature, C
volatile float shAirQ = -1;     // air quantity, raw ECU value
SemaphoreHandle_t dashMutex = NULL;
String   shStatus = "STARTING";
uint16_t shStatusCol = TFT_YELLOW;
bool     shStatusDirty = true;
bool     shStatusPulse = true;  // breathe the dot until the session is up
String   shBottom = "";
uint16_t shBottomCol = COL_LABEL;
bool     shBottomDirty = false;

// Animation. All of it runs in the display task on core 0, so none of it can
// delay a K-line echo.
#define BOOT_SWEEP_STEPS 28     // cluster-style self-test sweep after the splash
#define BOOT_SWEEP_MS    20     // ~0.6 s each way
#define PEAK_HOLD_MS     1200   // how long the peak tick waits before sliding back
#define PEAK_FALL        40     // rpm per frame once it slides (~1000 rpm/s)
#define PEAK_MIN_GAP     120    // hide the tick this close to the bar's own tip
#define PEAK_W           3      // tick thickness
#define SHIFT_BLINK_MS   125    // shift light: one phase, so ~4 Hz
#define SHIFT_W          16     // length of the blinking stretch at the bar's tip
#define SHIFT_TEXT       "Shift!"   // blinks in the top bar past RPM_RED
#define DOT_PULSE_MS     1400   // breathing period of the status dot
#define FLASH_FRAMES     8      // frames a cell title stays lit after a new value

// Backlight: PWM, following the light sensor. LDR_DARK and LDR_BRIGHT are the
// raw readings at the two ends; SWAP THEM if the screen dims in daylight
// instead of at night, because that is this board's sensor polarity.
#define BL_FREQ      5000
#define BL_BITS      8
#define BL_MIN       40         // never let it go fully dark
#define BL_MAX       255
#define LDR_DARK     3000
#define LDR_BRIGHT   300
#define LDR_SMOOTH   0.05f      // slow, so headlights sweeping past don't flicker it
#define LDR_STEP     4          // smallest duty change worth writing

// Rolling rpm history, in the space under the big number. One sample per
// SPARK_MS across SPARK_PW columns gives about 30 s of history.
#define SPARK_X  6
#define SPARK_Y  138
#define SPARK_W  122
#define SPARK_H  48
#define SPARK_PX (SPARK_X + 2)          // plot area inside the frame
#define SPARK_PY (SPARK_Y + 3)
#define SPARK_PW (SPARK_W - 4)
#define SPARK_PH (SPARK_H - 6)
#define SPARK_MS 250

#define SPLASH_MS       5000    // how long the boot splash stays on screen
#define DASH_FRAME_MS   40      // ~25 frames per second
#define RPM_SMOOTHING   0.18f   // fraction of the remaining distance per frame:
                                // ~90% of a new value within ~12 frames (~0.5 s).
                                // Lower = smoother but lags more; 0.30 was the
                                // first setting and looked slightly too twitchy.

uint16_t rpmColor(int rpm) {
  return rpm >= RPM_RED ? TFT_RED : rpm >= RPM_WARN ? TFT_YELLOW : TFT_GREEN;
}

// How far along each leg an rpm reaches, in pixels. Below RPM_MID only the
// bottom leg moves; above it the bottom leg stays full and the side leg grows.
int rpmBottomLen(int rpm) {
  if (rpm <= 0) return 0;
  if (rpm >= RPM_MID) return G_BX1 - G_BX0;
  return (long)rpm * (G_BX1 - G_BX0) / RPM_MID;
}

int rpmSideLen(int rpm) {
  if (rpm <= RPM_MID) return 0;
  if (rpm >= RPM_MAX) return G_VY0 - G_VY1;
  return (long)(rpm - RPM_MID) * (G_VY0 - G_VY1) / (RPM_MAX - RPM_MID);
}

// Screen y of an rpm on the side leg. The leg climbs, so a higher rpm is a
// SMALLER y - worth remembering when reading the track code below.
int sideY(int rpm) { return G_VY0 - rpmSideLen(rpm); }

// The empty track: grey, turning dark red past RPM_RED, as the ring did.
void trackBottom(int x0, int x1) {
  if (x1 > x0) tft.fillRect(x0, G_BY, x1 - x0, G_W, COL_FRAME);
}

void trackSide(int yTop, int yBot) {
  int yRed = sideY(RPM_RED);
  if (yTop < yRed) {                          // above the red line
    int b = min(yBot, yRed);
    if (b > yTop) tft.fillRect(G_VX, yTop, G_W, b - yTop, COL_REDZONE);
  }
  if (yBot > yRed) {
    int t = max(yTop, yRed);
    if (yBot > t) tft.fillRect(G_VX, t, G_W, yBot - t, COL_FRAME);
  }
}

void gaugeSet(int rpm) {
  int nb = rpmBottomLen(rpm), ns = rpmSideLen(rpm);
  uint16_t col = rpmColor(rpm);
  if (col != gaugeColor) {                    // zone changed: repaint both legs
    if (nb) tft.fillRect(G_BX0, G_BY, nb, G_W, col);
    trackBottom(G_BX0 + nb, G_BX1);
    if (ns) tft.fillRect(G_VX, G_VY0 - ns, G_W, ns, col);
    trackSide(G_VY1, G_VY0 - ns);
  } else {                                    // only the changed stretch
    if (nb > gaugeBottom) tft.fillRect(G_BX0 + gaugeBottom, G_BY, nb - gaugeBottom, G_W, col);
    else if (nb < gaugeBottom) trackBottom(G_BX0 + nb, G_BX0 + gaugeBottom);
    if (ns > gaugeSide) tft.fillRect(G_VX, G_VY0 - ns, G_W, ns - gaugeSide, col);
    else if (ns < gaugeSide) trackSide(G_VY0 - gaugeSide, G_VY0 - ns);
  }
  gaugeBottom = nb;
  gaugeSide = ns;
  gaugeColor = col;
}

int16_t sparkBuf[SPARK_PW];     // rpm history, newest last, -1 = no sample
volatile int ldrRaw = 0;        // last light-sensor reading, for the log line

// Onboard RGB LED as a shift light, so the warning is visible without looking
// at the screen. Active low, and only the red channel is used.
void ledShift(bool on) {
  digitalWrite(RGB_R_PIN, on ? LOW : HIGH);
}

// Newest sample on the right, so the trace scrolls left as it fills.
void sparkPush(int rpm) {
  memmove(sparkBuf, sparkBuf + 1, (SPARK_PW - 1) * sizeof(sparkBuf[0]));
  sparkBuf[SPARK_PW - 1] = rpm;
}

// Each column is cleared and redrawn in one pass, so no column is ever left
// blank long enough to see. A dim line marks the red zone behind the trace.
void sparkDraw() {
  int base = SPARK_PY + SPARK_PH - 1;
  int yRed = base - (long)RPM_RED * (SPARK_PH - 1) / RPM_MAX;
  for (int i = 0; i < SPARK_PW; i++) {
    int x = SPARK_PX + i;
    tft.drawFastVLine(x, SPARK_PY, SPARK_PH, COL_BG);
    tft.drawPixel(x, yRed, COL_REDZONE);
    int v = sparkBuf[i];
    if (v < 0) continue;
    int h = (long)min(v, RPM_MAX) * (SPARK_PH - 1) / RPM_MAX;
    if (h > 0) tft.drawFastVLine(x, base - h + 1, h, rpmColor(v));
    else       tft.drawPixel(x, base, rpmColor(v));
  }
}

// Blend two RGB565 colours; f = 0 gives a, f = 255 gives b. Used for the
// breathing dot and the fading cell titles.
uint16_t blend565(uint16_t a, uint16_t b, uint8_t f) {
  int ra = (a >> 11) & 0x1F, ga = (a >> 5) & 0x3F, ba = a & 0x1F;
  int rb = (b >> 11) & 0x1F, gb = (b >> 5) & 0x3F, bb = b & 0x1F;
  return (uint16_t)(((ra + (rb - ra) * f / 255) << 11) |
                    ((ga + (gb - ga) * f / 255) << 5) |
                     (ba + (bb - ba) * f / 255));
}

// Repaints a thin slice of one leg, for when an overlay moves off a spot:
// bar colour below the current reading, track colour above it. `pos` is pixels
// from the leg's zero end (left on the bottom leg, the corner on the side leg).
void gaugeSlice(bool side, int pos, int len) {
  if (!side) {
    int x0 = G_BX0 + pos, x1 = min(x0 + len, G_BX1);
    int edge = G_BX0 + gaugeBottom;                  // bar ends here
    if (x0 < edge) tft.fillRect(x0, G_BY, min(x1, edge) - x0, G_W, gaugeColor);
    if (x1 > edge) trackBottom(max(x0, edge), x1);
  } else {
    int yb = G_VY0 - pos, yt = max(yb - len, G_VY1);
    int edge = G_VY0 - gaugeSide;                    // bar ends here (smaller y)
    if (yb > edge) {
      int t = max(yt, edge);
      if (yb > t) tft.fillRect(G_VX, t, G_W, yb - t, gaugeColor);
    }
    if (yt < edge) trackSide(yt, min(yb, edge));
  }
}

// ---- Shift light: above RPM_RED the stretch at the bar's tip is blanked to
// the background on alternate phases, so the red bar looks like it is pulsing
// shorter. Nothing on screen goes white past the red line - the colour stays
// red throughout, which is why this blanks rather than overpaints.
bool shiftOn = false, shiftSide = false;
int  shiftPos = 0, shiftLen = 0;

void shiftLight(int rpm, bool force) {
  bool want = rpm >= RPM_RED && (millis() / SHIFT_BLINK_MS) % 2;
  bool side = rpm > RPM_MID;
  int filled = side ? rpmSideLen(rpm) : rpmBottomLen(rpm);
  int pos = max(filled - SHIFT_W, 0), len = filled - pos;
  if (!force && want == shiftOn && (!want || (side == shiftSide && pos == shiftPos)))
    return;
  if (shiftOn && !force) gaugeSlice(shiftSide, shiftPos, shiftLen);
  if (want && len > 0) {
    if (side) tft.fillRect(G_VX, G_VY0 - pos - len, G_W, len, COL_BG);
    else      tft.fillRect(G_BX0 + pos, G_BY, len, G_W, COL_BG);
  }
  shiftOn = want; shiftSide = side; shiftPos = pos; shiftLen = len;
}

// ---- Peak hold: a steady white tick rides at the highest rpm of the last
// PEAK_HOLD_MS, then slides back down to the bar. White and never blinking, on
// purpose: the blinking tip means "you are in the red zone", this means "this
// is the speed you actually hit", and keeping them visually distinct is what
// lets you read the two apart. It hides itself while it would sit on the bar's
// own tip, where it would only fight the shift light.
int  peakRpm = 0;
unsigned long peakAt = 0;
bool peakShown = false, peakSide = false;
int  peakPos = 0;

void peakMarker(int rpm, bool force) {
  unsigned long now = millis();
  if (rpm > peakRpm) {
    peakRpm = rpm;
    peakAt = now;
  } else if (now - peakAt > PEAK_HOLD_MS) {
    peakRpm = max(peakRpm - PEAK_FALL, rpm);
  }
  bool want = peakRpm - rpm >= PEAK_MIN_GAP;
  bool side = peakRpm > RPM_MID;
  int span = side ? G_VY0 - G_VY1 : G_BX1 - G_BX0;
  int pos = min(side ? rpmSideLen(peakRpm) : rpmBottomLen(peakRpm), span - PEAK_W);
  if (!force && want == peakShown && (!want || (side == peakSide && pos == peakPos)))
    return;
  if (peakShown && !force) gaugeSlice(peakSide, peakPos, PEAK_W);
  if (want) {
    if (side) tft.fillRect(G_VX, G_VY0 - pos - PEAK_W, G_W, PEAK_W, TFT_WHITE);
    else      tft.fillRect(G_BX0 + pos, G_BY, PEAK_W, G_W, TFT_WHITE);
  }
  peakShown = want; peakSide = side; peakPos = pos;
}

void drawSplash() {
  tft.fillScreen(COL_BG);
#ifdef HAVE_LOGO
  // Images need the two bytes per pixel swapped; the library's own drawing
  // calls handle that internally, pushImage() does not.
  tft.setSwapBytes(true);
  tft.pushImage((320 - LOGO_BIG_SIZE) / 2, 2, LOGO_BIG_SIZE, LOGO_BIG_SIZE, alfaLogo120);
  tft.setSwapBytes(false);
#endif
  // Two short lines under the badge: car name, then credits.
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("ALFA 145 QV", 160, 214, 2);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.drawString("by Paul Velzeboer", 160, 230, 1);
}

int cellX(int i) { return PANEL_X + (i % 2) * (CELL_W + CELL_GAP); }
int cellY(int i) { return PANEL_Y + (i / 2) * (CELL_H + CELL_GAP); }

// One symbol per cell, drawn from primitives in an ICON_W x ICON_H box with
// its top-left at (x, y). Each is painted in its own value's colour, so every
// cell doubles as a warning lamp: blue when cold, green when normal, red when
// out of range.
#define ICON_W 17
#define ICON_H 13

// Tachometer: a dial with a needle, for the rpm block.
void drawTachoIcon(int x, int y, uint16_t col) {
  int cx = x + 8, cy = y + 6;
  tft.drawCircle(cx, cy, 6, col);
  tft.drawLine(cx, cy, cx + 4, cy - 4, col);       // needle, up and to the right
  tft.fillCircle(cx, cy, 1, col);
}

// Stem and bulb, shared by the two temperatures.
void drawThermometer(int x, int y, uint16_t col) {
  int cx = x + 8;
  tft.fillRoundRect(cx - 2, y, 4, 7, 1, col);
  tft.fillCircle(cx, y + 8, 3, col);
}

// Coolant: a thermometer standing in water.
void drawCoolantIcon(int x, int y, uint16_t col) {
  drawThermometer(x, y, col);
  for (int s = 0; s < 2; s++) {                    // a wave either side
    int x0 = s ? x + 12 : x;
    tft.drawLine(x0,     y + 8,  x0 + 2, y + 6,  col);
    tft.drawLine(x0 + 2, y + 6,  x0 + 4, y + 8,  col);
    tft.drawLine(x0,     y + 11, x0 + 2, y + 9,  col);
    tft.drawLine(x0 + 2, y + 9,  x0 + 4, y + 11, col);
  }
}

// Air temperature: the same thermometer, but in still straight lines instead of
// waves, so the two temperatures are told apart at a glance.
void drawAirTempIcon(int x, int y, uint16_t col) {
  drawThermometer(x, y, col);
  for (int s = 0; s < 2; s++) {
    int x0 = s ? x + 12 : x;
    tft.drawFastHLine(x0, y + 6, 5, col);
    tft.drawFastHLine(x0 + (s ? 0 : 1), y + 9, 4, col);
  }
}

// Battery: case, two terminals, and the + / - marks.
void drawBatteryIcon(int x, int y, uint16_t col) {
  tft.drawRect(x + 1, y + 2, 15, 10, col);
  tft.fillRect(x + 4, y, 3, 2, col);
  tft.fillRect(x + 10, y, 3, 2, col);
  tft.drawFastHLine(x + 3, y + 7, 5, col);         // +
  tft.drawFastVLine(x + 5, y + 5, 5, col);
  tft.drawFastHLine(x + 10, y + 7, 5, col);        // -
}

// Air quantity: airflow, three arrows of different lengths.
void drawFlowIcon(int x, int y, uint16_t col) {
  const int len[3] = {11, 15, 11};
  for (int i = 0; i < 3; i++) {
    int yy = y + 2 + i * 4, x1 = x + 1 + len[i];
    tft.drawFastHLine(x + 1, yy, len[i], col);
    tft.drawLine(x1, yy, x1 - 3, yy - 2, col);
    tft.drawLine(x1, yy, x1 - 3, yy + 2, col);
  }
}

// Short titles: the symbol beside each one carries the meaning, and the full
// words would run into the cell's right edge once the icon takes its 17 px.
const char *cellTitle(int i) {
  return i == CELL_BATT ? "BATT" : i == CELL_COOL ? "COOL" :
         i == CELL_AIRT ? "AIR T" : "AIR Q";
}

int iconX(int i) { return cellX(i) + 5; }
int iconY(int i) { return cellY(i) + 4; }

void drawCellIcon(int i, uint16_t col) {
  int x = iconX(i), y = iconY(i);
  if (i == CELL_BATT)      drawBatteryIcon(x, y, col);
  else if (i == CELL_COOL) drawCoolantIcon(x, y, col);
  else if (i == CELL_AIRT) drawAirTempIcon(x, y, col);
  else                     drawFlowIcon(x, y, col);
}

void drawText(const String &txt, int x, int y, uint8_t datum, int font, uint16_t fg,
              uint16_t bg, int padding) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
  tft.setTextPadding(padding);
  tft.drawString(txt, x, y, font);
  tft.setTextPadding(0);
}

// Titles are redrawn on their own to fade the fresh-data cue, so they have one
// owner rather than being spelled out in dashFrame().
void drawCellTitle(int i, uint16_t col) {
  drawText(cellTitle(i), cellX(i) + 5 + ICON_W + 3, cellY(i) + 10, ML_DATUM, 1,
           col, COL_BG, 0);
}

// The big number has one owner for font, position and clear width, so the
// three places that draw it cannot drift apart.
void drawRpmNumber(const String &txt, uint16_t col) {
  drawText(txt, RPM_CX, RPM_CY, MC_DATUM, RPM_FONT, col, COL_BG,
           tft.textWidth("8888", RPM_FONT));
}

// Static parts; called once from setup() before the display task starts.
void dashFrame() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 320, 24, COL_BAR);
#ifdef HAVE_LOGO
  tft.setSwapBytes(true);
  tft.pushImage(4, (24 - LOGO_SMALL_SIZE) / 2, LOGO_SMALL_SIZE, LOGO_SMALL_SIZE, alfaLogo18);
  tft.setSwapBytes(false);
  drawText("ALFA 145 QV", 8 + LOGO_SMALL_SIZE, 12, ML_DATUM, 2, TFT_WHITE, COL_BAR, 0);
#else
  drawText("ALFA 145 QV", 6, 12, ML_DATUM, 2, TFT_WHITE, COL_BAR, 0);
#endif

  // L gauge: empty track, then the scale in thousands. The bottom leg is
  // labelled under the bar, the side leg to its left, so the labels never sit
  // on top of the cells.
  trackBottom(G_BX0, G_BX1);
  trackSide(G_VY1, G_VY0);
  gaugeBottom = 0;
  gaugeSide = 0;
  gaugeColor = TFT_GREEN;
  peakShown = false;
  shiftOn = false;
  peakRpm = 0;
  for (int k = 0; k <= RPM_MID / 1000; k++) {
    int x = G_BX0 + k * (G_BX1 - G_BX0) / (RPM_MID / 1000);
    drawText(String(k), x, G_BY + G_W + 6, MC_DATUM, 1, COL_LABEL, COL_BG, 0);
  }
  for (int k = RPM_MID / 1000 + 1; k <= RPM_MAX / 1000; k++) {
    drawText(String(k), G_VX - 4, sideY(k * 1000), MR_DATUM, 1,
             k * 1000 >= RPM_RED ? TFT_RED : COL_LABEL, COL_BG, 0);
  }
  // Dial and "RPM" are centred together as one group, so the pair sits over
  // the number rather than the text alone.
  int gw = ICON_W + 4 + tft.textWidth("RPM", 2);
  tachoX = RPM_CX - gw / 2;
  tachoY = RPM_CY - 58;
  drawTachoIcon(tachoX, tachoY, COL_LABEL);
  drawText("RPM", tachoX + ICON_W + 4, RPM_CY - 52, ML_DATUM, 2, COL_LABEL, COL_BG, 0);
  // The rpm history takes the space the "x1000" hint used to fill; both legs
  // of the bar already carry labels in thousands, so the hint was redundant.
  tft.drawRoundRect(SPARK_X, SPARK_Y, SPARK_W, SPARK_H, 5, COL_FRAME);
  for (int i = 0; i < SPARK_PW; i++) sparkBuf[i] = -1;
  sparkDraw();

  // 2 x 2 value grid: title (left), unit (right), value in the middle.
  // Units live in the labels so the value line can stay as wide as possible.
  for (int i = 0; i < 4; i++) {
    int x = cellX(i), y = cellY(i);
    tft.drawRoundRect(x, y, CELL_W, CELL_H, 5, COL_FRAME);
    // Titles in the small font (font 1): font 2 was too wide for a 74 px cell,
    // so "AIR TEMP" + unit didn't line up with the others.
    drawCellTitle(i, COL_LABEL);
    drawCellIcon(i, COL_LABEL);
    // Unit on its own line under the title, centred over the value.
    int ux = x + CELL_W / 2;
    if (i == CELL_BATT) {
      drawText("V", ux, y + 22, MC_DATUM, 1, COL_LABEL, COL_BG, 0);
    } else if (i == CELL_AIRQ) {
      // raw ADC count for now: the ECU's kg/h scaling is unknown (AIRQ_SCALE)
      drawText("raw", ux, y + 22, MC_DATUM, 1, COL_LABEL, COL_BG, 0);
    } else {                                   // degree sign: built-in fonts have none
      tft.drawCircle(ux - 2, y + 19, 2, COL_LABEL);
      drawText("C", ux + 5, y + 22, MC_DATUM, 1, COL_LABEL, COL_BG, 0);
    }
  }

  dashReady = true;
}

// ---- Called from the K-line code: only store, never draw.
void dashStatus(const String &txt, uint16_t color) {
  if (!dashMutex) return;
  xSemaphoreTake(dashMutex, portMAX_DELAY);
  shStatus = txt; shStatusCol = color; shStatusDirty = true;
  // A steady dot means the session is up; everything else breathes, so a
  // stalled handshake never looks like a working one.
  shStatusPulse = !(txt == "CONNECTED" || txt == "DEMO");
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

// battV < 0, rpm < 0, coolantC/airT <= -999, airQ < 0 mean "no value" (--).
void dashValues(float battV, int rpm, float coolantC, float airT, float airQ) {
  shBatt = battV;
  shRpm = rpm;
  shCool = coolantC;
  shAirT = airT;
  shAirQ = airQ;
}

void drawCell(int i, const String &txt, uint16_t color) {
  drawText(txt, cellX(i) + CELL_W / 2, cellY(i) + 54, MC_DATUM, 4, color, COL_BG, CELL_W - 8);
}

// ---- Display task (core 0): the only code that draws after setup().
// The rpm gauge and number glide towards the latest reading every frame, so
// the needle moves smoothly although new values arrive only ~3-4x/s.
void dashTask(void *param) {
  float shownRpm = 0;
  int drawnRpm = -2;            // -2 = nothing drawn yet, -1 = "--" drawn
  float drawnBatt = -2, drawnCool = -2, drawnAirT = -2, drawnAirQ = -2;
  int cellFlash[4] = {0, 0, 0, 0};
  int dotX = 0;                 // status dot, remembered so it can breathe
  uint16_t dotCol = 0;
  bool dotPulse = false;
  bool shiftTextOn = false;     // "Shift!" currently painted in the top bar
  uint16_t drawnTacho = COL_LABEL;
  unsigned long sparkAt = 0;
  float ldrAvg = analogRead(LDR_PIN);
  int blDuty = BL_MAX;

  // Self-test sweep, the way a cluster wakes up: run the bar to the top and
  // back once. It also proves both legs and the red zone draw before any data
  // arrives. The number follows it, then the first real reading overwrites it.
  for (int i = 0; i <= 2 * BOOT_SWEEP_STEPS; i++) {
    int step = i <= BOOT_SWEEP_STEPS ? i : 2 * BOOT_SWEEP_STEPS - i;
    int r = (long)RPM_MAX * step / BOOT_SWEEP_STEPS;
    gaugeSet(r);
    gaugeFullRepaint = false;   // no overlays during the sweep
    drawRpmNumber(String(r / 10 * 10), TFT_WHITE);
    vTaskDelay(pdMS_TO_TICKS(BOOT_SWEEP_MS));
  }

  for (;;) {
    // status bar and bottom line (copied under the mutex, drawn outside it)
    String st, bt;
    uint16_t sc = 0, bc = 0;
    bool sd = false, bd = false;
    bool sp = false;
    xSemaphoreTake(dashMutex, portMAX_DELAY);
    if (shStatusDirty) {
      st = shStatus; sc = shStatusCol; sp = shStatusPulse; sd = true;
      shStatusDirty = false;
    }
    if (shBottomDirty) { bt = shBottom; bc = shBottomCol; bd = true; shBottomDirty = false; }
    xSemaphoreGive(dashMutex);
    if (sd) {
      tft.fillRect(170, 0, 150, 24, COL_BAR);
      int w = tft.textWidth(st, 2);
      dotX = 314 - w - 10;
      dotCol = sc;
      dotPulse = sp;
      tft.fillCircle(dotX, 12, 5, sc);
      drawText(st, 314, 12, MR_DATUM, 2, sc, COL_BAR, 0);
    }
    // The dot breathes while the session isn't up, and sits steady once it is.
    if (dotPulse && dotX) {
      float ph = (millis() % DOT_PULSE_MS) / (float)DOT_PULSE_MS;
      uint8_t f = (uint8_t)(255.0f * (0.5f - 0.5f * cos(2 * PI * ph)));
      tft.fillCircle(dotX, 12, 5, blend565(COL_BAR, dotCol, f));
    }
    if (bd) drawText(bt, 6, 227, ML_DATUM, 2, bc, COL_BG, 308);

    // rpm: ease towards the target, redraw only when the shown value changes
    int target = shRpm;
    if (target < 0) {
      shownRpm = 0;
      if (drawnRpm != -1) {
        gaugeSet(0);
        drawRpmNumber("--", COL_LABEL);
        drawnRpm = -1;
      }
      peakRpm = 0;                        // nothing left to hold on to
    } else {
      shownRpm += (target - shownRpm) * RPM_SMOOTHING;
      if (fabsf(target - shownRpm) < 5) shownRpm = target;
      int r = (int)(shownRpm + 0.5f);
      if (r != drawnRpm) {
        gaugeSet(r);
        // 10 rpm steps, calmer to read. Clamped to RPM_MAX because a nonsense
        // reading - blk[2] x 40 can reach 10200 - would be five digits and
        // would run off the left edge and into the cells. The serial summary
        // still logs the unclamped value.
        int r10 = min((r + 5) / 10 * 10, (int)RPM_MAX);
        drawRpmNumber(String(r10), TFT_WHITE);
        drawnRpm = r;
      }
    }

    // The dial follows the rev zone, like each cell's symbol follows its value.
    uint16_t tcol = drawnRpm < 0 ? COL_LABEL : rpmColor(drawnRpm);
    if (tcol != drawnTacho) {
      drawTachoIcon(tachoX, tachoY, tcol);
      drawnTacho = tcol;
    }

    // Overlays go on top of the bar, so they run after gaugeSet(). A colour
    // change repaints both legs and wipes them, hence the force flag.
    int shown = drawnRpm < 0 ? 0 : drawnRpm;
    bool force = gaugeFullRepaint;
    gaugeFullRepaint = false;
    shiftLight(shown, force);
    peakMarker(shown, force);

    // "Shift!" blinks in the middle of the top bar past RPM_RED, on the same
    // phase as the bar's shift light so the two pulse together. Redrawn when
    // the status was repainted as well, since that clears part of this area.
    bool wantShift = shown >= RPM_RED && (millis() / SHIFT_BLINK_MS) % 2;
    if (wantShift != shiftTextOn || sd) {
      int w = tft.textWidth(SHIFT_TEXT, 2) + 6;
      tft.fillRect(160 - w / 2, 2, w, 20, COL_BAR);
      if (wantShift) drawText(SHIFT_TEXT, 160, 12, MC_DATUM, 2, TFT_RED, COL_BAR, 0);
      shiftTextOn = wantShift;
    }
    ledShift(wantShift);        // the onboard LED flashes on the same phase

    // rpm history: one sample per SPARK_MS, then the whole strip is redrawn
    if (millis() - sparkAt >= SPARK_MS) {
      sparkAt = millis();
      sparkPush(drawnRpm < 0 ? -1 : shown);
      sparkDraw();
    }

    // Backlight follows the light sensor, heavily smoothed so a hand passing
    // over it or oncoming headlights don't make the screen pump.
    ldrRaw = analogRead(LDR_PIN);
    ldrAvg += (ldrRaw - ldrAvg) * LDR_SMOOTH;
    int duty = constrain((int)map((long)ldrAvg, LDR_DARK, LDR_BRIGHT, BL_MIN, BL_MAX),
                         BL_MIN, BL_MAX);
    if (abs(duty - blDuty) >= LDR_STEP) {
      ledcWrite(TFT_BACKLIGHT_PIN, duty);
      blDuty = duty;
    }

    float bv = shBatt;
    if (bv != drawnBatt) {
      // Green across the whole usable range: red only under 11.5 V (flat or
      // not charging) and over 15 V (regulator letting go).
      uint16_t col = bv < 0 ? COL_LABEL :
                     (bv < 11.5f || bv > 15.0f) ? TFT_RED : TFT_GREEN;
      drawCell(CELL_BATT, bv < 0 ? String("--") : String(bv, 2), col);
      drawCellIcon(CELL_BATT, col);
      drawnBatt = bv;
      if (bv >= 0) cellFlash[CELL_BATT] = FLASH_FRAMES;
    }
    float cv = shCool;
    if (cv != drawnCool) {
      // Bands: up to 65 blue (still warming), 66-99 green, 100 and over red.
      uint16_t col = cv <= -999 ? COL_LABEL : cv < 66 ? TFT_SKYBLUE :
                     cv < 100 ? TFT_GREEN : TFT_RED;
      drawCell(CELL_COOL, cv <= -999 ? String("--") : String(cv, 1), col);
      drawCellIcon(CELL_COOL, col);
      drawnCool = cv;
      if (cv > -999) cellFlash[CELL_COOL] = FLASH_FRAMES;
    }
    float av = shAirT;
    if (av != drawnAirT) {
      uint16_t col = av <= -999 ? COL_LABEL : av < 5 ? TFT_SKYBLUE : av <= 60 ? TFT_GREEN : TFT_RED;
      drawCell(CELL_AIRT, av <= -999 ? String("--") : String(av, 1), col);
      drawCellIcon(CELL_AIRT, col);
      drawnAirT = av;
      if (av > -999) cellFlash[CELL_AIRT] = FLASH_FRAMES;
    }
    float qv = shAirQ;
    if (qv != drawnAirQ) {
      uint16_t qcol = qv < 0 ? COL_LABEL : TFT_WHITE;
      drawCell(CELL_AIRQ, qv < 0 ? String("--") : String((int)(qv + 0.5f)), qcol);
      drawCellIcon(CELL_AIRQ, qcol);
      drawnAirQ = qv;
      if (qv >= 0) cellFlash[CELL_AIRQ] = FLASH_FRAMES;
    }

    // Fresh-data cue: a title lights up as its value lands and fades back to
    // grey, so a session that has stopped polling is obvious at a glance.
    for (int i = 0; i < 4; i++) {
      if (cellFlash[i] <= 0) continue;
      cellFlash[i]--;
      drawCellTitle(i, blend565(COL_LABEL, TFT_WHITE, 255 * cellFlash[i] / FLASH_FRAMES));
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
float lastBatt = -1, lastCool = -1000, lastAirT = -1000, lastAirQ = -1;

// NTC sensor curve, from the Python tool's coolant formula; the ECU returns a
// raw ADC count that FALLS as the sensor gets hotter.
float ntcTemp(int raw) {
  float x = raw;
  return -0.000014482f * x * x * x + 0.006319247f * x * x - 1.35140625f * x + 144.4095455f;
}

bool pollAndDisplayParams() {
  uint8_t battReq[] = {0x01, 0x01, 0x00, 0x36};
  uint8_t rpmReq[]  = {0x01, 0x02, 0x00, 0x3B};
  uint8_t coolReq[] = {0x08, ADC_COOLANT};
  uint8_t airTReq[] = {0x08, ADC_AIRTEMP};
  uint8_t airQReq[] = {0x08, ADC_AIRQTY};

  uint8_t blk[MAX_BLOCK_LEN];
  int len;
  int slot = pollN++ % 10;

  if (slot == 1) {
    if (!requestParam(battReq, sizeof(battReq), "BATTERY", blk, &len)) return false;
    // Reply layout: [counter, type, data..., 03] - first data byte is blk[2].
    if (len >= 4 && blk[1] == 0xFE) {
      lastBatt = blk[2] * BATT_SCALE + BATT_OFFSET;
      dbg("  [BATT] raw 0x" + String(blk[2], HEX) + " (" + String(blk[2]) + ") -> " +
          String(lastBatt, 2) + " V" +
          (lastBatt < BATT_MIN_PLAUSIBLE || lastBatt > BATT_MAX_PLAUSIBLE
               ? "  !! outside 6-16 V: wrong byte or wrong scale" : ""));
    }
  } else if (slot == 3) {
    if (!requestParam(coolReq, sizeof(coolReq), "COOLANT", blk, &len)) return false;
    // ADC read -> type 0xFB + 2 bytes (hi, lo); the formula uses the low byte.
    if (len >= 5 && blk[1] == 0xFB) lastCool = ntcTemp(blk[3]);
  } else if (slot == 5) {
    if (!requestParam(airTReq, sizeof(airTReq), "AIRTEMP", blk, &len)) return false;
    // Same NTC curve as coolant, and the cold run confirms it: raw 197 with a
    // 14 C ambient reads 13 C. It drops as the engine bay warms (156 -> 32 C).
    if (len >= 5 && blk[1] == 0xFB) lastAirT = ntcTemp(blk[3]);
  } else if (slot == 7) {
    if (!requestParam(airQReq, sizeof(airQReq), "AIRQTY", blk, &len)) return false;
    if (len >= 5 && blk[1] == 0xFB) {
      int raw = blk[3] - AIRQ_ZERO;
      lastAirQ = (raw < 0 ? 0 : raw) * AIRQ_SCALE;
    }
  } else {
    if (!requestParam(rpmReq, sizeof(rpmReq), "RPM", blk, &len)) return false;
    // RPM: RAM read -> type 0xFE + 2 bytes, both counting the same speed at
    // different resolutions (discovery runs, 12 Oct 2026):
    //   blk[3] = rpm / 10, which saturates at 0xFF = 2550 rpm
    //   blk[2] = rpm / 40, which carries on past that
    // Idle read 20 / 82 (800 / 820 rpm) and a held 3000 rpm averaged 74.6 on
    // blk[2] (2984 rpm). Both read 00 00 with the engine off.
    if (len >= 5 && blk[1] == 0xFE)
      lastRpm = blk[3] < 0xFF ? blk[3] * 10 : blk[2] * 40;
  }
  dashValues(lastBatt, lastRpm, lastCool, lastAirT, lastAirQ);

  if (slot == 9) {                   // one summary line per 10 exchanges
    String line = "Batt: " + (lastBatt > -1 ? String(lastBatt, 2) + "V" : String("--"));
    line += "  RPM: " + (lastRpm >= 0 ? String(lastRpm) : String("--"));
    line += "  Cool: " + (lastCool > -999 ? String(lastCool, 1) + "C" : String("--"));
    line += "  AirT: " + (lastAirT > -999 ? String(lastAirT, 1) + "C" : String("--"));
    line += "  AirQ: " + (lastAirQ >= 0 ? String(lastAirQ, 0) : String("--"));
    line += "  LDR: " + String(ldrRaw);
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
  // Backlight on PWM rather than a plain HIGH, so the light sensor can dim it.
  ledcAttach(TFT_BACKLIGHT_PIN, BL_FREQ, BL_BITS);
  ledcWrite(TFT_BACKLIGHT_PIN, BL_MAX);
  // One reading at boot, so the two LDR constants can be checked against the
  // room the board is actually in.
  Serial.println("LDR at boot: " + String(analogRead(LDR_PIN)) +
                 "  (" + String(LDR_DARK) + " = dark -> dim, " +
                 String(LDR_BRIGHT) + " = bright -> full)");
  // RGB LED: all three off (active low) until the shift light wants red.
  pinMode(RGB_R_PIN, OUTPUT); digitalWrite(RGB_R_PIN, HIGH);
  pinMode(RGB_G_PIN, OUTPUT); digitalWrite(RGB_G_PIN, HIGH);
  pinMode(RGB_B_PIN, OUTPUT); digitalWrite(RGB_B_PIN, HIGH);
  tft.setRotation(1);
  // Colour order: this panel is BGR, TFT_eSPI's setup defaults to RGB (red
  // and blue swapped). Rewrite the orientation register the way setRotation(1)
  // does for the ST7789 (MX | MV), but with the BGR bit. Must come AFTER
  // setRotation(), which overwrites this register.
  tft.writecommand(TFT_MADCTL);
  tft.writedata(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_BGR);
  tft.setTextSize(1);
  dashMutex = xSemaphoreCreateMutex();
  drawSplash();
  delay(SPLASH_MS);
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
  float airT = 20.0f + 18.0f * warm + 0.8f * sin(t / 3.0f);
  float airQ = 7 + rpm / 45.0f;                 // stand-in: ~7 off, ~40 idle, ~75 at 3000
  dashValues(battV, rpm, coolantC, airT, airQ);
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
    lastRpm = -1; lastBatt = -1; lastCool = -1000; lastAirT = -1000; lastAirQ = -1;
    dashValues(-1, -1, -1000, -1000, -1);
    lastAttempt = millis();
    return;
  }
}
