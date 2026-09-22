# Alfa 145 QV KWP71 reader: ESP32 CYD + ISO 9141 Click

Reads a **Bosch Motronic M2.10.4** (Alfa Romeo 145 QV, part 0261204478) over the K-line with the old Bosch **KWP71** protocol. It runs on an **ESP32-2432S028 "Cheap Yellow Display"** with a **MikroE ISO 9141 Click**, and shows a live dashboard on the CYD's screen: an rpm gauge, four values (battery, coolant, air temp, air quantity), the ECU ID numbers, and a boot splash with the car badge.

**Status (22 Sep 2026):** the full handshake and live values work on the real ECU (bench, engine off). The ESP32 reads 12.27 V, 18 °C and 0 rpm, plus the ID blocks: HW 0261204478, SW 1037357941, PN 46525168. The smoother dashboard (rpm polled ~3–4×/s, display on the second core) compiles but hasn't been tested on the ECU yet.

The full bring-up story, wiring, protocol findings and troubleshooting are in **[docs/kwp71-m2104-bringup.md](docs/kwp71-m2104-bringup.md)**.

## Repository layout

| Path | What |
| --- | --- |
| `firmware/kwp71_m2104_esp32/` | Main sketch: KWP71 handshake, polling, TFT dashboard, demo mode |
| `firmware/kline_loopback_test/` | Wiring test: sends bytes on TX (GPIO27) and checks they come back on RX (GPIO22) |
| `tools/kwp71_m2_10_4.py` | PC tool: KWP71 over a KKL / VAG-COM 409.1 cable (`--probe` for wakeup + sync only) |
| `tools/m2104_sim.py` | M2.10.4 simulator: the PC + KKL cable plays the ECU for desk testing |
| `tools/check_wakeup.py` | Decodes the 5-baud 0x10 wakeup from a sigrok CSV capture |
| `docs/kwp71-m2104-bringup.md` | Documentation: bring-up plan, results, dashboard, troubleshooting |
| `captures/` | Logic analyzer captures (the existing ones show no K-line activity; see the doc, P0) |
| `archive/` | Older sketch version and an old KKL tool log, for reference only |

## Hardware and wiring

| From | To | Note |
| --- | --- | --- |
| CYD CN1 IO27 (K_TX) | Click mikroBUS pin marked **RX** | the Click's pin names are from its own side, so TX/RX cross |
| Click mikroBUS pin marked **TX** | CYD CN1 IO22 (K_RX) | |
| CYD CN1 3.3V / GND | Click +3.3V / GND | Click jumper **JP1 = 3V3** (5V would put 5 V on GPIO22) |
| Click K screw | ECU diag connector **pin 3 (K)** | 3-pin Fiat/Alfa connector: pin 1 not connected, pin 2 GND, pin 3 K |
| Click GND | ECU diag connector **pin 2 (GND)** and 12 V minus | shared ground is required |
| 12 V (1 A fuse) | Click VS terminal + ECU supply | |

Never connect the K-line or 12 V directly to the ESP32 or the logic analyzer. For probing the K-line, use a 47 kΩ / 15 kΩ divider (see the doc).

## Firmware

Needs the ESP32 Arduino core (tested with 3.3.11) and **TFT_eSPI** (2.5.43) set up for the CYD pins with `ST7789_DRIVER`. The sketch itself switches on the backlight, turns off colour inversion and sets BGR colour order; the stock setup gets all three wrong for this panel.

```sh
arduino-cli compile --upload -p /dev/cu.usbserial-XXXX \
  --fqbn esp32:esp32:esp32:UploadSpeed=115200 firmware/kwp71_m2104_esp32
arduino-cli monitor -p /dev/cu.usbserial-XXXX -c baudrate=115200
```

The CYD's CH340 USB chip drops out at the default 921600 baud, hence `UploadSpeed=115200`.

`#define DEMO_MODE 1` shows simulated values without an ECU. Set it to `0` for the car.

The Alfa badge on the boot splash and in the status bar comes from an image that isn't in this repository, since car maker logos are trademarks. To show it, put your own `logo.png` in the project root and run:

```sh
pip install pillow
python3 tools/make_logo_header.py logo.png
```

Without that header the sketch still builds: the splash is then text-only.

## PC tools

```sh
pip install pyserial
python3 tools/kwp71_m2_10_4.py --port /dev/cu.usbserial-AH01164M --probe   # KKL: wakeup + sync only
python3 tools/m2104_sim.py --port /dev/cu.usbserial-AH01164M              # KKL plays the ECU
python3 tools/check_wakeup.py capture.csv --channel K_LINE --address 0x10 # check a wakeup capture
```

Only one tester may be on the K-line at a time: unplug the KKL when the Click is connected, and the other way round (the simulator setup excepted).

## Protocol facts for this ECU (measured)

| Item | Value |
| --- | --- |
| Wakeup | address 0x10 at 5 baud, K-line idle high ≥ 2 s before the start bit |
| Sync | ECU sends 0x55 about 180–190 ms after the stop bit, then `32 86 04 15 26` at 4800 baud |
| Keyword reply | complement of byte #1: 0x86 → **0x79** |
| Blocks | `LEN CTR TYPE DATA… 03`. **LEN counts the 0x03.** Every byte is echoed inverted, except the 0x03 |
| Quirk | the ECU always sends the LEN byte of its first block twice; echo the repeat, don't store it |
| Echo timing | 10–30 ms before each echo works; 0 ms is rejected |
| ID blocks | type F6, ASCII reversed: 0261204478 / 1037357941 / 46525168 |
| Battery | `01 01 00 36` → FE + 1 byte, × 0.1319 V (calibrated at 12.4 V) |
| Coolant | `08 03` → FB + 2 bytes, polynomial on the low byte |
| RPM | `01 02 00 3B` → FE + 2 bytes; formula **not verified** with a running engine |

## Open items

- RPM formula and a second battery calibration point, with the engine running.
- The first request after the handshake always gets a NAK. It's harmless, but a short pause might avoid it.
- Air temperature and air quantity have display cells but aren't requested from the ECU yet (air temp is probably ADC channel 02; the air quantity address is unknown).

## License

MIT, see [LICENSE](LICENSE).
