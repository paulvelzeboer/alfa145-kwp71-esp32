# Alfa 145 QV KWP71 reader: ESP32 CYD + ISO 9141 Click

Reads a **Bosch Motronic M2.10.4** (Alfa Romeo 145 QV, part 0261204478) over the K-line with the old Bosch **KWP71** protocol. It runs on an **ESP32-2432S028 "Cheap Yellow Display"** with a **MikroE ISO 9141 Click**, and shows a live dashboard on the CYD's screen: an L-shaped rpm bar that runs along the bottom and up the right edge, wrapping four values (battery, coolant, air temp, air quantity), each with its own symbol and coloured by its own reading. It has a cluster-style boot sweep, a peak-hold tick, a 30 s rpm history trace, a blinking red zone with a `Shift!` warning that also flashes the board's own LED, the ECU ID numbers, and a boot splash with the car badge. The backlight dims itself using the CYD's light sensor. **Tapping the screen** switches to a diagnostics page: every ADC channel the ECU answers, raw, the session's extremes, and whether the ECU has any stored fault codes. **Every drive is logged to the micro-SD card** as `/logNNNN.csv`, one row per second.

**Status (12 Oct 2026):** the full handshake and live values work on the real ECU, including the ID blocks (HW 0261204478, SW 1037357941, PN 46525168). A discovery run over all the ECU's ADC channels, in five engine states, identified throttle, battery, coolant, air quantity, intake air temperature and lambda, and gave the rpm and battery scales; all five dashboard values are now read from the car. Tested on the car the same day: the handshake, the smoother dashboard, and the rpm and battery readings all behave, and air temp and air quantity return data that still has to be checked against something known.

The full bring-up story, wiring, protocol findings and troubleshooting are in **[docs/kwp71-m2104-bringup.md](docs/kwp71-m2104-bringup.md)**.

## Repository layout

| Path | What |
| --- | --- |
| `firmware/kwp71_m2104_esp32/` | Main sketch: KWP71 handshake, polling, TFT dashboard, demo mode |
| `firmware/kline_loopback_test/` | Wiring test: sends bytes on TX (GPIO27) and checks they come back on RX (GPIO22) |
| `tools/kwp71_m2_10_4.py` | PC tool: KWP71 over a KKL / VAG-COM 409.1 cable (`--probe` for wakeup + sync only) |
| `tools/m2104_sim.py` | M2.10.4 simulator: the PC + KKL cable plays the ECU for desk testing (`--fault` fakes a stored fault code) |
| `tools/check_wakeup.py` | Decodes the 5-baud 0x10 wakeup from a sigrok CSV capture |
| `tools/kwp71_discover.py` | Asks the ECU for everything it answers (ADC channels, RAM ranges) and logs it to CSV; `--compare` diffs two runs |
| `tools/make_case.py` | Generates the case (STL + 3MF) from the dimensions at the top of the file |
| `tools/trace_badge.py` | Traces a badge image into outlines for the lid |
| `tools/make_logo_header.py` | Converts a logo image into display data for the sketch |
| `docs/kwp71-m2104-bringup.md` | Documentation: bring-up plan, results, dashboard, troubleshooting |
| `captures/` | Logic analyzer captures, plus the `discover*.csv` runs that produced the ADC map |
| `hardware/` | Printable case for the CYD + Click: STL files plus `case_lid.3mf` for multi-colour printing, see [hardware/README.md](hardware/README.md) |
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
| Battery | `01 01 00 36` → FE + 1 byte, × 0.073 V (two in-car states, 164 raw at rest / 198 charging) |
| RPM | `01 02 00 3B` → FE + 2 bytes: low byte × 10 up to 2550 rpm, then high byte × 40 |
| Coolant | `08 03` → FB + 2 bytes, NTC polynomial on the low byte |
| Air temp | `08 05` → FB + 2 bytes, the same NTC polynomial |
| Air quantity | `08 04` → FB + 2 bytes, raw count (6 engine off, ~41 idle, ~70 at 3000 rpm) |
| Other ADCs | 00 throttle, 07 lambda, 06 load-dependent but unidentified; 02/08/0B unused, 09/0A zero, 0C-0F absent |

## 3D-printed case

`hardware/` holds a two-part case: a shell with posts for the CYD and a slide-in holder for the Click, and a lid with a window for the display and the quadrifoglio inlaid in its top layers. Regenerate after changing any dimension:

```sh
python3 tools/make_case.py
```

Print `case_clickfit.stl` and `case_fitcheck.stl` first; they check the fit in minutes. Details and print settings are in [hardware/README.md](hardware/README.md).

## Open items

- The **kg/h scale for air quantity**: the display shows the raw count until there is something to calibrate it against.
- **ADC 06** carries something that changes with load, still unidentified. The diagnostics page and the SD log are the tools for it.
- A **measured point for the battery scale**. It reads plausibly on the car, but the 0.073 V/step fit has never been compared with a meter; the sketch logs the raw byte for this.
- **Confirming air temp and air quantity**, which return sensible data but haven't been checked against a thermometer or a known airflow.
- **Fault codes are read but not decoded.** Request `07` answers type `FC` + `00` (nothing stored) on this car, so the layout of a real fault has never been seen; the diagnostics page shows the raw bytes. Unplugging a sensor to induce one would give us the format.
- The first request after the handshake always gets a NAK. It's harmless, but a short pause might avoid it.
- `LDR_DARK` (the backlight's dark end) is a guess. Cover the light sensor, reset, and read the `LDR at boot:` line to set it properly.
- The shift LED is on the board, so the printed lid hides it. It needs a hole or a light pipe above the LED to be any use in the car.
- The case is unprinted apart from the Click holder coupon. The terminal height under the Click (7.0 mm) and the pin height above it (8.0 mm) are estimates, and they set the case height.

## License

MIT, see [LICENSE](LICENSE).
