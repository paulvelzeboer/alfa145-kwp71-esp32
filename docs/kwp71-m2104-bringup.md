# ESP32 CYD → ISO 9141 Click → Bosch M2.10.4: KWP71 bring-up plan

Sep 21, 2026 · @Paul

## The finished reader

![The reader mounted in the car, connected and showing live values](images/end.jpg)

Installed and running on the car. The screen is a real session, not demo mode:
**CONNECTED**, 1280 rpm on a cold fast idle, **14.53 V** with the engine running
and the alternator charging, coolant and intake air both still at 20 °C so the
coolant reads blue, air quantity 57, and the ECU's own identification along the
bottom — `HW 0261204478  SW 1037357941`, exactly the numbers the handshake
returns. The L bar runs along the bottom with its scale beneath it, and the
quadrifoglio on the lid is the inlaid badge printed in white and green.

That 14.53 V is worth noting: it is the first real-world sanity check of the
battery scale, which was fitted from two raw counts and had never been seen
against a running alternator. A charging system should sit near 14.4 V, and it
does. It is still not a multimeter reading, so the open item stands, but the
figure is where it ought to be rather than somewhere alarming.

## Goal and starting point

The fastest route to a working ESP32 link is to copy the handshake that already works. First record the KKL cable's session on the K-line with the logic analyzer. Then make the ESP32's K-line trace match it, phase by phase.

| Path | Status | Evidence |
| --- | --- | --- |
| MacBook M4 → KKL / VAG-COM 409.1 (FTDI, `/dev/cu.usbserial-AH01164M`) → 3-pin Fiat/Alfa diag → M2.10.4 | Works | `kwp71_m2_10_4.py`: address 0x10, 4800 baud, 0x55 about 180 ms after the stop bit, then `32 86 04 15 26`, reply 0x79, info block = part number 0261204478. Confirmed again with `--probe` on 22 Sep 2026 |
| ESP32-2432S028 → MikroE ISO 9141 Click → K-line → M2.10.4 | **Works** on the real ECU: full handshake, ID blocks and live values (22 Sep 2026) | K on connector pin 3 plus shared ground on pin 2, 20 ms echo delay, LEN counts the 0x03, the ECU's repeated LEN byte handled. Reads 12.27 V, 18.0 °C and 0 rpm (engine off) |

The plan works outward from the ESP32 pin, and each phase has a pass check:

1. The ESP32 pin sends the correct 0x10 wakeup.
2. The Click reproduces that wakeup on the K-line.
3. The ECU answers with 0x55.
4. The ESP32 decodes the answer and replies 0x79.
5. The info blocks arrive.

If a check fails, the cause is in the one link that was added last.

```mermaid
flowchart LR
  A[P0 Golden capture<br/>KKL + analyzer] --> B[P1 ESP32 pin<br/>no Click]
  B --> C[P2 Click on bench<br/>12 V, no ECU]
  C --> S[P2b Full session<br/>vs simulator]
  S --> D[P3 Wakeup to ECU<br/>expect 0x55]
  D --> E[P4 Keyword reply<br/>0x79]
  E --> F[P5 Info blocks<br/>+ keep-alive]
```

Each box is one phase in the step-by-step plan below. Don't start a phase until the one before it passes its check.

## Parts and known protocol parameters

Everything the ESP32 has to reproduce is already proven by `kwp71_m2_10_4.py` against this ECU. Copy these values; don't guess new ones.

| Part | Role | Notes |
| --- | --- | --- |
| [ESP32-2432S028 "CYD"](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/PINS.md) | Tester MCU | Use connector CN1 (GND, IO22, IO27, 3.3V). UART2 on TX=GPIO27, RX=GPIO22. USB (UART0) stays free for logs. |
| [MikroE ISO 9141 Click](https://www.mikroe.com/iso-9141-click) (MIKROE-4331) | K-line transceiver, [L9637D](https://download.mikroe.com/documents/add-on-boards/click/iso_9141_click/iso-9141-click-schematic-v101.pdf) | JP1 selects logic level, and it **must be 3V3**. Has a 4.7 kΩ K-line pull-up to VS. The VS terminal takes 4.5–36 V. The L-line input is not needed. |
| USB logic analyzer (FX2 / "Saleae Logic" clone, sigrok `fx2lafw`) | Debugging | Pins CH1–CH8 (sigrok D0–D7), GND and CLK. Logic inputs only. It must never see 12 V directly. |
| Resistors 47 kΩ + 15 kΩ | K-line probe divider | Scales 14.4 V down to 3.5 V for analyzer CH3 |
| 12 V supply with 1 A inline fuse | ECU + Click VS | Use the same supply or battery that powered the ECU during the KKL tests |
| KKL / VAG-COM 409.1 + 3-pin Fiat/Alfa adapter | Golden reference only | Disconnect it before the Click touches the K-line |
| Bosch M2.10.4 (0261204478) | Target |  |

| Parameter | Proven value | Source |
| --- | --- | --- |
| Idle before wakeup | K-line high ≥ 2 s | script phase 1 |
| Wakeup address | 0x10, 5 baud, 8N1, LSB first, 200 ms/bit (start bit + 8 data + stop bit = 2.0 s) | `--address` default |
| Link speed after wakeup | 4800 baud 8N1 | `--baud` default, confirmed working |
| Sync | ECU sends 0x55 about **180 ms** after the stop bit (187 ms measured on the Mac, including up to \~16 ms of USB delay) | `--probe`, 22 Sep 2026 |
| Keyword | After 0x55 the ECU sends `32 86 04 15 26`. The tools read at most 5 bytes, so more may follow. Byte #1 = **0x86** | `--probe`, 22 Sep 2026 |
| Keyword reply | Complement of the keyword byte = **0x79** | `output.txt` |
| Reply delay | Both tools read up to 5 bytes after 0x55 (stopping early after a 150 ms gap), then wait 8 ms and send. This ECU sends exactly 5, so the reply goes out about 9 ms after the 5th byte | `--byte-timeout 0.15`, `--turnaround-ms 8`. Measure the real gap in the P0 capture |
| Block format | `LEN CTR TYPE DATA… 03`. **LEN counts the 0x03**: block 1 = `0D` → `01 F6` + 10 characters + `03`. The ECU always sends the LEN byte of its first block **twice**, because it doesn't accept the first echo. A 0x03 before the end is data | ESP32 raw trace, 22 Sep 2026. The old "LEN excludes 0x03" note was an illusion caused by the repeated byte |
| Lock-step echo | The receiver echoes every byte inverted, except the final 0x03. The ESP32 waits **20 ms** before each echo and before each byte it sends. At 0 ms the ECU rejects the echo; 10–30 ms all work | `send_block()` / `read_block()` |
| Self-echo | The transceiver's RX mirrors the K-line, so you receive every byte you send. Read it back and discard it | `write_byte_sync()`. The L9637D behaves the same as the KKL |
| ID blocks (3) | Type F6, ASCII reversed: `8744021620` = **0261204478** (Bosch hardware no.), `1497537301` = **1037357941** (Bosch software no.), `86152564` + `FF FF` = **46525168** (Alfa/Fiat part no.) | ESP32 on the real ECU, 22 Sep 2026 |
| Keep-alive | NOP / ACK block, type 0x09. The two sides strictly take turns: the ECU answers every tester block with exactly one block, the NOP included. Read that reply before sending again | `REQ["nop"]` |

## Wire diagram

Four signal wires, one K-line wire, and one ground star point. The Click's mikroBUS TX and RX are named from the Click's own side, so they cross over: **ESP32 GPIO27 goes to the Click's RX pin, and the Click's TX pin goes to GPIO22.** This matches the [schematic v101](https://download.mikroe.com/documents/add-on-boards/click/iso_9141_click/iso-9141-click-schematic-v101.pdf): L9637 pin 4 (TX) connects to mikroBUS RX, and pin 1 (RX) connects to mikroBUS TX.

```mermaid
flowchart LR
  MAC[MacBook M4<br/>USB] -->|USB-C: power + logs| CYD
  MAC -->|USB| LA
  CYD[ESP32-2432S028<br/>CN1] -->|IO27 = K_TX| CRX[Click pin marked RX<br/>= L9637 TX input]
  CTX[Click pin marked TX<br/>= L9637 RX output] -->|K_RX = IO22| CYD
  CYD -->|CN1 3.3V + GND| CPWR[Click pins<br/>+3.3V and GND]
  CRX --- CLK[ISO 9141 Click<br/>JP1 = 3V3]
  CTX --- CLK
  CPWR --- CLK
  CLK -->|K terminal| KL((K-line))
  KL --> ECU[M2.10.4<br/>diag K pin]
  PSU[12 V supply<br/>1 A fuse] -->|VS terminal| CLK
  PSU -->|+12 V, ign.| ECU
  KL --> DIV[Divider<br/>47k over 15k]
  DIV -->|CH3 = K_LINE| LA[Logic analyzer<br/>CH1-CH8, GND, CLK]
  CYD -.->|CH1 = IO27, CH2 = IO22| LA
  GND{{Ground star<br/>ECU GND}} --- CLK
  GND ---|analyzer GND| LA
```

Solid arrows show signal or power flow; the dotted arrow shows the analyzer probes on CN1. Every GND (ESP32 CN1, Click mikroBUS GND, Click VS−, ECU ground, 12 V supply −, divider bottom, analyzer GND) joins at the ground star.

**Logic analyzer pins.** The labels CH1–CH8 correspond to sigrok's names D0–D7, so the sigrok commands in this doc still say `-C D0=…`.

| Analyzer pin | sigrok name | Label in captures | Connect to |
| --- | --- | --- | --- |
| CH1 | D0 | K\_TX | CYD CN1 IO27 |
| CH2 | D1 | K\_RX | CYD CN1 IO22 |
| CH3 | D2 | K\_LINE | Divider node (47 kΩ / 15 kΩ), **never the K-line itself** |
| CH4–CH8 | D3–D7 |  | Leave unconnected |
| GND |  |  | Ground star (ECU GND) |
| CLK |  |  | Leave unconnected. It's an external clock input; sigrok samples on its own clock |

Check the mapping once: connect only CH1 to GND and run `sigrok-cli -d fx2lafw --samples 8 -O bits`. D0 should read all 0s, and the unconnected channels read 1s.

| From | Pin | To | Pin | Wire | Notes |
| --- | --- | --- | --- | --- | --- |
| CYD CN1 | IO27 | Click mikroBUS | **RX** | K\_TX | Crossover. Also probe it with analyzer CH1 |
| Click mikroBUS | **TX** | CYD CN1 | IO22 | K\_RX | Crossover. Also probe it with analyzer CH2 |
| CYD CN1 | 3.3V | Click mikroBUS | +3.3V | Logic supply | Only works with JP1 set to 3V3 |
| CYD CN1 | GND | Click mikroBUS | GND | Ground |  |
| Click | K-L terminal, **K** screw | ECU diag | K-line pin | K-line | Leave the L screw empty. Check which screw is K on the silkscreen |
| 12 V supply + (fused) |  | Click | VS terminal, VS | VS | 4.5–36 V |
| 12 V supply − |  | Click | VS terminal, GND | Ground | Also goes to the ECU ground |
| K-line |  | 47 kΩ → node → 15 kΩ → GND |  | Probe divider | Divider node goes to analyzer CH3 |
| Analyzer | CH1 / CH2 / CH3 / GND (CLK unused) | IO27 / IO22 / divider / ground star |  | Probes | sigrok D0 / D1 / D2, labelled K\_TX / K\_RX / K\_LINE |

**Finding the ECU's K pin:** use the same 3-pin Fiat/Alfa adapter the KKL worked through. On its OBD-16 side, K is pin 7, GND is pins 4 and 5, and +12 V is pin 16. Check with a multimeter which of the 3 pins connects to OBD pin 7. Tap the K-line, GND and +12 V there, for example with an OBD-16 breakout. That uses the path already proven with the KKL.

**3-pin Fiat/Alfa diagnostic connector on this ECU:** pin 1 has no wire to the ECU, pin 2 = **GND**, pin 3 = **K**. There's no L-line. Connect the Click's K screw to **pin 3**, and connect the Click's GND to **pin 2** as well as to the 12 V supply's minus. Without that shared ground the ECU doesn't respond.

## Safety and signal levels

The ESP32 and the analyzer run at 3.3 V logic; the K-line swings to about 12–14.4 V. One wrong wire can kill a GPIO or an analyzer channel, so check these before every power-up.

- [ ] **JP1 on the Click is set to 3V3.** At 5V the Click's RX output idles at 5 V, and GPIO22 is not 5 V tolerant.
- [ ] **Measure the Click's TX pin** (the wire going to GPIO22) with the Click powered but the ESP32 not yet connected. It should idle at about 3.3 V; if it reads higher, stop.
- [ ] **Never connect the K-line, VS or +12 V directly to the ESP32 or the analyzer.** Analyzer CH3 only connects to the divider node.
- [ ] **Check the divider:** 47 kΩ top, 15 kΩ bottom gives 14.4 V → 3.48 V and 12 V → 2.90 V. It reads as logic 1 above the 2.0 V threshold. A K-line low of ≤ 1 V reads as ≤ 0.25 V, logic 0.
- [ ] **One ground.** ESP32, Click, analyzer, 12 V supply and ECU all connect to the ECU ground. The Mac powers both the ESP32 and the analyzer, so its USB ground is already common with them.
- [ ] **Only one tester on the K-line.** Unplug the KKL before connecting the Click. Two pull-ups and two drivers corrupt every byte.
- [ ] **Fuse the 12 V feed at 1 A** and check VS polarity before closing the circuit.
- [ ] **Connect with the power off.** Make all connections with 12 V off, then power up in this order: 12 V, then the ESP32 over USB.

## Step-by-step bring-up

Seven phases, each ending in a pass check you can see on the analyzer. Capture at 100 kHz and save as `.sr`, then decode offline. Long captures sometimes stall on this Mac with `LIBUSB_ERROR_PIPE`. If one hangs, Ctrl-C and run it again.

### P0: Record the golden handshake with the KKL (about 20 min)

This records what "working" looks like on the wire. You'll compare every later phase against it.

1. Wire it the way that already works: KKL → 3-pin adapter → ECU, powered. Add only the divider on the K-line, with analyzer CH3 on the divider node and analyzer GND on ECU ground.
2. Start the capture, then the script, in one line:

```
sigrok-cli -d fx2lafw --config samplerate=100k --samples 1000000 -C D2=K_LINE -o golden.sr & sleep 0.5; python3 tools/kwp71_m2_10_4.py --port /dev/cu.usbserial-AH01164M; wait
```

3. Check the wakeup and decode the bytes:

```
sigrok-cli -i golden.sr -O csv:header=true:dedup=true:label=channel:time=true -o golden.csv
python3 tools/check_wakeup.py golden.csv --channel K_LINE --address 0x10
sigrok-cli -i golden.sr -P uart:rx=K_LINE:baudrate=4800 -A uart=rx-data --protocol-decoder-samplenum > golden_bytes.txt
```

4. In `golden_bytes.txt`, each sample number ÷ 100 = ms. Write down these five gaps:
   - W1: end of the wakeup stop bit → 0x55
   - KW1 → KW2 (0x86)
   - **W4: end of KW2 → start of 0x79** (the most important number)
   - 0x79 → the first ECU block's LEN byte
   - End of an ECU byte → start of the tester's inverted echo

**Pass:** `check_wakeup.py` prints `RESULT: OK - decoded 0x10`, and the decoded stream shows `55`, `86`, `79` and a first block containing ASCII digits.

**Fixed in `kwp71_m2_10_4.py` on 22 Sep 2026:** the tool used to clear its input 190 ms after the stop bit. This ECU's 0x55 arrives at about 187 ms, so it was thrown away, and `--probe` reported `never received 0x55` while the ECU was actually answering (it logged only `32 86 04`). The tool now listens from the end of the stop bit. It also waits for 0x55 by time, skipping stray bytes, instead of reading only 3 bytes.

### Firmware for P1–P5: `kwp71_m2104_esp32.ino`

Every ESP32 phase runs the same sketch unchanged. It's a port of the working Python script and repeats this cycle until it connects:

1. Hold the line idle high for 2 s.
2. Send the 0x10 wakeup on GPIO27, which takes 2.0 s.
3. Wait for 0x55, up to 3 × 800 ms.
4. Wait 1 s, then start again.

Follow it on the Serial Monitor at 115200 baud; the TFT shows the same log with less detail. These are the settings you may tune:

| Constant | Value | Controls |
| --- | --- | --- |
| `IDLE_BEFORE_MS` | 2000 | Idle-high time before the start bit |
| `BIT_MS` | 200 | 5-baud bit time |
| `N_SYNC`, `BYTE_TIMEOUT_MS` | 5, 150 | Reads up to 5 bytes after 0x55 and stops after a 150 ms gap. This ECU sends exactly 5 (`32 86 04 15 26`), so the reply goes out right after the 5th |
| `KW_POS` | 1 | Which byte after 0x55 is complemented (0x86 → 0x79) |
| `TURNAROUND_MS` | 8 | Delay before the keyword reply 0x79 (accepted by the ECU) |
| `ECHO_DELAY_MS` | 10 | Delay before every echo and every byte sent in a block exchange. Changed from 20 to 10 ms for faster rpm updates. 10 ms worked in the sweep, but hasn't been through a full session yet; go back to 20 if echoes get rejected |
| `ECU_LEN_INCLUDES_EOB` | 1 | LEN counts the 0x03 (confirmed on the real ECU) |
| `MAX_LEN_REPEATS` | 3 | How often the ECU may re-send a block's LEN byte (it always does once, for block 1) |
| `SELF_ECHO_TIMEOUT_MS` | 50 | How long to wait for our own byte to loop back through the L9637 |
| DEMO\_MODE | 0 | 1 = simulated dashboard values, no K-line traffic (see Dashboard) |

**How to capture a clean cycle:** start sigrok, then press the CYD's RST button within about 0.5 s. After a reset the start bit comes about 3 s in, so a 9 s capture holds one whole attempt. The first falling edge will be the real start bit, which is what `check_wakeup.py` needs.

### P1: ESP32 pin only (no Click, no 12 V)

1. Flash `kwp71_m2104_esp32.ino`. Connect analyzer CH1 to CN1 IO27 and analyzer GND to CN1 GND. Nothing else is connected.
2. Capture, pressing RST right after the command starts, then check:

```
sigrok-cli -d fx2lafw --config samplerate=100k --samples 900000 -C D0=K_TX -o p1.csv -O csv:header=true:dedup=true:label=channel:time=true
python3 tools/check_wakeup.py p1.csv --channel K_TX --address 0x10
```

**Pass:** `RESULT: OK - decoded 0x10`. The start bit comes after ≥ 2 s of high, and the frame is about 2.0 s long. The Serial log is expected to show `RX transitions …: 0` and `FAIL: no 0x55 sync byte` here, because nothing is connected to GPIO22.

### P2: Click on the bench, 12 V on, ECU not connected

1. Set JP1 to 3V3 and work through the safety checklist. Wire the ESP32 to the Click as in the wire table. Connect VS to the fused 12 V, and connect the divider to the Click's K screw only.
2. Probe CH1 = IO27, CH2 = IO22, CH3 = divider node.
3. Capture 900000 samples with `-C D0=K_TX,D1=K_RX,D2=K_LINE` (press RST), then run `check_wakeup.py` once for each channel.

**Pass:**

- All three channels decode 0x10. K\_LINE is the 12 V copy of the wakeup; K\_RX mirrors it.
- The Serial log shows `RX transitions seen during our own TX pulse: 3`. Anything else means the wakeup isn't getting from the K-line back to GPIO22.
- K\_LINE idles high between attempts, held up by the Click's 4.7 kΩ pull-up.

### P2b: Full session against the simulator (no ECU)

`m2104_sim.py` makes the MacBook and the KKL cable act as the M2.10.4. With it the whole firmware (wakeup, keyword, info blocks and polling) runs at your desk and can be repeated as often as you like, before the ECU is involved.

| Connect | To |
| --- | --- |
| Click K screw | KKL OBD pin 7 (K) |
| Fused 12 V | Click VS **and** KKL OBD pin 16 (the KKL needs 12 V to work) |
| GND | Click GND **and** KKL OBD pins 4 + 5 |

Keep the divider and CH1–CH3 as in P2. **Never have the real ECU on the same K-line.**

1. Start the simulator: `python3 tools/m2104_sim.py --port /dev/cu.usbserial-AH01164M`
2. Press RST on the CYD and watch both logs.

**Pass:**

- The simulator log shows `wakeup: … -> OK`, then `<- 79 keyword reply … -> OK`, the blocks `[info 1]`, `[info 2]` and `[final]`, then `== session up - answering requests ==`, followed by a steady flow of `RAM` / `ADC` answers.
- The ESP32 log shows `== HANDSHAKE COMPLETE ==` and `Batt:` lines, with no `Session lost`.

**Result on 22 Sep 2026:** passed. There were 35 poll cycles in 45 s (about 1.1 s each) with no `Session lost`, self-echo errors or echo errors. The keyword reply went out 160 ms after the keyword byte.

This run found one bug, now fixed. The sketch sent its NOP keep-alive but never read the ECU's reply block, so the next battery request collided with that reply. The log showed `self-echo 0x2 != sent 0x6`, and the session dropped after every first poll cycle.

The displayed values (`Batt: 0.21V RPM: 51 Cool: 140.4C`) are wrong with the simulator. The decode formulas read byte positions where the simulator's placeholder answers have the 0x03 end marker. A P0 capture of the real ECU will show whether the formulas or the placeholders are wrong.

**What the simulator can and can't tell you:**

- **Real values:** 0x10, 4800 baud, 0x55 after \~190 ms followed by 32 86 04 15 26 (keyword 0x86 → reply 79), all three ID blocks, the real battery/rpm/coolant answers, LEN counting the 0x03, and the ECU's two quirks (LEN of the first block sent twice, NAK for the first request).
- **Placeholders:** the gaps between the bytes after 0x55, every other delay, the block type bytes, info block 2, and the data answers.
- **Timing:** USB adds up to about 16 ms per read. A pass shows the firmware's logic is right; the real ECU remains the final test.
- **Expected note:** `note: tester LEN 03, this ECU profile would use 02` is normal. The sketch's `sendBlock()` counts the 0x03 in LEN, and the real ECU accepted that from the Python tool.

Once P0 gives you a real capture, copy the ECU's bytes and delays into a profile:

1. `python3 tools/m2104_sim.py --dump-profile > m2104.json`
2. Edit `m2104.json`.
3. Run with `--profile m2104.json`.

The simulator then replays your ECU instead of the placeholders.

### P3: Connect the ECU and expect 0x55

1. Turn 12 V off. Unplug the KKL. Connect the Click's K screw to the ECU's K pin, and join all grounds. Keep the divider on the K-line.
2. Turn 12 V and ignition on and wait 3 s. Start a capture of all three channels, then press RST.
3. Decode both lines:
   - `sigrok-cli -i p3.sr -P uart:rx=K_LINE:baudrate=4800 -A uart=rx-data --protocol-decoder-samplenum`
   - the same for K\_RX

**Pass:** 0x55 appears on K\_LINE and K\_RX, with a W1 close to P0. The Serial log no longer shows `FAIL: no 0x55`. The sketch's UART starts listening at the beginning of the stop bit, so it can't miss an early 0x55.

**Result on 22 Sep 2026: the first attempts failed, as below. Passed after moving K to pin 3 and adding the shared ground (see P3a).**

1. **Wrong pin first.** With the Click's K wire on the wrong ECU pin, the line was stuck low: `K-line idle level before wakeup: LOW` and `RX transitions: 0`. Taking the wire off the ECU brought back `HIGH` and 3, which proved the ECU side was pulling it low.
2. **Right pin, ECU on.** The line idles high and the wakeup goes out (`RX transitions: 3`), but the ECU sends nothing back: `nothing received at all` on every attempt.
3. **Same pin with the KKL** (Click's K wire removed): `kwp71_m2_10_4.py --probe` gets 0x55 after 187 ms, then `32 86 04 15 26`. So the ECU, its power and this pin are fine.

The KKL wakes the ECU and the Click doesn't. The most likely reason is the L-line: many KKL cables drive OBD pin 15 (L) together with K, and the Click can only receive on L, not drive it.

### P3a: Find out what wakes the ECU

There's one open question: what does the KKL do that wakes the ECU, which the Click doesn't? Two multimeter checks and one wiring test should answer it.

**Test 1: Map the 3-pin adapter** (multimeter on continuity, everything powered off). Check each of the adapter's 3 pins against its OBD socket. Also note which of the 3 pins the Click's K wire was on in P3.

| 3-pin adapter pin | → OBD 7 (K)? | → OBD 15 (L)? | → OBD 16 (+12 V) / 4, 5 (GND)? |
| --- | --- | --- | --- |
| Pin 1 |  |  |  |
| Pin 2 |  |  |  |
| Pin 3 |  |  |  |

**Test 2: Check the KKL cable** (cable unplugged). On its OBD plug, measure between pin 7 and pin 15.

- **About 0 Ω:** the cable ties K and L together, so its wakeup goes out on both lines.
- **Open:** it drives K only.

| Result | Meaning | Next |
| --- | --- | --- |
| An adapter pin goes to OBD 15, and the KKL connects 7 and 15 | The ECU most likely listens for the wakeup on **L** | Test 3 |
| No adapter pin goes to OBD 15 | L isn't the reason | If the Click was on the pin that goes to OBD 7, capture with the analyzer (below) |
| The Click was on a different pin than the one going to OBD 7 | Wrong pin | Move the Click's K wire to that pin and let the ESP32 retry |

**Test 3: Bridge L to K** (only if Test 1 finds an L pin):

1. Unplug the KKL.
2. Connect the Click's K wire to the adapter's K pin, and put a jumper wire from the L pin to the K pin. That copies what the KKL cable does internally.
3. Turn on the ECU and watch the ESP32 log. The ESP32 retries by itself about every 7 s.

**Pass:** `0x55 SYNC received`, then `Keyword[1]=0x86 -> reply 0x79`, then `== HANDSHAKE COMPLETE ==`.

**If Tests 1 and 2 don't settle it:** capture the KKL waking the ECU with the logic analyzer.

- **Probes:** CH3 through the divider on the K pin, and CH4 through a **second** 47 kΩ / 15 kΩ divider on the adapter's other signal pin.
- **What it shows:** which line carries the wakeup, and which one the ECU answers on.
- **What else it gives you:** the golden capture from P0, including whether the ECU sends more than 5 bytes after 0x55.

**Result of P3a (22 Sep 2026):** Test 1 answered it. There's **no L-line**: connector pin 1 has no wire to the ECU, pin 2 is GND and pin 3 is K. The earlier symptoms fit the Click's K wire being on pin 2 at first (line stuck low), then on pin 1 (idle high, no answer). With K on **pin 3** and the Click's GND on **pin 2**, the ECU answers the ESP32: 0x55 at 192 ms after the stop bit, then `32 86 04 15 26`.

### P4: Keyword reply

**Pass:** the Serial log shows:

```
0x55 SYNC received
Bytes after 0x55: xx 86
Keyword[1]=0x86 -> reply 0x79
Handshake OK - addr 0x10
```

There's no `WARN: no clean self-echo of the reply` message. K\_LINE shows `55 xx 86 79`. Compare the 86→79 gap with W4 from P0. If it differs by more than a few ms, adjust `BYTE_TIMEOUT_MS` (the main part of the gap) and `TURNAROUND_MS`.

**Result on 22 Sep 2026: passed.** The ESP32's raw trace shows 0x55 at 192 ms after the stop bit, then `32 86 04 15 26` at 10 ms intervals. Our 0x79 went out 7.4 ms after the 5th byte, and the ECU accepted it: its first block followed about 100 ms later.

### P5: Info blocks, echo and keep-alive

The sketch reads 2 info blocks, sending an ACK (type 0x09) after each, then the ECU's final block. After that, each poll cycle is four request / reply exchanges: battery, rpm, coolant, and the NOP keep-alive. The sketch reads the ECU's reply to every one, including the NOP, then waits 300 ms. It stops the cycle at the first failed exchange and reconnects. The Python REPL now reads the ECU's reply to its ACK after each command in the same way.

How `readBlock()` handles a block (corrected 22 Sep 2026 from a raw trace on the real ECU):

- **LEN counts the 0x03** (`ECU_LEN_INCLUDES_EOB` = 1). A 0x03 before that point is data, such as a counter of 03.
- **The ECU always sends the LEN byte of its first block twice,** because it doesn't accept our first echo, even with a 30 ms delay. `readBlock()` echoes the repeat without storing it and logs `ECU repeated LEN 0xd 1x`. Before this fix, the repeat was stored as the counter, our ACK went out with counter `0E` instead of `02`, and the ECU answered with a NAK.
- **Every echo and every sent byte waits `ECHO_DELAY_MS` = 20 ms.** Echoing at 0 ms made the ECU give up after three LEN repeats.
- **Fallback:** if the 0x03 isn't where LEN says, it logs `expected EOB after N bytes … scanning on` and keeps searching up to LEN + 4 bytes.
- **Raw trace:** when a block fails, the sketch prints every byte with a microsecond timestamp: `TX` sent, `SE` self-echo, `RX` from the ECU.

`kwp71_m2_10_4.py` and `m2104_sim.py` were corrected the same way on 22 Sep 2026: LEN counts the 0x03, a repeated LEN is echoed without being stored, the counter starts at 0, and every echo waits `--echo-delay-ms` (default 10). The simulator also copies both ECU quirks. Both were tested against each other over a simulated K-line, not against the ECU.

**Pass:**

- A `block ctr=…` line shows the ASCII part number reversed, `'8744021620'` (0261204478).
- The log then shows `== HANDSHAKE COMPLETE ==`.
- `Batt:` lines keep coming for ≥ 30 s with no `Session lost` message.
- Compare the K\_LINE decode with `golden_bytes.txt` byte by byte.

**Result on 22 Sep 2026: passed.** The log shows `== HANDSHAKE COMPLETE (echo delay 20 ms) ==`, and the ESP32 then polled for 40 s with no `Session lost`. The counters ran 01 → 02 → … without a NAK.

| ID block | As sent | Reversed | Meaning |
| --- | --- | --- | --- |
| 1 | 8744021620 | 0261204478 | Bosch hardware no. |
| 2 | 1497537301 | 1037357941 | Bosch software no. |
| 3 (final) | 86152564 + FF FF | 46525168 | Alfa/Fiat part no. |

Reply blocks are `[counter, type, data…, 03]`, so the first data byte is `blk[2]`. The Python formulas read one byte too far, which gave 0.21 V and 140 °C.

| Value | Request | Reply | Decode | Checked against |
| --- | --- | --- | --- | --- |
| Battery | `01 01 00 36` | type FE, 1 byte | `blk[2]` × 0.073 V | 164 raw with the ignition on, 197-200 running: 12.0 V and 14.4 V |
| RPM | `01 02 00 3B` | type FE, 2 bytes | `blk[3]` × 10, or `blk[2]` × 40 once `blk[3]` hits FF | idle 20/82 → 800-820 rpm; a held 3000 rpm averaged 74.6 on `blk[2]` → 2984 rpm |
| Coolant | `08 03` | type FB, 2 bytes | NTC polynomial on `blk[3]` | 202 raw cold → 10 °C; 44-51 raw hot → 91-96 °C |
| Air temp | `08 05` | type FB, 2 bytes | the same NTC polynomial | 197 raw → 13 °C with a 14 °C ambient |
| Air quantity | `08 04` | type FB, 2 bytes | `blk[3]` − 6, as a raw count | 6 engine off, ~41 idle, ~70 at 3000 rpm |

The first request after the handshake always gets a NAK (type `0A`). It shows as `--`, and the next cycle gets data.

### What the ECU answers: the ADC map

Found on 12 Oct 2026 with [`tools/kwp71_discover.py`](../tools/kwp71_discover.py), by sweeping all 16 channels in five conditions (cold and engine off at 14 °C; ignition on with a pedal sweep; idle; idle to high idle; a held 3000 rpm) and diffing the runs. `08 <ch>` returns type FB and two bytes, high then low; everything useful is in the low byte.

| Ch | What | Engine off | Idle | 3000 rpm | How it was identified |
| --- | --- | --- | --- | --- | --- |
| 00 | Throttle position | 23 | 23 | 36-40 | 23 → 209 during the pedal sweep with the engine off |
| 01 | Battery | 164 | 197-200 | 198 | Identical to RAM `0036` in every run |
| 03 | Coolant | 202 cold, 47 warm | 58-71 | 44-51 | Known; the NTC curve gives 10 °C cold and 91-96 °C hot |
| 04 | Air quantity | 6 | 40-42 | 67-72 | Follows engine load, and does **not** move with the pedal while the engine is off |
| 05 | Intake air temperature | 197 | 172-182 | 159-163 | 197 reads 13 °C on the coolant curve against a 14 °C ambient, and falls as the bay warms |
| 06 | Unidentified | 131 | 68-88 | 46-75 | Changes with load, but nothing lines it up with a known quantity |
| 07 | Lambda | 93 | 152-158 | 165-167 | Fixed with the engine off, rises and wanders with the engine running |
| 02, 08, 0B | Unused inputs | 254-255 | 254-255 | 254-255 | Pinned at the pull-up in every state |
| 09, 0A | Unused inputs | 0 | 0 | 0 | Always zero |
| 0C-0F | Do not exist | no reply | no reply | no reply | No answer at all |

The runs themselves are in `captures/`, and each row carries its condition in a `note` column:

| File | Condition | Also holds |
| --- | --- | --- |
| `discover.csv` | cold, engine off, 14 °C ambient | a RAM sweep of `0000`-`00FF` |
| `discover1.csv` | first run, warm idle | |
| `discover_pedal.csv` | ignition on, engine off, pedal swept | the throttle channel moving on its own |
| `discover_rev.csv` | idle, then revved | |
| `discover_rev3.csv` | idle, high idle, idle | |
| `discover_3000.csv` | a held 3000 rpm, 8 passes | the rpm calibration |

Two things from the RAM sweep worth knowing: `0036` mirrors ADC 01, and `00D4`-`00FF` is all `AA`, so that end is unused.

Still open:

- The **kg/h scale for air quantity**. The idle and 3000 rpm counts suggest roughly 0.37 kg/h per count for a 2.0 litre engine, which is an expectation rather than a measurement, so the dashboard shows the raw count.
- **ADC 06**, which clearly carries something load-dependent.
- A **measured point for the battery scale**. Both in-car states agree on 0.073 V/step with no offset, and the dashboard reads plausibly on the car, but neither point was ever compared with a meter; the sketch logs the raw byte as `[BATT] raw …` for exactly this.
- **Confirming air temp and air quantity.** Both return sensible data on the car. Air temp can be checked by warming the intake sensor, which should raise the reading and lower the raw byte; air quantity has no reference yet.
- A short pause after the handshake, to avoid the first NAK.

## Dashboard on the CYD screen

**There are two pages, and a tap anywhere switches between them.** Page 1 is the
dashboard below; page 2 is the diagnostics page further down.

The CYD's screen shows a live dashboard: an L-shaped rpm bar wrapping four values in a 2 × 2 grid, each with its own symbol, plus the connection status and the ECU's ID numbers. A boot splash shows the car badge for 5 s first. The detailed log stays on the Serial Monitor at 115200 baud. The layout was confirmed in demo mode on 22 Sep 2026, and the whole dashboard, smoother polling included, ran against the car on 12 Oct 2026.

```text
┌──────────────────────────────────────────────┐
│ (badge) ALFA 145 QV    Shift!   ● CONNECTED  │  status bar
│                                            8 │
│       (o) RPM     ┌(+-) BATT┬(T~) COOL┐      │
│                   │    V    │   °C    │    7 │
│        3250       │  12.27  │  91.0   │      │
│                   ├(T-)AIR T┼(>>)AIR Q┤    6 │
│  ┌──────────────┐ │   °C    │   raw   │      │
│  │ rpm history  │ │  13.0   │   41    │    5 │
│  └──────────────┘ └─────────┴─────────┘      │
│ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░ │  bottom leg
│ 0       1       2       3       4            │  scale
│ HW 0261204478  SW 1037357941  PN 46525168    │  bottom line
└──────────────────────────────────────────────┘
```

`Shift!` only appears past 6500 rpm. The `(..)` sketches stand for the drawn
symbols: a tachometer dial, a battery, a thermometer in water, a thermometer in
air, and three airflow arrows.

| Element | Position (320 × 240) | Shows | Colours |
| --- | --- | --- | --- |
| Status bar | top, 0–24 px | car badge (20 px), "ALFA 145 QV", and the connection status on the right | see the status table |
| RPM bar | an L: bottom leg x 6–314 at y 190–204, then up the right edge x 300–314 from y 190 to y 30, 14 px thick | 0–4000 along the bottom, 4000–8000 up the side, labelled in thousands under the bottom leg and left of the side leg | bar green < 5000, yellow 5000–6499, red ≥ 6500; track grey, red zone dark red above y 90 |
| Scale notches | 1 px across the full bar thickness at every 1000 rpm (`TICK_EVERY`), so 6 interior notches: 1–3 on the bottom leg, 5–7 on the side | segments the bar, lining up with the labels. They go **inside** the bar because nothing fits outside it — the cells and the history box come down to y 186, 4 px above the bar | background colour, so they read as gaps in both bar and track |
| RPM number | left block, centred on (67, 108) | the speed in 10 rpm steps, in font 7, the library's 7-segment face. 32 px per digit, so `8888` is 128 px and spans x 3–131, 3 px clear of the cells. **Clamped to `RPM_MAX`**: `blk[2]` × 40 can reach 10200, and a fifth digit would run off the edge and into a cell. The serial summary still logs the unclamped value | white, grey when there's no value |
| RPM history | framed box x 6–128, y 138–186; plot area 118 × 42 px | a filled rpm trace, one sample per 250 ms, so about **30 s** of history scrolling leftwards. A dim line marks `RPM_RED` behind it. 236 bytes of RAM | each column takes its own sample's zone colour, so peaks go yellow then red |
| RPM heading | dial + `RPM` centred as one group above the number | a tachometer dial: circle, needle, hub | follows the rev zone, like the bar |
| `Shift!` | centre of the status bar, x 144–176 | blinks at 4 Hz past `RPM_RED` | red |
| Value grid | inside the L: 2 × 2 cells of 74 × 76 px from (134, 30) | symbol + short title, unit centred below, then the value | per value, see below |
| Cell symbols | 17 × 13 px at +5, +4 inside each cell | battery with terminals, thermometer in water, thermometer in air, three airflow arrows | **each takes its own value's colour**, so every cell doubles as a warning lamp |
|  Battery (`BATT`, top left) |  | volts, 2 decimals | red under 11.5 and over 15, green in between |
|  Coolant (`COOL`, top right) |  | °C, 1 decimal | light blue up to 65, green 66–99, red 100 and over |
|  Air temp (`AIR T`, bottom left) |  | °C, 1 decimal, from ADC 05 | light blue < 5, green 5–60, red > 60 |
|  Air quantity (`AIR Q`, bottom right) |  | raw count above no-flow, from ADC 04. Labelled `raw`, not `kg/h`, because the scale is unknown | white. The titles are short because a 17 px symbol shares the row |
| Bottom scale | under the bottom leg, y 206–214 | 0 to 4 in thousands | grey |
| Bottom line | y 219–235 | the ID numbers once connected; before that, the latest step | grey / the step's colour |

`--` means there's no value yet, for example before the first reading, or after the ECU's first-request NAK.

The dot left of the status text **breathes** on a 1.4 s cycle while the session
is not up, and sits steady once it is. A stalled handshake therefore never looks
like a working one.

| Status | Colour | Meaning |
| --- | --- | --- |
| STARTING | yellow | booting |
| CONNECTING | yellow | wakeup and handshake running |
| CONNECTED | green | session up, values live |
| NO ECU | red | no 0x55 after the wakeup: check pin 3 / GND on pin 2, and that the ECU is on |
| HANDSHAKE FAIL | orange | 0x55 seen, but the ID blocks failed (see the raw trace in the Serial log) |
| RECONNECTING | orange | session lost; values go back to `--` and a new handshake starts |
| DEMO | cyan | demo mode, no K-line traffic |

### How the values update

- **Polling:** one request/reply at a time, with no pauses; the requests themselves keep the session alive. Of every 10 exchanges, 6 read rpm and one each reads battery (#1), coolant (#3), air temp (#5) and air quantity (#7); the summary log line goes out on #9. Rpm keeps the majority of the slots because it is the only value that moves fast.
- **Slower values:** battery, coolant, air temp and air quantity each refresh about every 3 s. That is far more often than any of them actually changes. A reply that doesn't decode keeps the previous value. Expected rate: about **3–4 new rpm values per second**, from about 13 bytes per exchange at 10 ms echo delay. That's an estimate, not measured yet.
- **Two cores:** the K-line code runs in `loop()` on core 1. The screen is drawn only by `dashTask` on core 0, at about 25 frames per second. The K-line code just hands over numbers (plain variables) and texts (through a lock, a mutex), so drawing can never delay a K-line echo.
- **Smooth bar:** every frame, the bar moves 18% of the remaining distance to the latest reading (`RPM_SMOOTHING`), reaching about 90% within about 0.5 s. So it glides instead of jumping between readings.
- **No flicker:** only the stretch of bar that changed is redrawn — growing paints bar colour over track, shrinking paints track back over bar — and numbers overwrite themselves in place.
- **Two resolutions, on purpose.** The corner is always `RPM_MID`, half of `RPM_MAX`, so the long bottom leg carries 0–4000 at 13 rpm per pixel and the shorter side leg carries 4000–8000 at 25 rpm per pixel. The scale is therefore not linear across the whole L; the labels in thousands are there so that is visible rather than misleading. Idle and cruising sit on the fine half, which is where the resolution is worth having.

### Animation

All of it runs in the display task on core 0, so none of it can delay a K-line
echo. Everything that blinks shares one phase (`SHIFT_BLINK_MS`), so the red
indicators pulse together rather than against each other.

| What | When | Looks like |
| --- | --- | --- |
| Boot sweep | once, after the splash | the bar runs 0 → 8000 → 0 in about 1.2 s with the number following, the way a cluster wakes up. It also proves both legs and the red zone draw before any data arrives |
| Peak hold | always | a **steady white** 3 px tick rides the highest rpm of the last 1.2 s, then slides back at about 1000 rpm/s. Hidden within `PEAK_MIN_GAP` of the bar's own tip, where it would only fight the shift light |
| Shift light | past `RPM_RED` | the last 16 px of the bar is **blanked to the background** at 4 Hz, so the red bar looks like it is pulsing shorter |
| `Shift!` | past `RPM_RED` | red text blinking in the middle of the status bar, on the same phase |
| Breathing dot | while not connected | the status dot fades in and out on a 1.4 s cycle |
| Fresh-data cue | as each value lands | the cell's title lights white and fades back to grey over about 320 ms, so a session that has stopped polling is obvious |

**Why the two red-zone indicators differ.** Past 6500 the bar is already red, so
a red marker on it would be invisible. The blinking tip therefore means "you are
in the red zone" and is done by blanking, while the peak tick stays steady white
and means "this is the speed you actually hit". Keeping one blinking and one
steady is what lets the two be read apart.

| RPM history | every 250 ms | a new sample lands on the right and the trace scrolls left. Each column is cleared and redrawn in one pass, so no column is blank long enough to see |
| Onboard LED | past `RPM_RED` | the CYD's own red LED (GPIO 4, active low) flashes on the same phase, so the shift warning carries in peripheral vision without looking at the screen |

### Backlight and the light sensor

The backlight is on LEDC PWM rather than a plain digital HIGH, driven by the
CYD's own light sensor (LDR on GPIO 34), so the screen dims at night instead of
glaring. The running average moves 5% per frame (`LDR_SMOOTH`), slow enough that
a hand passing over the sensor or oncoming headlights don't make it pump, and the
duty never drops below `BL_MIN` so it cannot go dark.

**Calibrating it.** `LDR_DARK` and `LDR_BRIGHT` are the raw readings at the two
ends, and `map()` handles them in either order — **so if the screen dims in
daylight instead of at night, swap those two numbers**, that is this board's
sensor polarity and nothing else needs to change. The raw reading is printed once
at boot (`LDR at boot: …`) and appears in every summary log line as `LDR:`, so
the two ends can be read off the board itself: cover the sensor, tap reset, and
note the number. Measured on this board: **657 in a lit room**, and it read 0
once in brighter light, so lower really is brighter here. `LDR_DARK` is still a
guessed 3000 and wants a real covered-sensor reading.

**The LED is inside the case.** The printed lid has a window for the screen and
nothing else, so once the board is cased the shift LED is hidden. It needs either
a small hole in the lid above the LED or a light pipe; until then the LED only
helps on the bench. Nothing in `tools/make_case.py` cuts that hole yet.

**Who paints the bar.** `barBottom()` and `barSide()` are the only two functions
that paint a stretch of a leg, and each one stamps the scale notches back in over
whatever it just filled. Everything else — the track, the bar itself, and the
slice restored when an overlay moves — goes through them, so no painted stretch
can come out without its notches. That is why the notches survive the bar growing
and shrinking over them without being a separate overlay that would need its own
redraw logic.

**No stale pixels.** The peak tick and the shift light sit on top of the bar, so
they are drawn after it, and `gaugeSlice()` repaints the slice they vacate with
the right colour — bar below the current reading, track above it. A colour-zone
change repaints both legs and wipes them, so `gaugeSet()` raises
`gaugeFullRepaint` and they redraw unconditionally on that frame.

This is checked by simulating 1200 frames of a full sweep, including a lost
session, and comparing every pixel of both legs against a freshly built ideal on
each frame. The check is run twice: once as written, which matches on every
frame, and once with `gaugeFullRepaint` suppressed, which mismatches — 42 pixels
wrong at the first zone crossing. Without that second run the first proves
little, which is exactly how the missing `gaugeFullRepaint = true` survived an
earlier version of this simulation: it modelled the intended design rather than
the code, and the flag was declared and read but never set, so the peak tick
could vanish for up to its hold time after each zone crossing.

| Constant | Value | Controls |
| --- | --- | --- |
| `RPM_MAX`, `RPM_WARN`, `RPM_RED` | 8000, 5000, 6500 | bar end, start of yellow, start of red |
| `RPM_MID` | `RPM_MAX / 2` | the rpm at the corner, where the bar turns upward |
| `TICK_EVERY` | 1000 | spacing of the scale notches cut across the bar |
| `G_W`, `G_BX0`/`G_BX1`/`G_BY`, `G_VX`/`G_VY0`/`G_VY1` | 14, 6/314/190, 300/190/30 | thickness and the two legs of the L |
| `RPM_CX`, `RPM_CY` | 67, 108 | centre of the big rpm number |
| `ICON_W`, `ICON_H` | 17, 13 | box every symbol is drawn in |
| `BOOT_SWEEP_STEPS`, `BOOT_SWEEP_MS` | 28, 20 | length and speed of the self-test sweep |
| `PEAK_HOLD_MS`, `PEAK_FALL`, `PEAK_MIN_GAP`, `PEAK_W` | 1200, 40, 120, 3 | peak tick: hold time, fall rate per frame, how close to the tip it hides, thickness |
| `SHIFT_BLINK_MS`, `SHIFT_W`, `SHIFT_TEXT` | 125, 16, `"Shift!"` | blink phase shared by every red indicator, length of the blanked tip, the warning text |
| `DOT_PULSE_MS` | 1400 | breathing period of the status dot |
| `FLASH_FRAMES` | 8 | frames a cell title stays lit after a new value |
| `RPM_FONT` | 7 | the 7-segment face for the big number (`LOAD_FONT7`, already in TFT_eSPI's setup; `LOAD_GFXFF` is **not** enabled, so the Free Fonts are unavailable) |
| `SPARK_X`/`SPARK_Y`/`SPARK_W`/`SPARK_H`, `SPARK_MS` | 6/138/122/48, 250 | the history box and its sample interval |
| `BL_FREQ`, `BL_BITS`, `BL_MIN`, `BL_MAX` | 5000, 8, 40, 255 | backlight PWM, and the floor it never dims past |
| `LDR_DARK`, `LDR_BRIGHT`, `LDR_SMOOTH`, `LDR_STEP` | 3000, 300, 0.05, 4 | the sensor's two ends, how fast it follows, smallest duty change worth writing |
| `RGB_R_PIN`/`RGB_G_PIN`/`RGB_B_PIN`, `LDR_PIN` | 4/16/17, 34 | onboard LED (active low) and light sensor |
| `TOUCH_IRQ_PIN`, `TOUCH_HOLD_FRAMES`, `TOUCH_DEBOUNCE_MS` | 36, 3, 350 | PENIRQ, how long a press must hold, and the gap between accepted taps |
| `ADC_SWEEP_N`, `DTC_MAX`, `DTC_EVERY_CYCLES` | 0x0C, 8, 10 | channels swept on page 2, raw fault bytes kept, and how often the dashboard asks for faults |
| `DIAG_X`/`DIAG_Y`, `DIAG_COLS`, `DIAG_CELL_W`/`DIAG_CELL_H` | 6/64, 4, 77/25 | the ADC grid on page 2 |
| `DASH_FRAME_MS` | 40 | frame time of the display task (\~25 fps) |
| `RPM_SMOOTHING` | 0.18 | how fast the gauge follows a new reading (0.30 first, looked too twitchy) |
| `SPLASH_MS` | 5000 | how long the boot splash stays up (it delays the first handshake by that much) |
| `DEMO_MODE` | 0 | 1 = simulated values, no K-line traffic |

### Page 2: diagnostics

Tap anywhere to reach it, and again to go back.

| Element | Position | Shows |
| --- | --- | --- |
| Fault line | y 30–46, the headline of the page | `FAULTS none stored` in green for the measured `FC 00` reply, `FAULTS raw XX XX …` in **red** for anything else, `FAULTS --` in grey before the first read |
| ADC grid | 4 × 3 cells of 77 × 25 px from (6, 64) | the raw low byte of every channel that answers, 00–0B. Channel numbers are white for the six identified ones and grey for the rest |
| Session extremes | y 160–220 | max rpm, and the min-to-max range of coolant and battery seen this session |

**Faults are shown raw, not decoded.** Request `07` answers with a type `FC`
block, and a single `00` byte means nothing is stored — that is what this ECU
returned on all 28 reads during the discovery runs. The byte layout of an actual
fault has never been seen, so decoding it in the firmware would be invention.
The serial log prints a `[DTC]` line whenever the bytes change.

To see the red branch without a fault in the car,
[`tools/m2104_sim.py`](../tools/m2104_sim.py) can fake one:

```sh
python3 tools/m2104_sim.py --port /dev/cu.usbserial-XXXX --fault "01 23 45"
```

That needs the bench wiring from P2b. The simulator's own reply to `07` was a
guess (empty data) until the discovery runs measured it; it now answers `FC 00`.

**The poller changes what it asks for on this page.** Instead of the 10-slot
rotation it sweeps the 12 ADC channels plus one fault read, 13 exchanges in all,
so the page refreshes in about 3.5 s. It deliberately stops at channel `0B`:
`0C`–`0F` never reply, and asking for them would look like a lost session and
trigger a reconnect. Rpm is not polled while this page is up, so the gauge is
stale when you go back — it catches up within a frame or two.

### Logging to the SD card

Every drive is written to the CYD's own micro-SD slot as a CSV, so values can be
looked at afterwards and intermittent faults chased.

| | |
| --- | --- |
| File | the next free `/logNNNN.csv`, chosen at boot, so a drive never overwrites an earlier one |
| Columns | `time_s,link,rpm,batt_v,coolant_c,airtemp_c,airqty_raw,dtc` |
| Rate | one row per second |
| Empty fields | a value that has not arrived yet is written as nothing, not as a sentinel, so the CSV loads cleanly |

**Faults are logged on every row**, as the raw bytes with no spaces (`00` for
nothing stored, `012345` for a reply of `01 23 45`), so the fault state always
sits beside the conditions at that instant rather than in a separate file that
has to be lined up afterwards. The field is empty until the first read.

For that to mean anything while driving, **the dashboard polls request `07` too**
— not just the diagnostics page, which would otherwise be the only thing that
ever asked. Faults change rarely, so it goes out once every `DTC_EVERY_CYCLES`
(10) rotations of the 10-slot poll, roughly every 30 s, costing one rpm reading
each time. Both pages go through the same `pollDtc()`, so they store and log it
identically.

**Pins and bus.** CS 5, SCK 18, MISO 19, MOSI 23 — its own pins, and its own SPI
peripheral, separate from the one driving the display. Both were confirmed
working in the same sketch before this was built on.

**It runs in its own task on core 0, and that is not optional.** A row normally
takes 6–7 ms to write and flush, but a measured spike of **186 ms** turned up on
the third row, where the card updated its allocation table. In the K-line loop
that would have overrun the echo timing by an order of magnitude and dropped the
session. On core 0 it cannot touch the protocol; the worst it can do is make the
display stutter for a frame.

**Every row is flushed.** In a car the power can disappear without warning, so
one lost second beats losing the whole file to an unwritten buffer.

**Logging starts at the first session and then keeps going through dropouts**,
with the `link` column reading `up` or `down`. A gap is exactly what you want
recorded when something is cutting out intermittently. If a write fails — card
pulled, or full — logging stops and says so on the serial log rather than
retrying into a void.

The diagnostics page shows the file name and the row count, in green while
logging and grey when there is no card.

**The card is never formatted or cleared.** Logging only ever adds one file. The
card in this unit already held an unrelated device's configuration, and wiping
it was neither needed — it mounted first try as FAT — nor safe. To wipe one
deliberately, take it out and do it on a computer.

### Touch

The XPT2046 is on **its own SPI bus** on this board (CS 33, MOSI 32, MISO 39,
CLK 25), which TFT_eSPI's built-in touch cannot drive, and `User_Setup.h` has no
`TOUCH_` settings at all.

None of that matters here, because switching page needs only press detection and
never a coordinate: **PENIRQ on GPIO 36 is low while the panel is pressed**, so
one `digitalRead` does the job — no second SPI bus, no library, and nothing to
calibrate. If a later page ever needs on-screen buttons, that is when the
coordinate side has to be built and calibrated.

GPIO 36 is input-only with no internal pull-up, so PENIRQ is held high only by
the touch controller, with the display's SPI and the backlight PWM as noisy
neighbours. A press therefore has to read low for `TOUCH_HOLD_FRAMES` frames in
a row (120 ms) before it counts; a finger easily lasts that long and a spike does
not. Measured with nobody touching the screen: 0 switches in 12 s.

### Screen setup for this CYD

This panel needs three settings that the TFT\_eSPI `User_Setup.h` doesn't make. They're in `setup()` of the sketch, so the library stays unchanged:

1. **Backlight on:** pin 21 high. TFT\_eSPI only switches it on when `TFT_BACKLIGHT_ON` is defined. Without this the screen stays black.
2. **Colour inversion off:** `tft.invertDisplay(false)`. The ST7789 driver switches inversion on during `init()`, which made black show as white.
3. **BGR colour order:** MADCTL = `MX | MV | BGR`, written after `setRotation(1)`. Without it, red and blue are swapped.
4. **Byte swap for images:** `tft.setSwapBytes(true)` around each `pushImage()` (and off again afterwards). The library's own drawing handles byte order itself; `pushImage()` doesn't, and without it the badge's colours come out wrong.

The alternative, which covers all sketches, is to add `#define TFT_BACKLIGHT_ON HIGH`, `#define TFT_INVERSION_OFF` and `#define TFT_RGB_ORDER TFT_BGR` to `User_Setup.h`.

### Boot splash and the badge

On power-up the screen shows the car badge (200 × 200) with "ALFA 145 QV" and "by Paul Velzeboer" under it, for `SPLASH_MS` = 5 s. A 20 px badge also sits in the status bar.

The image is **not in the repository**: a car maker's logo is their trademark, so `logo.png` and the generated `logo_data.h` are in `.gitignore`. Generate the header from your own image:

```sh
pip install pillow
python3 tools/make_logo_header.py logo.png
```

It cuts the background away (flood fill from the edges, so white inside the badge stays), crops to a square, scales to 200 px and 20 px, and writes RGB565 arrays. Options: `--big`, `--small`, `--out`, `--bar`. Without the header the sketch still builds, thanks to `__has_include`: the splash is then text-only and the status bar has no badge.

### Demo mode

Set `#define DEMO_MODE 1` and upload to check the screen without an ECU.

- **RPM:** sweeps from 800 to 7600 and back every 8 s.
- **Battery:** 12.4 V at idle, about 14.1 V above 1000 rpm.
- **Coolant:** warms from 20 to 90 °C in a minute.
- **Air temp:** rises from 20 to 38 °C.
- **Symbols and animation:** all of it runs, so demo mode is the way to judge the look — the sweep crosses every colour zone, so the red tip flash, `Shift!` and the peak tick all appear.
- **Air quantity:** `1.2 × √rpm`, which is the shape the measured counts have — 41 at idle and about 70 at 3000 is not a straight line.
- **Screen:** shows the ECU's real ID numbers, and the status says **DEMO**.

**Page 2 is filled in too**, so the diagnostics page can be judged without an ECU:

| Channel | Demo at idle | Demo at 3000 | Measured on the car |
| --- | --- | --- | --- |
| 00 throttle | 27 | 38 | 23 shut, 36–40 at 3000 |
| 04 air quantity | 39 | 71 | 41 idle, 67–72 at 3000 |
| 06 unidentified | 75 | 51 | 68–88 idle, 46–75 at 3000 |
| 07 lambda | 156 | 158 | 152–158 idle, 165–167 at 3000 |

Every one of those is **fitted to the discovery runs**, not invented, and lands
inside all eight measured ranges. Channels 01, 03 and 05 are derived from the
demo's own battery and temperatures, 02/08/0B sit at their real 254–255 and
09/0A at 0. The two temperatures use `ntcRaw()`, which finds the count by
searching the real `ntcTemp()` curve rather than using a second hand-written
polynomial that could drift from it; it round-trips to within 0.09 °C.

One deliberate inaccuracy: lambda reads 93 on the car with the engine stopped,
which demo never shows, because demo always has the engine running.

**The fault line cycles both states** — `none stored` in green for 18 s, then a
fake `01 23 45` in red for 6 s. That red branch has never run against the car,
which has no faults, so demo is the only way to see it on the device itself
(`tools/m2104_sim.py --fault` is the other, over the K-line).

**Session extremes** fill in as well. They are tracked in `dashValues()`, the one
funnel both the poller and demo pass values through; while they were tracked at
each decode inside the poller instead, demo mode never filled them and page 2
showed `--`.

Values are fed every 300 ms, about the real update rate, so it also shows how the smoothing will look. Set it back to `0` for the ECU; in demo mode the ESP32 never touches the K-line.

## 3D-printed case

`tools/make_case.py` generates the case as STL and 3MF files into `hardware/`. Every dimension is a named parameter at the top of that script, so a measurement that turns out wrong is one number and a re-run. Nothing has been printed yet except the Click holder coupon.

| File | What | Size |
| --- | --- | --- |
| `case_clickfit.stl` | Coupon with just the Click holder. **Confirmed to fit on 5 Oct 2026** | 37 × 39 × 13.9 mm |
| `case_fitcheck.stl` | Coupon with the CYD hole pattern, short posts | 122 × 56 × 7.8 mm |
| `case_bottom.stl` | Shell: floor, walls, CYD posts, Click holder, four screw ears | 143.2 × 60.8 × 24.1 mm |
| `case_lid.3mf` | Lid + badge as one object in three parts, for multi-colour printing | 143.2 × 60.8 × 11 mm |
| `case_lid.stl`, `case_lid_badge_*.stl` | The same three parts separately |  |
| `case_lid_embossed.stl` | Lid with the badge raised 1.4 mm, for one colour | 143.2 × 60.8 × 12.4 mm |

### What the measurements come from

- **CYD outline and holes** from the [manufacturer drawing](https://mischianti.org/wp-content/uploads/2025/04/ESP32-2432S028-Cheap-Yellow-Display-Dimensions.jpg.webp): board 86.0 × 50.0 mm, hole centres 4.0 mm in from each edge, so the pattern is 78 × 42 mm. Posts carry a 2.7 mm pin that enters the board hole.
- **ISO 9141 Click** from its DXF (`iso_9141_click_v101.dxf`): outline 25.40 × 28.57 mm, two 8-pin headers 22.86 mm apart (1.27 mm from the long edges), pins running 3.81–21.59 mm along the board.
- **Still estimated:** the CYD's hole diameter (3.0 mm), where the display's active area sits on the board, the terminal height under the Click (`CLICK_TERM_STACK` = 7.0 mm) and the pin height above it (`CLICK_PIN_H` = 8.0 mm). The last two set the whole case height — measure them.

### The Click holder

The Click's own mounting holes aren't usable, and its headers sit only 1.27 mm from the long edges, so nothing may grip those edges: a rail there rides over the pins while the board slides in. Instead:

- **A pocket at the back edge** takes the last 3 mm of the board, past the final pin.
- **Two ledges** support the front corners from below, clear of the screw terminals in the middle, with ramped snap nubs on top that also clear both pin columns.
- **The board sits 8.5 mm above the floor**, the terminals hanging underneath, **pins up**. Wire the terminals, slide the back edge into the pocket, press the front down until it snaps.

### Case and lid

- **The Click's pins set the inner height** (19.1 mm), because the lid is closed over the Click. The CYD posts are then derived, 11.5 mm, so the display ends up 2 mm under the lid instead of down a well.
- **One opening only:** the Click's terminal wires, in the front wall. Every other wall is solid — no USB cut-out, and nothing behind the CYD where its micro-SD slot sits.
- **The lid drops on with a 6 mm lip**, 0.3 mm clearance, no screws.

### The badge

The quadrifoglio is **traced from an image** by `tools/trace_badge.py`: green pixels and the outline around them become the clover, the remaining grey becomes the triangle, filled solid. It writes `tools/badge_data.py`, which the case generator picks up automatically; without it the generator falls back to a clover drawn from curves, so the files still build.

The badge is **inlaid, not raised**: lid, triangle and clover all end flush at the lid surface, with the colours filling the top **3.0 mm, 15 layers at 0.2 mm**. The lid's top plate was thickened from 2.0 to 5.0 mm for exactly this: a colour change only three layers deep came out washed out. The parts overlap on purpose, since slicers give an overlap to the part listed last, so the clover comes after the triangle. The screen window also moved 3 mm left (`CYD_SCREEN_OFF_X` = -3.0) and is 3 mm deeper, so the glass sits 7.0 mm below the lid surface.

**Not in the repository:** `clover.png`, `logo.png` and the generated data files are trademarked artwork and are in `.gitignore`.

## Troubleshooting

Find the first channel where the trace differs from the golden capture; the fault is in the link feeding that channel.

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| K\_TX: `no start bit found` | Sketch not running, wrong GPIO, or probe or ground not connected | Check that IO27 is on CN1 and the analyzer GND is connected. Check that the Serial log shows `Wakeup: addr 0x10` |
| K\_TX decodes 0x10 but K\_LINE is flat high | TX/RX not crossed over, JP1 not set, or no VS | IO27 must go to the Click's **RX** pin. Check JP1 = 3V3, 12 V on the VS terminal, and the PWR LED lit |
| K\_LINE is stuck low | Short to ground, the K wire on the wrong ECU pin (seen on 22 Sep 2026), a second transceiver on the line, or IO27 low | Unplug the KKL. IO27 must idle high; `setup()` drives it high |
| `RX transitions …: 0` while K\_LINE shows the wakeup | Click TX → IO22 wire wrong or missing | Check the crossover. Measure that the Click's TX pin idles at about 3.3 V |
| K\_LINE and K\_RX are fine, but `FAIL: no 0x55 sync byte, nothing received at all` | ECU not powered or not in ignition mode, wrong K pin, wakeup timing off, or the ECU expects the wakeup on the L-line (OBD 15), which the Click can't drive | Run kwp71\_m2\_10\_4.py --probe on the same pin. If the KKL wakes the ECU and the ESP32 doesn't, check whether the 3-pin adapter carries the L-line and bridge L to K. Compare the P3 and P0 captures edge by edge. Check `check_wakeup.py` bit widths (200 ms ± 5%) and the idle time (≥ 2 s) |
| 0x55 on K\_LINE, but the log says `nothing received at all` | UART not attached to IO22 | Nothing may call `pinMode()` on IO22 or IO27 after `Serial2.begin()` in `wakeupBreakBitbang()`. `K_RX_PIN` must be 22 |
| `FAIL: no 0x55, but saw: …` | Framing errors: a noisy or slow K-line, or 0x55 caught mid-byte | Decode K\_RX in the capture. If it's clean there, check the ground star |
| 0x55 is seen but no 0x79 on K\_LINE, or the ECU goes silent | Reply too late or too early | Make the 86→79 gap match W4 from P0 by tuning `BYTE_TIMEOUT_MS` and `TURNAROUND_MS` |
| `WARN: no clean self-echo of the reply`, or `!! no self-echo` / `!! self-echo xx != sent yy` | The L9637 isn't driving the K-line cleanly, or someone else is on the bus | Check VS and the K screw. Check that the KKL is unplugged (except in P2b). Compare K\_TX with K\_LINE for that byte |
| `expected EOB after N bytes (LEN=…), got xx - scanning on` on every block, or the session drops at info block 2 | This ECU's LEN convention differs from `ECU_LEN_INCLUDES_EOB` | Look at LEN and the position of the 0x03 in the P0 capture. If LEN counts the 0x03, set `ECU_LEN_INCLUDES_EOB` to 1 in the sketch and `True` in the Python tool |
| Blocks start, then `echo … != …` or `Session lost` | An ECU reply block not read (before the fix: the reply to the NOP, logged as `self-echo 0x2 != sent 0x6`), echo turnaround too slow, or a keep-alive gap too long | Every block sent must be followed by a `readBlock()` of the ECU's reply before the next send. Compare the echo gaps with P0. Keep TFT output out of `readBlock()` and `sendBlock()`. Shorten the `delay(300)` in `loop()` if the ECU times out |
| Simulator: `wakeup: … -> MISMATCH` | USB latency blurred the low periods, or the wakeup bits are wrong | Check K\_TX with `check_wakeup.py` first. If that's OK, run the simulator with `--no-check-wakeup` |
| Simulator: `!! self-echo of XX: none` | KKL not on the K-line, or no 12 V on OBD pin 16 | Check the KKL's OBD pins 7, 16 and 4/5 against the P2b table |
| Bytes are corrupted only when the ECU is connected | K-line rising edges too slow | The Click's pull-up is 4.7 kΩ, and a KKL typically uses about 510 Ω. Add a 1 kΩ resistor from K to VS if P3 decodes badly but P2 was clean |
| sigrok hangs or `LIBUSB_ERROR_PIPE` | FX2 clone USB dropout (seen at every sample rate) | Ctrl-C, `pkill sigrok-cli`, then rerun. Use a short cable straight into the Mac. Check that the capture reaches its full length |
| Block 1 stops after its LEN byte, and the trace shows the ECU re-sending 0D | The ECU didn't accept the echo: sent too fast (0 ms), or the repeated LEN was taken as data | Use ECHO\_DELAY\_MS = 20 and the current readBlock(), which echoes a repeated LEN without storing it |
| Nonsense values (0.21 V, 140 °C), or a reply of type 0A | Formula reading the wrong byte (data starts at blk\[2\]), or the ECU rejecting the first request after the ID blocks (NAK) | Use the current decode, which checks the reply type. A single NAK right after the handshake is normal: the next cycle gets data |

### Sources

- [MikroE ISO 9141 Click schematic v101](https://download.mikroe.com/documents/add-on-boards/click/iso_9141_click/iso-9141-click-schematic-v101.pdf)
- [ISO 9141 Click product page](https://www.mikroe.com/iso-9141-click)
- [ESP32 Cheap Yellow Display pin guide](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/PINS.md)
- Local: `kwp71_m2104_esp32.ino`, `m2104_sim.py`, `kwp71_m2_10_4.py`, `output.txt`, `check_wakeup.py`
