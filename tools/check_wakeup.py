#!/usr/bin/env python3
"""
check_wakeup.py - verify the ESP32's 5-baud slow-init wakeup pulse from a
sigrok-cli CSV capture.

WHY THIS EXISTS
----------------
The 5-baud wakeup (200ms/bit) isn't a protocol sigrok's built-in decoders
know about, so this script reconstructs the address byte by hand from the
raw edge transitions: it finds the start bit, then samples the level at the
middle of each subsequent 200ms bit window (LSB first, 8 data bits, no
parity), and reports the reconstructed byte alongside the timing of every
bit so you can see exactly what left the ESP32 (or arrived on the K-line,
depending on which channel you probe).

For the 0x55 sync byte and keyword bytes that come back from the ECU at
4800 baud, don't use this script - use sigrok-cli's own uart decoder
instead (it's a real UART, no reason to hand-roll it):

  sigrok-cli -d fx2lafw --config samplerate=100k --samples 600000 \
    -C D0=K_TX,D1=K_RX \
    -P uart:rx=K_RX:baudrate=4800:parity=none:num_data_bits=8:num_stop_bits=1 \
    -A uart=rx-data

CAPTURE THIS SCRIPT NEEDS
--------------------------
  sigrok-cli -d fx2lafw --config samplerate=100k --samples 600000 \
    -C D0=K_TX,D1=K_RX \
    -o wakeup_capture.csv \
    -O csv:header=true:dedup=true:label=channel:time=true

  --config samplerate=100k 10us resolution is far more than a 200ms/bit
                           signal needs, and keeps the CSV small. Note: on
                           this Mac the fx2lafw stream randomly stalls
                           (LIBUSB_ERROR_PIPE) at any rate - more often the
                           longer the capture - and sigrok-cli then hangs.
  --samples 600000         6s at 100 kHz - covers idle settle + wakeup + sync
                           wait + keyword read with margin (see the ESP32
                           sketch's timeout constants for the exact
                           worst-case window).
  dedup=true               only writes a row when a channel changes state
                           (plus one per 10240-sample USB transfer), so the
                           CSV stays small instead of one row per sample.
  label=channel            header row uses your -C names (K_TX, K_RX) so
                           this script can find the right column.
  time=true                REQUIRED - without it there's no Time column and
                           dedup'd rows carry no timing at all.

  If a capture hangs, Ctrl-C and rerun; check the last Time value in the CSV
  reaches ~6000000 (us) before trusting it.

USAGE
-----
  python3 check_wakeup.py wakeup_capture.csv --channel K_TX --address 0x10

  --channel   which column to analyze (K_TX for what the ESP32 sent, K_RX
              if you're probing the ECU's return path instead - unusual,
              since the wakeup is one-way, but useful for a loopback check)
  --address   expected address byte (default 0x10, per the working Python
              tool's confirmed-good target for this ECU)
  --bit-ms    bit period in ms (default 200.0, i.e. 5 baud)
"""
import argparse
import csv
import math
import sys


def load_rows(path, channel):
    """Return a sorted list of (time_seconds, level) for one channel,
    each entry meaning 'the level became this value at this time and held
    until the next entry'.

    Rows where the level didn't change are dropped: even with dedup=true,
    libsigrok writes a row at every USB transfer boundary (every 10240
    samples), and those would otherwise look like edges to find_start_bit."""
    with open(path, newline="") as f:
        text = f.read().splitlines()

    samplerate = None
    for line in text:
        if line.startswith("; Samplerate:"):
            samplerate = parse_samplerate(line.split(":", 1)[1])

    reader = csv.reader(line for line in text if not line.startswith(";"))
    try:
        header = next(reader)
    except StopIteration:
        sys.exit("empty CSV file")

    if "Time" not in header:
        sys.exit(
            f"no 'Time' column in header {header} - capture with "
            f"-O csv:...:time=true (sigrok-cli leaves it out by default)"
        )
    time_idx = header.index("Time")
    if channel not in header:
            sys.exit(
                f"channel '{channel}' not found in header {header} - "
                f"check the -C name you used when capturing"
            )
    ch_idx = header.index(channel)

    rows = []
    for row in reader:
        if not row:
            continue
        try:
            t = float(row[time_idx])
            v = int(row[ch_idx])
        except (ValueError, IndexError):
            continue
        rows.append((t, v))

    if not rows:
        sys.exit(f"no samples found for channel '{channel}'")
    rows.sort(key=lambda r: r[0])

    # libsigrok 0.5.x writes Time as an integer in a unit it picks from the
    # samplerate (us at 100k/1M). The first row is always sample #1, i.e.
    # exactly one sample period, which tells us the unit.
    if samplerate:
        scale = 10 ** round(math.log10((1.0 / samplerate) / rows[0][0]))
    else:
        scale = 1e-6
    rows = [(t * scale, v) for t, v in rows]

    edges = [rows[0]]
    for t, v in rows[1:]:
        if v != edges[-1][1]:
            edges.append((t, v))
    edges.append((rows[-1][0], rows[-1][1]))  # keep end-of-capture time
    return edges


def parse_samplerate(s):
    """'1 MHz' / '100 kHz' / '20 Hz' -> Hz as float, or None."""
    parts = s.split()
    if not parts:
        return None
    mult = {"Hz": 1, "kHz": 1e3, "MHz": 1e6, "GHz": 1e9}
    try:
        return float(parts[0]) * mult.get(parts[1] if len(parts) > 1 else "Hz", 1)
    except ValueError:
        return None


def level_at(rows, t):
    """Level active at time t, given the held-until-next-row rows list."""
    result = rows[0][1]
    for rt, rv in rows:
        if rt <= t:
            result = rv
        else:
            break
    return result


def find_start_bit(rows, bit_s):
    """First falling edge (1->0) that stays low for roughly one bit period -
    the candidate start bit of the wakeup frame. Returns its timestamp, or
    None if nothing matching was found."""
    tol = bit_s * 0.3
    for i in range(1, len(rows) - 1):
        t, v = rows[i]
        pt, pv = rows[i - 1]
        if pv == 1 and v == 0:
            nt, nv = rows[i + 1]
            low_duration = nt - t
            if abs(low_duration - bit_s) <= tol or low_duration > bit_s:
                return t
    return None


def decode_wakeup(rows, bit_s, address_bits=8):
    t0 = find_start_bit(rows, bit_s)
    if t0 is None:
        print("!! no start bit found - no ~%.0fms low pulse on this channel"
              % (bit_s * 1000))
        return None, []

    samples = []
    for i in range(address_bits):
        sample_t = t0 + (i + 1.5) * bit_s
        samples.append(level_at(rows, sample_t))

    stop_t = t0 + (address_bits + 1.5) * bit_s
    stop_level = level_at(rows, stop_t)

    value = 0
    for i, bit in enumerate(samples):  # LSB first
        value |= (bit << i)

    print("Start bit at t=%.4fs" % t0)
    print("Sampled data bits (LSB first, at bit-center):", samples)
    print("Stop bit sample: %d (expected 1/HIGH)" % stop_level)
    print("Reconstructed address byte: 0x%02X" % value)
    return value, samples


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_file")
    ap.add_argument("--channel", default="K_TX")
    ap.add_argument("--address", default="0x10",
                     help="expected address byte, e.g. 0x10")
    ap.add_argument("--bit-ms", type=float, default=200.0)
    args = ap.parse_args()

    expected = int(args.address, 0)
    bit_s = args.bit_ms / 1000.0

    rows = load_rows(args.csv_file, args.channel)
    print(f"Loaded {len(rows)} edge rows for channel '{args.channel}'")
    print(f"Capture spans t={rows[0][0]:.4f}s to t={rows[-1][0]:.4f}s")
    print()

    value, samples = decode_wakeup(rows, bit_s)
    print()

    if value is None:
        print("RESULT: FAIL - could not locate a wakeup pulse on this channel.")
        print("  If this is K_TX (ESP32 GPIO27), the ESP32 isn't toggling the")
        print("  pin at all, or the wire/ground isn't actually connected.")
        print("  If this is the raw K-line via a divider, GPIO27 is toggling")
        print("  fine but nothing is reaching the actual bus - check VS power,")
        print("  the click's TX->K path, and the divider wiring.")
        sys.exit(1)

    if value == expected:
        print(f"RESULT: OK - decoded 0x{value:02X}, matches expected 0x{expected:02X}")
    else:
        print(f"RESULT: MISMATCH - decoded 0x{value:02X}, expected 0x{expected:02X}")
        print("  Bit timing or bit order may be off - check the sample list")
        print("  above against the known address bits (0x10 = 0,0,0,0,1,0,0,0")
        print("  LSB first) and compare the actual t0/stop-bit timing to the")
        print("  expected ~200ms/bit, ~1.8s total frame.")


if __name__ == "__main__":
    main()
