#!/usr/bin/env python3
"""
m2104_sim.py - pretends to be a Bosch Motronic M2.10.4 on the K-line, so the
ESP32 + ISO 9141 Click tester can be developed at the desk without the ECU.

The MacBook plays the ECU through the KKL / VAG-COM 409.1 cable:

    ESP32 --> ISO 9141 Click --K--+-- KKL cable --USB--> MacBook (this script)
                                  |
    12 V (fused) -----------------+-- Click VS  +  KKL OBD pin 16
    GND --------------------------+-- Click GND +  KKL OBD pins 4 and 5
    K-line: Click K screw         <-> KKL OBD pin 7

    Both sides have a K-line pull-up (Click 4.7k, KKL ~510R); that's fine.
    Never have the real ECU on the same K-line while this runs.

WHAT IT DOES (one session, then waits for the next wakeup)
  1. Waits for the 5-baud 0x10 wakeup. At 4800 baud the KKL can't decode 5
     baud, but every low period shows up as 0x00 byte(s); the timing of those
     is checked against the expected bit pattern of the address.
  2. W1 after the stop bit: sends 0x55, then the keyword bytes, and expects
     the complement of the keyword byte (0x86 -> 0x79) back.
  3. Sends the info blocks, waiting for the tester's inverted echo of every
     byte (except the final 0x03) and for an ACK block (type 0x09) after each.
  4. Sends the final block, then answers every tester block until the tester
     goes quiet for longer than --session-timeout.

WHAT IS REAL AND WHAT IS A PLACEHOLDER
  Real (from kwp71_m2_10_4.py + output.txt against this ECU):
    address 0x10, 4800 baud, 0x55 sync about 180 ms after the stop bit, then
    "32 86 04 15 26" (keyword byte #1 = 0x86 -> reply 0x79), info block 1 =
    part number as reversed ASCII "8744021620", LEN does NOT count the
    trailing 0x03, inverted echo of every byte except 0x03.
  Placeholders (replace with values from the P0 golden capture via --profile):
    the gaps between the bytes after 0x55, all other delays, the TYPE bytes
    of the blocks, info block 2, the final block, and every answer to a data
    request.

USAGE
  python3 m2104_sim.py --port /dev/cu.usbserial-AH01164M
  python3 m2104_sim.py --port ... --profile my_ecu.json   # override defaults
  python3 m2104_sim.py --dump-profile > my_ecu.json       # start a profile
  python3 m2104_sim.py --port ... --w1-ms 60 --no-check-wakeup

  Timing is only as good as macOS + USB allow (the FTDI latency timer adds
  up to ~16 ms per read), so treat this as a development stand-in: a pass
  here means the ESP32 logic is right, the real ECU remains the final test.
"""
import argparse
import json
import sys
import time

EOB = 0x03

# Everything that describes "the ECU". Hex strings so a JSON profile can hold
# the same structure. Override any key with --profile.
DEFAULT_PROFILE = {
    "address": "10",
    "keywords": "32 86 04 15 26",  # bytes after 0x55, as captured from the real ECU
                                   # by kwp71_m2_10_4.py --probe (22 Sep 2026; that
                                   # tool reads max 5, so more may follow)
    "keyword_index": 1,            # which of them the tester must complement (0x86)
    "w1_ms": 180,                  # stop-bit end -> 0x55: measured 187 ms on the Mac,
                                   # which includes up to ~16 ms of USB latency
    "w2_ms": 10,                   # 0x55 -> first keyword byte (placeholder)
    "w3_ms": 5,                    # gap between the other keyword bytes (placeholder)
    "w4_timeout_ms": 1000,         # how long to wait for the complement
    "first_block_delay_ms": 30,    # complement -> first info block
    "byte_gap_ms": 1,              # after the tester's echo, before our next byte
    "turnaround_ms": 2,            # tester byte -> our inverted echo
    "echo_timeout_ms": 500,        # max wait for the tester's echo of our byte
    "len_includes_eob": False,     # confirmed on this ECU: LEN excludes the 0x03
    "info_blocks": [
        {"type": "F6", "ascii": "8744021620"},   # part number, reversed (real)
        {"type": "F6", "ascii": "0000"},         # placeholder: 2nd ID block
    ],
    "final_block": {"type": "09", "data": ""},   # placeholder
    # Answers to tester requests, keyed by the request TYPE byte.
    #   "ram": RAM read "01 <count> <addr hi> <addr lo>" -> <count> bytes
    #   "adc": ADC read "08 <channel>"                    -> fixed bytes
    #   "fixed": always the same answer
    "responses": {
        "09": {"kind": "fixed", "type": "09", "data": ""},          # ACK -> ACK
        "01": {"kind": "ram", "type": "FE",
               "memory": {"0036": "CB", "003B": "32 55"}},          # batt, rpm
        "08": {"kind": "adc", "type": "FB",
               "channels": {"01": "00 CB", "02": "00 60", "03": "00 32"}},
        "07": {"kind": "fixed", "type": "FC", "data": ""},          # no DTCs
    },
    "nak": {"type": "0A", "data": ""},           # unknown request
}

T0 = time.monotonic()


def now_ms():
    return (time.monotonic() - T0) * 1000.0


def log(msg):
    print("t=%9.1fms  %s" % (now_ms(), msg), flush=True)


def hx(s):
    """'01 86' / '0186' / '' -> list of ints"""
    s = s.replace(" ", "")
    return [int(s[i:i + 2], 16) for i in range(0, len(s), 2)]


def hexs(bs):
    return " ".join("%02X" % b for b in bs)


def block_body(spec):
    """{"type": "F6", "ascii": ...} or {"type": .., "data": "hex"} -> [type, data...]"""
    data = [ord(c) for c in spec["ascii"]] if "ascii" in spec else hx(spec.get("data", ""))
    return [int(spec["type"], 16)] + data


# ---------------------------------------------------------------------------
# K-line access through the KKL. Every byte we write comes straight back on
# our own RX (single-wire bus), so write_sync() consumes that self-echo.
# ---------------------------------------------------------------------------
class KLine:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0)

    def read(self, timeout_s):
        self.ser.timeout = max(timeout_s, 0)
        b = self.ser.read(1)
        return b[0] if b else None

    def write_sync(self, b):
        self.ser.write(bytes([b]))
        self.ser.flush()
        echo = self.read(0.1)
        if echo != b:
            log("!! self-echo of %02X: %s (KKL not on the K-line, or a bus collision)"
                % (b, "none" if echo is None else "%02X" % echo))
            return False
        return True

    def drain(self):
        self.ser.reset_input_buffer()


class EcuSim:
    def __init__(self, line, profile, args):
        self.k = line
        self.p = profile
        self.args = args
        self.counter = 0          # counter of the last block on the bus
        self.ms = lambda key: self.p[key] / 1000.0

    # ---- wakeup -----------------------------------------------------------
    def wait_for_wakeup(self):
        """Returns the estimated time (monotonic s) at which the stop bit ended."""
        address = int(self.p["address"], 16)
        bits = [0] + [(address >> i) & 1 for i in range(8)] + [1]
        # expected start times (s, from the start bit) of each low period
        lows = [i * 0.2 for i, b in enumerate(bits) if b == 0 and (i == 0 or bits[i - 1] == 1)]

        self.k.drain()
        log("waiting for 5-baud wakeup 0x%02X ..." % address)
        while True:
            b = self.k.read(1.0)
            if b is None:
                continue
            if b != 0x00:
                log("   ignoring stray byte %02X while idle" % b)
                continue
            t0 = time.monotonic()
            zeros = [0.0]
            while time.monotonic() < t0 + 2.1:
                b = self.k.read(0.05)
                if b is not None:
                    zeros.append(time.monotonic() - t0)
            # first 0x00 of each low period = that period's start
            seen = [zeros[0]] + [z for prev, z in zip(zeros, zeros[1:]) if z - prev > 0.1]
            ok = len(seen) == len(lows) and all(abs(s - e) < 0.08 for s, e in zip(seen, lows))
            log("<- wakeup: low periods start at %s s (expected %s for 0x%02X) -> %s"
                % ([round(s, 2) for s in seen], [round(e, 2) for e in lows], address,
                   "OK" if ok else "MISMATCH"))
            if ok or not self.args.check_wakeup:
                return t0 + len(bits) * 0.2
            log("   not a clean 0x%02X wakeup - ignored (use --no-check-wakeup to accept anyway)" % address)
            self.k.drain()

    # ---- handshake --------------------------------------------------------
    def handshake(self, stop_end):
        time.sleep(max(0, stop_end + self.ms("w1_ms") - time.monotonic()))
        log("-> 55 (sync)")
        self.k.write_sync(0x55)
        kws = hx(self.p["keywords"])
        for i, kw in enumerate(kws):
            time.sleep(self.ms("w2_ms") if i == 0 else self.ms("w3_ms"))
            log("-> %02X (keyword byte #%d)" % (kw, i))
            self.k.write_sync(kw)
        t_kw = time.monotonic()
        want = kws[self.p["keyword_index"]] ^ 0xFF
        r = self.k.read(self.ms("w4_timeout_ms"))
        if r is None:
            log("!! no keyword reply within %d ms" % self.p["w4_timeout_ms"])
            return False
        log("<- %02X keyword reply after %.1f ms -> %s"
            % (r, (time.monotonic() - t_kw) * 1000, "OK" if r == want else "WRONG, expected %02X" % want))
        return r == want

    # ---- blocks -----------------------------------------------------------
    def send_block(self, body, label):
        """body = [type, data...]; builds LEN CTR body 03 and runs the echo lock-step."""
        self.counter = (self.counter + 1) & 0xFF
        payload = [self.counter] + body
        length = len(payload) + (1 if self.p["len_includes_eob"] else 0)
        block = [length] + payload + [EOB]
        log("-> [%s] %s" % (label, hexs(block)))
        for i, b in enumerate(block):
            if i:
                time.sleep(self.ms("byte_gap_ms"))
            self.k.write_sync(b)
            if b == EOB and i == len(block) - 1:
                break                              # 0x03 is never echoed
            e = self.k.read(self.ms("echo_timeout_ms"))
            if e != b ^ 0xFF:
                log("!! echo of byte %d (%02X): %s, expected %02X - session dropped"
                    % (i, b, "none" if e is None else "%02X" % e, b ^ 0xFF))
                return False
        return True

    def recv_block(self, timeout_s):
        """Reads a tester block, echoing every byte inverted except the 0x03.
        Returns [counter, type, data...] or None."""
        b = self.k.read(timeout_s)
        if b is None:
            return None
        length = b
        raw = [b]
        body = []
        time.sleep(self.ms("turnaround_ms"))
        self.k.write_sync(b ^ 0xFF)
        for _ in range(length + 4):
            bb = self.k.read(self.ms("echo_timeout_ms"))
            if bb is None:
                log("!! tester stopped mid-block: %s" % hexs(raw))
                return None
            raw.append(bb)
            # A 0x03 is only the end marker once LEN's worth of bytes is in:
            # counters and data can be 0x03 too (e.g. the "08 03" coolant
            # request). LEN-1 accepts testers whose LEN counts the 0x03.
            if bb == EOB and len(body) >= length - 1:
                break
            body.append(bb)
            time.sleep(self.ms("turnaround_ms"))
            self.k.write_sync(bb ^ 0xFF)
        else:
            log("!! no 0x03 within LEN+4 bytes: %s" % hexs(raw))
            return None
        log("<- %s" % hexs(raw))
        if len(body) < 2:
            log("!! block too short")
            return None
        if body[0] != (self.counter + 1) & 0xFF:
            log("   note: tester counter %02X, expected %02X" % (body[0], (self.counter + 1) & 0xFF))
        self.counter = body[0]
        expect_len = len(body) + (1 if self.p["len_includes_eob"] else 0)
        if length != expect_len:
            log("   note: tester LEN %02X, this ECU profile would use %02X" % (length, expect_len))
        return body

    def answer(self, req):
        typ, data = req[1], req[2:]
        spec = self.p["responses"].get("%02X" % typ)
        if spec is None:
            nak = self.p["nak"]
            return block_body(nak), "NAK for type %02X" % typ
        kind = spec["kind"]
        rtype = int(spec["type"], 16)
        if kind == "ram" and len(data) >= 3:
            count, addr = data[0], (data[1] << 8) | data[2]
            mem = {int(a, 16): hx(v) for a, v in spec["memory"].items()}
            out = []
            for a in range(addr, addr + count):
                # a multi-byte entry fills consecutive addresses
                for base, vals in mem.items():
                    if base <= a < base + len(vals):
                        out.append(vals[a - base])
                        break
                else:
                    out.append(0x00)
            return [rtype] + out, "RAM %04X x%d" % (addr, count)
        if kind == "adc" and data:
            vals = hx(spec["channels"].get("%02X" % data[0], "00 00"))
            return [rtype] + vals, "ADC ch %02X" % data[0]
        return block_body(spec), "type %02X" % typ

    # ---- one full session ---------------------------------------------------
    def run_session(self):
        stop_end = self.wait_for_wakeup()
        self.counter = 0
        if not self.handshake(stop_end):
            return
        time.sleep(self.ms("first_block_delay_ms"))
        for i, spec in enumerate(self.p["info_blocks"]):
            if not self.send_block(block_body(spec), "info %d" % (i + 1)):
                return
            ack = self.recv_block(1.0)
            if ack is None:
                log("!! no ACK after info block %d" % (i + 1))
                return
        if not self.send_block(block_body(self.p["final_block"]), "final"):
            return
        log("== session up - answering requests ==")
        n = 0
        while True:
            req = self.recv_block(self.args.session_timeout)
            if req is None:
                log("== session ended after %d requests (tester quiet or broken block) ==" % n)
                return
            n += 1
            body, what = self.answer(req)
            time.sleep(self.ms("first_block_delay_ms"))
            if not self.send_block(body, what):
                return


def load_profile(path):
    profile = json.loads(json.dumps(DEFAULT_PROFILE))
    if path:
        with open(path) as f:
            profile.update(json.load(f))
    return profile


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="KKL serial port, e.g. /dev/cu.usbserial-AH01164M")
    ap.add_argument("--baud", type=int, default=4800)
    ap.add_argument("--profile", help="JSON file overriding DEFAULT_PROFILE keys")
    ap.add_argument("--dump-profile", action="store_true", help="print the default profile as JSON and exit")
    ap.add_argument("--w1-ms", type=int, help="override the profile's W1 (stop bit -> 0x55)")
    ap.add_argument("--no-check-wakeup", dest="check_wakeup", action="store_false",
                    help="accept any wakeup-like low pulse, not only a clean 0x10 pattern")
    ap.add_argument("--session-timeout", type=float, default=2.0,
                    help="seconds without a tester block before the session is dropped")
    ap.add_argument("--once", action="store_true", help="run one session, then exit")
    args = ap.parse_args()

    if args.dump_profile:
        print(json.dumps(DEFAULT_PROFILE, indent=2))
        return
    if not args.port:
        ap.error("--port is required")

    profile = load_profile(args.profile)
    if args.w1_ms is not None:
        profile["w1_ms"] = args.w1_ms

    try:
        line = KLine(args.port, args.baud)
    except ImportError:
        sys.exit("pyserial is required:  pip install pyserial")

    log("M2.10.4 simulator on %s @ %d baud (Ctrl-C to stop)" % (args.port, args.baud))
    sim = EcuSim(line, profile, args)
    try:
        while True:
            sim.run_session()
            if args.once:
                break
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
