#!/usr/bin/env python3
"""
kwp71_discover.py - ask a Bosch M2.10.4 for everything it will answer, and log
it, so the values we don't know yet (air temperature, air quantity, ...) can be
found by comparing runs.

It reuses the handshake from kwp71_m2_10_4.py and talks over the KKL cable.

    pip install pyserial
    python3 tools/kwp71_discover.py --port /dev/cu.usbserial-AH01164M \\
        --note "engine off, 14 C" --passes 2

Runs land in captures/ by default, alongside the ones that produced the ADC map
in docs/kwp71-m2104-bringup.md.

WHAT IT ASKS
  --adc            ADC channels 08 00 .. 08 0F  (sensor inputs). Known so far:
                   00 throttle, 01 battery, 03 coolant, 04 air quantity,
                   05 intake air temperature, 07 lambda, 06 load-dependent but
                   unidentified; 02/08/0B idle at 254-255, 09/0A at 0, and
                   0C-0F do not answer.
  --ram A-B        RAM reads 01 <n> <hi> <lo> over an address range
  --known          the requests we already use: battery, rpm, coolant, DTCs
  (with no flags it does --adc --known)

OUTPUT
  * a readable log on screen and in captures/discover.log
  * captures/discover.csv with one row per answer:
        time_s, pass, note, kind, key, type, data_hex, bytes...
    Rows from different runs can be compared directly: whatever changes when
    the intake sensor warms up is air temperature, whatever changes with
    engine speed or load is air quantity.

COMPARING TWO RUNS
  python3 tools/kwp71_discover.py --compare captures/discover.csv \\
      captures/discover_3000.csv

HOW TO USE IT (two runs, one condition each)
  1. python3 tools/kwp71_discover.py --port ... --note "cold, engine off"
  2. warm the intake sensor / run the engine
  3. python3 tools/kwp71_discover.py --port ... --note "warm, idling" \\
         --csv captures/discover2.csv
  4. send both CSVs; the differences name the channels and addresses.

A pass stops nothing: if the session drops, it re-wakes the ECU and carries on
where it left off, so a long RAM sweep survives the odd glitch.
"""
import argparse
import csv
import sys
import time
import types

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import kwp71_m2_10_4 as kwp71                      # noqa: E402  (handshake + transport)

KNOWN = [
    ("battery", bytes([0x01, 0x01, 0x00, 0x36])),
    ("rpm", bytes([0x01, 0x02, 0x00, 0x3B])),
    ("coolant_adc3", bytes([0x08, 0x03])),
    ("airtemp_adc5", bytes([0x08, 0x05])),         # confirmed: 197 raw at 14 C ambient
    ("airqty_adc4", bytes([0x08, 0x04])),          # 6 engine off, ~41 idle, ~70 at 3000
    ("dtc", bytes([0x07])),
]


def defaults(port, baud):
    """The argument set the handshake code expects."""
    return types.SimpleNamespace(
        port=port, baud=baud, address=0x10, tries=3, retry_settle=1.0,
        wakeup="break", ftdi_vidpid=(0x0403, 0x6001), info_blocks=2,
        n_sync=5, kw_pos=1, sync_timeout=0.8, sync_tries=3, byte_timeout=0.15,
        turnaround_ms=8, echo_delay_ms=10, no_echo=False, echo=True,
        probe=False, raw_trace=False)


class Session:
    """Keeps the ECU session alive; re-handshakes when it drops."""

    def __init__(self, args):
        self.args = args
        self.tr = None
        self.kwp = None

    def connect(self):
        if self.tr:
            try:
                self.tr.close()
            except Exception:
                pass
        try:
            self.tr = kwp71.make_transport(self.args)
        except Exception as e:
            # termios EINVAL here means macOS cannot configure the port: the
            # FTDI driver is wedged (often after sleep), not a protocol problem.
            raise SystemExit(
                "cannot open %s: %s\n"
                "  - unplug the cable from USB, wait 5 s, plug it back in\n"
                "  - check with: stty -f %s 4800   (silence = healthy)\n"
                "  - the port name can change after replugging: ls /dev/cu.usbserial*"
                % (self.args.port, e, self.args.port))
        for attempt in range(1, self.args.tries + 1):
            print("[session] handshake attempt %d ..." % attempt)
            if kwp71.wakeup_and_sync(self.args, self.tr, self.args.address):
                self.kwp = kwp71.KWP71(self.tr, self.args)
                self.kwp.dump_on = False
                if kwp71.run_init(self.args, self.tr, self.kwp):
                    print("[session] up")
                    return True
            time.sleep(self.args.retry_settle)
        return False

    def ask(self, payload, label):
        """One request/reply. Returns the reply block, or None."""
        for attempt in (1, 2):
            if self.kwp is None and not self.connect():
                return None
            try:
                self.kwp.send_block(payload, label=label)
                blk = self.kwp.read_block(echo=True)
            except Exception as e:                  # transport trouble
                print("  !! %s: %s" % (label, e))
                blk = b""
            if blk:
                return blk
            print("  !! no reply to %s - re-connecting" % label)
            self.kwp = None
        return None


def describe(blk):
    """(type, data bytes) from a reply block [counter, type, data..., 03]."""
    if not blk or len(blk) < 3:
        return None, b""
    return blk[1], bytes(blk[2:-1])


def row(writer, t0, npass, note, kind, key, blk):
    typ, data = describe(blk)
    name = {0x0A: "NAK", 0x09: "ACK"}.get(typ, "")
    writer.writerow([round(time.time() - t0, 2), npass, note, kind, key,
                     "" if typ is None else "%02X" % typ, name,
                     kwp71.hexs(data)] + list(data))
    return typ, data


def compare(path_a, path_b):
    """Show which answers differ between two runs: those are the values that
    follow whatever changed (temperature, engine speed, load)."""
    def load(path):
        out = {}
        with open(path) as f:
            for r in csv.DictReader(f):
                out[(r["kind"], r["key"])] = (r["data_hex"], r["note"])
        return out
    a, b = load(path_a), load(path_b)
    note_a = next(iter(a.values()))[1] if a else ""
    note_b = next(iter(b.values()))[1] if b else ""
    print("A = %s (%s)\nB = %s (%s)\n" % (path_a, note_a or "-", path_b, note_b or "-"))
    print("%-6s %-6s %-18s %-18s %s" % ("kind", "key", "A", "B", "change"))
    changed = 0
    for key in sorted(set(a) | set(b)):
        va = a.get(key, ("", ""))[0]
        vb = b.get(key, ("", ""))[0]
        if va == vb:
            continue
        changed += 1
        delta = ""
        try:                                        # numeric delta for 1-2 byte answers
            ia = int(va.replace(" ", ""), 16)
            ib = int(vb.replace(" ", ""), 16)
            delta = "%+d" % (ib - ia)
        except ValueError:
            pass
        print("%-6s %-6s %-18s %-18s %s" % (key[0], key[1], va or "-", vb or "-", delta))
    print("\n%d of %d answers changed" % (changed, len(set(a) | set(b))))


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--compare":
        return compare(sys.argv[2], sys.argv[3])
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", "-p", required=True)
    ap.add_argument("--baud", type=int, default=4800)
    ap.add_argument("--note", default="", help="what the car was doing, e.g. 'warm, idling'")
    ap.add_argument("--passes", type=int, default=1, help="repeat the whole sweep N times")
    ap.add_argument("--csv", default="captures/discover.csv")
    ap.add_argument("--log", default="captures/discover.log")
    ap.add_argument("--adc", action="store_true", help="sweep ADC channels 00-0F")
    ap.add_argument("--adc-max", type=lambda x: int(x, 0), default=0x0F)
    ap.add_argument("--ram", help="sweep RAM, e.g. 0x00-0xFF")
    ap.add_argument("--ram-chunk", type=int, default=4, help="bytes per RAM request")
    ap.add_argument("--known", action="store_true", help="the requests we already use")
    args = ap.parse_args()
    if not (args.adc or args.ram or args.known):
        args.adc = args.known = True

    log = open(args.log, "a")

    def say(msg):
        print(msg)
        log.write(msg + "\n")
        log.flush()

    session = Session(defaults(args.port, args.baud))
    if not session.connect():
        sys.exit("could not reach the ECU - is the ignition on and the KKL the only tester?")

    t0 = time.time()
    new_file = True
    try:
        new_file = open(args.csv).read(1) == ""
    except OSError:
        pass
    f = open(args.csv, "a", newline="")
    w = csv.writer(f)
    if new_file:
        w.writerow(["time_s", "pass", "note", "kind", "key", "type", "type_name", "data_hex"])

    say("\n=== discovery run: %s | note: %s ==="
        % (time.strftime("%Y-%m-%d %H:%M:%S"), args.note or "-"))

    for npass in range(1, args.passes + 1):
        say("--- pass %d of %d ---" % (npass, args.passes))

        if args.known:
            for name, payload in KNOWN:
                blk = session.ask(payload, name)
                typ, data = row(w, t0, npass, args.note, "known", name, blk or b"")
                say("  %-14s -> type %s data %s"
                    % (name, "--" if typ is None else "%02X" % typ, kwp71.hexs(data)))

        if args.adc:
            for ch in range(args.adc_max + 1):
                blk = session.ask(bytes([0x08, ch]), "adc%02X" % ch)
                typ, data = row(w, t0, npass, args.note, "adc", "%02X" % ch, blk or b"")
                say("  ADC %02X -> type %s data %s"
                    % (ch, "--" if typ is None else "%02X" % typ, kwp71.hexs(data)))

        if args.ram:
            lo, hi = (int(x, 0) for x in args.ram.split("-"))
            addr = lo
            while addr <= hi:
                n = min(args.ram_chunk, hi - addr + 1)
                payload = bytes([0x01, n, (addr >> 8) & 0xFF, addr & 0xFF])
                blk = session.ask(payload, "ram%04X" % addr)
                typ, data = row(w, t0, npass, args.note, "ram", "%04X" % addr, blk or b"")
                say("  RAM %04X+%d -> type %s data %s"
                    % (addr, n, "--" if typ is None else "%02X" % typ, kwp71.hexs(data)))
                addr += n
        f.flush()

    say("=== done: %s and %s written ===" % (args.csv, args.log))
    f.close()
    log.close()


if __name__ == "__main__":
    main()
