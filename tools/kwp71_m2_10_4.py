#!/usr/bin/env python3
"""
KWP71 handshake tool for Bosch Motronic ECUs (e.g. M2.10.4) over a KKL / VAG-COM 409.1 cable.

THIS VERSION FIXES A CONFIRMED BUG in read_block(): the previous version
trusted the ECU's LEN byte as the *exact* number of bytes to read, including
the final EOB (0x03) terminator. On this M2.10.4 (Bosch 0261204478), a real
capture showed the ECU sending its own part number as ASCII, reversed
("8744021620" -> reversed "0261204478" - an EXACT match), followed by an EOB
byte that arrived ONE byte AFTER the LEN-based read window closed. In other
words: LEN describes the COUNTER+TYPE+DATA payload size, and the EOB is an
*extra* byte on top of that - not counted in LEN. The old code therefore
always finished reading one byte too early and treated the last data byte
(an ASCII digit) as a bogus "missing EOB" error, even though the read was
otherwise 100% correct.

FIX: read_block() expects the EOB right AFTER LEN bytes. A 0x03 before that
point is data, not the end: block counters and values can be 0x03 (with the
usual 01/02/03 numbering the ECU's 2nd block has counter 0x03), and stopping
at the first 0x03 left that byte un-echoed and killed the session. If the
byte at the expected EOB position isn't 0x03, it keeps reading (and echoing)
until it sees one, bounded by a safety cap (LEN + BLOCK_SLACK) so a truly
desynced bus still times out instead of reading forever.

ALSO ADDED: a full raw trace mode (--raw-trace, on by default) that records
every single byte read from the wire with a timestamp and a tag (RAW / OWN
ECHO DISCARDED / ECU DATA), and prints the complete unfiltered trace whenever
a block fails to parse - so a parsing bug like the one above is immediately
visible in the raw data instead of hidden behind a filtered "does not end
with 0x03" message. This is also the data you should compare directly
against ESP32 Serial output when porting timing back to the microcontroller.

WHAT THIS IS
------------
KWP71 (Keyword Protocol 71, a.k.a. KW-71) is the pre-OBDII diagnostic protocol used by
Bosch Motronic ECUs in 90s Alfa/Fiat/Lancia/Opel/Peugeot/Citroen/VW/Volvo/BMW cars.
It is NOT ISO 9141-2 and NOT KWP2000 -- tools like ELM327 cannot talk to it.  This script
talks to the ECU directly, byte for byte, and shows you exactly what is happening so you
can learn how the handshake works and tune the parameters for your specific ECU.

THE HANDSHAKE (as implemented)
------------------------------
1. WAKEUP  - The K-line must be idle (high) for ~2 seconds.  We then drive the address
             byte (0x10 = Motronic engine ECU) onto the K-line at 5 bits/second (8N1,
             200 ms per bit).  UART chips can't do 5 baud, so we bit-bang it:
               * break()    -> toggles the UART TX *break condition* at 200 ms/bit.
                             Uses ONLY the normal serial driver, so it works on any
                             MacBook M-series and any KKL clone (CH340 / FTDI /
                             PL2303 / CP210x).  This is the default.
               * bitbang    -> drives the FTDI TX line directly via pyftdi (best
                             precision, but requires a genuine FTDI chip + libusb)
               * uart5      -> tries to set the port to 5 baud (real RS232 / MAX232)
               * rts        -> toggles the RTS line (cables wired with L-line on RTS)
             Targeting address 0x10 up to --tries (default 3) times with a
             --retry-settle pause in between.
2. SYNC     - The awakened ECU replies on the K-line at 4800 (or 9600/10400) baud with:
             byte 0x55 (synchronisation pattern), then a number of bytes that include
             a PAIR of keyword bytes (KW1, KW2).
3. KEYWORD  - The tester must reply with the logical complement of KW2, e.g. KW2=0x81
             -> reply 0x7E.  If this reply is wrong the ECU drops the session.
4. INFO BLK - The ECU then sends its hardware ID (reversed), firmware ID (reversed) etc.
             in small "blocks" ending with EOB 0x03.
5. PACKETS  - Every block looks like:  [LEN] [COUNTER] [TYPE/data ...] [0x03]
             LEN = number of COUNTER+TYPE+DATA bytes that follow (CONFIRMED: does NOT
             include the trailing EOB byte itself - see fix note above).
             COUNTER increments by one per message (mod 256).
             KWP71 is half-duplex and lock-step: whoever receives a byte echoes it back
             INVERTED, one byte at a time.  Only the final 0x03 is not echoed.
6. Keep-alive: an ACK/NOP packet (type 0x09) keeps the session alive.

TELL ME MORE WHEN READING
-------------------------
Each phase prints the exact bytes sent (TX ->) and received (RX <-), so you can watch the
handshake form.  Use --probe to just wake the ECU up and see its response without
committing to the handshake, then use the printed values to tune --n-sync / --kw-pos.

REQUIREMENTS
------------
  pip install pyserial
  # optional, strongly recommended for FTDI-based KKL cables:
  pip install pyftdi            # + libusb (on mac: brew install libusb)

NOTE ON TIMING / LATENCY
------------------------
KWP71 is lock-step: the ECU sends one byte and waits for the inverted echo before
sending the next.  A USB-serial chip buffers bytes in 1-16 ms chunks; if your OS/north
bridge keeps the FTDI latency timer high the echo reply can arrive too late and the
session dies.  The pyftdi transport sets the FTDI latency to 2 ms internally.
With the pyserial transport, if init blocks stall, lower the FTDI latency timer in the
OS driver (Linux: /sys/bus/usb-serial/devices/ttyUSBx/latency_timer, 1).

EXAMPLES
--------
  # Target address 0x10 (default) on auto-detected port:
  python3 kwp71_m2_10_4.py

  # Probe the ECU (wake it up and just watch what it sends back):
  python3 kwp71_m2_10_4.py --port /dev/tty.usbserial-* --probe

  # Pin a specific port and try once:
  python3 kwp71_m2_10_4.py --port /dev/cu.usbserial-AH01164M --tries 1

SAFETY
------
Read-only by default.  The interactive console only issues read requests / NOP keep-alives.
'raw' mode is provided for experimentation -- the ECU is not flash-safe via these packets,
but you are responsible for what you send.
"""

import argparse
import time
import sys

# ---------------------------------------------------------------------------
# Constants / defaults
# ---------------------------------------------------------------------------
EOB      = 0x03      # end of block marker
MOTRONIC = 0x10      # ECU address used for the 5-baud wakeup (Motronic engine ECU)
BAUDS    = [4800, 9600, 10400]
MAX_BLOCK_LEN = 64   # sanity cap on the LEN byte of an ECU block
BLOCK_SLACK = 4      # extra bytes allowed past LEN before giving up looking for EOB
                     # (confirmed needed: real EOB arrives 1 byte after LEN's count)
ECU_LEN_INCLUDES_EOB = False  # False = confirmed on this ECU: LEN counts COUNTER+TYPE+DATA,
                              # the 0x03 comes after them. True = standard KW1281-style
                              # LEN that also counts the 0x03.

# known read requests (data carried inside the packet after [LEN][COUNTER]).
# From the public kaihara/kwp71scan reverse engineering of the Alfa 155 Motronic.
REQ = {
    "nop":     bytes([0x09]),                     # ACK / keep-alive
    "batadc":  bytes([0x08, 0x01]),               # ADC1 battery voltage raw
    "airtemp": bytes([0x08, 0x02]),               # ADC2 air temp raw
    "coolant": bytes([0x08, 0x03]),               # ADC3 water temp raw
    "battery": bytes([0x01, 0x01, 0x00, 0x36]),     # scaled battery voltage
    "rpm":     bytes([0x01, 0x02, 0x00, 0x3B]),     # engine speed
    "dtc":     bytes([0x07]),                     # DTC list / count
}


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def comp(b: bytes) -> bytes:
    """Logical complement (KWP71 echo mechanism)."""
    return bytes([b ^ 0xFF for b in b])


def hexs(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def try_ascii_reversed(data: bytes) -> str:
    """Best-effort helper: many KWP71 ECU info blocks carry an ASCII ID
    string in REVERSED byte order. Returns the reversed-and-decoded string
    if it looks plausible (all printable), else ''."""
    if not data:
        return ""
    rev = bytes(reversed(data))
    if all(32 <= c < 127 for c in rev):
        return rev.decode("ascii", errors="replace")
    return ""


# ---------------------------------------------------------------------------
# Raw trace buffer - records EVERY byte read from the wire with a timestamp
# and a tag, independent of protocol-level interpretation. Printed in full
# whenever something fails to parse, so no real ECU data is ever silently
# hidden behind a filtered/derived error message again.
# ---------------------------------------------------------------------------
class RawTrace:
    def __init__(self):
        self.events = []   # list of (t_ms, tag, byte_or_None)
        self.t0 = None

    def reset(self):
        self.events = []
        self.t0 = time.monotonic()

    def add(self, tag, b):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t_ms = (time.monotonic() - self.t0) * 1000.0
        self.events.append((t_ms, tag, b))

    def dump(self, label=""):
        print("  ---- RAW TRACE %s (%d events) ----" % (label, len(self.events)))
        prev_t = None
        for t_ms, tag, b in self.events:
            delta = "" if prev_t is None else ("  (+%.1fms)" % (t_ms - prev_t))
            bstr = "----" if b is None else "0x%02X" % b
            print("    t=%8.1fms  %-24s %s%s" % (t_ms, tag, bstr, delta))
            prev_t = t_ms
        print("  ---- END RAW TRACE ----")


TRACE = RawTrace()


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------
class PyserialTransport:
    """Plain pyserial backend."""

    def __init__(self, port, baud):
        import serial
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(
            port, baud, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0.05, write_timeout=1,
        )
        self.ser.reset_input_buffer()
        self.last_tx = None            # last byte we transmitted, kept for diagnostics only

    def read_byte(self, timeout, tag="RAW"):
        self.ser.timeout = timeout
        chunk = self.ser.read(1)
        b = chunk[0] if chunk else None
        TRACE.add(tag, b)
        return b

    def write_byte(self, b):
        self.ser.write(bytes([b]))
        self.ser.flush()
        self.last_tx = b
        TRACE.add("WRITE", b)

    def write_byte_sync(self, b, echo_timeout=0.6):
        """Write one byte and immediately consume its own physical self-echo
        loopback on the single-wire K-line before returning. Every byte we
        transmit loops back into our own RX exactly once, REGARDLESS of
        whether the KWP71 protocol itself expects the ECU to echo it back
        (e.g. the final EOB byte of a sent block is never echoed BY THE
        ECU, but it still loops back to our own RX electrically). Consuming
        it synchronously - right here, right after the write - avoids the
        drift/mis-accounting bugs of deferred/counted echo tracking."""
        self.write_byte(b)
        echo = self.read_byte(echo_timeout, tag="self-echo")
        return echo

    def drain(self):
        self.ser.reset_input_buffer()

    def check_k_line_idle_state(self):
        """Check hardware lines to inspect idle level information."""
        try:
            cts = self.ser.cts
            dsr = self.ser.dsr
            return f"CTS pin: {'HIGH' if cts else 'LOW'}, DSR pin: {'HIGH' if dsr else 'LOW'}"
        except Exception:
            return "Unable to read hardware line status flags"

    def close(self):
        self.ser.close()


class FtdiTransport:
    """FTDI low-level transport: used for bit-banged 5-baud wakeup AND then
    switched back to normal UART for the byte exchange."""

    def __init__(self, baud, vid_pid=(0x0403, 0x6001)):
        from pyftdi.ftdi import Ftdi
        self.ftdi = Ftdi()
        self.baud = baud
        self.last_tx = None
        self.ftdi.open(vendor=vid_pid[0], product=vid_pid[1])  # FT232R default
        try:
            self.ftdi.set_latency_timer(2)
        except Exception:
            pass
        self.set_baud(baud)

    def set_baud(self, baud):
        self.ftdi.set_baudrate(baud)
        self.ftdi.set_line_property(8, 1, 'N')

    def _set_line(self, level: int, hold: float):
        """Set the FTDI TX line (level 0/1) and hold it for `hold` seconds."""
        target = time.monotonic() + hold
        self.ftdi.write_data(bytes([level]))
        while time.monotonic() < target:
            time.sleep(0.002)

    @staticmethod
    def _set_bitmode(ftdi, enter: bool):
        """Enable/disable async-bitbang on FTDI TXD."""
        mode_cls = getattr(ftdi, "BitMode", None)
        if mode_cls is not None:
            mode = mode_cls.BITBANG if enter else mode_cls.RESET
            ftdi.set_bitmode(0x01, mode)
        else:
            ftdi.set_bitmode(0x01 if enter else 0x00, 0x01 if enter else 0x00)

    def bitbang_wakeup(self, address: int, bit_time: float = 0.2):
        f = self.ftdi
        self._set_bitmode(f, True)
        self._set_line(1, 0.30)
        self._set_line(0, bit_time)                    # start bit (low)
        for i in range(8):                             # data bits, LSB first
            self._set_line((address >> i) & 1, bit_time)
        self._set_line(1, bit_time)                    # stop bit (high)
        # No extra idle margin: the ECU's 0x55 can follow within ~20 ms of
        # the stop bit, and wakeup_and_sync() drains + listens right here.
        self._set_bitmode(f, False)
        self.set_baud(self.baud)

    def read_byte(self, timeout, tag="RAW"):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                data = self.ftdi.read_data(1)
            except Exception:
                data = b""
            if data:
                TRACE.add(tag, data[0])
                return data[0]
            time.sleep(0.002)
        TRACE.add(tag, None)
        return None

    def write_byte(self, b):
        self.ftdi.write_data(bytes([b]))
        self.last_tx = b
        TRACE.add("WRITE", b)

    def write_byte_sync(self, b, echo_timeout=0.6):
        """See PyserialTransport.write_byte_sync - identical contract."""
        self.write_byte(b)
        echo = self.read_byte(echo_timeout, tag="self-echo")
        return echo

    def drain(self):
        try:
            self.ftdi.read_data(4096)
        except Exception:
            pass

    def check_k_line_idle_state(self):
        return "FTDI mode active (Line status mapped via internal transport layer)"

    def close(self):
        try:
            self.ftdi.close()
        except Exception:
            pass


def make_transport(args):
    if args.wakeup == "bitbang":
        vid, pid = args.ftdi_vidpid if hasattr(args, "ftdi_vidpid") else (0x0403, 0x6001)
        return FtdiTransport(args.baud, (vid, pid))
    return PyserialTransport(args.port, args.baud)


# ---------------------------------------------------------------------------
# KWP71 protocol layer
# ---------------------------------------------------------------------------
class KWP71:
    def __init__(self, tr, args):
        self.tr = tr
        self.args = args
        self.counter = 1            # message counter, starts at 1
        self.dump_on = True

    # ---- low level
    def tx(self, b: bytes):
        self.log("TX -> %s" % hexs(b))
        for x in b:
            # Every byte we write on this single-wire K-line loops back into
            # our own RX. Consume that self-echo synchronously, right here,
            # rather than deferring/counting it - this avoids drift bugs.
            self.tr.write_byte_sync(x)

    def log(self, msg):
        if self.dump_on:
            print("  " + msg)

    # ---- block receive.  LEN decides where the block ends: on this ECU LEN
    #      covers COUNTER+TYPE+DATA only and EOB arrives as one extra byte on
    #      top of that count.  A 0x03 before that point is data (a counter or
    #      value of 0x03) and is echoed like any other byte.  If the expected
    #      EOB isn't there, keep scanning for 0x03 up to LEN + BLOCK_SLACK
    #      bytes as a safety cap.  Self-echo
    #      of anything WE transmit is consumed synchronously inside tx(),
    #      so every byte read_byte() returns here is a genuine ECU byte.
    def read_block(self, echo=True, timeout=0.8) -> bytes:
        TRACE.reset()
        b = self.tr.read_byte(timeout, tag="LEN candidate")
        if b is None:
            print("  [DEBUG] !! no block start (expected LEN byte)")
            TRACE.dump("(no LEN byte)")
            return b""
        length = b
        self.log("  [DEBUG] Received Block Length (LEN): 0x%02X (%d bytes) - "
                  "note: EOB is expected AFTER these, not counted in LEN" % (length, length))
        if length == 0 or length > MAX_BLOCK_LEN:
            print("  [DEBUG] !! implausible block length 0x%02X (bus out of sync?)" % length)
            TRACE.dump("(implausible LEN)")
            return b""
        if self.args.turnaround_ms and echo:
            time.sleep(self.args.turnaround_ms / 1000.0)
            self.tx(comp(bytes([b])))

        # number of COUNTER+TYPE+DATA bytes before the EOB
        body_len = length - 1 if ECU_LEN_INCLUDES_EOB else length
        data = bytearray()
        max_reads = length + BLOCK_SLACK
        for i in range(max_reads):
            bb = self.tr.read_byte(timeout, tag="block byte")
            if bb is None:
                print("  [DEBUG] !! timeout mid-block (read %d bytes so far, EOB expected after %d)"
                      % (len(data), body_len))
                TRACE.dump("(timeout mid-block)")
                return b""
            if bb == EOB and len(data) >= body_len:
                # EOB reached - protocol says do NOT echo it, stop here.
                data.append(bb)
                if len(data) - 1 != body_len:
                    self.log("  [DEBUG] NOTE: EOB arrived after %d bytes, expected after %d (LEN=%d)"
                             % (len(data) - 1, body_len, length))
                break
            if bb == EOB:
                self.log("  [DEBUG] 0x03 at byte %d is data (EOB expected after %d bytes) - echoing it"
                         % (len(data), body_len))
            elif len(data) == body_len:
                self.log("  [DEBUG] !! expected EOB after %d bytes (LEN=%d), got %02X - scanning on"
                         % (body_len, length, bb))
            data.append(bb)
            if echo:
                self.tx(comp(bytes([bb])))
        else:
            # Safety cap hit without ever seeing EOB - genuine desync.
            print("  [DEBUG] !! never saw EOB (0x03) within %d bytes after LEN=0x%02X: %s"
                  % (max_reads, length, hexs(bytes(data))))
            TRACE.dump("(no EOB found)")
            return b""

        if data[-1] != EOB:
            # Should not happen given the loop above, but keep as a guard.
            print("  [DEBUG] !! block does not end with 0x03 (EOB): %s" % hexs(bytes(data)))
            TRACE.dump("(missing EOB guard)")
            return b""

        ascii_guess = try_ascii_reversed(bytes(data[2:-1])) if len(data) > 3 else ""
        if ascii_guess:
            self.log("  [DEBUG] Reversed-ASCII guess for payload: '%s'" % ascii_guess)

        if len(data) >= 2:
            self.counter = data[0]
            self.log("  [DEBUG] Updated message counter from ECU: 0x%02X" % self.counter)
        return bytes(data)

    # ---- block send.  We send a packet and the ECU echoes each byte as complement.
    #      Only the final 0x03 (EOB) is not echoed by the ECU (but still
    #      physically self-echoes on the K-line, consumed via write_byte_sync).
    def send_block(self, payload: bytes, label="", expect_echo=True) -> bool:
        block = bytes([len(payload) + 2]) + bytes([self.counter + 1]) + payload + bytes([EOB])
        print("[SEND %s] %s" % (label, hexs(block)))
        ok = True
        for i, b in enumerate(block):
            # write_byte_sync writes the byte and immediately consumes its own
            # physical self-echo loopback - this is ALWAYS present regardless
            # of whether the ECU also sends a protocol-level echo.
            self.tr.write_byte_sync(b)
            if not (expect_echo and i != len(block) - 1):
                # Final byte (EOB): ECU never echoes it per protocol; the
                # physical loopback was already consumed above. Nothing left
                # to do - no more deferred bookkeeping needed.
                continue
            e = self.tr.read_byte(0.6, tag="echo-verify")
            if e is None:
                self.log("  [DEBUG] no echo for byte index %d (value %02X)" % (i, b))
                ok = False
                continue
            wanted = comp(bytes([b]))[0]
            if e != wanted:
                self.log("  [DEBUG] echo mismatch: got %02X, expected complement %02X for byte index %d" % (e, wanted, i))
                ok = False
            else:
                self.log("  [DEBUG] echo %02X validated correctly" % e)
        if ok:
            self.counter += 1
        return ok


# ---------------------------------------------------------------------------
# The five-baud wakeup
# ---------------------------------------------------------------------------
def _hold(seconds):
    end = time.monotonic() + seconds
    while True:
        rem = end - time.monotonic()
        if rem <= 0:
            return
        time.sleep(min(rem, 0.01))


def wakeup_break(ser, address, bit_time=0.2):
    print("  [wakeup-break] Starting break condition toggling at %d ms/bit for address 0x%02X"
          % (bit_time * 1000, address))
    try:
        ser.break_condition = False
    except Exception:
        pass
    time.sleep(0.3)
    ser.break_condition = True                  # start bit (space / low)
    _hold(bit_time)
    for i in range(8):                          # data bits, LSB first
        val = not (address >> i) & 1
        ser.break_condition = val
        _hold(bit_time)
    ser.break_condition = False                 # stop bit (mark / high)
    _hold(bit_time)
    # Return at the END of the stop bit, not 190 ms later: this ECU's 0x55
    # arrives before that, and the drain in wakeup_and_sync() used to throw
    # it away (probe then saw only "32 86 04" and reported no sync).


def wakeup_uart5(port, address):
    import serial
    print("  [wakeup-uart5] Trying native 5-baud UART at address 0x%02X" % address)
    try:
        ser = serial.Serial(port, 5, timeout=0.2)
    except (ValueError, serial.SerialException) as e:
        print("  [DEBUG] !! cannot set port to 5 baud: %s" % e)
        return
    try:
        ser.write(bytes([address]))
        ser.flush()
        time.sleep(0.3)
    finally:
        ser.close()


def wakeup_rts(ser, address):
    print("  [wakeup-rts] Bit-banging address 0x%02X on RTS line" % address)
    t = 0.2
    ser.rts = True
    time.sleep(0.3)
    ser.rts = False
    time.sleep(t)
    for i in range(8):
        ser.rts = bool((address >> i) & 1)
        time.sleep(t)
    ser.rts = True
    time.sleep(t)                               # stop bit; return at its end


# ---------------------------------------------------------------------------
# Handshake
# ---------------------------------------------------------------------------
def wakeup_and_sync(args, tr, address) -> bool:
    print("[DIAG] Checking K-Line idle state before wakeup...")
    idle_info = tr.check_k_line_idle_state()
    print(f"  -> Interface Pin Status: {idle_info}")

    print("[PHASE 1] Waking ECU (address 0x%02X, 5 baud) ..." % address)
    print("          K-line must have been idle (high) for ~2 s first.")

    if args.wakeup == "break":
        if hasattr(tr, "ser"):
            wakeup_break(tr.ser, address)
        else:
            print("  [DEBUG] !! 'break' wakeup requires pyserial transport")
            return False
    elif args.wakeup == "bitbang":
        tr.bitbang_wakeup(address)
    elif args.wakeup == "uart5":
        time.sleep(0.2)
        tr.drain()
    elif args.wakeup == "rts":
        if hasattr(tr, "ser"):
            wakeup_rts(tr.ser, address)
        else:
            print("  [DEBUG] !! 'rts' wakeup requires pyserial transport")
            return False

    # We're at the end of the stop bit here. Everything received so far is
    # our own wakeup looping back (the last low bit ended >= 200 ms ago, so
    # even the FTDI latency timer has delivered it) - drop it and listen.
    tr.drain()
    t_stop = time.monotonic()

    print("[PHASE 2] Waiting for sync byte 0x55 @ %d baud ..." % args.baud)
    sync = None
    deadline = t_stop + args.sync_tries * args.sync_timeout
    while time.monotonic() < deadline:
        b = tr.read_byte(max(0.0, deadline - time.monotonic()), tag="sync-wait")
        if b is None:
            break
        print("  [DEBUG] Read raw byte during sync window: 0x%02X (%.0f ms after stop bit)"
              % (b, (time.monotonic() - t_stop) * 1000))
        if b == 0x55:
            sync = b
            break
    if sync != 0x55:
        print("  [DEBUG] !! never received 0x55 sync byte (check cable wiring, pinout, or ignition power)")
        return False
    print("  <- 0x55 SYNC received successfully")

    print("[PHASE 3] Reading keyword bytes after 0x55 ...")
    kw = bytearray()
    waited = 0
    while len(kw) < args.n_sync:
        b = tr.read_byte(args.byte_timeout, tag="keyword-byte")
        if b is None:
            waited += args.byte_timeout
            print("  [DEBUG] Read timeout waiting for post-sync byte (total idle: %.2fs)" % waited)
            break
        print(f"  [DEBUG] Captured keyword stream byte: 0x{b:02X}")
        kw.append(b)
    if not kw:
        print("  [DEBUG] !! no keyword bytes returned by ECU.")
        return False

    print("  <- Complete byte stream after 0x55: %s" % hexs(bytes(kw)))

    kw_idx = min(args.kw_pos, len(kw) - 1)
    kw2 = kw[kw_idx]
    if kw2 == 0:
        print("  [DEBUG] !! selected keyword byte is 0x00 at position %d. Try adjusting --kw-pos" % args.kw_pos)
        return False
    reply = kw2 ^ 0xFF
    print("  [OK] Target address 0x%02X @ %d baud | ECU keyword index #%d = 0x%02X -> Computed reply complement = 0x%02X"
          % (address, args.baud, kw_idx, kw2, reply))

    print("[DEBUG] Sending logical complement reply back to ECU...")
    time.sleep(args.turnaround_ms / 1000.0)
    tr.write_byte_sync(reply)
    time.sleep(0.1)
    return True


def run_init(args, tr, kwp) -> bool:
    print("[PHASE 4] Reading ECU info blocks (expecting %d info blocks) ..." % args.info_blocks)
    for i in range(args.info_blocks):
        print(f"  [DEBUG] Waiting for info block {i + 1} of {args.info_blocks}...")
        blk = kwp.read_block(echo=True)
        if not blk:
            print("  [DEBUG] !! info block %d failed to parse." % (i + 1))
            if args.raw_trace:
                TRACE.dump("(info block %d failure)" % (i + 1))
            return False
        describe_block(blk)
        print("  [DEBUG] Sending ACK/NOP packet back for info block...")
        kwp.send_block(REQ["nop"], label="ACK", expect_echo=args.echo)

    print("  [DEBUG] Waiting for final ECU session initialization block...")
    blk = kwp.read_block(echo=True)
    if not blk:
        print("  [DEBUG] !! no final session ACK block received")
        if args.raw_trace:
            TRACE.dump("(final ACK block failure)")
        return False
    describe_block(blk)
    return True


def describe_block(b: bytes):
    if len(b) < 3:
        print("  [BLOCK] Raw data: %s" % hexs(b))
        return
    ctr, typ = b[0], b[1]
    text = b[2:-1]
    ascii_ = "".join(chr(c) if 32 <= c < 127 else "." for c in text)
    ascii_rev = try_ascii_reversed(text)
    print("  [BLOCK PARSE] Counter=0x%02X | Type=0x%02X | Payload Hex=%s | ASCII='%s'%s" % (
        ctr, typ, hexs(text), ascii_,
        ("  | REVERSED-ASCII='%s'" % ascii_rev) if ascii_rev else ""))


# ---------------------------------------------------------------------------
# Interactive console
# ---------------------------------------------------------------------------
MENU = """Commands:
  battery    battery voltage (scaled)
  batadc     battery ADC raw
  coolant    water temp                 airtemp    air temp ADC
  rpm        engine speed
  dtc        DTC list
  nop        keep-alive / ACK
  raw <hex>  send raw packet data (e.g. raw 08 05)
  dump       toggle verbose logging
  trace      show the last raw byte trace
  help       this menu
  quit       exit"""


def decode(cmd, blk):
    # blk = [counter, type, data..., 03]: the first data byte is blk[2].
    # Battery calibrated 22 Sep 2026: 0x5E (94) at 12.4 V -> 0.1319 V/step.
    if cmd == "battery" and len(blk) >= 4 and blk[1] == 0xFE:
        print("  battery: %.2f V" % (blk[2] * 0.1319))
    elif cmd == "rpm" and len(blk) >= 5 and blk[1] == 0xFE:
        print("  rpm: %d  (formula not verified with a running engine)" % int(0.2 * blk[2] * blk[3]))
    elif cmd == "coolant" and len(blk) >= 5 and blk[1] == 0xFB:
        x = blk[3]
        c = -0.000014482 * x**3 + 0.006319247 * x**2 - 1.35140625 * x + 144.4095455
        print("  coolant: %.1f C" % c)
    elif cmd == "airtemp" and len(blk) >= 5 and blk[1] == 0xFB:
        x = blk[3]
        c = -2.01389e-05 * x**3 + 0.008784722 * x**2 - 1.676875 * x + 156.74375
        print("  air temp: %.1f C" % c)
    elif cmd == "dtc" and len(blk) > 0:
        print("  DTC data: %s" % hexs(bytes(blk[:-1])))


def repl(kwp):
    print()
    print("== KWP71 session established successfully ==")
    print(MENU)
    while True:
        try:
            line = input("\nk<p>: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        arg = arg.strip()
        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print(MENU)
        elif cmd == "dump":
            kwp.dump_on = not kwp.dump_on
            print("dump logging %s" % ("ON" if kwp.dump_on else "OFF"))
        elif cmd == "trace":
            TRACE.dump("(manual request)")
        elif cmd == "raw":
            try:
                payload = bytes(int(x, 16) for x in arg.split())
            except ValueError:
                print("  raw expects hex bytes, e.g. 08 05")
                continue
            if not payload:
                continue
            kwp.send_block(payload, label="RAW")
            blk = kwp.read_block(echo=True)
            print("  response: %s" % hexs(blk))
            describe_block(blk)
        elif cmd in REQ:
            kwp.send_block(REQ[cmd], label=cmd.upper())
            blk = kwp.read_block(echo=True)
            print("  response: %s" % hexs(blk))
            describe_block(blk)
            decode(cmd, blk)
            # KWP71 strictly alternates: the ECU answers our ACK with its own
            # block. Read it, or the next command collides with it on the bus.
            kwp.send_block(REQ["nop"], label="ACK")
            kwp.read_block(echo=True)
        else:
            print("  unknown command (try 'help')")


def find_port():
    import serial
    from serial.tools import list_ports
    ports = list_ports.comports()
    if not ports:
        return None
    for p in ports:
        print("  discovered serial port: %s  (%s)" % (p.device, p.description))
    return ports[0].device


def main():
    try:
        import serial
    except ImportError:
        print("pyserial is required:  pip install pyserial")
        sys.exit(1)
    ap = argparse.ArgumentParser(description="KWP71 handshake tool for Bosch Motronic M2.10.4 / KKL cable")
    ap.add_argument("--port", "-p", help="serial port, e.g. /dev/cu.usbserial-XXXX")
    ap.add_argument("--baud", type=int, default=4800, choices=BAUDS, help="K-line baud")
    ap.add_argument("--address", type=lambda x: int(x, 0), default=0x10, help="Wakeup address (default 0x10)")
    ap.add_argument("--tries", type=int, default=3, help="Attempts")
    ap.add_argument("--retry-settle", type=float, default=1.0, help="Settle time")
    ap.add_argument("--wakeup", choices=("break", "bitbang", "uart5", "rts"), default="break")
    ap.add_argument("--ftdi-vidpid", type=lambda x: tuple(int(v, 0) for v in x.split(":")), default=(0x0403, 0x6001))
    ap.add_argument("--info-blocks", type=int, default=2)
    ap.add_argument("--n-sync", type=int, default=5)
    ap.add_argument("--kw-pos", type=int, default=1)
    ap.add_argument("--sync-timeout", type=float, default=0.8)
    ap.add_argument("--sync-tries", type=int, default=3)
    ap.add_argument("--byte-timeout", type=float, default=0.15)
    ap.add_argument("--turnaround-ms", type=int, default=8)
    ap.add_argument("--no-echo", action="store_true", default=False)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--raw-trace", action="store_true", default=True,
                     help="dump the full unfiltered byte trace whenever a block fails to parse (default: on)")
    ap.add_argument("--no-raw-trace", dest="raw_trace", action="store_false",
                     help="disable automatic raw trace dump on failure")
    args = ap.parse_args()
    args.echo = not args.no_echo

    if not args.port:
        args.port = find_port()
        if not args.port:
            print("No serial port found. Pass --port.")
            sys.exit(1)

    address = args.address
    args.tries = max(1, args.tries)

    print("== KWP71 Debug Handshake Tool on %s @ %d baud ==" % (args.port, args.baud))
    tr = None
    try:
        ok = False
        for attempt in range(1, args.tries + 1):
            print("\n[TRY LOOP] Target address 0x%02X - attempt %d/%d" % (address, attempt, args.tries))
            tr = None
            try:
                tr = make_transport(args)
                ok = wakeup_and_sync(args, tr, address)
            except Exception as e:
                print("  [DEBUG EXCEPTION] !! %s" % e)
                ok = False
            finally:
                if not ok and tr:
                    tr.close()
                    tr = None
            if ok:
                break
            print("  -- no sync from address 0x%02X; settling bus for %gs --" % (address, args.retry_settle))
            time.sleep(args.retry_settle)

        if not tr:
            print("\nHandshake sequence failed on address 0x%02X." % address)
            return

        kwp = KWP71(tr, args)
        kwp.dump_on = not args.no_echo

        if args.probe:
            print("\n[PROBE MODE] Wakeup and sync bytes captured successfully. Exiting per --probe flag.")
            return

        if not run_init(args, tr, kwp):
            print("\nInitialization failed during ECU info block transfer.")
            return

        repl(kwp)
    finally:
        if tr:
            tr.close()


if __name__ == "__main__":
    main()