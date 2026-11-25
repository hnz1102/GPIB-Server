#!/usr/bin/env python3
"""Continuous binary trace capture from an ADVANTEST R3267 via the ESP32 GPIB-Server.

Connects with `ssh admin@<host> gpib raw` (Prologix-compatible), sets the
measurement conditions, then repeats:

    TS        take one sweep (the next command waits until it completes)
    TBA?      trace A, binary: 2 bytes x N points (high byte first), EOI at end

and converts each frame to dBm:  level = RL + (raw - 14592) / 1280 * dB/div
(R3267 Operation Manual Vol.1, 5.2.10.3).

The trace delimiter is set to EOI only (DL2) during the capture and restored to
CR LF + EOI (DL0) on exit.

Password: set the SSHPASS environment variable (uses `sshpass -e`), otherwise
ssh prompts for it.

Example:
    SSHPASS=... python3 tools/r3267_capture.py 192.168.0.173 --addr 30 \\
        --cf 119MZ --span 10MZ --rbw 100KZ --frames 0 --csv trace.csv
"""

import argparse
import os
import select
import signal
import subprocess
import sys
import time

TOP = 14592      # raw value at the top graticule line (= reference level)
PER_DIV = 1280   # raw counts per vertical division
DB_PER_DIV = {0: 10.0, 1: 5.0, 2: 2.0, 3: 1.0, 4: 0.5}


class GpibRaw:
    def __init__(self, host, user, timeout_ms):
        cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
               "-o", "LogLevel=ERROR", f"{user}@{host}", "gpib", "raw"]
        if os.environ.get("SSHPASS"):
            cmd = ["sshpass", "-e"] + cmd
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.buf = b""
        self.timeout = timeout_ms / 1000 + 5

    def send(self, s):
        self.p.stdin.write(s.encode())
        self.p.stdin.flush()

    def _fill(self, deadline):
        r, _, _ = select.select([self.p.stdout], [], [], max(0.0, deadline - time.time()))
        if not r:
            raise TimeoutError(f"no data from GPIB-Server (have {len(self.buf)} bytes)")
        d = os.read(self.p.stdout.fileno(), 65536)
        if not d:
            raise EOFError("SSH session closed")
        self.buf += d

    def readline(self):
        deadline = time.time() + self.timeout
        while b"\n" not in self.buf:
            self._fill(deadline)
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace").strip()

    def readexact(self, n):
        deadline = time.time() + self.timeout
        while len(self.buf) < n:
            self._fill(deadline)
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def query(self, cmd):
        """ASCII query (instrument delimiter must end with LF, e.g. DL0)."""
        self.send(f"{cmd}\n++read eoi\n")
        return self.readline()

    def close(self):
        try:
            self.p.stdin.close()
        except OSError:
            pass
        self.p.terminate()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host")
    ap.add_argument("--user", default="admin")
    ap.add_argument("--addr", type=int, default=30, help="R3267 GPIB address")
    ap.add_argument("--cf", default="119MZ", help="center frequency (Advantest units: GZ/MZ/KZ/HZ)")
    ap.add_argument("--span", default="10MZ")
    ap.add_argument("--rbw", default="100KZ")
    ap.add_argument("--frames", type=int, default=0, help="number of frames (0 = until Ctrl-C)")
    ap.add_argument("--csv", help="append each frame as one CSV row: time,dBm[0..N-1]")
    ap.add_argument("--png", help="save a plot of the last frame")
    ap.add_argument("--timeout-ms", type=int, default=5000)
    args = ap.parse_args()

    g = GpibRaw(args.host, args.user, args.timeout_ms)
    stop = False

    def on_sigint(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    try:
        time.sleep(1.0)
        g.send(f"++addr {args.addr}\n++eos 2\n++eoi 1\n++read_tmo_ms {args.timeout_ms}\n++addr\n")
        while g.readline() != str(args.addr):   # skip the SSH banner
            pass

        # Measurement conditions (ASCII replies while DL0 is active)
        g.send("DL0\n")
        g.send(f"CF {args.cf}\nSP {args.span}\nRB {args.rbw}\n")
        info = {q: g.query(q) for q in ["CF?", "SP?", "RB?", "FA?", "FB?", "RL?", "DD?", "TP?"]}
        cf, sp, rbw = float(info["CF?"]), float(info["SP?"]), float(info["RB?"])
        fa, fb, rl = float(info["FA?"]), float(info["FB?"]), float(info["RL?"])
        ddiv = DB_PER_DIV.get(int(info["DD?"]), 10.0)
        npts = 1001 if info["TP?"].strip() == "1" else 501
        print(f"CF={cf/1e6:.3f} MHz SPAN={sp/1e6:.3f} MHz RBW={rbw/1e3:.1f} kHz "
              f"RL={rl:g} dBm {ddiv:g} dB/div points={npts}", flush=True)
        freqs = [fa + (fb - fa) * i / (npts - 1) for i in range(npts)]

        g.send("DL2\n")  # trace delimiter: EOI only → pure 2*npts byte binary block
        csv = open(args.csv, "a") if args.csv else None
        if csv and csv.tell() == 0:
            csv.write("time," + ",".join(f"{f:.0f}" for f in freqs) + "\n")

        n = 0
        t0 = time.time()
        db = []
        while not stop and (args.frames == 0 or n < args.frames):
            g.send("TS\nTBA?\n++read eoi\n")
            raw = g.readexact(2 * npts)
            vals = [(raw[2 * i] << 8) | raw[2 * i + 1] for i in range(npts)]
            db = [rl + (v - TOP) / PER_DIV * ddiv for v in vals]
            n += 1
            pk = max(range(npts), key=lambda i: db[i])
            rate = n / (time.time() - t0)
            print(f"frame {n:5d}  peak {db[pk]:7.2f} dBm @ {freqs[pk]/1e6:9.4f} MHz  "
                  f"({rate:.2f} frames/s)", flush=True)
            if csv:
                csv.write(f"{time.time():.3f}," + ",".join(f"{x:.2f}" for x in db) + "\n")
                csv.flush()
    finally:
        try:
            g.buf = b""
            g.send("DL0\n")       # restore the default delimiter (CR LF EOI)
            time.sleep(0.5)
        finally:
            g.close()

    if args.png and db:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot([f / 1e6 for f in freqs], db, lw=0.8)
        plt.ylim(rl - 10 * ddiv, rl)
        plt.xlim(fa / 1e6, fb / 1e6)
        plt.grid(True, alpha=0.4)
        plt.xlabel("Frequency [MHz]")
        plt.ylabel("Level [dBm]")
        plt.title(f"R3267 Trace A (binary) CF {cf/1e6:g} MHz, SPAN {sp/1e6:g} MHz, RBW {rbw/1e3:g} kHz")
        plt.tight_layout()
        plt.savefig(args.png, dpi=110)
        print(f"saved {args.png}")


if __name__ == "__main__":
    sys.exit(main())
