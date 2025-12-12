# -*- coding: utf-8 -*-
"""
Single/Multiple-run capture/plot for Nucleo REPL dump over serial.

Init:
  - Sends ^C, ^C, ^D, waits, then sends:
        kp 1 1
        ki 0 0
        60000 60000
    (each followed by Enter, once)

Capture:
  - Detects "Samples (idx, tL, vL, tR, vR):" (new) or the old 7-field header
  - Parses only idx, tL, tR, vL, vR (positions discarded if present)
  - Finalizes a run when it sees "(end samples)" or when a block ends

Output:
  - 1 figure per run:
      velocity_run{N}.svg  (VL & VR vs index)
  - CSV functionality is INCLUDED but COMMENTED OUT below; uncomment to enable
"""

import re
# import csv  # <-- CSV is optional; left commented out per request
from datetime import datetime
from time import sleep
from matplotlib import pyplot
from serial import Serial

# ------------------- USER SETTINGS -------------------
PORT = "COM8"   #com 4 for wire           # Windows: "COM6", Linux: "/dev/ttyACM0", macOS: "/dev/tty.usbmodem*"
BAUD = 115_200

DO_RESET_SEQUENCE = True   # send ^C ^C ^D once at start
RESET_PAUSE_S = 0.20       # pause between control bytes
POST_RESET_WAIT_S = 1.50   # wait after ^D for MicroPython to reboot

SEND_CMDS = True           # send multiple command lines after reset/wait (once)
CMD_SEQUENCE = [
    b" kp 0.05       0       \r\n",  # range of 0.001
    b" ki 0.01        0        \r\n", # range of 0.001
    b"  60000        0             \r\n",
]
CMD_GAP_S = 0.25           # pause between command lines (lets UI parse & respond)

# New minimal header (your firmware now prints only velocities)
HEADER_TEXT_NEW = "Samples (idx, tL, vL, tR, vR):"
# Backward-compat (old firmware header which included positions)
HEADER_TEXT_OLD = "Samples (idx, tL, pL, vL, tR, pR, vR):"

# End-of-block marker printed by your UI
END_SAMPLES_TEXT = "(end samples)"
# -----------------------------------------------------

# Control chars for MicroPython REPL
CTRL_C = b"\x03"  # KeyboardInterrupt
CTRL_D = b"\x04"  # Soft reboot

# Regexes
LINE_RE_5 = re.compile(
    r"""
    \[\s*(\d+)\s*\]\s+        # index
    (\d+)\s+                  # TL
    ([+-]?\d+(?:\.\d+)?)\s+   # VL
    (\d+)\s+                  # TR
    ([+-]?\d+(?:\.\d+)?)      # VR
    """,
    re.VERBOSE
)
LINE_RE_7 = re.compile(
    r"""
    \[\s*(\d+)\s*\]\s+        # index
    (\d+)\s+                  # TL
    (\d+)\s+                  # PL (ignored)
    ([+-]?\d+(?:\.\d+)?)\s+   # VL
    (\d+)\s+                  # TR
    (\d+)\s+                  # PR (ignored)
    ([+-]?\d+(?:\.\d+)?)      # VR
    """,
    re.VERBOSE
)
END_RE = re.compile(r"\(end\s+samples\)", re.IGNORECASE)
# Parse setpoint echo like: "OK: L_sp=60000 cps  R_sp=60000 cps ..."
SP_RE = re.compile(
    r"L_sp\s*=\s*([+-]?\d+(?:\.\d+)?)\s*\D+R_sp\s*=\s*([+-]?\d+(?:\.\d+)?)",
    re.IGNORECASE
)

print("Opening serial port")
with Serial(PORT, baudrate=BAUD, timeout=1) as ser:
    sleep(0.5)

    # Flush any pre-existing bytes
    print("Flushing serial buffer")
    while ser.in_waiting:
        ser.read()

    # ---- ONE-TIME init: ^C ^C ^D, wait, then send commands ----
    if DO_RESET_SEQUENCE:
        print("Sending Ctrl-C, Ctrl-C, Ctrl-D (soft reboot)")
        ser.write(CTRL_C); sleep(RESET_PAUSE_S)
        ser.write(CTRL_C); sleep(RESET_PAUSE_S)
        ser.write(CTRL_D); sleep(RESET_PAUSE_S)
        print(f"Waiting {POST_RESET_WAIT_S:.2f}s for reboot")
        sleep(POST_RESET_WAIT_S)
        # drain boot text
        while ser.in_waiting:
            ser.read()

    if SEND_CMDS:
        for cmd in CMD_SEQUENCE:
            print(f"Sending: {cmd!r}")
            ser.write(cmd)
            sleep(CMD_GAP_S)

    runs = []                  # list of dicts, one per run
    collecting = False
    cur = None                 # current run buffers
    saw_header = None          # 'new' or 'old' header type (once detected)
    current_sp = (None, None)  # (L_sp, R_sp) if parsed from UI line

    print("Listening for runs...")
    while True:
        raw = ser.readline()
        if not raw:
            continue
        s = raw.decode("utf-8", errors="ignore").strip()
        if not s:
            continue

        # optional: pick up setpoint line from UI for annotation
        m_sp = SP_RE.search(s)
        if m_sp:
            try:
                current_sp = (float(m_sp.group(1)), float(m_sp.group(2)))
            except:
                current_sp = (None, None)

        # start of a new block (support both headers)
        if (HEADER_TEXT_NEW in s) or (HEADER_TEXT_OLD in s):
            header = 'new' if (HEADER_TEXT_NEW in s) else 'old'
            saw_header = header
            # finalize any previous run we were collecting
            if collecting and cur and cur["index"]:
                cur["setpoint"] = current_sp
                runs.append(cur)
            # start a new run buffer
            cur = {"index": [], "TL": [], "VL": [], "TR": [], "VR": [], "setpoint": current_sp}
            collecting = True
            continue

        # end marker from firmware
        if END_RE.search(s):
            if collecting and cur and cur["index"]:
                cur["setpoint"] = current_sp
                runs.append(cur)
            print("Detected end of samples.")
            # continue listening for more runs; comment the next line to keep capturing
            break

        # while in a block, try to parse data lines
        if collecting:
            if saw_header == 'new':
                m = LINE_RE_5.match(s)
                if m:
                    i, tL, vL, tR, vR = m.groups()
                    cur["index"].append(int(i))
                    cur["TL"].append(int(tL))
                    cur["VL"].append(float(vL))
                    cur["TR"].append(int(tR))
                    cur["VR"].append(float(vR))
                    continue
            else:  # 'old' header (7-field)
                m = LINE_RE_7.match(s)
                if m:
                    i, tL, _pL, vL, tR, _pR, vR = m.groups()
                    cur["index"].append(int(i))
                    cur["TL"].append(int(tL))
                    cur["VL"].append(float(vL))
                    cur["TR"].append(int(tR))
                    cur["VR"].append(float(vR))
                    continue

            # non-data line while collecting: finalize this run
            if cur and cur["index"]:
                cur["setpoint"] = current_sp
                runs.append(cur)
            cur = None
            collecting = False
            # continue scanning for the next header

# ---- CSV (COMMENTED OUT by request) ----
if not runs:
    print("No runs captured. Make sure your firmware printed a '(end samples)' block and that the commands were accepted.")
    raise SystemExit(1)

TS = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_name = f"nucleo_runs_{TS}.csv"

"""
# Uncomment this block to save CSV with all runs (velocity-only)
import csv
with open(csv_name, "w", newline="") as f:
    w = csv.writer(f)
    # header
    w.writerow(["run","L_sp(cps)","R_sp(cps)","index","tL(ms)","vL(cps)","tR(ms)","vR(cps)"])
    for k, run in enumerate(runs, start=1):
        Lsp, Rsp = run.get("setpoint", (None, None))
        for j in range(len(run["index"])):  # same length for all series
            w.writerow([
                k, Lsp, Rsp,
                run["index"][j],
                run["TL"][j], run["VL"][j],
                run["TR"][j], run["VR"][j],
            ])
        # blank line between runs
        w.writerow([])
print(f"Saved CSV: {csv_name}")
"""

# ---- Plot: 1 figure per run (velocity only) ----
print(f"Captured {len(runs)} run(s). Plotting velocity figure(s)...")
for k, run in enumerate(runs, start=1):
    idx = run["index"]
    Lsp, Rsp = run.get("setpoint", (None, None))

    pyplot.figure()
    pyplot.plot(idx, run["VL"], label="VL (cps)")
    pyplot.plot(idx, run["VR"], label="VR (cps)")
    title = f"Velocity vs index — run {k}"
    if (Lsp is not None) or (Rsp is not None):
        title += f" (L_sp={Lsp} cps, R_sp={Rsp} cps)"
    pyplot.title(title)
    pyplot.xlabel("index"); pyplot.ylabel("velocity (cps)")
    pyplot.legend(); pyplot.grid(True, linestyle=":")
    fn_v = f"velocity_run{k}.svg"
    pyplot.savefig(fn_v, bbox_inches="tight")

print("Done.")
