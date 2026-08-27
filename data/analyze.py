import csv
import sys
from collections import Counter

fname = sys.argv[1] if len(sys.argv) > 1 else r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\New Fw Fix 1.csv'
rows = list(csv.DictReader(open(fname)))
print(f"File : {fname}")
print(f"Rows : {len(rows)}")

# --- State summary ---
states = Counter(r['LockState'] for r in rows)
print("\n=== State counts ===")
for s, n in sorted(states.items()):
    print(f"  {s:12s} {n:6d}  ({100*n/len(rows):.1f}%)")

# --- State transitions ---
print("\n=== State transitions ===")
prev = None
for i, r in enumerate(rows):
    s = r['LockState']
    if s != prev:
        dac = float(r['DACVoltage_V'])
        ph  = float(r['PhaseErrorNs'])
        print(f"  Row {i+1:5d} | {prev or 'START':10s} -> {s:10s} | DAC={dac:7.4f} V | Phase={ph:9.1f} ns")
        prev = s

# --- LOCK stats ---
lock_rows = [r for r in rows if r['LockState']=='LOCK']
if lock_rows:
    dacs = [float(r['DACVoltage_V']) for r in lock_rows]
    print(f"\n=== LOCK stats ({len(lock_rows)} rows) ===")
    print(f"  DAC min={min(dacs):.4f} V  max={max(dacs):.4f} V  mean={sum(dacs)/len(dacs):.4f} V")

# --- First TRACK session row-by-row ---
print("\n=== First TRACK session (row by row) ===")
in_track = False
count = 0
prev_dac = None; prev_ph = None
for i, r in enumerate(rows):
    if r['LockState'] == 'TRACK':
        if not in_track:
            in_track = True
        dac = float(r['DACVoltage_V'])
        ph  = float(r['PhaseErrorNs'])
        ddac  = (dac - prev_dac)  if prev_dac  is not None else 0.0
        dph   = (ph  - prev_ph)   if prev_ph   is not None else 0.0
        wrong = (abs(ddac)>0.001 and abs(dph)>10) and ((ddac>0) != (dph>0))
        flag  = " <WRONG DIR>" if wrong else ""
        print(f"  Row {i+1:5d} | DAC={dac:7.4f} V (d={ddac:+.4f}) | Phase={ph:9.1f} ns (d={dph:+.1f}){flag}")
        prev_dac = dac; prev_ph = ph
        count += 1
        if count >= 60:
            print("  ... (truncated at 60 rows)")
            break
    else:
        if in_track:
            break  # end of first TRACK session

# State counts
c = Counter(r['LockState'] for r in rows)
print("\n--- State counts ---")
for k, v in sorted(c.items()):
    print(f"  {k:10s}: {v:6d} rows ({v/len(rows)*100:.1f}%)")

# Detailed trace of first TRACK session: row-by-row DAC + Phase
print("\n--- First TRACK session detail (until WAIT ZCD) ---")
in_track = False
for i, r in enumerate(rows):
    s = r['LockState']
    if s == 'TRACK' and not in_track:
        in_track = True
    if in_track:
        dac   = float(r['DACVoltage_V'])
        phase = float(r['PhaseErrorNs'])
        freq  = float(r['ReferenceFrequencyHz'])
        stale = r['PhaseStale']
        print(f"Row {i+1:6d} | {s:10s} | DAC={dac:7.4f} V | Phase={phase:9.1f} ns | Freq={freq:.1f} Hz | Stale={stale}")
        if s == 'WAIT ZCD' and in_track:
            break  # stop after first WAIT ZCD after first TRACK

# Compare: first TRACK of file 5 vs file 29
# Show ref freq + DAC range for each TRACK session
print("\n--- Per-session summary (TRACK only) ---")
print(f"{'Sess':>4} {'StartRow':>8} {'DAC_start':>10} {'DAC_end':>8} {'Phase_start':>12} {'Phase_end':>10} {'MinDAC':>7} {'MaxDAC':>7} {'Rows':>5}")
prev2 = None; sess = 0; sess_rows = []
def print_sess(sess, sr, er, srows):
    if not srows: return
    dacs   = [float(r['DACVoltage_V']) for r in srows]
    phases = [float(r['PhaseErrorNs']) for r in srows]
    print(f"{sess:>4} {sr:>8} {dacs[0]:>10.4f} {dacs[-1]:>8.4f} {phases[0]:>12.1f} {phases[-1]:>10.1f} {min(dacs):>7.4f} {max(dacs):>7.4f} {len(srows):>5}")

sess_start_row = 0
for i, r in enumerate(rows):
    s = r['LockState']
    if s == 'TRACK' and prev2 != 'TRACK':
        if sess_rows:
            print_sess(sess, sess_start_row, i, sess_rows)
        sess += 1
        sess_rows = [r]
        sess_start_row = i+1
    elif s == 'TRACK':
        sess_rows.append(r)
    elif s != 'TRACK' and prev2 == 'TRACK':
        print_sess(sess, sess_start_row, i, sess_rows)
        sess_rows = []
    prev2 = s

# State counts
c = Counter(r['LockState'] for r in rows)
print("\n--- State counts ---")
for k, v in sorted(c.items()):
    print(f"  {k:10s}: {v:6d} rows ({v/len(rows)*100:.1f}%)")

# All state transitions
print("\n--- State transitions ---")
prev = None
for i, r in enumerate(rows):
    s = r['LockState']
    dac = float(r['DACVoltage_V'])
    phase = float(r['PhaseErrorNs'])
    stale = r['PhaseStale']
    if s != prev:
        print(f"Row {i+1:6d} | state={s:10s} | DAC={dac:7.4f} V | Phase={phase:9.1f} ns | Stale={stale}")
        prev = s

# DAC distribution during TRACK
track_rows = [r for r in rows if r['LockState'] == 'TRACK']
if track_rows:
    dacs = [float(r['DACVoltage_V']) for r in track_rows]
    print(f"\n--- TRACK DAC stats ---")
    print(f"  min={min(dacs):.4f}  max={max(dacs):.4f}  mean={sum(dacs)/len(dacs):.4f} V")
    print(f"  DAC < 0.1 V: {sum(1 for d in dacs if d<0.1)} rows ({sum(1 for d in dacs if d<0.1)/len(dacs)*100:.1f}%)")
    print(f"  DAC > 3.0 V: {sum(1 for d in dacs if d>3.0)} rows ({sum(1 for d in dacs if d>3.0)/len(dacs)*100:.1f}%)")

# Phase during LOCK
lock_rows = [r for r in rows if r['LockState'] == 'LOCK']
if lock_rows:
    phases = [float(r['PhaseErrorNs']) for r in lock_rows]
    dacs   = [float(r['DACVoltage_V'])  for r in lock_rows]
    print(f"\n--- LOCK stats ({len(lock_rows)} rows) ---")
    print(f"  Phase: min={min(phases):.1f}  max={max(phases):.1f}  mean={sum(phases)/len(phases):.1f} ns")
    print(f"  DAC:   min={min(dacs):.4f}  max={max(dacs):.4f}  mean={sum(dacs)/len(dacs):.4f} V")

# Re-acquire: DAC when TRACK starts after WAIT ZCD
print("\n--- Re-acquire DAC_start ---")
prev2 = None; sess = 0
for i, r in enumerate(rows):
    s = r['LockState']
    if s == 'TRACK' and prev2 == 'WAIT ZCD':
        sess += 1
        dac = float(r['DACVoltage_V'])
        note = " <<ZERO" if dac < 0.1 else (" <<LOW" if dac < 0.5 else "")
        print(f"  Session {sess:3d} Row {i+1:6d} | DAC_start={dac:7.4f} V{note}")
    prev2 = s

# Frozen DAC at start of each WAIT ZCD period
print("\n--- Frozen DAC when WAIT ZCD starts ---")
prev3 = None
for i, r in enumerate(rows):
    s = r['LockState']
    if s == 'WAIT ZCD' and prev3 != 'WAIT ZCD':
        dac = float(r['DACVoltage_V'])
        note = " <<ZERO" if dac < 0.05 else (" <<LOW" if dac < 0.5 else "")
        print(f"  Row {i+1:6d} | frozen_DAC={dac:7.4f} V{note}")
    prev3 = s
