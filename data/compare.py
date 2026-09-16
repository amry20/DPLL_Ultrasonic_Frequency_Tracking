import csv
from collections import Counter

files = [
    r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\Lock to 31KHz Start at 32.9KHz.csv',
    r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\Lock to 33.4KHz Start at 33.9KHz.csv',
]

for fname in files:
    rows = list(csv.DictReader(open(fname)))
    label = fname.split('\\')[-1]
    print(f'=== {label} ===')
    print(f'Total rows: {len(rows)}')

    # State counts
    states = Counter(r['LockState'] for r in rows)
    for s, n in sorted(states.items()):
        print(f'  {s:12s} {n:5d} ({100*n/len(rows):.1f}%)')

    # All state transitions
    print('  Transitions:')
    prev = None
    for i, r in enumerate(rows):
        s = r['LockState']
        if s != prev:
            dac  = float(r['DACVoltage_V'])
            ph   = float(r['PhaseErrorNs'])
            freq = r.get('ReferenceFrequencyHz', '?')
            print(f'    Row {i+1:5d} | {str(prev or "START"):10s} -> {s:10s} | DAC={dac:.4f} V | Phase={ph:9.1f} ns | Freq={freq} Hz')
            prev = s

    # LOCK DAC/freq stats
    lock_rows = [r for r in rows if r['LockState'] == 'LOCK']
    if lock_rows:
        dacs  = [float(r['DACVoltage_V']) for r in lock_rows]
        freqs = [float(r['ReferenceFrequencyHz']) for r in lock_rows if r.get('ReferenceFrequencyHz','')]
        print(f'  LOCK DAC : min={min(dacs):.4f} max={max(dacs):.4f} mean={sum(dacs)/len(dacs):.4f} V')
        if freqs:
            print(f'  LOCK Freq: min={min(freqs):.1f} max={max(freqs):.1f} mean={sum(freqs)/len(freqs):.1f} Hz')
    print()
