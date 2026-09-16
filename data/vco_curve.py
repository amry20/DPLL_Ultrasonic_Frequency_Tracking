import csv

# Ambil semua data point DAC vs Freq dari semua file untuk plot VCO curve
files = [
    r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\Lock to 31KHz Start at 32.9KHz.csv',
    r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\Lock to 33.4KHz Start at 33.9KHz.csv',
    r'D:\National Seismic Instrument\Release\Firmware\Source\DPLL_Ultrasonic_Frequency_Tracking\data\New Fw Fix 1.csv',
]

points = {}  # dac_rounded -> [freqs]
for fname in files:
    try:
        rows = list(csv.DictReader(open(fname)))
    except:
        continue
    for r in rows:
        freq_str = r.get('ReferenceFrequencyHz', '')
        if not freq_str:
            continue
        try:
            dac  = float(r['DACVoltage_V'])
            freq = float(freq_str)
        except:
            continue
        if freq < 100:
            continue
        key = round(dac, 2)
        if key not in points:
            points[key] = []
        points[key].append(freq)

print('DAC(V)  | Freq mean(Hz) | min      | max')
print('-' * 50)
for dac in sorted(points.keys()):
    freqs = points[dac]
    print(f'{dac:.2f}    | {sum(freqs)/len(freqs):12.1f} | {min(freqs):8.1f} | {max(freqs):8.1f}')
