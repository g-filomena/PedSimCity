import csv
from collections import Counter

rows = []
with open(r'c:\Users\sgabalog\Documents\PedSim\Working_Version_Connected_to_Gab\PedSimCity\trip_diagnostic.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

print(f'Total trips: {len(rows)}')

if len(rows) > 0:
    dists = [float(r['distance_m']) for r in rows]
    durs = [int(r['duration_min']) for r in rows]
    starts = [r['start_time'] for r in rows]
    
    print(f'Distance range: {min(dists):.1f} - {max(dists):.1f} m')
    print(f'Distance avg: {sum(dists)/len(dists):.1f} m')
    print(f'Duration range: {min(durs)} - {max(durs)} min')
    print(f'Duration avg: {sum(durs)/len(durs):.1f} min')
    
    start_counts = Counter(starts)
    print(f'Unique start times: {len(start_counts)}')
    print('Start time distribution (top 10):')
    for t, c in start_counts.most_common(10):
        print(f'  {t}: {c}')
        
    print('\nSample of first 5 rows (agent_id, start, end, dur, dist):')
    for r in rows[:5]:
        print(f"Agent {r['agent_id']}: {r['start_time']} -> {r['end_time']}, {r['duration_min']}m, {float(r['distance_m']):.1f}m")
