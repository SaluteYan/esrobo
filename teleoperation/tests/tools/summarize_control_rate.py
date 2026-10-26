"""Offline summaries of stream logs and test-only timing traces."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np


def distribution(values):
    if not values: return None
    return dict(count=len(values), median=float(np.median(values)),
                p95=float(np.percentile(values,95)), maximum=float(max(values)),
                over_20=sum(v>20 for v in values), over_100=sum(v>100 for v in values))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',required=True)
    parser.add_argument('--session',required=True)
    parser.add_argument('--transport',required=True)
    args=parser.parse_args()
    root=Path(args.directory)
    session=[json.loads(line) for line in Path(args.session).read_text().splitlines()]
    active=[r for r in session if r.get('mode')=='ACTIVE' and 'rates' in r]
    first,last=active[0],active[-1]
    transport=[r for line in Path(args.transport).read_text().splitlines()
               if first['monotonic_s']<=(r:=json.loads(line))['generated_monotonic_ns']/1e9<=last['monotonic_s']]
    report=dict(test_only=True, session=args.session, profiles={}, fault_window=dict(
        start=first['monotonic_s'],end=last['monotonic_s'],
        active_records=len(active), terminal=[r['status'] for r in session if r.get('record_type')=='terminal_fault'],
        input_age_ms=distribution([1000*v for r in active for v in r['sources'].values()]),
        robot_send_interval_ms=distribution([1000*r['state_send_gap_s'] for r in active]),
        sdk_body_hz_range=[min(r['sdk']['body']['hz'] for r in transport),max(r['sdk']['body']['hz'] for r in transport)],
        body_forward_hz_range=[min(r['timing']['body_forward']['hz'] for r in transport),max(r['timing']['body_forward']['hz'] for r in transport)],
        receiver_counts_begin=first['input_receive_counts'],receiver_counts_end=last['input_receive_counts']))
    for path in sorted(root.glob('*.jsonl')):
        if path.name.endswith('.session.jsonl'):continue
        wall,cpu,gaps=defaultdict(list),defaultdict(list),defaultdict(list)
        work=False;worked=[]
        for line in path.read_text().splitlines():
            r=json.loads(line)
            if 'stage' not in r:continue
            key=r['stage']
            if key=='wrist_case':continue
            wall[key].append(r['duration_ms'])
            if 'thread_cpu_ms' in r:cpu[key].append(r['thread_cpu_ms'])
            if 'gap_ms' in r:gaps[key].append(r['gap_ms'])
            if key=='all_ik':work=True
            if key=='controller':
                if work:worked.append(r['duration_ms'])
                work=False
        report['profiles'][path.stem]=dict(
            durations_ms={k:distribution(v) for k,v in wall.items()},
            thread_cpu_ms={k:distribution(v) for k,v in cpu.items()},
            intercall_gaps_ms={k:distribution(v) for k,v in gaps.items()},
            controller_with_ik_ms=distribution(worked))
    (root/'analysis.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')


if __name__=='__main__':main()
