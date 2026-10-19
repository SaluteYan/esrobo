"""Offline contact retargeting comparison; never opens SDK, CAN or robot sockets.

Use --baseline with a preserved pre-change hand_retarget.py. Reads every raw
sample before the selected ACTIVE interval, retaining solver history. Reports
model fit separately from cached driver-command/feedback differences.
"""
import argparse
import importlib.util
import hashlib
import subprocess
import json
from pathlib import Path
import time

import numpy as np

from esrobo_laptop.config import ROOT, read_settings, read_robot_config
from esrobo_laptop.hand_retarget import HandRetargeter, TIPS, POLICY


def stats(values):
    return dict(zip(('median', 'p95', 'p99', 'max'),
                    np.percentile(values, (50, 95, 99, 100)).tolist())) if len(values) else None


def baseline_module(path):
    spec = importlib.util.spec_from_file_location('esrobo_laptop._hand_baseline', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def direction_error(solver, points, target):
    a = solver._directions(np.asarray(points)[2:6])
    b = solver._directions(np.asarray(target)[2:6])
    return np.degrees(np.arccos(np.clip(np.sum(a*b, axis=1), -1., 1.)))


def reference(solver, result, previous):
    """Independent, high-budget SLSQP with three starts and analytic gradients.

    This is a best-found constrained reference, not a global optimality proof.
    Infeasible cases are reported separately; they are never discarded silently.
    """
    from scipy.optimize import minimize
    m = solver.model; d = result['diagnostics']; active = d['coordination_fingers']
    base = np.asarray(d['geometric_seed_rad']); target = np.asarray(result['target_points'])
    pair = target[TIPS[1:]]-target[5]; length = np.linalg.norm(pair, axis=1)
    latched = np.asarray(d['contact_active'],bool); contacts = np.flatnonzero(latched)
    gates = np.clip((POLICY['pinch_start_width']-length/m.width)/POLICY['pinch_ramp_width'],0.,1.)
    gates = np.maximum(gates*gates*(3-2*gates),d['projection_fraction'])
    soft = np.flatnonzero((~latched)&(gates>1e-8))
    pair *= (np.asarray(d['projected_gap_mm'])/1000/np.maximum(length, 1e-12))[:, None]
    indices = [0, 1, 2]
    for f in active: indices += ([5] if f == 1 else [[3, 4], [], [6, 7], [8, 9]][f])
    indices = np.array(indices); n = len(indices)
    lo, hi = m.lower[indices].copy(), m.upper[indices].copy()
    for col, k in enumerate(indices[3:], 3):
        radius = np.deg2rad(POLICY['flex_correction_deg'] if k in (4, 5, 7, 9) else POLICY['side_correction_deg'])
        finger = next(f for f in active if k in ([5] if f == 1 else [[3,4],[],[6,7],[8,9]][f]))
        if not latched[finger]: radius *= d['projection_fraction'][finger]
        lo[col] = max(lo[col], base[k]-radius); hi[col] = min(hi[col], base[k]+radius)
    desired = solver._directions(target[2:6]); weights = np.sqrt(np.asarray(d.get('direction_weights', POLICY['direction_weights']))/3)[:, None]
    plane, _ = solver._plane(desired, np.zeros((3, 3, n)))
    plane_scale = np.sqrt(POLICY['plane_weight'])*np.linalg.norm(plane)
    regularization = np.full(n, POLICY['temporal_weight']); regularization[3:] = POLICY['finger_regularization']
    prior = np.asarray(previous)[indices].copy(); prior[3:] = base[indices[3:]]
    last_x = cache = None
    def evaluate(x):
        nonlocal last_x, cache
        if last_x is not None and np.array_equal(x, last_x): return cache
        q = base.copy(); q[indices] = x
        points, jac = m.forward(q); jac = jac[:, :, indices]
        directions, jd = solver._directions(points[2:6], jac[2:6]); pn, jp = solver._plane(directions, jd)
        r = np.r_[((directions-desired)*weights).ravel(), (pn-plane)*plane_scale]
        j = np.vstack(((jd*weights[:, :, None]).reshape(-1, n), jp*plane_scale))
        huber = POLICY['huber_delta']; gradient = 2*np.clip(r, -huber, huber)@j
        value = np.sum(np.where(abs(r)<=huber, r*r, 2*huber*abs(r)-huber*huber))
        value += np.sum(regularization*(x-prior)**2); gradient += 2*regularization*(x-prior)
        all_delta = points[TIPS[1:]]-points[5]-pair
        all_jac = jac[TIPS[1:]]-jac[5]
        if len(soft):
            w = POLICY['opening_vector_weight']*gates[soft]**2/m.width**2
            value += np.sum(w[:,None]*all_delta[soft]**2)
            gradient += 2*np.einsum('f,fi,fij->j',w,all_delta[soft],all_jac[soft])
        delta = all_delta[contacts]; dj = all_jac[contacts]
        constraint = (POLICY['contact_tolerance_m']**2-np.sum(delta*delta, axis=1))/m.width**2
        cj = -2*np.einsum('fi,fij->fj', delta, dj)/m.width**2
        cache = float(value), gradient, constraint, cj, points
        last_x = x.copy(); return cache
    constraints = [] if not len(contacts) else [dict(type='ineq', fun=lambda x:evaluate(x)[2], jac=lambda x:evaluate(x)[3])]
    starts = [np.asarray(result['joints'])[indices], np.clip(np.r_[m.neutral[:3], base[indices[3:]]], lo, hi), np.clip(np.asarray(previous)[indices], lo, hi)]
    candidates = []
    for start in starts:
        solved = minimize(lambda x:evaluate(x)[0], start, jac=lambda x:evaluate(x)[1],
                          bounds=list(zip(lo, hi)), constraints=constraints, method='SLSQP',
                          options=dict(maxiter=80, ftol=1e-9))
        for x in (start, solved.x):
            v = evaluate(x); violation = max(float(np.max(-v[2], initial=0)), 0.)
            candidates.append(((violation>1e-7, violation if violation>1e-7 else 0., v[0]), x.copy(), v[4].copy()))
    key, x, points = min(candidates, key=lambda c:c[0])
    return dict(feasible=not key[0], direction_deg=direction_error(solver, points, target).tolist(),
                objective=key[2], constraint_violation=key[1])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--raw', required=True); ap.add_argument('--session', required=True)
    ap.add_argument('--baseline', required=True); ap.add_argument('--output', required=True)
    ap.add_argument('--reference-stride', type=int, default=1)
    ap.add_argument('--execution', help='Original execution log; never interpreted as execution of the new algorithm')
    args = ap.parse_args(); out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    sessions = [json.loads(l) for l in Path(args.session).open()]
    active = [r for r in sessions if r.get('mode') == 'ACTIVE']
    start, end = active[0]['monotonic_s'], active[-1]['monotonic_s']
    cfg = read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    new = HandRetargeter(cfg); old = baseline_module(args.baseline).HandRetargeter(cfg)
    prefix = frames = 0; errors = []; records = []; refs = []
    with Path(args.raw).open() as source, (out/'frames.jsonl').open('w') as destination:
        for line in source:
            row = json.loads(line)
            if row.get('side') != 'left' or 'sample' not in row: continue
            sample = row['sample']; stamp = sample['received_monotonic_ns']/1e9
            if stamp > end: break
            # Preserve every observed input, including originally rejected solves.
            poses = np.asarray(sample['poses']); previous = new.previous.copy()
            try:
                nr = new.solve(poses, source_time=stamp, budget_s=1.)
                old_result = old.solve(poses, budget_s=1.)
            except (ValueError, TimeoutError) as exc:
                errors.append(dict(sequence=row['source_sequence'], error=str(exc), active=stamp>=start)); continue
            if stamp < start: prefix += 1; continue
            original = row.get('result')
            if original is None: continue  # Included in history, no paired baseline output.
            nd = nr['diagnostics']; baseline = original['diagnostics']
            oa = direction_error(new, original['solved_points'], original['target_points'])
            na = np.asarray(nd['thumb_direction_error_deg'])
            contact = np.flatnonzero(nd['contact_active']).tolist(); coordinated = nd['coordination_fingers']; near = int(np.argmin(nd['human_gap_mm']))
            item = dict(sequence=row['source_sequence'], source_time=stamp,
                        old_blend=baseline['pinch_blend'], nearest_finger=near,
                        old_direction=oa.tolist(), new_direction=na.tolist(),
                        old_solve_ms=old_result['diagnostics']['solve_ms'], new_solve_ms=nd['solve_ms'],
                        actual_recorded_old_solve_ms=baseline['solve_ms'],
                        active_contact=contact, coordinated_fingers=coordinated, max_contact_error_mm=max([nd['projected_error_mm'][f] for f in contact], default=0),
                        original_result=original, result=nr)
            if args.reference_stride > 0 and frames % args.reference_stride == 0:
                ref = reference(new, nr, previous); item['reference'] = ref; refs.append(item)
            destination.write(json.dumps(item)+'\n'); records.append(item); frames += 1
            if frames % 500 == 0: print('replayed', frames, 'prefix', prefix, flush=True)
    def summary(rows):
        if not rows: return dict(n=0)
        return dict(n=len(rows), old_proximal_deg=stats([r['old_direction'][0] for r in rows]),
                    new_proximal_deg=stats([r['new_direction'][0] for r in rows]),
                    old_mean_direction_deg=stats([np.mean(r['old_direction']) for r in rows]),
                    new_mean_direction_deg=stats([np.mean(r['new_direction']) for r in rows]),
                    old_solve_ms=stats([r['old_solve_ms'] for r in rows]),
                    new_solve_ms=stats([r['new_solve_ms'] for r in rows]),
                    projected_error_mm=stats([r['max_contact_error_mm'] for r in rows if r['active_contact']]),
                    human_gap_mm=stats([r['result']['diagnostics']['human_gap_mm'][r['nearest_finger']] for r in rows]),
                    projected_gap_mm=stats([r['result']['diagnostics']['projected_gap_mm'][r['nearest_finger']] for r in rows]),
                    model_gap_mm=stats([r['result']['diagnostics']['model_gap_mm'][r['nearest_finger']] for r in rows]))
    pinch = [r for r in records if r['old_blend'] >= .8]
    valid_ref = [r for r in refs if r['reference']['feasible']]
    report = dict(prefix_frames=prefix, paired_frames=frames, errors=errors,
                  all=summary(records), previous_high_pinch_cohort=summary(pinch),
                  projected_contact=summary([r for r in records if r['active_contact']]),
                  non_contact=summary([r for r in records if not r['active_contact']]),
                  multi_contact=summary([r for r in records if len(r['active_contact'])>1]),
                  exit_transition=summary([r for r in records if r['coordinated_fingers'] and set(r['coordinated_fingers'])!=set(r['active_contact'])]),
                  natural_open_candidates=summary([r for r in records
                      if min(r['result']['diagnostics']['human_gap_mm'])>25 and
                      max(np.asarray(r['result']['diagnostics']['geometric_seed_rad'])[[4,5,7,9]])<np.deg2rad(15)]),
                  per_finger={str(f):summary([r for r in pinch if r['nearest_finger']==f]) for f in range(4)},
                  reference=dict(evaluated=len(refs), feasible=len(valid_ref),
                      infeasible=len(refs)-len(valid_ref), extra_mean_error_deg=stats([
                        max(0.,np.mean(r['new_direction'])-np.mean(r['reference']['direction_deg'])) for r in valid_ref])),
                  note='Full prefix; geometry only, no hardware dynamics simulation; independent high-budget reference is best-found, not globally certified.')
    contact_ref = [r for r in valid_ref if r['active_contact']]
    report['reference']['contact_extra_mean_error_deg'] = stats([
        max(0.,np.mean(r['new_direction'])-np.mean(r['reference']['direction_deg'])) for r in contact_ref])
    report['reference']['extra_proximal_error_deg'] = stats([
        max(0.,r['new_direction'][0]-r['reference']['direction_deg'][0]) for r in valid_ref])
    report['reference']['online_outside_band_despite_feasible_reference'] = [r['sequence'] for r in contact_ref if r['max_contact_error_mm']>2.001]
    report['target_steps_deg'] = {
        variant:{str(k):stats(np.abs(np.diff([r[key]['joints'][k] for r in records]))*180/np.pi)
                 for k in range(10)} for variant,key in [('old','original_result'),('new','result')]}
    report['provenance'] = dict(mapping_fingerprint=new.model.digest, policy=POLICY, backend=new.backend,
        kernel_sha256=hashlib.sha256(Path(__import__('inspect').getfile(HandRetargeter)).with_name('_hand_native.cpp').read_bytes()).hexdigest(),
        revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256=hashlib.sha256(Path(__import__('inspect').getfile(HandRetargeter)).read_bytes()).hexdigest(),
        baseline_sha256=hashlib.sha256(Path(args.baseline).read_bytes()).hexdigest(),
        inputs={name:dict(path=str(Path(value).resolve()),sha256=hashlib.sha256(Path(value).read_bytes()).hexdigest())
                for name,value in [('raw',args.raw),('session',args.session)]},
        reference_stride=args.reference_stride,
        open_cohort='Heuristic: all direct finger flexion <15 deg and all human tip gaps >25 mm; not manually labelled.')
    if args.execution:
        observed=[]
        for line in Path(args.execution).open():
            r=json.loads(line)
            if not start<=r.get('monotonic_s',0)<=end:continue
            f=r.get('feedback_snapshot',{}).get('left',{})
            if f.get('command_unit') is None or f.get('position_unit') is None:continue
            if not 0<=f.get('age_s',1)<=.1:continue
            observed.append((f['command_seq'],np.array(f['command_unit'])-f['position_unit'],f['age_s']*1000))
        report['recorded_execution']=dict(n=len(observed),unique_consumed_commands=len(set(v[0] for v in observed)),
            absolute_command_feedback_raw={str(i):stats([abs(v[1][i]) for v in observed]) for i in range(10)},
            feedback_age_ms=stats([v[2] for v in observed]),
            note='Only original v3 actual consumed command versus cached actual feedback; raw actuator units, not geometric error or new-algorithm hardware validation. Age is freshness, not response delay.')
    a,b=report['previous_high_pinch_cohort']['new_proximal_deg'],report['previous_high_pinch_cohort']['old_proximal_deg']
    report['gates']=dict(proximal_median=a['median']<=b['median']*.9, proximal_p95=a['p95']<=b['p95']*.9,
        mean_p95=report['previous_high_pinch_cohort']['new_mean_direction_deg']['p95']<=report['previous_high_pinch_cohort']['old_mean_direction_deg']['p95']+1,
        isolated_p95=report['all']['new_solve_ms']['p95']<=5, isolated_p99=report['all']['new_solve_ms']['p99']<=10,
        reference=bool(valid_ref) and report['reference']['extra_mean_error_deg']['p95']<=3,
        contact_reference=bool(contact_ref) and report['reference']['contact_extra_mean_error_deg']['p95']<=3)
    (out/'report.json').write_text(json.dumps(report, indent=2)); print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
