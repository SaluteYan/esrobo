"""Offline before/after entry replay; never opens an SDK or motion interface.

Uses a supported test clock, measured poses and the real trajectory/cache
control. Collision distances are stubbed: this is not geometry certification
for a physical robot. All work-cost injection lives in this test tool.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
from unittest import mock

import numpy as np

from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from esrobo_teleop.robot.entry_execution import EntryCadence
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
from test_entry_execution import Clock, Collector, guard
from test_safe_return import make_driver


def replay(old=None, cold_geometry_s=.008):
    d, feedback, zero = make_driver('right')
    clock = Clock()
    d._return_cancel = clock
    goal = zero.copy(); goal[0] += .06
    target = np.zeros(14); target[7:] = goal-zero
    g = guard()
    d._collision_guard = g
    g.begin_entry_certificates()
    # Planning produced a whole-box certificate in URDF coordinates. The
    # executor may reuse it only through the guard's real coverage/context test.
    assert g(np.full(14,-.2), np.full(14,.2))
    original_clearance = g._clearance.side_effect
    def clearance(*a, **kw):
        clock.now += cold_geometry_s
        return original_clearance(*a, **kw)
    g._clearance.side_effect = clearance
    checks = []
    def path(a,b):
        clock.now += .0001
        x,y = np.zeros(14),np.zeros(14)
        x[7:],y[7:] = a-zero,b-zero
        ok = g(x,y)
        checks.append(bool(g.last_diagnostics.get('entry_cache_hit')))
        return ok
    d._check_physical_path = path
    def read(blocking=False):
        if blocking: clock.now += .01
        d._arm.last_feedback_timestamp = clock.now
        d._return_feedback_received = clock.now
        return feedback.copy()
    d.read_joints = lambda: read(True)
    d._read_entry_feedback = read
    sent, commits = [], []
    def send(q):
        clock.now += .0002
        sent.append((clock.now, np.asarray(q).copy()))
        feedback[:] = q
    d._arm.move_j.side_effect = send
    original_commit = CommandTrajectory.commit
    def commit(t,p):
        commits.append(p)
        return original_commit(t,p)
    if old:
        d._execute_return_group = old.NeroSingleArmDriver._execute_return_group.__get__(d)
        entry = old.NeroSingleArmDriver._return_group_to_zero.__get__(d)
    else:
        entry = d._return_group_to_zero
    module = old or sys.modules[NeroSingleArmDriver.__module__]
    with mock.patch.object(module.time,'monotonic',clock.clock), \
         mock.patch.object(module,'EntryCadence',lambda c:EntryCadence(c,clock.clock)), \
         mock.patch.object(module,'EntryDiagnostics',Collector), \
         mock.patch.object(CommandTrajectory,'commit',commit):
        ok = entry(target,np.arange(7),goal,np.deg2rad(1.5),2.,
                   handover_path=(zero,goal),entry_approach=True)
    gaps = np.diff([t for t,q in sent])*1000
    g.end_entry_certificates()
    return dict(success=bool(ok), sent=len(sent), committed=len(commits),
        goal_physical_rad=goal.tolist(), final_feedback_rad=feedback.tolist(),
        full_checks=g._clearance.call_count-1, requests=len(checks), cache_hits=sum(checks),
        send_interval_ms=dict(median=float(np.median(gaps)),p95=float(np.percentile(gaps,95)),
                              maximum=float(max(gaps))) if len(gaps) else None,
        failure=getattr(d,'_return_execution_failure',None))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before',required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    spec=importlib.util.spec_from_file_location('esrobo_teleop.robot._entry_before',args.before)
    old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
    report=dict(test_only=True, physical_motion=False, distance_backend='stubbed',
        cases=[dict(cold_geometry_ms=ms,before=replay(old,ms/1000),after=replay(None,ms/1000))
               for ms in (8,15,25)])
    Path(args.output).write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
