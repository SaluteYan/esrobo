"""Offline reproduction of filter catch-up entering the input recovery gate.

Uses the real orientation filter, wrist mapping and Controller with the existing
mock entry fixture. Does not subscribe to PICO or connect to a physical gateway.
Run with the project/test PYTHONPATH and --output <new JSON file>.
Reports fresh-frame recovery and unfiltered source diagnostics, including source changes.
The arm_only cases exercise the generic controller path with an injected wrist
filter; production arm_only does not construct the PICO palm orientation mapper.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame
from test_absolute_hand import points
from test_wrist_handover import setup
from test_wrist_jacobian import make_wrist
from esrobo_teleop.config import RetargetConfig


def replay(side, with_hand, *, source_jump=0., gap_s=.12):
    with pytest.MonkeyPatch.context() as patch:
        controller, pipeline, client, state, clock, tick = setup(patch, side, with_hand)
        controller.recovery_enabled = True
        pipeline.cfg.retarget = RetargetConfig()
        arm = points()
        carrier = ForearmFrame().update(arm)
        desired = Rotation.from_euler('z', 30, degrees=True).as_matrix()
        tracker = AbsoluteHandOrientation(arm, carrier, palm_rotation=carrier @ desired)
        tracker.reset_filter(np.eye(3))
        wrist = make_wrist()
        rows = []
        candidate = None

        def capture(**kwargs):
            nonlocal candidate
            candidate = copy.deepcopy(tracker)
            local = candidate.update(arm, carrier, palm_rotation=carrier @ desired,
                                     sample_time=clock[0])
            return ({side+'_arm':clock[0],side+'_palm':clock[0]},
                    {'local':local,'hand_local_orientations':{side:local}}, None)

        def commit(frame, targets):
            nonlocal tracker
            tracker = candidate

        def solve(frame, *args, **kwargs):
            local = frame[1]['local']
            terminal = wrist.terminal(local)
            raw = make_wrist().terminal(desired)
            rows.append(dict(t=clock[0],source_palm_deg=float(np.degrees(raw[2])),
                             filtered_j7_deg=float(np.degrees(terminal[2]))))
            return {side: dict(arm_urdf_rad=[0.]*4+terminal.tolist(),
                               source_arm_urdf_rad=[0.]*4+raw.tolist(),
                               hand_unit=[128]*10 if with_hand else None)}

        pipeline.capture.side_effect = capture
        pipeline.solve.side_effect = solve
        pipeline.commit_frame.side_effect = commit
        # Commit a real filter history before the input gap. Initial samples
        # seed clocks; never assume the removed fixed 2-degree step.
        for _ in range(5):
            target = tick()
            state['feedback']['arm_urdf_rad'] = copy.deepcopy(target[side]['arm_urdf_rad'])
        before_gap = float(np.degrees(Rotation.from_matrix(tracker.filtered).magnitude()))
        rows.clear()
        desired = Rotation.from_euler('z', 30+source_jump, degrees=True).as_matrix()
        clock[0] += .06
        client.received_at = clock[0]
        assert controller.hold_missing_input(state, 'simulated single input gap')
        clock[0] += max(0.,gap_s-.06)
        resumed_after = None
        for _ in range(20):
            result = tick()
            if result is not None:
                state['feedback']['arm_urdf_rad'] = copy.deepcopy(result[side]['arm_urdf_rad'])
                if resumed_after is None: resumed_after = len(rows)
            rows[-1]['recovery'] = copy.deepcopy(controller.input_resume_diagnostics)
            rows[-1]['committed_filter_deg'] = float(np.degrees(Rotation.from_matrix(tracker.filtered).magnitude()))
        try:
            tick(.12)
        except RuntimeError as exc:
            timeout = str(exc)
        else:
            timeout = None
        return dict(side=side, with_hand=with_hand, constant_human_target=not source_jump,
                    gap_s=gap_s, resumed_after_frames=resumed_after, before_gap_filter_deg=before_gap,
                    committed_filter_deg=float(np.degrees(Rotation.from_matrix(tracker.filtered).magnitude())),
                    last_sent_source_deg=float(np.degrees(controller.last_arm_source[side][6])),
                    matched=controller.input_resume_count, timeout=timeout, rows=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    results=[replay(side,hand,source_jump=jump) for side in ('left','right')
             for hand in (False,True) for jump in (0.,8.)]
    with Path(args.output).open('x') as stream:
        json.dump(results,stream,indent=2)
    for result in results:
        print(json.dumps({k:v for k,v in result.items() if k!='rows'}))


if __name__=='__main__':
    main()
