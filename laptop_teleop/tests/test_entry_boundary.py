"""Recorded J6 boundary fault; no robot interface is constructed."""
from unittest.mock import Mock
import json
import subprocess

import numpy as np
import pytest

from esrobo_laptop.config import ROOT
from esrobo_laptop.pipeline import Pipeline


def pipeline():
    p = Pipeline.__new__(Pipeline)
    lo, hi = np.full(7, -2.6), np.full(7, 2.6)
    lo[5], hi[5] = -.663225, .872665
    margin = np.deg2rad(.25)
    p._entry_limits = {'right': (lo+margin, hi-margin)}
    p._diagnostic_print = Mock()
    return p


def test_recorded_goal_insets_without_clipping_feedback_or_other_arm():
    p = pipeline()
    goal = np.zeros(14)
    goal[11] = -2.5999999999
    before = goal.copy()
    result = p._entry_goal_inside_limits('right', goal)
    assert result[11] >= p._entry_limits['right'][0][4]
    np.testing.assert_array_equal(goal, before)
    np.testing.assert_array_equal(result[:7], before[:7])
    assert p._diagnostic_print.call_count == 1
    assert 'J5' in p._diagnostic_print.call_args.args[0]


def test_arm_only_keeps_measured_wrist_and_interior_target_unchanged():
    p = pipeline()
    goal = np.zeros(14);goal[12] = -.6632249999
    np.testing.assert_array_equal(p._entry_goal_inside_limits('right', goal),goal)
    np.testing.assert_array_equal(p._entry_goal_inside_limits('right', np.zeros(14)),np.zeros(14))
    p._diagnostic_print.assert_not_called()


def test_initialization_reports_real_outside_feedback_without_relaxing_limit():
    from test_command_trajectory import DriverTrajectoryTests
    fixture = DriverTrajectoryTests()
    d = fixture.driver('right')
    lo, _ = d._effective_position_limits('right')
    fixture.current[5] = lo[5]-np.deg2rad(.006)
    assert d.capture_session_start()
    assert not d.initialize_command_trajectory()
    assert 'J6=' in d.safety_fault_reason
    assert d.trajectory_diagnostics['fault_context']['failed_joints'] == [6]
    assert d.trajectory_diagnostics['fault_context']['fault_code'] == 'initial_feedback_outside_position_envelope'
    assert d._trajectory.state is None
    d._arm.move_j.assert_not_called()
    assert fixture.current[5] < lo[5]


def test_viewer_distinguishes_pause_body_gap_and_stale_computation():
    path = ROOT.parent/'teleoperation/web/pico_skeleton_viewer/freshness.js'
    script = '''
const fs=require('fs');
(async()=>{
 const {freshnessWarning:f}=await import('data:text/javascript;base64,'+fs.readFileSync(process.argv[1]).toString('base64'));
 const state={age_ms:12,packet:{frames:{},body_sample_sequence:28801},diagnostics_age_ms:2000,diagnostics:{frames:{ik:{}}}};
 console.log(JSON.stringify([f(state,'robot',false),f(state,'robot',true),f({...state,age_ms:120000},'robot',false),f(state,'ik',false),f({},'raw',false)]));
})();
'''
    values=json.loads(subprocess.check_output(['/usr/lib/chatgpt/resources/cua_node/bin/node','-e',script,str(path)],text=True))
    assert values[0] == ''
    assert '暂停' in values[1]
    assert '120.0' in values[2] and '28801' in values[2]
    assert '计算层' in values[3]
    assert '尚未收到' in values[4]


@pytest.mark.parametrize('bias_deg,arrived', [(0., True), (-.006, False)])
def test_arrival_cannot_accept_stationary_feedback_outside_envelope(bias_deg, arrived, monkeypatch):
    from test_safe_return import make_driver
    from test_entry_execution import Clock, Collector, guard
    from esrobo_teleop.robot.entry_execution import EntryCadence
    from esrobo_teleop.robot import nero_driver
    d, feedback, zero = make_driver('right')
    clock = Clock(); d._return_cancel = clock
    goal = zero.copy();goal[0] -= .02
    lower, upper = zero-1., zero+1.;lower[0] = goal[0]
    d._effective_position_limits = lambda side: (lower.copy(), upper.copy())
    def read():
        d._arm.last_feedback_timestamp = clock.now
        d._return_feedback_received = clock.now
        return feedback.copy()
    def send(q):
        feedback[:] = q
        feedback[0] += np.deg2rad(bias_deg)
    d.read_joints = read;d._read_entry_feedback = read
    d._arm.move_j.side_effect = send
    g = guard(); d._collision_guard = g
    g.begin_entry_certificates()
    assert g(np.full(14, -2.), np.full(14, 2.))
    assert g.freeze_entry_certificates()
    g._clearance.side_effect = AssertionError('entry cannot recompute geometry')
    monkeypatch.setattr(nero_driver.time,'monotonic',clock.clock)
    monkeypatch.setattr(nero_driver,'EntryCadence',lambda cancel:EntryCadence(cancel,clock.clock))
    monkeypatch.setattr(nero_driver,'EntryDiagnostics',Collector)
    target=np.zeros(14);target[7:]=goal-zero
    assert d._return_group_to_zero(target,np.arange(7),goal,np.deg2rad(1.5),2.,
        handover_path=(zero,goal),entry_approach=True) is arrived
    if not arrived:
        assert 'timeout' in d._return_execution_failure
        assert feedback[0] < lower[0]
