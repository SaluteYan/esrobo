"""Slow visualization never runs FK or serialization on the controller thread."""
import json
import threading
import time
from unittest.mock import Mock

import numpy as np

from esrobo_laptop.config import Settings
from esrobo_laptop.pipeline import Pipeline
from esrobo_laptop.visualization import ArmVisualization
from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerState


def poses(q):
    return {'right_shoulder':np.array([0.,0.,0.]),
            'right_elbow':np.array([q[7],1.,0.]),
            'right_wrist':np.array([q[7],2.,0.])}


def pipeline(worker):
    p = Pipeline.__new__(Pipeline)
    p.settings = Settings(side='right',with_hand=False)
    p.solvers = {'right': Mock()}
    p.wrists = {}
    p._visualization = worker
    p._last_diagnostics_time = 0.
    return p


def source():
    return (({'right_arm':time.monotonic()}, poses(np.zeros(14)), None),
            {'right':dict(arm_urdf_rad=[.1]*7)}, np.zeros(14))


def test_slow_fk_does_not_block_publish_and_snapshot_owns_arrays():
    entered, release, delivered = threading.Event(), threading.Event(), threading.Event()
    rows, threads = [], []
    def fk(q):
        threads.append(threading.get_ident())
        entered.set()
        assert release.wait(2.)
        return poses(q)
    def send(data):
        rows.append(json.loads(data));delivered.set()
    worker = ArmVisualization(fk=fk, sender=send)
    p = pipeline(worker)
    frame, targets, measured = source()
    try:
        p.publish_diagnostics(frame, targets, measured, preview=True, robot_mode='ACTIVE')
        assert entered.wait(.5)
        frame[1]['right_elbow'][:] = 99
        targets['right']['arm_urdf_rad'][:] = [88]*7
        measured[:] = 77
        release.set()
        assert delivered.wait(.5)
        assert all(t != threading.get_ident() for t in threads)
        assert rows[0]['frames']['retarget']['right_elbow']['pos'] == [0.,1.,0.]
        assert rows[0]['frames']['ik']['right_elbow']['pos'] == [.1,1.,0.]
        assert rows[0]['frames']['feedback']['right_elbow']['pos'] == [0.,1.,0.]
        assert rows[0]['source_sample_monotonic_s'] == frame[0]['right_arm']
        p.solvers['right'].current_task_frame_poses.assert_not_called()
    finally:
        release.set();worker.close()


def test_blocked_sender_keeps_only_latest_pending_and_close_does_not_wait():
    entered, release = threading.Event(), threading.Event()
    def send(data):
        entered.set();release.wait(2.)
    worker = ArmVisualization(fk=poses,sender=send)
    p = pipeline(worker)
    frame, targets, measured = source()
    snapshot = p._diagnostic_snapshot(frame, targets, measured, preview=True, robot_mode='ACTIVE')
    try:
        worker.submit(snapshot)
        assert entered.wait(.5)
        for _ in range(1000):worker.submit(snapshot)
        assert worker.stats()['pending'] == 1
        assert worker.stats()['coalesced'] == 999
        assert not worker.close(timeout=0)
        assert worker.stats()['pending'] == 0
    finally:
        release.set();worker.close()


def test_viewer_uses_generation_time_and_rejects_older_or_invalid_updates(monkeypatch):
    monkeypatch.setattr(time, 'monotonic', lambda:10.)
    state = SkeletonViewerState()
    state.update_diagnostics({'generated_monotonic_s':9.8,'frames':{}})
    assert state.snapshot()['diagnostics_age_ms'] == 200.
    for stamp in (9.7,11.,float('nan'),True):
        state.update_diagnostics({'generated_monotonic_s':stamp,'frames':{'bad':True}})
        assert state.snapshot()['diagnostics']['frames'] == {}


def test_display_fk_matches_solver_without_sharing_data():
    from esrobo_laptop.config import ROOT
    from esrobo_laptop.visualization import DisplayFK
    from esrobo_teleop.config import load_config
    from esrobo_teleop.ik.solver import IkSolver
    cfg = load_config(str(ROOT.parent/'teleoperation/config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(ROOT.parent/'teleoperation/urdf/esrobo_waist_with_head.urdf')
    solver = IkSolver(cfg.ik)
    display = DisplayFK(cfg.ik)
    assert display.model is not solver._model and display.data is not solver._data
    for joints in (np.zeros(14), np.linspace(-.5,.5,14), np.full(14,10.)):
        expected, actual = solver.current_task_frame_poses(joints), display(joints)
        assert expected.keys() == actual.keys()
        for name in actual:
            np.testing.assert_allclose(actual[name], expected[name][:3], atol=1e-12)
