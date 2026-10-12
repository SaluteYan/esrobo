"""Deadline and certificate tests without SDK/CAN or robot motion."""
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from esrobo_teleop.robot.entry_execution import EntryCadence, EntryDiagnostics
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from esrobo_teleop.robot.nero_driver import NeroArm, JointFeedbackSnapshot
from test_safe_return import make_driver


class Clock:
    def __init__(self):
        self.now = 10.
        self.cancelled = False
    def clock(self):
        return self.now
    def wait(self, duration):
        self.now += duration
        return self.cancelled
    def is_set(self):
        return self.cancelled
    def set(self):
        self.cancelled = True


@pytest.mark.parametrize('work', [.002, .008, .015])
def test_deadlines_do_not_accumulate_work(work):
    clock = Clock()
    cadence = EntryCadence(clock, clock.clock)
    sends = []
    for _ in range(30):
        assert cadence.wait()
        clock.now += work
        assert cadence.can_send()
        sends.append(clock.now)
    np.testing.assert_allclose(np.diff(sends), .02, atol=1e-12)
    assert cadence.overruns == 0


@pytest.mark.parametrize('work', [.025, .05, .1])
def test_missed_candidate_discarded_and_ticks_skipped(work):
    clock = Clock()
    cadence = EntryCadence(clock, clock.clock)
    assert cadence.wait()
    clock.now += work
    assert not cadence.can_send()
    ended = clock.now
    assert cadence.wait()
    assert clock.now >= ended
    assert cadence.skipped_ticks >= 1
    tick = clock.now
    clock.now += .002
    assert cadence.can_send()
    assert cadence.wait()
    assert clock.now == pytest.approx(tick + .02)
    assert cadence.overruns == 1


def test_cancel_interrupts_real_deadline_wait():
    cancel = threading.Event()
    cadence = EntryCadence(cancel)
    cadence.next_tick += 10
    cadence._first = False
    done = threading.Event()
    result = []
    worker = threading.Thread(target=lambda: (result.append(cadence.wait()), done.set()))
    worker.start()
    cancel.set()
    assert done.wait(.1)
    worker.join()
    assert result == [False]


def guard():
    # Exercise the real box certification/cache control, stub distance backend.
    g = TorsoCollisionGuard.__new__(TorsoCollisionGuard)
    g.lock = threading.Lock()
    g.model, g.geometry = object(), object()
    g.indices = np.arange(14)
    g.include_fingers = True
    g.thumb_envelope = 'test_enclosing_hull'
    g._hand_interval_key = ('fingers',)
    g._hand_diagnostics = {}
    g.hand_motion_horizon_s = .1
    g.margin, g.shoulder_margin = .03, .005
    g.pair_margins, g.pairs, g.shapes, g.motion_weights = {}, [], [], []
    g.last_diagnostics = {}
    g.hand_state_provider = None
    g._entry_certificates = None
    g._entry_certificate_context = None
    g._refresh_hand = mock.Mock(return_value=True)
    def clearance(q, width, **kwargs):
        g._box_certified = True
        g.last_diagnostics = dict(clearance_m=.1, required_m=.03)
        return .1
    g._clearance = mock.Mock(side_effect=clearance)
    return g


def test_certificate_covers_whole_box_and_never_unions_or_expands():
    g = guard(); g.begin_entry_certificates()
    a, b = np.zeros(14), np.ones(14)
    assert g(a,b)
    count = g._clearance.call_count
    assert g(a+.2,b-.2) and g.last_diagnostics['entry_cache_hit']
    assert g._clearance.call_count == count
    # An arbitrarily small excursion is a miss, with no tolerance expansion.
    assert g(a,b+1e-12)
    assert g._clearance.call_count == count+1
    g.invalidate_entry_certificates()
    assert g(a,b)
    assert g(b,2*b)
    count = g._clearance.call_count
    assert g(a,2*b)  # Cannot assemble two certificates.
    assert g._clearance.call_count == count+1
    assert g._refresh_hand.call_count == 6


@pytest.mark.parametrize('change', ['hand', 'margin', 'shoulder', 'pair_margin', 'mode', 'model', 'geometry', 'horizon'])
def test_changed_context_requires_full_check(change):
    g = guard(); g.begin_entry_certificates()
    a, b = np.zeros(14), np.ones(14)
    assert g(a,b)
    if change == 'hand': g._hand_interval_key = ('new_envelope',)
    if change == 'margin': g.margin = .04
    if change == 'shoulder': g.shoulder_margin = .006
    if change == 'pair_margin': g.pair_margins[(0,1)] = .04
    if change == 'model': g.model = object()
    if change == 'geometry': g.geometry = object()
    if change == 'horizon': g.hand_motion_horizon_s = .2
    if change == 'mode': assert g.check_arm_only(a,b)
    else: assert g(a,b)
    assert g._clearance.call_count == 2


def test_cache_bounded_only_successful_and_cleared_on_stale_and_end():
    g = guard(); g.begin_entry_certificates()
    for i in range(40):
        a = np.ones(14)*i
        assert g(a,a+.1)
    assert len(g._entry_certificates) == 32
    g._clearance.return_value = 0.
    g._clearance.side_effect = None
    assert not g(np.ones(14)*100,np.ones(14)*101)
    assert len(g._entry_certificates) == 32
    g._refresh_hand.return_value = False
    assert not g(np.ones(14)*39,np.ones(14)*39.05)
    assert not g._entry_certificates
    g.end_entry_certificates()
    assert g._entry_certificates is None


class Collector:
    def __init__(self, *args, **kwargs):
        self.samples = []
        self.closed = False
    def submit(self, sample):
        self.samples.append(dict(sample))
    def message(self, *args, **kwargs):
        pass
    def close(self):
        self.closed = True
    def snapshot(self):
        return dict(active=not self.closed)


def run_entry(work=.002, feedback_mode=None):
    d, feedback, zero = make_driver('right')
    clock = Clock()
    d._return_cancel = clock
    goal = zero.copy(); goal[0] += .06
    full = np.zeros(14); full[7:] = goal-zero
    calls, commits, trajectories = [], [], []
    reads = 0
    proposals = 0
    def read():
        nonlocal reads
        reads += 1
        duplicate = ((feedback_mode == 'duplicate' and reads > 1)
                     or (feedback_mode == 'one_duplicate' and reads == 2))
        d._arm.last_feedback_timestamp = 10. if duplicate else clock.now
        d._return_feedback_received = clock.now - (.2 if feedback_mode == 'stale' else 0.)
        if feedback_mode == 'missing' or (feedback_mode == 'one_missing' and reads == 2):
            return None
        return feedback.copy()
    def send(q):
        calls.append((clock.now, np.array(q)))
        feedback[:] = q
    original_propose = CommandTrajectory.propose
    original_commit = CommandTrajectory.commit
    def propose(trajectory, *args, **kwargs):
        nonlocal proposals
        before = trajectory.state
        result = original_propose(trajectory, *args, **kwargs)
        assert trajectory.state is before
        clock.now += (work[min(proposals,len(work)-1)] if isinstance(work,list) else work)
        proposals += 1
        trajectories.append(trajectory)
        return result
    def commit(trajectory, proposal):
        commits.append(proposal)
        original_commit(trajectory, proposal)
    d.read_joints = read
    d._read_entry_feedback = read
    d._arm.move_j.side_effect = send
    g = guard()
    d._collision_guard = g
    g.begin_entry_certificates()
    assert g(np.full(14, -1.), np.ones(14))
    assert g.freeze_entry_certificates()
    g._clearance.side_effect = AssertionError('geometry must not run during execution')
    with mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic', clock.clock), \
         mock.patch('esrobo_teleop.robot.nero_driver.EntryCadence', lambda cancel: EntryCadence(cancel, clock.clock)), \
         mock.patch('esrobo_teleop.robot.nero_driver.EntryDiagnostics', Collector), \
         mock.patch.object(CommandTrajectory, 'propose', propose), \
         mock.patch.object(CommandTrajectory, 'commit', commit):
        ok = d._return_group_to_zero(full,np.arange(7),goal,np.deg2rad(1.5),2.,
            handover_path=(zero,goal),entry_approach=True)
    return d, ok, calls, commits, trajectories


@pytest.mark.parametrize('work', [.002, .008, .015])
def test_entry_exec_continuous_actual_trajectory_with_fixed_cadence(work):
    d, ok, calls, commits, trajectories = run_entry(work)
    assert ok
    assert len({id(t) for t in trajectories}) == 1
    assert len(commits) == len(calls)-1
    np.testing.assert_allclose(np.diff([t for t,q in calls])[1:],.02,atol=1e-10)
    assert all(np.isfinite(c.position).all() for c in commits)
    assert d._entry_diagnostics.closed
    assert d._entry_diagnostics.samples[-1]['sent']


@pytest.mark.parametrize('work', [.025, .05, .1])
def test_entry_overrun_never_sends_or_commits_old_candidate(work):
    d, ok, calls, commits, trajectories = run_entry(work)
    assert not ok
    assert len(calls) == 1  # Initial measured-pose hold only.
    assert commits == []
    assert all(np.array_equal(t.state.position,calls[0][1]) for t in trajectories)
    assert any(s['reason'] == 'deadline_missed_candidate_discarded' for s in d._entry_diagnostics.samples)


@pytest.mark.parametrize('mode', ['duplicate', 'missing', 'stale'])
def test_feedback_protections_remain_active(mode):
    d, ok, calls, commits, trajectories = run_entry(feedback_mode=mode)
    assert not ok
    assert not commits
    assert len(calls) <= 1


def test_shared_return_defaults_do_not_enable_entry_components():
    d,feedback,zero = make_driver('right')
    with mock.patch('esrobo_teleop.robot.nero_driver.EntryDiagnostics') as diag, \
         mock.patch('esrobo_teleop.robot.nero_driver.EntryCadence') as cadence, \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert d._return_group_to_zero(np.zeros(14),np.arange(7),zero,.03,1.)
    diag.assert_not_called(); cadence.assert_not_called()


def test_async_batch_preserves_owned_samples_and_summary(tmp_path):
    log = EntryDiagnostics('right', tmp_path/'entry.jsonl')
    for i in range(12):
        log.submit(dict(monotonic_s=10.+i*.02, sent=True, send_gap_ms=None if i==0 else 20.,
                        geometry_ms=2., cache_hits=1, overruns=0, skipped_ticks=0))
    log.close()
    log._file_thread.join(1.)
    records = [json.loads(line) for line in log.path.read_text().splitlines()]
    samples = [s for batch in records for s in batch['samples']]
    assert len(samples) == 12
    assert samples[0]['monotonic_s'] == 10.
    assert samples[-1]['diagnostic_seq'] == 12
    assert records[-1]['summary']['send_hz'] == 50.
    assert records[-1]['summary']['active'] is False


def test_variable_work_still_has_fixed_send_slots():
    clock = Clock(); cadence = EntryCadence(clock,clock.clock)
    sends = []
    for work in [.002,.008,.015]*10:
        assert cadence.wait()
        clock.now += work
        assert cadence.can_send()
        sends.append(clock.now)
    np.testing.assert_allclose(np.diff(sends),.02,atol=1e-12)


def test_slow_terminal_does_not_block_file_or_boundless_samples(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked(*args, **kwargs):
        entered.set(); release.wait(2.)
    with mock.patch('esrobo_teleop.robot.entry_execution.print',blocked,create=True):
        log = EntryDiagnostics('right',tmp_path/'entry.jsonl')
        log.message('blocked terminal')
        assert entered.wait(.2)
        for _ in range(100): log.message('bounded terminal')
        for i in range(1000):
            log.submit(dict(monotonic_s=float(i),sent=False,geometry_ms=0.))
        assert log.snapshot()['console_dropped'] >= 36
        assert log.snapshot()['pending'] <= 256
        log.close()
        log._file_thread.join(1.)
        assert not log._file_thread.is_alive()
        assert log.path.stat().st_size > 0
        release.set()
        log._console_thread.join(1.)


def test_blocked_disk_nonblocking_submission_and_close(tmp_path):
    entered, release = threading.Event(),threading.Event()
    path = tmp_path/'entry.jsonl'
    real_open = Path.open
    class Stream:
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def write(self,line):
            entered.set(); release.wait(2.)
        def flush(self): pass
    with mock.patch('esrobo_teleop.robot.entry_execution.Path.open',return_value=Stream()):
        log = EntryDiagnostics('right',path)
        log.submit(dict(monotonic_s=10.,sent=False,geometry_ms=0.))
        assert entered.wait(.5)
        for i in range(1000): log.submit(dict(monotonic_s=10.+i,sent=False,geometry_ms=0.))
        assert log.snapshot()['pending'] == 256
        assert log.snapshot()['dropped'] == 744
        log.close()
        assert log._file_thread.is_alive()  # close does not join a blocked writer.
        release.set()
        log._file_thread.join(1.)


def test_disk_error_recorded_without_sync_fallback(tmp_path):
    with mock.patch('esrobo_teleop.robot.entry_execution.Path.open',side_effect=OSError('disk full')):
        log = EntryDiagnostics('right',tmp_path/'entry.jsonl')
        log._file_thread.join(.5)
        assert 'disk full' in log.snapshot()['error']
        for i in range(300): log.submit(dict(monotonic_s=float(i)))
        assert log.snapshot()['pending'] == 256
        assert log.snapshot()['dropped'] == 44
        log.close()


def test_actual_mesh_certificates_preserve_acceptance_and_rejection():
    from esrobo_teleop.config import build_config
    from esrobo_teleop.ik.solver import IkSolver
    cfg = build_config().ik
    cfg.urdf_path = str(Path(__file__).resolve().parents[1]/'urdf/esrobo_waist_with_head.urdf')
    # Recorded rejection is a 30 mm fixture, independent of runtime defaults.
    g = TorsoCollisionGuard(IkSolver(cfg),'left',margin=.03,include_fingers=False)
    a, b = np.zeros(14), np.zeros(14)
    b[0] = .001
    assert g(a,b)
    g.begin_entry_certificates()
    assert g(a,b)
    assert g(a,b*.5) and g.last_diagnostics['entry_cache_hit']
    collision = np.zeros(14)
    collision[:7] = np.deg2rad([2.27,-12.645,-25.203,28.305,.001,-.092,-.162])
    assert not g(collision,collision)
    assert not g.last_diagnostics.get('entry_cache_hit',False)
    g.end_entry_certificates()
    assert not g(collision,collision)


def test_transient_overrun_recovers_using_new_feedback_and_no_replay():
    d, ok, calls, commits, trajectories = run_entry([.025,.002])
    assert ok
    assert calls[1][0]-calls[0][0] == pytest.approx(.06)
    assert commits[0].stamp > 10.025
    np.testing.assert_allclose(np.diff([t for t,q in calls])[1:],.02,atol=1e-10)
    assert len(commits) == len(calls)-1


@pytest.mark.parametrize('failure',['cancel','fault'])
def test_entry_cache_cleanup_on_cancel_or_exception(failure):
    d,_,_ = make_driver('right')
    g = guard();d._collision_guard = g
    g.begin_entry_certificates()
    assert g(np.zeros(14),np.ones(14))
    assert g.freeze_entry_certificates()
    def execute(*args,**kwargs):
        assert g(np.zeros(14),np.ones(14))
        assert len(g._entry_certificates) == 1
        if failure == 'fault': raise RuntimeError('injected execution fault')
        d._return_cancel.set()
        return False
    d._execute_return_group = execute
    with mock.patch('esrobo_teleop.robot.nero_driver.EntryDiagnostics',Collector):
        if failure == 'fault':
            with pytest.raises(RuntimeError,match='injected'):
                d._return_group_to_zero(entry_approach=True)
        else: assert d._return_group_to_zero(entry_approach=True) is False
    assert not g._entry_certificates and g._entry_coverage_only
    assert g._entry_certificate_context is None
    assert d._entry_diagnostics.closed


def test_log_sample_copies_mutable_position_list(tmp_path):
    log = EntryDiagnostics('right',tmp_path/'entry.jsonl')
    position = [0.]*7
    log.submit(dict(monotonic_s=10.,sent=False,command_physical_rad=position))
    position[0] = 999.
    log.close();log._file_thread.join(1.)
    samples = [s for line in log.path.read_text().splitlines() for s in json.loads(line)['samples']]
    assert samples[0]['command_physical_rad'] == [0.]*7


def test_entry_reader_pins_source_and_does_not_wait_or_reset_live_trajectory():
    d,_,zero = make_driver('right')
    arm = NeroArm.__new__(NeroArm)
    arm._runtime_feedback = JointFeedbackSnapshot(zero,100.,10.,'get_joint_angles')
    arm._runtime_feedback_recovery_candidate = None
    arm._last_feedback_timestamp = 100.
    message = SimpleNamespace(msg=zero.copy(),timestamp=100.02)
    arm._robot = SimpleNamespace(get_joint_angles=lambda:message)
    arm.get_joint_angles = mock.Mock(side_effect=AssertionError('blocking reader used'))
    d._arm = arm
    live_state = d._trajectory.state
    with mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic',return_value=10.02):
        actual=d._read_entry_feedback()
    np.testing.assert_array_equal(actual,zero)
    actual[0]=999.
    assert arm._runtime_feedback.position[0] != 999.
    assert d._return_feedback_received == 10.02
    assert arm.last_feedback_timestamp == 100.02
    assert d._trajectory.state is live_state
    message.msg[0] += .001
    with mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic',return_value=10.04):
        repeated=d._read_entry_feedback()
    np.testing.assert_array_equal(repeated,zero)
    assert d._return_feedback_received == 10.02
    assert arm._runtime_feedback.received_monotonic == 10.02
    arm.get_joint_angles.assert_not_called()


def test_existing_planning_certificates_survive_execution_wrapper():
    d,_,_ = make_driver('right')
    g=guard();d._collision_guard=g
    g.begin_entry_certificates()
    assert g(np.zeros(14),np.ones(14))
    assert g.freeze_entry_certificates()
    checks=g._clearance.call_count
    def execute(*args,**kwargs):
        assert g(np.ones(14)*.2,np.ones(14)*.8)
        assert g.last_diagnostics['entry_cache_hit']
        assert g._clearance.call_count == checks
        return True
    d._execute_return_group=execute
    with mock.patch('esrobo_teleop.robot.nero_driver.EntryDiagnostics',Collector):
        assert d._return_group_to_zero(entry_approach=True)
    assert not g._entry_certificates and g._entry_coverage_only
    g.end_entry_certificates()


def test_interval_fault_distinguishes_command_state_from_observed_feedback():
    d,ok,calls,commits,_=run_entry(.05)
    assert not ok and len(calls)==1 and not commits
    assert 'since last committed command state' in d._return_execution_failure


@pytest.mark.parametrize('obstructed', [False, True])
def test_entry_subdivision_reuses_only_parent_pair_proofs(obstructed):
    g = guard()
    calls = []
    leaves = []
    def clearance(center, halfwidth, *, include_fingers=True, active_pairs=None):
        pairs = ('near', 'far', 'finger') if active_pairs is None else (*active_pairs[0], *active_pairs[1])
        calls.extend(pairs)
        distances = {'near': .02 if obstructed and center[0] > .1 else .1,
                     'far': 1., 'finger': .14}
        unresolved = [pair for pair in pairs if distances[pair] - g.margin - halfwidth[0] <= 0]
        g._box_certified = not unresolved
        g._uncertified_pairs = (tuple(p for p in unresolved if p != 'finger'),
                                tuple(p for p in unresolved if p == 'finger'))
        g._split_weights = np.array([1.] + [0.]*13)
        g.last_diagnostics = {'clearance_m': min(distances[p] for p in pairs), 'required_m': g.margin}
        if g._box_certified:
            leaves.append((center[0]-halfwidth[0],center[0]+halfwidth[0]))
        return min(distances[p] for p in pairs)
    g._clearance = clearance
    g.begin_entry_certificates()
    a,b=np.zeros(14),np.zeros(14);a[0],b[0]=-.2,.2
    assert g(a,b) is (not obstructed)
    assert calls.count('far') == 1
    assert calls.count('near') > 1
    assert calls.count('finger') > 1
    if not obstructed:
        # The certified leaves still cover the entire original box, no gaps.
        leaves.sort()
        assert leaves[0][0] == pytest.approx(-.2)
        assert leaves[-1][1] == pytest.approx(.2)
        for left,right in zip(leaves,leaves[1:]):
            assert left[1] == pytest.approx(right[0])
    assert len(g._entry_certificates) == (0 if obstructed else 1)
