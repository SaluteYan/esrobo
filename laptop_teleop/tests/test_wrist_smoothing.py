"""SO(3) filtering, clocks, deadband and transactional acceptance; no hardware."""
import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame


def make():
    points = dict(shoulder=np.zeros(3), elbow=np.array([0., 0., -.3]),
                  wrist=np.array([.3, .1, -.4]))
    carrier = ForearmFrame().update(points)
    return AbsoluteHandOrientation(points, carrier, palm_rotation=carrier), points, carrier


def update(tracker, points, carrier, degrees, stamp, **kw):
    pose = Rotation.from_euler('x', degrees, degrees=True).as_matrix()
    return tracker.update(points, carrier, palm_rotation=carrier @ pose, sample_time=stamp, **kw)


def angle(matrix):
    return np.degrees(Rotation.from_matrix(matrix).as_rotvec()[0])


def test_adaptive_formula_and_fixed_two_degree_cap_removed():
    t,p,c = make()
    update(t,p,c,0,10.)
    result = update(t,p,c,30,10.02)
    assert angle(result) == pytest.approx(30*(1-np.exp(-2*np.pi*8*.02))-.25)
    assert angle(result) > 2
    assert not t.diagnostics['step_limited']
    assert t.diagnostics['filter_cutoff_hz'] == 8
    assert np.linalg.det(result) == pytest.approx(1)


@pytest.mark.parametrize('hz', [25, 50, 90])
def test_time_based_constant_cutoff_is_independent_of_frame_rate(hz):
    t,p,c = make()
    update(t,p,c,0,0.)
    for i in range(1,hz+1):
        r=update(t,p,c,10,i/hz,min_cutoff_hz=3,max_cutoff_hz=3,deadband_deg=0)
    assert angle(r) == pytest.approx(10*(1-np.exp(-2*np.pi*3)),abs=1e-9)


def test_deadband_holds_output_but_lowpass_continues_and_slow_motion_accumulates():
    t,p,c=make();update(t,p,c,0,0.)
    for i in range(1,51):
        r=update(t,p,c,.1*(-1)**i,i*.02)
        assert angle(r) == pytest.approx(0)
    assert t.diagnostics['deadband_active']
    assert abs(angle(t.smoothed)) > 0
    outputs=[]
    for i in range(1,101): outputs.append(angle(update(t,p,c,i*.01,1+i*.02)))
    assert max(outputs) > .7
    for a,b in zip([0]+outputs,outputs):
        assert 0 <= b-a < .025  # no deadband-sized catch-up jump


@pytest.mark.parametrize('stamp', [10., 9.9])
def test_duplicate_and_out_of_order_samples_do_not_advance(stamp):
    t,p,c=make();update(t,p,c,0,10.)
    before=copy.deepcopy(t)
    update(t,p,c,40,stamp)
    np.testing.assert_array_equal(t.filtered,before.filtered)
    np.testing.assert_array_equal(t.smoothed,before.smoothed)
    assert t.filter_stamp==10.


def test_long_gap_reseeds_clock_without_target_snap_then_resumes():
    t,p,c=make();update(t,p,c,0,10.)
    r=update(t,p,c,60,11.)
    assert angle(r)==pytest.approx(0)
    assert t.diagnostics['filter_clock_reset']
    r=update(t,p,c,60,11.02)
    assert 2 < angle(r) < 60


def test_full_target_bypasses_without_advancing_filter_or_clock():
    t,p,c=make();update(t,p,c,0,10.)
    r=update(t,p,c,80,10.02,full_target=True)
    assert angle(r)==pytest.approx(80)
    assert angle(t.filtered)==pytest.approx(0)
    assert angle(t.smoothed)==pytest.approx(0)
    assert t.filter_stamp==10.


def test_uncommitted_copy_cannot_change_time_or_smoothing_history():
    t,p,c=make();update(t,p,c,0,10.)
    candidate=copy.deepcopy(t);update(candidate,p,c,60,10.02)
    assert angle(t.filtered)==pytest.approx(0)
    assert angle(t.smoothed)==pytest.approx(0)
    assert t.filter_stamp==10.
    assert candidate.filter_stamp==10.02


def test_wraparound_uses_short_rotation_instead_of_euler_jump():
    t,p,c=make();t.reset_filter(Rotation.from_euler('x',179,degrees=True).as_matrix())
    update(t,p,c,179,10.)
    update(t,p,c,-179,10.02)
    change=Rotation.from_matrix(t.filtered @ Rotation.from_euler('x',-179,degrees=True).as_matrix()).magnitude()
    assert np.degrees(change)<2


@pytest.mark.parametrize('kw', [dict(min_cutoff_hz=0),dict(max_cutoff_hz=1),
    dict(speed_coefficient=-1),dict(deadband_deg=-1),dict(deadband_deg=float('nan')),
    dict(max_interval_s=0),dict(sample_time=float('nan'))])
def test_invalid_settings_rejected_without_filter_mutation(kw):
    t,p,c=make()
    before=t.filtered.copy()
    with pytest.raises(ValueError):
        t.update(p,c,palm_rotation=c,**kw)
    assert t.filter_stamp is None
    np.testing.assert_array_equal(t.filtered,before)


def test_whole_arm_rigid_rotation_does_not_change_local_filtered_pose():
    a,p,c=make(); b=copy.deepcopy(a)
    world=Rotation.from_euler('xyz',[25,-40,60],degrees=True).as_matrix()
    moved={k:world@v+np.array([1.,2.,3.]) for k,v in p.items()}
    # Forearm frame is built from anatomical segment directions.
    b=AbsoluteHandOrientation(moved,world@c,palm_rotation=world@c)
    for i in range(20):
        local=Rotation.from_euler('xyz',[i,2*i,-i],degrees=True).as_matrix()
        ra=a.update(p,c,palm_rotation=c@local,sample_time=i*.02)
        rb=b.update(moved,world@c,palm_rotation=world@c@local,sample_time=i*.02)
        np.testing.assert_allclose(ra,rb,atol=1e-12)


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('field',['arm_vector_filter_min_cutoff_hz',
    'arm_vector_filter_max_cutoff_hz','arm_vector_filter_speed_coefficient',
    'upper_arm_angular_deadband_deg','forearm_angular_deadband_deg',
    'wrist_orientation_filter_min_cutoff_hz',
    'wrist_orientation_filter_max_cutoff_hz','wrist_orientation_filter_speed_coefficient',
    'wrist_orientation_deadband_deg'])
def test_wrist_filter_settings_are_part_of_contract(side,field):
    from esrobo_laptop.config import ROOT, read_settings, read_robot_config
    from esrobo_laptop.contract import local_id
    cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    before=local_id(cfg,side,True)
    setattr(cfg.retarget,field,getattr(cfg.retarget,field)+.1)
    assert local_id(cfg,side,True)!=before
