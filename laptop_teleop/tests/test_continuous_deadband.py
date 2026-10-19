"""Angular continuity, slow intent and unchanged filtering admission; offline."""
import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import ANGULAR_DEADBAND_MODE, RetargetConfig
from esrobo_teleop.device.body_device import BodyDevice
from test_wrist_smoothing import make, update, angle


def device():
    d=BodyDevice.__new__(BodyDevice)
    d._cfg=RetargetConfig()
    d._held_segment_directions={s:{} for s in ('left','right')}
    d._filtered_segment_directions={s:{} for s in ('left','right')}
    d._filtered_segment_timestamps={s:{} for s in ('left','right')}
    d.motion_trace_enabled=True
    return d


def vector(degrees):
    r=np.deg2rad(degrees)
    return np.array([np.cos(r),np.sin(r),0.])


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('segment,db',[('upper_arm',.25),('forearm',.30)])
def test_direction_threshold_has_no_output_jump(side,segment,db):
    d=device()
    f=lambda v:d._apply_segment_direction_deadband(side,segment,vector(v),db)
    f(0)
    np.testing.assert_array_equal(f(db-.0001),vector(0))
    small=f(db+.0001)
    assert np.degrees(np.arctan2(small[1],small[0]))==pytest.approx(.0001,abs=2e-6)
    info=d._trace_filter_details[side][segment]
    assert info['deadband_mode']==ANGULAR_DEADBAND_MODE
    assert info['deadband_update_deg']<.001
    # A constant input stays constant, instead of creeping through the band.
    for _ in range(100):np.testing.assert_allclose(f(db+.0001),small,atol=1e-12)


@pytest.mark.parametrize('side',['left','right'])
def test_direction_slow_motion_and_reversal_remain_responsive(side):
    d=device();outs=[]
    for i,a in enumerate(np.r_[np.linspace(0,2,201),np.linspace(2,-2,401)]):
        d._last_body_frame_packet_time_monotonic=i*.02
        smooth=d._filter_segment_direction(side,'upper_arm',vector(a))
        v=d._apply_segment_direction_deadband(side,'upper_arm',smooth,.25)
        outs.append(np.degrees(np.arctan2(v[1],v[0])))
    assert outs[200]>1.65 and outs[-1]<-1.65
    assert np.max(abs(np.diff(outs)))<.025
    # Once settled, repeated timestamps cannot advance the low-pass state.
    old=d._filtered_segment_directions[side]['upper_arm'].copy()
    np.testing.assert_array_equal(d._filter_segment_direction(side,'upper_arm',vector(80)),old)


@pytest.mark.parametrize('target',[[-1,0,0],[-1,1e-10,0],[1,1e-10,0]])
def test_spherical_deadband_antipodal_and_tiny_angles(target):
    result,gap,step=mu.continuous_direction_deadband([1,0,0],target,np.deg2rad(.3))
    assert np.linalg.norm(result)==pytest.approx(1,abs=1e-12)
    assert np.isfinite(result).all() and 0<=step<=gap
    desired=np.array(target,dtype=float);desired/=np.linalg.norm(desired)
    remaining=np.arctan2(np.linalg.norm(np.cross(result,desired)),result@desired)
    assert remaining<=np.deg2rad(.3)+1e-9


def test_spherical_zero_deadband_and_rotated_coordinate_system():
    a,b=vector(10),vector(20);rot=Rotation.from_euler('xyz',[70,20,-50],degrees=True).as_matrix()
    out,_,_=mu.continuous_direction_deadband(a,b,.01)
    moved,_,_=mu.continuous_direction_deadband(rot@a,rot@b,.01)
    np.testing.assert_allclose(moved,rot@out,atol=1e-12)
    np.testing.assert_allclose(mu.continuous_direction_deadband(a,b,0)[0],b,atol=1e-12)


@pytest.mark.parametrize('axis',['x','y','z'])
def test_so3_threshold_continuity_and_constant_input(axis):
    t,p,c=make();t.filter_stamp=1.
    # A fixed low-pass state lets this test isolate the output deadband.
    pose=Rotation.from_euler(axis,.2501,degrees=True).as_matrix()
    t.smoothed=pose.copy()
    out=t.update(p,c,palm_rotation=c@pose,sample_time=1.02)
    assert np.degrees(Rotation.from_matrix(out).magnitude())==pytest.approx(.0001,abs=1e-9)
    assert t.diagnostics['deadband_update_deg']==pytest.approx(.0001,abs=1e-9)
    assert t.diagnostics['deadband_mode']==ANGULAR_DEADBAND_MODE
    for i in range(100):
        repeated=t.update(p,c,palm_rotation=c@pose,sample_time=1.04+i*.02)
        np.testing.assert_allclose(repeated,out,atol=1e-12)


def test_so3_reversal_and_unaccepted_candidate_do_not_move_committed_state():
    t,p,c=make();update(t,p,c,0,0.)
    outputs=[]
    for i,a in enumerate(np.r_[np.linspace(0,2,201),np.linspace(2,-2,401)],1):
        outputs.append(angle(update(t,p,c,a,i*.02)))
    assert outputs[200]>1.65 and outputs[-1]<-1.65
    assert max(abs(np.diff(outputs)))<.025
    before=copy.deepcopy(t)
    candidate=copy.deepcopy(t);update(candidate,p,c,80,t.filter_stamp+.02)
    np.testing.assert_array_equal(t.filtered,before.filtered)
    np.testing.assert_array_equal(t.smoothed,before.smoothed)
    assert t.filter_stamp==before.filter_stamp


def test_config_defaults_match_commissioned_values():
    from esrobo_laptop.config import ROOT,read_robot_config,read_settings
    for c in [RetargetConfig(),read_robot_config(read_settings(ROOT/'config/laptop.yaml')).retarget]:
        assert c.arm_vector_filter_min_cutoff_hz==c.wrist_orientation_filter_min_cutoff_hz==2
        assert c.arm_vector_filter_max_cutoff_hz==c.wrist_orientation_filter_max_cutoff_hz==8
        assert c.arm_vector_filter_speed_coefficient==c.wrist_orientation_filter_speed_coefficient==.7
        assert (c.upper_arm_angular_deadband_deg,c.forearm_angular_deadband_deg,c.wrist_orientation_deadband_deg)==(.25,.30,.25)
