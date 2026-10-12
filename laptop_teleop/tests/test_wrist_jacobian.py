import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.mapping import WristMapping, _wrist_residual_jacobian


@pytest.mark.parametrize('handedness', [-1, 1])
def test_rotation_log_jacobian_matches_independent_central_difference(handedness):
    rng = np.random.default_rng(20261010)
    for i in range(100):
        axes = Rotation.random(random_state=rng).as_matrix()
        axes[:, 0] *= handedness
        delta = rng.uniform(-2.7, 2.7, 3)
        def rotation(q):
            value = Rotation.identity()
            for axis, angle in zip(axes.T, q):
                value *= Rotation.from_rotvec(axis*angle)
            return value
        desired = (rotation(delta) if i == 0 else
                   rotation(delta)*Rotation.from_rotvec([1.e-7, 0., 0.]) if i == 1 else
                   Rotation.random(random_state=rng))
        residual, jacobian = _wrist_residual_jacobian(axes, delta, desired)
        def error(q):
            return (rotation(q).inv()*desired).as_rotvec()
        epsilon = 1.e-6
        numeric = np.column_stack([
            (error(delta+np.eye(3)[j]*epsilon)-error(delta-np.eye(3)[j]*epsilon))/(2*epsilon)
            for j in range(3)])
        np.testing.assert_allclose(residual, error(delta), atol=1.e-12)
        np.testing.assert_allclose(jacobian, numeric, atol=3.e-8, rtol=3.e-7)


def make_wrist(axes=None):
    wrist=WristMapping.__new__(WristMapping)
    wrist.axes=np.eye(3) if axes is None else axes
    wrist.neutral_urdf=np.zeros(3)
    wrist.terminal_lower=np.array([-2.8,-.65,-1.])
    wrist.terminal_upper=-wrist.terminal_lower
    wrist._last_terminal_target=np.array([1.,.2,.3])
    return wrist


@pytest.mark.parametrize('handedness',[-1,1])
def test_exact_wrist_has_unique_bounded_solution_for_both_axis_handedness(handedness):
    rng=np.random.default_rng(42)
    axes=Rotation.from_euler('xyz',[.3,-.2,.5]).as_matrix()
    axes[:,0]*=handedness
    wrist=make_wrist(axes)
    for _ in range(80):
        target=rng.uniform(wrist.terminal_lower,wrist.terminal_upper)
        desired=Rotation.identity()
        for axis,q in zip(axes.T,target):desired*=Rotation.from_rotvec(axis*q)
        result=wrist.terminal(desired.as_matrix())
        np.testing.assert_allclose(result,target,atol=1.e-10)
        assert wrist.last_diagnostics['solver_method']=='certified_exact_wrist'
        assert wrist.last_diagnostics['solver_evaluations']==0


@pytest.mark.parametrize('case',['outside','singular_range','nonorthogonal'])
def test_uncertified_wrist_keeps_bounded_solver(case):
    wrist=make_wrist()
    target=np.array([.3,.2,.1])
    if case=='outside':target[1]=1.1
    elif case=='singular_range':wrist.terminal_upper[1]=np.pi/2
    else:
        wrist.axes[:,1]=[.05,np.sqrt(1-.05**2),0.]
    desired=Rotation.identity()
    for axis,q in zip(wrist.axes.T,target):desired*=Rotation.from_rotvec(axis*q)
    result=wrist.terminal(desired.as_matrix())
    assert wrist.last_diagnostics['solver_method']=='bounded_least_squares'
    assert wrist.last_diagnostics['solver_evaluations']>0
    assert np.all(result>=wrist.terminal_lower) and np.all(result<=wrist.terminal_upper)


def test_rounded_urdf_axes_are_corrected_using_actual_geometry():
    axes=np.eye(3);axes[0,2]=2.7e-8;axes[1,2]=-2.7e-8
    axes/=np.linalg.norm(axes,axis=0)
    wrist=make_wrist(axes)
    target=np.array([-1.5,.35,-.6])
    desired=Rotation.identity()
    for axis,q in zip(axes.T,target):desired*=Rotation.from_rotvec(axis*q)
    actual=wrist.terminal(desired.as_matrix())
    np.testing.assert_allclose(actual,target,atol=1.e-10)
    assert wrist.last_diagnostics['solver_method']=='certified_exact_wrist'
