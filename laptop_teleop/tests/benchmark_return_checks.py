"""Test-only real-geometry timing at a recorded pose; never constructs hardware."""
import argparse
import json
from pathlib import Path
import time
import threading
from types import SimpleNamespace
import numpy as np
from esrobo_link.backends import ForwardModel
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES


def run(root, fixture, repeats, execute=False, driver_source=None):
    cfg = load_config(str(root/'teleoperation/config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(root/'teleoperation/urdf/esrobo_waist_with_head.urdf')
    model = ForwardModel(cfg.ik)
    data = json.loads(fixture.read_text())['feedback']
    q = np.zeros(14); q[:7] = data['arm_urdf_rad']
    hand = data['hand']['position_unit']
    guard = TorsoCollisionGuard(model, 'left', cfg.robot.collision_package_dirs,
        cfg.robot.torso_collision_margin_m, cfg.robot.shoulder_collision_margin_m)
    guard.hand_state_provider = lambda horizon: geometry_state(
        hand, time.monotonic()-.01, 'left', cfg.hand.geometry_feedback_calibration,
        L10_PHYSICAL_JOINT_NAMES, 0., cfg.hand.geometry_feedback_timeout_s)
    rows=[]
    for cache in (False, True):
        if cache: guard.begin_entry_certificates()
        costs=[]; checks=[]
        for n in range(repeats):
            # Three checks used by a return candidate: outstanding command,
            # combined tracking/braking box, then the small final sent segment.
            delta=np.zeros(14);delta[:7]=.001
            started=time.perf_counter()
            for a,b in ((q,q), (q-delta,q+delta), (q,q+delta*.1)):
                t=time.perf_counter(); ok=guard(a,b)
                checks.append(dict(ms=1000*(time.perf_counter()-t),ok=ok,
                    category=guard.last_diagnostics.get('category'),
                    cache_hit=guard.last_diagnostics.get('entry_cache_hit')))
            costs.append(1000*(time.perf_counter()-started))
        guard.end_entry_certificates()
        rows.append(dict(cache=cache, cycles_ms=costs, checks=checks,
                         median_ms=float(np.median(costs)),max_ms=max(costs)))
    result=dict(test_only=True, physical_motion=False, limitation=__doc__, results=rows)
    if execute:
        from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver, JointFeedbackSnapshot
        if driver_source is not None:
            import importlib.util
            import sys
            name='esrobo_teleop.robot._benchmark_driver'
            spec=importlib.util.spec_from_file_location(name, driver_source)
            module=importlib.util.module_from_spec(spec)
            sys.modules[name]=module
            spec.loader.exec_module(module)
            NeroSingleArmDriver=module.NeroSingleArmDriver
        from esrobo_teleop.robot.staged_return_planner import StagedReturnPlanner
        driver=NeroSingleArmDriver.__new__(NeroSingleArmDriver)
        driver._cfg,driver._side,driver._session_start=cfg.robot,'left',None
        driver._return_cancel=threading.Event()
        driver._trajectory_fk=model.fk('left')
        driver._trajectory_endpoint_speed=cfg.retarget.max_endpoint_translation_velocity_m_s
        driver.configure_collision_guard(guard)
        direction,zero=driver._mapping('left')
        actual=direction*q[:7]+zero
        driver._return_start_state=None
        driver._command_velocity={'left':np.zeros(7)}
        driver._arm=SimpleNamespace(last_feedback_timestamp=None)
        driver._controller_is_safe=lambda **kw:True
        driver._torque_is_safe=lambda *a:True
        driver._return_execution_failure=None
        sent=[];costs=[]
        original_check=driver._check_physical_path
        def check(a,b,**kw):
            t=time.perf_counter();ok=original_check(a,b,**kw)
            costs.append(dict(ms=1000*(time.perf_counter()-t),ok=ok,
                              boxes=guard.last_diagnostics.get('checked_boxes')))
            return ok
        driver._check_physical_path=check
        def read():
            time.sleep(.01)  # Existing two-sample polling, simulated only.
            driver._arm.last_feedback_timestamp=time.monotonic()
            return actual.copy()
        driver.read_joints=read
        def snapshot(**kwargs):
            stamp=time.monotonic()
            driver._arm.last_feedback_timestamp=stamp
            return JointFeedbackSnapshot(actual.copy(),stamp,stamp,'ideal-test-plant')
        driver._arm.runtime_feedback=snapshot
        def send(position):
            actual[:]=position  # Ideal plant, no hardware or actuator lag.
            sent.append(time.monotonic())
            return True
        driver._send_return_position=send
        def fk(p):
            full=np.zeros(14);full[:7]=(p-zero)/direction
            return driver._trajectory_fk(full)
        lower,upper=driver._effective_position_limits('left')
        guard.begin_entry_certificates()
        plan=StagedReturnPlanner(lower,upper,check,fk,direction,timeout=10.).plan(actual,zero)
        planning_checks=len(costs)
        ok=False
        if plan.path:
            ok=driver._execute_return_group(np.zeros(14),np.arange(7),zero,
                np.deg2rad(cfg.robot.full_arm_return_tolerance_deg),40.,
                handover_path=plan.path)
        result['execution']=dict(success=bool(ok),failure=driver._return_execution_failure,
            plan_reason=plan.reason,path=[p.tolist() for p in plan.path],
            checks=costs,planning_checks=planning_checks,sends=sent)
        guard.end_entry_certificates()
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path.cwd())
    p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repeats',type=int,default=10)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--driver-source',type=Path,help='Test an isolated driver copy; never construct its hardware')
    a=p.parse_args(); result=run(a.root,a.fixture,a.repeats,a.execute,a.driver_source)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in r.items() if k not in ('cycles_ms','checks')} for r in result['results']]))
