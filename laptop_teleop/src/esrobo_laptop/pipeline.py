"""Computation-only retargeting + per-side bounded position IK."""
import copy
import time

import numpy as np

from esrobo_teleop.ik.solver import IkSolver

from .inputs import FreshBodyDevice, InputUnavailable, SourceExpired
from .mapping import WristMapping, hand_units
from .config import ROOT


class Pipeline:
    def __init__(self, cfg, settings, contracts):
        if not settings.hand_only:
            # Load and warm the numerical backend before an arm gateway can be armed.
            from scipy.optimize import least_squares
            least_squares(lambda x: x, np.zeros(1), jac=lambda _x: np.ones((1, 1)), max_nfev=1)

        if (any(c.get("entry_protocol") in (2, 3, 4, 5, 6) for c in contracts.values())
                and not (cfg.ik.partition_terminal_wrist_ik and cfg.ik.enable_elbow_tasks)):
            raise ValueError("safe handover requires complete partitioned endpoint IK")
        self.cfg, self.settings = cfg, settings
        self._entry_limits = {}
        if not settings.hand_only:
            margin = np.deg2rad(cfg.ik.position_limit_hold_feedback_margin_deg)
            for side, contract in contracts.items():
                lower = np.asarray(contract['lower_rad'], dtype=float) + margin
                upper = np.asarray(contract['upper_rad'], dtype=float) - margin
                if not np.isfinite(margin) or margin <= 0 or np.any(lower >= upper):
                    raise ValueError('invalid entry goal clearance inside position envelope')
                self._entry_limits[side] = (lower, upper)
        self.solvers = ({} if settings.hand_only else
                        {s: IkSolver(copy.deepcopy(cfg.ik)) for s in settings.sides})
        for side, solver in self.solvers.items():
            solver.configure_position_envelope(side, contracts[side]["lower_rad"], contracts[side]["upper_rad"])
        retarget = copy.deepcopy(cfg.retarget)
        retarget.arm_vector_position_mode = "segment_direction_absolute"
        retarget.upper_arm_direction_mapping = "absolute"
        retarget.elbow_angle_mapping = "direct_absolute"
        retarget.host, retarget.port = settings.input_host, settings.input_port
        retarget.max_stale_time_s = settings.input_max_age_s
        retarget.use_hand_imu_orientation = False
        retarget.require_hand_imu_for_active_arm = False
        retarget.hand_imu_max_angle_deg = settings.pico_wrist_max_angle_deg
        self.body = FreshBodyDevice(retarget, settings.sides, settings.with_hand, settings.hand_only)
        self.body._reference_diagnostic_path = ROOT / "log" / f"pico_reference_{time.time_ns()}.jsonl"
        self.wrists = {}
        self.hand_references = {}
        self.preparation_status = "等待使能准备"
        self.position_limit_holds = {side: False for side in settings.sides}
        self.position_limit_approaching = {side: False for side in settings.sides}
        self.position_limit_recovery = {side: 0 for side in settings.sides}
        self.initialized = False
        from .visualization import ArmVisualization
        try:
            self._visualization = None if settings.hand_only else ArmVisualization(cfg.ik)
        except Exception:
            self.body.close()
            raise
        self._last_diagnostics_time = 0.0

    def set_diagnostics(self, diagnostics):
        self.diagnostics = diagnostics
        self.body.diagnostic_message_sink = diagnostics.message
        self.body.diagnostic_record_sink = diagnostics.reference
        for solver in self.solvers.values():
            solver.diagnostic_message_sink = diagnostics.message

    def _diagnostic_print(self, *args, **kwargs):
        sink = getattr(self, 'diagnostics', None)
        return (sink.message if sink is not None else print)(*args, **kwargs)

    def restart_preparation(self):
        self.body.restart_preparation()
        self.hand_references.clear()
        self.wrists.clear()
        self.body.wrist_input_diagnostics.clear()
        self.position_limit_holds = {side: False for side in self.settings.sides}
        self.position_limit_approaching = {side: False for side in self.settings.sides}
        self.position_limit_recovery = {side: 0 for side in self.settings.sides}
        self.initialized = False
        if self.settings.hand_only:
            self.preparation_status = "请保持所选侧人手追踪，采集有效初始姿态"
        elif self.settings.with_hand:
            self.preparation_status = "请保持完整 PICO 追踪，采集期望的初始手臂和手部姿态"
        else:
            self.preparation_status = "请保持完整 PICO 追踪，采集期望的初始手臂姿态"
        self._diagnostic_print(f"[遥操作准备] {self.preparation_status}", flush=True)

    def preparation_ready(self):
        with self.body.input_lock:
            body_ready = self.settings.hand_only or self.body.is_ready()
        if not body_ready:
            self.preparation_status = "等待完整有效的 PICO 初始位姿"
            return False
        references, message = self.body.initial_hand_reference()
        if references is None:
            self.preparation_status = message
            return False
        self.hand_references = references
        self.preparation_status = message or "PICO 准备姿态已确认"
        return True

    def initialize(self, measured):
        if self.settings.hand_only:
            self.initialized = True
            return
        measured_poses = next(iter(self.solvers.values())).current_task_frame_poses(measured)
        with self.body.input_lock:
            self.body.rebase_robot_reference(measured_poses)
            flexion = {}
            for side, solver in self.solvers.items():
                flexion[side] = self.body.arm_reference_flexion_rad(side)
                self.body.seed_arm_reference_filters(side)
                if self.settings.with_hand:
                    reference = self.body.wrist_references.get(side)
                    if reference is None or reference.initial_local is None:
                        raise InputUnavailable(f"{side} PICO anatomical palm calibration required")
        for side, solver in self.solvers.items():
            solver.initialize_arm_session(side, measured, flexion[side], relative_elbow_reference=False)
            if self.settings.with_hand:
                self.wrists[side] = WristMapping(
                    solver, self.cfg.robot, side, measured,
                    max_angle_deg=self.settings.pico_wrist_max_angle_deg,
                    absolute_pose=True)
        self.initialized = True

    def reset_command_start(self, measured):
        """No rebase_robot_reference/initialize_arm_session here: mapping stays fixed."""
        position_reference = np.asarray(measured, dtype=float).copy()
        for wrist in self.wrists.values():
            position_reference = wrist.with_reference(position_reference)
        poses = next(iter(self.solvers.values())).current_task_frame_poses(position_reference)
        local = {side: wrist.absolute_local_from_robot(measured) for side, wrist in self.wrists.items()}
        with self.body.input_lock:
            self.body.reset_command_reference(poses)
            for side, wrist in self.wrists.items():
                reference = self.body.wrist_references[side]
                reference.reset_filter(local[side])
                self.body.wrist_rotations[side] = reference.filtered.copy()
        for solver in self.solvers.values():
            solver.reset_command_seed(position_reference)

    def entry_source(self, frame=None):
        """Unfiltered source geometry, independent of frozen IK/limiter targets."""
        side = self.settings.side
        if frame is not None:
            try:
                return list(frame[1]['entry_sources'][side])
            except (KeyError, TypeError):
                raise InputUnavailable('handover requires source geometry from the captured frame') from None
        with self.body.input_lock:
            points = self.body._current_arm_points(self.body._source_side_for_target(side))
            if points is None:
                raise InputUnavailable("handover requires fresh raw arm points")
            # Waist-relative points detect torso/shoulder as well as limb movement.
            values = np.concatenate([points[k] for k in ("shoulder", "elbow", "wrist")])
            return values.tolist()

    def calibrated_entry_source(self):
        with self.body.input_lock:
            return self.body.calibrated_entry_source(self.settings.side)

    def capture_calibrated_entry(self):
        """Use the accepted single-frame PICO reference as the first arm goal."""
        with self.body.input_lock:
            ticket = self.body.ticket(self.settings.input_max_age_s)
            if ticket is None:
                return None
            poses = self.body.calibrated_entry_poses(self.settings.side)
            poses['entry_sources'] = {self.settings.side: self.entry_source()}
            poses["hand_local_orientations"] = {
                side: self.body.wrist_references[side].initial_local.copy() for side in self.wrists}
            hand = self.body.hand_joints() if self.settings.with_hand else None
            if self.settings.with_hand and hand is None:
                raise InputUnavailable("PICO finger tracking missing")
            return ticket, poses, hand

    def capture(self, *, full_target=False):
        return self.body.capture_targets(self.settings.input_max_age_s, full_target=full_target)

    def commit_frame(self, frame, targets):
        self.body.commit_wrist_filters(frame[1], [s for s,t in targets.items() if not t.get('hold')])

    def _entry_goal_inside_limits(self, side, joints):
        """Inset only the locked first goal; never clip measured feedback."""
        limits = self._entry_limits[side]
        offset = 0 if side == 'left' else 7
        count = 5
        selected = slice(offset, offset+count)
        result = np.asarray(joints, dtype=float).copy()
        original = result[selected].copy()
        result[selected] = np.clip(original, limits[0][:count], limits[1][:count])
        delta = np.degrees(result[selected]-original)
        if np.any(abs(delta) > 1e-8):
            changed = ', '.join(f'J{i+1} {delta[i]:+.3f}°' for i in range(count) if abs(delta[i]) > 1e-8)
            self._diagnostic_print(f'[接近目标] {side} 首帧目标保留软限位内侧裕量：{changed}；实时映射不变', flush=True)
        return result

    def solve(self, frame, measured, feedback, *, allow_limit_hold=False,
              hold_entry_wrist_and_hand=False):
        ticket, poses, hands = frame
        if self.settings.hand_only:
            result = {}
            for side in self.settings.sides:
                result[side] = dict(arm_urdf_rad=None,
                                    hand_unit=self.map_hand_target(side, hands, feedback))
            self.check_ticket(ticket)
            return result
        result = {}
        for side, solver in self.solvers.items():
            offset = 0 if side == "left" else 7
            wrist = self.wrists.get(side)
            terminal = (wrist.reference_terminal if wrist else measured[offset+4:offset+7])
            if hold_entry_wrist_and_hand:
                terminal = np.asarray(terminal, dtype=float).copy()
                terminal[1:] = measured[offset+5:offset+7]

            def solve_once(terminal_target):
                q = solver.solve(
                    poses["left_wrist"], poses["right_wrist"], measured,
                    poses["left_elbow"], poses["right_elbow"],
                    partitioned_side=side, terminal_joint_targets=terminal_target,
                    limit_hold_fast_retry=bool(
                        allow_limit_hold and self.position_limit_holds[side]
                    ))

                if np.shape(q) != (14,) or not np.all(np.isfinite(q)):
                    raise InputUnavailable(f"{side} IK rejected: {solver.solution_diagnostics}")
                arm_and_hand_pause = bool(
                    side in ("left", "right") and self.settings.with_hand
                    and solver.solution_diagnostics.get("position_target_unreachable")
                )
                recoverable = bool(
                    allow_limit_hold and len(self.settings.sides) == 1
                    and (arm_and_hand_pause or (
                        not self.settings.with_hand
                        and (solver.solution_diagnostics.get("recoverable_position_limit")
                             or solver.solution_diagnostics.get("position_limit_approach"))))
                    and solver._prev_targets is not None
                    and np.shape(solver._prev_targets) == (14,)
                    and np.all(np.isfinite(solver._prev_targets))
                )
                if not solver.last_solution_valid and not recoverable:
                    raise InputUnavailable(f"{side} IK rejected: {solver.solution_diagnostics}")
                approaching_limit = bool(
                    not solver.last_solution_valid
                    and not arm_and_hand_pause
                    and solver.solution_diagnostics.get("position_limit_approach")
                    and not self.position_limit_holds[side]
                )
                if approaching_limit != self.position_limit_approaching[side]:
                    self.position_limit_approaching[side] = approaching_limit
                    if approaching_limit:
                        self._diagnostic_print(f"[laptop] {side.upper()} LIMIT APPROACH: finishing the last accepted "
                              "joint target before braking at the bound.", flush=True)
                hold = not solver.last_solution_valid and not approaching_limit
                if self.position_limit_holds[side] and solver.last_solution_valid:
                    error = max(
                        float(solver.solution_diagnostics.get("elbow_error_m", float("inf"))),
                        float(solver.solution_diagnostics.get("wrist_error_m", float("inf"))),
                    )
                    self.position_limit_recovery[side] = (
                        self.position_limit_recovery[side] + 1
                        if error <= self.cfg.ik.position_limit_recovery_error_m else 0
                    )
                    hold = self.position_limit_recovery[side] < self.cfg.ik.position_limit_recovery_frames
                elif hold:
                    self.position_limit_recovery[side] = 0
                if hold != self.position_limit_holds[side]:
                    self.position_limit_holds[side] = hold
                    if hold:
                        bounds = solver.solution_diagnostics.get("active_position_bounds", [])
                        label = ", ".join(
                            f"J{item['joint']} {item['bound']} "
                            f"({np.degrees(item['limit_rad']):.1f}deg)"
                            for item in bounds
                        ) or "unreachable arm position"
                        self._diagnostic_print(f"[laptop] {side.upper()} TARGET PAUSED at {label}; "
                              "robot braking/hold active. Move PICO arm back into reachable range.", flush=True)
                    else:
                        self._diagnostic_print(f"[laptop] {side.upper()} target reachable; awaiting control admission.", flush=True)
                return q, hold, approaching_limit

            q, hold, approaching_limit = solve_once(terminal)
            if hold and self.settings.with_hand:
                # This is a diagnostic pose only: send_targets emits a hold
                # packet with no position goal. Do not expose the position
                # IK's calibrated wrist reference as a new wrist command.
                q = np.asarray(measured, dtype=float).copy()
            source_terminal = None
            if wrist and not hold:
                local = poses.get("hand_local_orientations", {}).get(side)
                if local is None:
                    raise InputUnavailable(f"{side} absolute anatomical hand target missing")
                terminal_target = wrist.absolute_terminal(local, q)
                source_local = poses.get('hand_source_orientations', {}).get(side)
                if source_local is not None:
                    source_terminal = (terminal_target.copy() if np.array_equal(source_local, local)
                                       else wrist.source_terminal(source_local, q))
                if hold_entry_wrist_and_hand:
                    # The locked first goal moves J1-J5 only, even with
                    # absolute palm mapping. Live mapping remains full wrist.
                    terminal_target = np.asarray(terminal_target, dtype=float).copy()
                    terminal_target[1:] = measured[offset+5:offset+7]
                wrist.last_diagnostics.update(
                    arm_elbow_error_m=solver.solution_diagnostics.get("elbow_error_m"),
                    arm_endpoint_error_m=solver.solution_diagnostics.get("wrist_error_m"),
                    entry_wrist_held=bool(hold_entry_wrist_and_hand),
                    entry_hand_held=bool(hold_entry_wrist_and_hand),
                    target_joint_deg=np.degrees(terminal_target).tolist(),
                )
                q = q.copy()
                q[offset+4:offset+7] = terminal_target
                # Keep the world attitude in the diagnostic pose explicit;
                # the position component remains the arm carrier endpoint.
                poses[f"{side}_wrist"] = np.asarray(poses[f"{side}_wrist"]).copy()
                poses[f"{side}_wrist"][3:] = wrist.last_diagnostics["desired_world_quat_wxyz"]
            if wrist:
                wrist.last_diagnostics["position_target_paused"] = bool(hold)
            if hold_entry_wrist_and_hand and not hold and len(self.settings.sides) == 1:
                q = self._entry_goal_inside_limits(side, q)
            hand = None
            if self.settings.with_hand:
                if hold_entry_wrist_and_hand or hold:
                    held = feedback[side]["hand"]["position_unit"]
                    if held is None or len(held) != 10 or not np.all(np.isfinite(held)):
                        raise InputUnavailable("fresh held hand feedback required for entry/hold")
                    hand = np.rint(held).astype(int).tolist()
                else:
                    hand = self.map_hand_target(side, hands, feedback)
            result[side] = dict(arm_urdf_rad=q[offset:offset+7].tolist(), hand_unit=hand,
                                hold=hold, limit_approach=approaching_limit)
            if source_terminal is not None:
                source_arm = q[offset:offset+7].copy()
                source_arm[4:] = source_terminal
                result[side]['source_arm_urdf_rad'] = source_arm.tolist()
        self.check_ticket(ticket)
        return result

    def map_hand_target(self, side, hands, feedback):
        """Absolute source target, separate from the held robot finger pose."""
        start = 0 if side == "left" else 10
        return hand_units(hands[start:start+10], feedback[side]["hand"]["position_unit"],
                          self.cfg.hand, side, self.hand_references.get(side), absolute_angles=True)

    def check_ticket(self, ticket):
        now = time.monotonic()
        if any(not 0 <= now-t <= self.settings.input_max_age_s for t in ticket.values()):
            raise SourceExpired("input expired during computation; no target sent")

    def _diagnostic_snapshot(self, frame, targets, measured, *, preview, robot_mode):
        if self.settings.hand_only or not self.solvers or frame[1] is None:
            return None
        computed = np.asarray(measured, dtype=float).copy()
        for side in self.settings.sides:
            offset = 0 if side == 'left' else 7
            arm = targets[side].get('arm_urdf_rad')
            if arm is not None:
                computed[offset:offset+7] = arm
        position_reference = computed.copy()
        for wrist in self.wrists.values():
            position_reference = wrist.with_reference(position_reference)
        return dict(sides=tuple(self.settings.sides),
                    retarget_poses={f'{s}_{part}': np.asarray(frame[1][f'{s}_{part}']).copy()
                                    for s in self.settings.sides for part in ('elbow', 'wrist')},
                    computed=computed, measured=np.asarray(measured, dtype=float).copy(),
                    position_reference=position_reference, preview=bool(preview), robot_mode=robot_mode,
                    timestamp=time.time(), generated_monotonic_s=time.monotonic(),
                    source_sample_monotonic_s=min(frame[0].values()))

    def arm_diagnostics(self, frame, targets, measured, *, preview, robot_mode):
        """Synchronous inspection API; the runtime publisher uses its worker."""
        from .visualization import build_arm_diagnostics
        snapshot = self._diagnostic_snapshot(frame, targets, measured, preview=preview, robot_mode=robot_mode)
        if snapshot is None:
            return None
        return build_arm_diagnostics(self.solvers[self.settings.sides[0]].current_task_frame_poses, **snapshot)

    def publish_diagnostics(self, frame, targets, measured, *, preview, robot_mode):
        now = time.monotonic()
        if self._visualization is None or now-self._last_diagnostics_time < .1:
            return
        try:
            snapshot = self._diagnostic_snapshot(frame, targets, measured, preview=preview, robot_mode=robot_mode)
            if snapshot is not None:
                self._visualization.submit(snapshot)
                self._last_diagnostics_time = now
        except (KeyError, TypeError, ValueError):
            return

    def close(self):
        try:
            self.body.close()
        finally:
            if self._visualization is not None:
                self._visualization.close()
