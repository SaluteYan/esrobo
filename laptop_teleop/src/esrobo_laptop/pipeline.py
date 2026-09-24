"""Computation-only retargeting + per-side bounded position IK."""
import copy
import json
import socket
import time

import numpy as np

from esrobo_teleop.ik.solver import IkSolver

from .inputs import FreshBodyDevice, InputUnavailable
from .mapping import WristMapping, hand_units
from .config import ROOT


class Pipeline:
    def __init__(self, cfg, settings, contracts):
        if not settings.hand_only:
            # Load and warm the numerical backend before an arm gateway can be armed.
            from scipy.optimize import least_squares
            least_squares(lambda x: x, np.zeros(1), jac=lambda _x: np.ones((1, 1)), max_nfev=1)

        self.cfg, self.settings = cfg, settings
        self.solvers = ({} if settings.hand_only else
                        {s: IkSolver(copy.deepcopy(cfg.ik)) for s in settings.sides})
        for side, solver in self.solvers.items():
            solver.configure_position_envelope(side, contracts[side]["lower_rad"], contracts[side]["upper_rad"])
        retarget = copy.deepcopy(cfg.retarget)
        retarget.host, retarget.port = settings.input_host, settings.input_port
        retarget.max_stale_time_s = settings.input_max_age_s
        retarget.use_hand_imu_orientation = False
        retarget.require_hand_imu_for_active_arm = False
        self.body = FreshBodyDevice(retarget, settings.sides, settings.with_hand, settings.hand_only)
        self.body._reference_diagnostic_path = ROOT / "log" / f"pico_reference_{time.time_ns()}.jsonl"
        self.wrists = {}
        self.hand_references = {}
        self.preparation_status = "等待使能准备"
        self.position_solutions = {}
        self.initialized = False
        self._diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._last_diagnostics_time = 0.0

    def restart_preparation(self):
        self.body.restart_preparation()
        self.hand_references.clear()
        self.wrists.clear()
        self.position_solutions.clear()
        self.initialized = False
        if self.settings.hand_only:
            self.preparation_status = "请自然张开所选侧人手并保持稳定"
        elif self.settings.with_hand:
            self.preparation_status = "请把手臂抬到胸前、肘部自然弯曲，并自然张开双手"
        else:
            self.preparation_status = "请把手臂抬到胸前、肘部自然弯曲并保持稳定"
        print(f"[遥操作准备] {self.preparation_status}", flush=True)

    def preparation_ready(self):
        if not self.settings.hand_only and not self.body.is_ready():
            self.preparation_status = "请把手臂抬到胸前、肘部自然弯曲并保持不动"
            return False
        references, message = self.body.natural_open_hand_reference()
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
        with self.body.input_lock:
            self.body.rebase_robot_reference(next(iter(self.solvers.values())).current_task_frame_poses(measured))
            for side, solver in self.solvers.items():
                solver.initialize_arm_session(
                    side, measured, self.body.arm_reference_flexion_rad(side),
                    relative_elbow_reference=self.cfg.retarget.arm_vector_position_mode == "segment_direction_relative")
                self.body.seed_arm_reference_filters(side)
                if self.settings.with_hand:
                    self.wrists[side] = WristMapping(solver, self.cfg.robot, side, measured)
                    # The calibrated pose is the first position solution.  On
                    # later frames J1-J4 from the last accepted solve provide
                    # the orientation-compensation baseline, avoiding a
                    # second IK solve in every control cycle.
                    self.position_solutions[side] = np.asarray(measured, dtype=float).copy()
        self.initialized = True

    def capture(self):
        with self.body.input_lock:
            ticket = self.body.ticket(self.settings.input_max_age_s)
            if ticket is None:
                return None
            if self.settings.hand_only:
                hand = self.body.hand_joints()
                if hand is None:
                    raise InputUnavailable("PICO finger tracking missing")
                return ticket, None, hand
            poses = self.body.advance()
            if poses is None:
                raise InputUnavailable("retargeting not ready")
            hand = self.body.hand_joints() if self.settings.with_hand else None
            if self.settings.with_hand and hand is None:
                raise InputUnavailable("PICO finger tracking missing")
            return ticket, poses, hand

    def solve(self, frame, measured, feedback):
        ticket, poses, hands = frame
        if self.settings.hand_only:
            result = {}
            for side in self.settings.sides:
                start = 0 if side == "left" else 10
                result[side] = dict(
                    arm_urdf_rad=None,
                    hand_unit=hand_units(
                        hands[start:start+10], feedback[side]["hand"]["position_unit"],
                        self.cfg.hand, side, self.hand_references.get(side),
                    ),
                )
            self.check_ticket(ticket)
            return result
        result = {}
        for side, solver in self.solvers.items():
            offset = 0 if side == "left" else 7
            wrist = self.wrists.get(side)
            terminal = (
                wrist.compensated(
                    poses[f"{side}_wrist"],
                    self.position_solutions.get(side, measured),
                )
                if wrist else measured[offset+4:offset+7]
            )

            def solve_once(terminal_target):
                q = solver.solve(
                    poses["left_wrist"], poses["right_wrist"], measured,
                    poses["left_elbow"], poses["right_elbow"],
                    partitioned_side=side, terminal_joint_targets=terminal_target)
                if not solver.last_solution_valid or not np.all(np.isfinite(q)):
                    raise InputUnavailable(f"{side} IK rejected: {solver.solution_diagnostics}")
                return q

            q = solve_once(terminal)
            if wrist:
                self.position_solutions[side] = q.copy()
            hand = None
            if self.settings.with_hand:
                start = 0 if side == "left" else 10
                hand = hand_units(hands[start:start+10], feedback[side]["hand"]["position_unit"],
                                  self.cfg.hand, side, self.hand_references.get(side))
            result[side] = dict(arm_urdf_rad=q[offset:offset+7].tolist(), hand_unit=hand)
        self.check_ticket(ticket)
        return result

    def check_ticket(self, ticket):
        now = time.monotonic()
        if any(not 0 <= now-t <= self.settings.input_max_age_s for t in ticket.values()):
            raise InputUnavailable("input expired during computation; no target sent")

    def arm_diagnostics(self, frame, targets, measured, *, preview, robot_mode):
        """Build the viewer layers produced by the original local teleop node."""
        if self.settings.hand_only or not self.solvers or frame[1] is None:
            return None
        _, retarget_poses, _ = frame
        focus = self.settings.sides[0]
        solver = self.solvers[focus]
        computed = np.asarray(measured, dtype=float).copy()
        for side in self.settings.sides:
            offset = 0 if side == "left" else 7
            arm = targets[side].get("arm_urdf_rad")
            if arm is not None:
                computed[offset:offset+7] = np.asarray(arm, dtype=float)
        ik_poses = solver.current_task_frame_poses(computed)
        feedback_poses = solver.current_task_frame_poses(measured)

        def frames_from(poses):
            return {
                name: {"pos": np.asarray(pose[:3], dtype=float).tolist()}
                for name, pose in poses.items()
                if any(name.startswith(f"{side}_") for side in self.settings.sides)
            }

        target_frames = {}
        for side in self.settings.sides:
            target_frames.update({
                f"{side}_shoulder": {"pos": np.asarray(feedback_poses[f"{side}_shoulder"][:3], dtype=float).tolist()},
                f"{side}_elbow": {"pos": np.asarray(retarget_poses[f"{side}_elbow"][:3], dtype=float).tolist()},
                f"{side}_wrist": {"pos": np.asarray(retarget_poses[f"{side}_wrist"][:3], dtype=float).tolist()},
            })

        def position_errors(side, reference, actual):
            shoulder = np.asarray(reference[f"{side}_shoulder"]["pos"], dtype=float)
            ref_elbow = np.asarray(reference[f"{side}_elbow"]["pos"], dtype=float)
            ref_wrist = np.asarray(reference[f"{side}_wrist"]["pos"], dtype=float)
            act_elbow = np.asarray(actual[f"{side}_elbow"][:3], dtype=float)
            act_wrist = np.asarray(actual[f"{side}_wrist"][:3], dtype=float)

            def direction_error(a, b):
                lengths = np.linalg.norm(a), np.linalg.norm(b)
                if min(lengths) <= 1e-9:
                    return None
                return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / (lengths[0]*lengths[1]), -1., 1.))))

            return dict(
                elbow_cm=float(100*np.linalg.norm(act_elbow-ref_elbow)),
                wrist_cm=float(100*np.linalg.norm(act_wrist-ref_wrist)),
                upper_deg=direction_error(ref_elbow-shoulder, act_elbow-shoulder),
                forearm_deg=direction_error(ref_wrist-ref_elbow, act_wrist-act_elbow),
            )

        offset = 0 if focus == "left" else 7
        return dict(
            timestamp=time.time(), side=focus, selected_sides=list(self.settings.sides),
            preview=bool(preview), robot_mode=robot_mode,
            frames=dict(retarget=target_frames, ik=frames_from(ik_poses),
                        command=frames_from(ik_poses), feedback=frames_from(feedback_poses)),
            joints_deg=dict(
                ik=np.degrees(computed[offset:offset+7]).tolist(),
                command=np.degrees(computed[offset:offset+7]).tolist(),
                feedback=np.degrees(np.asarray(measured)[offset:offset+7]).tolist(),
            ),
            errors=dict(
                ik_vs_retarget=position_errors(focus, target_frames, ik_poses),
                command_vs_retarget=position_errors(focus, target_frames, ik_poses),
                feedback_vs_retarget=position_errors(focus, target_frames, feedback_poses),
            ),
        )

    def publish_diagnostics(self, frame, targets, measured, *, preview, robot_mode):
        now = time.monotonic()
        if now-self._last_diagnostics_time < .1:
            return
        try:
            message = self.arm_diagnostics(frame, targets, measured, preview=preview, robot_mode=robot_mode)
            if message is None:
                return
            payload = json.dumps(message, separators=(",", ":"), allow_nan=False).encode()
            self._diagnostics_socket.sendto(payload, ("127.0.0.1", 15060))
            self._last_diagnostics_time = now
        except (KeyError, TypeError, ValueError, OSError):
            # Optional visualization must never interrupt the control loop.
            return

    def close(self):
        try:
            self.body.close()
        finally:
            self._diagnostics_socket.close()
