"""Latest-only viewer work with an independent model and no control-state access."""
import json
import socket
import threading

import numpy as np


class DisplayFK:
    def __init__(self, cfg):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(cfg.urdf_path)
        self.data = self.model.createData()
        names = cfg.left_arm_joints + cfg.right_arm_joints
        ids = [self.model.getJointId(name) for name in names]
        if any(i == 0 or i >= self.model.njoints for i in ids):
            raise ValueError('viewer model is missing arm joints')
        self.indices = np.array([self.model.idx_qs[i] for i in ids])
        self.frames = {f'{side}_{part}': self.model.getFrameId(getattr(cfg, f'{side}_{attr}'))
                       for side in ('left', 'right')
                       for part, attr in (('shoulder', 'shoulder_frame'), ('elbow', 'elbow_frame'),
                                          ('wrist', 'hand_frame'))}
        if any(i >= self.model.nframes for i in self.frames.values()):
            raise ValueError('viewer model is missing task frames')

    def __call__(self, joints):
        q = np.zeros(self.model.nq)
        q[self.indices] = np.clip(joints, self.model.lowerPositionLimit[self.indices],
                                 self.model.upperPositionLimit[self.indices])
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return {name: self.data.oMf[i].translation.copy() for name, i in self.frames.items()}


def build_arm_diagnostics(fk, *, sides, retarget_poses, computed, measured,
                          position_reference, preview, robot_mode, timestamp,
                          generated_monotonic_s, source_sample_monotonic_s):
    focus = sides[0]
    ik_poses = fk(computed)
    feedback_poses = fk(measured)
    position_poses = fk(position_reference)
    def frames_from(poses):
        return {
            name: {"pos": np.asarray(pose[:3], dtype=float).tolist()}
            for name, pose in poses.items()
            if any(name.startswith(f"{side}_") for side in sides)
        }

    target_frames = {}
    for side in sides:
        # Show the physical hand target, including the downstream wrist
        # offset. A bent wrist must not appear as arm IK position error.
        wrist_offset = ik_poses[f"{side}_wrist"][:3] - position_poses[f"{side}_wrist"][:3]
        target_frames.update({
            f"{side}_shoulder": {"pos": np.asarray(feedback_poses[f"{side}_shoulder"][:3], dtype=float).tolist()},
            f"{side}_elbow": {"pos": np.asarray(retarget_poses[f"{side}_elbow"][:3], dtype=float).tolist()},
            f"{side}_wrist": {"pos": (np.asarray(retarget_poses[f"{side}_wrist"][:3], dtype=float)
                                     + wrist_offset).tolist()},
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
        timestamp=timestamp, generated_monotonic_s=generated_monotonic_s,
        source_sample_monotonic_s=source_sample_monotonic_s, side=focus, selected_sides=list(sides),
        preview=bool(preview), robot_mode=robot_mode,
        frames=dict(retarget=target_frames, ik=frames_from(ik_poses),
                    command=frames_from(ik_poses), feedback=frames_from(feedback_poses),
                    arm_position_reference=frames_from(position_poses)),
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


class ArmVisualization:
    """One owned snapshot pending; a slow FK or socket cannot queue history."""
    def __init__(self, cfg=None, *, fk=None, sender=None):
        self.fk = fk if fk is not None else DisplayFK(cfg)
        self.socket = None if sender is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.socket is not None:
            self.socket.setblocking(False)
        self.sender = sender if sender is not None else lambda data: self.socket.sendto(data, ('127.0.0.1', 15060))
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.coalesced = 0
        self.errors = 0
        self.last_error = None
        self.thread = threading.Thread(target=self._run, name='arm-viewer', daemon=True)
        self.thread.start()

    def submit(self, snapshot):
        # Producer hands over independent arrays/scalars, never a live solver.
        with self.condition:
            if self.closed:
                return
            if self.pending is not None:
                self.coalesced += 1
            self.pending = snapshot
            self.condition.notify()

    def stats(self):
        with self.condition:
            return dict(pending=int(self.pending is not None), coalesced=self.coalesced,
                        errors=self.errors, last_error=self.last_error)

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.closed or self.pending is not None)
                    if self.closed:
                        return
                    snapshot, self.pending = self.pending, None
                try:
                    message = build_arm_diagnostics(self.fk, **snapshot)
                    encoded = json.dumps(message, separators=(',', ':'), allow_nan=False).encode()
                    with self.condition:
                        if self.closed:
                            return
                    self.sender(encoded)
                except Exception as exc:
                    with self.condition:
                        self.errors += 1
                        self.last_error = str(exc)[:200]
        finally:
            if self.socket is not None:
                self.socket.close()

    def close(self, timeout=.2):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout)
        return not self.thread.is_alive()
