"""Transactional source-continuity gate for bounded reacquisition.

This permits a new target, never a direct joint command. Robot trajectories
remain responsible for speed, acceleration, lead and feedback protection.
"""
import copy
import secrets

import numpy as np


class RecoveryRejected(RuntimeError):
    """Input-only stop: request the commissioned robot-owned latched hold."""


class InputRecovery:
    def __init__(self, settings):
        self.settings = settings
        self.anchor = None
        self.previous = None
        self.count = 0
        self.started = None
        self.epoch = ""
        self.matched = 0
        self.diagnostic = None

    @staticmethod
    def sources(targets):
        return {side: dict(
            arm=copy.deepcopy(t.get('source_arm_urdf_rad', t['arm_urdf_rad'])),
            hand=copy.deepcopy(t.get('hand_unit'))) for side, t in targets.items()}

    def begin(self, targets):
        if self.anchor is None:
            self.anchor = self.sources(targets)
            self.epoch = secrets.token_hex(8)

    def within(self, source, baseline, *, diagnostic_only=False):
        details = {}
        allowed = True
        for side in self.settings.sides:
            item = {}
            if not self.settings.hand_only:
                arm = np.asarray(source[side]['arm'], dtype=float)
                if arm.shape != (7,) or not np.all(np.isfinite(arm)):
                    raise RecoveryRejected(f'{side} recovery requires seven finite arm positions')
                delta = np.degrees(np.abs(arm - baseline[side]['arm']))
                limits = (None if diagnostic_only else
                          [self.settings.input_resume_arm_delta_deg] * 7)
                item.update(arm_delta_deg=delta.tolist(), arm_limit_deg=limits)
                if limits is not None:
                    allowed &= bool(np.all(delta <= np.asarray(limits) + 1e-9))
            if self.settings.with_hand:
                hand = np.asarray(source[side]['hand'], dtype=float)
                if (hand.shape != (10,) or not np.all(np.isfinite(hand))
                        or np.any((hand < 0) | (hand > 255))):
                    raise RecoveryRejected(f'{side} recovery requires ten valid hand positions')
                delta = np.abs(hand - baseline[side]['hand'])
                # Finger source excursion is diagnostic only. Fresh targets
                # are approached by the continuous hand rate limiter.
                item.update(hand_delta_unit=delta.tolist(), hand_limit_unit=None)
            details[side] = item
        return allowed, details

    def evaluate(self, targets, now):
        """Consume one independently ticketed frame, without starting a timer.

        The caller calls this only after input/feedback freshness checks, then
        consumes the ticket even when a candidate is rejected.
        """
        self.check_timeout(now)
        current = self.sources(targets)
        # Keep pre-gap displacement for diagnosis, not recovery eligibility.
        # Actual travel remains bounded by the robot's trajectory and feedback.
        _, details = self.within(current, self.anchor, diagnostic_only=True)
        adjacent = self.previous is None or self.within(current, self.previous)[0]
        held = any(t.get('hold') for t in targets.values())
        self.diagnostic = dict(at=now, anchor=copy.deepcopy(self.anchor), candidate=current,
                               limits=details, adjacent_allowed=bool(adjacent and not held),
                               allowed=bool(adjacent and not held))
        if held:
            raise RecoveryRejected('unreachable target during input reacquisition')
        self.count = self.count + 1 if adjacent else 1
        self.previous = copy.deepcopy(current)
        return self.count >= self.settings.input_resume_frames

    def check_timeout(self, now):
        if self.started is not None and now - self.started >= 3.:
            raise RecoveryRejected('input reacquisition exceeded 3 s; explicit recovery required')

    def commit(self, now):
        if self.started is None:
            self.started = now

    def pause(self):
        # Keep the diagnostic anchor and original deadline across repeated gaps.
        self.previous, self.count, self.matched = None, 0, 0

    def confirm_tracking(self, targets, feedback, enabled_hand_joints):
        # Recovery qualifies a live input stream, not a stationary pose. The
        # old 1.5-degree/2-unit arrival gate chased each new human target and
        # timed out during normal motion or a grasp. Keep these errors visible;
        # actual motion remains bounded by robot trajectories and hand limiting.
        # Called on a copied candidate only after ticket/feedback checks; its
        # counters are committed only after successful transmission.
        errors = {}
        for side in self.settings.sides:
            target, actual = targets[side], feedback[side]
            expected = [not self.settings.hand_only] * 7
            if (actual.get('enable_states') != expected or actual.get('controller_fault')
                    or actual.get('driver_fault')):
                raise RecoveryRejected(f'{side} recovery requires healthy arm feedback and enable state')
            item = {}
            if not self.settings.hand_only:
                source = target.get('source_arm_urdf_rad', target['arm_urdf_rad'])
                item['arm_error_deg'] = np.degrees(np.abs(np.asarray(actual['arm_urdf_rad'])-source)).tolist()
            if self.settings.with_hand:
                delta = np.abs(np.asarray(actual['hand']['position_unit'])-target['hand_unit'])
                item['hand_error_unit'] = delta[enabled_hand_joints].tolist()
            errors[side] = item
        ready = (self.count >= self.settings.input_resume_frames
                 and self.diagnostic is not None and self.diagnostic['adjacent_allowed']
                 and not any(t.get('hold') for t in targets.values()))
        if self.diagnostic is not None:
            self.diagnostic['tracking_error'] = errors
        self.matched = self.matched + 1 if ready else 0
        return self.matched >= 3

    def metadata(self, complete=False):
        return dict(phase='complete' if complete else 'reacquiring', epoch=self.epoch)

    def diagnostics(self):
        return dict(phase='REACQUIRING' if self.started is not None else 'INPUT_WAIT',
                    epoch=self.epoch, started=self.started, candidate_frames=self.count,
                    matched_frames=self.matched, check=self.diagnostic)
