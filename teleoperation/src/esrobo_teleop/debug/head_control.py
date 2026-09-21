"""Bounded, single-operator head adjustment, independent of the ROS executor."""
import threading
import time


class HeadControl:
    def __init__(self, adapter, lease_seconds=3.0):
        self.adapter = adapter
        self.lease_seconds = lease_seconds
        self.mutex = threading.RLock()
        self.command_mutex = threading.Lock()
        self.owner = None
        self.heartbeat_at = 0.0
        self.busy = False
        self.message = '调整已锁定'
        self.cancel = threading.Event()
        self.worker = None
        self.lock_confirmed = True

    def snapshot(self):
        with self.mutex:
            return dict(owner=self.owner, busy=self.busy, message=self.message,
                        lock_confirmed=self.lock_confirmed)

    def heartbeat(self, session):
        with self.mutex:
            if self.owner == session:
                self.heartbeat_at = time.monotonic()

    def expire(self):
        with self.mutex:
            expired = self.owner is not None and time.monotonic() - self.heartbeat_at > self.lease_seconds
        if expired:
            self.lock('页面连接超时，调整已锁定')

    def lock(self, reason='调整已锁定'):
        with self.mutex:
            self.cancel.set()
            self.owner = None
            self.message = reason
            self.lock_confirmed = False
        # Lock sends no position or torque-disable command. Workers check cancel before each jog.
        try:
            with self.command_mutex:
                self.adapter.gate(False)
            with self.mutex:
                self.lock_confirmed = True
        except Exception as exc:
            with self.mutex:
                self.message = f'锁定应答未确认：{exc}'

    def _fresh(self):
        data = self.adapter.state()
        if data['age_s'] is None or data['age_s'] > .4:
            raise RuntimeError('两轴反馈已过期')
        axes = data['axes']
        if any(str(i) not in axes or axes[str(i)].get('valid') != 'true' for i in (1, 2)):
            raise RuntimeError('缺少有效的两轴反馈')
        return axes

    def submit(self, action, session, servo_id=None, target=None):
        if not isinstance(session, str) or not 8 <= len(session) <= 100:
            raise ValueError('无效页面会话')
        if action not in ('enable', 'move'):
            raise ValueError('未知操作')
        if action == 'move':
            if type(servo_id) is not int or servo_id not in (1, 2) or type(target) is not int:
                raise ValueError('目标必须是整数计数，ID 为 1 或 2')
            low, high = (1000, 2700) if servo_id == 1 else (2000, 5000)
            if not low <= target <= high:
                raise ValueError(f'目标超出范围 {low}～{high}')
        with self.mutex:
            if self.busy:
                raise RuntimeError('上一操作未结束，不排队接收目标')
            if self.owner is not None and self.owner != session:
                raise RuntimeError('另一页面正在调整')
            if action == 'move' and self.owner != session:
                raise RuntimeError('请先开启调整')
            self.busy = True
            self.cancel.clear()
            self.owner = session
            self.heartbeat_at = time.monotonic()
            self.message = '正在检查反馈' if action == 'enable' else '正在调整'
            self.worker = threading.Thread(target=self._run, args=(action, servo_id, target), daemon=True)
            self.worker.start()

    def _run(self, action, servo_id, target):
        try:
            axes = self._fresh()
            if any(s.get('pending') == 'true' for s in axes.values()):
                raise RuntimeError('上一目标尚未到位，请检查电机状态')
            if action == 'enable':
                if any(s.get('allow_motion') != 'true' for s in axes.values()):
                    raise RuntimeError('驱动为只读模式，需 allow_motion:=true')
                if self.cancel.is_set():
                    return
                with self.command_mutex:
                    if self.cancel.is_set():
                        return
                    self.adapter.gate(True)
                # A service acknowledgement may precede the next /head/state publication.
                previous = self.adapter.state()['sequence']
                deadline = time.monotonic() + .5
                while not self.cancel.is_set():
                    data = self.adapter.state()
                    if data['sequence'] > previous:
                        axes = self._fresh()
                        if all(s.get('torque_enabled') == '1' and s.get('adjustment_enabled') == 'true'
                               for s in axes.values()):
                            break
                    if time.monotonic() > deadline:
                        raise RuntimeError('开启调整后未收到新鲜的使能反馈')
                    time.sleep(.02)
            else:
                distance = abs(target - int(axes[str(servo_id)]['position_ticks']))
                # Budget long moves by bounded steps; each motor step still has a 5 s watchdog.
                deadline = time.monotonic() + max(120, ((distance + 9) // 10 + 1) * 5 + 10)
                while not self.cancel.is_set():
                    if time.monotonic() > deadline:
                        raise RuntimeError('调整超时')
                    axes = self._fresh()
                    if any(s.get('torque_enabled') != '1' or s.get('adjustment_enabled') != 'true'
                           for s in axes.values()):
                        raise RuntimeError('驱动已锁定或力矩未开启')
                    if any(s.get('pending') == 'true' for s in axes.values()):
                        time.sleep(.05)
                        continue
                    axis = axes[str(servo_id)]
                    tolerance = int(axis.get('arrival_tolerance_ticks', '3'))
                    if not 1 <= tolerance <= 5:
                        raise RuntimeError('无效的到位容差')
                    error = target - int(axis['position_ticks'])
                    if abs(error) <= tolerance:
                        break
                    if self.cancel.is_set():
                        break
                    with self.command_mutex:
                        if self.cancel.is_set():
                            break
                        self.adapter.jog(servo_id, max(-10, min(10, error)), 5)
                    previous = self.adapter.state()['sequence']
                    # Do not issue another step from the snapshot preceding the accepted jog.
                    wait_until = time.monotonic() + .5
                    while self.adapter.state()['sequence'] <= previous and not self.cancel.is_set():
                        if time.monotonic() > wait_until:
                            raise RuntimeError('运动后的反馈未更新')
                        time.sleep(.02)
            with self.mutex:
                if not self.cancel.is_set():
                    self.message = '调整已开启' if action == 'enable' else '已到位，可继续调整'
        except Exception as exc:
            self.lock(str(exc))
        finally:
            # A concurrent lock may arrive while enable is in flight: enforce lock again afterwards.
            if self.cancel.is_set():
                self.lock(self.message)
            with self.mutex:
                self.busy = False

    def close(self):
        if self.owner is not None or self.busy:
            self.lock('服务退出，调整已锁定')
        if self.worker:
            self.worker.join(timeout=4)
