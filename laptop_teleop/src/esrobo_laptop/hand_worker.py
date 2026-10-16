"""One bounded computation process. The parent remains the sole SDK owner."""
import multiprocessing
import time


def _run(connection, cfg):
    try:
        from .hand_retarget import HandRetargeter
        from .pico_hand import PicoHands
        solver = HandRetargeter(cfg)
        # Warm imports/numerical kernels before admitting any live sample.
        points, _ = solver.model.forward(solver.previous)
        solver.solve(None, target=points, budget_s=1.)
        mapper = PicoHands(lambda side, poses: solver.solve(poses))
        connection.send(dict(ready=True, fingerprint=solver.model.digest))
        while True:
            request = connection.recv()
            if request is None:
                break
            seq, sample = request
            started = time.monotonic()
            try:
                result = mapper.map_sample('left', sample)
                if time.monotonic()-started > .020:
                    raise TimeoutError('left hand computation exceeded 20 ms')
                connection.send(dict(sequence=seq, result=result))
            except Exception as exc:
                connection.send(dict(sequence=seq, error=f'{type(exc).__name__}: {exc}'))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class ProcessHandMapper:
    def __init__(self, cfg):
        from .pico_hand import PicoHands
        self.legacy = PicoHands()
        self.pending = None
        self.last_submit = 0.
        self.closed = False
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_run, args=(child, cfg), name='left-hand-geometry', daemon=True)
        self.process.start()
        child.close()
        try:
            if not self.connection.poll(10.):
                raise RuntimeError('left hand geometry initialization timeout')
            ready = self.connection.recv()
            if not ready.get('ready'):
                raise RuntimeError('left hand geometry initialization failed')
            self.fingerprint = ready['fingerprint']
        except BaseException:
            self.close()
            raise

    def map_sample(self, side, sample):
        if side != 'left':
            return self.legacy.map_sample(side, sample)
        if self.closed or not self.process.is_alive():
            raise ValueError('left hand computation process exited; restart PICO')
        # Never queue a second request behind a slow one. Late replies are discarded.
        if self.pending is not None:
            if not self.connection.poll():
                raise ValueError('left hand computation still busy')
            try:
                self.connection.recv()
            except (EOFError, OSError) as exc:
                raise ValueError('left hand computation process failed') from exc
            self.pending = None
        now = time.monotonic()
        if now-self.last_submit < .020:
            return None
        self.last_submit = now
        self.pending = int(sample['sequence'])
        sent = time.monotonic()
        try:
            self.connection.send((self.pending, sample))
            if not self.connection.poll(.030):
                raise ValueError('left hand computation timeout')
            reply = self.connection.recv()
        except (EOFError, BrokenPipeError, OSError) as exc:
            raise ValueError('left hand computation process failed') from exc
        expected, self.pending = self.pending, None
        if reply.get('sequence') != expected or time.monotonic()-sent > .030:
            raise ValueError('left hand late/mismatched result')
        if reply.get('error'):
            raise ValueError(reply['error'])
        return reply['result']

    @property
    def next_due(self):
        return self.last_submit+.020

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.connection.close()
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(1.)
        if self.process.is_alive():
            self.process.kill(); self.process.join(1.)
        self.process.close()
