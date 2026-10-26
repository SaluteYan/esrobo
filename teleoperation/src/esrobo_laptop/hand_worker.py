"""One bounded computation process. The parent remains the sole SDK owner."""
import multiprocessing
import os
import subprocess
import time


def _run(connection, cfg):
    solver = None
    try:
        from .hand_solver import create_solver, contract, selection
        from .pico_hand import PicoHands
        vector = selection(cfg) == 'wrist5'
        solver = create_solver(cfg)
        version, fingerprint = contract(cfg, solver.model.digest)
        # Warm imports/numerical kernels before admitting any live sample.
        points, _ = solver.model.forward(solver.previous)
        solver.solve(None, target=points, source_time=0., budget_s=1., commit=False)
        solver.discard()
        source_stamp = None
        def fit(side, poses):
            options = dict(include_diagnostics=True) if vector else {}
            result = solver.solve(poses, source_time=source_stamp, commit=False, **options)
            result.update(mapping_version=version, mapping_fingerprint=fingerprint)
            return result
        mapper = PicoHands(fit)
        awaiting = None
        connection.send(dict(ready=True, fingerprint=fingerprint, mapping_version=version,
                             compute_backend='dex-wrist5' if vector else getattr(solver, 'backend', 'test')))
        while True:
            request = connection.recv()
            if request is None:
                break
            if request[0] == 'publish':
                _, seq, accepted = request
                if awaiting is not None and awaiting[0] == seq:
                    if accepted:
                        solver.commit(awaiting[1])
                    else:
                        solver.discard()
                    awaiting = None
                continue
            _, seq, sample = request
            solver.discard()
            awaiting = None
            source_stamp = int(sample['received_monotonic_ns'])/1e9
            started = time.monotonic()
            try:
                result = mapper.map_sample('left', sample)
                if time.monotonic()-started > .020:
                    raise TimeoutError('left hand computation exceeded 20 ms')
                if result is not None:
                    awaiting = (seq, result.pop('candidate_id'))
                connection.send(dict(sequence=seq, result=result))
            except Exception as exc:
                solver.discard()
                awaiting = None
                connection.send(dict(sequence=seq, error=f'{type(exc).__name__}: {exc}'))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send(dict(ready=False, error=f'{type(exc).__name__}: {exc}'))
        except (EOFError, BrokenPipeError, OSError):
            pass
    finally:
        if solver is not None and callable(getattr(solver, 'close', None)):
            solver.close()
        connection.close()


class _IsolatedVectorProcess:
    """Same bounded pipe protocol, with dependencies confined to the worker."""
    def __init__(self, child):
        from .config import ROOT, WORKSPACE
        python = ROOT/'external/dex-vector-venv/bin/python'
        if not python.is_file():
            raise RuntimeError('missing vector environment; run install_dex_vector_experiment.sh')
        env = os.environ.copy()
        env['PYTHONPATH'] = os.pathsep.join(str(WORKSPACE/p) for p in (
            'teleoperation/src', 'robot_link'))
        # This worker never loads the PICO/CAN SDK or the parent's Conda libraries.
        env.pop('LD_LIBRARY_PATH', None)
        for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
            env[key] = '1'
        self.process = subprocess.Popen([str(python), '-m', 'esrobo_laptop.hand_worker', str(child.fileno())],
                                        pass_fds=(child.fileno(),), env=env)

    def is_alive(self): return self.process.poll() is None
    def terminate(self): self.process.terminate()
    def kill(self): self.process.kill()
    def join(self, timeout):
        try: self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired: pass
    def close(self): pass


class ProcessHandMapper:
    def __init__(self, cfg):
        from .pico_hand import PicoHands
        self.legacy = PicoHands()
        self.pending = None
        self.awaiting_publication = None
        self.last_submit = 0.
        self.closed = False
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        try:
            from .hand_solver import selection
            if selection(cfg) == 'wrist5':
                self.process = _IsolatedVectorProcess(child)
                child.close()
                self.connection.send(cfg)
            else:
                self.process = context.Process(target=_run, args=(child, cfg), name='left-hand-geometry', daemon=True)
                self.process.start()
                child.close()
            if not self.connection.poll(10.):
                raise RuntimeError('left hand geometry initialization timeout')
            ready = self.connection.recv()
            if not ready.get('ready'):
                raise RuntimeError('left hand geometry initialization failed: '+str(ready.get('error', 'unknown')))
            self.fingerprint = ready['fingerprint']
            self.mapping_version = ready.get('mapping_version')
            from .hand_solver import contract
            if (self.mapping_version, self.fingerprint) != contract(cfg):
                raise RuntimeError('left hand worker mapping contract mismatch')
            self.compute_backend = ready.get('compute_backend')
        except BaseException:
            child.close()
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
        if self.awaiting_publication is not None:
            self.finish('left', self.awaiting_publication, False)
        now = time.monotonic()
        if now-self.last_submit < .020:
            return None
        self.last_submit = now
        self.pending = int(sample['sequence'])
        sent = time.monotonic()
        try:
            self.connection.send(('sample', self.pending, sample))
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
        if reply['result'] is not None:
            self.awaiting_publication = expected
        return reply['result']

    def finish(self, side, sequence, accepted):
        """Called only after actual publication, never after computation alone."""
        if side != 'left' or sequence != self.awaiting_publication:
            return
        self.awaiting_publication = None
        try:
            self.connection.send(('publish', sequence, bool(accepted)))
        except (EOFError, BrokenPipeError, OSError) as exc:
            raise ValueError('left hand publication acknowledgement failed') from exc

    @property
    def next_due(self):
        return self.last_submit+.020

    def close(self):
        if self.closed:
            return
        self.closed = True
        if hasattr(self, 'process') and self.process.is_alive():
            try:
                self.connection.send(None)
            except (EOFError, BrokenPipeError, OSError):
                pass
            # Let the vector worker remove its temporary URDF before forcing exit.
            self.process.join(.25)
        self.connection.close()
        if not hasattr(self, 'process'):
            return
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(1.)
        if self.process.is_alive():
            self.process.kill(); self.process.join(1.)
        self.process.close()


if __name__ == '__main__':
    import sys
    from multiprocessing.connection import Connection
    channel = Connection(int(sys.argv[1]))
    _run(channel, channel.recv())
