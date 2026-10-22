"""Fresh-measurement gate: hold during loss; stable recovery requires a new reference."""
from collections import deque
import math
import numpy as np


def rotation_distance(a, b):
    return 2 * math.acos(float(np.clip(abs(np.dot(a, b)), 0., 1.)))


class TrackingGate:
    def __init__(self, max_age=.65, loss_timeout=30., min_frames=3):
        self.max_age, self.loss_timeout, self.min_frames = max_age, loss_timeout, min_frames
        self.phase, self.note = 'idle', 'Clique Start para calibrar'
        self.sample = None
        self.window = deque()
        self.deadline = 0.
        self.paused_at = None
        self.start_stamp = -1
        self.events = deque(maxlen=12)
        self.latest_reason = 'Mostre a mão direita'

    def _set(self, phase, note, now):
        if phase != self.phase or note != self.note:
            self.events.append(dict(at=now, phase=phase, note=note))
        self.phase, self.note = phase, note

    def observe(self, stamp, capture_at, p, q):
        p,q=np.asarray(p,float),np.asarray(q,float)
        if p.shape!=(3,) or q.shape!=(4,) or not np.all(np.isfinite(np.r_[p,q])) or abs(np.linalg.norm(q)-1.)>.01:
            return
        if self.sample and stamp <= self.sample[0]:
            return
        self.sample = (stamp, capture_at, np.asarray(p).copy(), np.asarray(q).copy())

    def start(self, now):
        # Double clicks and retries must not restart a calibration already in progress.
        if self.phase in ('calibrating', 'active', 'paused'):
            return
        self.window.clear()
        self.start_stamp = self.sample[0] if self.sample else -1
        self.deadline = now + 60.
        self._set('calibrating', 'Start recebido: mantenha a palma parada por 0,4 s', now)

    def stop(self, now):
        self.window.clear()
        self.paused_at = None
        self._set('idle', 'Controle parado', now)

    def pause(self, now, reason):
        if self.phase == 'active':
            self.window.clear()
            self.paused_at = now
            self._set('paused', reason, now)

    def _stable(self, now, duration):
        sample = self.sample
        if sample is None or not 0 <= now - sample[1] <= self.max_age:
            self.window.clear()
            return False
        if sample[0] <= self.start_stamp or (self.window and sample[0] == self.window[-1][0]):
            return False
        if self.window:
            previous = self.window[-1]
            if sample[1] - previous[1] > self.max_age or any(
                    np.linalg.norm(sample[2] - v[2]) > .05 or
                    rotation_distance(sample[3], v[3]) > math.radians(15) for v in self.window):
                self.window.clear()
        self.window.append(sample)
        while len(self.window) > 30:
            self.window.popleft()
        return len(self.window) >= self.min_frames and sample[1] - self.window[0][1] >= duration

    def step(self, now):
        """Return True only on activation/recovery: caller must reanchor to measured TCP."""
        if self.phase == 'active' and (self.sample is None or now - self.sample[1] > self.max_age):
            self.pause(now, 'Pausado: aguardando mão estável; recuperação automática')
        if self.phase == 'calibrating':
            if now > self.deadline:
                self._set('idle', 'Calibração expirou: ' + self.latest_reason, now)
            elif self._stable(now, .4):
                self._set('active', 'ZED controla a Franka simulada', now)
                return True
        elif self.phase == 'paused':
            if now - self.paused_at > self.loss_timeout:
                self._set('idle', 'Sem rastreamento por 30 s; clique Start para rearmar', now)
            elif self._stable(now, .25):
                self._set('active', 'Rastreamento recuperado; referência recalibrada', now)
                return True
        return False

    def progress(self):
        span = self.window[-1][1] - self.window[0][1] if len(self.window) > 1 else 0.
        return dict(frames=len(self.window), required_frames=self.min_frames,
                    stable_ms=round(span * 1000), required_ms=250 if self.phase == 'paused' else 400,
                    reason=self.latest_reason)
