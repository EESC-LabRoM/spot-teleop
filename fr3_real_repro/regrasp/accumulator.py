"""Rolling tick accumulator: the loop-side always-ready cloud.

While the drill is static and unoccluded the loop stacks live ticks here;
cleaning is the dual2 recipe (2.5 mm dedupe, table cut, 12 mm / 8-nbr
outlier removal) with the same numbers, so a recorded 3-snapshot stack
reproduces the recorded cloud bit-for-bit. Capped in ticks and points;
reset() on a confirmed move, frozen (no add) on occlusion. Pure numpy.
"""
import numpy as np

from contracts import (ACC_MAX_PTS, ACC_MAX_TICKS, DEDUPE_M, OUTLIER_MIN_NBR,
                       OUTLIER_R_M, TABLE_Z)


class Accumulator:
    def __init__(self, max_ticks=ACC_MAX_TICKS, max_pts=ACC_MAX_PTS, seed=0):
        self.max_ticks = max_ticks
        self.max_pts = max_pts
        self._rng = np.random.default_rng(seed)
        self._ticks = []

    def __len__(self):
        return len(self._ticks)

    def add(self, tick):
        pts = np.asarray(tick.pts, float)
        if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 50:
            raise ValueError(f'bad tick cloud {pts.shape}')
        self._ticks.append(pts)
        self._ticks = self._ticks[-self.max_ticks:]

    def reset(self):
        self._ticks = []

    def tick_mean_n(self):
        """Mean points per stacked tick (0 when empty). This is the scale
        the occlusion count gate compares single live ticks against."""
        if not self._ticks:
            return 0
        return round(sum(len(t) for t in self._ticks) / len(self._ticks))

    def cloud(self):
        """Stacked + cleaned cloud (float32 Nx3). Empty array when no ticks."""
        if not self._ticks:
            return np.zeros((0, 3), np.float32)
        P = np.vstack(self._ticks)
        vox = np.floor(P / DEDUPE_M).astype(np.int64)
        _, keep = np.unique(vox, axis=0, return_index=True)
        P = P[np.sort(keep)]
        P = P[P[:, 2] >= TABLE_Z]
        if len(P) == 0:
            return P.astype(np.float32)
        v = np.floor(P / OUTLIER_R_M).astype(np.int64)
        keys, inv = np.unique(v, axis=0, return_inverse=True)
        cnt = np.bincount(inv)
        k2i = {tuple(k): i for i, k in enumerate(keys)}
        off = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
               for dz in (-1, 0, 1)]
        cmap = {k2: cnt[i] for k2, i in k2i.items()}
        nbr = np.zeros(len(P), int)
        for i, k in enumerate(keys):
            kx, ky, kz = tuple(k)
            nbr[inv == i] = sum(cmap.get((kx + dx, ky + dy, kz + dz), 0)
                                for dx, dy, dz in off)
        P = P[nbr >= OUTLIER_MIN_NBR]
        if len(P) > self.max_pts:
            P = P[self._rng.choice(len(P), self.max_pts, replace=False)]
        return P.astype(np.float32)
