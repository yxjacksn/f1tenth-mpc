import csv
import numpy as np


class WaypointTracker:
    def __init__(self, csv_path: str):
        xs, ys, thetas, vs = [], [], [], []
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row['x']))
                ys.append(float(row['y']))
                thetas.append(float(row['theta']))
                vs.append(float(row['v_ref']))

        self.waypoints = np.column_stack([xs, ys, thetas, vs])
        self.xy = self.waypoints[:, :2].copy()
        self.n = len(self.waypoints)
        self._last_idx = -1

    def find_nearest(self, x, y, theta=None):
        pos = np.array([x, y])

        if self._last_idx < 0 and theta is not None:
            # First call: heading-aware global search
            dists = np.sum((self.xy - pos) ** 2, axis=1)
            candidates = np.where(dists < 25.0)[0]
            if len(candidates) == 0:
                candidates = np.arange(self.n)
            best_idx = candidates[0]
            best_score = 1e9
            for idx in candidates:
                heading_diff = abs(np.arctan2(
                    np.sin(self.waypoints[idx, 2] - theta),
                    np.cos(self.waypoints[idx, 2] - theta)))
                if heading_diff < np.pi / 2:
                    if dists[idx] < best_score:
                        best_score = dists[idx]
                        best_idx = idx
            self._last_idx = best_idx
            return best_idx

        # FORWARD-ONLY search in a small window
        # Only search ahead of current index (with small lookback for correction)
        lookback = 5
        lookfwd = 40  # ~8m at 0.2m spacing — plenty
        start = self._last_idx - lookback
        end = self._last_idx + lookfwd
        idxs = np.arange(start, end) % self.n
        dists = np.sum((self.xy[idxs] - pos) ** 2, axis=1)
        best_local = idxs[np.argmin(dists)]
        self._last_idx = best_local
        return best_local

    def get_reference(self, x, y, theta, v, N, lookahead_idx=3):
        nearest = self.find_nearest(x, y, theta)
        start_idx = (nearest + lookahead_idx) % self.n

        ref = np.zeros((N + 1, 4))
        for k in range(N + 1):
            idx = (start_idx + k) % self.n
            ref[k] = self.waypoints[idx]

        for k in range(N + 1):
            diff = ref[k, 2] - theta
            ref[k, 2] = theta + np.arctan2(np.sin(diff), np.cos(diff))

        return ref, nearest
