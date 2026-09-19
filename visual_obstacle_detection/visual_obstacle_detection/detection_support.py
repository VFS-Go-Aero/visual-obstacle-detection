"""Spatial support and short confirmation history for obstacle candidates."""
from itertools import product
import numpy as np


def required_points(distance, base=100, reference=2.0, floor=20):
    """Bound inverse-square scaling so nearby obstacles retain the base count."""
    return np.ceil(np.clip(base * (reference / np.maximum(distance, reference)) ** 2,
                           floor, base)).astype(int)


def coherent_groups(points, cell_size=0.1):
    """Group points in connected occupied 10 cm voxels (26-neighbor adjacency)."""
    cells, inverse = np.unique(np.floor(points / cell_size).astype(np.int64),
                               axis=0, return_inverse=True)
    remaining = {tuple(cell): i for i, cell in enumerate(cells)}
    offsets = list(product((-1, 0, 1), repeat=3))
    while remaining:
        cell, index = remaining.popitem()
        stack = [cell]
        component = [index]
        while stack:
            current = stack.pop()
            for offset in offsets:
                neighbor = tuple(current[k] + offset[k] for k in range(3))
                found = remaining.pop(neighbor, None)
                if found is not None:
                    component.append(found)
                    stack.append(neighbor)
        yield np.flatnonzero(np.isin(inverse, component))


class CandidateConfirmation:
    """Confirm in two of three frames and expire briefly held observations.

    Body-frame matching is not pose-compensated tracking. A retained point
    expires based on its last measurement, never its last publication.
    """
    def __init__(self, tolerance=0.5, max_gap=1.0, hold_time=0.6):
        self.previous = {}
        self.history = []
        self.active = {}
        self.last_time = None
        self.tolerance = tolerance
        self.max_gap = max_gap
        self.hold_time = hold_time

    def update(self, points, sectors, now):
        if self.last_time is None or not 0 < now - self.last_time <= self.max_gap:
            self.history = []
            self.active = {}
        confirmed = np.zeros(len(points), dtype=bool)
        for i, (point, sector) in enumerate(zip(points, sectors)):
            confirmed[i] = any(
                int(sector) in frame and
                np.linalg.norm(point - frame[int(sector)]) <= self.tolerance
                for frame in self.history
            )
        self.previous = {int(s): p.copy() for p, s in zip(points, sectors)}
        self.history = (self.history + [self.previous])[-2:]
        self.last_time = now
        return confirmed

    def stabilize(self, points, sectors, now):
        confirmed = self.update(points, sectors, now)
        self.active = {s: item for s, item in self.active.items()
                       if now - item[1] <= self.hold_time}
        for point, sector, accepted in zip(points, sectors, confirmed):
            sector = int(sector)
            old = self.active.get(sector)
            matches_active = (old is not None and
                              np.linalg.norm(point - old[0]) <= self.tolerance)
            if accepted or matches_active:
                self.active[sector] = (point.copy(), now)
        ordered = sorted(self.active)
        return (np.array([self.active[s][0] for s in ordered], dtype=points.dtype).reshape(-1, 3),
                np.array(ordered, dtype=np.uint32))
