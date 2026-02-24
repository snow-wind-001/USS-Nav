from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from .common import FREE, FrontierState, UNKNOWN
from .gcm import GlobalCoverageMask
from .rolling_grid import RollingGrid


GridIndex = Tuple[int, int, int]


@dataclass
class FrontierConfig:
    min_cluster_size: int = 5


def _neighbors6(idx: GridIndex) -> List[GridIndex]:
    x, y, z = idx
    return [
        (x - 1, y, z),
        (x + 1, y, z),
        (x, y - 1, z),
        (x, y + 1, z),
        (x, y, z - 1),
        (x, y, z + 1),
    ]


def _is_in_bounds(idx: GridIndex, shape: Tuple[int, int, int]) -> bool:
    return 0 <= idx[0] < shape[0] and 0 <= idx[1] < shape[1] and 0 <= idx[2] < shape[2]


def _unknown_neighbors(data: np.ndarray, idx: GridIndex) -> int:
    count = 0
    for nidx in _neighbors6(idx):
        if _is_in_bounds(nidx, data.shape) and data[nidx] == UNKNOWN:
            count += 1
    return count


def _cluster_indices(indices: Set[GridIndex]) -> List[List[GridIndex]]:
    clusters: List[List[GridIndex]] = []
    remaining = set(indices)
    while remaining:
        seed = remaining.pop()
        queue = [seed]
        cluster = [seed]
        while queue:
            cur = queue.pop()
            for nidx in _neighbors6(cur):
                if nidx in remaining:
                    remaining.remove(nidx)
                    queue.append(nidx)
                    cluster.append(nidx)
        clusters.append(cluster)
    return clusters


def extract_frontiers(
    grid: RollingGrid,
    gcm: Optional[GlobalCoverageMask],
    config: FrontierConfig,
) -> List[FrontierState]:
    candidates: Dict[GridIndex, int] = {}
    free_indices = np.argwhere(grid.data == FREE)
    for idx_np in free_indices:
        idx = (int(idx_np[0]), int(idx_np[1]), int(idx_np[2]))
        unknown_n = _unknown_neighbors(grid.data, idx)
        if unknown_n == 0:
            continue
        world = grid.grid_to_world(idx)
        if gcm is not None and gcm.is_visited(world):
            continue
        candidates[idx] = unknown_n

    frontiers: List[FrontierState] = []
    for cluster in _cluster_indices(set(candidates.keys())):
        if len(cluster) < int(config.min_cluster_size):
            continue
        points = np.array([grid.grid_to_world(idx) for idx in cluster], dtype=np.float32)
        centroid = points.mean(axis=0)
        gain = float(np.mean([candidates[idx] for idx in cluster]) * len(cluster))
        frontiers.append(FrontierState(position=centroid, info_gain=gain, region_id=-1))
    return frontiers
