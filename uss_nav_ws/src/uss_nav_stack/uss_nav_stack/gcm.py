from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

from .common import UNKNOWN
from .rolling_grid import RollingGrid


CoarseIndex = Tuple[int, int, int]


@dataclass
class GCMConfig:
    cell_size: float = 1.0
    visited_unknown_threshold: int = 50


@dataclass
class GlobalCoverageMask:
    config: GCMConfig
    unknown_counts: Dict[CoarseIndex, int] = field(default_factory=dict)
    visited: Dict[CoarseIndex, bool] = field(default_factory=dict)

    def _coarse_index(self, point_xyz: np.ndarray) -> CoarseIndex:
        idx = np.floor(point_xyz / float(self.config.cell_size)).astype(np.int32)
        return int(idx[0]), int(idx[1]), int(idx[2])

    def update_from_grid(self, grid: RollingGrid) -> None:
        local_counts: Dict[CoarseIndex, int] = {}
        unknown_indices = np.argwhere(grid.data == UNKNOWN)
        for idx_xyz in unknown_indices:
            world = grid.grid_to_world((int(idx_xyz[0]), int(idx_xyz[1]), int(idx_xyz[2])))
            cidx = self._coarse_index(world)
            local_counts[cidx] = local_counts.get(cidx, 0) + 1

        for cidx, local_unknown in local_counts.items():
            prev = self.unknown_counts.get(cidx, local_unknown)
            merged = min(prev, local_unknown)
            self.unknown_counts[cidx] = merged
            self.visited[cidx] = merged < int(self.config.visited_unknown_threshold)

    def is_visited(self, point_xyz: np.ndarray) -> bool:
        cidx = self._coarse_index(point_xyz)
        return self.visited.get(cidx, False)

    def to_json_msg(self) -> str:
        payload = {
            "cell_size": float(self.config.cell_size),
            "visited_unknown_threshold": int(self.config.visited_unknown_threshold),
            "unknown_counts": {"|".join(map(str, k)): int(v) for k, v in self.unknown_counts.items()},
            "visited": {"|".join(map(str, k)): bool(v) for k, v in self.visited.items()},
        }
        return json.dumps(payload, ensure_ascii=True)

    @classmethod
    def from_json_msg(cls, text: str) -> "GlobalCoverageMask":
        raw = json.loads(text)
        config = GCMConfig(
            cell_size=float(raw.get("cell_size", 1.0)),
            visited_unknown_threshold=int(raw.get("visited_unknown_threshold", 50)),
        )
        gcm = cls(config=config)
        for key, value in raw.get("unknown_counts", {}).items():
            parts = tuple(int(x) for x in key.split("|"))
            gcm.unknown_counts[parts] = int(value)
        for key, value in raw.get("visited", {}).items():
            parts = tuple(int(x) for x in key.split("|"))
            gcm.visited[parts] = bool(value)
        return gcm
