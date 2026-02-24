from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np

from .common import FrontierState, PlannerContext


@dataclass
class PlannerConfig:
    alpha: float = 1.0
    beta: float = 0.5


def choose_target_region(context: PlannerContext) -> int:
    if context.region_object_scores:
        best = max(context.region_object_scores.items(), key=lambda x: x[1])
        if best[1] > 0.0:
            return int(best[0])
    if not context.frontiers_by_region:
        return -1
    gains: Dict[int, float] = {}
    for region_id, frontiers in context.frontiers_by_region.items():
        gains[int(region_id)] = float(sum(f.info_gain for f in frontiers))
    return int(max(gains.items(), key=lambda x: x[1])[0])


def _frontier_cost(current_pos: np.ndarray, frontier: FrontierState, alpha: float, beta: float) -> float:
    dist = float(np.linalg.norm(frontier.position - current_pos))
    return alpha * dist - beta * float(frontier.info_gain)


def order_frontiers_tsp_like(
    frontiers: Sequence[FrontierState],
    start_position: np.ndarray,
    config: PlannerConfig,
) -> List[FrontierState]:
    if not frontiers:
        return []
    remaining = list(frontiers)
    ordered: List[FrontierState] = []
    current = np.asarray(start_position, dtype=np.float32)
    while remaining:
        costs = [
            _frontier_cost(current, frontier, config.alpha, config.beta)
            for frontier in remaining
        ]
        best_idx = int(np.argmin(np.asarray(costs, dtype=np.float32)))
        chosen = remaining.pop(best_idx)
        ordered.append(chosen)
        current = chosen.position
    return ordered
