from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

# Occupancy encoding (paper-compatible in RollingGrid3D.msg).
UNKNOWN = np.uint8(0)
FREE = np.uint8(1)
OCCUPIED = np.uint8(2)

# SCG edge types.
EDGE_PARENT = 0
EDGE_COLLIDING = 1
EDGE_VISIBLE = 2


@dataclass
class SCGNodeState:
    node_id: int
    centroid: np.ndarray
    radius: float
    parent_id: int = -1


@dataclass
class SCGEdgeState:
    from_id: int
    to_id: int
    edge_type: int
    weight: float


@dataclass
class FrontierState:
    position: np.ndarray
    info_gain: float
    region_id: int = -1


@dataclass
class ObjectState:
    object_id: int
    label: str
    centroid: np.ndarray
    clip_vec: np.ndarray
    points: np.ndarray
    anchor_scg_node_id: int


@dataclass
class PlannerContext:
    frontiers_by_region: Dict[int, List[FrontierState]] = field(default_factory=dict)
    region_object_scores: Dict[int, float] = field(default_factory=dict)
    current_position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    target_label: str = ""


def safe_norm(vec: np.ndarray) -> float:
    return float(np.linalg.norm(vec) + 1e-8)


def normalize(vec: np.ndarray) -> np.ndarray:
    denom = safe_norm(vec)
    return (vec / denom).astype(np.float32)


def as_xyz_tuple(point: np.ndarray) -> Tuple[float, float, float]:
    return float(point[0]), float(point[1]), float(point[2])
