from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np
from uss_nav_interfaces.msg import SCGEdge, SCGGraph, SCGNode

from .common import EDGE_COLLIDING, EDGE_PARENT, EDGE_VISIBLE, SCGEdgeState, SCGNodeState, normalize
from .rolling_grid import RollingGrid


@dataclass
class SCGConfig:
    fibonacci_samples: int = 50
    max_ray_radius: float = 1.4
    max_seed_extension: float = 2.0
    r_vis: float = 3.0
    max_nodes: int = 400
    min_seed_clearance: float = 0.6


@dataclass
class SpatialConnectivityGraphBuilder:
    config: SCGConfig
    nodes: List[SCGNodeState] = field(default_factory=list)
    edges: List[SCGEdgeState] = field(default_factory=list)
    seed_queue: List[Tuple[np.ndarray, int]] = field(default_factory=list)
    _next_node_id: int = 0

    def _fibonacci_directions(self) -> np.ndarray:
        n = max(int(self.config.fibonacci_samples), 8)
        i = np.arange(n, dtype=np.float32)
        phi = np.pi * (3.0 - np.sqrt(5.0))
        y = 1.0 - 2.0 * (i + 0.5) / n
        r = np.sqrt(np.maximum(1.0 - y * y, 1e-6))
        theta = phi * i
        dirs = np.stack([np.cos(theta) * r, np.sin(theta) * r, y], axis=1)
        return dirs.astype(np.float32)

    def _ray_cast(self, grid: RollingGrid, seed: np.ndarray, direction: np.ndarray) -> Tuple[float, bool]:
        step = max(grid.resolution * 0.8, 0.05)
        max_r = float(self.config.max_ray_radius)
        dist = 0.0
        while dist <= max_r:
            point = seed + direction * dist
            idx = grid.world_to_grid(point)
            if idx is None:
                return dist, True
            if grid.data[idx] == 2:
                return dist, True
            dist += step
        return max_r, False

    def _expand_new_polyhedron(self, grid: RollingGrid, seed: np.ndarray) -> Tuple[float, List[np.ndarray]]:
        dists: List[float] = []
        boundary_dirs: List[np.ndarray] = []
        for direction in self._fibonacci_directions():
            dist, hit = self._ray_cast(grid, seed, direction)
            dists.append(dist)
            if not hit and dist >= 0.9 * float(self.config.max_ray_radius):
                boundary_dirs.append(direction.copy())
        radius = float(np.clip(np.median(np.asarray(dists, dtype=np.float32)), 0.2, self.config.max_ray_radius))
        return radius, boundary_dirs

    def _is_seed_valid(self, grid: RollingGrid, seed: np.ndarray) -> bool:
        idx = grid.world_to_grid(seed)
        if idx is None:
            return False
        if grid.data[idx] == 2:
            return False
        for node in self.nodes:
            if float(np.linalg.norm(seed - node.centroid)) < float(self.config.min_seed_clearance):
                return False
        return True

    def _line_of_sight_free(self, grid: RollingGrid, a: np.ndarray, b: np.ndarray) -> bool:
        seg = b - a
        dist = float(np.linalg.norm(seg))
        if dist < 1e-5:
            return True
        direction = normalize(seg)
        step = max(grid.resolution, 0.1)
        cur = 0.0
        while cur <= dist:
            idx = grid.world_to_grid(a + direction * cur)
            if idx is None:
                return False
            if grid.data[idx] == 2:
                return False
            cur += step
        return True

    def _append_node(self, grid: RollingGrid, seed: np.ndarray, parent_id: int) -> None:
        radius, boundary_dirs = self._expand_new_polyhedron(grid, seed)
        node_id = self._next_node_id
        self._next_node_id += 1
        new_node = SCGNodeState(node_id=node_id, centroid=seed, radius=radius, parent_id=parent_id)
        self.nodes.append(new_node)
        if parent_id >= 0:
            self.edges.append(SCGEdgeState(parent_id, node_id, EDGE_PARENT, 1.0))
        for prev in self.nodes[:-1]:
            dist = float(np.linalg.norm(prev.centroid - seed))
            if dist < 0.8 * (prev.radius + radius):
                self.edges.append(SCGEdgeState(prev.node_id, node_id, EDGE_COLLIDING, 1.0))
            elif dist <= float(self.config.r_vis) and self._line_of_sight_free(grid, prev.centroid, seed):
                self.edges.append(SCGEdgeState(prev.node_id, node_id, EDGE_VISIBLE, 1.0))
        for direction in boundary_dirs:
            cand = seed + direction * float(self.config.max_seed_extension)
            self.seed_queue.append((cand, node_id))

    def update(self, grid: RollingGrid, current_position: Sequence[float], max_steps: int = 2) -> None:
        if len(self.nodes) >= int(self.config.max_nodes):
            return
        cur = np.asarray(current_position, dtype=np.float32)
        if not self.nodes:
            self._append_node(grid, cur, -1)
        if not self.seed_queue:
            for direction in self._fibonacci_directions()[:8]:
                self.seed_queue.append((cur + direction * float(self.config.max_seed_extension), self.nodes[-1].node_id))
        steps = 0
        while self.seed_queue and steps < max_steps and len(self.nodes) < int(self.config.max_nodes):
            seed, parent_id = self.seed_queue.pop(0)
            if not self._is_seed_valid(grid, seed):
                continue
            self._append_node(grid, seed, parent_id)
            steps += 1

    def to_msg(self, stamp, frame_id: str) -> SCGGraph:
        msg = SCGGraph()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.nodes = []
        for node in self.nodes:
            ros_node = SCGNode()
            ros_node.id = int(node.node_id)
            ros_node.centroid.x = float(node.centroid[0])
            ros_node.centroid.y = float(node.centroid[1])
            ros_node.centroid.z = float(node.centroid[2])
            ros_node.radius = float(node.radius)
            msg.nodes.append(ros_node)
        msg.edges = []
        for edge in self.edges:
            ros_edge = SCGEdge()
            ros_edge.from_id = int(edge.from_id)
            ros_edge.to_id = int(edge.to_id)
            ros_edge.edge_type = int(edge.edge_type)
            ros_edge.weight = float(edge.weight)
            msg.edges.append(ros_edge)
        return msg
