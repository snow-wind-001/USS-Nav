from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose
from uss_nav_interfaces.msg import RollingGrid3D

from .common import FREE, OCCUPIED, UNKNOWN


@dataclass
class RollingGridSpec:
    size_xyz: Tuple[float, float, float] = (8.0, 8.0, 4.0)
    resolution: float = 0.1


class RollingGrid:
    def __init__(self, spec: RollingGridSpec) -> None:
        self.spec = spec
        self.size_xyz = np.array(spec.size_xyz, dtype=np.float32)
        self.resolution = float(spec.resolution)
        self.shape = tuple(np.ceil(self.size_xyz / self.resolution).astype(np.int32).tolist())
        self.origin = np.zeros(3, dtype=np.float32)
        self.data = np.full(self.shape, UNKNOWN, dtype=np.uint8)

    def reset_unknown(self) -> None:
        self.data.fill(UNKNOWN)

    def set_origin_centered(self, center_xyz: np.ndarray) -> None:
        center = np.asarray(center_xyz, dtype=np.float32)
        self.origin = center - 0.5 * self.size_xyz

    def world_to_grid(self, point_xyz: np.ndarray) -> Optional[Tuple[int, int, int]]:
        rel = (np.asarray(point_xyz, dtype=np.float32) - self.origin) / self.resolution
        idx = np.floor(rel).astype(np.int32)
        if np.any(idx < 0) or np.any(idx >= np.array(self.shape, dtype=np.int32)):
            return None
        return int(idx[0]), int(idx[1]), int(idx[2])

    def grid_to_world(self, idx_xyz: Tuple[int, int, int]) -> np.ndarray:
        idx = np.array(idx_xyz, dtype=np.float32)
        return self.origin + (idx + 0.5) * self.resolution

    def mark_free_ball(self, center_xyz: np.ndarray, radius: float) -> None:
        center = np.asarray(center_xyz, dtype=np.float32)
        radius_sq = float(radius * radius)
        min_corner = self.world_to_grid(center - radius) or (0, 0, 0)
        max_corner = self.world_to_grid(center + radius)
        if max_corner is None:
            max_corner = tuple((np.array(self.shape, dtype=np.int32) - 1).tolist())
        for ix in range(min_corner[0], max_corner[0] + 1):
            for iy in range(min_corner[1], max_corner[1] + 1):
                for iz in range(min_corner[2], max_corner[2] + 1):
                    p = self.grid_to_world((ix, iy, iz))
                    if float(np.sum((p - center) ** 2)) <= radius_sq:
                        self.data[ix, iy, iz] = FREE

    def mark_points_occupied(self, points_xyz: Iterable[np.ndarray]) -> None:
        for point in points_xyz:
            idx = self.world_to_grid(np.asarray(point, dtype=np.float32))
            if idx is not None:
                self.data[idx] = OCCUPIED

    def mark_free_rays(self, origin_xyz: np.ndarray, points_xyz: Iterable[np.ndarray], step: Optional[float] = None) -> None:
        origin = np.asarray(origin_xyz, dtype=np.float32)
        step_m = float(step) if step is not None else float(self.resolution * 1.5)
        if step_m <= 0.0:
            step_m = float(self.resolution)
        for point in points_xyz:
            p = np.asarray(point, dtype=np.float32)
            vec = p - origin
            dist = float(np.linalg.norm(vec))
            if dist <= self.resolution:
                continue
            direction = vec / dist
            sample_count = int(dist / step_m)
            if sample_count <= 1:
                continue
            for i in range(1, sample_count):
                sample = origin + direction * (i * step_m)
                idx = self.world_to_grid(sample)
                if idx is None:
                    continue
                if self.data[idx] != OCCUPIED:
                    self.data[idx] = FREE

    def as_msg(self, pose_center: np.ndarray, stamp, frame_id: str) -> RollingGrid3D:
        msg = RollingGrid3D()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.origin = Pose()
        msg.origin.position.x = float(self.origin[0])
        msg.origin.position.y = float(self.origin[1])
        msg.origin.position.z = float(self.origin[2])
        msg.origin.orientation.w = 1.0
        msg.resolution = float(self.resolution)
        msg.size_x = int(self.shape[0])
        msg.size_y = int(self.shape[1])
        msg.size_z = int(self.shape[2])
        msg.data = self.data.reshape(-1).tolist()
        return msg

    @classmethod
    def from_msg(cls, msg: RollingGrid3D) -> "RollingGrid":
        spec = RollingGridSpec(
            size_xyz=(
                float(msg.size_x) * float(msg.resolution),
                float(msg.size_y) * float(msg.resolution),
                float(msg.size_z) * float(msg.resolution),
            ),
            resolution=float(msg.resolution),
        )
        grid = cls(spec)
        grid.origin = np.array(
            [msg.origin.position.x, msg.origin.position.y, msg.origin.position.z],
            dtype=np.float32,
        )
        arr = np.asarray(msg.data, dtype=np.uint8)
        grid.data = arr.reshape((msg.size_x, msg.size_y, msg.size_z))
        return grid
