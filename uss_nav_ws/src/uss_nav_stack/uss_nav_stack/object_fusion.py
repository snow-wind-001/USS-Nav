from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np
from uss_nav_interfaces.msg import ObjectGraph, ObjectNode

from .common import ObjectState, safe_norm


@dataclass
class ObjectFusionConfig:
    search_radius: float = 2.0
    geo_tau: float = 0.2
    semantic_threshold: float = 0.75
    geo_semantic_threshold: float = 0.1
    geo_bidirectional_threshold: float = 0.5
    max_points_per_object: int = 256


def geometric_similarity(cloud_a: np.ndarray, cloud_b: np.ndarray, tau: float) -> float:
    if cloud_a.size == 0 or cloud_b.size == 0:
        return 0.0
    dists = np.linalg.norm(cloud_a[:, None, :] - cloud_b[None, :, :], axis=2)
    min_dists = np.min(dists, axis=1)
    return float(np.mean(min_dists <= float(tau)))


def semantic_similarity(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    denom = safe_norm(vec_a) * safe_norm(vec_b)
    return float(np.dot(vec_a, vec_b) / denom)


@dataclass
class ObjectGraphMemory:
    config: ObjectFusionConfig
    objects: List[ObjectState] = field(default_factory=list)
    _next_id: int = 0

    def _downsample(self, points: np.ndarray) -> np.ndarray:
        if len(points) <= int(self.config.max_points_per_object):
            return points.astype(np.float32)
        idx = np.linspace(0, len(points) - 1, int(self.config.max_points_per_object)).astype(np.int32)
        return points[idx].astype(np.float32)

    def _candidates(self, centroid: np.ndarray) -> List[ObjectState]:
        out = []
        for obj in self.objects:
            if float(np.linalg.norm(obj.centroid - centroid)) <= float(self.config.search_radius):
                out.append(obj)
        return out

    def _add_new(self, label: str, clip_vec: np.ndarray, points: np.ndarray, anchor_id: int) -> int:
        obj = ObjectState(
            object_id=self._next_id,
            label=label,
            centroid=points.mean(axis=0).astype(np.float32),
            clip_vec=clip_vec.astype(np.float32),
            points=self._downsample(points),
            anchor_scg_node_id=int(anchor_id),
        )
        self._next_id += 1
        self.objects.append(obj)
        return obj.object_id

    def _merge(self, obj: ObjectState, label: str, clip_vec: np.ndarray, points: np.ndarray, anchor_id: int) -> int:
        obj.label = label
        obj.clip_vec = (0.5 * obj.clip_vec + 0.5 * clip_vec).astype(np.float32)
        merged_points = np.concatenate([obj.points, points], axis=0)
        obj.points = self._downsample(merged_points)
        obj.centroid = obj.points.mean(axis=0).astype(np.float32)
        obj.anchor_scg_node_id = int(anchor_id)
        return obj.object_id

    def update_observation(
        self,
        label: str,
        clip_vec: Sequence[float],
        points_xyz: np.ndarray,
        anchor_scg_node_id: int,
    ) -> int:
        points = np.asarray(points_xyz, dtype=np.float32)
        if points.size == 0:
            return -1
        vec = np.asarray(clip_vec, dtype=np.float32)
        centroid = points.mean(axis=0).astype(np.float32)
        candidates = self._candidates(centroid)
        best_obj = None
        best_score = -1.0
        for obj in candidates:
            sem = semantic_similarity(vec, obj.clip_vec)
            geo_ab = geometric_similarity(points, obj.points, self.config.geo_tau)
            geo_ba = geometric_similarity(obj.points, points, self.config.geo_tau)
            sem_match = sem > self.config.semantic_threshold and geo_ab > self.config.geo_semantic_threshold
            geo_match = geo_ab > self.config.geo_bidirectional_threshold and geo_ba > self.config.geo_bidirectional_threshold
            if not (sem_match or geo_match):
                continue
            score = sem + 0.5 * geo_ab + 0.5 * geo_ba
            if score > best_score:
                best_score = score
                best_obj = obj
        if best_obj is None:
            return self._add_new(label, vec, points, anchor_scg_node_id)
        return self._merge(best_obj, label, vec, points, anchor_scg_node_id)

    def to_msg(self, stamp, frame_id: str) -> ObjectGraph:
        msg = ObjectGraph()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.objects = []
        for obj in self.objects:
            ros_obj = ObjectNode()
            ros_obj.id = int(obj.object_id)
            ros_obj.label = str(obj.label)
            ros_obj.centroid.x = float(obj.centroid[0])
            ros_obj.centroid.y = float(obj.centroid[1])
            ros_obj.centroid.z = float(obj.centroid[2])
            ros_obj.clip_vec = obj.clip_vec.astype(np.float32).tolist()
            ros_obj.anchor_scg_node_id = int(obj.anchor_scg_node_id)
            msg.objects.append(ros_obj)
        return msg
