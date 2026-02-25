#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def _build_reader(bag_dir: Path):
    import rosbag2_py

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _extract_topdown_layers(grid: np.ndarray, resolution: float) -> Tuple[np.ndarray, np.ndarray]:
    occ_mask = grid == 2
    free_mask = grid == 1

    topdown_state = np.zeros(grid.shape[:2], dtype=np.uint8)
    topdown_state[np.any(free_mask, axis=2)] = 1
    topdown_state[np.any(occ_mask, axis=2)] = 2

    has_occ = np.any(occ_mask, axis=2)
    rev_idx = np.argmax(occ_mask[:, :, ::-1], axis=2)
    max_z_idx = (grid.shape[2] - 1) - rev_idx
    occ_height_rel = np.full(grid.shape[:2], np.nan, dtype=np.float32)
    occ_height_rel[has_occ] = (max_z_idx[has_occ].astype(np.float32) + 0.5) * float(resolution)
    return topdown_state, occ_height_rel


def _read_last_grid(
    bag_dir: Path,
    grid_topic: str,
) -> Tuple[Optional[Dict[str, object]], int]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _build_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if grid_topic not in topic_types:
        return None, 0

    msg_cls = get_message(topic_types[grid_topic])
    last_grid: Optional[Dict[str, object]] = None
    count = 0

    while reader.has_next():
        topic_name, serialized_data, _ = reader.read_next()
        if topic_name != grid_topic:
            continue
        msg = deserialize_message(serialized_data, msg_cls)
        sx = int(msg.size_x)
        sy = int(msg.size_y)
        sz = int(msg.size_z)
        if sx <= 0 or sy <= 0 or sz <= 0:
            continue
        arr = np.asarray(msg.data, dtype=np.uint8)
        if arr.size != sx * sy * sz:
            continue
        grid = arr.reshape((sx, sy, sz))
        resolution = float(msg.resolution)
        topdown_state, occ_height_rel = _extract_topdown_layers(grid, resolution)
        last_grid = {
            "data": grid,
            "topdown_state": topdown_state,
            "occ_height_rel": occ_height_rel,
            "resolution": resolution,
            "size_xyz": [sx, sy, sz],
            "origin_xyz": [
                float(msg.origin.position.x),
                float(msg.origin.position.y),
                float(msg.origin.position.z),
            ],
        }
        count += 1
    return last_grid, count


def _read_trajectory_xy(
    bag_dir: Path,
    odom_topic: str,
) -> Tuple[np.ndarray, int]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _build_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if odom_topic not in topic_types:
        return np.empty((0, 2), dtype=np.float32), 0

    msg_type = topic_types[odom_topic]
    msg_cls = get_message(msg_type)
    points: List[List[float]] = []
    count = 0

    while reader.has_next():
        topic_name, serialized_data, _ = reader.read_next()
        if topic_name != odom_topic:
            continue
        msg = deserialize_message(serialized_data, msg_cls)
        if msg_type == "nav_msgs/msg/Odometry":
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
        elif msg_type == "geometry_msgs/msg/PoseStamped":
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
        else:
            continue
        points.append([x, y])
        count += 1
    if not points:
        return np.empty((0, 2), dtype=np.float32), count
    return np.asarray(points, dtype=np.float32), count


def _path_length_xy(xy: np.ndarray) -> float:
    if len(xy) < 2:
        return 0.0
    diffs = xy[1:] - xy[:-1]
    step = np.linalg.norm(diffs, axis=1)
    return float(np.sum(step))


def _export_figures(
    grid_info: Dict[str, object],
    trajectory_xy: np.ndarray,
    output_dir: Path,
) -> Dict[str, str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    topdown_state = np.asarray(grid_info["topdown_state"], dtype=np.uint8)
    occ_height_rel = np.asarray(grid_info["occ_height_rel"], dtype=np.float32)
    sx, sy, _ = [int(v) for v in grid_info["size_xyz"]]
    resolution = float(grid_info["resolution"])
    origin_x, origin_y, _ = [float(v) for v in grid_info["origin_xyz"]]
    extent = [
        origin_x,
        origin_x + sx * resolution,
        origin_y,
        origin_y + sy * resolution,
    ]

    output_dir.mkdir(parents=True, exist_ok=True)
    topdown_png = output_dir / "bringup_map_topdown.png"
    height_png = output_dir / "bringup_map_height.png"
    traj_png = output_dir / "bringup_trajectory_xy.png"
    overlay_png = output_dir / "bringup_map_trajectory_overlay.png"

    state_cmap = ListedColormap(["#2f2f2f", "#b3e5fc", "#d32f2f"])

    fig1, ax1 = plt.subplots(1, 1, figsize=(8, 7))
    ax1.imshow(
        topdown_state.T,
        origin="lower",
        interpolation="nearest",
        extent=extent,
        cmap=state_cmap,
        vmin=0,
        vmax=2,
    )
    ax1.set_title("Bringup Top-Down Occupancy")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.legend(
        handles=[
            Patch(color="#2f2f2f", label="unknown"),
            Patch(color="#b3e5fc", label="free"),
            Patch(color="#d32f2f", label="occupied"),
        ],
        loc="upper right",
    )
    fig1.tight_layout()
    fig1.savefig(topdown_png, dpi=140)
    plt.close(fig1)

    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 7))
    masked_h = np.ma.array(occ_height_rel.T, mask=np.isnan(occ_height_rel.T))
    h_im = ax2.imshow(
        masked_h,
        origin="lower",
        interpolation="nearest",
        extent=extent,
        cmap="terrain",
    )
    ax2.set_title("Bringup Occupied Height Map")
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    cbar = fig2.colorbar(h_im, ax=ax2, fraction=0.046, pad=0.04)
    cbar.set_label("occupied height above grid origin (m)")
    fig2.tight_layout()
    fig2.savefig(height_png, dpi=140)
    plt.close(fig2)

    fig3, ax3 = plt.subplots(1, 1, figsize=(8, 7))
    if len(trajectory_xy) > 0:
        ax3.plot(trajectory_xy[:, 0], trajectory_xy[:, 1], color="#1565c0", linewidth=1.8, label="trajectory")
        ax3.scatter(trajectory_xy[0, 0], trajectory_xy[0, 1], c="#2e7d32", s=40, label="start")
        ax3.scatter(trajectory_xy[-1, 0], trajectory_xy[-1, 1], c="#c62828", s=40, label="end")
    ax3.set_title("Bringup UAV Trajectory (XY)")
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.axis("equal")
    ax3.grid(True, linestyle="--", alpha=0.4)
    ax3.legend(loc="best")
    fig3.tight_layout()
    fig3.savefig(traj_png, dpi=140)
    plt.close(fig3)

    fig4, ax4 = plt.subplots(1, 1, figsize=(8, 7))
    ax4.imshow(
        topdown_state.T,
        origin="lower",
        interpolation="nearest",
        extent=extent,
        cmap=state_cmap,
        vmin=0,
        vmax=2,
        alpha=0.75,
    )
    if len(trajectory_xy) > 0:
        ax4.plot(trajectory_xy[:, 0], trajectory_xy[:, 1], color="#0d47a1", linewidth=1.8, label="trajectory")
        ax4.scatter(trajectory_xy[0, 0], trajectory_xy[0, 1], c="#2e7d32", s=40, label="start")
        ax4.scatter(trajectory_xy[-1, 0], trajectory_xy[-1, 1], c="#c62828", s=40, label="end")
    ax4.set_title("Bringup Mapping + Trajectory Overlay")
    ax4.set_xlabel("x (m)")
    ax4.set_ylabel("y (m)")
    ax4.legend(loc="best")
    fig4.tight_layout()
    fig4.savefig(overlay_png, dpi=140)
    plt.close(fig4)

    return {
        "topdown_png": str(topdown_png),
        "height_png": str(height_png),
        "trajectory_png": str(traj_png),
        "overlay_png": str(overlay_png),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Export bringup mapping and trajectory results from rosbag2.")
    parser.add_argument("--bag-dir", required=True, help="Path to rosbag2 directory.")
    parser.add_argument("--output-dir", required=True, help="Output directory for plots and summary.")
    parser.add_argument("--grid-topic", default="/map/rolling_grid", help="Rolling grid topic.")
    parser.add_argument("--odom-topic", default="/odom", help="Odometry or pose topic.")
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    if not bag_dir.exists():
        raise FileNotFoundError(f"bag dir not found: {bag_dir}")

    grid_info, grid_count = _read_last_grid(bag_dir, args.grid_topic)
    if grid_info is None:
        raise RuntimeError(f"no valid grid found in topic: {args.grid_topic}")

    trajectory_xy, odom_count = _read_trajectory_xy(bag_dir, args.odom_topic)
    paths = _export_figures(grid_info, trajectory_xy, output_dir)

    grid = np.asarray(grid_info["data"], dtype=np.uint8)
    topdown = np.asarray(grid_info["topdown_state"], dtype=np.uint8)
    summary = {
        "bag_dir": str(bag_dir),
        "grid_topic": args.grid_topic,
        "odom_topic": args.odom_topic,
        "grid_messages": int(grid_count),
        "odom_messages": int(odom_count),
        "trajectory_points": int(len(trajectory_xy)),
        "trajectory_length_xy_m": _path_length_xy(trajectory_xy),
        "resolution": float(grid_info["resolution"]),
        "size_xyz_voxels": [int(v) for v in grid_info["size_xyz"]],
        "origin_xyz": [float(v) for v in grid_info["origin_xyz"]],
        "total_voxels": int(grid.size),
        "occupied_voxels": int(np.count_nonzero(grid == 2)),
        "free_voxels": int(np.count_nonzero(grid == 1)),
        "unknown_voxels": int(np.count_nonzero(grid == 0)),
        "topdown_occupied_cells": int(np.count_nonzero(topdown == 2)),
        "topdown_free_cells": int(np.count_nonzero(topdown == 1)),
        "topdown_unknown_cells": int(np.count_nonzero(topdown == 0)),
        **paths,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "bringup_map_traj_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[ok] topdown_png: {paths['topdown_png']}")
    print(f"[ok] height_png: {paths['height_png']}")
    print(f"[ok] trajectory_png: {paths['trajectory_png']}")
    print(f"[ok] overlay_png: {paths['overlay_png']}")
    print(f"[ok] summary_json: {summary_path}")


if __name__ == "__main__":
    main()
