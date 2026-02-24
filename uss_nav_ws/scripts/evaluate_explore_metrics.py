#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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


def _msg_to_grid_info(msg, timestamp_ns: int) -> Optional[Dict[str, object]]:
    sx = int(msg.size_x)
    sy = int(msg.size_y)
    sz = int(msg.size_z)
    if sx <= 0 or sy <= 0 or sz <= 0:
        return None
    arr = np.asarray(msg.data, dtype=np.uint8)
    if arr.size != sx * sy * sz:
        return None
    grid = arr.reshape((sx, sy, sz))
    resolution = float(msg.resolution)
    topdown_state, occ_height_rel = _extract_topdown_layers(grid, resolution)
    return {
        "timestamp_ns": int(timestamp_ns),
        "resolution": resolution,
        "size_xyz": [sx, sy, sz],
        "origin_xyz": [
            float(msg.origin.position.x),
            float(msg.origin.position.y),
            float(msg.origin.position.z),
        ],
        "data": grid,
        "topdown_state": topdown_state,
        "occ_height_rel": occ_height_rel,
    }


def _read_metrics(bag_dir: Path, metrics_topic: str) -> List[Dict[str, object]]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _build_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if metrics_topic not in topic_types:
        raise RuntimeError(f"topic not found in bag: {metrics_topic}")
    msg_cls = get_message(topic_types[metrics_topic])

    rows: List[Dict[str, object]] = []
    first_stamp_ns = None
    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if topic_name != metrics_topic:
            continue
        if first_stamp_ns is None:
            first_stamp_ns = timestamp_ns
        msg = deserialize_message(serialized_data, msg_cls)
        payload = json.loads(msg.data)
        elapsed = payload.get("elapsed_sec")
        if elapsed is None:
            elapsed = (timestamp_ns - first_stamp_ns) * 1e-9
        row = {
            "elapsed_sec": float(elapsed),
            "coverage_ratio": float(payload.get("coverage_ratio", 0.0)),
            "frontier_count": int(payload.get("frontier_count", 0)),
            "visited_cells": int(payload.get("visited_cells", 0)),
            "tracked_cells": int(payload.get("tracked_cells", 0)),
            "coverage_delta_window": float(payload.get("coverage_delta_window", 0.0)),
            "frontier_delta_window": float(payload.get("frontier_delta_window", 0.0)),
            "done": bool(payload.get("done", False)),
            "done_reason": str(payload.get("done_reason", "")),
        }
        rows.append(row)
    if rows:
        base_elapsed = float(rows[0]["elapsed_sec"])
        for row in rows:
            row["elapsed_sec"] = float(row["elapsed_sec"]) - base_elapsed
    return rows


def _read_last_rolling_grid(bag_dir: Path, grid_topic: str) -> Optional[Dict[str, object]]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _build_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if grid_topic not in topic_types:
        return None
    msg_cls = get_message(topic_types[grid_topic])

    last_msg: Optional[Dict[str, object]] = None
    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if topic_name != grid_topic:
            continue
        msg = deserialize_message(serialized_data, msg_cls)
        info = _msg_to_grid_info(msg, int(timestamp_ns))
        if info is not None:
            last_msg = info
    return last_msg


def _read_rolling_grid_series(
    bag_dir: Path, grid_topic: str, sample_interval_sec: float, max_frames: int
) -> List[Dict[str, object]]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _build_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if grid_topic not in topic_types:
        return []
    msg_cls = get_message(topic_types[grid_topic])

    interval = max(0.01, float(sample_interval_sec))
    first_stamp_ns = None
    last_kept_elapsed = -1e9
    latest_frame: Optional[Dict[str, object]] = None
    frames: List[Dict[str, object]] = []

    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if topic_name != grid_topic:
            continue
        msg = deserialize_message(serialized_data, msg_cls)
        info = _msg_to_grid_info(msg, int(timestamp_ns))
        if info is None:
            continue
        if first_stamp_ns is None:
            first_stamp_ns = int(timestamp_ns)
        elapsed = (int(timestamp_ns) - first_stamp_ns) * 1e-9
        frame = {
            "elapsed_sec": float(elapsed),
            "origin_xyz": list(info["origin_xyz"]),
            "resolution": float(info["resolution"]),
            "size_xyz": list(info["size_xyz"]),
            "topdown_state": np.asarray(info["topdown_state"], dtype=np.uint8),
            "occ_height_rel": np.asarray(info["occ_height_rel"], dtype=np.float32),
        }
        latest_frame = frame
        if not frames or (float(elapsed) - last_kept_elapsed) >= interval:
            frames.append(frame)
            last_kept_elapsed = float(elapsed)

    if latest_frame is not None:
        if not frames or (latest_frame["elapsed_sec"] - frames[-1]["elapsed_sec"]) > 1e-6:
            frames.append(latest_frame)

    if max_frames > 0 and len(frames) > max_frames:
        indices = np.unique(np.linspace(0, len(frames) - 1, max_frames, dtype=np.int32))
        frames = [frames[int(i)] for i in indices.tolist()]
    return frames


def _export_csv(rows: List[Dict[str, object]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(
            fp,
            fieldnames=[
                "elapsed_sec",
                "coverage_ratio",
                "frontier_count",
                "visited_cells",
                "tracked_cells",
                "coverage_delta_window",
                "frontier_delta_window",
                "done",
                "done_reason",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _export_summary(rows: List[Dict[str, object]], summary_path: Path) -> None:
    final = rows[-1]
    done_idx = next((idx for idx, row in enumerate(rows) if bool(row["done"])), None)
    summary = {
        "samples": len(rows),
        "final_elapsed_sec": float(final["elapsed_sec"]),
        "final_coverage_ratio": float(final["coverage_ratio"]),
        "final_frontier_count": int(final["frontier_count"]),
        "done": bool(final["done"]),
        "done_reason": str(final["done_reason"]),
        "done_elapsed_sec": (float(rows[done_idx]["elapsed_sec"]) if done_idx is not None else None),
        "max_coverage_ratio": max(float(r["coverage_ratio"]) for r in rows),
        "min_frontier_count": min(int(r["frontier_count"]) for r in rows),
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def _export_plot(rows: List[Dict[str, object]], png_path: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required for curve export. "
            "Install with: python -m pip install matplotlib"
        ) from exc

    t = [float(r["elapsed_sec"]) for r in rows]
    cov = [float(r["coverage_ratio"]) for r in rows]
    fr = [int(r["frontier_count"]) for r in rows]
    done_idx = next((idx for idx, row in enumerate(rows) if bool(row["done"])), None)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, cov, color="#1f77b4", linewidth=2.0, label="coverage_ratio")
    axes[0].set_ylabel("Coverage Ratio")
    axes[0].set_ylim(0.0, 1.0)
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend(loc="lower right")

    axes[1].plot(t, fr, color="#ff7f0e", linewidth=2.0, label="frontier_count")
    axes[1].set_ylabel("Frontier Count")
    axes[1].set_xlabel("Elapsed Seconds")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend(loc="upper right")

    if done_idx is not None:
        done_t = t[done_idx]
        for ax in axes:
            ax.axvline(done_t, color="#2ca02c", linestyle="--", linewidth=1.5, label="done")
        axes[0].legend(loc="lower right")
        axes[1].legend(loc="upper right")

    fig.suptitle("A-Mode Exploration Metrics")
    fig.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=140)
    plt.close(fig)


def _export_terrain(
    grid_info: Dict[str, object], terrain_png_path: Path, terrain_summary_path: Path
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Patch
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required for terrain map export. "
            "Install with: python -m pip install matplotlib"
        ) from exc

    grid = np.asarray(grid_info["data"], dtype=np.uint8)
    sx, sy, sz = [int(v) for v in grid_info["size_xyz"]]
    resolution = float(grid_info["resolution"])
    origin_x, origin_y, origin_z = [float(v) for v in grid_info["origin_xyz"]]
    extent = [
        origin_x,
        origin_x + sx * resolution,
        origin_y,
        origin_y + sy * resolution,
    ]

    occ_mask = grid == 2
    free_mask = grid == 1
    unknown_mask = grid == 0
    topdown_state = np.asarray(grid_info["topdown_state"], dtype=np.uint8)
    occ_height_rel = np.asarray(grid_info["occ_height_rel"], dtype=np.float32)

    occ_height_abs = np.full((sx, sy), np.nan, dtype=np.float32)
    finite_mask = np.isfinite(occ_height_rel)
    occ_height_abs[finite_mask] = origin_z + occ_height_rel[finite_mask]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    state_cmap = ListedColormap(["#2f2f2f", "#b3e5fc", "#d32f2f"])
    axes[0].imshow(
        topdown_state.T,
        origin="lower",
        interpolation="nearest",
        extent=extent,
        cmap=state_cmap,
        vmin=0,
        vmax=2,
    )
    axes[0].set_title("Top-Down Terrain Occupancy")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].grid(False)
    axes[0].legend(
        handles=[
            Patch(color="#2f2f2f", label="unknown"),
            Patch(color="#b3e5fc", label="free"),
            Patch(color="#d32f2f", label="occupied"),
        ],
        loc="upper right",
    )

    masked_h = np.ma.array(occ_height_rel.T, mask=np.isnan(occ_height_rel.T))
    h_im = axes[1].imshow(
        masked_h,
        origin="lower",
        interpolation="nearest",
        extent=extent,
        cmap="terrain",
    )
    axes[1].set_title("Top-Down Occupied Height Map")
    axes[1].set_xlabel("x (m)")
    axes[1].set_ylabel("y (m)")
    axes[1].grid(False)
    cbar = fig.colorbar(h_im, ax=axes[1], fraction=0.046, pad=0.04)
    cbar.set_label("occupied height above grid origin (m)")

    fig.suptitle("A-Mode Reconstructed Terrain")
    fig.tight_layout()
    terrain_png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(terrain_png_path, dpi=140)
    plt.close(fig)

    total_voxels = int(grid.size)
    unknown_voxels = int(np.count_nonzero(unknown_mask))
    free_voxels = int(np.count_nonzero(free_mask))
    occupied_voxels = int(np.count_nonzero(occ_mask))
    top_total = int(topdown_state.size)
    top_occupied = int(np.count_nonzero(topdown_state == 2))
    top_free = int(np.count_nonzero(topdown_state == 1))
    top_unknown = int(np.count_nonzero(topdown_state == 0))

    terrain_summary = {
        "resolution": resolution,
        "size_xyz_voxels": [sx, sy, sz],
        "origin_xyz": [origin_x, origin_y, origin_z],
        "total_voxels": total_voxels,
        "unknown_voxels": unknown_voxels,
        "free_voxels": free_voxels,
        "occupied_voxels": occupied_voxels,
        "unknown_ratio": (unknown_voxels / total_voxels if total_voxels > 0 else 0.0),
        "free_ratio": (free_voxels / total_voxels if total_voxels > 0 else 0.0),
        "occupied_ratio": (occupied_voxels / total_voxels if total_voxels > 0 else 0.0),
        "topdown_cells": top_total,
        "topdown_unknown_cells": top_unknown,
        "topdown_free_cells": top_free,
        "topdown_occupied_cells": top_occupied,
        "topdown_occupied_ratio": (top_occupied / top_total if top_total > 0 else 0.0),
        "occupied_height_abs_min": (
            float(np.nanmin(occ_height_abs)) if np.any(np.isfinite(occ_height_abs)) else None
        ),
        "occupied_height_abs_max": (
            float(np.nanmax(occ_height_abs)) if np.any(np.isfinite(occ_height_abs)) else None
        ),
    }
    terrain_summary_path.parent.mkdir(parents=True, exist_ok=True)
    terrain_summary_path.write_text(
        json.dumps(terrain_summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _export_terrain_animation(
    frames: List[Dict[str, object]],
    gif_path: Path,
    mp4_path: Path,
    fps: int,
    export_gif: bool,
    export_mp4: bool,
) -> Dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Patch
    except Exception as exc:
        raise RuntimeError(
            "matplotlib animation backend is required for terrain animation export."
        ) from exc

    if not frames:
        return {"gif": "skipped(no frames)", "mp4": "skipped(no frames)"}

    fps = max(1, int(fps))
    h_values: List[np.ndarray] = []
    for frame in frames:
        h = np.asarray(frame["occ_height_rel"], dtype=np.float32)
        valid = h[np.isfinite(h)]
        if valid.size > 0:
            h_values.append(valid)
    if h_values:
        h_concat = np.concatenate(h_values)
        h_min = float(np.min(h_concat))
        h_max = float(np.max(h_concat))
        if abs(h_max - h_min) < 1e-6:
            h_max = h_min + 1.0
    else:
        h_min, h_max = 0.0, 1.0

    first_state = np.asarray(frames[0]["topdown_state"], dtype=np.uint8)
    first_height = np.asarray(frames[0]["occ_height_rel"], dtype=np.float32)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    state_cmap = ListedColormap(["#2f2f2f", "#b3e5fc", "#d32f2f"])
    state_im = axes[0].imshow(
        first_state.T,
        origin="lower",
        interpolation="nearest",
        cmap=state_cmap,
        vmin=0,
        vmax=2,
    )
    _ = state_im
    axes[0].set_title("Top-Down Terrain Occupancy")
    axes[0].set_xlabel("grid x")
    axes[0].set_ylabel("grid y")
    axes[0].grid(False)
    axes[0].legend(
        handles=[
            Patch(color="#2f2f2f", label="unknown"),
            Patch(color="#b3e5fc", label="free"),
            Patch(color="#d32f2f", label="occupied"),
        ],
        loc="upper right",
    )

    masked_h = np.ma.array(first_height.T, mask=np.isnan(first_height.T))
    h_im = axes[1].imshow(
        masked_h,
        origin="lower",
        interpolation="nearest",
        cmap="terrain",
        vmin=h_min,
        vmax=h_max,
    )
    axes[1].set_title("Top-Down Occupied Height Map")
    axes[1].set_xlabel("grid x")
    axes[1].set_ylabel("grid y")
    axes[1].grid(False)
    cbar = fig.colorbar(h_im, ax=axes[1], fraction=0.046, pad=0.04)
    cbar.set_label("occupied height above grid origin (m)")

    timeline_txt = fig.text(0.5, 0.02, "", ha="center", va="bottom", fontsize=10)

    def _update(frame_idx: int):
        frame = frames[frame_idx]
        state = np.asarray(frame["topdown_state"], dtype=np.uint8)
        h_rel = np.asarray(frame["occ_height_rel"], dtype=np.float32)
        state_im.set_data(state.T)
        h_im.set_data(np.ma.array(h_rel.T, mask=np.isnan(h_rel.T)))
        ox, oy, oz = [float(v) for v in frame["origin_xyz"]]
        timeline_txt.set_text(
            f"t={float(frame['elapsed_sec']):.1f}s, "
            f"origin=({ox:.2f}, {oy:.2f}, {oz:.2f}), "
            f"resolution={float(frame['resolution']):.2f}m"
        )
        return [state_im, h_im, timeline_txt]

    anim = FuncAnimation(
        fig,
        _update,
        frames=len(frames),
        interval=max(1, int(1000 / fps)),
        blit=False,
        repeat=True,
    )

    fig.suptitle("A-Mode Terrain Evolution")
    fig.tight_layout(rect=[0, 0.05, 1, 0.97])
    result = {"gif": "skipped", "mp4": "skipped"}
    try:
        if export_gif:
            gif_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                anim.save(gif_path, writer=PillowWriter(fps=fps))
                result["gif"] = "ok"
            except Exception as exc:
                result["gif"] = f"failed({exc})"
        if export_mp4:
            mp4_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                anim.save(mp4_path, writer=FFMpegWriter(fps=fps, bitrate=1800))
                result["mp4"] = "ok"
            except Exception as exc:
                result["mp4"] = f"failed({exc})"
    finally:
        plt.close(fig)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Export exploration curves from rosbag2 metrics topic.")
    parser.add_argument("--bag-dir", required=True, help="Path to rosbag2 directory.")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Directory for exported csv/png/summary. Defaults to <bag-dir>/eval.",
    )
    parser.add_argument(
        "--metrics-topic",
        default="/nav/explore_metrics",
        help="Metrics topic recorded in rosbag2.",
    )
    parser.add_argument(
        "--grid-topic",
        default="/map/rolling_grid",
        help="Rolling grid topic for terrain visualization export.",
    )
    parser.add_argument(
        "--terrain-anim-format",
        default="both",
        choices=["none", "gif", "mp4", "both"],
        help="Animation export format for terrain evolution.",
    )
    parser.add_argument(
        "--terrain-anim-fps",
        type=int,
        default=6,
        help="FPS for terrain evolution animation.",
    )
    parser.add_argument(
        "--terrain-anim-interval-sec",
        type=float,
        default=1.0,
        help="Sampling interval (sec) for terrain animation frames.",
    )
    parser.add_argument(
        "--terrain-anim-max-frames",
        type=int,
        default=240,
        help="Max frame count for terrain animation (uniformly downsampled if exceeded).",
    )
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir).expanduser().resolve()
    if not bag_dir.exists():
        raise FileNotFoundError(f"bag directory not found: {bag_dir}")
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else (bag_dir / "eval")
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = _read_metrics(bag_dir, args.metrics_topic)
    if not rows:
        raise RuntimeError(f"no metrics data found in topic {args.metrics_topic}")

    csv_path = output_dir / "explore_metrics_curve.csv"
    png_path = output_dir / "explore_curves.png"
    summary_path = output_dir / "explore_summary.json"
    terrain_png_path = output_dir / "explore_terrain_map.png"
    terrain_summary_path = output_dir / "explore_terrain_summary.json"
    terrain_anim_gif_path = output_dir / "explore_terrain_evolution.gif"
    terrain_anim_mp4_path = output_dir / "explore_terrain_evolution.mp4"
    _export_csv(rows, csv_path)
    _export_summary(rows, summary_path)
    _export_plot(rows, png_path)

    grid_info = _read_last_rolling_grid(bag_dir, args.grid_topic)
    if grid_info is not None:
        _export_terrain(grid_info, terrain_png_path, terrain_summary_path)
    else:
        print(f"[warn] no terrain data found in topic {args.grid_topic}, skip terrain export")

    anim_result = {"gif": "skipped", "mp4": "skipped"}
    anim_frames = 0
    anim_format = str(args.terrain_anim_format).lower()
    export_gif = anim_format in ("gif", "both")
    export_mp4 = anim_format in ("mp4", "both")
    if anim_format != "none":
        series = _read_rolling_grid_series(
            bag_dir=bag_dir,
            grid_topic=args.grid_topic,
            sample_interval_sec=float(args.terrain_anim_interval_sec),
            max_frames=int(args.terrain_anim_max_frames),
        )
        anim_frames = len(series)
        if series:
            anim_result = _export_terrain_animation(
                frames=series,
                gif_path=terrain_anim_gif_path,
                mp4_path=terrain_anim_mp4_path,
                fps=int(args.terrain_anim_fps),
                export_gif=export_gif,
                export_mp4=export_mp4,
            )
        else:
            print(f"[warn] no terrain frames found in topic {args.grid_topic}, skip terrain animation")

    print(f"[ok] rows: {len(rows)}")
    print(f"[ok] csv: {csv_path}")
    print(f"[ok] png: {png_path}")
    print(f"[ok] summary: {summary_path}")
    if grid_info is not None:
        print(f"[ok] terrain_png: {terrain_png_path}")
        print(f"[ok] terrain_summary: {terrain_summary_path}")
    if anim_format != "none":
        print(f"[ok] terrain_anim_frames: {anim_frames}")
        if export_gif:
            print(f"[ok] terrain_anim_gif: {terrain_anim_gif_path} ({anim_result['gif']})")
        if export_mp4:
            print(f"[ok] terrain_anim_mp4: {terrain_anim_mp4_path} ({anim_result['mp4']})")


if __name__ == "__main__":
    main()
