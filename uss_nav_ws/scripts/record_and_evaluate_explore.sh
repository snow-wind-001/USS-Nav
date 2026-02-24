#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION_SEC=120
USE_RVIZ=false
USE_VLM=false
PARAMS_FILE=""
OUTPUT_ROOT="${WS_DIR}/results/explore_runs"
SKIP_LAUNCH=false
VLM_IMAGE_TOPIC=""
VLM_IMAGE_IS_COMPRESSED=""
VLM_TARGET_TEXT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --use-rviz)
      USE_RVIZ=true
      shift
      ;;
    --use-vlm)
      USE_VLM=true
      shift
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --params-file)
      PARAMS_FILE="$2"
      shift 2
      ;;
    --marsim)
      PARAMS_FILE="${WS_DIR}/src/uss_nav_bringup/config/params_explore_a_marsim.yaml"
      shift
      ;;
    --skip-launch)
      SKIP_LAUNCH=true
      shift
      ;;
    --vlm-image-topic)
      VLM_IMAGE_TOPIC="$2"
      shift 2
      ;;
    --vlm-image-compressed)
      VLM_IMAGE_IS_COMPRESSED="$2"
      shift 2
      ;;
    --vlm-target-text)
      VLM_TARGET_TEXT="$2"
      shift 2
      ;;
    *)
      echo "[error] unknown arg: $1"
      echo "usage: $0 [--duration SEC] [--use-rviz] [--use-vlm] [--params-file PATH] [--marsim] [--output-root DIR] [--skip-launch] [--vlm-image-topic TOPIC] [--vlm-image-compressed true|false] [--vlm-target-text TEXT]"
      exit 1
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 command not found. Please source ROS2 environment first."
  exit 1
fi

if [[ "${CONDA_DEFAULT_ENV:-}" != "TLCForMer-main" ]]; then
  echo "[warn] current conda env is '${CONDA_DEFAULT_ENV:-none}', expected 'TLCForMer-main'."
fi

RUN_ID="explore_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"
BAG_DIR="${RUN_DIR}/bag"
EVAL_DIR="${RUN_DIR}/eval"
mkdir -p "${RUN_DIR}"

LAUNCH_PID=""
BAG_PID=""
STOP_REASON="timeout"

terminate_pid() {
  local pid="$1"
  local name="$2"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done
  echo "[warn] ${name} did not exit after SIGINT, escalating to SIGTERM"
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done
  echo "[warn] ${name} still alive, forcing SIGKILL"
  kill -KILL "${pid}" 2>/dev/null || true
}

cleanup_explore_nodes() {
  pkill -f "/uss_nav_stack/rolling_grid_node" 2>/dev/null || true
  pkill -f "/uss_nav_stack/gcm_node" 2>/dev/null || true
  pkill -f "/uss_nav_stack/frontier_node" 2>/dev/null || true
  pkill -f "/uss_nav_stack/planner_node" 2>/dev/null || true
  pkill -f "/uss_nav_stack/exploration_monitor_node" 2>/dev/null || true
  pkill -f "/uss_nav_stack/vlm_search_node" 2>/dev/null || true
}

cleanup() {
  set +e
  terminate_pid "${BAG_PID}" "rosbag2 recorder"
  terminate_pid "${LAUNCH_PID}" "explore launch"
  cleanup_explore_nodes
}
trap cleanup EXIT

if [[ "${SKIP_LAUNCH}" == "false" ]]; then
  # 避免历史残留节点导致多发布者混入评估数据。
  pkill -f "ros2 launch uss_nav_bringup explore_a.launch.py" 2>/dev/null || true
  cleanup_explore_nodes
  sleep 1

  echo "[info] launching explore_a stack ..."
  launch_cmd=(
    ros2 launch uss_nav_bringup explore_a.launch.py
    use_rviz:="${USE_RVIZ}"
    use_vlm:="${USE_VLM}"
    vlm_image_topic:="/camera/image/compressed"
    vlm_image_is_compressed:=true
  )
  if [[ -n "${PARAMS_FILE}" ]]; then
    launch_cmd+=(params_file:="${PARAMS_FILE}")
  fi
  if [[ "${USE_VLM}" == "true" && -n "${VLM_IMAGE_TOPIC}" ]]; then
    launch_cmd+=(vlm_image_topic:="${VLM_IMAGE_TOPIC}")
  fi
  if [[ "${USE_VLM}" == "true" && -n "${VLM_IMAGE_IS_COMPRESSED}" ]]; then
    launch_cmd+=(vlm_image_is_compressed:="${VLM_IMAGE_IS_COMPRESSED}")
  fi
  if [[ "${USE_VLM}" == "true" && -n "${VLM_TARGET_TEXT}" ]]; then
    launch_cmd+=(vlm_target_text:="${VLM_TARGET_TEXT}")
  fi

  "${launch_cmd[@]}" >"${RUN_DIR}/launch.log" 2>&1 &
  LAUNCH_PID="$!"
  sleep 4
fi

echo "[info] recording rosbag2 to ${BAG_DIR}"
ros2 bag record \
  /map/rolling_grid \
  /nav/frontiers \
  /nav/waypoint \
  /nav/explore_metrics \
  /nav/explore_done \
  /vlm/search_result \
  /nav/target_found \
  /viz/gcm \
  /viz/frontiers \
  /viz/waypoint \
  /viz/explore_status \
  /viz/vlm_status \
  -o "${BAG_DIR}" >"${RUN_DIR}/bag_record.log" 2>&1 &
BAG_PID="$!"

START_TS="$(date +%s)"
while true; do
  NOW_TS="$(date +%s)"
  ELAPSED="$((NOW_TS - START_TS))"
  if (( ELAPSED >= DURATION_SEC )); then
    STOP_REASON="timeout"
    break
  fi

  DONE_VAL="$(
    timeout 2s ros2 topic echo /nav/explore_done --once 2>/dev/null \
      | awk '/data:/{print $2; exit}' \
      || true
  )"
  if [[ "${DONE_VAL}" == "true" ]]; then
    STOP_REASON="explore_done"
    break
  fi
  sleep 2
done

echo "[info] stopping recording, reason=${STOP_REASON}"
terminate_pid "${BAG_PID}" "rosbag2 recorder"
BAG_PID=""

terminate_pid "${LAUNCH_PID}" "explore launch"
LAUNCH_PID=""
cleanup_explore_nodes

echo "[info] evaluating bag ..."
python "${SCRIPT_DIR}/evaluate_explore_metrics.py" --bag-dir "${BAG_DIR}" --output-dir "${EVAL_DIR}" | tee "${RUN_DIR}/eval.log"

echo "[done] run_dir=${RUN_DIR}"
echo "[done] stop_reason=${STOP_REASON}"
