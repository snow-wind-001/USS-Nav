#!/usr/bin/env bash
set -euo pipefail

ODOM_TOPIC="/odom"
POINTS_TOPIC="/points"
IMAGE_TOPIC="/camera/image/compressed"
CHECK_IMAGE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --odom-topic)
      ODOM_TOPIC="$2"
      shift 2
      ;;
    --points-topic)
      POINTS_TOPIC="$2"
      shift 2
      ;;
    --image-topic)
      IMAGE_TOPIC="$2"
      shift 2
      ;;
    --check-image)
      CHECK_IMAGE=true
      shift
      ;;
    *)
      echo "[error] unknown arg: $1"
      echo "usage: $0 [--odom-topic TOPIC] [--points-topic TOPIC] [--image-topic TOPIC] [--check-image]"
      exit 1
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 not found. Please source ROS2 env first."
  exit 1
fi

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | awk -v t="${topic}" '$0==t{found=1} END{exit(found?0:1)}'
}

topic_type() {
  local topic="$1"
  ros2 topic type "${topic}" 2>/dev/null | awk 'NR==1{print $1}'
}

OK=true

if topic_exists "${ODOM_TOPIC}"; then
  ODOM_TYPE="$(topic_type "${ODOM_TOPIC}")"
  if [[ "${ODOM_TYPE}" == "nav_msgs/msg/Odometry" || "${ODOM_TYPE}" == "geometry_msgs/msg/PoseStamped" ]]; then
    echo "[ok] ${ODOM_TOPIC}: ${ODOM_TYPE}"
  else
    echo "[error] ${ODOM_TOPIC} type mismatch: ${ODOM_TYPE} (expect nav_msgs/msg/Odometry or geometry_msgs/msg/PoseStamped)"
    OK=false
  fi
else
  echo "[error] topic missing: ${ODOM_TOPIC}"
  OK=false
fi

if topic_exists "${POINTS_TOPIC}"; then
  POINTS_TYPE="$(topic_type "${POINTS_TOPIC}")"
  if [[ "${POINTS_TYPE}" == "sensor_msgs/msg/PointCloud2" ]]; then
    echo "[ok] ${POINTS_TOPIC}: ${POINTS_TYPE}"
  else
    echo "[error] ${POINTS_TOPIC} type mismatch: ${POINTS_TYPE} (expect sensor_msgs/msg/PointCloud2)"
    OK=false
  fi
else
  echo "[error] topic missing: ${POINTS_TOPIC}"
  OK=false
fi

if [[ "${CHECK_IMAGE}" == "true" ]]; then
  if topic_exists "${IMAGE_TOPIC}"; then
    IMAGE_TYPE="$(topic_type "${IMAGE_TOPIC}")"
    if [[ "${IMAGE_TYPE}" == "sensor_msgs/msg/CompressedImage" || "${IMAGE_TYPE}" == "sensor_msgs/msg/Image" ]]; then
      echo "[ok] ${IMAGE_TOPIC}: ${IMAGE_TYPE}"
    else
      echo "[warn] ${IMAGE_TOPIC} type is ${IMAGE_TYPE}, VLM may need remap or converter"
    fi
  else
    echo "[warn] image topic missing: ${IMAGE_TOPIC} (VLM-only feature will be unavailable)"
  fi
fi

if [[ "${OK}" == "true" ]]; then
  echo "[ok] MARSIM topic contract is compatible with current USS-Nav input."
else
  echo "[fail] MARSIM topic contract check failed."
  exit 2
fi
