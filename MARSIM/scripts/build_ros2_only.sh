#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[err] ROS setup not found: ${ROS_SETUP}" >&2
  exit 2
fi

# ROS2 core runtime chain used by USS-Nav integration.
CORE_PACKAGES=(
  quadrotor_msgs
  pose_utils
  uav_utils
  map_generator
  mars_drone_sim
  cascadePID
  local_sensing_node
  odom_visualization
  test_interface
)

if [[ "${1:-}" == "--clean" ]]; then
  rm -rf "${ROOT_DIR}/build" "${ROOT_DIR}/install" "${ROOT_DIR}/log"
fi

set +u
source "${ROS_SETUP}"
set -u

# Optional dependency prefix (PCL/JPEG/spdlog from conda, etc.).
MARSIM_DEPS_PREFIX="${MARSIM_DEPS_PREFIX:-/home/a4201/anaconda3/envs/TLCForMer-main}"
CMAKE_ARGS=(-DPython3_EXECUTABLE=/usr/bin/python3)
if [[ -d "${MARSIM_DEPS_PREFIX}" ]]; then
  export CMAKE_PREFIX_PATH="${MARSIM_DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export CPATH="${MARSIM_DEPS_PREFIX}/include:${MARSIM_DEPS_PREFIX}/include/pcl-1.15:${CPATH:-}"
  export LIBRARY_PATH="${MARSIM_DEPS_PREFIX}/lib:${LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="${MARSIM_DEPS_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  CMAKE_ARGS+=("-DPCL_DIR=${MARSIM_DEPS_PREFIX}/share/pcl-1.15")
  if [[ -f "${MARSIM_DEPS_PREFIX}/lib/libjpeg.so" ]]; then
    CMAKE_ARGS+=("-DJPEG_LIBRARY=${MARSIM_DEPS_PREFIX}/lib/libjpeg.so")
    CMAKE_ARGS+=("-DJPEG_INCLUDE_DIR=${MARSIM_DEPS_PREFIX}/include")
  fi
fi

cd "${ROOT_DIR}"
echo "[info] building ROS2-only packages: ${CORE_PACKAGES[*]}"
colcon build --packages-select "${CORE_PACKAGES[@]}" --cmake-force-configure --cmake-args "${CMAKE_ARGS[@]}"

echo "[ok] build completed"
