#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DURATION_SEC=120
USE_RVIZ=false
USE_VLM=false
OUTPUT_ROOT=""
ODOM_TOPIC="/odom"
POINTS_TOPIC="/points"
IMAGE_TOPIC="/camera/image/compressed"
CHECK_IMAGE=false
SKIP_TOPIC_CHECK=false
STRICT_PASS=false
TARGET_TEXT="chair"

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
    --skip-topic-check)
      SKIP_TOPIC_CHECK=true
      shift
      ;;
    --strict-pass)
      STRICT_PASS=true
      shift
      ;;
    --target-text)
      TARGET_TEXT="$2"
      shift 2
      ;;
    *)
      echo "[error] unknown arg: $1"
      echo "usage: $0 [--duration SEC] [--use-rviz] [--use-vlm] [--output-root DIR] [--odom-topic TOPIC] [--points-topic TOPIC] [--image-topic TOPIC] [--check-image] [--skip-topic-check] [--strict-pass] [--target-text TEXT]"
      exit 1
      ;;
  esac
done

if [[ "${SKIP_TOPIC_CHECK}" == "false" ]]; then
  CHECK_CMD=(bash "${SCRIPT_DIR}/check_marsim_topics.sh" --odom-topic "${ODOM_TOPIC}" --points-topic "${POINTS_TOPIC}" --image-topic "${IMAGE_TOPIC}")
  if [[ "${CHECK_IMAGE}" == "true" ]]; then
    CHECK_CMD+=(--check-image)
  fi
  "${CHECK_CMD[@]}"
fi

RECORD_CMD=(bash "${SCRIPT_DIR}/record_and_evaluate_explore.sh" --duration "${DURATION_SEC}" --marsim)
if [[ "${USE_RVIZ}" == "true" ]]; then
  RECORD_CMD+=(--use-rviz)
fi
if [[ "${USE_VLM}" == "true" ]]; then
  RECORD_CMD+=(--use-vlm)
  RECORD_CMD+=(--vlm-image-topic "${IMAGE_TOPIC}")
  RECORD_CMD+=(--vlm-target-text "${TARGET_TEXT}")
  IMAGE_TYPE="$(ros2 topic type "${IMAGE_TOPIC}" 2>/dev/null | awk 'NR==1{print $1}')"
  if [[ "${IMAGE_TYPE}" == "sensor_msgs/msg/Image" ]]; then
    RECORD_CMD+=(--vlm-image-compressed false)
    echo "[info] detected raw image stream on ${IMAGE_TOPIC}, set image_is_compressed=false"
  elif [[ "${IMAGE_TYPE}" == "sensor_msgs/msg/CompressedImage" ]]; then
    RECORD_CMD+=(--vlm-image-compressed true)
    echo "[info] detected compressed image stream on ${IMAGE_TOPIC}, set image_is_compressed=true"
  else
    echo "[warn] cannot detect image type for ${IMAGE_TOPIC}, fallback image_is_compressed=true"
    RECORD_CMD+=(--vlm-image-compressed true)
  fi
fi
if [[ -n "${OUTPUT_ROOT}" ]]; then
  RECORD_CMD+=(--output-root "${OUTPUT_ROOT}")
fi

TMP_LOG="$(mktemp)"
set +e
"${RECORD_CMD[@]}" 2>&1 | tee "${TMP_LOG}"
RECORD_EXIT=$?
set -e
if [[ ${RECORD_EXIT} -ne 0 ]]; then
  echo "[fail] record_and_evaluate_explore failed with code ${RECORD_EXIT}"
  rm -f "${TMP_LOG}"
  exit ${RECORD_EXIT}
fi

RUN_DIR="$(awk -F= '/^\[done\] run_dir=/{print $2}' "${TMP_LOG}" | tail -n1)"
rm -f "${TMP_LOG}"
if [[ -z "${RUN_DIR}" ]]; then
  echo "[fail] cannot parse run_dir from record output"
  exit 3
fi

echo "[info] validating run: ${RUN_DIR}"
set +e
python "${SCRIPT_DIR}/validate_marsim_results.py" --run-dir "${RUN_DIR}" | tee "${RUN_DIR}/validation.log"
VALID_EXIT=$?
set -e

if [[ ${VALID_EXIT} -eq 0 ]]; then
  echo "[done] MARSIM validation PASS"
  echo "[done] report: ${RUN_DIR}/eval/marsim_validation_report.md"
  exit 0
fi

echo "[warn] MARSIM validation produced FAIL report"
echo "[warn] report: ${RUN_DIR}/eval/marsim_validation_report.md"
if [[ "${STRICT_PASS}" == "true" ]]; then
  exit ${VALID_EXIT}
fi
exit 0
