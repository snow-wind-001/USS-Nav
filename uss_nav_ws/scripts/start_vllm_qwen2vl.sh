#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH="${MODEL_PATH:-/home/a4201/checkpoints/Qwen2-VL-2B-Instruct}"
VLLM_HOST="${VLLM_HOST:-127.0.0.1}"
VLLM_PORT="${VLLM_PORT:-8001}"
PRIMARY_ENV="${PRIMARY_ENV:-youtu-agent}"
FALLBACK_ENV="${FALLBACK_ENV:-llmtest}"
LOG_PATH="${LOG_PATH:-/tmp/vllm_qwen2vl.log}"
PID_PATH="${PID_PATH:-/tmp/vllm_qwen2vl.pid}"

ensure_conda() {
  if ! command -v conda >/dev/null 2>&1; then
    echo "[error] conda not found in PATH"
    exit 1
  fi
}

wait_health() {
  local timeout_sec="${1:-180}"
  local i=0
  while [[ "${i}" -lt "${timeout_sec}" ]]; do
    if curl -sf "http://${VLLM_HOST}:${VLLM_PORT}/health" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  return 1
}

start_server_in_env() {
  local env_name="$1"
  echo "[info] start vLLM in conda env: ${env_name}"
  CUDA_DEVICE_ORDER=PCI_BUS_ID conda run -n "${env_name}" vllm serve "${MODEL_PATH}" \
    --host "${VLLM_HOST}" \
    --port "${VLLM_PORT}" \
    --served-model-name Qwen2-VL-2B-Instruct \
    --max-model-len 4096 \
    --generation-config vllm \
    >"${LOG_PATH}" 2>&1 &
  local pid=$!
  echo "${pid}" >"${PID_PATH}"

  if wait_health 120; then
    echo "[ok] vLLM is ready: http://${VLLM_HOST}:${VLLM_PORT}"
    echo "[ok] log: ${LOG_PATH}"
    echo "[ok] pid: ${pid}"
    return 0
  fi

  echo "[warn] vLLM did not become healthy in env ${env_name}"
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  fi
  return 1
}

clone_fallback_env_if_needed() {
  local exists=""
  exists="$(conda env list | awk -v e="${FALLBACK_ENV}" '$1==e {print $1}')"
  if [[ -n "${exists}" ]]; then
    echo "[info] fallback env exists: ${FALLBACK_ENV}"
    return
  fi
  echo "[info] clone env: conda create -n ${FALLBACK_ENV} --clone ${PRIMARY_ENV}"
  conda create -y -n "${FALLBACK_ENV}" --clone "${PRIMARY_ENV}"
}

main() {
  ensure_conda
  if [[ ! -d "${MODEL_PATH}" ]]; then
    echo "[error] model path not found: ${MODEL_PATH}"
    exit 1
  fi

  if start_server_in_env "${PRIMARY_ENV}"; then
    exit 0
  fi

  echo "[warn] failed in ${PRIMARY_ENV}, follow user rule and switch to ${FALLBACK_ENV}"
  clone_fallback_env_if_needed

  if start_server_in_env "${FALLBACK_ENV}"; then
    exit 0
  fi

  echo "[error] vLLM failed in both ${PRIMARY_ENV} and ${FALLBACK_ENV}"
  echo "[hint] check log: ${LOG_PATH}"
  exit 2
}

main "$@"
