#!/usr/bin/env bash
set -euo pipefail

PID_PATH="${PID_PATH:-/tmp/vllm_qwen2vl.pid}"
PORT="${PORT:-8001}"

if [[ -f "${PID_PATH}" ]]; then
  pid="$(cat "${PID_PATH}" || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL "${pid}" >/dev/null 2>&1 || true
    echo "[ok] stopped pid ${pid}"
  fi
  rm -f "${PID_PATH}"
fi

pkill -f "vllm serve.*--port ${PORT}" >/dev/null 2>&1 || true
echo "[ok] requested stop for vLLM on port ${PORT}"
