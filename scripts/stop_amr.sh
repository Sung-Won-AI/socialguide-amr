#!/usr/bin/env bash
set -u
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$PROJECT_DIR/.run/amr.pids"
[[ -f "$PID_FILE" ]] || { echo "실행 중인 AMR PID 파일이 없습니다."; exit 0; }
while read -r pid name; do
  if kill -0 "$pid" 2>/dev/null; then echo "[STOP] $name ($pid)"; kill "$pid"; fi
done < "$PID_FILE"
rm -f "$PID_FILE"
