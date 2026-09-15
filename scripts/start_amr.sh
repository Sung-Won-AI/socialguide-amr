#!/usr/bin/env bash
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_FILE="$PROJECT_DIR/config/runtime.env"
[[ -f "$RUNTIME_FILE" ]] && source "$RUNTIME_FILE"
ROS_DISTRO="${ROS_DISTRO:-humble}"
OAK_LAUNCH_PACKAGE="${OAK_LAUNCH_PACKAGE:-depthai_ros_driver}"
OAK_LAUNCH_FILE="${OAK_LAUNCH_FILE:-camera.launch.py}"
VISION_MODE="${VISION_MODE:-docker}"
YOLO_WEB_URL="${YOLO_WEB_URL:-http://127.0.0.1:8081}"
LIDAR_LAUNCH_PACKAGE="${LIDAR_LAUNCH_PACKAGE:-ldlidar_stl_ros2}"
LIDAR_LAUNCH_FILE="${LIDAR_LAUNCH_FILE:-stl27l.launch.py}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB_lidar}"
LIDAR_PORT_ARGUMENT="${LIDAR_PORT_ARGUMENT:-port_name}"
OPEN_GUI="${OPEN_GUI:-1}"
OPEN_YOLO_WINDOW="${OPEN_YOLO_WINDOW:-1}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"
OPEN_DASHBOARD="${OPEN_DASHBOARD:-1}"
RUN_DIR="$PROJECT_DIR/.run"
LOG_DIR="$PROJECT_DIR/logs/runtime"
PID_FILE="$RUN_DIR/amr.pids"
CONTROLLER_FILE="$RUN_DIR/amr.controller.pid"
mkdir -p "$RUN_DIR" "$LOG_DIR"

if [[ -f "$CONTROLLER_FILE" ]]; then
  read -r old_controller < "$CONTROLLER_FILE" || true
  if [[ "${old_controller:-}" =~ ^[0-9]+$ ]] && kill -0 "$old_controller" 2>/dev/null; then
    echo "[FAIL] AMR가 이미 실행 중입니다 (controller PID: $old_controller)."
    echo "       먼저 $PROJECT_DIR/scripts/stop_amr.sh 를 실행하세요."
    exit 1
  fi
  echo "[WARN] 오래된 실행 정보 파일을 정리합니다."
  rm -f "$CONTROLLER_FILE" "$PID_FILE"
fi

# ROS-generated setup files read these optional variables during initialization.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$PROJECT_DIR/jetson_ws/install/setup.bash"
if [[ -n "${LIDAR_SETUP_FILE:-}" && -f "$LIDAR_SETUP_FILE" ]]; then
  source "$LIDAR_SETUP_FILE"
fi
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PROJECT_DIR/scripts/preflight_amr.sh"

echo "$$" > "$CONTROLLER_FILE"
: > "$PID_FILE"

start_process() {
  local name="$1"; shift
  echo "[START] $name"
  # Give each component its own process group so ROS launch children are also
  # stopped, including the dashboard HTTP server.
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  echo "$! $name" >> "$PID_FILE"
}

cleanup_done=0
cleanup() {
  local pid name
  (( cleanup_done == 0 )) || return 0
  cleanup_done=1
  trap - EXIT INT TERM
  echo "AMR 프로세스를 종료합니다."
  if [[ -f "$PID_FILE" ]]; then
    while read -r pid name; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        echo "[STOP] $name ($pid)"
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"

    sleep 2
    while read -r pid name; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        echo "[KILL] 종료되지 않은 $name ($pid)"
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"
  fi
  docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
  rm -f "$PID_FILE" "$CONTROLLER_FILE"
}
trap cleanup EXIT INT TERM

if [[ "$VISION_MODE" == "docker" ]]; then
  start_process yolo_docker "$PROJECT_DIR/scripts/start_yolo_docker.sh"
else
  start_process oak_driver ros2 launch "$OAK_LAUNCH_PACKAGE" "$OAK_LAUNCH_FILE"
fi
start_process lidar_driver ros2 launch "$LIDAR_LAUNCH_PACKAGE" "$LIDAR_LAUNCH_FILE" "$LIDAR_PORT_ARGUMENT:=$LIDAR_DEVICE"
start_process amr_core ros2 launch amr_bringup hardware_system.launch.py "mcu_port:=$MCU_DEVICE"

sleep 5
for topic in /yolo/detections /scan /mcu/status /safety/state; do
  if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "[ OK ] 토픽 수신: $topic"
  else
    echo "[WARN] 토픽 미수신: $topic (관련 로그를 확인하세요)"
  fi
done

if [[ "$OPEN_GUI" == "1" && -n "${DISPLAY:-}" ]]; then
  [[ "$OPEN_YOLO_WINDOW" == "1" ]] && xdg-open "$YOLO_WEB_URL" >/dev/null 2>&1 || true
  [[ "$OPEN_RVIZ" == "1" ]] && start_process lidar_rviz rviz2 -d "$PROJECT_DIR/config/amr_lidar.rviz"
  [[ "$OPEN_DASHBOARD" == "1" ]] && xdg-open http://127.0.0.1:8080 >/dev/null 2>&1 || true
else
  echo "GUI를 열지 않습니다. 관제 주소: http://127.0.0.1:8080"
fi

echo "AMR가 READY 상태로 실행되었습니다. 실제 주행은 안전 조건 확인 후 푸시스위치로 허용됩니다."
echo "종료 명령: $PROJECT_DIR/scripts/stop_amr.sh"
wait
