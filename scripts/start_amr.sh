#!/usr/bin/env bash
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_FILE="$PROJECT_DIR/config/runtime.env"
[[ -f "$RUNTIME_FILE" ]] && source "$RUNTIME_FILE"
ROS_DISTRO="${ROS_DISTRO:-humble}"
OAK_LAUNCH_PACKAGE="${OAK_LAUNCH_PACKAGE:-depthai_ros_driver}"
OAK_LAUNCH_FILE="${OAK_LAUNCH_FILE:-camera.launch.py}"
LIDAR_LAUNCH_PACKAGE="${LIDAR_LAUNCH_PACKAGE:-ldlidar_stl_ros2}"
LIDAR_LAUNCH_FILE="${LIDAR_LAUNCH_FILE:-ld19.launch.py}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyUSB_mcu}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB_lidar}"
LIDAR_PORT_ARGUMENT="${LIDAR_PORT_ARGUMENT:-port_name}"
OPEN_GUI="${OPEN_GUI:-1}"
OPEN_YOLO_WINDOW="${OPEN_YOLO_WINDOW:-1}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"
OPEN_DASHBOARD="${OPEN_DASHBOARD:-1}"
RUN_DIR="$PROJECT_DIR/.run"
LOG_DIR="$PROJECT_DIR/logs/runtime"
PID_FILE="$RUN_DIR/amr.pids"
mkdir -p "$RUN_DIR" "$LOG_DIR"
: > "$PID_FILE"

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

start_process() {
  local name="$1"; shift
  echo "[START] $name"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  echo "$! $name" >> "$PID_FILE"
}

cleanup() {
  echo "AMR 프로세스를 종료합니다."
  if [[ -f "$PID_FILE" ]]; then
    while read -r pid name; do
      if kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; fi
    done < "$PID_FILE"
  fi
  rm -f "$PID_FILE"
}
trap cleanup EXIT INT TERM

start_process oak_driver ros2 launch "$OAK_LAUNCH_PACKAGE" "$OAK_LAUNCH_FILE"
start_process lidar_driver ros2 launch "$LIDAR_LAUNCH_PACKAGE" "$LIDAR_LAUNCH_FILE" "$LIDAR_PORT_ARGUMENT:=$LIDAR_DEVICE"
start_process amr_core ros2 launch amr_bringup hardware_system.launch.py "mcu_port:=$MCU_DEVICE"

sleep 5
for topic in /oak/rgb/image_raw /scan /mcu/status /safety/state; do
  if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "[ OK ] 토픽 수신: $topic"
  else
    echo "[WARN] 토픽 미수신: $topic (관련 로그를 확인하세요)"
  fi
done

if [[ "$OPEN_GUI" == "1" && -n "${DISPLAY:-}" ]]; then
  [[ "$OPEN_YOLO_WINDOW" == "1" ]] && start_process yolo_view rqt_image_view /yolo/annotated_image
  [[ "$OPEN_RVIZ" == "1" ]] && start_process lidar_rviz rviz2 -d "$PROJECT_DIR/config/amr_lidar.rviz"
  [[ "$OPEN_DASHBOARD" == "1" ]] && start_process dashboard xdg-open http://127.0.0.1:8080
else
  echo "GUI를 열지 않습니다. 관제 주소: http://127.0.0.1:8080"
fi

echo "AMR가 READY 상태로 실행되었습니다. 실제 주행은 안전 조건 확인 후 푸시스위치로 허용됩니다."
wait
