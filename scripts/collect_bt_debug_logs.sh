#!/usr/bin/env bash
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_LOG_DIR="${1:-}"

if [[ -z "$RUN_LOG_DIR" ]]; then
  RUN_LOG_DIR="$(find /tmp/krac_ros_logs -maxdepth 1 -type d -name 'bt_run_*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | awk '{print $2}')"
fi

if [[ -z "$RUN_LOG_DIR" || ! -d "$RUN_LOG_DIR" ]]; then
  echo "Usage: $0 <run_log_dir>"
  echo "No bt_run_* directory found under /tmp/krac_ros_logs."
  exit 2
fi

SUMMARY="$RUN_LOG_DIR/summary.txt"

section() {
  printf '\n===== %s =====\n' "$1"
}

collect_live_topic() {
  local topic="$1"
  local label="$2"
  section "$label"
  timeout 4 ros2 topic echo "$topic" --once 2>&1 || true
}

collect_topic_hz() {
  local topic="$1"
  local label="$2"
  section "$label"
  timeout 6 ros2 topic hz "$topic" 2>&1 || true
}

{
  echo "BT debug summary"
  echo "repo: $REPO_DIR"
  echo "run_log_dir: $RUN_LOG_DIR"
  echo "generated_at: $(date '+%Y-%m-%d %H:%M:%S %Z')"

  section "Files"
  find "$RUN_LOG_DIR" -maxdepth 3 -type f \
    ! -path '*/collector_ros_logs/*' \
    ! -path '*/sampler_ros_logs/*' \
    -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null | sort || true

  section "High Signal Log Lines"
  rg -n \
    "BT finished|GlobalMissionRecovery|Mission Upload|mission received|mission rejected|Requested flight mode|Requested arm|Requested VTOL|WaitForWaypointReached|PrecisionLand|Closing gripper|Opening gripper|Ready for takeoff|Landing detected|Disarmed|ERROR|WARN|process has died|Traceback|timeout|failed|Quad-chute|Preflight Fail" \
    "$RUN_LOG_DIR" \
    -g '!summary.txt' \
    -g '!collector_ros_logs/**' \
    -g '!sampler_ros_logs/**' \
    2>/dev/null | tail -240 || true

  section "BT Runner Tail"
  find "$RUN_LOG_DIR" -type f \( -name '*krac_bt_runner*.log' -o -name 'krac_bt_runner.launch.log' \) \
    ! -path '*/collector_ros_logs/*' \
    ! -path '*/sampler_ros_logs/*' \
    -print0 2>/dev/null \
    | xargs -0 -r tail -n 220 || true

  section "Launch Tail"
  find "$RUN_LOG_DIR" -type f -name '*launch.log' \
    ! -path '*/collector_ros_logs/*' \
    ! -path '*/sampler_ros_logs/*' \
    -print0 2>/dev/null \
    | xargs -0 -r tail -n 160 || true

  section "Mission Loader Tail"
  find "$RUN_LOG_DIR" -type f \( -name 'python3_*.log' -o -name '*mission*loader*.log' \) \
    ! -path '*/collector_ros_logs/*' \
    ! -path '*/sampler_ros_logs/*' \
    -print0 2>/dev/null \
    | xargs -0 -r tail -n 160 || true

  section "Vision Tail"
  find "$RUN_LOG_DIR" -type f \( -name '*vision*.log' -o -name '*yolo*.log' -o -name '*image*republisher*.log' -o -name '*precision_lander*.log' \) \
    ! -path '*/collector_ros_logs/*' \
    ! -path '*/sampler_ros_logs/*' \
    -print0 2>/dev/null \
    | xargs -0 -r tail -n 180 || true

  if [[ -f /opt/ros/humble/setup.bash && -f "$REPO_DIR/ros2_ws/install/setup.bash" ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    source "$REPO_DIR/ros2_ws/install/setup.bash"
    set -u
    export ROS_LOG_DIR="$RUN_LOG_DIR/collector_ros_logs"
    mkdir -p "$ROS_LOG_DIR"
    ros2 daemon stop >/dev/null 2>&1 || true

    collect_live_topic /mavros/state "Live /mavros/state"
    collect_live_topic /mavros/extended_state "Live /mavros/extended_state"
    collect_live_topic /mavros/global_position/rel_alt "Live /mavros/global_position/rel_alt"
    collect_live_topic /mavros/mission/reached "Live /mavros/mission/reached"
    collect_live_topic /mavros/mission/waypoints "Live /mavros/mission/waypoints"
    collect_live_topic /vision/target_error "Live /vision/target_error"

    collect_topic_hz /image_raw "Hz /image_raw"
    collect_topic_hz /vision/target_error "Hz /vision/target_error"
    collect_topic_hz /precision_lander/cmd_vel "Hz /precision_lander/cmd_vel"
  fi
} >"$SUMMARY"

cat "$SUMMARY"
