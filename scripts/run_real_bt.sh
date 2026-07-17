#!/usr/bin/env bash
set -eo pipefail

# =============================================================================
#  bt_real 실기체 BT 진입점 (Gazebo/PX4-SITL 없음).
#
#  시뮬용 run_sitl_bt.sh 를 실기체용으로 정리한 버전. 다음 시뮬 전용 요소를
#  전부 제거했다:
#    - make px4_sitl (Gazebo/PX4 SITL)          - apply_px4_assets.sh
#    - ros_gz_bridge 그리퍼/짐벌 서보 브리지      - auto_spawn*.sh (gz 물체 스폰)
#    - gimbal_relay.py (gz 조인트)               - gz_image_republisher (gz 카메라)
#    - SITL 전용 PX4 파라미터 조작(NAV_DLL_ACT=0, CBRK_AIRSPD_CHK) ★안전상 제거★
#    - SITL 로그 기반 EKF heading 대기
#
#  대신 실기체 노드를 올린다:
#    - MAVROS(real fcu_url)  - SIYI 카메라(my_package) → /camera/image_raw
#    - SIYI 짐벌(krac_gimbal) - 비전 스택 - rescue 컨트롤러 - krac_bt_runner
#    - BT viewer 창 + rqt_image_view 이미지 창 (Gazebo 없이도 항상 뜸)
#
#  터미널 구성(권장):
#    T1) QGroundControl (선택)      : ~/QGroundControl.AppImage &
#    T2) 이 스크립트                : ./scripts/run_real_bt.sh
#    T3) 그리퍼 수동 파지(선택)     : python3 scripts/gripper_teleop.py
# =============================================================================

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true

ROS_LOG_BASE="${ROS_LOG_DIR:-/tmp/krac_ros_logs}"
mkdir -p "$ROS_LOG_BASE"
RUN_LOG_DIR="${RUN_LOG_DIR:-$ROS_LOG_BASE/real_run_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
export ROS_LOG_DIR="$RUN_LOG_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"
echo "[REAL] Logs: ${RUN_LOG_DIR}"

# ----------------------------- 토글 -----------------------------------------
START_CAMERA="${START_CAMERA:-true}"          # SIYI RTSP → /camera/image_raw
START_GIMBAL="${START_GIMBAL:-true}"          # SIYI 짐벌(SIYI SDK/UDP)
START_VISION_BT="${START_VISION_BT:-true}"    # yolo/tracker/precision_lander
START_RESCUE_PLACEHOLDER="${START_RESCUE_PLACEHOLDER:-true}"  # 외부 rescue 모듈(핸드오프)
START_GRIPPER="${START_GRIPPER:-true}"        # ★실기체 그리퍼(ESP32)★ 포트 없으면 자동 스킵
GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyUSB0}"  # ESP32 USB-시리얼. MAVROS(/dev/ttyTHS1)와 별개
GZ_MODEL_NAME="${GZ_MODEL_NAME:-standard_vtol_0}"  # 서보 토픽 네임스페이스
START_VISION_VIEW="${START_VISION_VIEW:-true}"   # ★이미지 창(rqt_image_view)★
START_RQT_GRAPH="${START_RQT_GRAPH:-false}"      # 노드/토픽 그래프
ENABLE_BT_VIEWER="${ENABLE_BT_VIEWER:-true}"     # ★BT viewer 창★

# 카메라/비전 공유 이미지 토픽 (실기체 네이티브 규약)
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/camera/image_raw}"
export ROS_IMAGE_TOPIC
VISION_DEBUG_TOPIC="${VISION_DEBUG_TOPIC:-/vision/dbg_image}"

# 실기체 미션(구조 leg): 기본은 krac24_split. 필요시 override.
BT_XML_PATH="${BT_XML_PATH:-$(ros2 pkg prefix krac_control)/share/krac_control/bt/krac_mission_bt_krac24_split.xml}"
BT_PARAMS_FILE="${BT_PARAMS_FILE:-$(ros2 pkg prefix krac_control)/share/krac_control/config/krac_bt_params_krac24.yaml}"

# YOLO 가중치: vtol-rescue BT 버전의 best.pt
VISION_MODEL_PATH="${VISION_MODEL_PATH:-$REPO_DIR/ros2_ws/src/krac_vision/weights/best.pt}"

SUPPORT_PIDS=()
start_support_process() {
  local name="$1"; shift
  echo "[REAL] Starting ${name}..."
  "$@" >"$RUN_LOG_DIR/${name// /_}.log" 2>&1 &
  SUPPORT_PIDS+=("$!")
}
start_optional_ros_gui() {
  local name="$1"; shift
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[REAL] Skipping ${name}: DISPLAY 가 없음(헤드리스). GUI 를 보려면 X 디스플레이 필요."
    return
  fi
  start_support_process "$name" "$@"
}

# ----------------------------- MAVROS ---------------------------------------
echo "[REAL] Starting MAVROS (real fcu_url from mavros_params_real.yaml)..."
ros2 launch krac_mission real_vtol.launch.py \
  start_mavros:=true \
  start_logger:="${START_LOGGER:-false}" \
  >"$RUN_LOG_DIR/real_vtol.launch.log" 2>&1 &
LAUNCH_PID=$!

# ----------------------------- 카메라 ---------------------------------------
if [[ "${START_CAMERA}" == "true" ]]; then
  start_support_process "SIYI camera" \
    env ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC}" "$REPO_DIR/scripts/run_siyi_camera.sh"
fi

# ----------------------------- 짐벌 -----------------------------------------
# 실기체 짐벌은 SIYI SDK(UDP 192.168.144.25:37260) 로 직접 제어한다.
# (시뮬의 gimbal_relay.py + ros_gz_bridge 경로는 사용하지 않음)
if [[ "${START_GIMBAL}" == "true" ]]; then
  start_support_process "SIYI gimbal" ros2 run krac_gimbal gimbal_node
fi

# ----------------------------- rescue 컨트롤러 -------------------------------
# BT 가 상공 도착 후 핸드오프하는 '외부 rescue 모듈'. AUTO.LAND→대기→SUCCESS 는
# MAVROS 로 실기체에서 동작한다. ★단, 파지(그리퍼) 부분은 아직 Gazebo 서보 토픽
# 기반이라 실기체 그리퍼는 움직이지 않는다 → 실제 그리퍼 드라이버로 교체 필요.★
if [[ "${START_RESCUE_PLACEHOLDER}" == "true" ]]; then
  start_support_process "Rescue placeholder" \
    ros2 run krac_control rescue_controller_placeholder.py \
    --ros-args \
    -p manual_grasp_enable:="${MANUAL_GRASP:-true}" \
    -p selftest_sweep_enable:="${GIMBAL_SELFTEST:-false}"
  if [[ "${MANUAL_GRASP:-true}" == "true" ]]; then
    echo "[REAL] MANUAL_GRASP on: 착륙 후 별도 터미널에서"
    echo "[REAL]   cd ${REPO_DIR} && source ros2_ws/install/setup.bash && python3 scripts/gripper_teleop.py"
  fi
fi

# ----------------------------- 그리퍼(실기체) -------------------------------
# 제어팀 payload_control(github: rionxst/payload_control) 의 ESP32 시리얼 브리지와
# 우리 서보 토픽을 잇는다. 비행 코드는 수정하지 않고 어댑터가 번역만 한다:
#
#   rescue_controller ──/model/<m>/servo_4,5,6 (Float64, rad)──▶ gripper_adapter
#     ──/gripper/set, /gripper/set_yaw, /gripper/arm (서비스)──▶ servo_serial_bridge
#     ──시리얼 115200──▶ ESP32
#
# ESP32 가 안 꽂혀 있으면 조용히 건너뛴다. 그리퍼만 죽고 나머지 미션은 그대로 간다
# (지금까지와 동일한 동작). 포트가 다르면 GRIPPER_PORT=/dev/ttyACM0 처럼 지정.
if [[ "${START_GRIPPER}" == "true" ]]; then
  if [[ -e "${GRIPPER_PORT}" ]]; then
    echo "[REAL] Gripper: ESP32 at ${GRIPPER_PORT}"
    start_support_process "gripper serial bridge" \
      ros2 run servo_serial_bridge servo_bridge_node \
      --ros-args -p port:="${GRIPPER_PORT}" -p baud:=115200
    # gz_model_name 은 rescue_controller_placeholder 의 같은 이름 파라미터와
    # 반드시 일치해야 한다(서보 토픽 이름이 여기서 결정됨).
    start_support_process "gripper adapter" \
      ros2 run krac_control gripper_adapter.py \
      --ros-args -p gz_model_name:="${GZ_MODEL_NAME}"
  else
    echo "[REAL] Gripper SKIPPED: ${GRIPPER_PORT} 없음(ESP32 미연결)."
    echo "[REAL]   실기체 그리퍼는 동작하지 않는다(미션은 그대로 진행)."
    echo "[REAL]   포트가 다르면: GRIPPER_PORT=/dev/ttyACM0 ./scripts/run_rescue_test.sh"
  fi
fi

# ----------------------------- 비전 스택 ------------------------------------
if [[ "${START_VISION_BT}" == "true" ]]; then
  start_support_process "BT vision stack" \
    env ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC}" \
        VISION_MODEL_PATH="${VISION_MODEL_PATH}" \
        BT_XML_PATH="${BT_XML_PATH}" \
        "$REPO_DIR/scripts/run_vision_bt.sh"
fi

# ----------------------------- GUI 창 ---------------------------------------
# ★ Gazebo 가 없어도 이미지 창과 rqt_graph 는 여기서 뜬다 ★
if [[ "${START_VISION_VIEW}" == "true" ]]; then
  start_optional_ros_gui "vision image view" \
    ros2 run rqt_image_view rqt_image_view "${VISION_DEBUG_TOPIC}"
fi
if [[ "${START_RQT_GRAPH}" == "true" ]]; then
  start_optional_ros_gui "rqt graph" ros2 run rqt_graph rqt_graph
fi

cleanup() {
  echo ""
  echo "[CLEANUP] Stopping MAVROS launch and support processes..."
  kill "$LAUNCH_PID" 2>/dev/null || true
  for pid in "${SUPPORT_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT

# ----------------------------- MAVROS 연결 대기 -----------------------------
echo "[REAL] Waiting for MAVROS <-> FCU connection (/mavros/state)..."
CONNECTED=false
for i in {1..60}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "connected: true"; then
    echo "[OK] MAVROS connected to FCU."
    CONNECTED=true
    break
  fi
  sleep 1
done
if [[ "$CONNECTED" != "true" ]]; then
  echo "[ERROR] MAVROS 가 60초 내 FCU 에 연결되지 않음."
  echo "        mavros_params_real.yaml 의 fcu_url(포트/보레이트/권한)을 확인하세요."
  exit 3
fi

# ★ 실기체에서는 SITL 전용 PX4 파라미터 조작을 하지 않는다 ★
#   - NAV_DLL_ACT=0  : 데이터링크 상실 failsafe 를 끄는 것 → 실기체 금지.
#   - CBRK_AIRSPD_CHK: airspeed preflight 우회 → 실기체 금지(실제 센서 사용).
#   무장 게이트(heading/airspeed/EKF)는 PX4 자체 preflight 로 판단하게 둔다.

# ----------------------------- BT 실행 --------------------------------------
echo "[REAL] Starting krac_bt_runner (BT viewer=${ENABLE_BT_VIEWER})..."
BT_LAUNCH_ARGS=(
  print_bt_transitions:="${PRINT_BT_TRANSITIONS:-true}"
  enable_bt_viewer:="${ENABLE_BT_VIEWER}"
  bt_viewer_direction:="${BT_VIEWER_DIRECTION:-Vertical}"
  bt_xml_path:="${BT_XML_PATH}"
  params_file:="${BT_PARAMS_FILE}"
  mission_upload_stub_success:="${MISSION_UPLOAD_STUB_SUCCESS:-false}"
  gripper_stub_success:="${GRIPPER_STUB_SUCCESS:-false}"
)
# NOTE(sim_bypass): krac_bt_runner.launch.py 기본 sim_bypass=true. 실기체 의미는
#   BT 흐름 감사 결과에 따라 확정할 것(BT_SIM_BYPASS 로 override 가능).
if [[ -n "${BT_SIM_BYPASS:-}" ]]; then
  BT_LAUNCH_ARGS+=(sim_bypass:="${BT_SIM_BYPASS}")
fi

set +e
ros2 launch krac_control krac_bt_runner.launch.py "${BT_LAUNCH_ARGS[@]}" \
  >"$RUN_LOG_DIR/krac_bt_runner.launch.log" 2>&1
BT_EXIT=$?
set -e
echo "[REAL] krac_bt_runner exited with code ${BT_EXIT}"
exit "$BT_EXIT"
