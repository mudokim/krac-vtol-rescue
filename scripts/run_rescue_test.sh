#!/usr/bin/env bash
set -eo pipefail

# =============================================================================
#  bt_rescue_test 실행 래퍼 (REP 구조지점 반복 테스트).
#
#  run_real_bt.sh 를 그대로 재사용하되 BT 만 bt_rescue_test.xml 로 바꾼다.
#  → MAVROS + 카메라 + 짐벌 + 비전 + rescue 컨트롤러 + BT viewer + 이미지 창 동일.
#
#  흐름: 이륙(천이 없음) → REP 구조(원래 RescuePickupModule 그대로) → 원점 복귀 → 착륙.
#  REP 좌표/고도는 bt/bt_rescue_test.xml 에서 수정.
#
#  사용:
#    ./scripts/run_rescue_test.sh
#    START_GIMBAL=false ./scripts/run_rescue_test.sh    # 짐벌 없이
# =============================================================================

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 소스 트리의 테스트 BT 를 직접 지정(재빌드 없이도 최신 편집 반영).
# 재빌드 후 설치본을 쓰려면:
#   BT_XML_PATH=$(ros2 pkg prefix krac_control)/share/krac_control/bt/bt_rescue_test.xml
export BT_XML_PATH="${BT_XML_PATH:-$REPO_DIR/ros2_ws/src/krac_control/bt/bt_rescue_test.xml}"

echo "[rescue_test] BT_XML_PATH=${BT_XML_PATH}"
exec "$REPO_DIR/scripts/run_real_bt.sh"
