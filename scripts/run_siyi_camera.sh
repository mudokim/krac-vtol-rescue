#!/usr/bin/env bash
set -eo pipefail

# 실제 드론 카메라 브링업 (bt_real).
# 시뮬의 scripts/run_gz_image_republisher.sh 를 대체한다.
#
# SIYI A8 mini 의 RTSP 스트림을 받아 비전 스택이 구독하는 공유 이미지 토픽으로 발행.
# 기본 토픽은 /image_raw 로 krac_vision vision_bt.launch.py 기본값과 일치한다.
#
# 사용 예:
#   ./run_siyi_camera.sh
#   RTSP_URL=rtsp://192.168.144.25:8554/sub.264 ./run_siyi_camera.sh   # 저해상도 서브스트림
#   ROS_IMAGE_TOPIC=/image_raw RTSP_TRANSPORT=udp ./run_siyi_camera.sh

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

RTSP_URL="${RTSP_URL:-rtsp://192.168.144.25:8554/main.264}"
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/camera/image_raw}"
RTSP_TRANSPORT="${RTSP_TRANSPORT:-tcp}"
FRAME_ID="${FRAME_ID:-siyi_a8_camera}"
FPS_LIMIT="${FPS_LIMIT:-0.0}"

echo "[siyi_camera] rtsp=${RTSP_URL} -> ${ROS_IMAGE_TOPIC} (transport=${RTSP_TRANSPORT}, fps_limit=${FPS_LIMIT})"

exec ros2 launch my_package siyi_camera.launch.py \
  rtsp_url:="${RTSP_URL}" \
  image_topic:="${ROS_IMAGE_TOPIC}" \
  rtsp_transport:="${RTSP_TRANSPORT}" \
  frame_id:="${FRAME_ID}" \
  fps_limit:="${FPS_LIMIT}"
