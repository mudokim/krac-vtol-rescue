#!/usr/bin/env bash
set -eo pipefail

# BT vision entry point: starts yolo_node, vision_tracker, and precision_lander
# together. Run scripts/run_gz_image_republisher.sh separately to publish the
# shared camera topic from Gazebo. Use scripts/run_vision.sh for the legacy
# yolo-only flow.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

YOLO_INITIAL_TARGET="${YOLO_INITIAL_TARGET:-disabled}"
VISION_OBB_CONFIDENCE="${VISION_OBB_CONFIDENCE:-0.20}"
VISION_TRACKING_HOLD_SEC="${VISION_TRACKING_HOLD_SEC:-75.0}"
VISION_INFERENCE_INTERVAL_SEC="${VISION_INFERENCE_INTERVAL_SEC:-0.25}"
VISION_MODEL_PATH="${VISION_MODEL_PATH:-$REPO_DIR/ros2_ws/src/krac_vision/weights/best.pt}"
if [[ ! -f "$VISION_MODEL_PATH" ]]; then
  echo "[vision_bt][ERROR] YOLO weight not found: $VISION_MODEL_PATH"
  exit 2
fi

# krac24_split has no drop zone, so its BT never reads yolo_node's
# /vision/detections (only ActivateYOLO -> camera/set_target, which
# vision_tracker doesn't even subscribe to) - starting it there just burns a
# full extra CPU YOLO pass per frame for nothing. Auto-skip it for that BT
# unless the caller explicitly overrides START_YOLO_DETECTOR.
if [[ -z "${START_YOLO_DETECTOR:-}" && "${BT_XML_PATH:-}" == *krac24_split* ]]; then
  START_YOLO_DETECTOR=false
fi
START_YOLO_DETECTOR="${START_YOLO_DETECTOR:-true}"

echo "[vision_bt] model=${VISION_MODEL_PATH} image_topic=${ROS_IMAGE_TOPIC:-/image_raw} initial_target=${YOLO_INITIAL_TARGET} obb_confidence=${VISION_OBB_CONFIDENCE} tracking_hold_sec=${VISION_TRACKING_HOLD_SEC} inference_interval_sec=${VISION_INFERENCE_INTERVAL_SEC} start_yolo_detector=${START_YOLO_DETECTOR}"

ros2 launch krac_vision vision_bt.launch.py \
  model:="${VISION_MODEL_PATH}" \
  image_topic:="${ROS_IMAGE_TOPIC:-/image_raw}" \
  initial_target:="${YOLO_INITIAL_TARGET}" \
  obb_confidence:="${VISION_OBB_CONFIDENCE}" \
  tracking_hold_sec:="${VISION_TRACKING_HOLD_SEC}" \
  inference_interval_sec:="${VISION_INFERENCE_INTERVAL_SEC}" \
  start_yolo_detector:="${START_YOLO_DETECTOR}"
