#!/usr/bin/env python3

import random
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetector(Node):
    """YOLO detector with mission-controlled inference gating.

    /camera/set_target command semantics:
      - "off" or "none": skip YOLO inference and publish target_error.z = 0
      - "all": run YOLO and publish all classes
      - any other string: run YOLO and keep only that class name

    This keeps Jetson load low during cruise/mission phases where vision is not needed.
    """

    def __init__(self) -> None:
        super().__init__('yolo_detector_node')

        self.declare_parameter('model', '')
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('initial_target', 'disabled')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('publish_debug', True)

        self.model_path = self.get_parameter('model').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.target_class = self.get_parameter('initial_target').get_parameter_value().string_value.strip()
        self.max_rate_hz = self.get_parameter('max_rate_hz').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug').get_parameter_value().bool_value

        self.cv_bridge = CvBridge()
        self._class_to_color = {}
        self._last_inference_time = 0.0
        self._last_status_log_time = 0.0
        self.yolo = None

        self._pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self._dbg_pub = self.create_publisher(Image, '/dbg_image', 10)
        self._error_pub = self.create_publisher(Point, '/camera/target_error', 10)

        self._target_sub = self.create_subscription(
            String,
            '/camera/set_target',
            self.target_cb,
            10,
        )
        self._sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_profile_sensor_data,
        )

        if not self.model_path:
            self.get_logger().error('YOLO model path is empty. Pass parameter model:=/path/to/best.pt')
            return

        if not Path(self.model_path).exists():
            self.get_logger().error(f'YOLO model file does not exist: {self.model_path}')
            return

        try:
            self.yolo = YOLO(self.model_path)
            self.yolo.to(self.device)
            self.get_logger().info(
                f'YOLO loaded: model={self.model_path}, device={self.device}, '
                f'threshold={self.threshold}, image_topic={self.image_topic}, '
                f'initial_target={self.target_class}'
            )
        except Exception as exc:
            self.get_logger().error(f'YOLO model load failed: {exc}')
            self.yolo = None

    def target_cb(self, msg: String) -> None:
        new_target = msg.data.strip()
        if new_target == '':
            new_target = 'off'
        if new_target != self.target_class:
            self.target_class = new_target
            self.get_logger().info(f'Vision target changed: {self.target_class}')

    def image_cb(self, msg: Image) -> None:
        now = time.monotonic()
        if self.max_rate_hz > 0.0:
            min_dt = 1.0 / self.max_rate_hz
            if now - self._last_inference_time < min_dt:
                return
        self._last_inference_time = now

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            self._publish_empty(msg.header)
            return

        target = self.target_class.lower()
        if target in ('off', 'none', 'disable', 'disabled'):
            self._publish_empty(msg.header)
            self._publish_debug_image(cv_image, msg.header, status_text='YOLO OFF')
            self._log_throttled('Vision inference is OFF. Publish /camera/set_target = basket/all/... to enable.')
            return

        if self.yolo is None:
            self._publish_empty(msg.header)
            self._publish_debug_image(cv_image, msg.header, status_text='YOLO NOT LOADED')
            return

        h, w = cv_image.shape[:2]
        img_cx, img_cy = w // 2, h // 2
        self._draw_center_cross(cv_image, img_cx, img_cy)

        try:
            results = self.yolo.track(
                source=cv_image,
                conf=self.threshold,
                persist=True,
                verbose=False,
            )
            result = results[0].cpu()
        except Exception as exc:
            self.get_logger().warn(f'YOLO inference failed: {exc}')
            self._publish_empty(msg.header)
            self._publish_debug_image(cv_image, msg.header, status_text='INFERENCE FAILED')
            return

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        best_box = None
        best_label = ''
        best_score = -1.0

        if result.boxes is not None:
            for box_result in result.boxes:
                label_id = int(box_result.cls)
                label_name = str(self.yolo.names[label_id])
                score = float(box_result.conf)

                if target != 'all' and label_name.lower() != target:
                    continue

                box = box_result.xywh[0]
                self._append_detection(detections_msg, label_name, score, box)
                self._draw_detection(cv_image, label_name, score, box)

                if score > best_score:
                    best_score = score
                    best_box = box
                    best_label = label_name

        if best_box is not None:
            obj_x = float(best_box[0])
            obj_y = float(best_box[1])
            err_msg = Point()
            err_msg.x = obj_x - float(img_cx)
            err_msg.y = obj_y - float(img_cy)
            err_msg.z = 1.0
            self._error_pub.publish(err_msg)

            cv2.circle(cv_image, (int(obj_x), int(obj_y)), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (img_cx, img_cy), (int(obj_x), int(obj_y)), (0, 255, 255), 2)
            cv2.putText(
                cv_image,
                f'Target {best_label}: dx={err_msg.x:.1f}, dy={err_msg.y:.1f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            self._log_throttled(f'Detected {best_label} score={best_score:.2f} dx={err_msg.x:.1f} dy={err_msg.y:.1f}')
        else:
            self._publish_empty(msg.header)
            cv2.putText(
                cv_image,
                f'Searching: {self.target_class}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        self._pub.publish(detections_msg)
        self._publish_debug_image(cv_image, msg.header)

    def _publish_empty(self, header) -> None:
        detections_msg = Detection2DArray()
        detections_msg.header = header
        self._pub.publish(detections_msg)

        err_msg = Point()
        err_msg.x = 0.0
        err_msg.y = 0.0
        err_msg.z = 0.0
        self._error_pub.publish(err_msg)

    def _append_detection(self, detections_msg: Detection2DArray, label_name: str, score: float, box) -> None:
        detection = Detection2D()
        detection.bbox.center.position.x = float(box[0])
        detection.bbox.center.position.y = float(box[1])
        detection.bbox.size_x = float(box[2])
        detection.bbox.size_y = float(box[3])

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label_name
        hypothesis.hypothesis.score = score
        detection.results.append(hypothesis)
        detections_msg.detections.append(detection)

    def _draw_center_cross(self, image, cx: int, cy: int) -> None:
        cv2.line(image, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(image, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

    def _draw_detection(self, image, label_name: str, score: float, box) -> None:
        color = self._get_color(label_name)
        min_pt = (int(box[0] - box[2] / 2), int(box[1] - box[3] / 2))
        max_pt = (int(box[0] + box[2] / 2), int(box[1] + box[3] / 2))
        cv2.rectangle(image, min_pt, max_pt, color, 2)
        cv2.putText(
            image,
            f'{label_name} {score:.2f}',
            (min_pt[0], max(20, min_pt[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

    def _publish_debug_image(self, cv_image, header, status_text: str = '') -> None:
        if not self.publish_debug:
            return
        if status_text:
            cv2.putText(cv_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        dbg_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        dbg_msg.header = header
        self._dbg_pub.publish(dbg_msg)

    def _get_color(self, label: str):
        if label not in self._class_to_color:
            self._class_to_color[label] = (
                random.randint(0, 255),
                random.randint(0, 255),
                random.randint(0, 255),
            )
        return self._class_to_color[label]

    def _log_throttled(self, text: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_status_log_time >= period_sec:
            self.get_logger().info(text)
            self._last_status_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
