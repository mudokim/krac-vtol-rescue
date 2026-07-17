#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
from krac_interfaces.msg import TargetError
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import math
from ultralytics import YOLO

class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        self.bridge = CvBridge()

        self.declare_parameter('model', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('obb_confidence', 0.20)
        self.declare_parameter('tracking_hold_sec', 75.0)
        self.declare_parameter('inference_interval_sec', 0.25)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.obb_confidence = self.get_parameter('obb_confidence').get_parameter_value().double_value
        self.tracking_hold_sec = self.get_parameter('tracking_hold_sec').get_parameter_value().double_value
        self.inference_interval_sec = self.get_parameter('inference_interval_sec').get_parameter_value().double_value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.cmd_sub = self.create_subscription(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.cmd_cb, qos_profile_sensor_data)

        self.error_pub = self.create_publisher(TargetError, '/vision/target_error', 10)
        self.dbg_pub = self.create_publisher(Image, '/vision/dbg_image', 10)
        self.hold_timer = self.create_timer(0.1, self.hold_publish_cb)
        self.target_class = 'basket'
        self.target_sub = self.create_subscription(String, '/camera/set_target', self.target_cb, 10)
        self.reset_sub = self.create_subscription(Empty, '/vision/reset', self.reset_cb, 10)

        self.current_alt = 0.0
        self.current_cmd = Twist()
        self._last_status_log_time = 0.0
        self._last_detection_time = 0.0
        self._last_inference_time = 0.0
        self._last_target_publish_time = 0.0
        self._held_target_msg = None

        # 1. YOLO OBB 모델 로드
        model_path = self.get_parameter('model').get_parameter_value().string_value
        if not model_path:
            model_path = os.path.join(get_package_share_directory('krac_vision'), 'weights', 'best.pt')
        self.model = YOLO(model_path)

        # 💡 2. [추가] ArUco 마커 설정 (OpenCV 버전에 따른 호환성 처리)
        # 일반적으로 버티포트 착륙에는 5x5 사이즈의 마커를 많이 사용합니다.
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.img_size = 1024
        self.center_x = self.img_size / 2.0
        self.center_y = self.img_size / 2.0

        # 칼만 필터 초기화
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf_initialized = False

    def _reset_tracking(self):
        # KF 상태와 hold 기억을 통째로 버린다. kf_initialized=False 로 되돌리는 게
        # 핵심이다: 다음 실탐지에서 image_callback 이 statePre/statePost 를 그 관측에
        # 속도 0 으로 스냅시킨다. 이게 아니면 measurementNoiseCov(기본 1.0)가
        # processNoiseCov(0.03)보다 30배 커서, 오래 dead-reckoning 한 필터는 실탐지가
        # 재개돼도 쌓인 속도 성분 때문에 여러 프레임 동안 엉뚱한 좌표를 계속 낸다.
        self.kf_initialized = False
        self._held_target_msg = None
        self._last_detection_time = 0.0
        self._last_target_publish_time = 0.0

    def reset_cb(self, msg):
        # 하강을 처음부터 다시 시작하는 쪽(rescue 컨트롤러의 ESC 재시도 등)이 호출한다.
        # 리셋하지 않으면 tracking_hold_sec(75초) 창이 land -> 파지 -> 재상승 사이클을
        # 통째로 덮어서, 이전 하강 막판의 픽셀오차를 새 고도에 환산한 유령 타깃으로
        # 다시 내려가게 된다.
        self.get_logger().info("vision tracking reset requested: dropping KF/hold state.")
        self._reset_tracking()

    def target_cb(self, msg):
        new_target = msg.data.strip().lower()
        if not new_target or new_target == self.target_class:
            return
        self.get_logger().info(f"vision target changed: {self.target_class} -> {new_target}")
        self.target_class = new_target
        self._reset_tracking()

    def pose_cb(self, msg):
        self.current_alt = msg.pose.position.z

    def cmd_cb(self, msg):
        self.current_cmd = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        resized_img = cv2.resize(cv_image, (self.img_size, self.img_size))
        dbg_img = resized_img.copy()

        # 화면 십자선(Center) 그리기
        cv2.line(dbg_img, (int(self.center_x) - 30, int(self.center_y)), (int(self.center_x) + 30, int(self.center_y)), (0, 255, 0), 2)
        cv2.line(dbg_img, (int(self.center_x), int(self.center_y) - 30), (int(self.center_x), int(self.center_y) + 30), (0, 255, 0), 2)

        target_msg = TargetError()
        target_msg.is_detected = False
        target_msg.target_class = self.target_class
        target_msg.source = 'LOST'
        target_msg.confidence = 0.0
        target_msg.bbox_width = 0.0
        target_msg.bbox_height = 0.0
        target_msg.bbox_area = 0.0

        dist_px = 0.0
        theta_val = 0.0
        tracking_mode = "LOST"

        best_cx, best_cy, best_theta = 0.0, 0.0, 0.0
        best_bbox_w, best_bbox_h, best_conf = 0.0, 0.0, 0.0
        is_found = False

        # =======================================================
        # 🔍 1. ArUco 마커 탐지 (우선순위 1순위 - 초정밀)
        # =======================================================
        gray_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_params)
        except AttributeError:
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray_img)

        if ids is not None and len(ids) > 0:
            # 첫 번째 발견된 마커를 타겟으로 지정
            c = corners[0][0] # 4개 모서리: [top_left, top_right, bottom_right, bottom_left]
            best_cx = np.mean(c[:, 0])
            best_cy = np.mean(c[:, 1])

            # 상단 모서리(top_left -> top_right)를 기준으로 회전각(Theta) 도출
            dx = c[1][0] - c[0][0]
            dy = c[1][1] - c[0][1]
            best_theta = math.atan2(dy, dx)
            best_bbox_w = float(np.linalg.norm(c[1] - c[0]))
            best_bbox_h = float(np.linalg.norm(c[2] - c[1]))
            best_conf = 1.0

            is_found = True
            tracking_mode = f"ArUco (ID: {ids[0][0]})"

            # 디버그용 폴리곤 및 중심점 그리기 (마젠타 색상)
            cv2.aruco.drawDetectedMarkers(dbg_img, corners, ids)
            cv2.circle(dbg_img, (int(best_cx), int(best_cy)), 8, (255, 0, 255), -1)

        # =======================================================
        # 🔍 2. YOLO OBB 탐지 (우선순위 2순위 - ArUco가 안 보일 때)
        # =======================================================
        now = time.monotonic()
        recently_detected = self.kf_initialized and (now - self._last_detection_time) <= self.tracking_hold_sec
        should_run_yolo = (
            not is_found and
            (not recently_detected or (now - self._last_inference_time) >= self.inference_interval_sec)
        )

        if should_run_yolo:
            self._last_inference_time = now
            results = self.model(resized_img, imgsz=self.img_size, conf=self.obb_confidence, verbose=False)

            if len(results[0].obb) > 0:
                best_obb = None
                selected_conf = -1.0
                for obb in results[0].obb:
                    cls_id = int(obb.cls[0]) if obb.cls is not None else -1
                    label = str(self.model.names.get(cls_id, cls_id)).lower()
                    conf = float(obb.conf[0]) if obb.conf is not None else 0.0
                    if self.target_class not in ('all', label):
                        continue
                    if conf > selected_conf:
                        selected_conf = conf
                        best_obb = obb
                if best_obb is None:
                    best_obb = results[0].obb[0]
                    selected_conf = float(best_obb.conf[0]) if best_obb.conf is not None else 0.0
                yolo_cx, yolo_cy = float(best_obb.xywhr[0][0]), float(best_obb.xywhr[0][1])
                yolo_w, yolo_h = float(best_obb.xywhr[0][2]), float(best_obb.xywhr[0][3])
                yolo_theta = float(best_obb.xywhr[0][4])

                # 디버그용 폴리곤 그리기 (파란색)
                try:
                    obb_pts = best_obb.xyxyxyxy[0].cpu().numpy().astype(int)
                    cv2.polylines(dbg_img, [obb_pts], isClosed=True, color=(255, 0, 0), thickness=2)
                except Exception:
                    pass

                # 만약 ArUco를 못 찾았다면 YOLO의 좌표를 타겟으로 사용
                if not is_found:
                    best_cx = yolo_cx
                    best_cy = yolo_cy
                    best_theta = yolo_theta
                    best_bbox_w = yolo_w
                    best_bbox_h = yolo_h
                    best_conf = selected_conf
                    is_found = True
                    tracking_mode = "YOLO (OBB)"

        # =======================================================
        # 🎯 3. 칼만 필터(Kalman Filter) 업데이트 및 퍼블리시
        # =======================================================
        if is_found:
            if not self.kf_initialized:
                self.kf.statePre = np.array([[best_cx], [best_cy], [0], [0]], np.float32)
                self.kf.statePost = np.array([[best_cx], [best_cy], [0], [0]], np.float32)
                self.kf_initialized = True

            measurement = np.array([[np.float32(best_cx)], [np.float32(best_cy)]])
            self.kf.correct(measurement)
            predicted = self.kf.predict()

            filtered_cx, filtered_cy = predicted[0][0], predicted[1][0]
            theta_val = best_theta

            target_msg.pixel_err_x = filtered_cx - self.center_x
            target_msg.pixel_err_y = self.center_y - filtered_cy
            target_msg.yaw_err_rad = theta_val
            target_msg.is_detected = True
            target_msg.target_class = self.target_class
            target_msg.source = tracking_mode
            target_msg.confidence = float(best_conf)
            target_msg.bbox_width = float(best_bbox_w)
            target_msg.bbox_height = float(best_bbox_h)
            target_msg.bbox_area = float(best_bbox_w * best_bbox_h)
            self._last_detection_time = time.monotonic()

            dist_px = math.hypot(target_msg.pixel_err_x, target_msg.pixel_err_y)

            # 추적 타겟으로 선 긋기
            t_x, t_y = int(filtered_cx), int(filtered_cy)
            cv2.circle(dbg_img, (t_x, t_y), 6, (0, 0, 255), -1)
            cv2.line(dbg_img, (int(self.center_x), int(self.center_y)), (t_x, t_y), (0, 255, 255), 2)
            self._log_status(
                f"tracking={tracking_mode} err_px=({target_msg.pixel_err_x:.1f},"
                f"{target_msg.pixel_err_y:.1f}) yaw={target_msg.yaw_err_rad:.3f}"
            )

        else:
            if self.kf_initialized:
                predicted = self.kf.predict()
                target_msg.pixel_err_x = predicted[0][0] - self.center_x
                target_msg.pixel_err_y = self.center_y - predicted[1][0]
                recently_detected = (time.monotonic() - self._last_detection_time) <= self.tracking_hold_sec
                target_msg.is_detected = bool(recently_detected)
                target_msg.target_class = self.target_class
                target_msg.source = 'KF_HOLD' if recently_detected else 'LOST'

                t_x, t_y = int(predicted[0][0]), int(predicted[1][0])
                cv2.circle(dbg_img, (t_x, t_y), 6, (150, 150, 150), -1)
                if recently_detected:
                    dist_px = math.hypot(target_msg.pixel_err_x, target_msg.pixel_err_y)
                    cv2.putText(dbg_img, "KF HOLD", (t_x + 10, t_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                    self._log_status(
                        f"tracking=KF_HOLD err_px=({target_msg.pixel_err_x:.1f},"
                        f"{target_msg.pixel_err_y:.1f})"
                    )
                else:
                    cv2.putText(dbg_img, "LOST (KF Tracking)", (t_x + 10, t_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)
                    self._log_status("tracking=LOST")
            else:
                self._log_status("tracking=LOST")

        if target_msg.is_detected:
            self._held_target_msg = self._copy_target_msg(target_msg)
        self.error_pub.publish(target_msg)
        self._last_target_publish_time = time.monotonic()

        # =======================================================
        # 🖥️ 4. HUD (디버그 화면) 텍스트 출력
        # =======================================================
        status_color = (0, 255, 0) if target_msg.is_detected else (0, 0, 255)
        text_lines = [
            f"Tracker: {tracking_mode}",
            f"Alt (Ground): {self.current_alt:.2f} m",
            f"Center Dist: {dist_px:.1f} px",
            f"Target Theta: {theta_val:.3f} rad ({math.degrees(theta_val):.1f} deg)",
            f"Cmd_Vel X: {self.current_cmd.linear.x:.2f}, Y: {self.current_cmd.linear.y:.2f}",
            f"Cmd_Vel Z: {self.current_cmd.linear.z:.2f}, Yaw: {self.current_cmd.angular.z:.2f}"
        ]

        y_offset = 40
        for i, line in enumerate(text_lines):
            color = status_color if i == 0 else (0, 255, 255)
            cv2.putText(dbg_img, line, (22, y_offset + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)
            cv2.putText(dbg_img, line, (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            y_offset += 35

        self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(dbg_img, "bgr8"))

    def hold_publish_cb(self):
        if self._held_target_msg is None:
            return
        now = time.monotonic()
        if (now - self._last_detection_time) > self.tracking_hold_sec:
            return
        if (now - self._last_target_publish_time) < 0.08:
            return
        # 이건 정의상 '지금 보고 있는 것'이 아니라 마지막 탐지의 재발행이다. 원본이
        # 실탐지(YOLO/ArUco)였더라도 소비자가 실탐지로 오해하지 않게 source 를 덮어쓴다.
        held = self._copy_target_msg(self._held_target_msg)
        held.source = 'KF_HOLD'
        self.error_pub.publish(held)
        self._last_target_publish_time = now

    def _copy_target_msg(self, src):
        msg = TargetError()
        msg.is_detected = src.is_detected
        msg.pixel_err_x = src.pixel_err_x
        msg.pixel_err_y = src.pixel_err_y
        msg.yaw_err_rad = src.yaw_err_rad
        msg.target_class = src.target_class
        msg.source = src.source
        msg.confidence = src.confidence
        msg.bbox_width = src.bbox_width
        msg.bbox_height = src.bbox_height
        msg.bbox_area = src.bbox_area
        return msg

    def _log_status(self, text):
        now = time.monotonic()
        if now - self._last_status_log_time >= 1.0:
            self._last_status_log_time = now
            self.get_logger().info(text)

def main(args=None):
    rclpy.init(args=args)
    node = VisionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
