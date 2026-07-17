#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
krac_gimbal.camera_node
=======================
SIYI A8 mini 의 RTSP 영상을 받아 ROS 2 `sensor_msgs/Image` 로 퍼블리시하는
카메라 브리지 노드.

A8 mini 는 "짐벌 각도 제어(UDP 37260)" 와 "영상(RTSP 8554)" 이 물리적으로는
한 카메라지만 소프트웨어 경로가 완전히 분리돼 있다. gimbal_node 가 각도를
담당하고, 이 노드가 영상을 담당한다.

    RTSP  rtsp://192.168.144.25:8554/main.264
      │
      ▼
  camera_node ──/camera/image_raw (bgr8)──▶ krac_vision (YOLO 트래커)

원리:
  - OpenCV VideoCapture 로 RTSP 스트림을 연다.
      * backend=ffmpeg  : 이식성 높음(기본). rtsp_transport/저지연 옵션 적용.
      * backend=gstreamer: Jetson HW 디코드(nvv4l2decoder) 파이프라인으로 저지연.
  - 별도 캡처 스레드가 프레임을 계속 읽어 최신 프레임만 즉시 퍼블리시(저지연).
  - 스트림이 끊기면 backoff 로 자동 재연결한다.

의존: cv_bridge, python3-opencv (Jetson 에는 보통 기본 설치되어 있음)
"""
import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
    _IMPORT_ERR = None
# ImportError(미설치) 뿐 아니라 cv_bridge 의 NumPy ABI 불일치(AttributeError:
# _ARRAY_API) 등 import 시점 오류도 잡아 친절한 메시지로 종료시킨다.
except Exception as e:  # 빌드 환경엔 없어도 되고, 실행(Jetson)에서만 필요
    cv2 = None
    CvBridge = None
    _IMPORT_ERR = e


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera')
        # ffmpeg(기본, 이식성) 또는 gstreamer(Jetson HW 디코드)
        self.declare_parameter('backend', 'ffmpeg')
        # ffmpeg 전송 방식: tcp(권장, 손실에 강함) 또는 udp(더 낮은 지연)
        self.declare_parameter('rtsp_transport', 'tcp')
        # gstreamer 지터버퍼(ms). 낮을수록 저지연·손실에 약함
        self.declare_parameter('gst_latency_ms', 100)
        # H.264(main.264) 기본. sub 스트림이 H.265면 'h265' 로.
        self.declare_parameter('codec', 'h264')
        # 사용자 지정 GStreamer 파이프라인(비면 codec/latency 로 자동 생성)
        self.declare_parameter('gst_pipeline', '')
        # 재연결 backoff(초)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        # 프레임을 못 받는 상태가 이 시간(초)을 넘으면 재연결
        self.declare_parameter('read_timeout_sec', 5.0)

        gp = self.get_parameter
        self.rtsp_url = str(gp('rtsp_url').value)
        self.image_topic = str(gp('image_topic').value)
        self.frame_id = str(gp('frame_id').value)
        self.backend = str(gp('backend').value).lower()
        self.rtsp_transport = str(gp('rtsp_transport').value).lower()
        self.gst_latency_ms = int(gp('gst_latency_ms').value)
        self.codec = str(gp('codec').value).lower()
        self.gst_pipeline = str(gp('gst_pipeline').value)
        self.reconnect_delay = float(gp('reconnect_delay_sec').value)
        self.read_timeout = float(gp('read_timeout_sec').value)

        if cv2 is None:
            self.get_logger().error(
                f'opencv/cv_bridge import 실패: {_IMPORT_ERR}. '
                'Jetson 에서 `sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv` '
                '등으로 설치하세요.')
            raise SystemExit(1)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, 10)

        # ---------------- 캡처 스레드 시작 ----------------
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'camera_node 시작. RTSP={self.rtsp_url} (backend={self.backend}) '
            f'→ {self.image_topic}')

    # ---------------- 캡처 열기 ----------------
    def _build_gst_pipeline(self):
        if self.gst_pipeline:
            return self.gst_pipeline
        depay = 'rtph265depay ! h265parse' if self.codec == 'h265' \
            else 'rtph264depay ! h264parse'
        # Jetson HW 디코드(nvv4l2decoder) → BGR appsink. 최신 프레임만(drop=true).
        return (
            f'rtspsrc location={self.rtsp_url} protocols={self.rtsp_transport} '
            f'latency={self.gst_latency_ms} ! {depay} ! nvv4l2decoder ! '
            'nvvidconv ! video/x-raw,format=BGRx ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true max-buffers=1 sync=false')

    def _open_capture(self):
        if self.backend == 'gstreamer':
            pipeline = self._build_gst_pipeline()
            self.get_logger().info(f'GStreamer 파이프라인으로 연결: {pipeline}')
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            # FFMPEG 저지연 옵션. VideoCapture 열기 직전에 설정해야 반영됨.
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
                f'rtsp_transport;{self.rtsp_transport}|fflags;nobuffer|max_delay;0')
            self.get_logger().info(
                f'FFMPEG 백엔드로 연결(rtsp_transport={self.rtsp_transport})')
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 최소화(저지연)
            except Exception:
                pass
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    # ---------------- 캡처 루프 ----------------
    def _capture_loop(self):
        cap = None
        last_ok = self.get_clock().now()
        while not self._stop.is_set() and rclpy.ok():
            if cap is None:
                cap = self._open_capture()
                if cap is None:
                    self.get_logger().warn(
                        f'RTSP 열기 실패 — {self.reconnect_delay:.0f}s 후 재시도 '
                        f'({self.rtsp_url})')
                    self._stop.wait(self.reconnect_delay)
                    continue
                self.get_logger().info('RTSP 스트림 연결 성공')
                last_ok = self.get_clock().now()

            ok, frame = cap.read()
            now = self.get_clock().now()
            if not ok or frame is None:
                # 일시적 실패는 넘기되, read_timeout 초과 시 재연결
                if (now - last_ok).nanoseconds > self.read_timeout * 1e9:
                    self.get_logger().warn('프레임 수신 끊김 — 재연결합니다.')
                    cap.release()
                    cap = None
                continue

            last_ok = now
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
                continue
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

        if cap is not None:
            cap.release()

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
