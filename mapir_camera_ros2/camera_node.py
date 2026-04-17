#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: GNU General Public License v3.0 (GPL-3.0-only)
# Repository: mapir_survey3
#
# Description:
#   ROS 2 Jazzy camera driver for MAPIR Survey3 cameras.
#   Publishes /<ns>/image_raw and /<ns>/camera_info at up to 60 Hz.
#   Uses OpenCV with V4L2 backend and forces MJPG/H264 (device dependent).
#   Uses CameraInfoManager; publishes default CameraInfo if calibration missing.
#   Includes best-practice debug logging controlled by a 'debug' parameter.
#
# Notes:
#   - Best-practice QoS for camera streams is BEST_EFFORT (RViz/rqt compatible).
#   - Debug logs are throttled with debug_period_s to avoid console spam at 30–60 Hz.
#

from __future__ import annotations

import threading
import time
from typing import Optional

from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from mapir_camera_ros2.camera_capture import (
    apply_runtime_vignette_correction,
    configure_camera,
    configure_vignette_if_enabled,
    get_camerainfo_safe,
    open_camera,
    update_dimensions_from_frame,
)
from mapir_camera_ros2.camera_controls import apply_uvc_controls_if_requested
from mapir_camera_ros2.camera_metadata import (
    build_frame_metadata,
    publish_metadata,
    shutdown_metadata,
    start_metadata_reader,
)
from mapir_camera_ros2.camera_params import (
    apply_camera_parameters,
    build_publisher_qos,
    declare_camera_parameters,
    load_camera_parameters,
    log_parameter_banner,
)


class MapirSurvey3CameraNode(Node):
    """
    MAPIR Survey3 camera driver node.

    Publishes (under the node namespace):
      - image_raw   (sensor_msgs/Image)
      - camera_info (sensor_msgs/CameraInfo)

    Debugging:
      - debug:=true enables additional logging.
      - debug_period_s throttles periodic debug output.
    """

    def __init__(self) -> None:
        super().__init__('mapir_camera')

        declare_camera_parameters(self)
        params = load_camera_parameters(self)
        apply_camera_parameters(self, params)

        if self.req_fps <= 0.0:
            self.get_logger().warn('Invalid framerate; defaulting to 30 Hz')
            self.req_fps = 30.0

        # -----------------------
        # Debug banner
        # -----------------------
        if self.debug:
            log_parameter_banner(self, params)

        # -----------------------
        # QoS (best practice for camera images is BEST_EFFORT)
        # -----------------------
        self.pub_qos = build_publisher_qos(params)

        if self.debug:
            self.get_logger().info(
                f'Publisher QoS: reliability={self.pub_qos.reliability.name}, '
                f'depth={self.pub_qos.depth}'
            )

        # -----------------------
        # Open camera (force V4L2)
        # -----------------------
        self.cap = open_camera(self, self.video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video device: {self.video_device}')
            raise RuntimeError('Camera open failed')

        configure_camera(self)

        # Validate first frame
        ok, test_frame = self.cap.read()
        if not ok or test_frame is None:
            self.get_logger().error(
                'Camera opened but no frames could be read. '
                'Check /dev/video permissions, device busy, or negotiated pixel format.'
            )
        else:
            update_dimensions_from_frame(self, test_frame)
            self.get_logger().info(
                f'First frame OK: shape={test_frame.shape}, dtype={test_frame.dtype}'
            )

        self.bridge = CvBridge()

        # -----------------------
        # Publishers
        # -----------------------
        self.image_pub = self.create_publisher(Image, 'image_raw', self.pub_qos)
        self.cinfo_pub = self.create_publisher(CameraInfo, 'camera_info', self.pub_qos)
        self.metadata_pub: Optional[rclpy.publisher.Publisher] = None
        if self.metadata_enabled:
            self.metadata_pub = self.create_publisher(String, self.metadata_topic, self.pub_qos)

        # -----------------------
        # CameraInfoManager
        # -----------------------
        self.cinfo_manager = CameraInfoManager(
            node=self,
            cname=self.camera_name,
            url=self.camera_info_url,
        )

        loaded = False
        if self.camera_info_url:
            self.get_logger().info(f'camera calibration URL: {self.camera_info_url}')

        try:
            loaded = self.cinfo_manager.loadCameraInfo()
        except Exception as ex:
            self.get_logger().warn(f'CameraInfo loadCameraInfo() raised: {ex}')

        if self.camera_info_url and loaded:
            self.get_logger().info(f'Loaded camera calibration: {self.camera_info_url}')
        else:
            self.get_logger().warn(
                'No valid calibration loaded; publishing default (uncalibrated) CameraInfo. '
                'Set "camera_info_url" to a file:// YAML to enable calibration.'
            )

        # -----------------------
        # Runtime stats (debug) — separate throttles
        # -----------------------
        self._fail_reads = 0
        self._pub_frames = 0

        now = time.time()
        self._last_stats_log_t = now
        self._last_fail_log_t = 0.0
        self._last_stats_pub_frames = 0

        self._last_frame_t: Optional[float] = None
        self._max_dt = 0.0
        self._metadata_lock = threading.Lock()
        self._latest_metadata: dict = {}
        self._metadata_thread: Optional[threading.Thread] = None
        self._stop_metadata = threading.Event()
        self._metadata_proc: Optional[subprocess.Popen] = None
        self._metadata_bytes = 0
        self._metadata_records = 0
        self._metadata_log_file: Optional[object] = None
        self._metadata_log_records_since_flush = 0
        self._flat_fields_bgr: Optional[tuple] = None
        self._uvc_controls_device_applied = ''
        self._uvc_controls_requested: dict[str, int] = {}
        self._uvc_controls_applied: dict[str, str] = {}
        self._uvc_controls_locked = False

        apply_uvc_controls_if_requested(self)

        if self.metadata_enabled:
            start_metadata_reader(self)

        configure_vignette_if_enabled(self)

        # Timer
        self.timer_period = 1.0 / self.req_fps
        self.timer = self.create_timer(self.timer_period, self.capture_and_publish)

        self.get_logger().info(
            f'MAPIR Survey3 Camera started: {self.req_width}x{self.req_height} '
            f'@ {self.req_fps:.1f} Hz ({self.video_device}), '
            f'fmt={self.pixel_format}, qos={self.pub_qos.reliability.name}'
        )

    def capture_and_publish(self) -> None:
        """Timer callback: capture one frame and publish Image + CameraInfo."""
        ret, frame = self.cap.read()

        # Read failure (throttled independently)
        if not ret or frame is None:
            self._fail_reads += 1
            if self.debug:
                now = time.time()
                if (now - self._last_fail_log_t) >= self.debug_period_s:
                    self.get_logger().warn(
                        f'Camera read failed (fail_reads={self._fail_reads}) '
                        f'(device={self.video_device}, fmt={self.pixel_format})'
                    )
                    self._last_fail_log_t = now
            return

        if self.vignette_enabled and self._flat_fields_bgr is not None:
            frame = apply_runtime_vignette_correction(self, frame)

        # Timing stats
        now_t = time.time()
        if self._last_frame_t is not None:
            dt = now_t - self._last_frame_t
            if dt > self._max_dt:
                self._max_dt = dt
        self._last_frame_t = now_t

        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id

        cinfo_msg = get_camerainfo_safe(self)
        cinfo_msg.header.stamp = stamp
        cinfo_msg.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.cinfo_pub.publish(cinfo_msg)
        self._pub_frames += 1
        if self.metadata_enabled:
            payload = build_frame_metadata(self, stamp, img_msg)
            publish_metadata(self, payload)

        # Periodic debug stats (throttled)
        if self.debug:
            now = time.time()
            if (now - self._last_stats_log_t) >= self.debug_period_s:
                frames_since = self._pub_frames - self._last_stats_pub_frames
                dt = max(1e-6, now - self._last_stats_log_t)
                est_hz = frames_since / dt

                self.get_logger().info(
                    'Stats: '
                    f'published_frames={self._pub_frames}, '
                    f'fail_reads={self._fail_reads}, '
                    f'est_pub_hz={est_hz:.2f}, '
                    f'timer_period={self.timer_period:.4f}s, '
                    f'max_inter_frame_dt={self._max_dt:.4f}s'
                )

                self._last_stats_pub_frames = self._pub_frames
                self._last_stats_log_t = now

    def destroy_node(self):
        """Ensure camera resource is released cleanly."""
        try:
            shutdown_metadata(self)
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapirSurvey3CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
