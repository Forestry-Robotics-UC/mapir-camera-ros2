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

import os
import time
from typing import Optional

from camera_info_manager import CameraInfoManager
from camera_info_manager.camera_info_manager import CameraInfoMissingError
import cv2
from cv_bridge import CvBridge
from mapir_camera_core import configure_v4l2_capture, open_v4l2_capture
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


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

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_period_s', 1.0)  # log periodic stats at this interval
        self.declare_parameter('video_device', '/dev/video0')  # '/dev/video0' or '0'
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('framerate', 30.0)
        self.declare_parameter('frame_id', 'mapir3_optical_frame')
        self.declare_parameter('camera_name', 'mapir3_ocn')
        self.declare_parameter('camera_info_url', '')  # file:///...
        self.declare_parameter('pixel_format', 'MJPG')  # MJPG or H264
        self.declare_parameter('use_gstreamer', False)  # use GStreamer pipeline
        self.declare_parameter('gstreamer_pipeline', '')  # custom pipeline string
        self.declare_parameter('qos_depth', 5)  # queue depth for pub/sub
        self.declare_parameter('qos_best_effort', True)  # BEST_EFFORT recommended for images

        # Read parameters
        self.debug = bool(self.get_parameter('debug').value)
        self.debug_period_s = float(self.get_parameter('debug_period_s').value)

        self.video_device = str(self.get_parameter('video_device').value)
        self.req_width = int(self.get_parameter('image_width').value)
        self.req_height = int(self.get_parameter('image_height').value)
        self.req_fps = float(self.get_parameter('framerate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_info_url = str(self.get_parameter('camera_info_url').value)
        self.pixel_format = str(self.get_parameter('pixel_format').value).upper()
        self.use_gstreamer = bool(self.get_parameter('use_gstreamer').value)
        self.gstreamer_pipeline = str(self.get_parameter('gstreamer_pipeline').value)

        self.qos_depth = max(1, int(self.get_parameter('qos_depth').value))
        self.qos_best_effort = bool(self.get_parameter('qos_best_effort').value)

        if self.req_fps <= 0.0:
            self.get_logger().warn('Invalid framerate; defaulting to 30 Hz')
            self.req_fps = 30.0

        # -----------------------
        # Debug banner
        # -----------------------
        if self.debug:
            self.get_logger().info(f'RUNNING FILE: {os.path.abspath(__file__)}')
            self.get_logger().info(
                'Params: '
                f'video_device={self.video_device}, size={self.req_width}x{self.req_height}, '
                f'fps={self.req_fps}, fmt={self.pixel_format}, frame_id={self.frame_id}, '
                f'camera_name={self.camera_name}, camera_info_url={self.camera_info_url!r}, '
                f'use_gstreamer={self.use_gstreamer}, '
                f'qos_best_effort={self.qos_best_effort}, qos_depth={self.qos_depth}, '
                f'debug_period_s={self.debug_period_s}'
            )

        # -----------------------
        # QoS (best practice for camera images is BEST_EFFORT)
        # -----------------------
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if self.qos_best_effort
            else ReliabilityPolicy.RELIABLE
        )
        self.pub_qos = QoSProfile(
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
        )

        if self.debug:
            self.get_logger().info(
                f'Publisher QoS: reliability={self.pub_qos.reliability.name}, '
                f'depth={self.pub_qos.depth}'
            )

        # -----------------------
        # Open camera (force V4L2)
        # -----------------------
        self.cap = self._open_camera(self.video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video device: {self.video_device}')
            raise RuntimeError('Camera open failed')

        self._configure_camera()

        # Validate first frame
        ok, test_frame = self.cap.read()
        if not ok or test_frame is None:
            self.get_logger().error(
                'Camera opened but no frames could be read. '
                'Check /dev/video permissions, device busy, or negotiated pixel format.'
            )
        else:
            self._update_dimensions_from_frame(test_frame)
            self.get_logger().info(
                f'First frame OK: shape={test_frame.shape}, dtype={test_frame.dtype}'
            )

        self.bridge = CvBridge()

        # -----------------------
        # Publishers
        # -----------------------
        self.image_pub = self.create_publisher(Image, 'image_raw', self.pub_qos)
        self.cinfo_pub = self.create_publisher(CameraInfo, 'camera_info', self.pub_qos)

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

        # Timer
        self.timer_period = 1.0 / self.req_fps
        self.timer = self.create_timer(self.timer_period, self.capture_and_publish)

        self.get_logger().info(
            f'MAPIR Survey3 Camera started: {self.req_width}x{self.req_height} '
            f'@ {self.req_fps:.1f} Hz ({self.video_device}), '
            f'fmt={self.pixel_format}, qos={self.pub_qos.reliability.name}'
        )

    def _open_camera(self, video_device: str) -> cv2.VideoCapture:
        """Open camera using V4L2 backend. Accepts '/dev/videoX' or numeric index."""
        if not self.use_gstreamer:
            return open_v4l2_capture(video_device)

        pipeline = self._build_gstreamer_pipeline()
        if self.debug:
            self.get_logger().info(f'GStreamer pipeline: {pipeline}')
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    def _configure_camera(self) -> None:
        """Configure pixel format, resolution, and fps; log negotiated values."""
        if self.use_gstreamer:
            self.width = self.req_width
            self.height = self.req_height
            self.get_logger().info('GStreamer capture enabled; skipping V4L2 negotiation')
            return

        if self.pixel_format not in ('MJPG', 'H264'):
            self.get_logger().warn(
                f'Unsupported pixel_format={self.pixel_format!r}, forcing MJPG'
            )
            self.pixel_format = 'MJPG'

        negotiation = configure_v4l2_capture(
            self.cap,
            req_width=self.req_width,
            req_height=self.req_height,
            req_fps=self.req_fps,
            pixel_format=self.pixel_format,
        )

        self.width = negotiation.width if negotiation.width > 0 else self.req_width
        self.height = negotiation.height if negotiation.height > 0 else self.req_height

        self.get_logger().info(
            f'Negotiated capture: {negotiation.width}x{negotiation.height} '
            f'@ {negotiation.fps:.2f} Hz, '
            f'FOURCC={negotiation.fourcc_str!r}'
        )

        if self.debug and negotiation.backend_id is not None:
            self.get_logger().debug(f'OpenCV backend id: {negotiation.backend_id}')

    def _build_gstreamer_pipeline(self) -> str:
        """Return a GStreamer pipeline string for USB camera capture."""
        custom = self.gstreamer_pipeline.strip()
        if custom:
            return custom

        device = self.video_device
        width = self.req_width
        height = self.req_height
        fps = int(round(self.req_fps))

        if self.pixel_format == 'H264':
            return (
                f'v4l2src device={device} ! '
                f'video/x-h264,width={width},height={height},framerate={fps}/1 ! '
                'h264parse ! '
                'avdec_h264 ! '
                'videoconvert ! '
                'video/x-raw,format=BGR ! '
                'appsink drop=true max-buffers=1 sync=false'
            )

        return (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height},framerate={fps}/1 ! '
            'jpegdec ! '
            'videoconvert ! '
            'video/x-raw,format=BGR ! '
            'appsink drop=true max-buffers=1 sync=false'
        )

    def _update_dimensions_from_frame(self, frame) -> None:
        """Update width/height from the first frame if needed."""
        try:
            height, width = frame.shape[:2]
        except Exception:
            return
        if width > 0 and height > 0:
            self.width = int(width)
            self.height = int(height)

    def _default_camerainfo(self) -> CameraInfo:
        """Fallback CameraInfo when no calibration is available."""
        msg = CameraInfo()
        msg.width = int(self.width)
        msg.height = int(self.height)
        return msg

    def _get_camerainfo_safe(self) -> CameraInfo:
        """Return calibrated CameraInfo if available, otherwise a valid default."""
        try:
            return self.cinfo_manager.getCameraInfo()
        except CameraInfoMissingError:
            if self.debug:
                self.get_logger().debug('CameraInfo missing; using default CameraInfo()')
            return self._default_camerainfo()
        except Exception as ex:
            self.get_logger().warn(
                f'getCameraInfo() failed; using default CameraInfo. Reason: {ex}'
            )
            return self._default_camerainfo()

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

        cinfo_msg = self._get_camerainfo_safe()
        cinfo_msg.header.stamp = stamp
        cinfo_msg.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.cinfo_pub.publish(cinfo_msg)
        self._pub_frames += 1

        # Periodic debug stats (throttled)
        if self.debug:
            now = time.time()
            if (now - self._last_stats_log_t) >= self.debug_period_s:
                frames_since = self._pub_frames - self._last_stats_pub_frames
                dt = max(1e-6, now - self._last_stats_log_t)
                est_hz = frames_since / dt

                self.get_logger().debug(
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
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapirSurvey3CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
