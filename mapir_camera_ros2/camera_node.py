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

import json
import os
import shutil
import struct
import subprocess
import threading
import time
from typing import Optional

from camera_info_manager import CameraInfoManager
from camera_info_manager.camera_info_manager import CameraInfoMissingError
import cv2
from cv_bridge import CvBridge
from mapir_camera_core import (
    apply_vignette_correction,
    configure_v4l2_capture,
    load_vignette_images,
    open_v4l2_capture,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


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
        self.declare_parameter('uvc_controls_enabled', False)  # lock camera controls via v4l2-ctl
        self.declare_parameter('uvc_controls_device', '')  # explicit device for controls; empty uses video_device
        self.declare_parameter('auto_exposure_mode', -1)  # -1 keeps current, 1 manual on many UVC cameras
        self.declare_parameter('exposure_time_absolute', -1)  # -1 keeps current
        self.declare_parameter('gain', -1)  # -1 keeps current
        self.declare_parameter('exposure_dynamic_framerate', -1)  # -1 keeps current, 0 disables dynamic fps
        self.declare_parameter('white_balance_automatic', -1)  # -1 keeps current, 0 manual/1 auto
        self.declare_parameter('white_balance_temperature', -1)  # -1 keeps current
        self.declare_parameter('power_line_frequency', -1)  # -1 keeps current
        self.declare_parameter(
            'metadata_enabled', False
        )  # publish metadata topic from /dev/videoX
        self.declare_parameter(
            'metadata_device', '/dev/video1'
        )  # UVC metadata node (e.g. /dev/video5)
        self.declare_parameter('metadata_topic', 'metadata')  # relative output topic
        self.declare_parameter('metadata_log_path', '')  # optional JSONL output path
        self.declare_parameter('metadata_log_flush_every_n', 30)  # flush JSONL every N records
        self.declare_parameter(
            'vignette_enabled', False
        )  # apply per-channel flat-field correction
        self.declare_parameter(
            'vignette_flatfield_b_path', ''
        )  # absolute path to B flat-field image
        self.declare_parameter(
            'vignette_flatfield_g_path', ''
        )  # absolute path to G flat-field image
        self.declare_parameter(
            'vignette_flatfield_r_path', ''
        )  # absolute path to R flat-field image
        self.declare_parameter('vignette_dark_current_b', 0)  # dark-current offset for B channel
        self.declare_parameter('vignette_dark_current_g', 0)  # dark-current offset for G channel
        self.declare_parameter('vignette_dark_current_r', 0)  # dark-current offset for R channel

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
        self.uvc_controls_enabled = bool(self.get_parameter('uvc_controls_enabled').value)
        self.uvc_controls_device = str(self.get_parameter('uvc_controls_device').value)
        self.auto_exposure_mode = int(self.get_parameter('auto_exposure_mode').value)
        self.exposure_time_absolute = int(self.get_parameter('exposure_time_absolute').value)
        self.gain = int(self.get_parameter('gain').value)
        self.exposure_dynamic_framerate = int(
            self.get_parameter('exposure_dynamic_framerate').value
        )
        self.white_balance_automatic = int(self.get_parameter('white_balance_automatic').value)
        self.white_balance_temperature = int(self.get_parameter('white_balance_temperature').value)
        self.power_line_frequency = int(self.get_parameter('power_line_frequency').value)
        self.metadata_enabled = bool(self.get_parameter('metadata_enabled').value)
        self.metadata_device = str(self.get_parameter('metadata_device').value)
        self.metadata_topic = str(self.get_parameter('metadata_topic').value)
        self.metadata_log_path = str(self.get_parameter('metadata_log_path').value)
        self.metadata_log_flush_every_n = max(
            1, int(self.get_parameter('metadata_log_flush_every_n').value)
        )
        self.vignette_enabled = bool(self.get_parameter('vignette_enabled').value)
        self.vignette_flatfield_b_path = str(
            self.get_parameter('vignette_flatfield_b_path').value
        )
        self.vignette_flatfield_g_path = str(
            self.get_parameter('vignette_flatfield_g_path').value
        )
        self.vignette_flatfield_r_path = str(
            self.get_parameter('vignette_flatfield_r_path').value
        )
        self.vignette_dark_current = (
            int(self.get_parameter('vignette_dark_current_b').value),
            int(self.get_parameter('vignette_dark_current_g').value),
            int(self.get_parameter('vignette_dark_current_r').value),
        )

        if self.req_fps <= 0.0:
            self.get_logger().warn('Invalid framerate; defaulting to 30 Hz')
            self.req_fps = 30.0

        # -----------------------
        # Debug banner
        # -----------------------
        if self.debug:
            self.get_logger().info(
                'Parameter priority: launch/CLI overrides > params file > node defaults'
            )
            self.get_logger().info(f'RUNNING FILE: {os.path.abspath(__file__)}')
            self.get_logger().info(
                'Params: '
                f'video_device={self.video_device}, size={self.req_width}x{self.req_height}, '
                f'fps={self.req_fps}, fmt={self.pixel_format}, frame_id={self.frame_id}, '
                f'camera_name={self.camera_name}, camera_info_url={self.camera_info_url!r}, '
                f'use_gstreamer={self.use_gstreamer}, '
                f'qos_best_effort={self.qos_best_effort}, qos_depth={self.qos_depth}, '
                f'uvc_controls_enabled={self.uvc_controls_enabled}, '
                f'uvc_controls_device={self.uvc_controls_device!r}, '
                f'auto_exposure_mode={self.auto_exposure_mode}, '
                f'exposure_time_absolute={self.exposure_time_absolute}, '
                f'gain={self.gain}, '
                f'exposure_dynamic_framerate={self.exposure_dynamic_framerate}, '
                f'white_balance_automatic={self.white_balance_automatic}, '
                f'white_balance_temperature={self.white_balance_temperature}, '
                f'power_line_frequency={self.power_line_frequency}, '
                f'metadata_enabled={self.metadata_enabled}, metadata_device={self.metadata_device}, '
                f'metadata_topic={self.metadata_topic}, metadata_log_path={self.metadata_log_path!r}, '
                f'metadata_log_flush_every_n={self.metadata_log_flush_every_n}, '
                f'vignette_enabled={self.vignette_enabled}, '
                f'vignette_flatfield_b_path={self.vignette_flatfield_b_path!r}, '
                f'vignette_flatfield_g_path={self.vignette_flatfield_g_path!r}, '
                f'vignette_flatfield_r_path={self.vignette_flatfield_r_path!r}, '
                f'vignette_dark_current={self.vignette_dark_current}, '
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

        self._apply_uvc_controls_if_requested()

        if self.metadata_enabled:
            self._start_metadata_reader()

        self._configure_vignette_if_enabled()

        # Timer
        self.timer_period = 1.0 / self.req_fps
        self.timer = self.create_timer(self.timer_period, self.capture_and_publish)

        self.get_logger().info(
            f'MAPIR Survey3 Camera started: {self.req_width}x{self.req_height} '
            f'@ {self.req_fps:.1f} Hz ({self.video_device}), '
            f'fmt={self.pixel_format}, qos={self.pub_qos.reliability.name}'
        )

    @staticmethod
    def _parse_uvch_record(record: bytes) -> Optional[dict]:
        """Parse one uvc_meta_buf-like 22-byte record into a dict."""
        if len(record) != 22:
            return None
        ns, sof, header_len, header_flags = struct.unpack_from('<QHBB', record, 0)
        if header_len < 2 or header_len > 12:
            return None

        payload = record[12:22]
        metadata = {
            'uvch_ns': int(ns),
            'uvch_sof': int(sof),
            'uvc_header_length': int(header_len),
            'uvc_header_flags': int(header_flags),
            'fid': int(header_flags & 0x01),
            'eof': int((header_flags >> 1) & 0x01),
            'pts_present': bool(header_flags & 0x04),
            'scr_present': bool(header_flags & 0x08),
            'eoh': bool(header_flags & 0x80),
        }

        if metadata['pts_present'] and len(payload) >= 6:
            metadata['uvc_pts'] = int(struct.unpack_from('<I', payload, 2)[0])
        if metadata['scr_present'] and len(payload) >= 12:
            metadata['uvc_scr_stc'] = int(struct.unpack_from('<I', payload, 6)[0])
            metadata['uvc_scr_sof'] = int(struct.unpack_from('<H', payload, 10)[0])
        return metadata

    def _start_metadata_reader(self) -> None:
        """Start background reader for the UVC metadata capture device."""
        if not self.metadata_topic:
            self.get_logger().warn('metadata_enabled=true but metadata_topic is empty; disabling metadata publisher')
            self.metadata_enabled = False
            return
        if shutil.which('v4l2-ctl') is None:
            self.get_logger().warn(
                'metadata_enabled=true but v4l2-ctl is not available in PATH. '
                'Continuing without metadata capture.'
            )
            self.metadata_enabled = False
            return

        if self.metadata_log_path:
            try:
                log_dir = os.path.dirname(self.metadata_log_path)
                if log_dir:
                    os.makedirs(log_dir, exist_ok=True)
                self._metadata_log_file = open(self.metadata_log_path, 'a', encoding='utf-8')
            except OSError as ex:
                self.get_logger().warn(f'Could not open metadata_log_path={self.metadata_log_path!r}: {ex}')

        cmd = [
            'v4l2-ctl',
            '-d',
            self.metadata_device,
            '--stream-mmap',
            '--stream-to=-',
        ]
        try:
            self._metadata_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError as ex:
            self.get_logger().warn(
                f'Could not start metadata stream command ({cmd}): {ex}. '
                'Continuing without metadata capture.'
            )
            self.metadata_enabled = False
            return

        self._metadata_thread = threading.Thread(
            target=self._metadata_reader_loop,
            name='mapir_uvch_reader',
            daemon=True,
        )
        self._metadata_thread.start()
        self.get_logger().info(
            f'Metadata capture enabled: device={self.metadata_device}, topic={self.metadata_topic}'
        )

    def _metadata_reader_loop(self) -> None:
        """Read and parse metadata records from v4l2 metadata stream."""
        if self._metadata_proc is None or self._metadata_proc.stdout is None:
            return
        stream = self._metadata_proc.stdout
        buffer = bytearray()
        try:
            while not self._stop_metadata.is_set():
                chunk = stream.read(4096)
                if not chunk:
                    if self._metadata_proc.poll() is not None:
                        break
                    time.sleep(0.002)
                    continue
                self._metadata_bytes += len(chunk)
                buffer.extend(chunk)
                while len(buffer) >= 22:
                    record = bytes(buffer[:22])
                    del buffer[:22]
                    parsed = self._parse_uvch_record(record)
                    if parsed is None:
                        continue
                    self._metadata_records += 1
                    with self._metadata_lock:
                        self._latest_metadata = parsed
        except Exception as ex:  # pragma: no cover - runtime device errors
            self.get_logger().warn(f'Metadata reader stopped: {ex}')
        finally:
            if self._metadata_proc is not None and self._metadata_proc.poll() is not None:
                self.get_logger().warn(
                    f'Metadata stream exited with code {self._metadata_proc.returncode}. '
                    'Continuing without new metadata records.'
                )

    def _build_frame_metadata(self, stamp_msg, image_msg: Image) -> dict:
        """Build a metadata payload aligned with the published image frame."""
        metadata = {
            'schema_version': 1,
            'source': 'mapir_camera_ros2',
            'frame_id': self.frame_id,
            'stamp_sec': int(stamp_msg.sec),
            'stamp_nanosec': int(stamp_msg.nanosec),
            'image_width': int(image_msg.width),
            'image_height': int(image_msg.height),
            'pixel_format': self.pixel_format,
            'video_device': self.video_device,
            'metadata_device': self.metadata_device if self.metadata_enabled else '',
            'published_frame_count': int(self._pub_frames),
            'uvc_controls_device': self._uvc_controls_device_applied,
            'uvc_controls_locked': bool(self._uvc_controls_locked),
            'uvc_controls_requested': dict(self._uvc_controls_requested),
            'uvc_controls_applied': dict(self._uvc_controls_applied),
        }
        if self.metadata_enabled:
            with self._metadata_lock:
                latest = dict(self._latest_metadata)
            metadata.update(latest)
            metadata['metadata_records_seen'] = int(self._metadata_records)
            metadata['metadata_bytes_seen'] = int(self._metadata_bytes)
        return metadata

    def _publish_metadata(self, payload: dict) -> None:
        """Publish metadata JSON and optionally append a JSONL record."""
        if self.metadata_pub is None:
            return
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.metadata_pub.publish(msg)
        if self._metadata_log_file is not None:
            self._metadata_log_file.write(json.dumps(payload) + '\n')
            self._metadata_log_records_since_flush += 1
            if self._metadata_log_records_since_flush >= self.metadata_log_flush_every_n:
                self._metadata_log_file.flush()
                self._metadata_log_records_since_flush = 0

    def _configure_vignette_if_enabled(self) -> None:
        """Load vignette flat-fields once when vignette correction is enabled."""
        if not self.vignette_enabled:
            return

        paths = [
            self.vignette_flatfield_b_path.strip(),
            self.vignette_flatfield_g_path.strip(),
            self.vignette_flatfield_r_path.strip(),
        ]
        if not all(paths):
            self.get_logger().warn('vignette_enabled=true but flat-field paths are incomplete')
            self.get_logger().warn('Disabling vignette correction')
            self.vignette_enabled = False
            return

        try:
            flat_fields = load_vignette_images(paths)
        except Exception as ex:
            self.get_logger().warn(f'Failed to load vignette flat-fields: {ex}')
            self.get_logger().warn('Disabling vignette correction')
            self.vignette_enabled = False
            return

        expected_shape = (int(self.height), int(self.width))
        for idx, flat_field in enumerate(flat_fields):
            if flat_field.shape != expected_shape:
                self.get_logger().warn(
                    f'Flat-field {idx} shape={flat_field.shape} does not match '
                    f'camera frame shape={expected_shape}'
                )
                self.get_logger().warn('Disabling vignette correction')
                self.vignette_enabled = False
                return

        self._flat_fields_bgr = flat_fields
        self.get_logger().info('Vignette correction enabled')

    def _apply_uvc_controls_if_requested(self) -> None:
        """Optionally lock UVC controls once at startup via v4l2-ctl."""
        if not self.uvc_controls_enabled:
            return

        if shutil.which('v4l2-ctl') is None:
            self.get_logger().warn('uvc_controls_enabled=true but v4l2-ctl not found')
            return

        controls_device = self.uvc_controls_device.strip() or self.video_device
        if not controls_device:
            self.get_logger().warn('No device available for UVC controls')
            return
        self._uvc_controls_device_applied = controls_device

        requested = {
            'auto_exposure': self.auto_exposure_mode,
            'exposure_time_absolute': self.exposure_time_absolute,
            'gain': self.gain,
            'exposure_dynamic_framerate': self.exposure_dynamic_framerate,
            'white_balance_automatic': self.white_balance_automatic,
            'white_balance_temperature': self.white_balance_temperature,
            'power_line_frequency': self.power_line_frequency,
        }
        requested = {k: v for k, v in requested.items() if int(v) >= 0}
        self._uvc_controls_requested = dict(requested)
        if not requested:
            self.get_logger().warn(
                'uvc_controls_enabled=true but no control values set (all < 0); skipping lock'
            )
            return

        set_arg = ','.join(f'{k}={v}' for k, v in requested.items())
        set_cmd = ['v4l2-ctl', '-d', controls_device, '-c', set_arg]
        set_proc = subprocess.run(
            set_cmd,
            capture_output=True,
            text=True,
            check=False,
        )
        if set_proc.returncode != 0:
            stderr = set_proc.stderr.strip()
            self.get_logger().warn(
                f'Failed to set UVC controls on {controls_device}: {stderr or "unknown error"}'
            )
            return

        get_arg = ','.join(requested.keys())
        get_cmd = ['v4l2-ctl', '-d', controls_device, f'--get-ctrl={get_arg}']
        get_proc = subprocess.run(
            get_cmd,
            capture_output=True,
            text=True,
            check=False,
        )
        if get_proc.returncode != 0:
            stderr = get_proc.stderr.strip()
            self.get_logger().warn(
                f'Failed to read back UVC controls on {controls_device}: {stderr or "unknown error"}'
            )
            return

        applied: dict[str, str] = {}
        for line in get_proc.stdout.splitlines():
            line = line.strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            applied[key.strip()] = value.strip()

        self._uvc_controls_applied = applied
        self._uvc_controls_locked = True
        self.get_logger().info(
            f'UVC controls locked on {controls_device}: {self._uvc_controls_applied}'
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
            self.get_logger().info(f'OpenCV backend id: {negotiation.backend_id}')

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
                self.get_logger().info('CameraInfo missing; using default CameraInfo()')
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

        if self.vignette_enabled and self._flat_fields_bgr is not None:
            try:
                frame = apply_vignette_correction(
                    frame,
                    self._flat_fields_bgr,
                    dark_current=self.vignette_dark_current,
                )
            except Exception as ex:
                self.get_logger().warn(
                    f'Vignette correction failed: {ex}. Disabling vignette correction at runtime.'
                )
                self.vignette_enabled = False

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
        if self.metadata_enabled:
            payload = self._build_frame_metadata(stamp, img_msg)
            self._publish_metadata(payload)

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
            self._stop_metadata.set()
            if self._metadata_thread is not None:
                self._metadata_thread.join(timeout=0.5)
            if self._metadata_proc is not None and self._metadata_proc.poll() is None:
                self._metadata_proc.terminate()
                try:
                    self._metadata_proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self._metadata_proc.kill()
            if self._metadata_log_file is not None:
                if self._metadata_log_records_since_flush > 0:
                    self._metadata_log_file.flush()
                self._metadata_log_file.close()
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
