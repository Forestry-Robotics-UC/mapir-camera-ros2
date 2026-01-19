#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2025 Duda Andrada
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: GNU General Public License v3.0 (GPL-3.0-only)
#
# Description:
#   ROS 2 node that performs per-channel linear reflectance calibration using
#   a 2x2 calibration target (fiducial + reflectance panel).
#
# Notes:
#   - Input is treated as O/C/N channels; channel order is preserved.
#   - Outputs a 32FC3 reflectance image plus optional preview/debug overlays.
#

from __future__ import annotations

import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from mapir_camera_ros2.core import (
    apply_gamma_reflectance,
    apply_linear_reflectance,
    compute_panel_quad_from_fiducial,
    detect_aruco_fiducial,
    fit_gamma_reflectance,
    fit_linear_reflectance,
    normalize_patch_reflectances,
    quad_area,
    ReflectanceFitError,
    roi_to_quad,
    sample_patch_stats,
    split_panel_quad,
)
from mapir_camera_ros2.core.target_detection import to_gray_u8


class MapirReflectanceNode(Node):
    """Estimate and apply per-channel reflectance calibration."""

    def __init__(self) -> None:
        super().__init__('mapir_reflectance')

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_period_s', 1.0)
        self.declare_parameter('enable', True)

        self.declare_parameter('input_topic', 'image_raw')
        self.declare_parameter('output_topic', 'image_reflectance')
        self.declare_parameter('preview_topic', 'image_reflectance_preview')
        self.declare_parameter('debug_topic', 'reflectance/debug')
        self.declare_parameter('status_topic', 'reflectance/status')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('rectify_output', False)

        self.declare_parameter('calibration_mode', 'continuous')  # once | continuous
        self.declare_parameter('recalibration_interval_s', 5.0)
        self.declare_parameter('detect_rate_hz', 5.0)
        self.declare_parameter('detect_downscale', 1.0)
        self.declare_parameter('min_target_area_px', 5000)

        self.declare_parameter('model', 'linear')  # linear | linear_gray | gamma | gamma_gray
        self.declare_parameter('clamp_min', 0.0)
        self.declare_parameter('clamp_max', 1.2)
        self.declare_parameter('min_dn_range', 5.0)
        self.declare_parameter('max_abs_slope', 2.0)
        self.declare_parameter('min_gamma', 0.2)
        self.declare_parameter('max_gamma', 5.0)
        self.declare_parameter('max_gain', 10.0)
        self.declare_parameter('ema_alpha', 0.2)

        self.declare_parameter('detection_mode', 'fiducial')  # fiducial | manual
        self.declare_parameter('fiducial_type', 'aruco')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('fiducial_id', 0)
        self.declare_parameter('aruco_detector', 'legacy')  # legacy | opencv | auto

        self.declare_parameter('panel_side', 'right')  # right | left | top | bottom
        self.declare_parameter('panel_scale_w', 1.0)
        self.declare_parameter('panel_scale_h', 1.0)
        self.declare_parameter('panel_gap_scale', 0.10)
        self.declare_parameter('min_panel_in_bounds', 4)

        self.declare_parameter('patch_grid', [2, 2])
        self.declare_parameter('patch_inset_ratio', 0.20)
        self.declare_parameter('patch_stat', 'median')  # mean | median | trimmed_mean
        self.declare_parameter('patch_warp_size', 32)
        self.declare_parameter('patch_trim_ratio', 0.1)

        self.declare_parameter('patch_reflectances', [0.10, 0.20, 0.50, 0.90])
        self.declare_parameter('initial_slopes', [])
        self.declare_parameter('initial_intercepts', [])
        self.declare_parameter('publish_debug_overlay', True)

        # Manual ROI fallback
        self.declare_parameter('fiducial_roi', [])
        self.declare_parameter('panel_roi', [])
        self.declare_parameter('patch_rois', [])

        self.declare_parameter('qos_best_effort', True)
        self.declare_parameter('qos_depth', 5)

        # -----------------------
        # Read parameters
        # -----------------------
        self.debug = bool(self.get_parameter('debug').value)
        self.debug_period_s = float(self.get_parameter('debug_period_s').value)
        self.enable = bool(self.get_parameter('enable').value)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.preview_topic = str(self.get_parameter('preview_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.rectify_output = bool(self.get_parameter('rectify_output').value)

        self.calibration_mode = (
            str(self.get_parameter('calibration_mode').value).strip().lower()
        )
        self.recalibration_interval_s = float(
            self.get_parameter('recalibration_interval_s').value
        )
        self.detect_rate_hz = float(self.get_parameter('detect_rate_hz').value)
        self.detect_downscale = float(self.get_parameter('detect_downscale').value)
        self.min_target_area_px = int(self.get_parameter('min_target_area_px').value)

        self.model = str(self.get_parameter('model').value).strip().lower()
        self.clamp_min = float(self.get_parameter('clamp_min').value)
        self.clamp_max = float(self.get_parameter('clamp_max').value)
        self.min_dn_range = float(self.get_parameter('min_dn_range').value)
        self.max_abs_slope = float(self.get_parameter('max_abs_slope').value)
        self.min_gamma = float(self.get_parameter('min_gamma').value)
        self.max_gamma = float(self.get_parameter('max_gamma').value)
        self.max_gain = float(self.get_parameter('max_gain').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)

        self.detection_mode = (
            str(self.get_parameter('detection_mode').value).strip().lower()
        )
        self.fiducial_type = (
            str(self.get_parameter('fiducial_type').value).strip().lower()
        )
        self.aruco_dictionary = str(self.get_parameter('aruco_dictionary').value)
        self.fiducial_id = int(self.get_parameter('fiducial_id').value)
        self.aruco_detector = (
            str(self.get_parameter('aruco_detector').value).strip().lower()
        )

        self.panel_side = str(self.get_parameter('panel_side').value).strip().lower()
        self.panel_scale_w = float(self.get_parameter('panel_scale_w').value)
        self.panel_scale_h = float(self.get_parameter('panel_scale_h').value)
        self.panel_gap_scale = float(self.get_parameter('panel_gap_scale').value)
        self.min_panel_in_bounds = int(self.get_parameter('min_panel_in_bounds').value)

        self.patch_grid = self._parse_patch_grid(self.get_parameter('patch_grid').value)
        self.patch_inset_ratio = float(self.get_parameter('patch_inset_ratio').value)
        self.patch_stat = str(self.get_parameter('patch_stat').value).lower()
        self.patch_warp_size = int(self.get_parameter('patch_warp_size').value)
        self.patch_trim_ratio = float(self.get_parameter('patch_trim_ratio').value)

        patch_reflectances_raw = self.get_parameter('patch_reflectances').value
        self.patch_reflectances = normalize_patch_reflectances(
            patch_reflectances_raw,
            num_patches=self.patch_grid[0] * self.patch_grid[1],
        )
        self.initial_slopes = self._parse_linear_params(
            self.get_parameter('initial_slopes').value
        )
        self.initial_intercepts = self._parse_linear_params(
            self.get_parameter('initial_intercepts').value
        )

        self.publish_debug_overlay = bool(
            self.get_parameter('publish_debug_overlay').value
        )

        self.manual_fiducial_roi = self._parse_roi(self.get_parameter('fiducial_roi').value)
        self.manual_panel_roi = self._parse_roi(self.get_parameter('panel_roi').value)
        self.manual_patch_rois = self._parse_patch_rois(
            self.get_parameter('patch_rois').value
        )
        self._manual_patches_cache: Optional[list[np.ndarray]] = None
        self._cache_manual_patches()

        qos_best_effort = bool(self.get_parameter('qos_best_effort').value)
        qos_depth = max(1, int(self.get_parameter('qos_depth').value))
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if qos_best_effort
            else ReliabilityPolicy.RELIABLE
        )
        self.qos = QoSProfile(
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )

        if self.model != 'linear':
            if self.model not in ('linear', 'linear_gray', 'gamma', 'gamma_gray'):
                self.get_logger().warn(
                    f'Unknown model={self.model!r}; using "linear"'
                )
                self.model = 'linear'

        if self.calibration_mode not in ('once', 'continuous'):
            self.get_logger().warn(
                f'Unknown calibration_mode={self.calibration_mode!r}; using "continuous"'
            )
            self.calibration_mode = 'continuous'

        if self.detection_mode not in ('fiducial', 'manual'):
            self.get_logger().warn(
                f'Unknown detection_mode={self.detection_mode!r}; using "fiducial"'
            )
            self.detection_mode = 'fiducial'
        if self.patch_stat not in ('mean', 'median', 'trimmed_mean'):
            self.get_logger().warn(
                f'Unknown patch_stat={self.patch_stat!r}; using "median"'
            )
            self.patch_stat = 'median'
        if not (0.0 <= self.ema_alpha <= 1.0):
            self.get_logger().warn('ema_alpha must be in [0,1]; using 0.2')
            self.ema_alpha = 0.2

        # -----------------------
        # ROS interfaces
        # -----------------------
        cv2.setUseOptimized(True)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.input_topic, self._on_image, self.qos)
        self.pub = self.create_publisher(Image, self.output_topic, self.qos)
        self.preview_pub = (
            self.create_publisher(Image, self.preview_topic, self.qos)
            if self.preview_topic
            else None
        )
        self.cinfo_sub = (
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self._on_camera_info,
                self.qos,
            )
            if self.rectify_output and self.camera_info_topic
            else None
        )
        self.debug_pub = (
            self.create_publisher(Image, self.debug_topic, self.qos)
            if self.publish_debug_overlay and self.debug_topic
            else None
        )
        self.status_pub = (
            self.create_publisher(String, self.status_topic, self.qos)
            if self.status_topic
            else None
        )

        # -----------------------
        # State
        # -----------------------
        self._last_detect_t = 0.0
        self._last_calib_t = 0.0
        self._last_debug_t = 0.0
        self._last_status_t = 0.0
        self._last_status_msg = ''
        self._last_rectify_warn_t = 0.0

        self._has_calibration = False
        self._slopes = np.ones((3,), dtype=np.float32)
        self._intercepts = np.zeros((3,), dtype=np.float32)
        self._gains = np.ones((3,), dtype=np.float32)
        self._gammas = np.ones((3,), dtype=np.float32)
        self._last_compute_s: Optional[float] = None
        self._rectify_map1: Optional[np.ndarray] = None
        self._rectify_map2: Optional[np.ndarray] = None
        self._rectify_size: Optional[tuple[int, int]] = None

        if self.initial_slopes is not None and self.initial_intercepts is not None:
            if self.initial_slopes.shape == self.initial_intercepts.shape:
                if self.initial_slopes.size in (1, 3):
                    self._slopes = self.initial_slopes
                    self._intercepts = self.initial_intercepts
                    self._has_calibration = True
                    self.get_logger().info('Loaded initial reflectance calibration.')
                else:
                    self.get_logger().warn(
                        'initial_slopes/intercepts must have length 1 or 3.'
                    )
            else:
                self.get_logger().warn(
                    'initial_slopes and initial_intercepts length mismatch.'
                )

        self._last_fiducial = None
        self._last_panel = None
        self._last_patches = None
        self._last_patch_means = None
        self._last_target_visible = False
        self._dtype_scale = {}
        self._last_reason = 'init'
        self._last_method = ''
        self._last_marker_id = None

        self.get_logger().info(
            f'MAPIR reflectance node started: input={self.input_topic!r}, '
            f'output={self.output_topic!r}, mode={self.detection_mode}, '
            f'calibration={self.calibration_mode}, detect_rate_hz={self.detect_rate_hz}'
        )

        self.add_on_set_parameters_callback(self._on_params)

    def _on_params(self, params):
        for param in params:
            if param.name == 'enable':
                self.enable = bool(param.value)
            elif param.name == 'calibration_mode':
                self.calibration_mode = str(param.value).strip().lower()
            elif param.name == 'recalibration_interval_s':
                self.recalibration_interval_s = float(param.value)
            elif param.name == 'detect_rate_hz':
                self.detect_rate_hz = float(param.value)
            elif param.name == 'detect_downscale':
                self.detect_downscale = float(param.value)
            elif param.name == 'patch_reflectances':
                try:
                    self.patch_reflectances = normalize_patch_reflectances(
                        param.value,
                        num_patches=self.patch_grid[0] * self.patch_grid[1],
                    )
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
            elif param.name == 'rectify_output':
                self.rectify_output = bool(param.value)
            elif param.name == 'patch_stat':
                patch_stat = str(param.value).lower()
                if patch_stat not in ('mean', 'median', 'trimmed_mean'):
                    return SetParametersResult(
                        successful=False,
                        reason=f'Unknown patch_stat: {patch_stat}',
                    )
                self.patch_stat = patch_stat
            elif param.name == 'patch_warp_size':
                self.patch_warp_size = int(param.value)
            elif param.name == 'patch_trim_ratio':
                self.patch_trim_ratio = float(param.value)
            elif param.name == 'ema_alpha':
                ema_alpha = float(param.value)
                if not (0.0 <= ema_alpha <= 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason='ema_alpha must be in [0,1]',
                    )
                self.ema_alpha = ema_alpha
        return SetParametersResult(successful=True)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.width == 0 or msg.height == 0:
            return

        k = np.array(msg.k, dtype=np.float32).reshape((3, 3))
        r = np.array(msg.r, dtype=np.float32).reshape((3, 3))
        d = np.array(msg.d, dtype=np.float32).reshape((-1, 1))
        if len(msg.p) >= 9:
            p = np.array(msg.p, dtype=np.float32).reshape((3, 4))
            new_k = p[:, :3]
        else:
            new_k = k

        size = (int(msg.width), int(msg.height))
        self._rectify_map1, self._rectify_map2 = cv2.initUndistortRectifyMap(
            k,
            d,
            r,
            new_k,
            size,
            cv2.CV_32FC1,
        )
        self._rectify_size = size

    def _parse_patch_grid(self, value) -> tuple[int, int]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            rows = int(value[0])
            cols = int(value[1])
            return max(1, rows), max(1, cols)
        return (2, 2)

    def _parse_linear_params(self, value) -> Optional[np.ndarray]:
        if not value:
            return None
        if isinstance(value, (list, tuple)):
            try:
                values = [float(item) for item in value]
            except Exception:
                return None
            if not values:
                return None
            return np.asarray(values, dtype=np.float32)
        return None

    def _parse_roi(self, value) -> Optional[np.ndarray]:
        if not value:
            return None
        if isinstance(value, (list, tuple)) and len(value) == 4:
            return roi_to_quad(value)
        return None

    def _parse_patch_rois(self, value) -> Optional[list[np.ndarray]]:
        if not value:
            return None
        if not isinstance(value, (list, tuple)):
            return None
        rois = []
        for entry in value:
            if isinstance(entry, (list, tuple)) and len(entry) == 4:
                rois.append(roi_to_quad(entry))
        if len(rois) < self.patch_grid[0] * self.patch_grid[1]:
            return None
        return rois

    def _cache_manual_patches(self) -> None:
        if self.manual_patch_rois:
            patches = self.manual_patch_rois[
                : self.patch_grid[0] * self.patch_grid[1]
            ]
            self._manual_patches_cache = [
                split_panel_quad(
                    patch,
                    grid=(1, 1),
                    inset_ratio=self.patch_inset_ratio,
                )[0]
                for patch in patches
            ]
            return

        if self.manual_panel_roi is None:
            self._manual_patches_cache = None
            return

        self._manual_patches_cache = split_panel_quad(
            self.manual_panel_roi,
            grid=self.patch_grid,
            inset_ratio=self.patch_inset_ratio,
        )

    def _should_run_detection(self, now: float) -> bool:
        if self.detection_mode == 'manual':
            return True
        if self.detect_rate_hz <= 0.0:
            return False
        period = 1.0 / max(self.detect_rate_hz, 1e-3)
        return (now - self._last_detect_t) >= period

    def _compute_patch_quads(self, detect_image: np.ndarray, sample_image: np.ndarray):
        fiducial = None
        panel = None

        if self.detection_mode == 'manual':
            if self._manual_patches_cache is None:
                self._set_reason('manual_patches_missing')
                return None, None, None

            panel = self.manual_panel_roi
            patches = self._manual_patches_cache
            fiducial = self.manual_fiducial_roi
            self._set_reason('manual_ok')
            return fiducial, panel, patches

        if self.fiducial_type != 'aruco':
            self._set_reason('fiducial_type_unsupported')
            return None, None, None

        image_for_detect = detect_image
        scale = float(self.detect_downscale)
        if 0.0 < scale < 1.0:
            height, width = detect_image.shape[:2]
            new_w = max(1, int(round(width * scale)))
            new_h = max(1, int(round(height * scale)))
            image_for_detect = cv2.resize(
                image_for_detect,
                (new_w, new_h),
                interpolation=cv2.INTER_AREA,
            )
            min_area = float(self.min_target_area_px) * scale * scale
        else:
            min_area = float(self.min_target_area_px)
            scale = 1.0

        fiducial = detect_aruco_fiducial(
            image_for_detect,
            dictionary_name=self.aruco_dictionary,
            fiducial_id=self.fiducial_id,
            min_area_px=min_area,
            method=self.aruco_detector,
        )
        if fiducial is not None and scale != 1.0:
            fiducial = fiducial / scale
        if fiducial is None:
            self._set_reason('marker_not_found')
            return None, None, None

        if self.panel_side == 'auto':
            best_panel = None
            best_patches = None
            best_valid = -1
            for side in ('left', 'right'):
                panel = compute_panel_quad_from_fiducial(
                    fiducial,
                    side=side,
                    panel_scale_w=self.panel_scale_w,
                    panel_scale_h=self.panel_scale_h,
                    panel_gap_scale=self.panel_gap_scale,
                )
                if quad_area(panel) < float(self.min_target_area_px):
                    continue
                if not self._quad_in_bounds(
                    panel, sample_image.shape[1], sample_image.shape[0]
                ):
                    continue
                patches = split_panel_quad(
                    panel,
                    grid=self.patch_grid,
                    inset_ratio=self.patch_inset_ratio,
                )
                patch_stats = sample_patch_stats(
                    sample_image,
                    patches,
                    method=self.patch_stat,
                    warp_size=self.patch_warp_size,
                    trim_ratio=self.patch_trim_ratio,
                )
                valid = np.isfinite(patch_stats).all(axis=1)
                valid_count = int(np.count_nonzero(valid))
                if valid_count > best_valid:
                    best_valid = valid_count
                    best_panel = panel
                    best_patches = patches
            if best_panel is None:
                self._set_reason('panel_not_valid')
                return fiducial, None, None
            self._set_reason('fiducial_ok')
            return fiducial, best_panel, best_patches

        panel = compute_panel_quad_from_fiducial(
            fiducial,
            side=self.panel_side,
            panel_scale_w=self.panel_scale_w,
            panel_scale_h=self.panel_scale_h,
            panel_gap_scale=self.panel_gap_scale,
        )

        if quad_area(panel) < float(self.min_target_area_px):
            self._set_reason('panel_too_small')
            return fiducial, panel, None
        if not self._quad_in_bounds(panel, sample_image.shape[1], sample_image.shape[0]):
            self._set_reason('panel_out_of_bounds')
            return fiducial, panel, None

        patches = split_panel_quad(
            panel,
            grid=self.patch_grid,
            inset_ratio=self.patch_inset_ratio,
        )
        self._set_reason('fiducial_ok')
        return fiducial, panel, patches

    def _publish_status(self, text: str, now: float, force: bool = False) -> None:
        if not self.status_pub:
            return
        if not force and text == self._last_status_msg:
            return
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self._last_status_msg = text
        self._last_status_t = now

    def _set_reason(self, reason: str) -> None:
        self._last_reason = reason

    def _quad_in_bounds(self, quad: np.ndarray, width: int, height: int) -> bool:
        quad_arr = np.asarray(quad, dtype=np.float32).reshape((4, 2))
        inside = (
            (quad_arr[:, 0] >= 0.0)
            & (quad_arr[:, 0] <= float(width - 1))
            & (quad_arr[:, 1] >= 0.0)
            & (quad_arr[:, 1] <= float(height - 1))
        )
        return int(np.count_nonzero(inside)) >= max(1, self.min_panel_in_bounds)

    def _compute_gray_patch_means(self, patch_means: np.ndarray) -> np.ndarray:
        if patch_means.shape[1] == 1:
            return patch_means
        b = patch_means[:, 0]
        g = patch_means[:, 1]
        r = patch_means[:, 2]
        gray = 0.114 * b + 0.587 * g + 0.299 * r
        return gray.reshape((-1, 1)).astype(np.float32)

    def _apply_ema(self, prev: np.ndarray, new: np.ndarray) -> np.ndarray:
        alpha = float(self.ema_alpha)
        return alpha * new + (1.0 - alpha) * prev

    def _on_image(self, msg: Image) -> None:
        if not self.enable:
            return

        t0 = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return

        if image.ndim == 2:
            channels = 1
        else:
            channels = int(image.shape[2])

        now = time.time()
        target_visible = self._last_target_visible
        patch_means = None

        if self._should_run_detection(now):
            self._last_detect_t = now
            detect_image = to_gray_u8(image)
            if detect_image is None:
                self._last_fiducial = None
                self._last_panel = None
                self._last_patches = None
                self._last_patch_means = None
                self._last_target_visible = False
                self._set_reason('invalid_image')
                self._publish_status('detection skipped: invalid image format', now)
                detect_image = None
                fiducial = panel = patches = None
            else:
                fiducial, panel, patches = self._compute_patch_quads(
                    detect_image,
                    image,
                )
            self._last_fiducial = fiducial
            self._last_panel = panel
            self._last_patches = patches

            if patches:
                patch_means = sample_patch_stats(
                    image,
                    patches,
                    method=self.patch_stat,
                    warp_size=self.patch_warp_size,
                    trim_ratio=self.patch_trim_ratio,
                )
                self._last_patch_means = patch_means
                valid = np.isfinite(patch_means).all(axis=1)
                target_visible = bool(np.count_nonzero(valid) >= 2)
                if not target_visible:
                    self._set_reason('patches_invalid')
            else:
                self._last_patch_means = None
                target_visible = False
            self._last_target_visible = target_visible

        if self.calibration_mode == 'once':
            should_calibrate = (not self._has_calibration) and target_visible
        else:
            interval_ok = self.recalibration_interval_s <= 0.0 or (
                now - self._last_calib_t >= self.recalibration_interval_s
            )
            should_calibrate = target_visible and interval_ok

        if should_calibrate and patch_means is not None:
            try:
                if self.model in ('linear_gray', 'gamma_gray'):
                    patch_dn = self._compute_gray_patch_means(patch_means)
                    patch_ref = self.patch_reflectances[:, :1]
                else:
                    patch_dn = patch_means
                    patch_ref = self.patch_reflectances

                if self.model in ('gamma', 'gamma_gray'):
                    dn_scale = self._dtype_scale.get(image.dtype, 1.0)
                    if np.issubdtype(image.dtype, np.integer):
                        if dn_scale == 1.0:
                            dn_scale = 1.0 / float(np.iinfo(image.dtype).max)
                            self._dtype_scale[image.dtype] = dn_scale
                    patch_dn_norm = patch_dn * float(dn_scale)
                    gains, gammas = fit_gamma_reflectance(
                        patch_dn_norm,
                        patch_ref,
                        min_dn_range=self.min_dn_range,
                        min_gamma=self.min_gamma,
                        max_gamma=self.max_gamma,
                        max_gain=self.max_gain,
                    )
                    if gains.size >= 3:
                        gains = gains[:3]
                        gammas = gammas[:3]
                    if self._has_calibration:
                        self._gains = self._apply_ema(self._gains, gains)
                        self._gammas = self._apply_ema(self._gammas, gammas)
                    else:
                        self._gains = gains
                        self._gammas = gammas
                else:
                    slopes, intercepts = fit_linear_reflectance(
                        patch_dn,
                        patch_ref,
                        min_dn_range=self.min_dn_range,
                        max_abs_slope=self.max_abs_slope,
                    )
                    if slopes.size >= 3:
                        slopes = slopes[:3]
                        intercepts = intercepts[:3]
                    if self._has_calibration:
                        self._slopes = self._apply_ema(self._slopes, slopes)
                        self._intercepts = self._apply_ema(self._intercepts, intercepts)
                    else:
                        self._slopes = slopes
                        self._intercepts = intercepts

                self._has_calibration = True
                self._last_calib_t = now
                self._set_reason('calibrated')
                self._publish_status('calibrated (target visible)', now, force=True)
            except ReflectanceFitError as exc:
                self._set_reason('fit_failed')
                self._publish_status(f'calibration failed: {exc}', now)
                if self.debug:
                    self.get_logger().warn(f'Reflectance fit failed: {exc}')

        if not self._has_calibration:
            reflectance = image.astype(np.float32, copy=True)
            if np.issubdtype(image.dtype, np.integer):
                inv_denom = self._dtype_scale.get(image.dtype)
                if inv_denom is None:
                    inv_denom = 1.0 / float(np.iinfo(image.dtype).max)
                    self._dtype_scale[image.dtype] = inv_denom
                reflectance *= inv_denom
            np.clip(reflectance, self.clamp_min, self.clamp_max, out=reflectance)
            self._publish_status('uncalibrated; using normalized input', now)
        else:
            if self.model in ('gamma', 'gamma_gray'):
                gains = self._gains
                gammas = self._gammas
                if channels == 1 and gains.size > 1:
                    gains = gains[:1]
                    gammas = gammas[:1]
                dn_scale = self._dtype_scale.get(image.dtype, 1.0)
                if np.issubdtype(image.dtype, np.integer):
                    if dn_scale == 1.0:
                        dn_scale = 1.0 / float(np.iinfo(image.dtype).max)
                        self._dtype_scale[image.dtype] = dn_scale
                reflectance = apply_gamma_reflectance(
                    image.astype(np.float32, copy=False),
                    gains,
                    gammas,
                    dn_scale=dn_scale,
                    clamp_min=self.clamp_min,
                    clamp_max=self.clamp_max,
                )
            else:
                slopes = self._slopes
                intercepts = self._intercepts
                if channels == 1 and slopes.size > 1:
                    slopes = slopes[:1]
                    intercepts = intercepts[:1]
                reflectance = apply_linear_reflectance(
                    image.astype(np.float32, copy=False),
                    slopes,
                    intercepts,
                    clamp_min=self.clamp_min,
                    clamp_max=self.clamp_max,
                )
            if not target_visible:
                self._set_reason('target_not_visible')
                self._publish_status('target not detected; using cached calibration', now)

        if self.rectify_output:
            if (
                self._rectify_map1 is not None
                and self._rectify_map2 is not None
                and self._rectify_size == (reflectance.shape[1], reflectance.shape[0])
            ):
                reflectance = cv2.remap(
                    reflectance,
                    self._rectify_map1,
                    self._rectify_map2,
                    interpolation=cv2.INTER_LINEAR,
                )
            else:
                if (now - self._last_rectify_warn_t) >= self.debug_period_s:
                    self._publish_status(
                        'rectify enabled but no valid CameraInfo map', now
                    )
                    self._last_rectify_warn_t = now

        # Publish reflectance output
        encoding = '32FC1' if channels == 1 else '32FC3'
        out_msg = self.bridge.cv2_to_imgmsg(reflectance, encoding=encoding)
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        # Preview (8-bit)
        if self.preview_pub and self.preview_pub.get_subscription_count() > 0:
            preview = cv2.convertScaleAbs(reflectance, alpha=255.0)
            if channels == 1:
                prev_msg = self.bridge.cv2_to_imgmsg(preview, encoding='mono8')
            else:
                prev_msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
            prev_msg.header = msg.header
            self.preview_pub.publish(prev_msg)

        # Debug overlay
        if self.debug_pub and self.debug_pub.get_subscription_count() > 0:
            overlay = image.copy()
            if overlay.ndim == 2:
                overlay = cv2.cvtColor(overlay, cv2.COLOR_GRAY2BGR)

            if self._last_fiducial is not None:
                cv2.polylines(
                    overlay,
                    [np.round(self._last_fiducial).astype(np.int32)],
                    isClosed=True,
                    color=(0, 255, 0),
                    thickness=2,
                )
            if self._last_panel is not None:
                cv2.polylines(
                    overlay,
                    [np.round(self._last_panel).astype(np.int32)],
                    isClosed=True,
                    color=(255, 0, 0),
                    thickness=2,
                )
            if self._last_patches:
                for idx, patch in enumerate(self._last_patches):
                    quad = np.round(patch).astype(np.int32)
                    cv2.polylines(
                        overlay,
                        [quad],
                        isClosed=True,
                        color=(0, 255, 255),
                        thickness=1,
                    )
                    center = quad.mean(axis=0).astype(int)
                    label = f'P{idx}'
                    if self._last_patch_means is not None:
                        means = self._last_patch_means[idx]
                        mean_text = ','.join(f'{value:.1f}' for value in means.tolist())
                        label = f'P{idx}:{mean_text}'
                    cv2.putText(
                        overlay,
                        label,
                        (int(center[0]), int(center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

            if self._has_calibration:
                if self.model in ('gamma', 'gamma_gray'):
                    gains_text = ','.join(f'{value:.3f}' for value in self._gains.tolist())
                    gammas_text = ','.join(
                        f'{value:.3f}' for value in self._gammas.tolist()
                    )
                    text = f'gain=[{gains_text}] gamma=[{gammas_text}]'
                else:
                    slopes_text = ','.join(
                        f'{value:.4f}' for value in self._slopes.tolist()
                    )
                    intercepts_text = ','.join(
                        f'{value:.3f}' for value in self._intercepts.tolist()
                    )
                    text = f'a=[{slopes_text}] b=[{intercepts_text}]'
            else:
                text = 'uncalibrated'
            reason_text = f'reason={self._last_reason}'
            cv2.putText(
                overlay,
                text,
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                reason_text,
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

            dbg_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)

        self._last_compute_s = time.perf_counter() - t0
        if self.debug and (now - self._last_debug_t) >= self.debug_period_s:
            calib = 'on' if self._has_calibration else 'off'
            hz = 0.0
            if self._last_compute_s is not None and self._last_compute_s > 0:
                hz = 1.0 / self._last_compute_s
            if self.model in ('gamma', 'gamma_gray'):
                param_rows = [
                    ('gains', np.array2string(self._gains, precision=4)),
                    ('gammas', np.array2string(self._gammas, precision=3)),
                ]
            else:
                param_rows = [
                    ('slopes', np.array2string(self._slopes, precision=4)),
                    ('intercepts', np.array2string(self._intercepts, precision=3)),
                ]
            self._log_table(
                'Reflectance Stats',
                [
                    ('calibration', calib),
                    ('target_visible', str(target_visible)),
                    ('compute_ms', f'{self._last_compute_s * 1000.0:.2f}'),
                    ('est_compute_hz', f'{hz:.1f}'),
                    ('reason', self._last_reason),
                ]
                + param_rows,
            )
            self._last_debug_t = now

    def _log_table(self, title: str, rows: list[tuple[str, str]]) -> None:
        if not rows:
            return
        key_width = max(len(key) for key, _ in rows)
        val_width = max(len(value) for _, value in rows)
        header = f'{"Param":<{key_width}} | {"Value":<{val_width}}'
        sep = '-' * len(header)
        lines = [title, header, sep]
        for key, value in rows:
            lines.append(f'{key:<{key_width}} | {value}')
        self.get_logger().info('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapirReflectanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
