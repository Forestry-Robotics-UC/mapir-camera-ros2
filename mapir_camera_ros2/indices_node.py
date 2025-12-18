#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: GNU General Public License v3.0 (GPL-3.0)
#
# Description:
#   ROS 2 node that computes vegetation / spectral indices from a 3-channel
#   Survey3 image stream (or any BGR image) using NumPy.

from __future__ import annotations

import threading
import time
from typing import Optional

from cv_bridge import CvBridge
from mapir_camera_core import (
    colorize_scalar_field,
    compute_spectral_indices,
    extract_bands_from_bgr,
    preset_band_channels,
    SpectralIndexParams,
    supported_spectral_indices,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class MapirIndicesNode(Node):
    """Compute spectral indices from an incoming Image stream."""

    def __init__(self) -> None:
        super().__init__('mapir_indices')

        self._lock = threading.Lock()

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_period_s', 1.0)

        self.declare_parameter('enabled', True)

        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('indices', ['ndvi'])
        self.declare_parameter('filter_set', '')  # e.g., RGN, NGB, OCN, RGB

        self.declare_parameter('normalize_input', True)
        self.declare_parameter('downsample_factor', 1)
        self.declare_parameter('publish_every_n', 1)

        self.declare_parameter('publish_color', False)
        self.declare_parameter('colormap', 'viridis')  # e.g., viridis, jet, gray, custom
        self.declare_parameter('colorize_min', -1.0)
        self.declare_parameter('colorize_max', 1.0)
        self.declare_parameter('custom_colormap', '')  # 'value,r,g,b; value,r,g,b; ...'

        self.declare_parameter('eps', 1e-6)
        self.declare_parameter('gari_gamma', 1.7)
        self.declare_parameter('wdrvi_alpha', 0.2)
        self.declare_parameter('mnli_L', 0.5)

        # Optional explicit band→channel overrides (0/1/2), -1 disables.
        self.declare_parameter('blue_channel', -1)
        self.declare_parameter('green_channel', -1)
        self.declare_parameter('red_channel', -1)
        self.declare_parameter('rededge_channel', -1)
        self.declare_parameter('nir_channel', -1)
        self.declare_parameter('nir1_channel', -1)
        self.declare_parameter('nir2_channel', -1)
        self.declare_parameter('cyan_channel', -1)
        self.declare_parameter('orange_channel', -1)

        # QoS (match camera stream best-practices).
        self.declare_parameter('qos_best_effort', True)
        self.declare_parameter('qos_depth', 5)

        self.debug = bool(self.get_parameter('debug').value)
        self.debug_period_s = float(self.get_parameter('debug_period_s').value)

        self.enabled = bool(self.get_parameter('enabled').value)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.indices = [str(x).strip().lower() for x in self.get_parameter('indices').value]
        self.filter_set = str(self.get_parameter('filter_set').value)

        self.normalize_input = bool(self.get_parameter('normalize_input').value)
        self.downsample_factor = max(1, int(self.get_parameter('downsample_factor').value))
        self.publish_every_n = max(1, int(self.get_parameter('publish_every_n').value))

        self.publish_color = bool(self.get_parameter('publish_color').value)
        self.colormap = str(self.get_parameter('colormap').value)
        self.colorize_min = float(self.get_parameter('colorize_min').value)
        self.colorize_max = float(self.get_parameter('colorize_max').value)
        self.custom_colormap = str(self.get_parameter('custom_colormap').value)

        self.params = SpectralIndexParams(
            eps=float(self.get_parameter('eps').value),
            gari_gamma=float(self.get_parameter('gari_gamma').value),
            wdrvi_alpha=float(self.get_parameter('wdrvi_alpha').value),
            mnli_L=float(self.get_parameter('mnli_L').value),
        )

        self.qos_depth = max(1, int(self.get_parameter('qos_depth').value))
        qos_best_effort = bool(self.get_parameter('qos_best_effort').value)
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if qos_best_effort
            else ReliabilityPolicy.RELIABLE
        )
        self.qos = QoSProfile(
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
        )

        self._channel_overrides = {
            'blue': int(self.get_parameter('blue_channel').value),
            'green': int(self.get_parameter('green_channel').value),
            'red': int(self.get_parameter('red_channel').value),
            'rededge': int(self.get_parameter('rededge_channel').value),
            'nir': int(self.get_parameter('nir_channel').value),
            'nir1': int(self.get_parameter('nir1_channel').value),
            'nir2': int(self.get_parameter('nir2_channel').value),
            'cyan': int(self.get_parameter('cyan_channel').value),
            'orange': int(self.get_parameter('orange_channel').value),
        }

        self.band_channels = self._make_band_channels(self.filter_set, self._channel_overrides)

        supported = supported_spectral_indices()
        self.indices = self._normalize_indices(self.indices, supported)
        requested_values = self.get_parameter('indices').value
        requested_bases = {str(i).strip().lower().split('_')[0] for i in requested_values}
        unknown = sorted(requested_bases - supported)
        if unknown:
            self.get_logger().warn(
                f'Unknown indices ignored: {unknown}. Supported: {sorted(supported)}'
            )

        # Publishers per-index.
        self.bridge = CvBridge()
        self.index_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self.color_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._sync_publishers()

        if not self.index_pubs:
            self.get_logger().warn(
                'No valid indices configured. Set parameter "indices" to a list like '
                '["ndvi", "gndvi"].'
            )

        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, self.qos)

        self._frame_count = 0
        self._last_debug_t: float = 0.0
        self._last_missing_warn_t: float = 0.0
        self._last_compute_s: Optional[float] = None

        self.get_logger().info(
            f'MAPIR indices node started: image_topic={self.image_topic!r}, '
            f'indices={sorted(self.index_pubs.keys())}, '
            f'downsample_factor={self.downsample_factor}, '
            f'publish_every_n={self.publish_every_n}, '
            f'qos={self.qos.reliability.name} depth={self.qos.depth}'
        )
        if self.band_channels:
            self.get_logger().info(f'Band mapping: {self.band_channels}')
        else:
            self.get_logger().warn(
                'No band mapping configured. Set "filter_set" (e.g., RGN/NGB/OCN/RGB) '
                'or explicitly set *_channel parameters.'
            )

        self.add_on_set_parameters_callback(self._on_params)

    def _normalize_indices(self, requested: list[str], supported: set[str]) -> list[str]:
        out: list[str] = []
        seen: set[str] = set()
        for raw in requested:
            idx_norm = str(raw).strip().lower()
            if not idx_norm:
                continue
            base = idx_norm.split('_')[0]
            if base not in supported:
                continue
            if idx_norm in seen:
                continue
            out.append(idx_norm)
            seen.add(idx_norm)
        return out

    def _make_band_channels(self, filter_set: str, overrides: dict[str, int]) -> dict[str, int]:
        band_channels = preset_band_channels(filter_set)
        for band_name, channel in overrides.items():
            if channel in (0, 1, 2):
                band_channels[band_name] = channel
            elif channel != -1:
                self.get_logger().warn(
                    f'Ignoring invalid {band_name}_channel={channel}; use -1/0/1/2'
                )
        return band_channels

    def _sync_publishers(self) -> None:
        desired = set(self.indices)

        for name in list(self.index_pubs.keys()):
            if name not in desired:
                self.destroy_publisher(self.index_pubs.pop(name))

        for name in sorted(desired):
            if name not in self.index_pubs:
                self.index_pubs[name] = self.create_publisher(Image, f'indices/{name}', self.qos)

        if not self.publish_color:
            for name in list(self.color_pubs.keys()):
                self.destroy_publisher(self.color_pubs.pop(name))
            return

        for name in list(self.color_pubs.keys()):
            if name not in desired:
                self.destroy_publisher(self.color_pubs.pop(name))

        for name in sorted(desired):
            if name not in self.color_pubs:
                self.color_pubs[name] = self.create_publisher(
                    Image,
                    f'indices_color/{name}',
                    self.qos,
                )

    def _on_params(self, params) -> SetParametersResult:
        with self._lock:
            new_enabled = self.enabled
            new_publish_color = self.publish_color
            new_colormap = self.colormap
            new_colorize_min = self.colorize_min
            new_colorize_max = self.colorize_max
            new_custom_colormap = self.custom_colormap

            new_image_topic = self.image_topic
            new_filter_set = self.filter_set
            new_downsample_factor = self.downsample_factor
            new_publish_every_n = self.publish_every_n
            new_normalize_input = self.normalize_input

            new_overrides = dict(self._channel_overrides)

            supported = supported_spectral_indices()
            new_indices_raw = list(self.indices)

            for p in params:
                if p.name == 'enabled':
                    new_enabled = bool(p.value)
                elif p.name == 'publish_color':
                    new_publish_color = bool(p.value)
                elif p.name == 'colormap':
                    new_colormap = str(p.value)
                elif p.name == 'colorize_min':
                    new_colorize_min = float(p.value)
                elif p.name == 'colorize_max':
                    new_colorize_max = float(p.value)
                elif p.name == 'custom_colormap':
                    new_custom_colormap = str(p.value)
                elif p.name == 'image_topic':
                    new_image_topic = str(p.value)
                elif p.name == 'indices':
                    new_indices_raw = [str(x).strip().lower() for x in p.value]
                elif p.name == 'filter_set':
                    new_filter_set = str(p.value)
                elif p.name == 'downsample_factor':
                    new_downsample_factor = max(1, int(p.value))
                elif p.name == 'publish_every_n':
                    new_publish_every_n = max(1, int(p.value))
                elif p.name == 'normalize_input':
                    new_normalize_input = bool(p.value)
                elif p.name.endswith('_channel'):
                    key = p.name[:-8]
                    new_overrides[key] = int(p.value)

            if new_colorize_max <= new_colorize_min:
                return SetParametersResult(
                    successful=False,
                    reason='colorize_max must be > colorize_min',
                )

            if new_image_topic != self.image_topic:
                self.get_logger().warn('image_topic changes require node restart to re-subscribe')

            self.enabled = new_enabled
            self.publish_color = new_publish_color
            self.colormap = new_colormap
            self.colorize_min = new_colorize_min
            self.colorize_max = new_colorize_max
            self.custom_colormap = new_custom_colormap

            self.filter_set = new_filter_set
            self.normalize_input = new_normalize_input
            self.downsample_factor = new_downsample_factor
            self.publish_every_n = new_publish_every_n

            self._channel_overrides = new_overrides
            self.band_channels = self._make_band_channels(self.filter_set, self._channel_overrides)

            self.indices = self._normalize_indices(new_indices_raw, supported)
            self._sync_publishers()

        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image) -> None:
        if not self.enabled:
            return

        self._frame_count += 1
        if (self._frame_count % self.publish_every_n) != 0:
            return

        if not self.index_pubs:
            return

        if not self.band_channels:
            return

        t0 = time.perf_counter()
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:
            if self.debug:
                self.get_logger().warn(f'cv_bridge conversion failed: {ex}')
            return

        if self.downsample_factor > 1:
            f = self.downsample_factor
            frame_bgr = frame_bgr[::f, ::f]

        try:
            bands = extract_bands_from_bgr(
                frame_bgr,
                self.band_channels,
                normalize=self.normalize_input,
            )
        except Exception as ex:
            self.get_logger().warn(f'Band extraction failed: {ex}')
            return

        computed = compute_spectral_indices(
            requested=list(self.index_pubs.keys()),
            bands=bands,
            params=self.params,
            on_missing='skip',
        )

        missing = sorted(set(self.index_pubs.keys()) - set(computed.keys()))
        if missing and self.debug:
            now = time.time()
            if (now - self._last_missing_warn_t) >= self.debug_period_s:
                self.get_logger().warn(f'Indices skipped due to missing bands: {missing}')
                self._last_missing_warn_t = now

        for name, idx_img in computed.items():
            out_msg = self.bridge.cv2_to_imgmsg(idx_img, encoding='32FC1')
            out_msg.header = msg.header
            self.index_pubs[name].publish(out_msg)

            if self.publish_color and name in self.color_pubs:
                try:
                    color_bgr = colorize_scalar_field(
                        idx_img,
                        vmin=self.colorize_min,
                        vmax=self.colorize_max,
                        colormap=self.colormap,
                        custom_colormap=self.custom_colormap,
                    )
                except Exception as ex:
                    if self.debug:
                        self.get_logger().warn(f'Colorize failed for {name}: {ex}')
                    continue

                color_msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding='bgr8')
                color_msg.header = msg.header
                self.color_pubs[name].publish(color_msg)

        t1 = time.perf_counter()
        self._last_compute_s = t1 - t0

        if self.debug:
            now = time.time()
            if (now - self._last_debug_t) >= self.debug_period_s:
                hz = 0.0
                if self._last_compute_s is not None and self._last_compute_s > 0:
                    hz = 1.0 / self._last_compute_s
                self.get_logger().debug(
                    f'Computed {len(computed)}/{len(self.index_pubs)} indices; '
                    f'compute_time={self._last_compute_s * 1000.0:.2f}ms (~{hz:.1f}Hz)'
                )
                self._last_debug_t = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapirIndicesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
