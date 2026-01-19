#!/usr/bin/env python3
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
"""Capture one OCN frame and generate doc images for vegetation indices."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Any, Dict, Iterable, List

import cv2
import numpy as np

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


def _parse_yaml(path: Path) -> Dict[str, Any]:
    if yaml is None:
        return {}
    try:
        with path.open(encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except OSError:
        return {}
    if not isinstance(data, dict):
        return {}
    if '/**' in data and isinstance(data['/**'], dict):
        return data['/**'].get('ros__parameters', {}) or {}
    if 'ros__parameters' in data and isinstance(data['ros__parameters'], dict):
        return data['ros__parameters']
    return data


def _parse_yaml_fallback(path: Path, keys: Iterable[str]) -> Dict[str, Any]:
    data: Dict[str, Any] = {}
    try:
        text = path.read_text(encoding='utf-8')
    except OSError:
        return data
    for key in keys:
        match = re.search(rf'^\s*{re.escape(key)}\s*:\s*(.+)$', text, re.MULTILINE)
        if not match:
            continue
        value = match.group(1).strip()
        data[key] = value
    return data


def load_params(path: Path, keys: Iterable[str]) -> Dict[str, Any]:
    params = _parse_yaml(path)
    if params:
        return params
    return _parse_yaml_fallback(path, keys)


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def parse_indices(value: Any) -> List[str]:
    if isinstance(value, (list, tuple)):
        return [str(item).strip().lower() for item in value if str(item).strip()]
    if not value:
        return []
    text = str(value).strip()
    if text.startswith('[') and text.endswith(']'):
        text = text[1:-1]
    items = [item.strip().strip('\'"').lower() for item in text.split(',') if item.strip()]
    return items


def filter_available_indices(
    indices: Iterable[str],
    bands: Dict[str, int],
    params,
) -> List[str]:
    from mapir_camera_ros2.core.spectral_indices import MissingBandError, compute_spectral_indices

    dummy = {name: np.ones((1, 1), dtype=np.float32) for name in bands}
    available = []
    for idx in indices:
        try:
            compute_spectral_indices([idx], dummy, params=params, on_missing='raise')
        except MissingBandError:
            continue
        except Exception:
            continue
        available.append(idx)
    return available


def build_band_channels(filter_set: str, params: Dict[str, Any]) -> Dict[str, int]:
    from mapir_camera_ros2.core.spectral_indices import preset_band_channels

    band_channels = preset_band_channels(filter_set)
    overrides = {
        'blue': as_int(params.get('blue_channel', -1), -1),
        'green': as_int(params.get('green_channel', -1), -1),
        'red': as_int(params.get('red_channel', -1), -1),
        'rededge': as_int(params.get('rededge_channel', -1), -1),
        'nir': as_int(params.get('nir_channel', -1), -1),
        'nir1': as_int(params.get('nir1_channel', -1), -1),
        'nir2': as_int(params.get('nir2_channel', -1), -1),
        'cyan': as_int(params.get('cyan_channel', -1), -1),
        'orange': as_int(params.get('orange_channel', -1), -1),
    }
    for name, channel in overrides.items():
        if channel in (0, 1, 2):
            band_channels[name] = channel
    return band_channels


def _open_with_v4l2(device: str) -> cv2.VideoCapture:
    return cv2.VideoCapture(device, cv2.CAP_V4L2)


def _open_with_default(device: str) -> cv2.VideoCapture:
    return cv2.VideoCapture(device)


def _open_with_gstreamer(pipeline: str) -> cv2.VideoCapture:
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


def open_camera(
    device: str,
    width: int,
    height: int,
    fps: float,
    pixel_format: str,
    use_gstreamer: bool,
    pipeline: str,
) -> cv2.VideoCapture:
    if use_gstreamer and pipeline:
        cap = _open_with_gstreamer(pipeline)
        if not cap.isOpened():
            raise RuntimeError('Failed to open camera with gstreamer pipeline.')
    else:
        cap = _open_with_v4l2(device)
        if not cap.isOpened():
            cap = _open_with_default(device)
        if not cap.isOpened():
            gst_pipeline = (
                f'v4l2src device={device} ! '
                f'image/jpeg,width={width},height={height},framerate={int(fps)}/1 '
                '! jpegdec ! videoconvert ! video/x-raw,format=BGR '
                '! appsink drop=true max-buffers=1 sync=false'
            )
            cap = _open_with_gstreamer(gst_pipeline)
        if not cap.isOpened():
            raise RuntimeError(f'Failed to open camera: {device}')

    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)
    if pixel_format and len(pixel_format) == 4:
        fourcc = cv2.VideoWriter_fourcc(*pixel_format)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    return cap


def load_input_image(path: Path, *, raw: bool, width: int, height: int) -> np.ndarray:
    if raw:
        from mapir_camera_ros2.core.raw_survey3 import decode_survey3_raw12

        raw_bytes = path.read_bytes()
        bayer_u16 = decode_survey3_raw12(raw_bytes, width=width, height=height)
        return cv2.cvtColor(bayer_u16, cv2.COLOR_BAYER_RG2BGR)

    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f'Failed to read image: {path}')
    return image


def save_channel_image(out_dir: Path, prefix: str, name: str, channel: np.ndarray) -> None:
    out_path = out_dir / f'{prefix}_channel_{name}.png'
    cv2.imwrite(str(out_path), channel)


def save_index_image(
    out_dir: Path,
    prefix: str,
    index_name: str,
    index_img: np.ndarray,
    vmin: float,
    vmax: float,
    cmap_name: str,
    custom_colormap: str,
) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import colors as mpl_colors

    from mapir_camera_ros2.core.colormap import colorize_scalar_field

    gradient = np.linspace(vmin, vmax, 256, dtype=np.float32)
    gradient = gradient[:, None]
    lut_bgr = colorize_scalar_field(
        gradient,
        vmin=vmin,
        vmax=vmax,
        colormap=cmap_name,
        custom_colormap=custom_colormap,
    )
    lut_rgb = (lut_bgr[:, 0, ::-1] / 255.0).clip(0.0, 1.0)
    cmap = mpl_colors.ListedColormap(lut_rgb, name=f'mapir_{cmap_name}')
    norm = mpl_colors.Normalize(vmin=vmin, vmax=vmax, clip=True)

    img_plot = np.nan_to_num(index_img, nan=vmin, posinf=vmax, neginf=vmin)
    img_plot = np.clip(img_plot, vmin, vmax)

    fig, ax = plt.subplots(figsize=(7, 5))
    im = ax.imshow(img_plot, cmap=cmap, norm=norm)
    ax.set_title(index_name)
    ax.axis('off')
    fig.colorbar(im, ax=ax)
    out_path = out_dir / f'{prefix}_index_{index_name.lower()}.png'
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--index',
        action='append',
        default=[],
        help='Index name to render (repeatable).',
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Render all OCN-available indices.',
    )
    parser.add_argument(
        '--camera-config',
        default='config/mapir_camera_params.yaml',
        help='Camera params YAML (default: config/mapir_camera_params.yaml).',
    )
    parser.add_argument(
        '--indices-config',
        default='config/mapir_indices_ocn_params.yaml',
        help='Indices params YAML (default: config/mapir_indices_ocn_params.yaml).',
    )
    parser.add_argument(
        '--image',
        default='',
        help='Optional image path (skip camera capture).',
    )
    parser.add_argument(
        '--raw',
        action='store_true',
        help='Treat --image as Survey3 12-bit Bayer RAW.',
    )
    parser.add_argument('--raw-width', type=int, default=4000, help='RAW width (pixels).')
    parser.add_argument('--raw-height', type=int, default=3000, help='RAW height (pixels).')
    parser.add_argument('--output-dir', default='tools/doc_images')
    parser.add_argument('--output-prefix', default='ocn')
    parser.add_argument('--device', default='')
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    camera_path = (root / args.camera_config).resolve()
    indices_path = (root / args.indices_config).resolve()

    camera_keys = [
        'video_device',
        'pixel_format',
        'use_gstreamer',
        'gstreamer_pipeline',
        'image_width',
        'image_height',
        'framerate',
    ]
    indices_keys = [
        'indices',
        'filter_set',
        'normalize_input',
        'colorize_min',
        'colorize_max',
        'eps',
        'gari_gamma',
        'wdrvi_alpha',
        'mnli_L',
        'blue_channel',
        'green_channel',
        'red_channel',
        'rededge_channel',
        'nir_channel',
        'nir1_channel',
        'nir2_channel',
        'cyan_channel',
        'orange_channel',
    ]

    camera_params = load_params(camera_path, camera_keys)
    indices_params = load_params(indices_path, indices_keys)

    filter_set = str(indices_params.get('filter_set', 'OCN')).strip()
    if filter_set.lower() != 'ocn':
        raise RuntimeError(f'Expected OCN filter_set, got {filter_set!r}')

    from mapir_camera_ros2.core.spectral_indices import (
        SpectralIndexParams,
        compute_spectral_indices,
        extract_bands_from_bgr,
        supported_spectral_indices,
    )

    band_channels = build_band_channels(filter_set, indices_params)
    params = SpectralIndexParams(
        eps=as_float(indices_params.get('eps', 1.0e-6), 1.0e-6),
        gari_gamma=as_float(indices_params.get('gari_gamma', 1.7), 1.7),
        wdrvi_alpha=as_float(indices_params.get('wdrvi_alpha', 0.2), 0.2),
        mnli_L=as_float(indices_params.get('mnli_L', 0.5), 0.5),
    )

    indices = [name.lower() for name in args.index if name.strip()]
    if not indices:
        indices = parse_indices(indices_params.get('indices', []))
    if not indices:
        indices = sorted(supported_spectral_indices())

    if args.all or not args.index:
        indices = filter_available_indices(indices, band_channels, params)
    else:
        available = set(filter_available_indices(indices, band_channels, params))
        missing = [idx for idx in indices if idx not in available]
        if missing:
            raise RuntimeError(f'Indices not available for OCN: {missing}')

    if not indices:
        raise RuntimeError('No indices to render.')

    if args.image:
        frame = load_input_image(
            Path(args.image),
            raw=args.raw,
            width=args.raw_width,
            height=args.raw_height,
        )
    else:
        video_device = args.device or str(camera_params.get('video_device', '/dev/video0'))
        pixel_format = str(camera_params.get('pixel_format', 'MJPG')).strip()
        use_gstreamer = as_bool(camera_params.get('use_gstreamer', False))
        pipeline = str(camera_params.get('gstreamer_pipeline', '')).strip()
        width = as_int(camera_params.get('image_width', 0), 0)
        height = as_int(camera_params.get('image_height', 0), 0)
        fps = as_float(camera_params.get('framerate', 0.0), 0.0)

        cap = open_camera(video_device, width, height, fps, pixel_format, use_gstreamer, pipeline)
        ret, frame = cap.read()
        cap.release()

        if not ret or frame is None:
            raise RuntimeError('Failed to capture frame from camera.')

    out_dir = Path(args.output_dir)
    if not out_dir.is_absolute():
        out_dir = root / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    prefix = args.output_prefix
    cv2.imwrite(str(out_dir / f'{prefix}_original.png'), frame)

    channel_names = {0: 'cyan', 1: 'orange', 2: 'nir1'}
    for ch in range(3):
        save_channel_image(out_dir, prefix, channel_names.get(ch, f'ch{ch}'), frame[:, :, ch])

    normalize_input = as_bool(indices_params.get('normalize_input', True))
    bands = extract_bands_from_bgr(frame, band_channels, normalize=normalize_input)
    computed = compute_spectral_indices(indices, bands, params=params, on_missing='skip')

    vmin = as_float(indices_params.get('colorize_min', -1.0), -1.0)
    vmax = as_float(indices_params.get('colorize_max', 1.0), 1.0)

    for name, idx_img in computed.items():
        save_index_image(
            out_dir,
            prefix,
            name.upper(),
            idx_img,
            vmin,
            vmax,
            'viridis',
            str(indices_params.get('custom_colormap', '')).strip(),
        )

    print(f'Saved doc images to {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
