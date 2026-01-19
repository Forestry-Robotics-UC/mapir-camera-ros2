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
"""Calibrate reflectance from a raw (Bayer) or standard image and emit YAML."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List, Sequence

import cv2
import numpy as np

from mapir_camera_ros2.core import (
    apply_linear_reflectance,
    compute_panel_quad_from_fiducial,
    detect_aruco_fiducial,
    fit_linear_reflectance,
    normalize_patch_reflectances,
    roi_to_quad,
    sample_patch_stats,
    split_panel_quad,
)
from mapir_camera_ros2.core.raw_survey3 import decode_survey3_raw12


def _parse_patch_reflectances(raw: str, patch_count: int) -> List[Sequence[float]]:
    if not raw:
        raise ValueError('patch_reflectances must be provided')

    groups = [entry.strip() for entry in raw.split(';') if entry.strip()]
    if not groups:
        raise ValueError('patch_reflectances is empty')

    parsed: List[Sequence[float]] = []
    for entry in groups:
        parts = [item.strip() for item in entry.split(',') if item.strip()]
        values = [float(item) for item in parts]
        if len(values) == 1:
            parsed.append(values[0])
        elif len(values) == 3:
            parsed.append(values)
        else:
            raise ValueError(
                'Each patch reflectance must be a scalar or 3 comma-separated values'
            )

    if len(parsed) != patch_count:
        raise ValueError(f'Expected {patch_count} patches, got {len(parsed)}')

    return parsed


def _parse_roi(raw: str) -> np.ndarray:
    parts = [item.strip() for item in raw.split(',') if item.strip()]
    if len(parts) != 4:
        raise ValueError('ROI must be "x,y,w,h"')
    return roi_to_quad([float(item) for item in parts])


def _load_image(
    path: Path,
    *,
    raw_bayer: bool,
    width: int,
    height: int,
) -> np.ndarray:
    if raw_bayer:
        raw_bytes = path.read_bytes()
        bayer_u16 = decode_survey3_raw12(raw_bytes, width=width, height=height)
        return cv2.cvtColor(bayer_u16, cv2.COLOR_BAYER_RG2BGR)

    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f'Failed to read image: {path}')
    return image


def _fit_from_panel(
    image: np.ndarray,
    panel_quad: np.ndarray,
    *,
    patch_grid: Sequence[int],
    patch_inset_ratio: float,
    patch_stat: str,
    patch_warp_size: int,
    patch_trim_ratio: float,
    patch_reflectances: Sequence[object],
    min_dn_range: float,
    max_abs_slope: float,
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
    patches = split_panel_quad(panel_quad, grid=patch_grid, inset_ratio=patch_inset_ratio)
    patch_means = sample_patch_stats(
        image,
        patches,
        method=patch_stat,
        warp_size=patch_warp_size,
        trim_ratio=patch_trim_ratio,
    )
    patch_reflect = normalize_patch_reflectances(
        patch_reflectances,
        num_patches=len(patches),
    )
    slopes, intercepts = fit_linear_reflectance(
        patch_means,
        patch_reflect,
        min_dn_range=min_dn_range,
        max_abs_slope=max_abs_slope,
    )
    return slopes, intercepts, patches


def _write_yaml(
    output: Path,
    slopes: np.ndarray,
    intercepts: np.ndarray,
    *,
    clamp_min: float,
    clamp_max: float,
) -> None:
    lines = [
        'reflectance_node:',
        '  ros__parameters:',
        '    enable: true',
        "    calibration_mode: 'once'",
        f'    clamp_min: {clamp_min}',
        f'    clamp_max: {clamp_max}',
        f'    initial_slopes: [{", ".join(f"{v:.6f}" for v in slopes.tolist())}]',
        f'    initial_intercepts: [{", ".join(f"{v:.6f}" for v in intercepts.tolist())}]',
    ]
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _normalize_preview(image: np.ndarray) -> np.ndarray:
    if np.issubdtype(image.dtype, np.integer):
        denom = float(np.iinfo(image.dtype).max)
        img = image.astype(np.float32) / denom
    else:
        img = image.astype(np.float32, copy=False)
    img = np.clip(img, 0.0, 1.0)
    preview = (img * 255.0).astype(np.uint8)
    if preview.ndim == 2:
        preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
    return preview


def _save_previews(
    image: np.ndarray,
    slopes: np.ndarray,
    intercepts: np.ndarray,
    *,
    before_path: str,
    after_path: str,
) -> None:
    if before_path:
        cv2.imwrite(before_path, _normalize_preview(image))

    if after_path:
        calibrated = apply_linear_reflectance(
            image.astype(np.float32, copy=False),
            slopes,
            intercepts,
            clamp_min=0.0,
            clamp_max=1.2,
        )
        cv2.imwrite(after_path, _normalize_preview(calibrated))


def _draw_debug(
    image: np.ndarray,
    panel_quad: np.ndarray,
    patches: Iterable[np.ndarray],
    output: Path,
) -> None:
    overlay = image.copy()
    if overlay.ndim == 2:
        overlay = cv2.cvtColor(overlay, cv2.COLOR_GRAY2BGR)
    cv2.polylines(
        overlay,
        [np.round(panel_quad).astype(np.int32)],
        isClosed=True,
        color=(255, 0, 0),
        thickness=2,
    )
    for patch in patches:
        cv2.polylines(
            overlay,
            [np.round(patch).astype(np.int32)],
            isClosed=True,
            color=(0, 255, 255),
            thickness=1,
        )
    cv2.imwrite(str(output), overlay)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, help='Path to image or raw Bayer file.')
    parser.add_argument(
        '--raw',
        action='store_true',
        help='Treat input as Survey3 12-bit Bayer RAW.',
    )
    parser.add_argument('--width', type=int, default=4000, help='RAW width (pixels).')
    parser.add_argument('--height', type=int, default=3000, help='RAW height (pixels).')
    parser.add_argument(
        '--patch-reflectances',
        required=True,
        help='Patch reflectances: "0.1;0.2;0.5;0.9" or '
        '"0.1,0.1,0.1;0.2,0.2,0.2;0.5,0.5,0.5;0.9,0.9,0.9".',
    )
    parser.add_argument('--panel-roi', default='', help='Manual panel ROI "x,y,w,h".')
    parser.add_argument('--fiducial-id', type=int, default=0, help='ArUco ID.')
    parser.add_argument(
        '--aruco-detector',
        default='auto',
        choices=('auto', 'legacy', 'opencv'),
        help='ArUco detection method.',
    )
    parser.add_argument(
        '--aruco-dictionary',
        default='DICT_4X4_50',
        help='ArUco dictionary name.',
    )
    parser.add_argument(
        '--panel-side',
        default='right',
        choices=('right', 'left', 'top', 'bottom'),
        help='Panel location relative to fiducial.',
    )
    parser.add_argument('--panel-scale-w', type=float, default=1.0)
    parser.add_argument('--panel-scale-h', type=float, default=1.0)
    parser.add_argument('--panel-gap-scale', type=float, default=0.10)
    parser.add_argument('--patch-inset-ratio', type=float, default=0.20)
    parser.add_argument(
        '--patch-stat',
        default='median',
        choices=('mean', 'median', 'trimmed_mean'),
        help='Patch statistic for reflectance sampling.',
    )
    parser.add_argument('--patch-warp-size', type=int, default=32)
    parser.add_argument('--patch-trim-ratio', type=float, default=0.1)
    parser.add_argument('--min-dn-range', type=float, default=5.0)
    parser.add_argument('--max-abs-slope', type=float, default=2.0)
    parser.add_argument('--clamp-min', type=float, default=0.0)
    parser.add_argument('--clamp-max', type=float, default=1.2)
    parser.add_argument('--output', required=True, help='Output YAML path.')
    parser.add_argument('--debug-image', default='', help='Optional debug overlay output.')
    parser.add_argument(
        '--preview-before',
        default='',
        help='Optional preview image path before calibration.',
    )
    parser.add_argument(
        '--preview-after',
        default='',
        help='Optional preview image path after calibration.',
    )
    args = parser.parse_args()

    patch_reflectances = _parse_patch_reflectances(args.patch_reflectances, 4)
    image = _load_image(
        Path(args.image),
        raw_bayer=args.raw,
        width=args.width,
        height=args.height,
    )

    panel_quad = None
    patches = []
    if args.panel_roi:
        panel_quad = _parse_roi(args.panel_roi)
        slopes, intercepts, patches = _fit_from_panel(
            image,
            panel_quad,
            patch_grid=(2, 2),
            patch_inset_ratio=args.patch_inset_ratio,
            patch_stat=args.patch_stat,
            patch_warp_size=args.patch_warp_size,
            patch_trim_ratio=args.patch_trim_ratio,
            patch_reflectances=patch_reflectances,
            min_dn_range=args.min_dn_range,
            max_abs_slope=args.max_abs_slope,
        )
    else:
        fiducial = detect_aruco_fiducial(
            image,
            dictionary_name=args.aruco_dictionary,
            fiducial_id=args.fiducial_id,
            method=args.aruco_detector,
        )
        if fiducial is None:
            raise RuntimeError('Fiducial not detected; provide --panel-roi to override.')

        panel_quad = compute_panel_quad_from_fiducial(
            fiducial,
            side=args.panel_side,
            panel_scale_w=args.panel_scale_w,
            panel_scale_h=args.panel_scale_h,
            panel_gap_scale=args.panel_gap_scale,
        )
        slopes, intercepts, patches = _fit_from_panel(
            image,
            panel_quad,
            patch_grid=(2, 2),
            patch_inset_ratio=args.patch_inset_ratio,
            patch_stat=args.patch_stat,
            patch_warp_size=args.patch_warp_size,
            patch_trim_ratio=args.patch_trim_ratio,
            patch_reflectances=patch_reflectances,
            min_dn_range=args.min_dn_range,
            max_abs_slope=args.max_abs_slope,
        )

    _write_yaml(
        Path(args.output),
        slopes,
        intercepts,
        clamp_min=args.clamp_min,
        clamp_max=args.clamp_max,
    )

    _save_previews(
        image,
        slopes,
        intercepts,
        before_path=args.preview_before,
        after_path=args.preview_after,
    )

    if args.debug_image:
        _draw_debug(image, panel_quad, patches, Path(args.debug_image))

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
