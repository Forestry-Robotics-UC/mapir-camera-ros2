#!/usr/bin/env python3
"""Empirical line reflectance calibration from MAPIR target panel ROIs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

from mapir_camera_core import apply_vignette_correction, load_vignette_images


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Compute per-channel empirical-line reflectance calibration and apply it.'
    )
    parser.add_argument('--calibration-image', required=True)
    parser.add_argument('--panel-config', required=True)
    parser.add_argument('--apply-image', default='')
    parser.add_argument('--apply-dir', default='')
    parser.add_argument('--glob', default='*.png')
    parser.add_argument('--out-dir', default='/outputs/reflectance_calibration')
    parser.add_argument('--flat-b', default='')
    parser.add_argument('--flat-g', default='')
    parser.add_argument('--flat-r', default='')
    parser.add_argument('--dark-current-b', type=int, default=0)
    parser.add_argument('--dark-current-g', type=int, default=0)
    parser.add_argument('--dark-current-r', type=int, default=0)
    return parser.parse_args()


def _read_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f'Could not read image: {path}')
    if image.ndim != 3 or image.shape[2] != 3:
        raise RuntimeError(f'Expected 3-channel image, got shape={image.shape} for {path}')
    return image


def _load_panel_config(path: Path) -> list[dict]:
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or 'panels' not in data:
        raise RuntimeError('Panel config must be a JSON object with key "panels"')
    panels = data['panels']
    if not isinstance(panels, list) or len(panels) < 2:
        raise RuntimeError('Panel config must contain at least two panel entries')
    return panels


def _roi_mean_bgr(image: np.ndarray, roi: list[int]) -> np.ndarray:
    if len(roi) != 4:
        raise RuntimeError(f'Invalid ROI (expected [x,y,w,h]): {roi}')
    x, y, w, h = [int(v) for v in roi]
    if w <= 0 or h <= 0:
        raise RuntimeError(f'Invalid ROI size: {roi}')
    y2 = min(image.shape[0], y + h)
    x2 = min(image.shape[1], x + w)
    crop = image[max(0, y):y2, max(0, x):x2]
    if crop.size == 0:
        raise RuntimeError(f'ROI out of bounds: {roi}')
    return crop.reshape(-1, 3).mean(axis=0)


def _fit_empirical_line(panel_means_bgr: np.ndarray, panel_reflectance_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # Fits: reflectance = slope * DN + intercept, separately for B,G,R.
    slopes = np.zeros(3, dtype=np.float64)
    intercepts = np.zeros(3, dtype=np.float64)
    for c in range(3):
        x = panel_means_bgr[:, c]
        y = panel_reflectance_bgr[:, c]
        coeff = np.polyfit(x, y, 1)
        slopes[c] = float(coeff[0])
        intercepts[c] = float(coeff[1])
    return slopes, intercepts


def _apply_empirical_line(image: np.ndarray, slopes: np.ndarray, intercepts: np.ndarray) -> np.ndarray:
    image_f = image.astype(np.float32)
    out = np.empty_like(image_f, dtype=np.float32)
    for c in range(3):
        out[:, :, c] = slopes[c] * image_f[:, :, c] + intercepts[c]
    return np.clip(out, 0.0, 1.0)


def _write_overlay(image: np.ndarray, panels: list[dict], path: Path) -> None:
    overlay = image.copy()
    for panel in panels:
        x, y, w, h = [int(v) for v in panel['roi']]
        name = str(panel.get('name', 'panel'))
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            overlay,
            name,
            (x, max(20, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    cv2.imwrite(str(path), overlay)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    panel_cfg_path = Path(args.panel_config).resolve()
    calibration_image_path = Path(args.calibration_image).resolve()

    panels = _load_panel_config(panel_cfg_path)
    calibration_image = _read_image(calibration_image_path)

    flat_fields = None
    if args.flat_b and args.flat_g and args.flat_r:
        flat_fields = load_vignette_images([args.flat_b, args.flat_g, args.flat_r])
        calibration_image = apply_vignette_correction(
            calibration_image,
            flat_fields,
            dark_current=(args.dark_current_b, args.dark_current_g, args.dark_current_r),
        )

    _write_overlay(calibration_image, panels, out_dir / 'panel_rois_overlay.png')

    panel_means: list[np.ndarray] = []
    panel_refs: list[np.ndarray] = []
    panel_rows: list[dict] = []
    for panel in panels:
        roi = panel['roi']
        refl = panel['reflectance_bgr']
        if len(refl) != 3:
            raise RuntimeError(f'Panel reflectance_bgr must have 3 values: {panel}')
        mean_bgr = _roi_mean_bgr(calibration_image, roi)
        ref_bgr = np.array(refl, dtype=np.float64)
        panel_means.append(mean_bgr)
        panel_refs.append(ref_bgr)
        panel_rows.append(
            {
                'name': panel.get('name', ''),
                'roi': [int(v) for v in roi],
                'mean_bgr': [float(v) for v in mean_bgr],
                'reflectance_bgr': [float(v) for v in ref_bgr],
            }
        )

    means = np.array(panel_means, dtype=np.float64)
    refs = np.array(panel_refs, dtype=np.float64)
    slopes, intercepts = _fit_empirical_line(means, refs)

    targets: list[Path] = []
    if args.apply_image:
        targets.append(Path(args.apply_image).resolve())
    if args.apply_dir:
        targets.extend(sorted(Path(args.apply_dir).resolve().glob(args.glob)))
    if not targets:
        targets = [calibration_image_path]

    apply_dir = out_dir / 'applied'
    apply_dir.mkdir(parents=True, exist_ok=True)

    pbar = tqdm(targets, desc='reflectance apply', unit='img')
    for src in pbar:
        image = _read_image(src)
        if flat_fields is not None:
            image = apply_vignette_correction(
                image,
                flat_fields,
                dark_current=(args.dark_current_b, args.dark_current_g, args.dark_current_r),
            )
        refl = _apply_empirical_line(image, slopes, intercepts)
        refl_f32_path = apply_dir / f'{src.stem}_reflectance_f32.tiff'
        refl_u16_path = apply_dir / f'{src.stem}_reflectance_u16.tiff'
        cv2.imwrite(str(refl_f32_path), refl.astype(np.float32))
        cv2.imwrite(str(refl_u16_path), np.clip(refl * 65535.0, 0.0, 65535.0).astype(np.uint16))

    report = {
        'calibration_image': str(calibration_image_path),
        'panel_config': str(panel_cfg_path),
        'panels': panel_rows,
        'model': {
            'equation': 'reflectance = slope * DN + intercept',
            'slope_bgr': [float(v) for v in slopes],
            'intercept_bgr': [float(v) for v in intercepts],
        },
        'applied_count': len(targets),
        'vignette_pre_applied': bool(flat_fields is not None),
    }
    (out_dir / 'reflectance_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f'Reflectance calibration complete. Outputs written to: {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
