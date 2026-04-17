#!/usr/bin/env python3
"""Empirical line reflectance calibration from MAPIR target panel ROIs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

from mapir_camera_core import apply_vignette_correction, load_vignette_images
from mapir_camera_core.raw_survey3 import load_survey3_raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Compute per-channel empirical-line reflectance calibration and apply it.'
    )
    parser.add_argument('--calibration-image', required=True)
    parser.add_argument('--panel-config', required=True)
    parser.add_argument(
        '--select-rois',
        action='store_true',
        help='Interactively select panel ROIs with OpenCV and save an updated panel config.',
    )
    parser.add_argument(
        '--select-rois-only',
        action='store_true',
        help='Select panel ROIs, write the updated panel config, and exit without calibrating.',
    )
    parser.add_argument(
        '--aruco-mode',
        action='store_true',
        help='Automatically detect panel ROIs using ArUco marker fiducial (requires --aruco-corners or auto-detection).',
    )
    parser.add_argument(
        '--aruco-corners',
        default='',
        help='Manual ArUco marker corner coordinates as JSON array: "[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]". Overrides auto-detection.',
    )
    parser.add_argument(
        '--target-layout',
        default='',
        help='Path to target layout JSON (required for --aruco-mode). Defaults to config/target_layout.json if in repo.',
    )
    parser.add_argument(
        '--camera-intrinsics',
        default='',
        help='Path to camera intrinsics YAML (required for --aruco-mode). Defaults to config/mapir3_ocn_1920x1440.yaml if in repo.',
    )
    parser.add_argument(
        '--panel-config-out',
        default='',
        help='Optional output JSON path for the ROI-updated panel config.',
    )
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
    parser.add_argument(
        '--allow-8bit',
        action='store_true',
        help='Allow uint8 inputs (not ideal for radiometric workflows).',
    )
    parser.add_argument(
        '--panel-min-frac',
        type=float,
        default=0.02,
        help='Reject panel ROIs whose mean DN is below this fraction of full-scale.',
    )
    parser.add_argument(
        '--panel-max-frac',
        type=float,
        default=0.98,
        help='Reject panel ROIs whose mean DN is above this fraction of full-scale.',
    )
    return parser.parse_args()


def _read_image(path: Path) -> np.ndarray:
    if path.suffix.upper() == '.RAW':
        # Use the repository's Bayer decoder, but keep preview/calibration pixels in
        # the natural non-inverted demosaiced space so ROI picking remains visible.
        bayer_u16 = load_survey3_raw(path)
        return cv2.cvtColor(bayer_u16, cv2.COLOR_BAYER_RG2BGR)
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f'Could not read image: {path}')
    if image.ndim != 3 or image.shape[2] != 3:
        raise RuntimeError(f'Expected 3-channel image, got shape={image.shape} for {path}')
    return image


def _dtype_max(image: np.ndarray) -> float:
    if image.dtype == np.uint16:
        return 65535.0
    if image.dtype == np.uint8:
        return 255.0
    raise RuntimeError(f'Unsupported image dtype for reflectance calibration: {image.dtype}')


def _load_panel_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or 'panels' not in data:
        raise RuntimeError('Panel config must be a JSON object with key "panels"')
    panels = data['panels']
    if not isinstance(panels, list) or len(panels) < 2:
        raise RuntimeError('Panel config must contain at least two panel entries')
    data['panels'] = panels
    return data


def _write_panel_config(path: Path, panel_config: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(panel_config, indent=2), encoding='utf-8')


def _to_display_u8(image: np.ndarray) -> np.ndarray:
    if image.dtype == np.uint8:
        return image
    image_f = image.astype(np.float32)
    max_value = float(np.max(image_f))
    if max_value <= 0.0:
        return np.zeros_like(image, dtype=np.uint8)
    scaled = np.clip(image_f * (255.0 / max_value), 0.0, 255.0)
    return scaled.astype(np.uint8)


def _screen_size() -> tuple[int, int]:
    for env_key in ('DISPLAY_WIDTH', 'DISPLAY_HEIGHT'):
        if env_key in os.environ:
            break
    width = int(os.environ.get('DISPLAY_WIDTH', '1920'))
    height = int(os.environ.get('DISPLAY_HEIGHT', '1080'))
    return max(width, 640), max(height, 480)


def _fit_for_display(image: np.ndarray, margin: int = 120) -> tuple[np.ndarray, float]:
    screen_w, screen_h = _screen_size()
    max_w = max(screen_w - margin, 320)
    max_h = max(screen_h - margin, 240)
    h, w = image.shape[:2]
    scale = min(float(max_w) / float(w), float(max_h) / float(h), 1.0)
    if scale >= 0.999:
        return image, 1.0
    resized = cv2.resize(
        image,
        (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
        interpolation=cv2.INTER_AREA,
    )
    return resized, scale


def _select_panel_rois(image: np.ndarray, panels: list[dict]) -> list[dict]:
    if len(panels) < 1:
        raise RuntimeError('Need at least one panel entry to select ROIs')

    preview = _to_display_u8(image)

    # Use full resolution - selectROIs has better zoom/pan than pre-scaled version
    # Only scale if image is extremely large for performance
    h, w = preview.shape[:2]
    if w > 5000 or h > 4000:
        # Only scale very large images
        preview, scale = _fit_for_display(preview, margin=200)
    else:
        scale = 1.0

    window_name = 'Select reflectance panel ROIs'
    prompt = (
        "Select panel ROIs in config order. Drag one box per panel, then press Enter. "
        "Use scroll wheel to zoom, and zoom controls to adjust view. "
        "Esc cancels."
    )
    print(prompt)
    print('Panel order:', ', '.join(str(panel.get('name', f'panel_{idx + 1}')) for idx, panel in enumerate(panels)))

    rois = cv2.selectROIs(window_name, preview, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window_name)

    if rois is None or len(rois) == 0:
        raise RuntimeError('No ROIs selected')
    if len(rois) != len(panels):
        raise RuntimeError(
            f'Selected {len(rois)} ROIs but panel config expects {len(panels)} panels'
        )

    updated_panels: list[dict] = []
    for panel, roi in zip(panels, rois):
        x, y, w, h = [int(v) for v in roi]
        if scale != 1.0:
            x = int(round(x / scale))
            y = int(round(y / scale))
            w = int(round(w / scale))
            h = int(round(h / scale))
        updated = dict(panel)
        updated['roi'] = [x, y, w, h]
        updated_panels.append(updated)
    return updated_panels


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


def _bbox_to_roi_xywh(bounding_box: tuple[int, int, int, int]) -> list[int]:
    """Convert (x_min, y_min, x_max, y_max) into the ROI format [x, y, w, h]."""
    x_min, y_min, x_max, y_max = [int(v) for v in bounding_box]
    return [
        x_min,
        y_min,
        max(0, x_max - x_min),
        max(0, y_max - y_min),
    ]


def _detect_panels_with_manual_aruco(
    image: np.ndarray,
    marker_corners_px: np.ndarray,
    target_layout_path: Path,
    camera_intrinsics_path: Path,
    output_dir: Path,
) -> tuple[list[dict], dict]:
    """
    Detect panel ROIs using manually-specified ArUco marker corners.

    Args:
        image: Calibration image (BGR, uint16 or uint8)
        marker_corners_px: ArUco marker corners as (4, 2) array in image coordinates
        target_layout_path: Path to target_layout.json
        camera_intrinsics_path: Path to camera intrinsics YAML
        output_dir: Output directory for diagnostic images

    Returns:
        Tuple of (updated_panels, detection_metadata)
    """
    from target_detection import (
        estimate_board_pose,
        project_panel_rois,
        load_target_layout,
        load_camera_intrinsics_from_yaml,
        ArUcoDetection,
    )

    # Load configurations
    target_layout = load_target_layout(target_layout_path)
    camera_matrix, dist_coeffs = load_camera_intrinsics_from_yaml(camera_intrinsics_path)

    # Create synthetic ArUco detection from manual coordinates
    marker_center = marker_corners_px.mean(axis=0)
    aruco = ArUcoDetection(
        marker_id=9,  # From target_layout.json
        marker_corners_px=marker_corners_px.astype(np.float32),
        marker_center_px=tuple(marker_center),
    )

    # Parse target layout
    aruco_config = target_layout.get('aruco_marker', {})
    marker_size_mm = aruco_config.get('size_mm', 40.0)
    panels_3d = target_layout.get('panels_in_board_coordinates', [])

    # Estimate pose from manual ArUco corners
    pose = estimate_board_pose(aruco, marker_size_mm, camera_matrix, dist_coeffs)
    if pose is None:
        raise RuntimeError('Failed to estimate board pose from ArUco marker corners')

    # Project panel ROIs
    image_shape = image.shape[:2]
    panel_rois = project_panel_rois(panels_3d, pose, camera_matrix, dist_coeffs, image_shape)

    # Convert to panel config format
    updated_panels = []
    for panel in panel_rois:
        updated = {
            'name': panel.name,
            'roi': _bbox_to_roi_xywh(panel.bounding_box),
            'reflectance_bgr': [0.5, 0.5, 0.5],
        }
        updated_panels.append(updated)

    metadata = {
        'method': 'aruco_manual',
        'marker_corners_px': marker_corners_px.tolist(),
        'board_pose_reprojection_error_px': pose.reprojection_error,
        'detected_panel_count': len(panel_rois),
    }

    return updated_panels, metadata


def _detect_panels_with_aruco(
    image: np.ndarray,
    target_layout_path: Path,
    camera_intrinsics_path: Path,
    output_dir: Path,
) -> tuple[list[dict], dict]:
    """
    Detect panel ROIs using ArUco marker fiducial detection.

    Args:
        image: Calibration image (BGR, uint16 or uint8)
        target_layout_path: Path to target_layout.json
        camera_intrinsics_path: Path to camera intrinsics YAML
        output_dir: Output directory for diagnostic images

    Returns:
        Tuple of (updated_panels, detection_metadata) where updated_panels have 'roi' fields
        and detection_metadata contains ArUco and pose information.
    """
    # Import here to avoid circular imports
    from target_detection import (
        detect_target_with_aruco,
        load_target_layout,
        load_camera_intrinsics_from_yaml,
    )

    # Load configurations
    target_layout = load_target_layout(target_layout_path)
    camera_matrix, dist_coeffs = load_camera_intrinsics_from_yaml(camera_intrinsics_path)

    # Detect target and panels
    aruco_config = target_layout.get('aruco_marker', {})
    dictionary_name = aruco_config.get('dictionary', '4X4_250')
    marker_id = aruco_config.get('marker_id', None)

    panel_rois, metadata = detect_target_with_aruco(
        image,
        target_layout_path,
        camera_intrinsics_path,
        dictionary_name=dictionary_name,
        marker_id=marker_id,
    )

    if panel_rois is None:
        # Save diagnostic image for troubleshooting
        vis = _to_display_u8(image).copy()
        cv2.putText(vis, f"ArUco Detection Failed", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
        cv2.putText(vis, f"Check ArUco dictionary/ID in target_layout.json", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 1)
        cv2.putText(vis, f"Dictionary: {dictionary_name}, ID: {marker_id}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1)
        cv2.imwrite(str(output_dir / 'aruco_detection_failed.png'), vis)
        raise RuntimeError(f'ArUco detection failed\nSaved diagnostic image: aruco_detection_failed.png')

    # Convert detected panels to panel config format
    updated_panels: list[dict] = []
    for panel in panel_rois:
        updated = {
            'name': panel.name,
            'roi': _bbox_to_roi_xywh(panel.bounding_box),
            'reflectance_bgr': [0.5, 0.5, 0.5],  # Placeholder; should be from target_layout
        }
        updated_panels.append(updated)

    # Metadata about detection (already provided by detect_target_with_aruco)
    if metadata is None:
        metadata = {'method': 'aruco_auto', 'detected_panel_count': len(updated_panels)}

    return updated_panels, metadata


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    panel_cfg_path = Path(args.panel_config).resolve()
    calibration_image_path = Path(args.calibration_image).resolve()

    panel_config = _load_panel_config(panel_cfg_path)
    panels = panel_config['panels']
    calibration_image = _read_image(calibration_image_path)
    if calibration_image.dtype == np.uint8 and not args.allow_8bit:
        raise RuntimeError(
            'calibration_image is uint8; this is not recommended for reflectance calibration. '
            'Use 16-bit images or pass --allow-8bit explicitly.'
        )
    cal_dtype_max = _dtype_max(calibration_image)

    flat_fields = None
    if args.flat_b and args.flat_g and args.flat_r:
        flat_fields = load_vignette_images([args.flat_b, args.flat_g, args.flat_r])
        calibration_image = apply_vignette_correction(
            calibration_image,
            flat_fields,
            dark_current=(args.dark_current_b, args.dark_current_g, args.dark_current_r),
        )

    # Handle QR mode (auto-detect or manual)
    qr_metadata: dict = {}
    if args.aruco_mode:
        # Resolve target layout and camera intrinsics paths
        if args.target_layout:
            target_layout_path = Path(args.target_layout).resolve()
        else:
            target_layout_path = calibration_image_path.parent.parent / 'target_layout.json'
            if not target_layout_path.exists():
                raise RuntimeError(
                    'Target layout not found. Provide --target-layout or place target_layout.json in config/'
                )

        if args.camera_intrinsics:
            camera_intrinsics_path = Path(args.camera_intrinsics).resolve()
        else:
            camera_intrinsics_path = calibration_image_path.parent.parent / 'mapir3_ocn_1920x1440.yaml'
            if not camera_intrinsics_path.exists():
                raise RuntimeError(
                    'Camera intrinsics not found. Provide --camera-intrinsics or place mapir3_ocn_1920x1440.yaml in config/'
                )

        # Check if manual ArUco corners provided
        if args.aruco_corners:
            print(f'ArUco mode: Using manual coordinates')
            marker_corners = np.array(json.loads(args.aruco_corners), dtype=np.float32)
            detected_panels, aruco_metadata = _detect_panels_with_manual_aruco(
                calibration_image, marker_corners, target_layout_path, camera_intrinsics_path, out_dir
            )
            print(f'✓ Detected {len(detected_panels)} panels from manual ArUco corners')
        else:
            # Try auto-detection
            print(f'ArUco mode: Attempting auto-detection')
            detected_panels, aruco_metadata = _detect_panels_with_aruco(
                calibration_image, target_layout_path, camera_intrinsics_path, out_dir
            )
            print(f'✓ Detected {len(detected_panels)} panels via auto ArUco detection')
        qr_metadata = aruco_metadata or {}

        # Update panel config with detected ROIs but keep reflectance values from original
        for detected, orig in zip(detected_panels, panels):
            detected['reflectance_bgr'] = orig.get('reflectance_bgr', [0.5, 0.5, 0.5])

        panel_config['panels'] = detected_panels
        panels = detected_panels

        # Save detected panel config
        if args.panel_config_out:
            panel_cfg_out_path = Path(args.panel_config_out).resolve()
        else:
            panel_cfg_out_path = out_dir / 'reflectance_panels.detected.json'
        _write_panel_config(panel_cfg_out_path, panel_config)
        print(f'Detected panel config written to: {panel_cfg_out_path}')

    elif args.select_rois or args.select_rois_only:
        panel_config['panels'] = _select_panel_rois(calibration_image, panels)
        panels = panel_config['panels']
        if args.panel_config_out:
            panel_cfg_out_path = Path(args.panel_config_out).resolve()
        else:
            panel_cfg_out_path = out_dir / 'reflectance_panels.selected.json'
        _write_panel_config(panel_cfg_out_path, panel_config)
        print(f'Updated panel config written to: {panel_cfg_out_path}')
        if args.select_rois_only:
            return 0

    _write_overlay(calibration_image, panels, out_dir / 'panel_rois_overlay.png')

    panel_means: list[np.ndarray] = []
    panel_refs: list[np.ndarray] = []
    panel_rows: list[dict] = []
    rejected_panels: list[dict] = []
    min_dn = float(args.panel_min_frac) * cal_dtype_max
    max_dn = float(args.panel_max_frac) * cal_dtype_max
    if min_dn >= max_dn:
        raise RuntimeError('Invalid panel DN thresholds: panel-min-frac must be < panel-max-frac')

    for panel in panels:
        roi = panel['roi']
        refl = panel['reflectance_bgr']
        if len(refl) != 3:
            raise RuntimeError(f'Panel reflectance_bgr must have 3 values: {panel}')
        mean_bgr = _roi_mean_bgr(calibration_image, roi)
        within_range = bool(np.all(mean_bgr >= min_dn) and np.all(mean_bgr <= max_dn))
        if not within_range:
            rejected_panels.append(
                {
                    'name': panel.get('name', ''),
                    'roi': [int(v) for v in roi],
                    'mean_bgr': [float(v) for v in mean_bgr],
                    'reason': f'mean DN outside [{min_dn:.2f}, {max_dn:.2f}]',
                }
            )
            continue
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

    if len(panel_means) < 2:
        raise RuntimeError(
            f'Need at least 2 valid panels after DN quality filtering; got {len(panel_means)}'
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
        if image.dtype == np.uint8 and not args.allow_8bit:
            raise RuntimeError(
                f'Input image {src} is uint8; use 16-bit images or pass --allow-8bit.'
            )
        _dtype_max(image)
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
        'panel_config_metadata': {
            key: value for key, value in panel_config.items() if key != 'panels'
        },
        'panels': panel_rows,
        'model': {
            'equation': 'reflectance = slope * DN + intercept',
            'slope_bgr': [float(v) for v in slopes],
            'intercept_bgr': [float(v) for v in intercepts],
        },
        'quality_thresholds': {
            'panel_min_frac': float(args.panel_min_frac),
            'panel_max_frac': float(args.panel_max_frac),
            'panel_dn_min': float(min_dn),
            'panel_dn_max': float(max_dn),
        },
        'rejected_panels': rejected_panels,
        'applied_count': len(targets),
        'vignette_pre_applied': bool(flat_fields is not None),
        'allow_8bit': bool(args.allow_8bit),
        'panel_detection_method': str(qr_metadata.get('method', 'manual')) if qr_metadata else 'manual',
        'qr_detection': qr_metadata if qr_metadata else None,
    }
    (out_dir / 'reflectance_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f'Reflectance calibration complete. Outputs written to: {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
