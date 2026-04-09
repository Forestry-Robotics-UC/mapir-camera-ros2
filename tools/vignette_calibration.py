#!/usr/bin/env python3
"""Generate MAPIR-style vignette flat-field maps from captured calibration-board frames.

The map generation aligns with MAPIR flat-field workflow semantics:
per-channel correction maps (B/G/R) are estimated from uniformly lit captures and
stored as floating point divisor maps for runtime correction.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

from mapir_camera_core import (
    apply_vignette_correction,
    configure_v4l2_capture,
    open_v4l2_capture,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Capture flat-field frames and export vignette correction maps.'
    )
    parser.add_argument('--video-device', default='/dev/video0')
    parser.add_argument('--pixel-format', default='MJPG', choices=['MJPG', 'H264'])
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--frames', type=int, default=80)
    parser.add_argument('--warmup-frames', type=int, default=20)
    parser.add_argument('--blur-kernel', type=int, default=41)
    parser.add_argument('--epsilon', type=float, default=1e-3)
    parser.add_argument('--out-dir', default='/outputs/vignette_calibration')
    return parser.parse_args()


def _ensure_odd(value: int) -> int:
    return value if value % 2 == 1 else value + 1


def _normalize_channel(channel: np.ndarray, eps: float) -> np.ndarray:
    center = channel[channel.shape[0] // 2, channel.shape[1] // 2]
    center = max(float(center), eps)
    normalized = channel / center
    return np.clip(normalized, eps, None).astype(np.float32)


def _write_preview(normalized: np.ndarray, path: Path) -> None:
    scaled = normalized / np.max(normalized)
    img = np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
    cv2.imwrite(str(path), img)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).resolve()
    raw_dir = out_dir / 'raw_frames'
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_dir.mkdir(parents=True, exist_ok=True)

    cap = open_v4l2_capture(str(args.video_device))
    if not cap.isOpened():
        raise RuntimeError(f'Failed to open video_device={args.video_device}')
    negotiation = configure_v4l2_capture(
        cap,
        req_width=int(args.width),
        req_height=int(args.height),
        req_fps=float(args.fps),
        pixel_format=str(args.pixel_format),
    )

    for _ in range(int(args.warmup_frames)):
        cap.read()

    captured: list[np.ndarray] = []
    pbar = tqdm(total=int(args.frames), desc='vignette captures', unit='frame')
    try:
        while len(captured) < int(args.frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            captured.append(frame.copy())
            idx = len(captured) - 1
            cv2.imwrite(str(raw_dir / f'frame_{idx:03d}.png'), frame)
            pbar.update(1)
    finally:
        pbar.close()
        cap.release()

    stack = np.stack(captured, axis=0).astype(np.float32)
    median_frame = np.median(stack, axis=0)
    b, g, r = cv2.split(median_frame)

    kernel = _ensure_odd(max(3, int(args.blur_kernel)))
    b_blur = cv2.GaussianBlur(b, (kernel, kernel), 0)
    g_blur = cv2.GaussianBlur(g, (kernel, kernel), 0)
    r_blur = cv2.GaussianBlur(r, (kernel, kernel), 0)

    eps = max(1e-9, float(args.epsilon))
    b_norm = _normalize_channel(b_blur, eps)
    g_norm = _normalize_channel(g_blur, eps)
    r_norm = _normalize_channel(r_blur, eps)

    # Store canonical float maps for the ROS node's vignette correction inputs.
    cv2.imwrite(str(out_dir / 'flat_b.tiff'), b_norm)
    cv2.imwrite(str(out_dir / 'flat_g.tiff'), g_norm)
    cv2.imwrite(str(out_dir / 'flat_r.tiff'), r_norm)
    _write_preview(b_norm, out_dir / 'flat_b_preview.png')
    _write_preview(g_norm, out_dir / 'flat_g_preview.png')
    _write_preview(r_norm, out_dir / 'flat_r_preview.png')

    np.savez(
        out_dir / 'flat_fields.npz',
        flat_b=b_norm,
        flat_g=g_norm,
        flat_r=r_norm,
        median_frame=median_frame,
    )

    sample_frame = captured[-1]
    corrected = apply_vignette_correction(
        sample_frame,
        (b_norm, g_norm, r_norm),
        dark_current=(0, 0, 0),
    )
    side_by_side = np.hstack((sample_frame, corrected))
    cv2.imwrite(str(out_dir / 'sample_before_after.png'), side_by_side)

    report = {
        'frames_used': int(args.frames),
        'warmup_frames': int(args.warmup_frames),
        'shape': [int(median_frame.shape[0]), int(median_frame.shape[1]), 3],
        'blur_kernel': int(kernel),
        'epsilon': float(eps),
        'negotiation': {
            'width': int(negotiation.width),
            'height': int(negotiation.height),
            'fps': float(negotiation.fps),
            'fourcc': str(negotiation.fourcc_str),
        },
        'flat_stats': {
            'b_min_max': [float(np.min(b_norm)), float(np.max(b_norm))],
            'g_min_max': [float(np.min(g_norm)), float(np.max(g_norm))],
            'r_min_max': [float(np.min(r_norm)), float(np.max(r_norm))],
        },
    }
    (out_dir / 'vignette_report.json').write_text(
        json.dumps(report, indent=2), encoding='utf-8'
    )

    print(f'Vignette calibration complete. Outputs written to: {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
