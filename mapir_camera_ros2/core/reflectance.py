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
from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np


class ReflectanceFitError(ValueError):
    """Raised when reflectance calibration cannot be fitted."""


def normalize_patch_reflectances(
    patch_reflectances: Sequence[object],
    *,
    num_patches: int,
    num_channels: int = 3,
) -> np.ndarray:
    """
    Normalize patch reflectances into a (num_patches, num_channels) float32 array.

    Each patch entry can be either a scalar or a per-channel sequence.
    """
    if len(patch_reflectances) != num_patches:
        raise ValueError(
            f'Expected {num_patches} patch reflectances, got {len(patch_reflectances)}'
        )

    out = np.zeros((num_patches, num_channels), dtype=np.float32)
    for idx, value in enumerate(patch_reflectances):
        if np.isscalar(value):
            out[idx, :] = float(value)
            continue

        seq = list(value)
        if len(seq) != num_channels:
            raise ValueError(
                f'Patch {idx} reflectance expects {num_channels} values, got {len(seq)}'
            )
        out[idx, :] = np.asarray(seq, dtype=np.float32)

    return out


def sample_patch_means(
    image: np.ndarray,
    patch_quads: Sequence[np.ndarray],
    *,
    min_area_px: int = 0,
) -> np.ndarray:
    """
    Compute per-channel mean DN for each patch quad.

    Returns a (num_patches, channels) float32 array with NaN for invalid patches.
    """
    if image.ndim not in (2, 3):
        raise ValueError(f'Expected 2D or 3D image, got shape={image.shape}')

    height, width = image.shape[:2]
    channels = 1 if image.ndim == 2 else int(image.shape[2])

    means = np.full((len(patch_quads), channels), np.nan, dtype=np.float32)

    for idx, quad in enumerate(patch_quads):
        if quad is None:
            continue

        quad_arr = np.asarray(quad, dtype=np.float32).reshape((-1, 2))
        if quad_arr.shape != (4, 2):
            raise ValueError('Patch quad must be shaped (4,2)')
        if not np.isfinite(quad_arr).all():
            continue

        quad_int = np.round(quad_arr).astype(np.int32)
        x, y, w, h = cv2.boundingRect(quad_int)
        if w <= 0 or h <= 0:
            continue

        x0 = max(x, 0)
        y0 = max(y, 0)
        x1 = min(x + w, width)
        y1 = min(y + h, height)
        if x1 <= x0 or y1 <= y0:
            continue

        roi = image[y0:y1, x0:x1]
        mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        shifted = quad_int - np.array([x0, y0], dtype=np.int32)
        cv2.fillPoly(mask, [shifted], 255)

        area_px = int(mask.sum() / 255)
        if min_area_px > 0 and area_px < min_area_px:
            continue

        if channels == 1:
            means[idx, 0] = float(cv2.mean(roi, mask=mask)[0])
        else:
            mean_vals = cv2.mean(roi, mask=mask)[:channels]
            means[idx, :] = np.asarray(mean_vals, dtype=np.float32)

    return means


def sample_patch_stats(
    image: np.ndarray,
    patch_quads: Sequence[np.ndarray],
    *,
    method: str = 'median',
    warp_size: int = 32,
    trim_ratio: float = 0.1,
    min_area_px: int = 0,
) -> np.ndarray:
    """
    Compute per-channel robust statistics for each patch quad.

    The patch quad is warped to a square (warp_size x warp_size) before
    computing the statistic to reduce perspective bias.
    """
    if image.ndim not in (2, 3):
        raise ValueError(f'Expected 2D or 3D image, got shape={image.shape}')

    warp = max(2, int(warp_size))
    method_norm = str(method).strip().lower()
    channels = 1 if image.ndim == 2 else int(image.shape[2])

    stats = np.full((len(patch_quads), channels), np.nan, dtype=np.float32)
    src = image.astype(np.float32, copy=False)
    dst_quad = np.array(
        [[0.0, 0.0], [warp - 1.0, 0.0], [warp - 1.0, warp - 1.0], [0.0, warp - 1.0]],
        dtype=np.float32,
    )

    for idx, quad in enumerate(patch_quads):
        if quad is None:
            continue

        quad_arr = np.asarray(quad, dtype=np.float32).reshape((-1, 2))
        if quad_arr.shape != (4, 2):
            raise ValueError('Patch quad must be shaped (4,2)')

        if min_area_px > 0:
            area = float(cv2.contourArea(quad_arr))
            if area < float(min_area_px):
                continue

        matrix = cv2.getPerspectiveTransform(quad_arr, dst_quad)
        patch = cv2.warpPerspective(src, matrix, (warp, warp), flags=cv2.INTER_LINEAR)
        if channels == 1 and patch.ndim == 3:
            patch = patch[:, :, 0]

        stats[idx, :] = _robust_patch_stat(
            patch, method=method_norm, trim_ratio=trim_ratio, channels=channels
        )

    return stats


def _robust_patch_stat(
    patch: np.ndarray,
    *,
    method: str,
    trim_ratio: float,
    channels: int,
) -> np.ndarray:
    if channels == 1:
        values = _robust_stat_1d(patch.reshape(-1), method, trim_ratio)
        return np.array([values], dtype=np.float32)

    out = np.empty((channels,), dtype=np.float32)
    for ch in range(channels):
        out[ch] = _robust_stat_1d(patch[..., ch].reshape(-1), method, trim_ratio)
    return out


def _robust_stat_1d(
    values: np.ndarray,
    method: str,
    trim_ratio: float,
) -> float:
    clean = values[np.isfinite(values)]
    if clean.size == 0:
        return np.nan

    if method == 'mean':
        return float(np.mean(clean))
    if method == 'median':
        return float(np.median(clean))
    if method != 'trimmed_mean':
        raise ValueError(f'Unknown patch_stat method: {method}')

    ratio = float(np.clip(trim_ratio, 0.0, 0.49))
    if ratio <= 0.0:
        return float(np.mean(clean))

    sorted_vals = np.sort(clean, axis=None)
    cut = int(round(sorted_vals.size * ratio))
    if sorted_vals.size - 2 * cut <= 0:
        return float(np.mean(sorted_vals))
    return float(np.mean(sorted_vals[cut:-cut]))


def fit_linear_reflectance(
    patch_means: np.ndarray,
    patch_reflectances: np.ndarray,
    *,
    min_dn_range: float = 5.0,
    max_abs_slope: float = 2.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit per-channel linear reflectance model R = a * DN + b.

    Raises ReflectanceFitError on invalid inputs or failed fits.
    """
    if patch_means.shape != patch_reflectances.shape:
        raise ReflectanceFitError(
            f'patch_means shape {patch_means.shape} does not match '
            f'patch_reflectances shape {patch_reflectances.shape}'
        )

    if patch_means.ndim != 2:
        raise ReflectanceFitError('patch_means must be 2D (patches, channels)')

    num_channels = patch_means.shape[1]
    slopes = np.zeros((num_channels,), dtype=np.float32)
    intercepts = np.zeros((num_channels,), dtype=np.float32)

    for ch in range(num_channels):
        dn = patch_means[:, ch]
        ref = patch_reflectances[:, ch]

        mask = np.isfinite(dn) & np.isfinite(ref)
        if int(mask.sum()) < 2:
            raise ReflectanceFitError('Not enough valid patches for linear fit')

        dn_sel = dn[mask]
        ref_sel = ref[mask]

        dn_span = float(np.ptp(dn_sel))
        if dn_span < float(min_dn_range):
            raise ReflectanceFitError('Patch DN range too small for a stable fit')

        A = np.vstack([dn_sel, np.ones_like(dn_sel)]).T
        sol, *_ = np.linalg.lstsq(A, ref_sel, rcond=None)
        slope = float(sol[0])
        intercept = float(sol[1])

        if not np.isfinite(slope) or not np.isfinite(intercept):
            raise ReflectanceFitError('Non-finite fit parameters')
        if slope <= 0.0:
            raise ReflectanceFitError('Negative slope in linear fit')
        if abs(slope) > float(max_abs_slope):
            raise ReflectanceFitError('Slope exceeds max_abs_slope guardrail')

        slopes[ch] = slope
        intercepts[ch] = intercept

    return slopes, intercepts


def fit_gamma_reflectance(
    patch_dn: np.ndarray,
    patch_reflectances: np.ndarray,
    *,
    min_dn_range: float = 1e-3,
    min_gamma: float = 0.2,
    max_gamma: float = 5.0,
    max_gain: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit per-channel gamma model R = gain * (DN ** gamma).

    DN values must be normalized to [0, 1] before fitting.
    """
    if patch_dn.shape != patch_reflectances.shape:
        raise ReflectanceFitError(
            f'patch_dn shape {patch_dn.shape} does not match '
            f'patch_reflectances shape {patch_reflectances.shape}'
        )

    if patch_dn.ndim != 2:
        raise ReflectanceFitError('patch_dn must be 2D (patches, channels)')

    num_channels = patch_dn.shape[1]
    gains = np.zeros((num_channels,), dtype=np.float32)
    gammas = np.zeros((num_channels,), dtype=np.float32)

    eps = np.float32(1e-6)
    for ch in range(num_channels):
        dn = patch_dn[:, ch]
        ref = patch_reflectances[:, ch]

        mask = np.isfinite(dn) & np.isfinite(ref) & (dn > 0.0) & (ref > 0.0)
        if int(mask.sum()) < 2:
            raise ReflectanceFitError('Not enough valid patches for gamma fit')

        dn_sel = dn[mask]
        ref_sel = ref[mask]

        dn_span = float(np.ptp(dn_sel))
        if dn_span < float(min_dn_range):
            raise ReflectanceFitError('Patch DN range too small for a gamma fit')

        x = np.log(np.maximum(dn_sel, eps))
        y = np.log(np.maximum(ref_sel, eps))

        A = np.vstack([x, np.ones_like(x)]).T
        sol, *_ = np.linalg.lstsq(A, y, rcond=None)
        gamma = float(sol[0])
        log_gain = float(sol[1])
        gain = float(np.exp(log_gain))

        if not np.isfinite(gamma) or not np.isfinite(gain):
            raise ReflectanceFitError('Non-finite gamma fit parameters')
        if gamma < float(min_gamma) or gamma > float(max_gamma):
            raise ReflectanceFitError('Gamma outside guardrails')
        if gain <= 0.0 or gain > float(max_gain):
            raise ReflectanceFitError('Gain outside guardrails')

        gammas[ch] = gamma
        gains[ch] = gain

    return gains, gammas


def apply_linear_reflectance(
    image: np.ndarray,
    slopes: np.ndarray,
    intercepts: np.ndarray,
    *,
    clamp_min: float = 0.0,
    clamp_max: float = 1.2,
) -> np.ndarray:
    """Apply a per-channel linear reflectance model to an image."""
    if image.ndim not in (2, 3):
        raise ValueError(f'Expected 2D or 3D image, got shape={image.shape}')

    img = image.astype(np.float32, copy=False)
    if image.ndim == 2:
        out = np.empty_like(img, dtype=np.float32)
        np.multiply(img, float(slopes[0]), out=out)
        out += float(intercepts[0])
    else:
        slopes = np.asarray(slopes, dtype=np.float32).reshape((1, 1, -1))
        intercepts = np.asarray(intercepts, dtype=np.float32).reshape((1, 1, -1))
        out = np.empty_like(img, dtype=np.float32)
        np.multiply(img, slopes, out=out)
        out += intercepts

    np.clip(out, float(clamp_min), float(clamp_max), out=out)
    return out


def apply_gamma_reflectance(
    image: np.ndarray,
    gains: np.ndarray,
    gammas: np.ndarray,
    *,
    dn_scale: float = 1.0,
    clamp_min: float = 0.0,
    clamp_max: float = 1.2,
) -> np.ndarray:
    """Apply a per-channel gamma reflectance model to an image."""
    if image.ndim not in (2, 3):
        raise ValueError(f'Expected 2D or 3D image, got shape={image.shape}')

    img = image.astype(np.float32, copy=False) * float(dn_scale)
    if image.ndim == 2:
        out = np.empty_like(img, dtype=np.float32)
        np.power(img, float(gammas[0]), out=out)
        out *= float(gains[0])
    else:
        gains = np.asarray(gains, dtype=np.float32).reshape((1, 1, -1))
        gammas = np.asarray(gammas, dtype=np.float32).reshape((1, 1, -1))
        out = np.empty_like(img, dtype=np.float32)
        np.power(img, gammas, out=out)
        out *= gains

    np.clip(out, float(clamp_min), float(clamp_max), out=out)
    return out
