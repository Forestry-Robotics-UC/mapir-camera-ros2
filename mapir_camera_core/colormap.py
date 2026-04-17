from __future__ import annotations

from typing import Final

import cv2
import numpy as np

_OPENCV_COLORMAP_CONSTS: Final[dict[str, str]] = {
    'autumn': 'COLORMAP_AUTUMN',
    'bone': 'COLORMAP_BONE',
    'cool': 'COLORMAP_COOL',
    'hot': 'COLORMAP_HOT',
    'hsv': 'COLORMAP_HSV',
    'jet': 'COLORMAP_JET',
    'ocean': 'COLORMAP_OCEAN',
    'parula': 'COLORMAP_PARULA',
    'pink': 'COLORMAP_PINK',
    'rainbow': 'COLORMAP_RAINBOW',
    'spring': 'COLORMAP_SPRING',
    'summer': 'COLORMAP_SUMMER',
    'winter': 'COLORMAP_WINTER',
    'inferno': 'COLORMAP_INFERNO',
    'magma': 'COLORMAP_MAGMA',
    'plasma': 'COLORMAP_PLASMA',
    'viridis': 'COLORMAP_VIRIDIS',
    'cividis': 'COLORMAP_CIVIDIS',
    'turbo': 'COLORMAP_TURBO',
}


def supported_colormaps() -> set[str]:
    """Return supported colormap names (lowercase)."""
    supported = {'gray', 'custom'}
    for name, const in _OPENCV_COLORMAP_CONSTS.items():
        if hasattr(cv2, const):
            supported.add(name)
    return supported


def parse_custom_colormap(spec: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Parse a custom colormap spec.

    The spec format is: 'value,r,g,b; value,r,g,b; ...'

    Example spec: '-1,0,0,0; 0,0,255,0; 1,255,255,255'

    Returns a tuple (values, colors_bgr), where:
      - values is float32 array of shape (N,)
      - colors_bgr is uint8 array of shape (N,3)
    """
    text = str(spec).strip()
    if not text:
        raise ValueError('custom colormap spec is empty')

    points: list[tuple[float, tuple[int, int, int]]] = []
    for raw_point in text.split(';'):
        item = raw_point.strip()
        if not item:
            continue
        item = item.replace(':', ',')
        parts = [p.strip() for p in item.split(',') if p.strip()]
        if len(parts) != 4:
            raise ValueError(
                'custom colormap points must have 4 values: value,r,g,b '
                f'(got {len(parts)} in {raw_point!r})'
            )
        value = float(parts[0])
        r = int(parts[1])
        g = int(parts[2])
        b = int(parts[3])
        for c in (r, g, b):
            if c < 0 or c > 255:
                raise ValueError(f'RGB values must be in [0,255] (got {r},{g},{b})')
        points.append((value, (b, g, r)))

    if len(points) < 2:
        raise ValueError('custom colormap must contain at least 2 points')

    points.sort(key=lambda x: x[0])

    values = np.asarray([p[0] for p in points], dtype=np.float32)
    if np.unique(values).size != values.size:
        raise ValueError('custom colormap contains duplicate value entries')

    colors_bgr = np.asarray([p[1] for p in points], dtype=np.uint8)
    return values, colors_bgr


def _normalize_to_uint8(values: np.ndarray, *, vmin: float, vmax: float) -> np.ndarray:
    if vmax <= vmin:
        raise ValueError('vmax must be > vmin')

    vals = np.asarray(values, dtype=np.float32)
    scaled = (vals - np.float32(vmin)) / np.float32(vmax - vmin)
    scaled = np.clip(scaled, 0.0, 1.0)
    scaled = np.nan_to_num(scaled, nan=0.0, posinf=1.0, neginf=0.0)
    return (scaled * np.float32(255.0)).astype(np.uint8)


def _apply_custom_colormap(
    values: np.ndarray,
    *,
    vmin: float,
    vmax: float,
    points_values: np.ndarray,
    points_colors_bgr: np.ndarray,
) -> np.ndarray:
    vals = np.asarray(values, dtype=np.float32)
    vals = np.clip(vals, np.float32(vmin), np.float32(vmax))

    x = points_values.astype(np.float32, copy=False)
    c = points_colors_bgr.astype(np.float32, copy=False)

    if x.size < 2:
        raise ValueError('custom colormap must have at least 2 points')

    vals = np.clip(vals, x[0], x[-1])

    idx = np.searchsorted(x, vals, side='right') - 1
    idx = np.clip(idx, 0, x.size - 2)

    x0 = x[idx]
    x1 = x[idx + 1]

    denom = np.maximum(x1 - x0, np.float32(1e-12))
    t = (vals - x0) / denom
    t = t[..., None]

    c0 = c[idx]
    c1 = c[idx + 1]
    out = (np.float32(1.0) - t) * c0 + t * c1

    out_u8 = np.round(out).astype(np.uint8)

    invalid = ~np.isfinite(np.asarray(values, dtype=np.float32))
    if invalid.any():
        out_u8[invalid] = np.array([0, 0, 0], dtype=np.uint8)

    return out_u8


def colorize_scalar_field(
    values: np.ndarray,
    *,
    vmin: float,
    vmax: float,
    colormap: str = 'viridis',
    custom_colormap: str = '',
) -> np.ndarray:
    """
    Colorize a float scalar field into a BGR8 image.

    `vmin` maps to the low end of the colormap and `vmax` maps to the high end.
    NaN/inf values are rendered as black.
    """
    name = str(colormap).strip().lower()
    if not name:
        raise ValueError('colormap name is empty')

    if name == 'custom':
        points_values, points_colors_bgr = parse_custom_colormap(custom_colormap)
        return _apply_custom_colormap(
            values,
            vmin=vmin,
            vmax=vmax,
            points_values=points_values,
            points_colors_bgr=points_colors_bgr,
        )

    u8 = _normalize_to_uint8(values, vmin=vmin, vmax=vmax)
    if name in ('gray', 'grey'):
        return np.dstack([u8, u8, u8])

    const = _OPENCV_COLORMAP_CONSTS.get(name)
    if const is None or not hasattr(cv2, const):
        supported = sorted(supported_colormaps())
        raise ValueError(f'Unsupported colormap {name!r}. Supported: {supported}')

    code = int(getattr(cv2, const))
    return cv2.applyColorMap(u8, code)
