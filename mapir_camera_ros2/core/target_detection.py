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

from functools import lru_cache
from typing import Sequence

import cv2
import numpy as np

DEFAULT_ARUCO_DICTIONARIES = (
    'DICT_4X4_50',
    'DICT_5X5_100',
    'DICT_6X6_250',
    'DICT_7X7_1000',
)


def to_gray_u8(image: np.ndarray) -> np.ndarray | None:
    """Normalize input image to contiguous uint8 grayscale."""
    if image is None or not isinstance(image, np.ndarray) or image.size == 0:
        return None

    if image.ndim == 2:
        gray = image
    elif image.ndim == 3:
        channels = image.shape[2]
        if channels == 1:
            gray = image[..., 0]
        elif channels == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif channels == 4:
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        else:
            return None
    else:
        return None

    if gray.dtype != np.uint8:
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        gray = gray.astype(np.uint8)

    return np.ascontiguousarray(gray)


def gray_variants(gray_u8: np.ndarray) -> list[np.ndarray]:
    """Generate alternative grayscale views for robust detection."""
    variants = [gray_u8]
    try:
        variants.append(cv2.equalizeHist(gray_u8))
    except Exception:
        pass
    try:
        variants.append(cv2.GaussianBlur(gray_u8, (3, 3), 0))
    except Exception:
        pass
    return variants


@lru_cache(maxsize=8)
def aruco_dictionary_from_name(name: str) -> cv2.aruco_Dictionary:
    """Return a predefined ArUco dictionary from a string name."""
    if not hasattr(cv2, 'aruco'):
        raise ValueError('OpenCV aruco module is not available')

    name_norm = name.strip().upper()
    if not name_norm.startswith('DICT_'):
        name_norm = f'DICT_{name_norm}'

    if not hasattr(cv2.aruco, name_norm):
        raise ValueError(f'Unknown ArUco dictionary: {name}')

    dict_id = getattr(cv2.aruco, name_norm)
    return cv2.aruco.getPredefinedDictionary(dict_id)


@lru_cache(maxsize=8)
def _aruco_detector_from_name(name: str):
    if not hasattr(cv2, 'aruco'):
        return None
    dictionary = aruco_dictionary_from_name(name)
    params = _aruco_params()
    if params is None:
        return None
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector_cls = getattr(cv2.aruco, 'ArucoDetector')
        if callable(detector_cls):
            return detector_cls(dictionary, params)
    return None


@lru_cache(maxsize=8)
def _aruco_params():
    if not hasattr(cv2, 'aruco'):
        return None
    if hasattr(cv2.aruco, 'DetectorParameters'):
        params = cv2.aruco.DetectorParameters()
    elif hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        return None

    if params is None:
        return None

    if hasattr(params, 'adaptiveThreshWinSizeMin'):
        params.adaptiveThreshWinSizeMin = 3
    if hasattr(params, 'adaptiveThreshWinSizeMax'):
        params.adaptiveThreshWinSizeMax = 23
    if hasattr(params, 'adaptiveThreshWinSizeStep'):
        params.adaptiveThreshWinSizeStep = 10
    if hasattr(params, 'minMarkerPerimeterRate'):
        params.minMarkerPerimeterRate = 0.03
    if hasattr(params, 'maxMarkerPerimeterRate'):
        params.maxMarkerPerimeterRate = 4.0
    if hasattr(params, 'polygonalApproxAccuracyRate'):
        params.polygonalApproxAccuracyRate = 0.03
    if hasattr(params, 'minCornerDistanceRate'):
        params.minCornerDistanceRate = 0.05
    if hasattr(params, 'cornerRefinementMethod') and hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(params, 'cornerRefinementWinSize'):
        params.cornerRefinementWinSize = 5
    if hasattr(params, 'cornerRefinementMaxIterations'):
        params.cornerRefinementMaxIterations = 30
    if hasattr(params, 'cornerRefinementMinAccuracy'):
        params.cornerRefinementMinAccuracy = 0.1
    if hasattr(params, 'minDistanceToBorder'):
        params.minDistanceToBorder = 3

    return params


def detect_aruco_fiducial(
    image: np.ndarray,
    *,
    dictionary_name: str,
    fiducial_id: int,
    min_area_px: float = 0.0,
    method: str = 'auto',
) -> np.ndarray | None:
    """Detect a single ArUco marker and return its corners (4x2) if found."""
    if not hasattr(cv2, 'aruco'):
        return None
    gray = to_gray_u8(image)
    if gray is None:
        return None

    method_norm = method.strip().lower()
    dict_norm = dictionary_name.strip().lower()
    if dict_norm in ('auto', 'any'):
        dictionaries = list(DEFAULT_ARUCO_DICTIONARIES)
    else:
        dictionaries = [dictionary_name]

    def _detect_with_opencv(img: np.ndarray, dict_name: str):
        if not hasattr(cv2.aruco, 'ArucoDetector'):
            return None, None
        detector = _aruco_detector_from_name(dict_name)
        if detector is None:
            return None, None
        corners_list, ids, _ = detector.detectMarkers(img)
        return corners_list, ids

    def _detect_with_legacy(img: np.ndarray, dict_name: str):
        if not hasattr(cv2.aruco, 'detectMarkers'):
            return None, None
        dictionary = aruco_dictionary_from_name(dict_name)
        params = _aruco_params()
        if params is None:
            corners_list, ids, _ = cv2.aruco.detectMarkers(img, dictionary)
        else:
            corners_list, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=params)
        return corners_list, ids

    variants = gray_variants(gray)

    corners_list = None
    ids = None
    methods = []
    if method_norm == 'auto':
        methods = ['opencv', 'legacy']
    else:
        methods = [method_norm]

    for dict_name in dictionaries:
        try:
            aruco_dictionary_from_name(dict_name)
        except ValueError:
            continue
        for img in variants:
            for det_method in methods:
                if det_method == 'opencv':
                    corners_list, ids = _detect_with_opencv(img, dict_name)
                else:
                    corners_list, ids = _detect_with_legacy(img, dict_name)
                if ids is not None and corners_list is not None and len(corners_list) > 0:
                    break
            if ids is not None and corners_list is not None and len(corners_list) > 0:
                break
        if ids is not None and corners_list is not None and len(corners_list) > 0:
            break

    if ids is None or corners_list is None or len(corners_list) == 0:
        return None

    ids_flat = ids.flatten().tolist()
    target_index = None
    if fiducial_id < 0:
        areas = [
            float(cv2.contourArea(np.asarray(c, dtype=np.float32).reshape((4, 2))))
            for c in corners_list
        ]
        target_index = int(np.argmax(areas)) if areas else None
    else:
        for idx, found_id in enumerate(ids_flat):
            if int(found_id) == int(fiducial_id):
                target_index = idx
                break

    if target_index is None:
        return None

    corners = np.asarray(corners_list[target_index], dtype=np.float32).reshape((4, 2))
    if not np.isfinite(corners).all():
        return None

    if min_area_px > 0.0:
        area = float(cv2.contourArea(corners.astype(np.float32)))
        if area < float(min_area_px):
            return None

    return corners


def quad_area(quad: np.ndarray) -> float:
    """Compute the area of a quadrilateral."""
    quad_arr = np.asarray(quad, dtype=np.float32).reshape((4, 2))
    return float(cv2.contourArea(quad_arr))


def roi_to_quad(roi: Sequence[float]) -> np.ndarray:
    """Convert ROI [x, y, w, h] to a quad (tl, tr, br, bl)."""
    if len(roi) != 4:
        raise ValueError('ROI must be [x, y, w, h]')
    x, y, w, h = [float(v) for v in roi]
    return np.array(
        [
            [x, y],
            [x + w, y],
            [x + w, y + h],
            [x, y + h],
        ],
        dtype=np.float32,
    )


def compute_panel_quad_from_fiducial(
    fiducial_corners: np.ndarray,
    *,
    side: str,
    panel_scale_w: float,
    panel_scale_h: float,
    panel_gap_scale: float,
) -> np.ndarray:
    """Compute an adjacent panel quad from fiducial corners."""
    corners = np.asarray(fiducial_corners, dtype=np.float32).reshape((4, 2))
    tl, tr, br, bl = corners

    width_vec = tr - tl
    height_vec = bl - tl
    width = float(np.linalg.norm(width_vec))
    height = float(np.linalg.norm(height_vec))

    if width <= 0.0 or height <= 0.0:
        raise ValueError('Fiducial corners have zero area')

    u = width_vec / width
    v = height_vec / height

    panel_w = float(panel_scale_w) * width
    panel_h = float(panel_scale_h) * height
    gap = float(panel_gap_scale) * width

    side_norm = side.strip().lower()
    if side_norm == 'right':
        panel_tl = tl + (width + gap) * u
    elif side_norm == 'left':
        panel_tl = tl - (panel_w + gap) * u
    elif side_norm == 'bottom':
        panel_tl = tl + (height + gap) * v
    elif side_norm == 'top':
        panel_tl = tl - (panel_h + gap) * v
    else:
        raise ValueError(f'Unsupported panel_side: {side}')

    panel_tr = panel_tl + panel_w * u
    panel_bl = panel_tl + panel_h * v
    panel_br = panel_bl + panel_w * u

    return np.stack([panel_tl, panel_tr, panel_br, panel_bl])


def inset_quad(quad: np.ndarray, inset_ratio: float) -> np.ndarray:
    """Shrink a quad inward by inset_ratio on each side."""
    quad_arr = np.asarray(quad, dtype=np.float32).reshape((4, 2))
    ratio = float(np.clip(inset_ratio, 0.0, 0.49))
    if ratio <= 0.0:
        return quad_arr

    tl, tr, br, bl = quad_arr
    w_vec = tr - tl
    h_vec = bl - tl
    inset_w = w_vec * ratio
    inset_h = h_vec * ratio

    new_tl = tl + inset_w + inset_h
    new_tr = tr - inset_w + inset_h
    new_br = br - inset_w - inset_h
    new_bl = bl + inset_w - inset_h

    return np.stack([new_tl, new_tr, new_br, new_bl])


def split_panel_quad(
    panel_quad: np.ndarray,
    *,
    grid: Sequence[int] = (2, 2),
    inset_ratio: float = 0.0,
) -> list[np.ndarray]:
    """Split a panel quad into an (rows, cols) grid of patch quads."""
    panel = np.asarray(panel_quad, dtype=np.float32).reshape((4, 2))
    rows = int(grid[0]) if len(grid) > 0 else 2
    cols = int(grid[1]) if len(grid) > 1 else 2
    rows = max(1, rows)
    cols = max(1, cols)

    tl, tr, br, bl = panel
    width_vec = tr - tl
    height_vec = bl - tl

    patch_w = width_vec / float(cols)
    patch_h = height_vec / float(rows)

    patches: list[np.ndarray] = []
    for row in range(rows):
        for col in range(cols):
            patch_tl = tl + patch_w * col + patch_h * row
            patch_tr = patch_tl + patch_w
            patch_bl = patch_tl + patch_h
            patch_br = patch_bl + patch_w
            patch = np.stack([patch_tl, patch_tr, patch_br, patch_bl])
            patches.append(inset_quad(patch, inset_ratio))

    return patches
