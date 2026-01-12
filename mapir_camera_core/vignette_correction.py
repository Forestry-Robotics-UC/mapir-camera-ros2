#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: GNU General Public License v3.0 (GPL-3.0-only)
# Repository: mapir_survey3
#
# Derived from:
#   MAPIR camera-scripts (GPLv3)
#   https://github.com/mapircamera/camera-scripts
#   Original author: MAPIR, Inc.
#
# Description:
#   Apply MAPIR Survey3 vignette (flat-field) correction to 3-channel images.
#

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np

DEFAULT_DARK_CURRENT_TIFF = (120, 119, 119)
DEFAULT_DARK_CURRENT_JPG = (0, 0, 0)


def load_vignette_images(paths: Sequence[str | Path]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load per-channel flat-field (vignette) images in B, G, R order."""
    if len(paths) != 3:
        raise ValueError('paths must contain exactly 3 vignette images (B, G, R)')

    images: list[np.ndarray] = []
    for path in paths:
        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError(f'Failed to read vignette image: {path}')
        images.append(img)

    return images[0], images[1], images[2]


def apply_vignette_correction(
    image_bgr: np.ndarray,
    flat_fields_bgr: Sequence[np.ndarray],
    *,
    dark_current: Iterable[int] | None = None,
) -> np.ndarray:
    """
    Apply per-channel flat-field (vignette) correction to a BGR image.

    The output dtype matches the input dtype (uint16 or uint8).
    """
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('image_bgr must be a 3-channel image')
    if len(flat_fields_bgr) != 3:
        raise ValueError('flat_fields_bgr must contain 3 channels (B, G, R)')

    dtype = image_bgr.dtype
    max_value = 65535.0 if dtype == np.uint16 else 255.0

    if dark_current is None:
        dark_current = (
            DEFAULT_DARK_CURRENT_TIFF if dtype == np.uint16 else DEFAULT_DARK_CURRENT_JPG
        )

    dc_b, dc_g, dc_r = list(dark_current)
    vig_b, vig_g, vig_r = flat_fields_bgr

    for vig in (vig_b, vig_g, vig_r):
        if vig.shape != image_bgr.shape[:2]:
            raise ValueError('vignette images must match the input image shape')

    b, g, r = cv2.split(image_bgr)
    b = b.astype(np.float32) - float(dc_b)
    g = g.astype(np.float32) - float(dc_g)
    r = r.astype(np.float32) - float(dc_r)

    b = np.divide(b, vig_b.astype(np.float32))
    g = np.divide(g, vig_g.astype(np.float32))
    r = np.divide(r, vig_r.astype(np.float32))

    b = np.clip(b, 0.0, max_value)
    g = np.clip(g, 0.0, max_value)
    r = np.clip(r, 0.0, max_value)

    corrected = cv2.merge((b, g, r))
    return corrected.astype(dtype)
