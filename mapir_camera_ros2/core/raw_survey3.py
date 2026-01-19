#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
#   Decode MAPIR Survey3 RAW (12-bit) data to a 16-bit Bayer image, then
#   demosaic to RGB and optionally apply MAPIR color correction.
#

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np

COLOR_CORRECTION_VECTORS = (
    1.398822546,
    -0.09047482163,
    0.1619316638,
    -0.01290435996,
    0.8994362354,
    0.1134681329,
    0.007306902204,
    -0.05995989591,
    1.577814579,
)


def apply_mapir_color_correction(
    color_rgb: np.ndarray,
    *,
    vectors: Sequence[float] | None = None,
    offsets: Iterable[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """
    Apply MAPIR color correction to an RGB image in [0,1] float space.

    This follows the original camera-scripts ordering and clamps to [0,1].
    """
    if color_rgb.ndim != 3 or color_rgb.shape[2] != 3:
        raise ValueError('color_rgb must be an RGB image with 3 channels')

    coeffs = list(vectors or COLOR_CORRECTION_VECTORS)
    if len(coeffs) != 9:
        raise ValueError('vectors must contain 9 coefficients')

    roff, goff, boff = list(offsets)
    out = color_rgb.astype(np.float32, copy=True)

    red_coeffs = coeffs[6:9]
    green_coeffs = coeffs[3:6]
    blue_coeffs = coeffs[0:3]

    out[:, :, 2] = (
        red_coeffs[0] * out[:, :, 0]
        + red_coeffs[1] * out[:, :, 1]
        + red_coeffs[2] * out[:, :, 2]
        + roff
    )
    out[:, :, 1] = (
        green_coeffs[0] * out[:, :, 0]
        + green_coeffs[1] * out[:, :, 1]
        + green_coeffs[2] * out[:, :, 2]
        + goff
    )
    out[:, :, 0] = (
        blue_coeffs[0] * out[:, :, 0]
        + blue_coeffs[1] * out[:, :, 1]
        + blue_coeffs[2] * out[:, :, 2]
        + boff
    )

    out[out > 1.0] = 1.0
    out[out < 0.0] = 0.0
    return out


def decode_survey3_raw12(raw_bytes: bytes, *, width: int = 4000, height: int = 3000) -> np.ndarray:
    """
    Decode MAPIR Survey3 RAW (12-bit) data into a 16-bit Bayer image.

    Returns a uint16 array of shape (height, width).
    """
    pixel_count = width * height
    expected_bits = pixel_count * 12

    data = np.frombuffer(raw_bytes, dtype=np.uint8)
    bits = np.unpackbits(data)

    if bits.size < expected_bits:
        raise ValueError(
            f'RAW data too small: got {bits.size} bits, expected {expected_bits}'
        )
    if bits.size > expected_bits:
        bits = bits[:expected_bits]

    if expected_bits % 4 != 0:
        raise ValueError('Expected bit count is not divisible by 4')

    bits = bits.reshape((-1, 4))
    if bits.shape[0] != pixel_count * 3:
        raise ValueError(
            f'Unexpected RAW layout: got {bits.shape[0]} nibbles, expected {pixel_count * 3}'
        )

    even_rows = bits[0::2].copy()
    bits[0::2] = bits[1::2]
    bits[1::2] = even_rows

    zero_pad = np.zeros((pixel_count, 4), dtype=np.uint8)
    packed = np.concatenate(
        [bits[0::3], zero_pad, bits[2::3], bits[1::3]],
        axis=1,
    )
    packed = packed.reshape((pixel_count * 16, 1))
    raw_u16 = np.packbits(packed).tobytes()
    image = np.frombuffer(raw_u16, dtype=np.uint16, count=pixel_count)
    return image.reshape((height, width))


def load_survey3_raw(path: str | Path, *, width: int = 4000, height: int = 3000) -> np.ndarray:
    """Load a RAW file from disk and decode it into a Bayer image."""
    raw_bytes = Path(path).read_bytes()
    return decode_survey3_raw12(raw_bytes, width=width, height=height)


def demosaic_survey3_raw(
    bayer_u16: np.ndarray,
    *,
    apply_white_balance: bool = False,
    rgb_sensor: bool = False,
    invert_output: bool | None = None,
) -> np.ndarray:
    """
    Demosaic a Survey3 Bayer image to uint16 RGB.

    If apply_white_balance is True, rgb_sensor must also be True to apply
    MAPIR color correction. The default invert_output behavior mirrors the
    original camera-scripts logic.
    """
    if bayer_u16.ndim != 2:
        raise ValueError('bayer_u16 must be a 2D Bayer image')

    color = cv2.cvtColor(bayer_u16, cv2.COLOR_BAYER_RG2RGB).astype(np.float32)

    if apply_white_balance and rgb_sensor:
        color = color / 65535.0
        color = apply_mapir_color_correction(color)
        color = color * 65535.0
    else:
        color = color * 65535.0

    color = np.clip(color, 0.0, 65535.0).astype(np.uint16)

    if invert_output is None:
        invert_output = not (apply_white_balance and rgb_sensor)
    if invert_output:
        color = cv2.bitwise_not(color)

    return color


def survey3_raw_to_rgb(
    path: str | Path,
    *,
    width: int = 4000,
    height: int = 3000,
    apply_white_balance: bool = False,
    rgb_sensor: bool = False,
    invert_output: bool | None = None,
) -> np.ndarray:
    """Load a Survey3 RAW file and return a demosaiced RGB uint16 image."""
    bayer = load_survey3_raw(path, width=width, height=height)
    return demosaic_survey3_raw(
        bayer,
        apply_white_balance=apply_white_balance,
        rgb_sensor=rgb_sensor,
        invert_output=invert_output,
    )
