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
import numpy as np
import pytest

from mapir_camera_ros2.core import (
    apply_mapir_color_correction,
    decode_survey3_raw12,
    demosaic_survey3_raw,
    load_survey3_raw,
)


def test_apply_mapir_color_correction_clamps():
    img = np.array([[[2.0, -1.0, 0.5]]], dtype=np.float32)
    corrected = apply_mapir_color_correction(img)
    assert corrected.shape == img.shape
    assert np.all(corrected >= 0.0)
    assert np.all(corrected <= 1.0)


def test_apply_mapir_color_correction_invalid_shape():
    with pytest.raises(ValueError):
        apply_mapir_color_correction(np.zeros((2, 2), dtype=np.float32))


def test_apply_mapir_color_correction_invalid_vectors():
    with pytest.raises(ValueError):
        apply_mapir_color_correction(
            np.zeros((1, 1, 3), dtype=np.float32),
            vectors=[1.0, 2.0],
        )


def test_decode_survey3_raw12_zero_bytes():
    raw_bytes = bytes([0] * 6)
    out = decode_survey3_raw12(raw_bytes, width=2, height=2)
    assert out.shape == (2, 2)
    assert out.dtype == np.uint16
    assert np.count_nonzero(out) == 0


def test_decode_survey3_raw12_too_small():
    with pytest.raises(ValueError):
        decode_survey3_raw12(b'', width=2, height=2)


def test_load_survey3_raw_round_trip(tmp_path):
    raw_path = tmp_path / 'test.RAW'
    raw_path.write_bytes(bytes([0] * 6))
    out = load_survey3_raw(str(raw_path), width=2, height=2)
    assert out.shape == (2, 2)


def test_demosaic_survey3_raw_invalid_dim():
    with pytest.raises(ValueError):
        demosaic_survey3_raw(np.zeros((2, 2, 2), dtype=np.uint16))


def test_demosaic_survey3_raw_invert_output():
    bayer = np.zeros((2, 2), dtype=np.uint16)
    out = demosaic_survey3_raw(bayer, invert_output=True)
    assert out.shape == (2, 2, 3)
    assert np.all(out == 65535)


def test_demosaic_survey3_raw_white_balance_path():
    bayer = np.zeros((2, 2), dtype=np.uint16)
    out = demosaic_survey3_raw(bayer, apply_white_balance=True, rgb_sensor=True)
    assert out.shape == (2, 2, 3)
