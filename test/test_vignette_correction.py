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
import cv2
import numpy as np
import pytest

from mapir_camera_ros2.core import apply_vignette_correction, load_vignette_images


def test_load_vignette_images_requires_three_paths():
    with pytest.raises(ValueError):
        load_vignette_images(['only_one.tif'])


def test_load_vignette_images_round_trip(tmp_path):
    paths = []
    for idx in range(3):
        path = tmp_path / f'vig_{idx}.png'
        img = np.full((2, 2), 10 + idx, dtype=np.uint8)
        assert cv2.imwrite(str(path), img)
        paths.append(str(path))

    b, g, r = load_vignette_images(paths)
    assert b.shape == (2, 2)
    assert g.shape == (2, 2)
    assert r.shape == (2, 2)


def test_apply_vignette_correction_shape_mismatch():
    img = np.zeros((2, 2, 3), dtype=np.uint8)
    flat = [np.ones((3, 3), dtype=np.uint8)] * 3
    with pytest.raises(ValueError):
        apply_vignette_correction(img, flat)


def test_apply_vignette_correction_identity():
    img = np.array(
        [
            [[10, 20, 30], [40, 50, 60]],
            [[70, 80, 90], [100, 110, 120]],
        ],
        dtype=np.uint8,
    )
    flat = [np.ones((2, 2), dtype=np.uint8)] * 3
    out = apply_vignette_correction(img, flat)
    assert np.array_equal(out, img)


def test_apply_vignette_correction_uint16_defaults():
    img = np.zeros((2, 2, 3), dtype=np.uint16)
    flat = [np.ones((2, 2), dtype=np.uint16)] * 3
    out = apply_vignette_correction(img, flat)
    assert out.dtype == np.uint16
