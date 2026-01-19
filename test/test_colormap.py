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
    colorize_scalar_field,
    parse_custom_colormap,
    supported_colormaps,
)


def test_supported_colormaps_includes_core_entries():
    supported = supported_colormaps()
    assert 'gray' in supported
    assert 'custom' in supported


def test_parse_custom_colormap_parses_bgr_and_sorts():
    values, colors = parse_custom_colormap('1,255,0,0; -1,0,0,255')
    assert values.dtype == np.float32
    assert colors.dtype == np.uint8
    assert np.allclose(values, [-1.0, 1.0])
    assert np.array_equal(colors[0], [255, 0, 0])
    assert np.array_equal(colors[1], [0, 0, 255])


def test_parse_custom_colormap_rejects_duplicates():
    with pytest.raises(ValueError):
        parse_custom_colormap('0,0,0,0; 0,255,255,255')


def test_parse_custom_colormap_invalid_point_format():
    with pytest.raises(ValueError):
        parse_custom_colormap('0,0,0')


def test_colorize_scalar_field_gray():
    vals = np.array([[0.0, 1.0]], dtype=np.float32)
    img = colorize_scalar_field(vals, vmin=0.0, vmax=1.0, colormap='gray')
    assert img.shape == (1, 2, 3)
    assert np.array_equal(img[0, 0], [0, 0, 0])
    assert np.array_equal(img[0, 1], [255, 255, 255])


def test_colorize_scalar_field_custom_nan_is_black():
    vals = np.array([[0.0, np.nan]], dtype=np.float32)
    img = colorize_scalar_field(
        vals,
        vmin=-1.0,
        vmax=1.0,
        colormap='custom',
        custom_colormap='-1,0,0,0; 1,255,255,255',
    )
    assert np.array_equal(img[0, 1], [0, 0, 0])


def test_colorize_scalar_field_invalid_range():
    vals = np.array([0.0], dtype=np.float32)
    with pytest.raises(ValueError):
        colorize_scalar_field(vals, vmin=1.0, vmax=1.0)


def test_colorize_scalar_field_invalid_colormap():
    vals = np.array([0.0], dtype=np.float32)
    with pytest.raises(ValueError):
        colorize_scalar_field(vals, vmin=0.0, vmax=1.0, colormap='does_not_exist')
