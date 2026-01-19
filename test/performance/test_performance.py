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
import os
import time

import numpy as np
import pytest

from mapir_camera_ros2.core import (
    apply_linear_reflectance,
    apply_vignette_correction,
    compute_spectral_indices,
)


def _perf_limit_ms() -> float:
    return float(os.environ.get('MAPIR_PERF_MAX_MS_CORE', '200'))


def _run_and_time(func) -> float:
    start = time.perf_counter()
    func()
    end = time.perf_counter()
    return (end - start) * 1000.0


@pytest.mark.performance
def test_compute_spectral_indices_perf():
    h, w = 256, 256
    bands = {
        'red': np.full((h, w), 0.2, dtype=np.float32),
        'nir': np.full((h, w), 0.6, dtype=np.float32),
        'green': np.full((h, w), 0.3, dtype=np.float32),
        'blue': np.full((h, w), 0.1, dtype=np.float32),
    }

    def _work():
        compute_spectral_indices(['ndvi', 'gndvi', 'gli'], bands)

    elapsed_ms = _run_and_time(_work)
    assert elapsed_ms <= _perf_limit_ms()


@pytest.mark.performance
def test_apply_vignette_correction_perf():
    h, w = 256, 256
    image = np.full((h, w, 3), 128, dtype=np.uint8)
    flat = [np.ones((h, w), dtype=np.uint8)] * 3

    def _work():
        apply_vignette_correction(image, flat)

    elapsed_ms = _run_and_time(_work)
    assert elapsed_ms <= _perf_limit_ms()


@pytest.mark.performance
def test_apply_linear_reflectance_perf():
    h, w = 256, 256
    image = np.random.uniform(0.0, 1.0, size=(h, w, 3)).astype(np.float32)
    slopes = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    intercepts = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    def _work():
        apply_linear_reflectance(image, slopes, intercepts)

    elapsed_ms = _run_and_time(_work)
    assert elapsed_ms <= _perf_limit_ms()
