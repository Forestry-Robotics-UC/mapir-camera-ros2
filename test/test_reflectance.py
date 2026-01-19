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
    apply_gamma_reflectance,
    apply_linear_reflectance,
    fit_gamma_reflectance,
    fit_linear_reflectance,
    normalize_patch_reflectances,
    ReflectanceFitError,
    sample_patch_means,
    sample_patch_stats,
)


def test_normalize_patch_reflectances_scalar_and_vector():
    out = normalize_patch_reflectances([0.1, [0.2, 0.3, 0.4]], num_patches=2)
    assert out.shape == (2, 3)
    assert np.allclose(out[0], [0.1, 0.1, 0.1])
    assert np.allclose(out[1], [0.2, 0.3, 0.4])


def test_normalize_patch_reflectances_invalid_count():
    with pytest.raises(ValueError):
        normalize_patch_reflectances([0.1], num_patches=2)


def test_fit_linear_reflectance_recovers_parameters():
    slopes = np.array([0.01, 0.02, 0.03], dtype=np.float32)
    intercepts = np.array([0.1, 0.0, 0.2], dtype=np.float32)

    dn_vals = np.array([10.0, 20.0, 30.0, 40.0], dtype=np.float32)
    patch_means = np.stack(
        [dn_vals, dn_vals + 1.0, dn_vals + 2.0],
        axis=1,
    )
    patch_refs = patch_means * slopes + intercepts

    fit_slopes, fit_intercepts = fit_linear_reflectance(patch_means, patch_refs)
    assert np.allclose(fit_slopes, slopes, atol=1e-4)
    assert np.allclose(fit_intercepts, intercepts, atol=1e-4)


def test_fit_linear_reflectance_rejects_invalid_shape():
    patch_means = np.zeros((2, 3), dtype=np.float32)
    patch_refs = np.zeros((2, 2), dtype=np.float32)
    with pytest.raises(ReflectanceFitError):
        fit_linear_reflectance(patch_means, patch_refs)


def test_sample_patch_means_matches_expected_values():
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[0:10, 0:10, :] = [10, 20, 30]
    image[0:10, 10:20, :] = [40, 50, 60]

    patch0 = np.array([[0, 0], [9, 0], [9, 9], [0, 9]], dtype=np.float32)
    patch1 = np.array([[10, 0], [19, 0], [19, 9], [10, 9]], dtype=np.float32)

    means = sample_patch_means(image, [patch0, patch1])
    assert np.allclose(means[0], [10, 20, 30])
    assert np.allclose(means[1], [40, 50, 60])


def test_sample_patch_means_invalid_patch_shape():
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        sample_patch_means(image, [np.zeros((3, 2), dtype=np.float32)])


def test_sample_patch_means_invalid_image_dim():
    image = np.zeros((2, 2, 2, 2), dtype=np.uint8)
    with pytest.raises(ValueError):
        sample_patch_means(image, [])


def test_sample_patch_means_min_area_filters_patch():
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    patch = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    means = sample_patch_means(image, [patch], min_area_px=10)
    assert np.isnan(means[0]).all()


def test_sample_patch_stats_median_matches_uniform_patch():
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[0:10, 0:10, :] = [10, 20, 30]
    patch = np.array([[0, 0], [9, 0], [9, 9], [0, 9]], dtype=np.float32)
    stats = sample_patch_stats(image, [patch], method='median', warp_size=8)
    assert np.allclose(stats[0], [10, 20, 30])


def test_sample_patch_stats_trimmed_mean_rejects_outlier():
    image = np.full((4, 4), 10.0, dtype=np.float32)
    image[0, 0] = 100.0
    patch = np.array([[0, 0], [3, 0], [3, 3], [0, 3]], dtype=np.float32)
    stats = sample_patch_stats(
        image,
        [patch],
        method='trimmed_mean',
        warp_size=4,
        trim_ratio=0.1,
    )
    assert stats[0, 0] == pytest.approx(10.0, abs=1.0)


def test_apply_linear_reflectance_clamps_output():
    img = np.array([[[-1.0, 0.5, 2.0]]], dtype=np.float32)
    slopes = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    intercepts = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    out = apply_linear_reflectance(img, slopes, intercepts, clamp_min=0.0, clamp_max=1.0)
    assert np.all(out >= 0.0)
    assert np.all(out <= 1.0)


def test_apply_linear_reflectance_grayscale():
    img = np.array([[0.2, 0.4]], dtype=np.float32)
    out = apply_linear_reflectance(
        img,
        np.array([2.0], dtype=np.float32),
        np.array([0.1], dtype=np.float32),
        clamp_min=0.0,
        clamp_max=1.0,
    )
    assert out.shape == img.shape


def test_fit_linear_reflectance_not_enough_valid_patches():
    patch_means = np.array([[10.0], [np.nan]], dtype=np.float32)
    patch_refs = np.array([[0.2], [0.3]], dtype=np.float32)
    with pytest.raises(ReflectanceFitError):
        fit_linear_reflectance(patch_means, patch_refs)


def test_fit_linear_reflectance_dn_range_too_small():
    patch_means = np.array([[10.0], [11.0]], dtype=np.float32)
    patch_refs = np.array([[0.2], [0.4]], dtype=np.float32)
    with pytest.raises(ReflectanceFitError):
        fit_linear_reflectance(patch_means, patch_refs, min_dn_range=5.0)


def test_fit_linear_reflectance_negative_slope():
    patch_means = np.array([[0.0], [10.0]], dtype=np.float32)
    patch_refs = np.array([[1.0], [0.0]], dtype=np.float32)
    with pytest.raises(ReflectanceFitError):
        fit_linear_reflectance(patch_means, patch_refs)


def test_fit_linear_reflectance_slope_exceeds_limit():
    patch_means = np.array([[0.0], [1.0]], dtype=np.float32)
    patch_refs = np.array([[0.0], [10.0]], dtype=np.float32)
    with pytest.raises(ReflectanceFitError):
        fit_linear_reflectance(patch_means, patch_refs, max_abs_slope=2.0)


def test_apply_linear_reflectance_invalid_image_dim():
    img = np.zeros((1, 1, 1, 1), dtype=np.float32)
    with pytest.raises(ValueError):
        apply_linear_reflectance(img, np.array([1.0]), np.array([0.0]))


def test_fit_gamma_reflectance_recovers_parameters():
    gains = np.array([0.8, 1.1, 0.9], dtype=np.float32)
    gammas = np.array([1.2, 0.9, 1.1], dtype=np.float32)

    dn_vals = np.array([0.1, 0.3, 0.6, 0.9], dtype=np.float32)
    patch_dn = np.stack([dn_vals, dn_vals + 0.05, dn_vals + 0.1], axis=1)
    patch_refs = gains * np.power(patch_dn, gammas)

    fit_gains, fit_gammas = fit_gamma_reflectance(patch_dn, patch_refs)
    assert np.allclose(fit_gains, gains, atol=1e-2)
    assert np.allclose(fit_gammas, gammas, atol=1e-2)


def test_apply_gamma_reflectance_clamps_output():
    img = np.array([[0.1, 0.5]], dtype=np.float32)
    out = apply_gamma_reflectance(
        img,
        np.array([2.0], dtype=np.float32),
        np.array([1.0], dtype=np.float32),
        clamp_min=0.0,
        clamp_max=1.0,
    )
    assert np.all(out >= 0.0)
    assert np.all(out <= 1.0)
