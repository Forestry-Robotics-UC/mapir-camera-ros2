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
"""ROS-agnostic core utilities for MAPIR Survey3 workflows."""

from .colormap import (
    colorize_scalar_field,
    parse_custom_colormap,
    supported_colormaps,
)
from .raw_survey3 import (
    apply_mapir_color_correction,
    COLOR_CORRECTION_VECTORS,
    decode_survey3_raw12,
    demosaic_survey3_raw,
    load_survey3_raw,
    survey3_raw_to_rgb,
)
from .reflectance import (
    apply_linear_reflectance,
    apply_gamma_reflectance,
    fit_linear_reflectance,
    fit_gamma_reflectance,
    normalize_patch_reflectances,
    ReflectanceFitError,
    sample_patch_means,
    sample_patch_stats,
)
from .spectral_indices import (
    compute_spectral_indices,
    extract_bands_from_bgr,
    MissingBandError,
    preset_band_channels,
    SpectralIndexParams,
    supported_spectral_indices,
)
from .target_detection import (
    aruco_dictionary_from_name,
    compute_panel_quad_from_fiducial,
    detect_aruco_fiducial,
    inset_quad,
    quad_area,
    roi_to_quad,
    split_panel_quad,
)
from .v4l2_camera import (
    configure_v4l2_capture,
    fourcc_to_str,
    open_v4l2_capture,
    V4L2Negotiation,
)
from .vignette_correction import (
    apply_vignette_correction,
    DEFAULT_DARK_CURRENT_JPG,
    DEFAULT_DARK_CURRENT_TIFF,
    load_vignette_images,
)

__all__ = [
    'colorize_scalar_field',
    'parse_custom_colormap',
    'supported_colormaps',
    'MissingBandError',
    'SpectralIndexParams',
    'compute_spectral_indices',
    'extract_bands_from_bgr',
    'preset_band_channels',
    'supported_spectral_indices',
    'COLOR_CORRECTION_VECTORS',
    'apply_mapir_color_correction',
    'decode_survey3_raw12',
    'demosaic_survey3_raw',
    'load_survey3_raw',
    'survey3_raw_to_rgb',
    'ReflectanceFitError',
    'apply_linear_reflectance',
    'apply_gamma_reflectance',
    'fit_linear_reflectance',
    'fit_gamma_reflectance',
    'normalize_patch_reflectances',
    'sample_patch_means',
    'sample_patch_stats',
    'aruco_dictionary_from_name',
    'compute_panel_quad_from_fiducial',
    'detect_aruco_fiducial',
    'inset_quad',
    'quad_area',
    'roi_to_quad',
    'split_panel_quad',
    'DEFAULT_DARK_CURRENT_JPG',
    'DEFAULT_DARK_CURRENT_TIFF',
    'apply_vignette_correction',
    'load_vignette_images',
    'V4L2Negotiation',
    'configure_v4l2_capture',
    'fourcc_to_str',
    'open_v4l2_capture',
]
