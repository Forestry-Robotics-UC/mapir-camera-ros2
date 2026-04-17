"""ROS-agnostic core utilities for MAPIR Survey3 workflows."""

from .colormap import (
    colorize_scalar_field,
    parse_custom_colormap,
    supported_colormaps,
)
from .spectral_indices import (
    compute_spectral_indices,
    extract_bands_from_bgr,
    MissingBandError,
    preset_band_channels,
    SpectralIndexParams,
    supported_spectral_indices,
)
from .raw_survey3 import (
    COLOR_CORRECTION_VECTORS,
    apply_mapir_color_correction,
    decode_survey3_raw12,
    demosaic_survey3_raw,
    load_survey3_raw,
    survey3_raw_to_rgb,
)
from .vignette_correction import (
    DEFAULT_DARK_CURRENT_JPG,
    DEFAULT_DARK_CURRENT_TIFF,
    apply_vignette_correction,
    load_vignette_images,
)
from .v4l2_camera import (
    configure_v4l2_capture,
    fourcc_to_str,
    open_v4l2_capture,
    V4L2Negotiation,
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
    'DEFAULT_DARK_CURRENT_JPG',
    'DEFAULT_DARK_CURRENT_TIFF',
    'apply_vignette_correction',
    'load_vignette_images',
    'V4L2Negotiation',
    'configure_v4l2_capture',
    'fourcc_to_str',
    'open_v4l2_capture',
]
