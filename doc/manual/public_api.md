# Public API

This package follows REP-2004 guidance: the public API is explicitly defined
below. Anything not listed here is considered internal and may change without
notice.

## Python API

The stable Python API is exposed through `mapir_camera_ros2.core` and matches
its `__all__` export list:

- `colorize_scalar_field`, `parse_custom_colormap`, `supported_colormaps`
- `COLOR_CORRECTION_VECTORS`, `apply_mapir_color_correction`,
  `decode_survey3_raw12`, `demosaic_survey3_raw`, `load_survey3_raw`,
  `survey3_raw_to_rgb`
- `ReflectanceFitError`, `apply_linear_reflectance`, `fit_linear_reflectance`,
  `apply_gamma_reflectance`, `fit_gamma_reflectance`,
  `normalize_patch_reflectances`, `sample_patch_means`, `sample_patch_stats`
- `MissingBandError`, `SpectralIndexParams`, `compute_spectral_indices`,
  `extract_bands_from_bgr`, `preset_band_channels`, `supported_spectral_indices`
- `aruco_dictionary_from_name`, `detect_aruco_fiducial`,
  `compute_panel_quad_from_fiducial`, `inset_quad`, `quad_area`, `roi_to_quad`,
  `split_panel_quad`
- `V4L2Negotiation`, `configure_v4l2_capture`, `fourcc_to_str`,
  `open_v4l2_capture`
- `DEFAULT_DARK_CURRENT_JPG`, `DEFAULT_DARK_CURRENT_TIFF`,
  `apply_vignette_correction`, `load_vignette_images`

The node modules are also public entry points:

- `mapir_camera_ros2.nodes.camera_node` (executable `camera_node`)
- `mapir_camera_ros2.nodes.indices_node` (executable `indices_node`)
- `mapir_camera_ros2.nodes.reflectance_node` (executable `reflectance_node`)

## C++ API

Installed headers in `include/mapir_camera_ros2/` are public. Currently:

- `include/mapir_camera_ros2/gstreamer_pipeline.hpp`
  - Namespace `mapir_camera_cpp`
  - `GstreamerConfig`, `BuildGstreamerPipeline()`

## ROS Interfaces

The following are part of the stable interface and are documented in detail:

- Topics, services, and frames: `doc/manual/ros_contract.md`
- Parameters: `doc/manual/parameters.md`
- Launch behavior: `launch/mapir_camera.launch.py`

Any CLI flags or parameters listed in the documentation are considered
public API.
