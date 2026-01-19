# Parameters

Parameter defaults come from:

- code defaults (declared in the nodes), and
- package YAML presets in `config/` (recommended for repeatability).

Tables below reflect the packaged preset defaults. If a parameter is not set in
the YAML, the node's code default applies.

## Camera node (`camera_node`)

| Param | Type | Default | Description |
|---|---|---|---|
| `debug` | `bool` | `false` | Enable extra logs. |
| `debug_period_s` | `float` | `1.0` | Throttle for periodic debug logs. |
| `video_device` | `str` | `'/dev/video0'` | V4L2 device path or numeric index. |
| `image_width` | `int` | `1920` | Requested width. |
| `image_height` | `int` | `1440` | Requested height. |
| `framerate` | `float` | `30.0` | Requested FPS. |
| `pixel_format` | `str` | `'MJPG'` | `'MJPG'` or `'H264'` (device-dependent). |
| `use_gstreamer` | `bool` | `true` | Use a GStreamer pipeline for capture. |
| `gstreamer_pipeline` | `str` | `''` | Custom GStreamer pipeline string. |
| `reconnect_interval_s` | `float` | `2.0` | Retry camera open when disconnected. |
| `use_capture_thread` | `bool` | `true` | C++ node only; dedicated capture thread. |
| `frame_id` | `str` | `'mapir3_optical_frame'` | Output frame_id for Image/CameraInfo. |
| `camera_name` | `str` | `'mapir3_ocn'` | Camera name for calibration lookup. |
| `camera_info_url` | `str` | `''` | Calibration URL (`file:///.../calib.yaml`). |
| `qos_best_effort` | `bool` | `true` | BEST_EFFORT vs RELIABLE. |
| `qos_depth` | `int` | `5` | Publisher queue depth. |

Preset file:

- `config/mapir_camera_params.yaml`

## Indices node (`indices_node`)

| Param | Type | Default | Description |
|---|---|---|---|
| `debug` | `bool` | `false` | Enable extra logs. |
| `debug_period_s` | `float` | `1.0` | Throttle for periodic debug logs. |
| `enabled` | `bool` | `true` | Runtime toggle (skip compute/publish when false). |
| `image_topic` | `str` | `'image_raw'` | Input image topic (relative to namespace). |
| `indices` | `list[str]` | `['ndvi', 'osavi']` | Index names; add `_1`/`_2` to prefer NIR1/NIR2. |
| `filter_set` | `str` | `'OCN'` | Band preset: `RGN`, `NGB`, `OCN`, `RGB`. |
| `normalize_input` | `bool` | `true` | Normalize integer images to `[0,1]`. |
| `downsample_factor` | `int` | `1` | Stride downsample for CPU savings. |
| `publish_every_n` | `int` | `1` | Publish every Nth frame (drop others). |
| `publish_color` | `bool` | `false` | Publish a colorized BGR image per index. |
| `colormap` | `str` | `'viridis'` | Colormap name (`viridis`, `jet`, `gray`, `custom`, ...). |
| `colorize_min` | `float` | `-1.0` | Value mapped to the low end of the colormap. |
| `colorize_max` | `float` | `1.0` | Value mapped to the high end of the colormap. |
| `custom_colormap` | `str` | `''` | Custom points: `value,r,g,b; value,r,g,b; ...` (when `colormap=custom`). |
| `eps` | `float` | `1e-6` | Safe-divide epsilon. |
| `gari_gamma` | `float` | `1.7` | GARI gamma constant. |
| `wdrvi_alpha` | `float` | `0.2` | WDRVI alpha constant. |
| `mnli_L` | `float` | `0.5` | MNLI background adjustment factor. |
| `*_channel` | `int` | `-1` | Override mapping (0=B,1=G,2=R); `-1` leaves unset. |
| `qos_best_effort` | `bool` | `true` | Match camera stream QoS. |
| `qos_depth` | `int` | `5` | Subscription/publisher depth. |

Performance note: for real-time use, keep the indices list to 1-2 entries.

Preset files:

- `config/mapir_indices_params.yaml`

## Reflectance node (`reflectance_node`)

| Param | Type | Default | Description |
|---|---|---|---|
| `enable` | `bool` | `true` | Runtime toggle. |
| `input_topic` | `str` | `'image_raw'` | Input image topic. |
| `output_topic` | `str` | `'image_rect'` | Output reflectance topic (32FC3). |
| `preview_topic` | `str` | `'image_reflectance_preview'` | 8-bit preview topic (bgr8/mono8). |
| `debug_topic` | `str` | `'reflectance/debug'` | Debug overlay topic. |
| `status_topic` | `str` | `'reflectance/status'` | Status text topic. |
| `camera_info_topic` | `str` | `'camera_info'` | CameraInfo topic used for rectification. |
| `rectify_output` | `bool` | `false` | Undistort reflectance output using CameraInfo. |
| `calibration_mode` | `str` | `'once'` | `'once'` or `'continuous'`. |
| `recalibration_interval_s` | `float` | `5.0` | Only for continuous mode. |
| `detect_rate_hz` | `float` | `5.0` | Fiducial detection rate. |
| `detect_downscale` | `float` | `0.5` | Downscale factor for detection. |
| `min_target_area_px` | `int` | `5000` | Minimum fiducial/panel area. |
| `model` | `str` | `'linear'` | Reflectance model: `linear`, `linear_gray`, `gamma`, `gamma_gray`. |
| `clamp_min` | `float` | `0.0` | Minimum reflectance clamp. |
| `clamp_max` | `float` | `1.2` | Maximum reflectance clamp. |
| `min_gamma` | `float` | `0.2` | Minimum gamma for gamma models. |
| `max_gamma` | `float` | `5.0` | Maximum gamma for gamma models. |
| `max_gain` | `float` | `10.0` | Maximum gain for gamma models. |
| `ema_alpha` | `float` | `0.2` | EMA smoothing factor for calibration params. |
| `detection_mode` | `str` | `'fiducial'` | `'fiducial'` or `'manual'`. |
| `fiducial_type` | `str` | `'aruco'` | Fiducial detector (ArUco). |
| `aruco_detector` | `str` | `'legacy'` | ArUco method: `legacy` (`detectMarkers`), `opencv` (`ArucoDetector`), or `auto`. |
| `aruco_dictionary` | `str` | `'DICT_4X4_50'` | ArUco dictionary (only used for `fiducial_type: aruco`). Use `auto` to try common dictionaries. |
| `fiducial_id` | `int` | `0` | Marker ID (`-1` selects the largest detected marker). |
| `panel_side` | `str` | `'right'` | Panel side relative to fiducial (`left`, `right`, `top`, `bottom`, `auto`). |
| `panel_scale_w` | `float` | `1.0` | Panel width in fiducial widths. |
| `panel_scale_h` | `float` | `1.0` | Panel height in fiducial heights. |
| `panel_gap_scale` | `float` | `0.10` | Gap from fiducial. |
| `min_panel_in_bounds` | `int` | `4` | Minimum panel quad vertices inside image bounds. |
| `patch_grid` | `list[int]` | `[2, 2]` | Patch grid. |
| `patch_inset_ratio` | `float` | `0.20` | Inset for patch sampling. |
| `patch_stat` | `str` | `'median'` | Patch statistic: `mean`, `median`, or `trimmed_mean`. |
| `patch_warp_size` | `int` | `32` | Warp size for per-patch perspective normalization. |
| `patch_trim_ratio` | `float` | `0.1` | Trim ratio for `trimmed_mean` (0-0.49). |
| `patch_reflectances` | `list` | `[0.10, 0.20, 0.50, 0.90]` | Per-patch reflectance (scalar or [O,C,N]). |
| `initial_slopes` | `list` | `[]` | Optional initial per-channel slopes. |
| `initial_intercepts` | `list` | `[]` | Optional initial per-channel intercepts. |
| `publish_debug_overlay` | `bool` | `true` | Publish debug overlay. |
| `fiducial_roi` | `list` | `[]` | Manual fiducial ROI [x, y, w, h]. |
| `panel_roi` | `list` | `[]` | Manual panel ROI [x, y, w, h]. |
| `patch_rois` | `list` | `[]` | Manual patch ROIs (list of [x, y, w, h]). |
| `qos_best_effort` | `bool` | `true` | Match camera stream QoS. |
| `qos_depth` | `int` | `5` | Subscription/publisher depth. |

Notes:

- `fiducial_id` and `aruco_dictionary` apply to ArUco markers.

Preset files:

- `config/reflectance_ocn.yaml`
