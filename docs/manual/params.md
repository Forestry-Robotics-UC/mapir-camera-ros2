# Params

This page consolidates launch arguments, node parameters, and config files.

## Priority Order

Effective values follow this order (highest first):

1. Launch/CLI overrides (for example `video_device:=/dev/video0`)
2. YAML params files (`camera_params_file`, `indices_params_file`)
3. Node-declared defaults in code

When `debug:=true`, launch prints the active precedence and explicit override
sources before starting the camera node.

## Config Files

- `config/mapir_camera_params.yaml`: camera node baseline
- `config/mapir_indices_params.yaml`: indices node RGN baseline
- `config/mapir_indices_ocn_params.yaml`: indices node OCN baseline

## Launch Args (`mapir_camera.launch.py`)

| Arg | Default | Notes |
|---|---|---|
| `namespace` | `mapir` | Namespace for camera and indices nodes. |
| `camera_params_file` | package `config/mapir_camera_params.yaml` | Camera YAML source. |
| `camera_impl` | `cpp` | `cpp` or `py`. |
| `video_device` | `''` | Optional camera parameter override. |
| `image_width` | `''` | Optional camera parameter override. |
| `image_height` | `''` | Optional camera parameter override. |
| `framerate` | `''` | Optional camera parameter override. |
| `pixel_format` | `''` | Optional camera parameter override (`MJPG`/`H264`). |
| `use_gstreamer` | `''` | Optional camera parameter override (`true`/`false`). |
| `gstreamer_pipeline` | `''` | Optional camera parameter override. |
| `frame_id` | `''` | Optional camera parameter override. |
| `camera_name` | `''` | Optional camera parameter override. |
| `camera_info_url` | `''` | Optional camera parameter override. |
| `qos_best_effort` | `''` | Optional camera parameter override (`true`/`false`). |
| `qos_depth` | `''` | Optional camera parameter override. |
| `debug` | `false` | Camera debug logs + parameter source logs. |
| `debug_period_s` | `''` | Optional camera parameter override. |
| `publish_static_tf` | `true` | Publish placeholder static optical transform. |
| `static_tf_parent_frame` | `mapir3_link` | Static TF parent. |
| `static_tf_child_frame` | `mapir3_optical_frame` | Static TF child. |
| `static_tf_x` | `0` | Static TF x (m). |
| `static_tf_y` | `0` | Static TF y (m). |
| `static_tf_z` | `0` | Static TF z (m). |
| `static_tf_roll` | `1.5707963` | Static TF roll (rad). |
| `static_tf_pitch` | `0` | Static TF pitch (rad). |
| `static_tf_yaw` | `3.1415926` | Static TF yaw (rad). |
| `enable_indices` | `false` | Launch indices node(s). |
| `indices_enabled` | `true` | Initial runtime state for indices node(s). |
| `indices_params_file` | package `config/mapir_indices_params.yaml` if present | Indices YAML source. |
| `indices_per_node` | `false` | Launch one indices node per index. |
| `indices_all` | `false` | Use all supported indices. |

## Camera Node Params (`camera_node`)

| Param | Type | Default |
|---|---|---|
| `debug` | `bool` | `false` |
| `debug_period_s` | `float` | `1.0` |
| `video_device` | `str` | `/dev/video0` |
| `image_width` | `int` | `1280` |
| `image_height` | `int` | `720` |
| `framerate` | `float` | `30.0` |
| `pixel_format` | `str` | `MJPG` |
| `use_gstreamer` | `bool` | `false` |
| `gstreamer_pipeline` | `str` | `''` |
| `frame_id` | `str` | `mapir3_optical_frame` |
| `camera_name` | `str` | `mapir3_ocn` |
| `camera_info_url` | `str` | `''` |
| `qos_best_effort` | `bool` | `true` |
| `qos_depth` | `int` | `5` |
| `uvc_controls_enabled` | `bool` | `false` |
| `uvc_controls_device` | `str` | `''` |
| `auto_exposure_mode` | `int` | `-1` |
| `exposure_time_absolute` | `int` | `-1` |
| `gain` | `int` | `-1` |
| `exposure_dynamic_framerate` | `int` | `-1` |
| `white_balance_automatic` | `int` | `-1` |
| `white_balance_temperature` | `int` | `-1` |
| `power_line_frequency` | `int` | `-1` |
| `metadata_enabled` | `bool` | `false` |
| `metadata_device` | `str` | `/dev/video1` |
| `metadata_topic` | `str` | `metadata` |
| `metadata_log_path` | `str` | `''` |
| `metadata_log_flush_every_n` | `int` | `30` |
| `vignette_enabled` | `bool` | `false` |
| `vignette_flatfield_b_path` | `str` | `''` |
| `vignette_flatfield_g_path` | `str` | `''` |
| `vignette_flatfield_r_path` | `str` | `''` |
| `vignette_dark_current_b` | `int` | `0` |
| `vignette_dark_current_g` | `int` | `0` |
| `vignette_dark_current_r` | `int` | `0` |

## Indices Node Params (`indices_node`)

| Param | Type | Default |
|---|---|---|
| `debug` | `bool` | `false` |
| `debug_period_s` | `float` | `1.0` |
| `enabled` | `bool` | `true` |
| `image_topic` | `str` | `image_raw` |
| `indices` | `list[str]` | `['ndvi', 'osavi']` |
| `filter_set` | `str` | `''` |
| `normalize_input` | `bool` | `true` |
| `downsample_factor` | `int` | `1` |
| `publish_every_n` | `int` | `1` |
| `publish_color` | `bool` | `false` |
| `colormap` | `str` | `viridis` |
| `colorize_min` | `float` | `-1.0` |
| `colorize_max` | `float` | `1.0` |
| `custom_colormap` | `str` | `''` |
| `eps` | `float` | `1e-6` |
| `gari_gamma` | `float` | `1.7` |
| `wdrvi_alpha` | `float` | `0.2` |
| `mnli_L` | `float` | `0.5` |
| `blue_channel` | `int` | `-1` |
| `green_channel` | `int` | `-1` |
| `red_channel` | `int` | `-1` |
| `rededge_channel` | `int` | `-1` |
| `nir_channel` | `int` | `-1` |
| `nir1_channel` | `int` | `-1` |
| `nir2_channel` | `int` | `-1` |
| `cyan_channel` | `int` | `-1` |
| `orange_channel` | `int` | `-1` |
| `qos_best_effort` | `bool` | `true` |
| `qos_depth` | `int` | `5` |
