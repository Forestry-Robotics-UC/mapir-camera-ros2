# Parameters

Parameter defaults come from:

- code defaults (declared in the nodes), and
- package YAML presets in `config/` (recommended for repeatability).

## Camera node (`camera_node`)

| Param | Type | Default | Description |
|---|---|---|---|
| `debug` | `bool` | `false` | Enable extra logs. |
| `debug_period_s` | `float` | `1.0` | Throttle for periodic debug logs. |
| `video_device` | `str` | `'/dev/video0'` | V4L2 device path or numeric index. |
| `image_width` | `int` | `1280` | Requested width. |
| `image_height` | `int` | `720` | Requested height. |
| `framerate` | `float` | `30.0` | Requested FPS. |
| `pixel_format` | `str` | `'MJPG'` | `'MJPG'` or `'H264'` (device-dependent). |
| `use_gstreamer` | `bool` | `false` | Use a GStreamer pipeline for capture. |
| `gstreamer_pipeline` | `str` | `''` | Custom GStreamer pipeline string. |
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
| `filter_set` | `str` | `''` | Band preset: `RGN`, `NGB`, `OCN`, `RGB`. |
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

- `config/mapir_indices_params.yaml` (RGN baseline)
- `config/mapir_indices_ocn_params.yaml` (Survey3W OCN)
