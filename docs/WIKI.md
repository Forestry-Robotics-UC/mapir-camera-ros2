# MAPIR Survey3 Camera (ROS 2 Jazzy) Wiki

This page is a plain Markdown reference intended for GitHub Wiki use.
It summarizes setup, launch, parameters, and troubleshooting for the
`mapir_camera_ros2` package.

---

## Overview

`mapir_camera_ros2` is a ROS 2 Jazzy camera driver for MAPIR Survey3 cameras
(including OCN variants). It publishes synchronized camera streams and can
optionally compute multispectral indices (e.g., NDVI).

---

## Repository Layout

- `mapir_camera_ros2/`: ROS-facing nodes (rclpy publishers/subscribers, parameters, QoS).
- `mapir_camera_core/`: ROS-agnostic logic (V4L2 helpers, index math).
- `launch/`: Launch files.
- `config/`: Parameter presets and calibration template.
- `docs/`: Reference docs (Sphinx/MyST).

---

## Build (ROS 2)

From the repo root:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run the Camera Node

Baseline launch:
```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  video_device:=/dev/video0 \
  image_width:=1920 image_height:=1440 \
  framerate:=30.0 \
  pixel_format:=MJPG
```

Validate:
```bash
ros2 topic hz /mapir/image_raw
ros2 topic echo --once /mapir/image_raw --field header
```

---

## Enable Indices

RGN baseline:
```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  enable_indices:=true
```

Survey3W OCN preset:
```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  enable_indices:=true \
  indices_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_indices_ocn_params.yaml
```

---

## Calibration

Calibration files follow the standard ROS camera calibration format.

Template:
- `config/mapir3_ocn.example.yaml`

Copy it to:
- `~/.ros/camera_info/mapir3_ocn.yaml`

Run with:
```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_info_url:=file:///root/.ros/camera_info/mapir3_ocn.yaml
```

If calibration is missing or invalid, the node continues publishing
with an uncalibrated (but valid) CameraInfo.

---

## ROS Contract

Nodes:
- `camera_node` (python entry: `mapir_camera_ros2.camera_node`)
- `indices_node` (python entry: `mapir_camera_ros2.indices_node`)

Topics (relative to namespace):
- `/<ns>/image_raw` (`sensor_msgs/Image`, encoding `bgr8`)
- `/<ns>/camera_info` (`sensor_msgs/CameraInfo`)
- `/<ns>/indices/<index>` (`sensor_msgs/Image`, encoding `32FC1`)
- `/<ns>/indices_color/<index>` (`sensor_msgs/Image`, encoding `bgr8`, optional)

QoS:
- Default: BEST_EFFORT, depth 5

Frame IDs:
- Default `frame_id` is `mapir3_optical_frame` (REP-105 optical frame convention)

Timestamps:
- Image and CameraInfo stamps use the node clock at publish time.

---

## Parameters

Camera node (`camera_node`):

| Param | Type | Default | Description |
|---|---|---|---|
| `debug` | `bool` | `false` | Enable extra logs. |
| `debug_period_s` | `float` | `1.0` | Throttle for periodic debug logs. |
| `video_device` | `str` | `'/dev/video0'` | V4L2 device path or numeric index. |
| `image_width` | `int` | `1920` | Requested width. |
| `image_height` | `int` | `1440` | Requested height. |
| `framerate` | `float` | `30.0` | Requested FPS. |
| `pixel_format` | `str` | `'MJPG'` | `'MJPG'` or `'H264'` (device-dependent). |
| `frame_id` | `str` | `'mapir3_optical_frame'` | Output frame_id for Image/CameraInfo. |
| `camera_name` | `str` | `'mapir3_ocn'` | Camera name for calibration lookup. |
| `camera_info_url` | `str` | `''` | Calibration URL (`file:///.../calib.yaml`). |
| `qos_best_effort` | `bool` | `true` | BEST_EFFORT vs RELIABLE. |
| `qos_depth` | `int` | `5` | Publisher queue depth. |

Indices node (`indices_node`):

| Param | Type | Default | Description |
|---|---|---|---|
| `debug` | `bool` | `false` | Enable extra logs. |
| `debug_period_s` | `float` | `1.0` | Throttle for periodic debug logs. |
| `enabled` | `bool` | `true` | Runtime toggle (skip compute/publish when false). |
| `image_topic` | `str` | `'image_raw'` | Input image topic (relative to namespace). |
| `indices` | `list[str]` | `['ndvi']` | Index names; add `_1`/`_2` to prefer NIR1/NIR2. |
| `filter_set` | `str` | `''` | Band preset: `RGN`, `NGB`, `OCN`, `RGB`. |
| `normalize_input` | `bool` | `true` | Normalize integer images to `[0,1]`. |
| `downsample_factor` | `int` | `1` | Stride downsample for CPU savings. |
| `publish_every_n` | `int` | `1` | Publish every Nth frame (drop others). |
| `publish_color` | `bool` | `false` | Publish a colorized BGR image per index. |
| `colormap` | `str` | `'viridis'` | Colormap name (`viridis`, `jet`, `gray`, `custom`, ...). |
| `colorize_min` | `float` | `-1.0` | Value mapped to the low end of the colormap. |
| `colorize_max` | `float` | `1.0` | Value mapped to the high end of the colormap. |
| `custom_colormap` | `str` | `''` | Custom points: `value,r,g,b; value,r,g,b; ...`. |
| `eps` | `float` | `1e-6` | Safe-divide epsilon. |
| `gari_gamma` | `float` | `1.7` | GARI gamma constant. |
| `wdrvi_alpha` | `float` | `0.2` | WDRVI alpha constant. |
| `mnli_L` | `float` | `0.5` | MNLI background adjustment factor. |
| `*_channel` | `int` | `-1` | Override mapping (0=B,1=G,2=R); `-1` leaves unset. |
| `qos_best_effort` | `bool` | `true` | Match camera stream QoS. |
| `qos_depth` | `int` | `5` | Subscription/publisher depth. |

Preset files:
- `config/mapir_camera_params.yaml` (camera)
- `config/mapir_indices_params.yaml` (RGN baseline)
- `config/mapir_indices_ocn_params.yaml` (Survey3W OCN)

---

## Indices Summary

Supported indices:
`evi`, `fci1`, `fci2`, `gemi`, `gari`, `gci`, `gli`, `gndvi`, `gosavi`, `grvi`,
`gsavi`, `lai`, `lci`, `mnli`, `msavi2`, `ndre`, `ndvi`, `nli`, `osavi`,
`rdvi`, `savi`, `tdvi`, `vari`, `wdrvi`.

Filter-set compatibility (3-band streams):

| Filter set | Available bands (logical) | Notes |
|---|---|---|
| RGB | Blue, Green, Red | No NIR. |
| RGN | Green, Red, NIR2 | Common NDVI workflow. |
| NGB | Blue, Green, NIR2 | No Red. |
| OCN | Cyan, Orange, NIR1 | Cyan -> Blue, Orange -> Red aliases. |

---

## Troubleshooting

`/dev/video0` permission denied:
- Check `ls -l /dev/video0` and `id`
- Ensure your user is in the `video` group (or add a udev rule)

Device busy:
- Find holder with `fuser -v /dev/video0` and stop it

Not hitting 30 Hz:
- Try `pixel_format:=H264`
- Reduce resolution
- Use `downsample_factor` and/or `publish_every_n` for indices

Indices missing:
- Ensure `filter_set` matches the camera and bands
- Set explicit `*_channel` overrides

---

## Docker (Optional)

Build:
```bash
docker build -t mapir_camera_ros2:jazzy .
```

Run:
```bash
docker run --rm -it --net=host --device=/dev/video0 mapir_camera_ros2:jazzy
```
