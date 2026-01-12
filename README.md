# MAPIR Survey3 ROS 2 Camera Driver (Jazzy)

ROS 2 Jazzy camera driver for MAPIR Survey3 cameras (including OCN variants),
implemented in Python (ament-python) using OpenCV + V4L2.

This package publishes synchronized camera streams suitable for real-time robotics
pipelines and vegetation analysis workflows.

---

## Documentation

- Manual (Sphinx/MyST): `docs/index.md`
- Parameters reference: `docs/manual/parameters.md`

Build docs (GitHub Pages):
```
pip install -r docs/requirements.txt
sphinx-build -b html docs docs/_build/html
touch docs/_build/html/.nojekyll
```

---

## Features

- ROS 2 Jazzy compatible
- Publishes:
  - /<ns>/image_raw (sensor_msgs/Image)
  - /<ns>/camera_info (sensor_msgs/CameraInfo)
- Supports MJPG and H264 pixel formats (device-dependent)
- Uses V4L2 backend explicitly (no GStreamer ambiguity)
- Supports up to 60 Hz
- BEST_EFFORT QoS by default (RViz / rqt compatible)
- Uses camera_info_manager
  - Publishes default CameraInfo if calibration is missing
- Extensive debug logging (rate, failures, timing)
- Designed for Jetson / embedded and desktop Linux

---

## Topics

All topics are published relative to the node namespace (recommended: /mapir).

- /mapir/image_raw
- /mapir/camera_info

---

## Parameters

- Full parameter reference: `docs/manual/parameters.md`

Preset YAML files (recommended):
- `config/mapir_camera_params.yaml`
- `config/mapir_indices_params.yaml`
- `config/mapir_indices_ocn_params.yaml`

Common camera_node parameters:

| Param | Type | Default | Notes |
|---|---|---|---|
| `video_device` | `str` | `/dev/video0` | V4L2 device path or numeric index. |
| `image_width` | `int` | `1280` | Requested width. |
| `image_height` | `int` | `720` | Requested height. |
| `framerate` | `float` | `30.0` | Requested FPS. |
| `pixel_format` | `str` | `MJPG` | `MJPG` or `H264` (device-dependent). |
| `use_gstreamer` | `bool` | `false` | Use GStreamer pipeline for capture. |
| `gstreamer_pipeline` | `str` | `''` | Custom GStreamer pipeline string. |
| `frame_id` | `str` | `mapir3_optical_frame` | REP-105 optical frame. |
| `camera_info_url` | `str` | `''` | `file:///.../calib.yaml`. |
| `qos_best_effort` | `bool` | `true` | BEST_EFFORT recommended. |

Common indices_node parameters:

| Param | Type | Default | Notes |
|---|---|---|---|
| `indices` | `list[str]` | `['ndvi', 'osavi']` | Add `_1`/`_2` to prefer NIR1/NIR2. |
| `filter_set` | `str` | `''` | `RGN`, `NGB`, `OCN`, `RGB`. |
| `normalize_input` | `bool` | `true` | Normalize integer images to `[0,1]`. |
| `downsample_factor` | `int` | `1` | Stride downsample for CPU savings. |
| `publish_every_n` | `int` | `1` | Publish every Nth frame. |
| `publish_color` | `bool` | `false` | Publish colorized outputs. |
| `colormap` | `str` | `viridis` | `viridis`, `jet`, `gray`, `custom`, ... |
| `*_channel` | `int` | `-1` | Explicit band mapping overrides. |

---

## Multispectral Indices

This package includes an optional processing node, `indices_node`, that computes common
vegetation / spectral indices (e.g., NDVI, GNDVI) from the incoming 3-channel image.
For real-time performance, keep the indices list to 1-2 entries at a time.

Published topics (relative to the namespace):

- /<ns>/indices/<index_name> (sensor_msgs/Image, encoding 32FC1)

Enable via launch (uses the package default params file unless overridden):
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py namespace:=mapir enable_indices:=true
```

Adjust indices + band mapping via `config/mapir_indices_params.yaml` (installed with the package).
The `filter_set` preset is best-effort and can be overridden with explicit `*_channel` parameters.

Core computations live in `mapir_camera_core`.

For Survey3W OCN streams, set `filter_set: OCN` (or use `config/mapir_indices_ocn_params.yaml`).
The preset aliases `cyan→blue` and `orange→red` for index computation (no green/rededge band).

Index summary (best use + required bands):

| Index | Best use | Bands |
|---|---|---|
| NDVI | General vegetation vigor | NIR + Red |
| NDRE | Early stress / chlorophyll | NIR + RedEdge |
| EVI | High-LAI vegetation, less saturation | NIR + Red + Blue |
| LAI | LAI proxy (derived from EVI) | NIR + Red + Blue |
| SAVI | Sparse vegetation, soil background | NIR + Red |
| OSAVI | Soil-adjusted, fixed L | NIR + Red |
| MSAVI2 | Soil noise reduction | NIR + Red |
| RDVI | Reduced soil sensitivity | NIR + Red |
| TDVI | Reduced saturation in mixed scenes | NIR + Red |
| WDRVI | High NDVI range sensitivity | NIR + Red |
| NLI | Non-linear canopy response | NIR + Red |
| MNLI | NLI with soil adjustment | NIR + Red |
| GEMI | Atmospheric effect reduction | NIR + Red |
| FCI1 | Forest cover (red-edge) | Red + RedEdge |
| FCI2 | Forest cover (no red-edge) | Red + NIR |
| LCI | Chlorophyll with red-edge | NIR + RedEdge + Red |
| GNDVI | Chlorophyll (green-based) | NIR + Green |
| GCI | Chlorophyll index | NIR + Green |
| GRVI | Pigment/vigor proxy | NIR + Green |
| GOSAVI | Soil-adjusted (green-based) | NIR + Green |
| GSAVI | Soil-adjusted (green-based) | NIR + Green |
| GARI | Atmospheric-resistant greenness | NIR + Green + Blue + Red |
| GLI | RGB-only greenness | Green + Red + Blue |
| VARI | RGB vegetation fraction | Green + Red + Blue |

---

## Calibration

Calibration files follow the standard ROS camera calibration format.

Example path:
~/.ros/camera_info/mapir3_ocn.yaml

Run with:
-p camera_info_url:=file:///root/.ros/camera_info/mapir3_ocn.yaml

If calibration is missing or invalid, the node continues publishing
with a valid but uncalibrated CameraInfo.

---

## Build (ROS 2)

From the repo root:
```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Node

Direct execution:
```
ros2 run mapir_camera_ros2 camera_node 
```
Direct execution (C++ node):
```
ros2 run mapir_camera_cpp camera_node_cpp
```
Direct execution using the installed preset params file:
```
ros2 run mapir_camera_ros2 camera_node --ros-args --params-file \
  $(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_camera_params.yaml
```
Debug with explicit log level:
```
ros2 run mapir_camera_ros2 camera_node --ros-args --log-level mapir_camera:=debug -r __ns:=/mapir -p debug:=true
```
GStreamer capture (optional, for Jetson hardware decode):
```
ros2 run mapir_camera_ros2 camera_node --ros-args -r __ns:=/mapir \
  -p use_gstreamer:=true \
  -p gstreamer_pipeline:="v4l2src device=/dev/video0 ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! nvv4l2decoder ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
```
## Launch File
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_camera_params.yaml \
  indices_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_indices_params.yaml \
  debug:=true qos_best_effort:=true \
  camera_info_url:=file:///root/.ros/camera_info/mapir3_ocn.yaml
```

Note: `$(ros2 pkg prefix ...)` points to the installed share folder. If you are running
from source without installing, point directly at the repo:
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_params_file:=/path/to/mapir_camera/config/mapir_camera_params.yaml \
  indices_params_file:=/path/to/mapir_camera/config/mapir_indices_params.yaml
```

## Debugging Tips

Check negotiated formats:
```
v4l2-ctl --device=/dev/video0 --list-formats-ext
```
Verify publishing:
```
ros2 topic info /mapir/image_raw -v  
ros2 topic hz /mapir/image_raw
```
RViz QoS (image not updating):
- Set Image display QoS Reliability to `Best Effort`, or
- launch with `qos_best_effort:=false` to publish RELIABLE.
Run without rebuild (fast iteration):

```
python3 -m mapir_camera_ros2.camera_node --ros-args -r __ns:=/mapir -p debug:=true
```

## Docker

Build:
```
docker build -t mapir_camera_ros2:jazzy .
```

Run (example with V4L2 camera at `/dev/video0`):
```
docker run --rm -it --net=host --device=/dev/video0 mapir_camera_ros2:jazzy
```

Inside the container:
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py namespace:=mapir video_device:=/dev/video0
```

Per-index nodes (avoid multi-index bottleneck; keep 1-2 indices per node):
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir enable_indices:=true indices_per_node:=true
```
Use all supported indices:
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir enable_indices:=true indices_per_node:=true indices_all:=true
```

Note: you may need extra device permissions depending on your host setup (e.g., `--privileged`).

Docker compose (runtime, in `Docker/`):
```
cd Docker
docker compose build
docker compose up mapir_camera
```

## License

GNU General Public License v3.0 (GPL-3.0-only)

© Duda Andrada

---

## Maintainer

Duda Andrada  
duda.andrada@isr.uc.pt
