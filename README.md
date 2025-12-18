# MAPIR Survey3 ROS 2 Camera Driver (Jazzy)

ROS 2 Jazzy camera driver for MAPIR Survey3 cameras (including OCN variants),
implemented in Python (ament-python) using OpenCV + V4L2.

This package publishes synchronized camera streams suitable for real-time robotics
pipelines and vegetation analysis workflows.

---

## Documentation

- Manual (Sphinx/MyST): `docs/index.md`
- Parameters reference: `docs/manual/parameters.md`

Build docs (optional):
```
pip install -r docs/requirements.txt
sphinx-build -b html docs docs/_build/html
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
- Preset YAML files (recommended):
  - `config/mapir_camera_params.yaml`
  - `config/mapir_indices_params.yaml`
  - `config/mapir_indices_ocn_params.yaml`

---

## Multispectral Indices

This package includes an optional processing node, `indices_node`, that computes common
vegetation / spectral indices (e.g., NDVI, GNDVI) from the incoming 3-channel image.

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

---

## Calibration

Calibration files follow the standard ROS camera calibration format.

Example path:
~/.ros/camera_info/mapir3.yaml

Run with:
-p camera_info_url:=file:///root/.ros/camera_info/mapir3.yaml

If calibration is missing or invalid, the node continues publishing
with a valid but uncalibrated CameraInfo.

---

## Running the Node

Direct execution:
```
ros2 run mapir_camera_ros2 camera_node 
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
## Launch File
```
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_camera_params.yaml \
  indices_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_indices_params.yaml \
  debug:=true qos_best_effort:=true \
  camera_info_url:=file:///root/.ros/camera_info/mapir3.yaml
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

Note: you may need extra device permissions depending on your host setup (e.g., `--privileged`).

## License

GNU General Public License v3.0 (GPL-3.0)

© Duda Andrada

---

## Maintainer

Duda Andrada  
duda.andrada@isr.uc.pt
