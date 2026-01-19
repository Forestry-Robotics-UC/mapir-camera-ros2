# Architecture

![Architecture placeholder](../_static/architecture_placeholder.svg)

## Repository layout

- `mapir_camera_ros2/`: Python package (nodes and ROS-agnostic core logic).
  - V4L2/OpenCV helpers (`core/v4l2_camera.py`)
  - multispectral index math (`core/spectral_indices.py`)
  - reflectance calibration utilities (`core/reflectance.py`, `core/target_detection.py`)
- `src/`: C++ sources.
- `include/mapir_camera_ros2/`: installed C++ headers.
- `tools/`: Offline utilities (RAW calibration, dataset helpers).
- `launch/`, `config/`, `doc/`: launch files, parameters, and docs.

## Workspace best practices

- Keep source code organized in the `src/` directory of a ROS 2 workspace.
- Track source with Git; avoid committing `build/`, `install/`, or `log/`.
- Use separate workspaces for separate projects or development contexts.
- Use workspace overlays only when needed for multi-repo development.

## Package layout reference

This package follows the standard ROS 2 layout. Not every folder is required
for every package, but this is the recommended structure:

```
mapir_camera_ros2/
├── README.md
├── LICENSE
├── CMakeLists.txt
├── package.xml
├── config/
├── doc/
├── launch/
├── include/
│   └── mapir_camera_ros2/
├── src/
├── mapir_camera_ros2/
│   ├── __init__.py
│   ├── core/
│   └── nodes/
└── test/
```

## Data flow

1. `camera_node`/`camera_node_cpp` capture frames from `/dev/video*` and publish:
   - `/<ns>/image_raw` (`sensor_msgs/Image`)
   - `/<ns>/camera_info` (`sensor_msgs/CameraInfo`)
2. Optional `indices_node` subscribes to `/<ns>/image_raw` and publishes:
   - `/<ns>/indices/<index>` (`sensor_msgs/Image`, `32FC1`)
3. Optional `reflectance_node` subscribes to `/<ns>/image_raw` and publishes:
   - `/<ns>/image_rect` (reflectance-corrected, `32FC3`)

## Design intent

Keep ROS-independent processing in `mapir_camera_ros2/core` so it can be reused later
for non-ROS interfaces (e.g., HTTP SDK, file batch processing).
