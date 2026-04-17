#!/bin/bash
set -euo pipefail

stamp="${MAPIR_CAL_RUN_TAG:-$(date +%Y_%m%d_%H%M%S)}"
purpose="${MAPIR_REFLECTANCE_PURPOSE:-aruco_auto}"
out_dir="/outputs/reflectance_calibration/${stamp}__${purpose}"

mkdir -p "${out_dir}"

echo "Running reflectance calibration with ArUco auto-detection..."

python3 /workspaces/ros2_ws/src/mapir_camera/tools/reflectance_calibration.py \
  --calibration-image /outputs/reflectance_calibration/2026_0417_153120_001.RAW \
  --panel-config /workspaces/ros2_ws/src/mapir_camera/config/calibration_files/reflection/reflectance_panels.selected.json \
  --aruco-mode \
  --target-layout /workspaces/ros2_ws/src/mapir_camera/config/t4-r50_mapir_reflactance_target_layout.json \
  --camera-intrinsics /workspaces/ros2_ws/src/mapir_camera/config/mapir3_ocn_4000x3000.yaml \
  --out-dir "${out_dir}" \
  "$@"

ln -sfn "$(basename "${out_dir}")" /outputs/reflectance_calibration/latest
echo "✓ Calibration complete: ${out_dir}"
