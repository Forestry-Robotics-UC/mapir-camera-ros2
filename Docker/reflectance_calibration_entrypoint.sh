#!/bin/bash
set -euo pipefail

stamp="${MAPIR_CAL_RUN_TAG:-$(date +%Y_%m%d_%H%M%S)}"
purpose="${MAPIR_REFLECTANCE_PURPOSE:-reflection}"
out_dir="/outputs/reflectance_calibration/${stamp}__${purpose}"

mkdir -p "${out_dir}"

python3 /workspaces/ros2_ws/src/mapir_camera/tools/reflectance_calibration.py \
  --calibration-image /outputs/reflectance_calibration/2026_0417_153148_009.RAW \
  --panel-config /workspaces/ros2_ws/src/mapir_camera/config/calibration_files/reflection/reflectance_panels.example.json \
  --apply-dir /outputs/reflectance_calibration/input \
  --glob "*.png" \
  --panel-min-frac "${MAPIR_REF_PANEL_MIN_FRAC:-0.02}" \
  --panel-max-frac "${MAPIR_REF_PANEL_MAX_FRAC:-0.98}" \
  --out-dir "${out_dir}" \
  "$@"

ln -sfn "$(basename "${out_dir}")" /outputs/reflectance_calibration/latest
