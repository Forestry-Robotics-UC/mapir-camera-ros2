# Calibration

## Why three steps

For consistent quantitative outputs, keep calibration separated:

1. **Intrinsic calibration** (geometry/lens distortion)
2. **Vignette correction** (spatial non-uniformity / flat-field)
3. **Reflectance calibration** (DN to reflectance mapping)

This separation follows standard remote-sensing/radiometric practice.

Empirical-line calibration model used here (per channel):

- `rho = a * DN + b`
  - `rho`: unitless reflectance in `[0, 1]`
  - `DN`: measured pixel value
  - `a`, `b`: fitted gain/offset from panel ROIs

## 1) Intrinsic (ROS package)

Use ROS `camera_calibration` (`cameracalibrator`) from Docker:

```bash
cd Docker
docker compose up -d mapir_camera
docker compose run --rm camera_calibration
```

Use a printed chessboard, cover diverse orientations/distances, then click
**CALIBRATE** and **SAVE**.

## 2) Vignette (flat-field)

Capture a uniformly lit target/scene and generate per-channel flat maps:

```bash
cd Docker
docker compose run --rm vignette_calibration
```

Outputs:
- `flat_b.tiff`, `flat_g.tiff`, `flat_r.tiff`
- `sample_before_after.png`
- `vignette_report.json`

Enable in node:

```bash
ros2 run mapir_camera_ros2 camera_node --ros-args \
  -p vignette_enabled:=true \
  -p vignette_flatfield_b_path:=/abs/path/flat_b.tiff \
  -p vignette_flatfield_g_path:=/abs/path/flat_g.tiff \
  -p vignette_flatfield_r_path:=/abs/path/flat_r.tiff
```

## 3) Reflectance (empirical line)

Prepare:
- one image containing the MAPIR reflectance target (`calibration_target.png`)
- known panel reflectance values (from panel datasheet), mapped to ROIs in
  `config/reflectance_panels.example.json`
- optional batch input images in `outputs/reflectance_calibration/input/*.png`

Run:

```bash
cd Docker
docker compose run --rm reflectance_calibration
```

Method:
- For each panel ROI and each channel, compute mean DN.
- Fit per-channel linear model:
  - `reflectance = slope * DN + intercept`
- Apply model to each image and clamp to `[0, 1]`.

Outputs:
- `reflectance_report.json` (fitted coefficients, panel stats)
- `panel_rois_overlay.png`
- `applied/*_reflectance_f32.tiff`
- `applied/*_reflectance_u16.tiff`

## Best-practice capture notes

- Lock exposure/gain/white balance before calibration captures.
- Avoid clipping and shadows on target panels.
- Use the same camera settings and illumination geometry between target capture
  and the images you calibrate.
- Re-run reflectance calibration when illumination conditions change
  significantly.

## References

- ROS camera calibration package:
  - `camera_calibration` (`cameracalibrator`) in ROS 2
- MAPIR calibration target / reflectance workflow:
  - https://mapir.gitbook.io/mapir-camera-control-mcc/calibration-targets
- Flat-field correction (sensor non-uniformity):
  - https://docs.baslerweb.com/flat-field-correction
- Remote-sensing calibration concepts (flat field + empirical line):
  - https://pages.mtu.edu/~scarn/teaching/GE4250/1999-fundamentals-of-remote-sensing.pdf
