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

To run the camera with a fixed-exposure calibration profile (recommended for
lower reprojection error), point `mapir_camera` to the calibration params file:

```bash
cd Docker
MAPIR_CAMERA_PARAMS_FILE=/workspaces/ros2_ws/src/mapir_camera/config/mapir_camera_calibration_params.yaml \
  docker compose up -d mapir_camera
MAPIR_CAMERA_PARAMS_FILE=/workspaces/ros2_ws/src/mapir_camera/config/mapir_camera_calibration_params.yaml \
  docker compose run --rm camera_calibration
```

For MAPIR OCN captures, keep white-balance controls unchanged to avoid
cross-channel remapping that is acceptable for RGB cameras but harmful for
multispectral channel consistency.

The GUI services mount the X11 cookie at `/root/.Xauthority` inside the
container. If your desktop session still rejects the connection, allow local
root access before starting calibration:

```bash
xhost +si:localuser:root
```

If the calibration window opens but Mesa reports DRM or `iris` driver errors,
force software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 docker compose run --rm camera_calibration
```

The compose service sets this fallback by default.

Use a printed chessboard, cover diverse orientations/distances, then click
**CALIBRATE** and **SAVE**.

## 2) Vignette (flat-field)

Capture a uniformly lit target/scene and generate per-channel flat maps:

```bash
cd Docker
docker compose run --rm vignette_calibration
```

Best-quality setup checklist:

- Use a bright, matte, uniform target (white board / opal diffuser), not black cardboard.
- Fill the full frame with the target and keep the lens slightly defocused.
- Use diffuse, stable illumination (avoid hard shadows, hot spots, and flicker).
- Keep capture and runtime resolution identical (flat fields must match frame size exactly).
- Keep OCN-safe controls: lock exposure/gain, disable dynamic FPS, leave white balance unchanged.

Quick do/don't:

- Do: evenly lit white target, full-frame coverage, 80-120 frames.
- Don't: dark textured surfaces, glossy reflections, mixed indoor/outdoor lighting.

Example files to inspect after a run:

- `outputs/vignette_calibration/flat_b_preview.png`
- `outputs/vignette_calibration/flat_g_preview.png`
- `outputs/vignette_calibration/flat_r_preview.png`
- `outputs/vignette_calibration/sample_before_after.png`
- `outputs/vignette_calibration/vignette_report.json`

In good results, the flat previews are smooth (no sharp blobs/texture), and
`sample_before_after.png` shows reduced edge darkening without clipping.

Automatic quality gate (in `vignette_calibration.py`):

- `frame_mean_cv` (frame-to-frame brightness stability), default pass: `<= 0.03`
- `clipping_fraction` (near-black + near-saturated pixels), default pass: `<= 0.01`
- `texture_rms` (high-frequency residual texture after low-pass removal), default pass: `<= 0.06`

The run writes these metrics into `vignette_report.json` and fails fast if any
threshold is exceeded.

Optional threshold overrides:

```bash
docker compose run --rm vignette_calibration \
  --max-frame-mean-cv 0.02 \
  --max-clipping-fraction 0.005 \
  --max-texture-rms 0.05
```

Outputs:
- `flat_b.tiff`, `flat_g.tiff`, `flat_r.tiff`
- `sample_before_after.png`
- `vignette_report.json`

By default, the Docker vignette workflow writes these files into:
- `config/calibration_files/vignette/`
- Each run is saved into a dated folder like `config/calibration_files/vignette/2026_0417_153500__vignette/`.
- `config/calibration_files/vignette/latest/` is refreshed to point at the newest run.

Enable in node:

```bash
ros2 run mapir_camera_ros2 camera_node --ros-args \
  -p vignette_enabled:=true \
  -p vignette_flatfield_b_path:=config/calibration_files/vignette/latest/flat_b.tiff \
  -p vignette_flatfield_g_path:=config/calibration_files/vignette/latest/flat_g.tiff \
  -p vignette_flatfield_r_path:=config/calibration_files/vignette/latest/flat_r.tiff
```

## 3) Reflectance (empirical line)

Prepare:
- one image containing the MAPIR reflectance target
- known panel reflectance values (from panel datasheet), mapped to ROIs in
  `config/reflectance_panels.example.json`
- optional batch input images in `config/calibration_files/reflection/input/*.png`
- prefer 16-bit input images for reflectance calibration (8-bit is non-ideal)

Default target profile in this repo:
- `config/reflectance_panels.example.json` now assumes the MAPIR `T4-R50` form factor.
- The default panel template now contains 4 panel entries to match the `T4-R50`
  target layout.
- The default calibration image is the local RAW target capture
  `config/calibration_files/reflection/2026_0417_153148_009.RAW`.
- MAPIR lists each reflectance panel as `2.0" x 2.0"` (`50 x 50 mm`).
- The default OCN `reflectance_bgr` values are now populated from the MAPIR
  workbook `MAPIR_Diffuse_Reflectance_Standard_Calibration_Target_Data_T4.xlsx`
  using the `Diffuse Reflectivity` curves sampled at the OCN band centers
  `cyan=494 nm`, `orange=619 nm`, and `nir1=823 nm`.
- MAPIR also lists recommended capture distance as `0.2-5.0 m` for Survey3W and
  `0.5-12.0 m` for Survey3N.
- Keep in mind: these values are correct for the provided `T4` diffuse
  reflectance workbook and the OCN band-center approximation. If you later use
  a different filter set, derive the per-channel values from the same spectral
  curves for that filter set instead of reusing the OCN numbers unchanged.

Run:

```bash
cd Docker
docker compose run --rm reflectance_calibration
```

Simplest ROI setup workflow:

```bash
cd Docker
xhost +si:localuser:root
docker compose run --rm reflectance_calibration \
  --select-rois \
  --panel-config /workspaces/ros2_ws/src/mapir_camera/config/calibration_files/reflection/reflectance_panels.example.json \
  --calibration-image /outputs/reflectance_calibration/2026_0417_153120_001.RAW \
  --panel-config-out /tmp/reflectance_panels.selected.json
```

This uses OpenCV's built-in ROI selector. Drag one rectangle per panel in the
same order as the panel entries in the JSON file, then press Enter to save the
updated config. If you want to pick ROIs on a different target capture, override
`--calibration-image`. `--apply-image` is only for images that should receive the
reflectance model after calibration.


Method:
- For each panel ROI and each channel, compute mean DN.
- Reject panel ROIs with mean DN outside configured quality bounds
  (`panel_min_frac`, `panel_max_frac`) to avoid near-black/saturated fits.
- RAW `.RAW` captures are decoded with the existing Survey3 RAW loader before ROI
  sampling so the calibration stays on the high-fidelity sensor data path.
- Fit per-channel linear model:
  - `reflectance = slope * DN + intercept`
- Apply model to each image and clamp to `[0, 1]`.

Outputs:
- `reflectance_report.json` (fitted coefficients, panel stats)
- `panel_rois_overlay.png`
- `applied/*_reflectance_f32.tiff`
- `applied/*_reflectance_u16.tiff`
- `reflectance_report.json` includes rejected panels, DN threshold settings, and
  any non-panel metadata from the panel config file (for example the default
  `T4-R50` target size metadata).

By default, the Docker reflectance workflow writes these files into:
- `config/calibration_files/reflection/`
- Each run is saved into a dated folder like `config/calibration_files/reflection/2026_0417_154200__reflection/`.
- `config/calibration_files/reflection/latest/` is refreshed to point at the newest run.

## Best-practice capture notes

- Lock exposure/gain/white balance before calibration captures.
- Avoid clipping and shadows on target panels.
- Use the same camera settings and illumination geometry between target capture
  and the images you calibrate.
- Re-run reflectance calibration when illumination conditions change
  significantly.
- Avoid using gamma-encoded 8-bit exports for scientific calibration whenever
  possible; calibrate from highest-fidelity radiometric source.

## References

- ROS camera calibration package:
  - `camera_calibration` (`cameracalibrator`) in ROS 2
- MAPIR calibration target / reflectance workflow:
  - https://mapir.gitbook.io/mapir-camera-control-mcc/calibration-targets
- MAPIR `T4-R50` product/spec page:
  - https://www.mapir.camera/products/diffuse-reflectance-standard-calibration-target-package-t4-r50
- Flat-field correction (sensor non-uniformity):
  - https://docs.baslerweb.com/flat-field-correction
- Remote-sensing calibration concepts (flat field + empirical line):
  - https://pages.mtu.edu/~scarn/teaching/GE4250/1999-fundamentals-of-remote-sensing.pdf
