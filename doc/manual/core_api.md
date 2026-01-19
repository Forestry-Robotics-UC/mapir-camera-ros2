# Core API Reference

## Complexity (Big-O)

Approximate time complexity for core operations (per frame):

| Function | Big-O | Notes |
|---|---|---|
| `extract_bands_from_bgr` | `O(N)` | Single pass over pixels. |
| `compute_spectral_indices` | `O(N * I)` | `I` = number of requested indices. |
| `sample_patch_means` | `O(P * A)` | `P` patches; `A` pixels per patch ROI. |
| `fit_linear_reflectance` | `O(P * C)` | `C` channels (usually 3). |
| `detect_aruco_fiducial` | `O(N)` | Runs on the detection image (downsampled if configured). |

`N` = number of pixels in the processed image.

## Spectral indices

```{autofunction} mapir_camera_ros2.core.spectral_indices.compute_spectral_indices
```

```{autofunction} mapir_camera_ros2.core.spectral_indices.supported_spectral_indices
```

```{autofunction} mapir_camera_ros2.core.spectral_indices.preset_band_channels
```

```{autofunction} mapir_camera_ros2.core.spectral_indices.extract_bands_from_bgr
```

```{autoclass} mapir_camera_ros2.core.spectral_indices.SpectralIndexParams
:members:
```

## V4L2/OpenCV helpers

```{autofunction} mapir_camera_ros2.core.v4l2_camera.open_v4l2_capture
```

```{autofunction} mapir_camera_ros2.core.v4l2_camera.configure_v4l2_capture
```

```{autoclass} mapir_camera_ros2.core.v4l2_camera.V4L2Negotiation
:members:
```

## Survey3 RAW utilities

```{autofunction} mapir_camera_ros2.core.raw_survey3.decode_survey3_raw12
```

```{autofunction} mapir_camera_ros2.core.raw_survey3.load_survey3_raw
```

```{autofunction} mapir_camera_ros2.core.raw_survey3.demosaic_survey3_raw
```

```{autofunction} mapir_camera_ros2.core.raw_survey3.survey3_raw_to_rgb
```

```{autofunction} mapir_camera_ros2.core.raw_survey3.apply_mapir_color_correction
```

## Vignette correction

```{autofunction} mapir_camera_ros2.core.vignette_correction.load_vignette_images
```

```{autofunction} mapir_camera_ros2.core.vignette_correction.apply_vignette_correction
```
