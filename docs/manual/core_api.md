# Core API Reference

## Spectral indices

```{autofunction} mapir_camera_core.spectral_indices.compute_spectral_indices
```

```{autofunction} mapir_camera_core.spectral_indices.supported_spectral_indices
```

```{autofunction} mapir_camera_core.spectral_indices.preset_band_channels
```

```{autofunction} mapir_camera_core.spectral_indices.extract_bands_from_bgr
```

```{autoclass} mapir_camera_core.spectral_indices.SpectralIndexParams
:members:
```

## V4L2/OpenCV helpers

```{autofunction} mapir_camera_core.v4l2_camera.open_v4l2_capture
```

```{autofunction} mapir_camera_core.v4l2_camera.configure_v4l2_capture
```

```{autoclass} mapir_camera_core.v4l2_camera.V4L2Negotiation
:members:
```

## Survey3 RAW utilities

```{autofunction} mapir_camera_core.raw_survey3.decode_survey3_raw12
```

```{autofunction} mapir_camera_core.raw_survey3.load_survey3_raw
```

```{autofunction} mapir_camera_core.raw_survey3.demosaic_survey3_raw
```

```{autofunction} mapir_camera_core.raw_survey3.survey3_raw_to_rgb
```

```{autofunction} mapir_camera_core.raw_survey3.apply_mapir_color_correction
```

## Vignette correction

```{autofunction} mapir_camera_core.vignette_correction.load_vignette_images
```

```{autofunction} mapir_camera_core.vignette_correction.apply_vignette_correction
```
