# Multispectral Indices

The indices math lives in `mapir_camera_ros2/core/spectral_indices.py` and is
published by the ROS node `mapir_camera_ros2.nodes.indices_node`.
For real-time performance, keep the indices list to 1-2 entries.

Outputs:

- Float index image: `/<ns>/indices/<index>` (`sensor_msgs/Image`, encoding `32FC1`)
- Optional colorized image: `/<ns>/indices_color/<index>` (`sensor_msgs/Image`, encoding `bgr8`)

Configuration:

- Edit `config/mapir_indices_params.yaml` (installed with the package) for
  `indices`, `filter_set`, and performance tuning parameters.

## Survey3 bands (reference)

These names follow MAPIR Survey3 filter naming conventions (average transmission
shown for convenience):

| Band name | Survey3 filter | Approx. center |
|---|---|---|
| Blue | NGB - Blue | ~475 nm |
| Cyan | OCN - Cyan | ~494 nm |
| Green | RGN/NGB - Green | ~547 nm |
| Orange | OCN - Orange | ~619 nm |
| Red | RGN - Red | ~661 nm |
| RedEdge | Re - RedEdge | ~724 nm |
| NIR1 | OCN - NIR1 | ~823 nm |
| NIR2 | RGN/NGB/NIR - NIR2 | ~850 nm |

## Supported indices

Indices are requested by lowercase name (case-insensitive):

`evi`, `fci1`, `fci2`, `gemi`, `gari`, `gci`, `gli`, `gndvi`, `gosavi`, `grvi`,
`gsavi`, `lai`, `lci`, `mnli`, `msavi2`, `ndre`, `ndvi`, `nli`, `osavi`,
`rdvi`, `savi`, `tdvi`, `vari`, `wdrvi`.

## NIR selection

If you provide both `nir1` and `nir2`, the default preference is `nir2`.
You can force the choice by suffixing the index name:

- `ndvi_1` uses `nir1` if available
- `ndvi_2` uses `nir2` if available

## Filter-set compatibility (Survey3 3-band streams)

Survey3 cameras output 3 bands per stream. Some indices require bands that are
not available together on a given 3-band filter set.

Common presets used by this package:

| Filter set | Available bands (logical) | Notes |
|---|---|---|
| RGB | Blue, Green, Red | No NIR → no vegetation indices like NDVI. |
| RGN | Green, Red, NIR2 | Common NDVI-like workflow. |
| NGB | Blue, Green, NIR2 | No Red → NDVI not directly available. |
| OCN | Cyan, Orange, NIR1 | No Green/RedEdge. This package aliases Cyan→Blue and Orange→Red. |

## Colormap publishing (low → high)

Enable colorized outputs with:

- `publish_color: true`
- `colormap: viridis` (or `jet`, `gray`, `custom`, ...)
- `colorize_min` / `colorize_max` define the numeric range mapped to the palette.

Interpretation:

- `colorize_min` maps to the *low* end of the palette.
- `colorize_max` maps to the *high* end of the palette.

Examples (typical perception):

- `viridis`: low≈dark purple, high≈yellow
- `jet`: low≈dark blue, high≈red
- `gray`: low=black, high=white

Custom colormap:

- set `colormap: custom`
- provide `custom_colormap` points:
  - `'value,r,g,b; value,r,g,b; ...'`
  - example: `'-1,0,0,0; 0,0,255,0; 1,255,255,255'`

NaN/inf values are rendered as black.

## Index guide (meaning, usage, and bands)

### NDVI — Normalized Difference Vegetation Index

- Meaning: classic greenness / vegetation vigor proxy based on chlorophyll red absorption vs NIR reflectance.
- Best used: general vegetation presence and relative vigor; can saturate in dense canopy/high LAI.
- Bands: NIR + Red.
- Survey3 filter sets: RGN (Red+NIR2), OCN (Orange→Red + NIR1). Not directly supported on NGB/RGB.
- Reference: Rouse et al. (1973).[^ndvi]

### NDRE — Normalized Difference RedEdge

- Meaning: red-edge variant of NDVI; often more sensitive to chlorophyll and stress earlier than NDVI.
- Best used: moderate-to-dense vegetation where NDVI saturates; red-edge enabled cameras.
- Bands: NIR + RedEdge.
- Survey3 filter sets: requires a RedEdge band in the 3-band stream (not present in default RGN/NGB/OCN/RGB presets).
- Reference: related red-edge chlorophyll work in Datt (1999).[^lci]

### EVI — Enhanced Vegetation Index

- Meaning: NDVI-like index with coefficients designed to reduce soil/background effects and atmospheric influences.
- Best used: high-LAI regions where NDVI saturates; requires a Blue band.
- Bands: NIR + Red + Blue.
- Survey3 filter sets: OCN (Cyan→Blue + Orange→Red + NIR1). Not directly supported on RGN/NGB/RGB presets.
- Reference: Huete et al. (2002).[^evi]

### LAI — Leaf Area Index (derived)

- Meaning: empirical LAI estimate derived from EVI.
- Best used: coarse LAI proxy; treat as model-dependent and validate for your crop/biome.
- Bands: same as EVI (NIR + Red + Blue).
- Survey3 filter sets: OCN (via EVI).
- Reference: Boegh et al. (2002).[^lai]

### SAVI — Soil Adjusted Vegetation Index

- Meaning: NDVI-like with soil brightness correction parameter (L=0.5 here).
- Best used: sparse vegetation with visible soil background.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Huete (1988).[^savi]

### OSAVI — Optimized Soil Adjusted Vegetation Index

- Meaning: SAVI variant with fixed canopy background adjustment (0.16).
- Best used: sparse vegetation where soil is visible; often preferred over SAVI when L is unknown.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Rondeaux et al. (1996).[^osavi]

### MSAVI2 — Modified Soil Adjusted Vegetation Index 2

- Meaning: SAVI family index designed to reduce soil noise without a fixed L parameter.
- Best used: scenes with mixed vegetation/soil; improves dynamic range over NDVI in low cover.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Qi et al. (1994).[^msavi2]

### RDVI — Renormalized Difference Vegetation Index

- Meaning: combines NDVI-style difference with normalization to reduce some sensitivity to illumination/soil.
- Best used: vegetation monitoring with reduced soil influence compared to simple ratios.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Roujean & Breon (1995).[^rdvi]

### TDVI — Transformed Difference Vegetation Index

- Meaning: transformed NDVI family index designed to avoid saturation in some environments.
- Best used: vegetation mapping in heterogeneous scenes (including urban vegetation).
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Bannari et al. (2002).[^tdvi]

### WDRVI — Wide Dynamic Range Vegetation Index

- Meaning: NDVI-like index with an `alpha` coefficient to reduce NIR dominance and extend sensitivity when NDVI is high.
- Best used: moderate-to-high vegetation density where NDVI > ~0.6.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Gitelson (2004); Henebry et al. (2004).[^wdrvi][^wdrvi2]

### NLI — Non-Linear Index

- Meaning: non-linear transform intended to linearize relationships with canopy parameters.
- Best used: when NDVI-like linearity is insufficient; requires validation for your domain.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Goel & Qin (1994).[^nli]

### MNLI — Modified Non-Linear Index

- Meaning: NLI variant incorporating soil adjustment factor (L=0.5 here).
- Best used: mixed soil/vegetation scenes; requires domain validation.
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Yang et al. (2008).[^mnli]

### GEMI — Global Environmental Monitoring Index

- Meaning: non-linear vegetation index designed to reduce atmospheric effects relative to NDVI.
- Best used: global monitoring; can be affected by bare soil (not recommended for sparse vegetation).
- Bands: NIR + Red.
- Survey3 filter sets: RGN, OCN.
- Reference: Pinty & Verstraete (1992).[^gemi]

### FCI1 — Forest Cover Index 1

- Meaning: distinguishes forest canopies from other vegetation using a red-edge band.
- Best used: forest/non-forest separation when a RedEdge band is available.
- Bands: Red + RedEdge.
- Survey3 filter sets: requires a RedEdge band in the 3-band stream.
- Reference: Becker et al. (2018).[^fci]

### FCI2 — Forest Cover Index 2

- Meaning: forest cover separation without RedEdge, using NIR instead.
- Best used: forest/non-forest separation without a RedEdge band (still requires NIR+Red).
- Bands: Red + NIR.
- Survey3 filter sets: RGN, OCN.
- Reference: Becker et al. (2018).[^fci]

### GNDVI — Green Normalized Difference Vegetation Index

- Meaning: NDVI variant using Green instead of Red; often more sensitive to chlorophyll concentration.
- Best used: chlorophyll-related changes, especially when Red saturates or is not ideal.
- Bands: NIR + Green.
- Survey3 filter sets: RGN (Green+NIR2), NGB (Green+NIR2). Not supported on OCN/RGB presets.
- Reference: Gitelson & Merzlyak (1998).[^gndvi]

### GCI — Green Chlorophyll Index

- Meaning: ratio-based chlorophyll proxy (NIR/Green - 1).
- Best used: estimating chlorophyll content across a wide range of species (requires calibration for absolute).
- Bands: NIR + Green.
- Survey3 filter sets: RGN, NGB.
- Reference: Gitelson et al. (2003).[^gci]

### GRVI — Green Ratio Vegetation Index

- Meaning: ratio NIR/Green; sensitive to pigment-driven changes.
- Best used: relative changes in photosynthetic activity / canopy vigor.
- Bands: NIR + Green.
- Survey3 filter sets: RGN, NGB.
- Reference: Sripada et al. (2006).[^grvi]

### GOSAVI — Green Optimized Soil Adjusted Vegetation Index

- Meaning: OSAVI-like soil-adjusted index using Green instead of Red.
- Best used: nitrogen/crop vigor proxies in workflows designed around green/NIR.
- Bands: NIR + Green.
- Survey3 filter sets: RGN, NGB.
- Reference: Sripada et al. (2005).[^gosavi]

### GSAVI — Green Soil Adjusted Vegetation Index

- Meaning: SAVI-like soil-adjusted index using Green instead of Red.
- Best used: soil-heavy scenes where Green is preferred for agronomic reasons.
- Bands: NIR + Green.
- Survey3 filter sets: RGN, NGB.
- Reference: Sripada et al. (2005).[^gsavi]

### GLI — Green Leaf Index

- Meaning: RGB-only greenness index originally designed for digital RGB DNs.
- Best used: simple green cover estimation when only RGB is available.
- Bands: Green + Red + Blue.
- Survey3 filter sets: RGB only.
- Reference: Louhaichi et al. (2001).[^gli]

### VARI — Visible Atmospherically Resistant Index

- Meaning: visible-band vegetation fraction proxy with reduced sensitivity to atmospheric effects.
- Best used: RGB-only vegetation fraction estimation in visible imagery.
- Bands: Green + Red + Blue.
- Survey3 filter sets: RGB only.
- Reference: Gitelson et al. (2002).[^vari]

### GARI — Green Atmospherically Resistant Index

- Meaning: NDVI-like but uses a green channel with an aerosol-weighting term to reduce atmospheric sensitivity.
- Best used: multi-band (NIR+Red+Green+Blue) sensors; not generally feasible with 3-band Survey3 streams.
- Bands: NIR + Green + Blue + Red (4 unique bands).
- Survey3 filter sets: not directly supported on 3-band presets.
- Reference: Gitelson et al. (1996).[^gari]

### LCI — Leaf Chlorophyll Index

- Meaning: red-edge chlorophyll index sensitive to chlorophyll absorption around the red edge.
- Best used: chlorophyll estimation with a RedEdge band.
- Bands: NIR + RedEdge + Red.
- Survey3 filter sets: requires RedEdge+Red+NIR in the stream.
- Reference: Datt (1999).[^lci]

## References

[^evi]: Huete, A., et al. "Overview of the Radiometric and Biophysical Performance of the MODIS Vegetation Indices." *Remote Sensing of Environment* 83 (2002): 195–213.
[^fci]: Becker, Sarah J., Craig S.T. Daughtry, and Andrew L. Russ. "Robust forest cover indices for multispectral images." *Photogrammetric Engineering & Remote Sensing* 84.8 (2018): 505–512.
[^gemi]: Pinty, B., and M. Verstraete. "GEMI: a Non-Linear Index to Monitor Global Vegetation From Satellites." *Vegetation* 101 (1992): 15–20.
[^gari]: Gitelson, A., Y. Kaufman, and M. Merzylak. "Use of a Green Channel in Remote Sensing of Global Vegetation from EOS-MODIS." *Remote Sensing of Environment* 58 (1996): 289–298.
[^gci]: Gitelson, A., Y. Gritz, and M. Merzlyak. "Relationships Between Leaf Chlorophyll Content and Spectral Reflectance and Algorithms for Non-Destructive Chlorophyll Assessment in Higher Plant Leaves." *Journal of Plant Physiology* 160 (2003): 271–282.
[^gli]: Louhaichi, M., M. Borman, and D. Johnson. "Spatially Located Platform and Aerial Photography for Documentation of Grazing Impacts on Wheat." *Geocarto International* 16.1 (2001): 65–70.
[^gndvi]: Gitelson, A., and M. Merzlyak. "Remote Sensing of Chlorophyll Concentration in Higher Plant Leaves." *Advances in Space Research* 22 (1998): 689–692.
[^gosavi]: Sripada, R., et al. "Determining In-Season Nitrogen Requirements for Corn Using Aerial Color-Infrared Photography." Ph.D. dissertation, North Carolina State University, 2005.
[^grvi]: Sripada, R., et al. "Aerial Color Infrared Photography for Determining Early In-season Nitrogen Requirements in Corn." *Agronomy Journal* 98 (2006): 968–977.
[^gsavi]: Sripada, R., et al. "Determining In-Season Nitrogen Requirements for Corn Using Aerial Color-Infrared Photography." Ph.D. dissertation, North Carolina State University, 2005.
[^lai]: Boegh, E., et al. "Airborne Multi-spectral Data for Quantifying Leaf Area Index, Nitrogen Concentration and Photosynthetic Efficiency in Agriculture." *Remote Sensing of Environment* 81.2–3 (2002): 179–193.
[^lci]: Datt, B. "Remote Sensing of Water Content in Eucalyptus Leaves." *Journal of Plant Physiology* 154.1 (1999): 30–36.
[^mnli]: Yang, Z., P. Willis, and R. Mueller. "Impact of Band-Ratio Enhanced AWIFS Image to Crop Classification Accuracy." *Proceedings of the Pecora 17 Remote Sensing Symposium* (2008), Denver, CO.
[^msavi2]: Qi, J., et al. "A Modified Soil Adjusted Vegetation Index." *Remote Sensing of Environment* 48 (1994): 119–126.
[^ndvi]: Rouse, J., R. Haas, J. Schell, and D. Deering. "Monitoring Vegetation Systems in the Great Plains with ERTS." Third ERTS Symposium, NASA (1973): 309–317.
[^nli]: Goel, N., and W. Qin. "Influences of Canopy Architecture on Relationships Between Various Vegetation Indices and LAI and Fpar: A Computer Simulation." *Remote Sensing Reviews* 10 (1994): 309–347.
[^osavi]: Rondeaux, G., M. Steven, and F. Baret. "Optimization of Soil-Adjusted Vegetation Indices." *Remote Sensing of Environment* 55 (1996): 95–107.
[^rdvi]: Roujean, J., and F. Breon. "Estimating PAR Absorbed by Vegetation from Bidirectional Reflectance Measurements." *Remote Sensing of Environment* 51 (1995): 375–384.
[^savi]: Huete, A. "A Soil-Adjusted Vegetation Index (SAVI)." *Remote Sensing of Environment* 25 (1988): 295–309.
[^tdvi]: Bannari, A., H. Asalhi, and P. Teillet. "Transformed Difference Vegetation Index (TDVI) for Vegetation Cover Mapping." In *Proceedings of IGARSS '02* (2002).
[^vari]: Gitelson, A., et al. "Vegetation and Soil Lines in Visible Spectral Space: A Concept and Technique for Remote Estimation of Vegetation Fraction." *International Journal of Remote Sensing* 23 (2002): 2537–2562.
[^wdrvi]: Gitelson, A. "Wide Dynamic Range Vegetation Index for Remote Quantification of Biophysical Characteristics of Vegetation." *Journal of Plant Physiology* 161.2 (2004): 165–173.
[^wdrvi2]: Henebry, G., A. Viña, and A. Gitelson. "The Wide Dynamic Range Vegetation Index and its Potential Utility for Gap Analysis." *Gap Analysis Bulletin* 12 (2004): 50–56.
