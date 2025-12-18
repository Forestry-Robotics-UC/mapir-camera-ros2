from mapir_camera_core import (
    compute_spectral_indices,
    extract_bands_from_bgr,
    preset_band_channels,
    SpectralIndexParams,
    supported_spectral_indices,
)

import numpy as np


def test_supported_indices_contains_expected_entries():
    supported = supported_spectral_indices()
    expected = {
        'ndvi',
        'ndre',
        'gndvi',
        'evi',
        'vari',
        'gli',
        'msavi2',
        'fci1',
        'fci2',
        'lci',
    }
    assert expected.issubset(supported)


def test_extract_bands_from_bgr_uint8_normalizes_to_float32():
    img = np.zeros((2, 3, 3), dtype=np.uint8)
    img[:, :, 0] = 10
    img[:, :, 1] = 20
    img[:, :, 2] = 30

    bands = extract_bands_from_bgr(
        img,
        {'blue': 0, 'green': 1, 'red': 2},
        normalize=True,
    )
    assert bands['blue'].dtype == np.float32
    assert bands['green'].dtype == np.float32
    assert bands['red'].dtype == np.float32

    assert np.isclose(bands['blue'][0, 0], 10.0 / 255.0)
    assert np.isclose(bands['green'][0, 0], 20.0 / 255.0)
    assert np.isclose(bands['red'][0, 0], 30.0 / 255.0)


def test_preset_band_channels_ocn_maps_cyan_to_blue_and_orange_to_red():
    mapping = preset_band_channels('OCN')
    assert mapping['blue'] == 0
    assert mapping['cyan'] == 0
    assert mapping['red'] == 1
    assert mapping['orange'] == 1
    assert mapping['nir1'] == 2


def test_ndvi_defaults_to_nir2_when_available():
    red = np.array([[0.2, 0.3], [0.4, 0.5]], dtype=np.float32)
    nir1 = np.array([[0.9, 0.9], [0.9, 0.9]], dtype=np.float32)
    nir2 = np.array([[0.6, 0.7], [0.8, 0.9]], dtype=np.float32)

    out = compute_spectral_indices(
        ['ndvi'],
        {'red': red, 'nir1': nir1, 'nir2': nir2},
    )
    expected = (nir2 - red) / (nir2 + red)
    assert np.allclose(out['ndvi'], expected)


def test_ndvi_suffix_selects_nir1_or_nir2():
    red = np.array([[0.2, 0.3], [0.4, 0.5]], dtype=np.float32)
    nir1 = np.array([[0.9, 0.9], [0.9, 0.9]], dtype=np.float32)
    nir2 = np.array([[0.6, 0.7], [0.8, 0.9]], dtype=np.float32)

    out = compute_spectral_indices(
        ['ndvi_1', 'ndvi_2'],
        {'red': red, 'nir1': nir1, 'nir2': nir2},
    )

    expected_1 = (nir1 - red) / (nir1 + red)
    expected_2 = (nir2 - red) / (nir2 + red)

    assert np.allclose(out['ndvi_1'], expected_1)
    assert np.allclose(out['ndvi_2'], expected_2)


def test_evi_matches_formula():
    blue = np.array([[0.1, 0.2], [0.3, 0.4]], dtype=np.float32)
    red = np.array([[0.2, 0.2], [0.2, 0.2]], dtype=np.float32)
    nir2 = np.array([[0.6, 0.6], [0.6, 0.6]], dtype=np.float32)

    out = compute_spectral_indices(
        ['evi'],
        {'blue': blue, 'red': red, 'nir2': nir2},
    )

    expected = 2.5 * (nir2 - red) / (nir2 + 6.0 * red - 7.5 * blue + 1.0)
    assert np.allclose(out['evi'], expected)


def test_fci1_does_not_require_nir():
    red = np.array([[0.2, 0.3], [0.4, 0.5]], dtype=np.float32)
    rededge = np.array([[0.1, 0.1], [0.2, 0.2]], dtype=np.float32)

    out = compute_spectral_indices(
        ['fci1'],
        {'red': red, 'rededge': rededge},
    )
    assert np.allclose(out['fci1'], red * rededge)


def test_msavi2_is_finite():
    red = np.array([[0.2, 0.3], [0.4, 0.5]], dtype=np.float32)
    nir2 = np.array([[0.6, 0.7], [0.8, 0.9]], dtype=np.float32)

    out = compute_spectral_indices(
        ['msavi2'],
        {'red': red, 'nir2': nir2},
    )
    assert np.isfinite(out['msavi2']).all()


def test_wdrvi_alpha_parameter_changes_output():
    red = np.array([[0.2, 0.3], [0.4, 0.5]], dtype=np.float32)
    nir2 = np.array([[0.6, 0.7], [0.8, 0.9]], dtype=np.float32)

    out_a = compute_spectral_indices(
        ['wdrvi'],
        {'red': red, 'nir2': nir2},
        params=SpectralIndexParams(wdrvi_alpha=0.1),
    )
    out_b = compute_spectral_indices(
        ['wdrvi'],
        {'red': red, 'nir2': nir2},
        params=SpectralIndexParams(wdrvi_alpha=0.2),
    )

    assert not np.allclose(out_a['wdrvi'], out_b['wdrvi'])
