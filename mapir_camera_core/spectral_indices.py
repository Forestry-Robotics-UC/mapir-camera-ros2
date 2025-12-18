from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final

import numpy as np


class MissingBandError(ValueError):
    """Raised when a requested index cannot be computed due to missing bands."""


_SUPPORTED_INDICES: Final[set[str]] = {
    'evi',
    'fci1',
    'fci2',
    'gemi',
    'gari',
    'gci',
    'gli',
    'gndvi',
    'gosavi',
    'grvi',
    'gsavi',
    'lai',
    'lci',
    'mnli',
    'msavi2',
    'ndre',
    'ndvi',
    'nli',
    'osavi',
    'rdvi',
    'savi',
    'tdvi',
    'vari',
    'wdrvi',
}


def supported_spectral_indices() -> set[str]:
    """Return the set of supported spectral index names (lowercase, no suffix)."""
    return set(_SUPPORTED_INDICES)


def preset_band_channels(filter_set: str) -> dict[str, int]:
    """
    Return a best-effort band→BGR-channel mapping for common Survey3 filter sets.

    Assumption: the camera outputs a 3-channel image whose channels are ordered
    by increasing wavelength (shortest→longest) mapped to B, G, R respectively.

    Presets:
      - RGB: blue=B, green=G, red=R
      - NGB: blue=B, green=G, nir2=R
      - RGN: green=B, red=G, nir2=R
      - OCN: cyan=B, orange=G, nir1=R (and aliases cyan→blue, orange→red)

    If your camera uses a different mapping, override explicitly in the ROS node.
    """
    preset = filter_set.strip().lower()
    if preset in ('rgb', 'bgr'):
        return {'blue': 0, 'green': 1, 'red': 2}
    if preset == 'ngb':
        return {'blue': 0, 'green': 1, 'nir2': 2}
    if preset == 'rgn':
        return {'green': 0, 'red': 1, 'nir2': 2}
    if preset == 'ocn':
        # Common MAPIR convention for vegetation indices:
        # - treat Cyan as "Blue"
        # - treat Orange as "Red"
        return {
            'blue': 0,
            'cyan': 0,
            'red': 1,
            'orange': 1,
            'nir1': 2,
        }

    return {}


def _parse_index_name(name: str) -> tuple[str, str | None]:
    index = name.strip().lower()
    if not index:
        raise ValueError('Empty index name')

    if index.endswith('_1'):
        return index[:-2], '1'
    if index.endswith('_2'):
        return index[:-2], '2'
    return index, None


def _choose_nir_key(bands: Mapping[str, np.ndarray], nir_suffix: str | None) -> str:
    if nir_suffix == '1':
        if 'nir1' in bands:
            return 'nir1'
        if 'nir' in bands:
            return 'nir'
        raise MissingBandError('Requested NIR1 but no nir1/nir band is available')

    if nir_suffix == '2':
        if 'nir2' in bands:
            return 'nir2'
        if 'nir' in bands:
            return 'nir'
        raise MissingBandError('Requested NIR2 but no nir2/nir band is available')

    if 'nir2' in bands:
        return 'nir2'
    if 'nir1' in bands:
        return 'nir1'
    if 'nir' in bands:
        return 'nir'
    raise MissingBandError('Requested an index needing NIR but no nir/nir1/nir2 band is available')


def _require(bands: Mapping[str, np.ndarray], band_name: str) -> np.ndarray:
    try:
        return bands[band_name]
    except KeyError as exc:
        raise MissingBandError(f'Missing required band {band_name!r}') from exc


def _safe_divide(numer: np.ndarray, denom: np.ndarray, *, eps: float) -> np.ndarray:
    out = np.zeros_like(numer, dtype=np.float32)
    np.divide(numer, denom, out=out, where=np.abs(denom) > eps)
    return out


@dataclass(frozen=True)
class SpectralIndexParams:
    eps: float = 1e-6
    gari_gamma: float = 1.7
    wdrvi_alpha: float = 0.2
    mnli_L: float = 0.5


def compute_spectral_indices(
    requested: Sequence[str],
    bands: Mapping[str, np.ndarray],
    *,
    params: SpectralIndexParams = SpectralIndexParams(),
    on_missing: str = 'skip',
) -> dict[str, np.ndarray]:
    """
    Compute requested spectral indices.

    Names are case-insensitive. If an entry ends in `_1` or `_2`, the function
    prefers `nir1` or `nir2` respectively. Input bands should be float32 images
    (typically scaled to [0, 1]).
    """
    if on_missing not in ('skip', 'raise'):
        raise ValueError('on_missing must be "skip" or "raise"')

    cache: dict[tuple[str, str | None], np.ndarray] = {}
    out: dict[str, np.ndarray] = {}

    def compute_one(name: str) -> np.ndarray:
        base, nir_suffix = _parse_index_name(name)
        key = (base, nir_suffix)
        if key in cache:
            return cache[key]

        if base not in _SUPPORTED_INDICES:
            supported_indices = sorted(_SUPPORTED_INDICES)
            raise ValueError(f'Unsupported index {name!r}. Supported: {supported_indices}')

        needs_nir = base not in ('fci1', 'gli', 'vari')
        nir_key = _choose_nir_key(bands, nir_suffix) if needs_nir else ''

        blue = None
        green = None
        red = None
        rededge = None
        nir = None

        try:
            if base in ('evi', 'gari', 'gli', 'vari'):
                blue = _require(bands, 'blue')
            if base in ('gari', 'gci', 'gli', 'gndvi', 'gosavi', 'grvi', 'gsavi', 'vari'):
                green = _require(bands, 'green')
            if base in (
                'evi',
                'fci1',
                'fci2',
                'gemi',
                'gari',
                'gli',
                'lai',
                'lci',
                'mnli',
                'msavi2',
                'ndvi',
                'nli',
                'osavi',
                'rdvi',
                'savi',
                'tdvi',
                'vari',
                'wdrvi',
            ):
                red = _require(bands, 'red')
            if base in ('fci1', 'lci', 'ndre'):
                rededge = _require(bands, 'rededge')
            if needs_nir:
                nir = _require(bands, nir_key)

            idx = _compute_index(
                base,
                blue=blue,
                green=green,
                red=red,
                rededge=rededge,
                nir=nir,
                params=params,
            )
        except MissingBandError:
            if on_missing == 'raise':
                raise
            idx = None

        if idx is None:
            raise MissingBandError('Missing bands for requested index')

        cache[key] = idx
        return idx

    for name in requested:
        norm = name.strip().lower()
        if not norm:
            continue
        try:
            out[norm] = compute_one(norm)
        except MissingBandError:
            if on_missing == 'raise':
                raise
            continue

    return out


def _compute_index(
    base: str,
    *,
    blue: np.ndarray | None,
    green: np.ndarray | None,
    red: np.ndarray | None,
    rededge: np.ndarray | None,
    nir: np.ndarray | None,
    params: SpectralIndexParams,
) -> np.ndarray:
    eps = float(params.eps)

    if base == 'ndvi':
        assert nir is not None and red is not None
        return _safe_divide(nir - red, nir + red, eps=eps)

    if base == 'ndre':
        assert nir is not None and rededge is not None
        return _safe_divide(nir - rededge, nir + rededge, eps=eps)

    if base == 'osavi':
        assert nir is not None and red is not None
        denom = nir + red + np.float32(0.16)
        return _safe_divide(nir - red, denom, eps=eps)

    if base == 'savi':
        assert nir is not None and red is not None
        denom = nir + red + np.float32(0.5)
        return np.float32(1.5) * _safe_divide(nir - red, denom, eps=eps)

    if base == 'rdvi':
        assert nir is not None and red is not None
        denom = np.sqrt(np.maximum(nir + red, np.float32(0.0)))
        return _safe_divide(nir - red, denom, eps=eps)

    if base == 'tdvi':
        assert nir is not None and red is not None
        denom = np.sqrt(np.maximum(nir * nir + red + np.float32(0.5), np.float32(0.0)))
        return np.float32(1.5) * _safe_divide(nir - red, denom, eps=eps)

    if base == 'wdrvi':
        assert nir is not None and red is not None
        a = np.float32(params.wdrvi_alpha)
        numer = a * nir - red
        denom = a * nir + red
        return _safe_divide(numer, denom, eps=eps)

    if base == 'evi':
        assert nir is not None and red is not None and blue is not None
        numer = nir - red
        denom = nir + np.float32(6.0) * red - np.float32(7.5) * blue + np.float32(1.0)
        return np.float32(2.5) * _safe_divide(numer, denom, eps=eps)

    if base == 'lai':
        assert nir is not None and red is not None and blue is not None
        evi = _compute_index(
            'evi',
            blue=blue,
            green=green,
            red=red,
            rededge=rededge,
            nir=nir,
            params=params,
        )
        return np.float32(3.618) * evi - np.float32(0.118)

    if base == 'gndvi':
        assert nir is not None and green is not None
        return _safe_divide(nir - green, nir + green, eps=eps)

    if base == 'gosavi':
        assert nir is not None and green is not None
        denom = nir + green + np.float32(0.16)
        return _safe_divide(nir - green, denom, eps=eps)

    if base == 'gsavi':
        assert nir is not None and green is not None
        denom = nir + green + np.float32(0.5)
        return np.float32(1.5) * _safe_divide(nir - green, denom, eps=eps)

    if base == 'gci':
        assert nir is not None and green is not None
        return _safe_divide(nir, green, eps=eps) - np.float32(1.0)

    if base == 'grvi':
        assert nir is not None and green is not None
        return _safe_divide(nir, green, eps=eps)

    if base == 'gari':
        assert nir is not None and green is not None and blue is not None and red is not None
        gamma = np.float32(params.gari_gamma)
        x = green - gamma * (blue - red)
        return _safe_divide(nir - x, nir + x, eps=eps)

    if base == 'gli':
        assert green is not None and red is not None and blue is not None
        numer = np.float32(2.0) * green - red - blue
        denom = np.float32(2.0) * green + red + blue
        return _safe_divide(numer, denom, eps=eps)

    if base == 'vari':
        assert green is not None and red is not None and blue is not None
        denom = green + red - blue
        return _safe_divide(green - red, denom, eps=eps)

    if base == 'fci1':
        assert red is not None and rededge is not None
        return red * rededge

    if base == 'fci2':
        assert red is not None and nir is not None
        return red * nir

    if base == 'gemi':
        assert nir is not None and red is not None
        eta_num = (
            np.float32(2.0) * (nir * nir - red * red)
            + np.float32(1.5) * nir
            + np.float32(0.5) * red
        )
        eta_den = nir + red + np.float32(0.5)
        eta = _safe_divide(eta_num, eta_den, eps=eps)
        term1 = eta * (np.float32(1.0) - np.float32(0.25) * eta)
        term2 = _safe_divide(red - np.float32(0.125), np.float32(1.0) - red, eps=eps)
        return term1 - term2

    if base == 'lci':
        assert nir is not None and rededge is not None and red is not None
        denom = nir + red
        return _safe_divide(nir - rededge, denom, eps=eps)

    if base == 'nli':
        assert nir is not None and red is not None
        nir2 = nir * nir
        return _safe_divide(nir2 - red, nir2 + red, eps=eps)

    if base == 'mnli':
        assert nir is not None and red is not None
        mnli_L = np.float32(params.mnli_L)
        nir2 = nir * nir
        numer = (nir2 - red) * (np.float32(1.0) + mnli_L)
        denom = nir2 + red + mnli_L
        return _safe_divide(numer, denom, eps=eps)

    if base == 'msavi2':
        assert nir is not None and red is not None
        a = np.float32(2.0) * nir + np.float32(1.0)
        radicand = a * a - np.float32(8.0) * (nir - red)
        radicand = np.maximum(radicand, np.float32(0.0))
        return (a - np.sqrt(radicand)) / np.float32(2.0)

    raise ValueError(f'Unhandled supported index {base!r}')


def extract_bands_from_bgr(
    image_bgr: np.ndarray,
    band_channels: Mapping[str, int],
    *,
    normalize: bool = True,
) -> dict[str, np.ndarray]:
    """
    Extract named float32 bands from a BGR (or mono) image.

    `band_channels` maps band names to channel indices (0=B, 1=G, 2=R). If
    `normalize` is True and the input is an integer dtype, values are scaled to
    [0, 1] by dividing by the dtype max.
    """
    if image_bgr.ndim not in (2, 3):
        raise ValueError(f'Expected 2D or 3D image, got shape={image_bgr.shape}')

    img = image_bgr
    if np.issubdtype(img.dtype, np.integer):
        img = img.astype(np.float32)
        if normalize:
            img = img / np.float32(np.iinfo(image_bgr.dtype).max)
    else:
        img = img.astype(np.float32, copy=False)

    out: dict[str, np.ndarray] = {}

    if img.ndim == 2:
        for band_name, channel in band_channels.items():
            if channel != 0:
                raise ValueError(f'Mono image cannot provide {band_name!r} from channel {channel}')
            out[band_name] = img
        return out

    if img.shape[2] < 3:
        raise ValueError(f'Expected at least 3 channels, got shape={img.shape}')

    for band_name, channel in band_channels.items():
        if channel not in (0, 1, 2):
            raise ValueError(f'Invalid channel index for {band_name!r}: {channel}')
        out[band_name] = img[:, :, channel]

    return out
