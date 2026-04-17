from mapir_camera_core import apply_vignette_correction

import numpy as np


def test_apply_vignette_correction_handles_zero_flatfield_values():
    image = np.array(
        [
            [[10, 20, 30], [40, 50, 60]],
            [[70, 80, 90], [100, 110, 120]],
        ],
        dtype=np.uint8,
    )
    flat_b = np.array([[0.0, 1.0], [2.0, 0.0]], dtype=np.float32)
    flat_g = np.array([[1.0, 0.0], [2.0, 1.0]], dtype=np.float32)
    flat_r = np.array([[1.0, 2.0], [0.0, 1.0]], dtype=np.float32)

    out = apply_vignette_correction(
        image,
        (flat_b, flat_g, flat_r),
        dark_current=(0, 0, 0),
    )

    assert out.dtype == np.uint8
    assert np.isfinite(out.astype(np.float32)).all()


def test_apply_vignette_correction_rejects_mismatched_flatfield_shape():
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    flat = np.ones((2, 2), dtype=np.float32)

    try:
        apply_vignette_correction(image, (flat, flat, flat))
    except ValueError as exc:
        assert 'must match the input image shape' in str(exc)
        return

    raise AssertionError('Expected ValueError for shape mismatch')
