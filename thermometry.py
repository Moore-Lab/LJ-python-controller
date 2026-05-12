"""Thermometry channels for the LabJack T7.

The T7's AIN extended features do the heavy lifting:

  * RTD channels are configured with ``AINx_EF_INDEX = 40`` (PT100), so the
    device reports temperature, resistance and excitation voltage directly via
    ``AINx_EF_READ_A`` (K), ``AINx_EF_READ_B`` (ohms) and ``AINx_EF_READ_C``
    (volts).

  * Thermocouple channels are configured with ``AINx_EF_INDEX = 22`` (type K),
    but they are wired as differential *gradiometers* (a small ΔEMF between two
    closely-spaced sense points), so the device's internal-CJC temperature
    (``AINx_EF_READ_A``) is not a useful absolute value.  We take the measured
    thermocouple voltage (``AINx_EF_READ_B``) and re-reference it to a nearby
    RTD: T = invK( fwdK(T_ref) + V_measured ).  Because the ΔT is small, either
    end of the thermocouple sits within a few K of the reference, so this gives
    a sensible absolute temperature for comparison against standard curves.

Only this module needs the ITS-90 type-K polynomials; the PT100 conversion is
left entirely to the T7.
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# ITS-90 type-K thermocouple reference functions (°C ↔ mV)
# Coefficients from NIST Monograph 175 / the ITS-90 thermocouple tables.
# ---------------------------------------------------------------------------

# E(T) [mV], T in °C
_FWD_K_NEG = (  # -270 °C … 0 °C
    0.0,
    0.394501280250e-1,
    0.236223735980e-4,
    -0.328589067840e-6,
    -0.499048287770e-8,
    -0.675090591730e-10,
    -0.574103274280e-12,
    -0.310888728940e-14,
    -0.104516093650e-16,
    -0.198892668780e-19,
    -0.163226974860e-22,
)
_FWD_K_POS = (  # 0 °C … 1372 °C  (plus the exponential term below)
    -0.176004136860e-1,
    0.389212049750e-1,
    0.185587700320e-4,
    -0.994575928740e-7,
    0.318409457190e-9,
    -0.560728448890e-12,
    0.560750590590e-15,
    -0.320207200030e-18,
    0.971511471520e-22,
    -0.121047212750e-25,
)
_FWD_K_POS_EXP = (0.118597600000e0, -0.118343200000e-3, 0.126968600000e3)  # a0, a1, a2

# T(E) [°C], E in mV
_INV_K_LOW = (  # -5.891 mV … 0 mV   (-200 °C … 0 °C)
    0.0,
    2.5173462e1,
    -1.1662878e0,
    -1.0833638e0,
    -8.9773540e-1,
    -3.7342377e-1,
    -8.6632643e-2,
    -1.0450598e-2,
    -5.1920577e-4,
)
_INV_K_MID = (  # 0 mV … 20.644 mV   (0 °C … 500 °C)
    0.0,
    2.508355e1,
    7.860106e-2,
    -2.503131e-1,
    8.315270e-2,
    -1.228034e-2,
    9.804036e-4,
    -4.413030e-5,
    1.057734e-6,
    -1.052755e-8,
)


def _poly(coeffs, x: float) -> float:
    acc = 0.0
    for c in reversed(coeffs):
        acc = acc * x + c
    return acc


def typek_emf_mv(temp_c: float) -> float:
    """Type-K thermocouple EMF [mV] for a junction at ``temp_c`` (cold side at 0 °C)."""
    if temp_c < 0.0:
        return _poly(_FWD_K_NEG, temp_c)
    a0, a1, a2 = _FWD_K_POS_EXP
    return _poly(_FWD_K_POS, temp_c) + a0 * math.exp(a1 * (temp_c - a2) ** 2)


def typek_temp_c(emf_mv: float) -> float:
    """Inverse type-K: junction temperature [°C] for an EMF of ``emf_mv`` (cold side 0 °C)."""
    if emf_mv < 0.0:
        # Valid -5.891 .. 0 mV; clamp gently below that (cryogenic over-range).
        return _poly(_INV_K_LOW, max(emf_mv, -5.891))
    if emf_mv <= 20.644:
        return _poly(_INV_K_MID, emf_mv)
    # Above ~500 °C — not expected here; fall back to the mid polynomial.
    return _poly(_INV_K_MID, emf_mv)


def referenced_typek_temp_c(emf_mv: float, reference_c: float) -> float:
    """Absolute temperature [°C] of a type-K gradiometer end whose measured ΔEMF
    is ``emf_mv`` and whose other end (the reference) is at ``reference_c``."""
    return typek_temp_c(typek_emf_mv(reference_c) + emf_mv)


KELVIN = 273.15
