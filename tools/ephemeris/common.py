"""Shared constants and helpers for the Helios ephemeris tools.

Conventions (match the C++ side):
  * time: TDB seconds since J2000 (JD 2451545.0 TDB)
  * axes: J2000 ecliptic (SPICE "ECLIPJ2000"), obtained from ICRF by a rotation of
    ε = 84381.448″ about +x
  * units: metres, metres per second
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

J2000_JD = 2451545.0
SECONDS_PER_DAY = 86400.0
KM_TO_M = 1000.0

OBLIQUITY_J2000_RAD = np.deg2rad(84381.448 / 3600.0)
_c, _s = np.cos(OBLIQUITY_J2000_RAD), np.sin(OBLIQUITY_J2000_RAD)
# ecliptic = R · icrf
ICRF_TO_ECLIPTIC = np.array([[1.0, 0.0, 0.0], [0.0, _c, _s], [0.0, -_s, _c]])

# NAIF ids used by the JPL DE kernels → Helios frame names. The Mercury, Venus and Mars
# barycentres coincide with the planet at this level (the 1→199 and 2→299 segments of DE440, and
# 4→499 where a kernel has it, are identically zero).
NAIF_NAMES = {
    0: "Solar System barycentre",
    1: "Mercury",
    2: "Venus",
    3: "Earth-Moon barycentre",
    4: "Mars",
    5: "Jupiter barycentre",
    6: "Saturn barycentre",
    7: "Uranus barycentre",
    8: "Neptune barycentre",
    9: "Pluto barycentre",
    10: "Sun",
    301: "Moon",
    399: "Earth",
}

# Parent-first order, as the C++ frame tree requires.
EXPORTED_SEGMENTS = [(0, 10), (0, 1), (0, 2), (0, 3), (3, 399), (3, 301), (0, 4),
                     (0, 5), (0, 6), (0, 7), (0, 8), (0, 9)]


def kernel_label(kernel_path) -> str:
    """'DE440' for '.../de440.bsp': how reports and data files name the source."""
    return Path(str(kernel_path)).stem.upper()


def kernel_coverage_years(kernel) -> tuple[float, float]:
    """The span every exported segment covers, as Julian years (2000.0 = J2000)."""
    start_jd = max(kernel[center, target].start_jd for center, target in EXPORTED_SEGMENTS)
    end_jd = min(kernel[center, target].end_jd for center, target in EXPORTED_SEGMENTS)
    return 2000.0 + (start_jd - J2000_JD) / 365.25, 2000.0 + (end_jd - J2000_JD) / 365.25


def seconds_to_jd_pair(whole_seconds: np.ndarray, fraction_s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split TDB seconds into (jd_whole, jd_fraction) the way jplephem expects, without losing precision."""
    whole_days, remainder_s = np.divmod(whole_seconds.astype(np.int64), 86400)
    return J2000_JD + whole_days.astype(np.float64), (remainder_s + fraction_s) / SECONDS_PER_DAY


def reference_state_m(kernel, center: int, target: int, whole_seconds, fraction_s):
    """Position [m] and velocity [m/s] of `target` relative to `center` in ecliptic axes, from the SPK kernel.

    Walks the kernel's segment chains through the Solar System barycentre where needed.
    """
    jd, jd_fraction = seconds_to_jd_pair(np.asarray(whole_seconds), np.asarray(fraction_s))

    def chain(body):
        # Path from body up to the SSB (0) through available segments.
        path = []
        while body != 0:
            parent = next(c for (c, t) in kernel.pairs if t == body)
            path.append((parent, body))
            body = parent
        return path

    def absolute(body):
        position_km = np.zeros((3, jd.size))
        velocity_km_day = np.zeros((3, jd.size))
        for (c, t) in chain(body):
            p, v = kernel[c, t].compute_and_differentiate(jd, jd_fraction)
            position_km += p
            velocity_km_day += v
        return position_km, velocity_km_day

    target_p, target_v = absolute(target)
    center_p, center_v = absolute(center) if center != 0 else (0.0, 0.0)
    position_m = ICRF_TO_ECLIPTIC @ ((target_p - center_p) * KM_TO_M)
    velocity_m_s = ICRF_TO_ECLIPTIC @ ((target_v - center_v) * KM_TO_M / SECONDS_PER_DAY)
    return position_m.T, velocity_m_s.T
