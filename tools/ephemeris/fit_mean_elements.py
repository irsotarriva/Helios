#!/usr/bin/env python3
"""Fit Helios secular (mean) elements to a JPL DE kernel and write data/solar_system/mean_elements.csv.

For each body, the 12 parameters (6 elements at J2000 + 6 linear rates) minimise the RMS
*position* error against DE over the fit window, starting from a linear regression of the
osculating elements. This mirrors how Standish derived his approximate planetary elements,
but is reproducible from the kernel we ship.

Usage: fit_mean_elements.py de440.bsp out.csv [--start-year 2000 --end-year 2200]

Rationale for the default window: linear rates extrapolate poorly, so the window should be the
period the elements are used in. The game starts at the present and runs forward, and fits on
2000-2200 are the best or tied for every body over 2000-2100 and 2000-2200 (for Uranus, Neptune
and Pluto 6-22 times better over 2000-2100 than the 1900-2050 window used with DE421), and no
worse than the alternatives over the whole of DE440 (docs/validation/ephemeris/README.md).
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from jplephem.spk import SPK
from scipy.optimize import least_squares

from common import NAIF_NAMES, SECONDS_PER_DAY, kernel_label, reference_state_m
from kepler_model import PARAMETER_NAMES, osculating_elements, position_m

JULIAN_CENTURY_S = 36525.0 * SECONDS_PER_DAY
GM_SUN_M3_S2 = 1.32712440041e20        # initial guess only; the fit determines μ_eff = n²a³
GM_EARTH_MOON_M3_S2 = 4.0350323e14     # initial guess only

# (target, centre, sampling step in days).
# Rationale: Uranus, Neptune and Pluto orbit the Solar System barycentre far more closely than
# the Sun, whose reflex motion around the SSB is ~1 solar radius; fitting them barycentrically
# cuts their error 4-13x (see docs/validation/ephemeris/README.md). Jupiter and Saturn are
# limited by their mutual "great inequality" instead, and fit marginally better heliocentrically.
BODIES = [(1, 10, 2.0), (2, 10, 4.0), (3, 10, 4.0), (4, 10, 8.0), (5, 10, 16.0), (6, 10, 16.0),
          (7, 0, 32.0), (8, 0, 32.0), (9, 0, 32.0), (301, 399, 0.5)]


def fit_body(kernel, target: int, center: int, step_days: float, start_year: float, end_year: float):
    start_s = (start_year - 2000.0) * 365.25 * SECONDS_PER_DAY
    end_s = (end_year - 2000.0) * 365.25 * SECONDS_PER_DAY
    elapsed_s = np.arange(start_s, end_s, step_days * SECONDS_PER_DAY)
    whole = np.floor(elapsed_s).astype(np.int64)
    truth_position, truth_velocity = reference_state_m(kernel, center, target, whole, elapsed_s - whole)

    mu = GM_EARTH_MOON_M3_S2 if center == 399 else GM_SUN_M3_S2  # SSB-centred: ≈ GM_sun too
    osc = osculating_elements(truth_position, truth_velocity, mu)
    centuries = elapsed_s / JULIAN_CENTURY_S
    guess = []
    for key in ["a", "e", "i", "node", "peri", "mean_lon"]:
        values = np.unwrap(osc[key]) if key in ("node", "peri", "mean_lon") else osc[key]
        slope, intercept = np.polyfit(centuries, values, 1)
        guess += [intercept, slope]
    guess = np.array(guess)

    # Optimise in (element, rate per century) with a in AU-ish units for conditioning.
    scale = np.ones(12)
    scale[0] = scale[1] = guess[0]

    def residual(scaled):
        parameters = scaled * scale
        per_second = parameters.copy()
        per_second[1::2] /= JULIAN_CENTURY_S
        return ((position_m(per_second, elapsed_s) - truth_position) / guess[0]).ravel()

    solution = least_squares(residual, guess / scale, method="lm", xtol=1e-15, ftol=1e-15, gtol=1e-15)
    parameters = solution.x * scale
    parameters[1::2] /= JULIAN_CENTURY_S
    parameters[4] = np.mod(parameters[4], 2 * np.pi) if parameters[4] > np.pi else parameters[4]
    for angle in (6, 8, 10):
        parameters[angle] = np.mod(parameters[angle], 2 * np.pi)

    error_m = np.linalg.norm(position_m(parameters, elapsed_s) - truth_position, axis=1)
    return parameters, float(np.sqrt(np.mean(error_m**2))), float(np.max(error_m))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kernel")
    parser.add_argument("output")
    parser.add_argument("--start-year", type=float, default=2000.0)
    parser.add_argument("--end-year", type=float, default=2200.0)
    arguments = parser.parse_args()

    kernel = SPK.open(arguments.kernel)
    rows = []
    for target, center, step_days in BODIES:
        parameters, rms_m, max_m = fit_body(kernel, target, center, step_days, arguments.start_year, arguments.end_year)
        name, center_name = NAIF_NAMES[target], NAIF_NAMES[center]
        rows.append((name, center_name, parameters, rms_m, max_m))
        print(f"{name:>22}: RMS {rms_m / 1e3:12.1f} km   max {max_m / 1e3:12.1f} km", file=sys.stderr)

    with open(arguments.output, "w", encoding="utf-8") as stream:
        stream.write("# Helios secular (mean) elements, J2000 ecliptic, TDB seconds since J2000, SI units.\n")
        stream.write(f"# Fitted to JPL {kernel_label(arguments.kernel)} over "
                     f"{arguments.start_year:.0f}-{arguments.end_year:.0f} by "
                     "tools/ephemeris/fit_mean_elements.py (least-squares on position).\n")
        for name, _, _, rms_m, max_m in rows:
            stream.write(f"# {name}: fit RMS {rms_m / 1e3:.1f} km, max {max_m / 1e3:.1f} km\n")
        stream.write("target,center,reference_epoch_tdb_s," + ",".join(PARAMETER_NAMES) + "\n")
        for name, center_name, parameters, _, _ in rows:
            stream.write(f"{name},{center_name},0," + ",".join(f"{value:.17g}" for value in parameters) + "\n")


if __name__ == "__main__":
    main()
