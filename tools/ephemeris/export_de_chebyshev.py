#!/usr/bin/env python3
"""Convert a JPL DE SPK kernel (type 2 segments) into a Helios Chebyshev ephemeris file (.hce).

Only coordinates change: km → m, ICRF → J2000 ecliptic (a linear map, so it applies to the
Chebyshev coefficients directly), JD → TDB seconds since J2000. The series themselves are
JPL's, so evaluating the .hce file reproduces DE to floating-point precision.

Usage: export_de_chebyshev.py de440.bsp out.hce [--start-year 1900 --end-year 2100]
Without the years, the whole kernel is exported (DE440: 1550-2650, about 110 MB).
"""

from __future__ import annotations

import argparse
import struct
import sys

import numpy as np
from jplephem.spk import SPK

from common import EXPORTED_SEGMENTS, ICRF_TO_ECLIPTIC, J2000_JD, KM_TO_M, NAIF_NAMES, SECONDS_PER_DAY

MAGIC = b"HLSCHEB1"


def year_to_jd(year: float) -> float:
    return J2000_JD + (year - 2000.0) * 365.25


def write_name(stream, name: str) -> None:
    encoded = name.encode("utf-8")
    stream.write(struct.pack("<I", len(encoded)))
    stream.write(encoded)


def export(kernel_path: str, output_path: str, start_year: float | None, end_year: float | None) -> None:
    kernel = SPK.open(kernel_path)
    start_jd = -np.inf if start_year is None else year_to_jd(start_year)
    end_jd = np.inf if end_year is None else year_to_jd(end_year)
    with open(output_path, "wb") as stream:
        stream.write(MAGIC)
        stream.write(struct.pack("<I", len(EXPORTED_SEGMENTS)))
        for center, target in EXPORTED_SEGMENTS:
            segment = kernel[center, target]
            init_jd, interval_days, coefficients_km = segment.load_array()  # (3, records, n)
            record_count = coefficients_km.shape[1]
            first = 0 if np.isinf(start_jd) else max(0, int(np.floor((start_jd - init_jd) / interval_days)))
            last = record_count if np.isinf(end_jd) else min(
                record_count, int(np.ceil((end_jd - init_jd) / interval_days)))
            selected_km = coefficients_km[:, first:last, :]
            # Rotate each coefficient triple into ecliptic axes and scale to metres.
            ecliptic_m = np.einsum("ij,jrk->rik", ICRF_TO_ECLIPTIC, selected_km) * KM_TO_M  # (records, 3, n)

            start_offset_days = (init_jd + first * interval_days) - J2000_JD
            start_seconds = start_offset_days * SECONDS_PER_DAY
            whole_seconds = int(np.floor(start_seconds))
            record_duration_s = interval_days * SECONDS_PER_DAY
            assert float(whole_seconds) == start_seconds, "DE records start on whole seconds"
            assert record_duration_s == np.round(record_duration_s)

            write_name(stream, NAIF_NAMES[target])
            write_name(stream, NAIF_NAMES[center])
            stream.write(struct.pack("<qddII", whole_seconds, 0.0, record_duration_s,
                                     ecliptic_m.shape[0], ecliptic_m.shape[2]))
            stream.write(np.ascontiguousarray(ecliptic_m, dtype="<f8").tobytes())
            print(f"{NAIF_NAMES[target]:>24} ← {NAIF_NAMES[center]:<24} "
                  f"{ecliptic_m.shape[0]:6d} records × {ecliptic_m.shape[2]:2d} coeffs, "
                  f"{interval_days:4.0f}-day records", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kernel")
    parser.add_argument("output")
    parser.add_argument("--start-year", type=float, default=None)
    parser.add_argument("--end-year", type=float, default=None)
    arguments = parser.parse_args()
    export(arguments.kernel, arguments.output, arguments.start_year, arguments.end_year)


if __name__ == "__main__":
    main()
