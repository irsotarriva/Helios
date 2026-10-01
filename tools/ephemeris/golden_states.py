#!/usr/bin/env python3
"""Print the golden states of test/unit/frames/test_de440_golden.cpp from a JPL DE kernel.

The states are computed by jplephem, independently of Helios, at fixed epochs inside the
committed excerpt (test/data), in J2000 ecliptic axes and SI units.

Usage: golden_states.py de440.bsp
"""

from __future__ import annotations

import argparse

import numpy as np
from jplephem.spk import SPK

from common import reference_state_m

# (label, centre, target) and the epochs: TDB seconds since J2000, whole part and fraction.
PAIRS = [("Moon-Earth", 399, 301), ("Earth-Sun", 10, 399), ("Mars-Sun", 10, 4)]
EPOCHS = [(631238400, 0.0), (632275200, 0.25), (633657600, 0.5), (635212800, 0.75)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kernel")
    arguments = parser.parse_args()
    kernel = SPK.open(arguments.kernel)
    for label, center, target in PAIRS:
        for whole, fraction in EPOCHS:
            position, velocity = reference_state_m(kernel, center, target, np.array([whole]), np.array([fraction]))
            vector = lambda values: "{" + ", ".join(repr(float(value)) for value in values) + "}"  # noqa: E731
            print(f'    GoldenState{{"{label}", {whole}, {fraction!r}, {vector(position[0])}, {vector(velocity[0])}}},')


if __name__ == "__main__":
    main()
