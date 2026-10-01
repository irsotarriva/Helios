#!/usr/bin/env python3
"""Validate the Helios on-rails body models against a JPL DE kernel and write figures + a data summary.

Everything Helios-side is computed by the C++ code (helios_ephemeris_probe); this script only
prepares epochs, calls the probe, computes the truth with jplephem and compares.

Usage: validate.py --probe build/tools/helios_ephemeris_probe --kernel de440.bsp \
                   --hce de440.hce --elements data/solar_system/mean_elements.csv \
                   --work build/validation --out docs/validation/ephemeris

The .hce file must cover the validated window (by default the whole kernel). The window the mean
elements were fitted on is read from the header of the elements file.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402
import numpy as np  # noqa: E402
from jplephem.spk import SPK  # noqa: E402

from common import (J2000_JD, NAIF_NAMES, SECONDS_PER_DAY, kernel_coverage_years, kernel_label,  # noqa: E402
                    reference_state_m)

NAME_TO_NAIF = {name: naif for naif, name in NAIF_NAMES.items()}
YEAR_S = 365.25 * SECONDS_PER_DAY
ARCSEC_PER_RAD = 180.0 / np.pi * 3600.0

# Reference palette (dataviz skill, light mode): categorical slots in fixed order + ink.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
SURFACE, INK, INK_2, INK_3, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8983", "#e6e5e0"

CHEBYSHEV_PAIRS = [("Sun", "Solar System barycentre"), ("Mercury", "Solar System barycentre"),
                   ("Venus", "Solar System barycentre"), ("Earth-Moon barycentre", "Solar System barycentre"),
                   ("Earth", "Earth-Moon barycentre"), ("Moon", "Earth-Moon barycentre"),
                   ("Mars", "Solar System barycentre"), ("Jupiter barycentre", "Solar System barycentre"),
                   ("Saturn barycentre", "Solar System barycentre"), ("Uranus barycentre", "Solar System barycentre"),
                   ("Neptune barycentre", "Solar System barycentre"), ("Pluto barycentre", "Solar System barycentre")]
# Pairs whose relative state the frame tree must *compose* through a common ancestor.
COMPOSED_PAIRS = [("Moon", "Earth"), ("Earth", "Sun"), ("Moon", "Sun"), ("Mars", "Earth"), ("Mars", "Sun"),
                  ("Jupiter barycentre", "Sun"),
                  ("Jupiter barycentre", "Earth")]
MEAN_ELEMENT_BODIES = [("Mercury", "Sun"), ("Venus", "Sun"), ("Earth-Moon barycentre", "Sun"), ("Mars", "Sun"),
                       ("Jupiter barycentre", "Sun"), ("Saturn barycentre", "Sun"),
                       ("Uranus barycentre", "Solar System barycentre"), ("Neptune barycentre", "Solar System barycentre"),
                       ("Pluto barycentre", "Solar System barycentre"), ("Moon", "Earth")]
SHORT_NAMES = {"Earth-Moon barycentre": "Earth–Moon bary.", "Jupiter barycentre": "Jupiter",
               "Saturn barycentre": "Saturn", "Uranus barycentre": "Uranus", "Neptune barycentre": "Neptune",
               "Pluto barycentre": "Pluto", "Solar System barycentre": "SSB"}


def short(name: str) -> str:
    return SHORT_NAMES.get(name, name)


def style_axes(axes, title: str | None = None) -> None:
    axes.set_facecolor(SURFACE)
    for spine in ("top", "right"):
        axes.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        axes.spines[spine].set_color(INK_3)
    axes.tick_params(colors=INK_2, labelsize=8)
    axes.grid(True, color=GRID, linewidth=0.6)
    axes.set_axisbelow(True)
    if title:
        axes.set_title(title, color=INK, fontsize=10, loc="left")


def new_figure(width: float, height: float, rows: int = 1, columns: int = 1, **kwargs):
    figure, axes = plt.subplots(rows, columns, figsize=(width, height), facecolor=SURFACE, **kwargs)
    return figure, axes


def run_probe(probe: Path, hce: Path, elements: Path, pairs: list[tuple[str, str]], epochs_s: np.ndarray,
              work: Path, tag: str) -> dict[tuple[str, str], dict[str, np.ndarray]]:
    whole = np.floor(epochs_s).astype(np.int64)
    fraction = epochs_s - whole
    pairs_path, epochs_path, output_path = work / f"{tag}_pairs.csv", work / f"{tag}_epochs.csv", work / f"{tag}.csv"
    pairs_path.write_text("".join(f"{t},{o}\n" for t, o in pairs))
    with epochs_path.open("w") as stream:
        for w, f in zip(whole, fraction):
            stream.write(f"{w},{f:.17g}\n")
    subprocess.run([str(probe), str(hce), str(elements), str(pairs_path), str(epochs_path), str(output_path)],
                   check=True)
    table = np.genfromtxt(output_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    results = {}
    for target, observer in pairs:
        rows = table[(table["target"] == target) & (table["observer"] == observer)]
        results[(target, observer)] = {
            "whole": rows["whole_s"].astype(np.int64), "fraction": rows["fraction_s"].astype(float),
            "position": np.stack([rows["x_m"], rows["y_m"], rows["z_m"]], axis=1),
            "velocity": np.stack([rows["vx_m_s"], rows["vy_m_s"], rows["vz_m_s"]], axis=1),
            "acceleration": np.stack([rows["ax_m_s2"], rows["ay_m_s2"], rows["az_m_s2"]], axis=1)}
    return results


def truth(kernel, target: str, observer: str, whole, fraction):
    return reference_state_m(kernel, NAME_TO_NAIF[observer], NAME_TO_NAIF[target], whole, fraction)


def fit_window_years(elements: Path) -> tuple[float, float]:
    """The window the mean elements were fitted on, from the '# Fitted to ... over A-B' header line."""
    match = re.search(r"over (\d+)-(\d+)", elements.read_text(encoding="utf-8"))
    if match is None:
        raise SystemExit(f"{elements}: no '# Fitted to ... over <start>-<end>' header line")
    return float(match.group(1)), float(match.group(2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag in ("--probe", "--kernel", "--hce", "--elements", "--work", "--out"):
        parser.add_argument(flag, type=Path, required=True)
    parser.add_argument("--start-year", type=float, default=None, help="default: start of the kernel")
    parser.add_argument("--end-year", type=float, default=None, help="default: end of the kernel")
    arguments = parser.parse_args()
    arguments.work.mkdir(parents=True, exist_ok=True)
    arguments.out.mkdir(parents=True, exist_ok=True)
    kernel = SPK.open(str(arguments.kernel))
    label = kernel_label(arguments.kernel)
    coverage_start_year, coverage_end_year = kernel_coverage_years(kernel)
    # Stay a little inside the coverage: the acceleration check differences JPL's velocity.
    start_year = arguments.start_year if arguments.start_year is not None else coverage_start_year + 0.1
    end_year = arguments.end_year if arguments.end_year is not None else coverage_end_year - 0.1
    fit_start_year, fit_end_year = fit_window_years(arguments.elements)
    window = f"{start_year:.0f}–{end_year:.0f}"
    generator = np.random.default_rng(20260930)
    summary: dict[str, object] = {"source": f"JPL {label} ({start_year:.0f}-{end_year:.0f})",
                                  "axes": "J2000 ecliptic",
                                  "mean_elements_fit_window": [fit_start_year, fit_end_year]}

    # ── 1. Chebyshev model: direct segments and frame-tree compositions ──────────────────────
    start_s, end_s = (start_year - 2000.0) * YEAR_S, (end_year - 2000.0) * YEAR_S
    random_epochs = np.sort(generator.uniform(start_s, end_s, 20_000))
    # Exact record boundaries of the lunar records (Moon/Earth), where continuity matters most:
    # about 2,000 of them, spread over the whole window.
    moon_init_jd, moon_interval_days, _ = kernel[3, 301].load_array()
    moon_origin_s = (moon_init_jd - J2000_JD) * SECONDS_PER_DAY
    moon_record_s = moon_interval_days * SECONDS_PER_DAY
    first_record = np.ceil((start_s - moon_origin_s) / moon_record_s)
    last_record = np.floor((end_s - moon_origin_s) / moon_record_s)
    stride = max(1, int(np.ceil((last_record - first_record) / 2_000)))
    boundary_epochs = moon_origin_s + moon_record_s * np.arange(first_record, last_record, stride)
    chebyshev_epochs = np.concatenate([random_epochs, boundary_epochs])

    chebyshev = run_probe(arguments.probe, arguments.hce, arguments.elements, CHEBYSHEV_PAIRS + COMPOSED_PAIRS,
                          chebyshev_epochs, arguments.work, "chebyshev")
    chebyshev_rows = []
    for (target, observer), result in chebyshev.items():
        true_position, true_velocity = truth(kernel, target, observer, result["whole"], result["fraction"])
        position_error = np.linalg.norm(result["position"] - true_position, axis=1)
        velocity_error = np.linalg.norm(result["velocity"] - true_velocity, axis=1)
        distance = np.linalg.norm(true_position, axis=1)
        chebyshev_rows.append({
            "target": target, "observer": observer,
            "kind": "composed" if (target, observer) in COMPOSED_PAIRS else "segment",
            "samples": int(position_error.size),
            "max_position_error_m": float(position_error.max()),
            "rms_position_error_m": float(np.sqrt(np.mean(position_error**2))),
            "max_relative_position_error": float((position_error / distance).max()),
            "max_velocity_error_m_s": float(velocity_error.max())})

    # Acceleration: Helios' analytic second derivative vs a central difference of JPL velocity.
    acceleration_rows = []
    step_s = 60.0
    for target, observer in [("Moon", "Earth"), ("Earth", "Sun"), ("Mars", "Sun"), ("Jupiter barycentre", "Sun")]:
        result = chebyshev[(target, observer)]
        whole, fraction = result["whole"][:2000], result["fraction"][:2000]
        _, velocity_after = truth(kernel, target, observer, whole + int(step_s), fraction)
        _, velocity_before = truth(kernel, target, observer, whole - int(step_s), fraction)
        finite_difference = (velocity_after - velocity_before) / (2 * step_s)
        analytic = result["acceleration"][:2000]
        relative = np.linalg.norm(analytic - finite_difference, axis=1) / np.linalg.norm(finite_difference, axis=1)
        acceleration_rows.append({"target": target, "observer": observer,
                                  "median_relative_error": float(np.median(relative)),
                                  "max_relative_error": float(relative.max()),
                                  "typical_acceleration_m_s2": float(np.median(np.linalg.norm(analytic, axis=1)))})

    # ── 2. Mean-element (Keplerian) model, every 5 days over the whole window ───────────────
    # Statistics are given inside the window the elements were fitted on and over everything:
    # outside the fit window the model extrapolates.
    kepler_epochs = np.arange(start_s, end_s, 5 * SECONDS_PER_DAY)
    kepler_pairs = [(f"{target} (mean elements)", center) for target, center in MEAN_ELEMENT_BODIES]
    kepler = run_probe(arguments.probe, arguments.hce, arguments.elements, kepler_pairs, kepler_epochs,
                       arguments.work, "kepler")
    kepler_rows, kepler_series = [], {}
    for (target, center), (model_name, _) in zip(MEAN_ELEMENT_BODIES, kepler_pairs):
        result = kepler[(model_name, center)]
        true_position, _ = truth(kernel, target, center, result["whole"], result["fraction"])
        error_m = np.linalg.norm(result["position"] - true_position, axis=1)
        cosine = np.sum(result["position"] * true_position, axis=1) / (
            np.linalg.norm(result["position"], axis=1) * np.linalg.norm(true_position, axis=1))
        angle_arcsec = np.arccos(np.clip(cosine, -1.0, 1.0)) * ARCSEC_PER_RAD
        years = 2000.0 + (result["whole"] + result["fraction"]) / YEAR_S
        kepler_series[target] = (years, error_m, angle_arcsec)
        fitted = (years >= fit_start_year) & (years <= fit_end_year)
        kepler_rows.append({"target": target, "center": center,
                            "rms_position_error_km": float(np.sqrt(np.mean(error_m[fitted]**2)) / 1e3),
                            "max_position_error_km": float(error_m[fitted].max() / 1e3),
                            "rms_angular_error_arcsec": float(np.sqrt(np.mean(angle_arcsec[fitted]**2))),
                            "max_angular_error_arcsec": float(angle_arcsec[fitted].max()),
                            "whole_window_rms_position_error_km": float(np.sqrt(np.mean(error_m**2)) / 1e3),
                            "whole_window_rms_angular_error_arcsec": float(np.sqrt(np.mean(angle_arcsec**2))),
                            "whole_window_max_angular_error_arcsec": float(angle_arcsec.max()),
                            "mean_distance_km": float(np.mean(np.linalg.norm(true_position, axis=1)) / 1e3)})

    # Moon, densely over two years, decomposed into radial / along-track / cross-track.
    moon_epochs = np.arange((2024.0 - 2000) * YEAR_S, (2026.0 - 2000) * YEAR_S, 3 * 3600.0)
    moon = run_probe(arguments.probe, arguments.hce, arguments.elements, [("Moon (mean elements)", "Earth")],
                     moon_epochs, arguments.work, "moon")[("Moon (mean elements)", "Earth")]
    moon_true_position, moon_true_velocity = truth(kernel, "Moon", "Earth", moon["whole"], moon["fraction"])
    radial = moon_true_position / np.linalg.norm(moon_true_position, axis=1)[:, None]
    normal = np.cross(moon_true_position, moon_true_velocity)
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    along = np.cross(normal, radial)
    moon_delta = moon["position"] - moon_true_position
    moon_components_km = {name: np.sum(moon_delta * axis, axis=1) / 1e3
                          for name, axis in (("radial", radial), ("along-track", along), ("cross-track", normal))}
    moon_years = 2000.0 + (moon["whole"] + moon["fraction"]) / YEAR_S

    # Orbit overview from the Chebyshev model, heliocentric, one year per inner planet / 1900-2050 outer.
    overview_epochs = np.arange((2000.0 - 2000) * YEAR_S, (2001.9 - 2000) * YEAR_S, 2 * SECONDS_PER_DAY)
    overview_outer_epochs = np.arange((1900.2 - 2000) * YEAR_S, (2049.8 - 2000) * YEAR_S, 30 * SECONDS_PER_DAY)
    inner_bodies = ["Mercury", "Venus", "Earth", "Mars"]
    outer_bodies = ["Jupiter barycentre", "Saturn barycentre", "Uranus barycentre", "Neptune barycentre",
                    "Pluto barycentre"]
    overview_inner = run_probe(arguments.probe, arguments.hce, "-", [(b, "Sun") for b in inner_bodies],
                               overview_epochs, arguments.work, "overview_inner")
    overview_outer = run_probe(arguments.probe, arguments.hce, "-", [(b, "Sun") for b in outer_bodies],
                               overview_outer_epochs, arguments.work, "overview_outer")

    # ── Write data ──────────────────────────────────────────────────────────────────────────
    def write_csv(name: str, rows: list[dict]) -> None:
        with (arguments.out / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    write_csv(f"chebyshev_vs_{label.lower()}.csv", chebyshev_rows)
    write_csv("acceleration_consistency.csv", acceleration_rows)
    write_csv(f"mean_elements_vs_{label.lower()}.csv", kepler_rows)
    summary.update({"chebyshev": chebyshev_rows, "acceleration": acceleration_rows, "mean_elements": kepler_rows,
                    "moon_mean_elements_components_rms_km": {k: float(np.sqrt(np.mean(v**2)))
                                                             for k, v in moon_components_km.items()}})
    (arguments.out / "summary.json").write_text(json.dumps(summary, indent=2))

    # ── Figures ─────────────────────────────────────────────────────────────────────────────
    plt.rcParams.update({"font.size": 9, "axes.labelcolor": INK_2, "text.color": INK})

    # Fig 1: orbits (Helios Chebyshev model), heliocentric ecliptic.
    figure, (left, right) = new_figure(11, 5.4, 1, 2)
    for axes, bodies, results, limit_au in ((left, inner_bodies, overview_inner, 1.75),
                                            (right, outer_bodies, overview_outer, 50.0)):
        for slot, body in enumerate(bodies):
            position_au = results[(body, "Sun")]["position"] / 1.495978707e11
            axes.plot(position_au[:, 0], position_au[:, 1], color=SERIES[slot], linewidth=1.6)
            axes.annotate(short(body), position_au[0, :2], xytext=(6, 4), textcoords="offset points",
                          color=INK_2, fontsize=8)
        axes.plot(0, 0, marker="o", markersize=5, color=INK)
        axes.set_aspect("equal")
        axes.set_xlim(-limit_au, limit_au)
        axes.set_ylim(-limit_au, limit_au)
        axes.set_xlabel("x (AU, J2000 ecliptic)")
        axes.set_ylabel("y (AU)")
        style_axes(axes)
    left.set_title("Inner planets, 2000–2002", loc="left", color=INK, fontsize=10)
    right.set_title("Outer planets, 1900–2050", loc="left", color=INK, fontsize=10)
    figure.suptitle("Heliocentric orbits evaluated by the Helios frame tree (Chebyshev model)", color=INK,
                    fontsize=11, x=0.01, ha="left")
    figure.tight_layout()
    figure.savefig(arguments.out / "fig1_orbits.png", dpi=150)
    plt.close(figure)

    # Fig 2: how exactly the C++ Chebyshev evaluation reproduces the kernel (max error per body).
    rows_sorted = sorted(chebyshev_rows, key=lambda row: row["max_position_error_m"])
    figure, axes = new_figure(8.5, 5.2)
    labels = [f"{short(r['target'])} ← {short(r['observer'])}" + ("  (composed)" if r["kind"] == "composed" else "")
              for r in rows_sorted]
    values_mm = [r["max_position_error_m"] * 1e3 for r in rows_sorted]
    axes.barh(labels, values_mm, color=SERIES[0], height=0.62)
    axes.set_xscale("log")
    for index, value in enumerate(values_mm):
        axes.annotate(f"{value:.2g} mm", (value, index), xytext=(4, 0), textcoords="offset points",
                      va="center", color=INK_2, fontsize=7.5)
    axes.set_xlabel(f"maximum position difference vs JPL {label} over {chebyshev_epochs.size:,} epochs, "
                    f"{window} (mm, log scale)")
    style_axes(axes, f"Helios C++ vs JPL {label}: ≤ {max(values_mm):.2g} mm everywhere "
                     "(≈1e-15 relative, the float64 floor)")
    axes.tick_params(axis="y", labelsize=8, colors=INK)
    figure.tight_layout()
    figure.savefig(arguments.out / "fig2_chebyshev_error.png", dpi=150)
    plt.close(figure)

    # Fig 3: mean-element model angular error vs time, small multiples.
    figure, grid = new_figure(12, 8.2, 4, 3, sharex=True)
    band_tint = "#b7d3f6"  # sequential blue step 150: the same hue, receding
    for axes, (target, _center) in zip(grid.flat, MEAN_ELEMENT_BODIES):
        years, _error_m, angle_arcsec = kepler_series[target]
        # Rationale: 5-day samples of month-scale oscillations alias into a solid band; yearly
        # RMS (line) over the yearly maximum (tint) shows the structure without hiding extremes.
        year_bins = np.floor(years).astype(int)
        centres, rms, maximum = [], [], []
        for year in np.unique(year_bins):
            values = angle_arcsec[year_bins == year]
            centres.append(year + 0.5)
            rms.append(np.sqrt(np.mean(values**2)))
            maximum.append(values.max())
        # Rationale: outside the fit window the error grows by orders of magnitude, so the scale
        # is logarithmic; the shaded span is the window the elements were fitted on.
        axes.axvspan(fit_start_year, fit_end_year, color=GRID, linewidth=0, label="fit window")
        axes.fill_between(centres, rms, maximum, color=band_tint, linewidth=0, label="yearly max")
        axes.plot(centres, rms, color=SERIES[0], linewidth=1.6, label="yearly RMS")
        axes.set_yscale("log")
        low, high = axes.get_ylim()
        if high / low >= 10.0:  # otherwise the minor ticks are the only labels there are
            axes.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        else:
            axes.yaxis.set_minor_locator(matplotlib.ticker.LogLocator(subs=(2.0, 5.0)))
            axes.yaxis.set_minor_formatter(matplotlib.ticker.FuncFormatter(lambda value, _: f"{value:,.0f}"))
            axes.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda value, _: f"{value:,.0f}"))
        row = next(r for r in kepler_rows if r["target"] == target)
        rms_arcsec = row["rms_angular_error_arcsec"]
        rms_text = f"{rms_arcsec:,.0f}" if rms_arcsec >= 100.0 else f"{rms_arcsec:.2g}"
        style_axes(axes, f"{short(target)}: {rms_text}″ RMS ({row['rms_position_error_km']:,.0f} km)")
    # The legend goes in the empty space of the last row, clear of the data.
    handles, legend_labels = grid.flat[0].get_legend_handles_labels()
    legend = figure.legend(handles, legend_labels, frameon=False, loc="lower center", fontsize=9,
                           bbox_to_anchor=(0.66, 0.12))
    for text in legend.get_texts():
        text.set_color(INK_2)
    for axes in grid.flat[len(MEAN_ELEMENT_BODIES):]:
        axes.set_visible(False)
    for axes in grid[:, 0]:
        axes.set_ylabel("error (arcsec)")
    # The lowest visible panel of each column carries the year axis.
    for column in range(grid.shape[1]):
        lowest = [axes for axes in grid[:, column] if axes.get_visible()][-1]
        lowest.set_xlabel("year")
        lowest.tick_params(labelbottom=True)
    figure.suptitle("Mean-element (Keplerian + secular rates) model: angular error seen from the fitted centre "
                    "(Sun; SSB for Uranus–Pluto; Earth for the Moon).\n"
                    f"Panel titles give the RMS inside the fit window, {fit_start_year:.0f}–{fit_end_year:.0f}.",
                    color=INK, fontsize=11, x=0.01, ha="left")
    figure.tight_layout()
    figure.savefig(arguments.out / "fig3_mean_elements_error.png", dpi=150)
    plt.close(figure)

    # Fig 4: Moon mean-element error decomposition.
    figure, axes = new_figure(10, 4.2)
    for slot, (name, component_km) in enumerate(moon_components_km.items()):
        axes.plot(moon_years, component_km, color=SERIES[slot], linewidth=1.2, label=name)
    axes.axhline(0, color=INK_3, linewidth=0.8)
    axes.set_xlabel("year")
    axes.set_ylabel(f"Helios − {label} (km)")
    legend = axes.legend(frameon=False, ncols=3, loc="upper left", fontsize=8)
    for text in legend.get_texts():
        text.set_color(INK_2)
    style_axes(axes, "Moon with mean elements: the residual is the Sun's perturbation (evection, variation)")
    figure.tight_layout()
    figure.savefig(arguments.out / "fig4_moon_mean_elements.png", dpi=150)
    plt.close(figure)

    # The tables of docs/validation/ephemeris/README.md, ready to paste.
    lines = ["| Target | Observer | Kind | Max Δr (mm) | RMS Δr (mm) | Max Δr / r | Max Δv (m/s) |", "|---|---|---|---|---|---|---|"]
    for row in chebyshev_rows:
        lines.append(f"| {short(row['target'])} | {short(row['observer'])} | {row['kind']} | "
                     f"{row['max_position_error_m'] * 1e3:.3g} | {row['rms_position_error_m'] * 1e3:.3g} | "
                     f"{row['max_relative_position_error']:.1e} | {row['max_velocity_error_m_s']:.1e} |")
    lines += ["", "| Pair | Typical \\|a\\| (m/s²) | Median relative diff. | Max relative diff. |", "|---|---|---|---|"]
    for row in acceleration_rows:
        lines.append(f"| {short(row['target'])} ← {short(row['observer'])} | {row['typical_acceleration_m_s2']:.3g} | "
                     f"{row['median_relative_error']:.1e} | {row['max_relative_error']:.1e} |")
    lines += ["", f"| Body | Fitted around | RMS error (″) | Max (″) | RMS (km) | Max (km) | RMS over {window} (″) |",
              "|---|---|---|---|---|---|---|"]
    for row in kepler_rows:
        lines.append(f"| {short(row['target'])} | {short(row['center'])} | {row['rms_angular_error_arcsec']:,.1f} | "
                     f"{row['max_angular_error_arcsec']:,.1f} | {row['rms_position_error_km']:,.0f} | "
                     f"{row['max_position_error_km']:,.0f} | {row['whole_window_rms_angular_error_arcsec']:,.1f} |")
    lines += ["", "Moon components RMS (km): " + ", ".join(
        f"{name} {np.sqrt(np.mean(values**2)):,.0f}" for name, values in moon_components_km.items())]
    (arguments.work / "tables.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps({"chebyshev_worst_mm": max(values_mm),
                      "acceleration": acceleration_rows,
                      "mean_elements": [(r["target"], round(r["rms_angular_error_arcsec"], 1)) for r in kepler_rows]},
                     indent=1))


if __name__ == "__main__":
    main()
