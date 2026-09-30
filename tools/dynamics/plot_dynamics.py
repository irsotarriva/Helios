#!/usr/bin/env python3
"""Plot the Phase 1 dynamics validation produced by helios_dynamics_scenarios.

Usage: plot_dynamics.py <scenario CSV directory> <output directory>
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.integrate import solve_ivp  # noqa: E402

# Reference palette (dataviz skill, light mode): categorical slots in fixed order + ink.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
SURFACE, INK, INK_2, INK_3, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8983", "#e6e5e0"
SEPARATION_M = 3.844e8
MOON_MASS_RATIO = 4.902800066e12 / (3.986004354e14 + 4.902800066e12)


def read(path: Path) -> dict[str, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def style(axes, title: str | None = None) -> None:
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


def legend(axes, **kwargs) -> None:
    handle = axes.legend(frameon=False, fontsize=8, **kwargs)
    for text in handle.get_texts():
        text.set_color(INK_2)


def figure(width: float, height: float, columns: int = 1):
    fig, axes = plt.subplots(1, columns, figsize=(width, height), facecolor=SURFACE)
    return fig, axes


def main() -> None:
    source, output = Path(sys.argv[1]), Path(sys.argv[2])
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.labelcolor": INK_2, "text.color": INK})
    summary: dict[str, object] = {}

    # 1 — Encke vs Cowell on a 20-day HEO in the real Solar System.
    history = read(source / "encke_vs_cowell.csv")
    fig, axes = figure(10, 4.2)
    axes.semilogy(history["time_days"], history["kepler_minus_cowell_m"], color=SERIES[1], linewidth=1.6,
                  label="pure two-body (Kepler) prediction")
    axes.semilogy(history["time_days"], np.maximum(history["encke_minus_cowell_m"], 1e-6), color=SERIES[0],
                  linewidth=1.6, label="Helios Encke propagator")
    axes.set_xlabel("days since 2020-01-02 TDB")
    axes.set_ylabel("distance from Cowell reference (m, log)")
    legend(axes, loc="center right")
    style(axes, "HEO (7,000 × ~150,000 km) with Sun, Moon and planets from DE421: Encke vs Cowell")
    fig.tight_layout()
    fig.savefig(output / "fig1_encke_vs_cowell.png", dpi=150)
    plt.close(fig)
    summary["encke_vs_cowell"] = {"max_encke_minus_cowell_m": float(history["encke_minus_cowell_m"].max()),
                                  "max_kepler_minus_cowell_m": float(history["kepler_minus_cowell_m"].max())}

    # 2 — accuracy and cost vs tolerance (two panels, never a dual axis).
    sweep = read(source / "tolerance_sweep.csv")
    fig, (left, right) = figure(11, 4.2, 2)
    left.loglog(sweep["relative_tolerance"], sweep["encke_error_m"], "o-", color=SERIES[0], linewidth=1.6,
                markersize=5, label="Encke")
    left.loglog(sweep["relative_tolerance"], sweep["cowell_error_m"], "s-", color=SERIES[1], linewidth=1.6,
                markersize=5, label="Cowell")
    left.set_xlabel("relative tolerance")
    left.set_ylabel("position error after 20 days (m)")
    legend(left, loc="upper left")
    style(left, "Accuracy vs tolerance")
    right.loglog(sweep["encke_error_m"], sweep["encke_seconds"] * 1e3, "o-", color=SERIES[0], linewidth=1.6,
                 markersize=5, label="Encke")
    right.loglog(sweep["cowell_error_m"], sweep["cowell_seconds"] * 1e3, "s-", color=SERIES[1], linewidth=1.6,
                 markersize=5, label="Cowell")
    right.invert_xaxis()
    right.set_xlabel("position error after 20 days (m) → more accurate")
    right.set_ylabel("wall time for 20 simulated days (ms)")
    legend(right, loc="upper left")
    style(right, "Cost vs accuracy (lower-right is better)")
    fig.tight_layout()
    fig.savefig(output / "fig2_tolerance_sweep.png", dpi=150)
    plt.close(fig)
    summary["tolerance_sweep"] = {key: values.tolist() for key, values in sweep.items()}

    # 3 — L4: libration in the co-rotating frame, the Jacobi integral, and an independent check:
    # the classic non-dimensional rotating-frame CR3BP equations integrated with SciPy's DOP853.
    l4 = read(source / "l4_libration.csv")
    mean_motion = np.sqrt((3.986004354e14 + 4.902800066e12) / SEPARATION_M**3)
    mu = MOON_MASS_RATIO

    def cr3bp(_, state):
        x, y, z, vx, vy, vz = state
        r1 = np.sqrt((x + mu) ** 2 + y**2 + z**2)
        r2 = np.sqrt((x - 1 + mu) ** 2 + y**2 + z**2)
        return [vx, vy, vz,
                2 * vy + x - (1 - mu) * (x + mu) / r1**3 - mu * (x - 1 + mu) / r2**3,
                -2 * vx + y - (1 - mu) * y / r1**3 - mu * y / r2**3,
                -(1 - mu) * z / r1**3 - mu * z / r2**3]

    start = [l4["rotating_x_m"][0] / SEPARATION_M, l4["rotating_y_m"][0] / SEPARATION_M, 0, 0, 0, 0]
    tau = l4["time_days"] * 86400.0 * mean_motion
    independent = solve_ivp(cr3bp, (0, tau[-1]), start, method="DOP853", rtol=1e-13, atol=1e-15, t_eval=tau)
    difference_km = np.hypot(independent.y[0] * SEPARATION_M - l4["rotating_x_m"],
                             independent.y[1] * SEPARATION_M - l4["rotating_y_m"]) / 1e3

    fig, (left, middle, right) = plt.subplots(1, 3, figsize=(14, 4.4), facecolor=SURFACE)
    l4_x, l4_y = (0.5 - MOON_MASS_RATIO) * SEPARATION_M, np.sqrt(3) / 2 * SEPARATION_M
    left.plot((l4["rotating_x_m"] - l4_x) / 1e3, (l4["rotating_y_m"] - l4_y) / 1e3, color=SERIES[0], linewidth=0.8)
    left.plot(0, 0, "o", color=INK, markersize=4)
    left.annotate("L4", (0, 0), xytext=(5, 5), textcoords="offset points", color=INK_2, fontsize=8)
    left.set_aspect("equal")
    left.set_xlabel("x − x(L4), co-rotating (km)")
    left.set_ylabel("y − y(L4) (km)")
    style(left, "5 years around L4 (start 1,000 km off)")
    middle.plot(l4["time_days"] / 365.25, l4["jacobi_relative_drift"], color=SERIES[0], linewidth=1.0)
    middle.set_xlabel("years")
    middle.set_ylabel("Jacobi constant, relative change")
    style(middle, "Jacobi integral (CR3BP invariant)")
    right.semilogy(l4["time_days"] / 365.25, np.maximum(difference_km * 1e3, 1e-6), color=SERIES[0], linewidth=1.0)
    right.set_xlabel("years")
    right.set_ylabel("|Helios − independent CR3BP| (m, log)")
    style(right, "vs rotating-frame CR3BP (SciPy DOP853)")
    fig.tight_layout()
    fig.savefig(output / "fig3_l4_libration.png", dpi=150)
    plt.close(fig)
    summary["l4"] = {"max_jacobi_relative_drift": float(np.abs(l4["jacobi_relative_drift"]).max()),
                     "max_distance_from_l4_km": float(np.max(np.hypot(l4["rotating_x_m"] - l4_x,
                                                                      l4["rotating_y_m"] - l4_y)) / 1e3),
                     "max_difference_vs_independent_m": float(difference_km.max() * 1e3),
                     "difference_vs_independent_after_1_year_m": float(
                         difference_km[np.argmin(np.abs(l4["time_days"] - 365.25))] * 1e3)}

    # 4 — L1: the growth rate is physics, the seed is numerics.
    l1 = read(source / "l1_divergence.csv")
    expected_rate_per_day = float(l1["expected_rate_per_day"][0])
    fig, axes = figure(10, 4.4)
    fitted = {}
    for slot, seed in enumerate(np.unique(l1["seed_offset_m"])):
        mask = l1["seed_offset_m"] == seed
        days, distance = l1["time_days"][mask], l1["distance_from_l1_m"][mask]
        axes.semilogy(days, distance, color=SERIES[slot], linewidth=1.6, label=f"seed offset {seed:g} m")
        # Linear regime only: from 10× the seed until the first time the orbit reaches 10,000 km
        # (after that it is nonlinear, and may even come back below the threshold).
        first_exit = int(np.argmax(distance >= 1e7)) if np.any(distance >= 1e7) else distance.size
        linear = (distance > 10 * seed) & (np.arange(distance.size) < first_exit)
        slope, _ = np.polyfit(days[linear], np.log(distance[linear]), 1)
        fitted[f"{seed:g}"] = float(slope)
    reference_days = np.linspace(0, 60, 50)
    for slot, seed in enumerate(np.unique(l1["seed_offset_m"])):
        axes.semilogy(reference_days, seed * np.exp(expected_rate_per_day * reference_days), "--", color=INK_3,
                      linewidth=0.9, label="linear theory e^{λt}" if slot == 0 else None)
    axes.set_ylim(1e-4, 1e9)
    axes.set_xlabel("days")
    axes.set_ylabel("distance from Earth–Moon L1 (m, log)")
    legend(axes, loc="lower right")
    style(axes, f"L1 instability: e-folding time {1 / expected_rate_per_day:.2f} d (theory); seeds only shift the curve")
    fig.tight_layout()
    fig.savefig(output / "fig4_l1_divergence.png", dpi=150)
    plt.close(fig)
    summary["l1"] = {"expected_rate_per_day": expected_rate_per_day, "fitted_rate_per_day_by_seed": fitted}

    # 5 — domain switch Moon → Earth.
    switch = read(source / "domain_switch.csv")
    fig, (left, right) = figure(11, 4.4, 2)
    in_moon = switch["domain_is_moon"] == 1
    left.plot(switch["earth_frame_x_m"][in_moon] / 1e3, switch["earth_frame_y_m"][in_moon] / 1e3, color=SERIES[0],
              linewidth=2.0, label="Moon domain")
    left.plot(switch["earth_frame_x_m"][~in_moon] / 1e3, switch["earth_frame_y_m"][~in_moon] / 1e3, color=SERIES[1],
              linewidth=2.0, label="Earth domain")
    left.set_xlabel("x, Earth frame (km)")
    left.set_ylabel("y (km)")
    legend(left, loc="upper left")
    style(left, "Escape from the Moon: coordinates re-anchored at the SOI")
    right.semilogy(switch["time_hours"], np.maximum(switch["encke_minus_cowell_m"], 1e-9), color=SERIES[0],
                   linewidth=1.6)
    switch_hour = float(switch["time_hours"][~in_moon][0])
    right.axvline(switch_hour, color=INK_3, linewidth=0.9, linestyle="--")
    right.annotate("domain change", (switch_hour, right.get_ylim()[1]), xytext=(4, -12), textcoords="offset points",
                   color=INK_2, fontsize=8)
    right.set_xlabel("hours")
    right.set_ylabel("distance from Cowell in Earth frame (m, log)")
    style(right, "Continuous across the switch (re-anchoring costs ~40 µm)")
    fig.tight_layout()
    fig.savefig(output / "fig5_domain_switch.png", dpi=150)
    plt.close(fig)
    summary["domain_switch"] = {"switch_hour": switch_hour,
                                "max_encke_minus_cowell_m": float(switch["encke_minus_cowell_m"].max())}

    # 6 — opening angle: accuracy vs θ at several places.
    angle = read(source / "opening_angle.csv")
    with (source / "opening_angle_probes.csv").open() as stream:
        probe_names = {int(row["probe_index"]): row["name"] for row in csv.DictReader(stream)}
    fig, axes = figure(10, 4.4)
    for slot, (index, name) in enumerate(sorted(probe_names.items())):
        mask = angle["probe_index"] == index
        axes.semilogy(angle["opening_angle"][mask], np.maximum(angle["relative_error_vs_theta0"][mask], 1e-17), "o-",
                      color=SERIES[slot], linewidth=1.4, markersize=4, label=name)
    axes.axvline(0.25, color=INK_3, linewidth=0.9, linestyle="--")
    axes.annotate("default θ = 0.25", (0.25, 1e-9), xytext=(4, 0), textcoords="offset points", color=INK_2,
                  fontsize=8)
    axes.set_xlabel("opening angle θ")
    axes.set_ylabel("relative acceleration error vs θ = 0 (log; 1e-17 = exact)")
    legend(axes, loc="center right")
    style(axes, "Tree-code fidelity knob: which subsystems may be merged into one point mass")
    fig.tight_layout()
    fig.savefig(output / "fig6_opening_angle.png", dpi=150)
    plt.close(fig)
    summary["opening_angle_mean_microseconds"] = float(np.mean(angle["microseconds_per_evaluation"]))

    warp = read(source / "warp_invariance.csv")
    summary["warp_invariance"] = {key: values.tolist() for key, values in warp.items()}
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    for name in ("warp_invariance.csv", "tolerance_sweep.csv"):
        (output / name).write_text((source / name).read_text())
    print(json.dumps({k: v for k, v in summary.items() if k not in ("tolerance_sweep", "warp_invariance")}, indent=1))


if __name__ == "__main__":
    main()
