"""NumPy port of the Helios secular-element model (source/ephemeris/keplerian_ephemeris.cpp).

Used by the fitter, which needs a vectorised model; the validation always evaluates the
C++ implementation through helios_ephemeris_probe, never this port.
"""

from __future__ import annotations

import numpy as np

PARAMETER_NAMES = [
    "semi_major_axis_m", "semi_major_axis_rate_m_s",
    "eccentricity", "eccentricity_rate_per_s",
    "inclination_rad", "inclination_rate_rad_s",
    "longitude_of_ascending_node_rad", "longitude_of_ascending_node_rate_rad_s",
    "longitude_of_periapsis_rad", "longitude_of_periapsis_rate_rad_s",
    "mean_longitude_rad", "mean_longitude_rate_rad_s",
]


def solve_kepler(mean_anomaly_rad: np.ndarray, eccentricity: np.ndarray) -> np.ndarray:
    reduced = np.remainder(mean_anomaly_rad + np.pi, 2.0 * np.pi) - np.pi
    eccentric_anomaly = reduced + 0.85 * eccentricity * np.sign(np.sin(reduced))
    for _ in range(50):
        sin_e, cos_e = np.sin(eccentric_anomaly), np.cos(eccentric_anomaly)
        residual = eccentric_anomaly - eccentricity * sin_e - reduced
        first = 1.0 - eccentricity * cos_e
        step = residual / (first - 0.5 * residual * eccentricity * sin_e / first)
        eccentric_anomaly = eccentric_anomaly - step
        if np.max(np.abs(step)) < 1e-15:
            break
    return eccentric_anomaly


def position_m(parameters: np.ndarray, elapsed_s: np.ndarray) -> np.ndarray:
    """Positions (N, 3) of the secular-element orbit at `elapsed_s` since the reference epoch."""
    a, a_dot, e, e_dot, i, i_dot, node, node_dot, peri, peri_dot, mean_lon, mean_lon_dot = parameters
    semi_major_axis = a + a_dot * elapsed_s
    eccentricity = e + e_dot * elapsed_s
    inclination = i + i_dot * elapsed_s
    node_rad = node + node_dot * elapsed_s
    periapsis_longitude = peri + peri_dot * elapsed_s
    mean_longitude = mean_lon + mean_lon_dot * elapsed_s
    argument_of_periapsis = periapsis_longitude - node_rad
    mean_anomaly = mean_longitude - periapsis_longitude

    eccentric_anomaly = solve_kepler(mean_anomaly, eccentricity)
    x_perifocal = semi_major_axis * (np.cos(eccentric_anomaly) - eccentricity)
    y_perifocal = semi_major_axis * np.sqrt(1.0 - eccentricity**2) * np.sin(eccentric_anomaly)

    cos_node, sin_node = np.cos(node_rad), np.sin(node_rad)
    cos_inc, sin_inc = np.cos(inclination), np.sin(inclination)
    cos_arg, sin_arg = np.cos(argument_of_periapsis), np.sin(argument_of_periapsis)
    x = (cos_node * cos_arg - sin_node * sin_arg * cos_inc) * x_perifocal + \
        (-cos_node * sin_arg - sin_node * cos_arg * cos_inc) * y_perifocal
    y = (sin_node * cos_arg + cos_node * sin_arg * cos_inc) * x_perifocal + \
        (-sin_node * sin_arg + cos_node * cos_arg * cos_inc) * y_perifocal
    z = (sin_arg * sin_inc) * x_perifocal + (cos_arg * sin_inc) * y_perifocal
    return np.stack([x, y, z], axis=1)


def osculating_elements(position: np.ndarray, velocity: np.ndarray, mu_m3_s2: float) -> dict[str, np.ndarray]:
    """Vectorised osculating elements (as longitudes) for an initial guess."""
    radius = np.linalg.norm(position, axis=1)
    h = np.cross(position, velocity)
    h_norm = np.linalg.norm(h, axis=1)
    energy = 0.5 * np.sum(velocity**2, axis=1) - mu_m3_s2 / radius
    a = -mu_m3_s2 / (2.0 * energy)
    e_vec = ((np.sum(velocity**2, axis=1) - mu_m3_s2 / radius)[:, None] * position
             - np.sum(position * velocity, axis=1)[:, None] * velocity) / mu_m3_s2
    e = np.linalg.norm(e_vec, axis=1)
    inclination = np.arctan2(np.hypot(h[:, 0], h[:, 1]), h[:, 2])
    node = np.arctan2(h[:, 0], -h[:, 1])
    node_vector = np.stack([np.cos(node), np.sin(node), np.zeros_like(node)], axis=1)
    normal = h / h_norm[:, None]
    argument = np.arctan2(np.sum(np.cross(node_vector, e_vec) * normal, axis=1), np.sum(node_vector * e_vec, axis=1))
    true_anomaly = np.arctan2(np.sum(np.cross(e_vec, position) * normal, axis=1), np.sum(e_vec * position, axis=1))
    eccentric = 2.0 * np.arctan2(np.sqrt(1 - e) * np.sin(true_anomaly / 2), np.sqrt(1 + e) * np.cos(true_anomaly / 2))
    mean_anomaly = eccentric - e * np.sin(eccentric)
    periapsis_longitude = node + argument
    return {"a": a, "e": e, "i": inclination, "node": node, "peri": periapsis_longitude,
            "mean_lon": periapsis_longitude + mean_anomaly}
