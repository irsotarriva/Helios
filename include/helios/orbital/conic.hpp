#ifndef HELIOS_ORBITAL_CONIC_HPP
#define HELIOS_ORBITAL_CONIC_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace helios::orbital {

// The geometry of the osculating two-body conic through a state, valid for every eccentricity
// (ellipse, parabola, hyperbola). Unlike KeplerianElements it has no singular angles: the orbit
// is described by the perifocal basis (P towards periapsis, W along the angular momentum,
// Q = W × P). For a circular orbit P points at the current position.
struct ConicGeometry {
    double gravitational_parameter_m3_s2 = 0.0;
    double eccentricity = 0.0;
    double semi_latus_rectum_m = 0.0; // p = h²/μ
    double periapsis_radius_m = 0.0;  // p / (1 + e)
    double apoapsis_radius_m = 0.0;   // p / (1 − e); +inf when e ≥ 1
    double true_anomaly_rad = 0.0;    // current ν, in (−π, π]
    math::Vector3 periapsis_unit;     // P
    math::Vector3 normal_unit;        // W
    math::Vector3 in_plane_unit;      // Q = W × P

    [[nodiscard]] bool is_bound() const noexcept { return eccentricity < 1.0; }
};

// Fails for a degenerate state (r = 0 or a purely radial motion, which has no plane), μ ≤ 0 or
// non-finite input.
[[nodiscard]] core::Result<ConicGeometry> conic_geometry(const StateVector& state,
                                                         double gravitational_parameter_m3_s2) noexcept;

// r(ν) = p / (1 + e cos ν) along P cos ν + Q sin ν. The caller keeps ν inside the conic's range
// (|ν| < acos(−1/e) for e ≥ 1).
[[nodiscard]] math::Vector3 conic_position_at_true_anomaly(const ConicGeometry& conic,
                                                           double true_anomaly_rad) noexcept;

// The largest |ν| the conic reaches: π for an ellipse, acos(−1/e) (the asymptote) otherwise.
[[nodiscard]] double true_anomaly_limit_rad(double eccentricity) noexcept;

// Time from periapsis to true anomaly ν (negative before periapsis), by Kepler's equation in
// its elliptic, hyperbolic or parabolic (Barker) form. For an ellipse the result is in
// (−T/2, T/2].
[[nodiscard]] double time_since_periapsis_s(const ConicGeometry& conic, double true_anomaly_rad) noexcept;

// Orbital period; +inf for unbound conics.
[[nodiscard]] double orbital_period_s(const ConicGeometry& conic) noexcept;

// Time from the conic's current point until it next reaches periapsis. None if the orbit is
// (numerically) circular, or unbound and already past periapsis. Never 0: a state exactly at
// periapsis reports the *next* passage (a full period ahead, or none when unbound).
[[nodiscard]] std::optional<double> time_to_periapsis_s(const ConicGeometry& conic) noexcept;

// Same for apoapsis; none for unbound or circular orbits.
[[nodiscard]] std::optional<double> time_to_apoapsis_s(const ConicGeometry& conic) noexcept;

enum class RadialDirection {
    Outbound, // r increasing through the target radius (leaving a sphere of influence)
    Inbound,  // r decreasing through it (hitting a surface)
};

// Time until the conic next crosses `radius_m` in `direction`. None when the conic never
// reaches that radius (or only in the past, for an unbound conic).
[[nodiscard]] std::optional<double> time_to_radius_s(const ConicGeometry& conic, double radius_m,
                                                     RadialDirection direction) noexcept;

struct ConicArc {
    std::vector<math::Vector3> points_m; // relative to the focus, in the state's axes
    bool closed = false;                 // true when the whole ellipse fits inside the clip radius
};

// Polyline of the conic for display, clipped to r ≤ clip_radius_m (use the sphere of influence).
// Points are spaced uniformly in eccentric (or hyperbolic) anomaly, which concentrates them
// where the curvature is highest; for a closed ellipse the last point is not a repeat of the
// first. Fails for fewer than 4 points or a clip radius below the periapsis.
[[nodiscard]] core::Result<ConicArc> sample_conic(const ConicGeometry& conic, double clip_radius_m,
                                                  std::size_t point_count);

// Unit vectors of the velocity–normal–binormal frame used for manoeuvres (KSP-style
// "prograde / normal / radial-out"): V along the velocity, N along r × v, B = V × N, which
// points away from the centre on a circular orbit.
struct ManeuverBasis {
    math::Vector3 prograde_unit;
    math::Vector3 normal_unit;
    math::Vector3 radial_out_unit;
};

// Fails for zero velocity or a radial trajectory.
[[nodiscard]] core::Result<ManeuverBasis> maneuver_basis(const StateVector& state) noexcept;

// Δv in the inertial axes of the state's frame from its prograde / normal / radial-out components.
[[nodiscard]] math::Vector3 maneuver_to_inertial(const ManeuverBasis& basis, double prograde_m_s,
                                                 double normal_m_s, double radial_out_m_s) noexcept;

} // namespace helios::orbital

#endif // HELIOS_ORBITAL_CONIC_HPP
