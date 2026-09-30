#ifndef HELIOS_ORBITAL_KEPLER_HPP
#define HELIOS_ORBITAL_KEPLER_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"

#include <numbers>

namespace helios::orbital {

inline constexpr double k_pi = std::numbers::pi;
inline constexpr double k_two_pi = 2.0 * k_pi;

// Position and velocity of a point relative to some frame origin, in that frame's axes.
struct StateVector {
    math::Vector3 position_m;
    math::Vector3 velocity_m_s;
};

// Classical osculating elements of a bound (elliptic) orbit. Angles are radians in the
// frame's reference plane; for the stock Solar System that is the J2000 ecliptic.
struct KeplerianElements {
    double semi_major_axis_m = 0.0;
    double eccentricity = 0.0; // [0, 1)
    double inclination_rad = 0.0;
    double longitude_of_ascending_node_rad = 0.0;
    double argument_of_periapsis_rad = 0.0;
    double mean_anomaly_rad = 0.0;
};

// Wraps any finite angle into [0, 2π).
[[nodiscard]] double wrap_angle_rad(double angle_rad) noexcept;

// Eccentric anomaly E solving M = E − e·sin E, for 0 ≤ e < 1.
// Newton iteration from Danby's starter; converges to |residual| ≲ 4 ulp(π) for all M, e.
[[nodiscard]] core::Result<double> solve_kepler_equation(double mean_anomaly_rad,
                                                         double eccentricity) noexcept;

// Cartesian state from elliptic elements. Fails on a ≤ 0, e ∉ [0, 1), μ ≤ 0 or non-finite input.
[[nodiscard]] core::Result<StateVector> state_from_elements(const KeplerianElements& elements,
                                                            double gravitational_parameter_m3_s2) noexcept;

// Osculating elliptic elements from a state. Fails for unbound (e ≥ 1) or degenerate
// (r = 0, h = 0) states. Conventions for undefined angles: Ω = 0 when i = 0; ω = 0 when e = 0
// (the anomaly is then measured from the node, or from +x when also equatorial).
[[nodiscard]] core::Result<KeplerianElements>
elements_from_state(const StateVector& state, double gravitational_parameter_m3_s2) noexcept;

// Two-body propagation of any conic (elliptic, parabolic, hyperbolic) by `elapsed_s`, which may
// be negative, using universal variables (Battin §4.5) and Laguerre–Conway iteration.
// Elliptic spans longer than one period are reduced modulo the period first.
// Fails on a degenerate state (r = 0), μ ≤ 0 or non-convergence.
[[nodiscard]] core::Result<StateVector> propagate_conic(const StateVector& initial_state,
                                                        double gravitational_parameter_m3_s2,
                                                        double elapsed_s) noexcept;

// Stumpff functions C(z) = (1 − cos √z)/z and S(z) = (√z − sin √z)/√z³, continued to z ≤ 0,
// evaluated by series near z = 0 to avoid cancellation.
[[nodiscard]] double stumpff_c(double z) noexcept;
[[nodiscard]] double stumpff_s(double z) noexcept;

// Newtonian point-mass acceleration −μ r/|r|³.
[[nodiscard]] math::Vector3 two_body_acceleration_m_s2(const math::Vector3& position_m,
                                                       double gravitational_parameter_m3_s2) noexcept;

} // namespace helios::orbital

#endif // HELIOS_ORBITAL_KEPLER_HPP
