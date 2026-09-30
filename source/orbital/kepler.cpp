#include "helios/orbital/kepler.hpp"

#include <cmath>
#include <format>

namespace helios::orbital {

namespace {

using core::ErrorCode;
using math::Vector3;

// Below these, an orbit is treated as circular / equatorial for the angle conventions.
constexpr double k_circular_eccentricity = 1e-11;
constexpr double k_equatorial_sine = 1e-11;

constexpr int k_max_kepler_iterations = 64;
constexpr double k_kepler_tolerance_rad = 1e-15;

[[nodiscard]] bool all_finite(const KeplerianElements& elements) noexcept {
    return std::isfinite(elements.semi_major_axis_m) && std::isfinite(elements.eccentricity)
           && std::isfinite(elements.inclination_rad)
           && std::isfinite(elements.longitude_of_ascending_node_rad)
           && std::isfinite(elements.argument_of_periapsis_rad) && std::isfinite(elements.mean_anomaly_rad);
}

[[nodiscard]] bool all_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

// Perifocal → reference frame: R3(−Ω) · R1(−i) · R3(−ω).
[[nodiscard]] Vector3 rotate_perifocal_to_reference(const Vector3& perifocal,
                                                    const KeplerianElements& elements) noexcept {
    const double cos_node = std::cos(elements.longitude_of_ascending_node_rad);
    const double sin_node = std::sin(elements.longitude_of_ascending_node_rad);
    const double cos_inclination = std::cos(elements.inclination_rad);
    const double sin_inclination = std::sin(elements.inclination_rad);
    const double cos_periapsis = std::cos(elements.argument_of_periapsis_rad);
    const double sin_periapsis = std::sin(elements.argument_of_periapsis_rad);

    const double row0_col0 = cos_node * cos_periapsis - sin_node * sin_periapsis * cos_inclination;
    const double row0_col1 = -cos_node * sin_periapsis - sin_node * cos_periapsis * cos_inclination;
    const double row1_col0 = sin_node * cos_periapsis + cos_node * sin_periapsis * cos_inclination;
    const double row1_col1 = -sin_node * sin_periapsis + cos_node * cos_periapsis * cos_inclination;
    const double row2_col0 = sin_periapsis * sin_inclination;
    const double row2_col1 = cos_periapsis * sin_inclination;

    // The perifocal z component is always zero for a point on the orbit.
    return {row0_col0 * perifocal.x + row0_col1 * perifocal.y,
            row1_col0 * perifocal.x + row1_col1 * perifocal.y,
            row2_col0 * perifocal.x + row2_col1 * perifocal.y};
}

// Signed angle from `from` to `to`, measured about `axis_unit` (both vectors ⟂ axis).
[[nodiscard]] double signed_angle_rad(const Vector3& from, const Vector3& to,
                                      const Vector3& axis_unit) noexcept {
    return std::atan2(math::dot(math::cross(from, to), axis_unit), math::dot(from, to));
}

} // namespace

double wrap_angle_rad(double angle_rad) noexcept {
    const double wrapped_rad = std::fmod(angle_rad, k_two_pi);
    return wrapped_rad < 0.0 ? wrapped_rad + k_two_pi : wrapped_rad;
}

core::Result<double> solve_kepler_equation(double mean_anomaly_rad, double eccentricity) noexcept {
    if (!std::isfinite(mean_anomaly_rad) || !std::isfinite(eccentricity)) {
        return core::fail(ErrorCode::NotFinite, "Kepler equation input is not finite");
    }
    if (eccentricity < 0.0 || eccentricity >= 1.0) {
        return core::fail(ErrorCode::OutOfRange,
                          std::format("eccentricity {} is not in [0, 1)", eccentricity));
    }

    // Solve on M ∈ [−π, π) where the starter is reliable, then restore the full turns.
    const double reduced_mean_anomaly_rad = wrap_angle_rad(mean_anomaly_rad + k_pi) - k_pi;
    const double removed_turns_rad = mean_anomaly_rad - reduced_mean_anomaly_rad;

    // Danby (1987) starter: E₀ = M + 0.85·e·sign(sin M).
    const double sign = std::sin(reduced_mean_anomaly_rad) >= 0.0 ? 1.0 : -1.0;
    double eccentric_anomaly_rad = reduced_mean_anomaly_rad + 0.85 * eccentricity * sign;

    // Halley iteration: cubic convergence and robust for e → 1 near periapsis.
    for (int iteration = 0; iteration < k_max_kepler_iterations; ++iteration) {
        const double sin_e = std::sin(eccentric_anomaly_rad);
        const double cos_e = std::cos(eccentric_anomaly_rad);
        const double residual_rad = eccentric_anomaly_rad - eccentricity * sin_e - reduced_mean_anomaly_rad;
        const double first_derivative = 1.0 - eccentricity * cos_e;
        const double second_derivative = eccentricity * sin_e;
        const double step_rad =
            residual_rad / (first_derivative - 0.5 * residual_rad * second_derivative / first_derivative);
        eccentric_anomaly_rad -= step_rad;
        if (std::abs(step_rad) <= k_kepler_tolerance_rad) {
            return eccentric_anomaly_rad + removed_turns_rad;
        }
    }
    return core::fail(ErrorCode::OutOfRange, std::format("Kepler equation did not converge (M = {}, e = {})",
                                                         mean_anomaly_rad, eccentricity));
}

core::Result<StateVector> state_from_elements(const KeplerianElements& elements,
                                              double gravitational_parameter_m3_s2) noexcept {
    if (!all_finite(elements) || !std::isfinite(gravitational_parameter_m3_s2)) {
        return core::fail(ErrorCode::NotFinite, "orbital elements are not finite");
    }
    if (elements.semi_major_axis_m <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "semi-major axis must be positive for a bound orbit");
    }
    if (gravitational_parameter_m3_s2 <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "gravitational parameter must be positive");
    }

    return solve_kepler_equation(elements.mean_anomaly_rad, elements.eccentricity)
        .transform([&](double eccentric_anomaly_rad) {
            const double eccentricity = elements.eccentricity;
            const double semi_major_axis_m = elements.semi_major_axis_m;
            const double cos_e = std::cos(eccentric_anomaly_rad);
            const double sin_e = std::sin(eccentric_anomaly_rad);
            const double semi_minor_ratio = std::sqrt((1.0 - eccentricity) * (1.0 + eccentricity));
            const double radius_m = semi_major_axis_m * (1.0 - eccentricity * cos_e);
            const double speed_scale_m_s =
                std::sqrt(gravitational_parameter_m3_s2 * semi_major_axis_m) / radius_m;

            const Vector3 perifocal_position_m{semi_major_axis_m * (cos_e - eccentricity),
                                               semi_major_axis_m * semi_minor_ratio * sin_e, 0.0};
            const Vector3 perifocal_velocity_m_s{-speed_scale_m_s * sin_e,
                                                 speed_scale_m_s * semi_minor_ratio * cos_e, 0.0};
            return StateVector{.position_m = rotate_perifocal_to_reference(perifocal_position_m, elements),
                               .velocity_m_s =
                                   rotate_perifocal_to_reference(perifocal_velocity_m_s, elements)};
        });
}

core::Result<KeplerianElements> elements_from_state(const StateVector& state,
                                                    double gravitational_parameter_m3_s2) noexcept {
    const double mu_m3_s2 = gravitational_parameter_m3_s2;
    if (!all_finite(state.position_m) || !all_finite(state.velocity_m_s) || !std::isfinite(mu_m3_s2)) {
        return core::fail(ErrorCode::NotFinite, "state vector is not finite");
    }
    if (mu_m3_s2 <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "gravitational parameter must be positive");
    }

    const Vector3& position_m = state.position_m;
    const Vector3& velocity_m_s = state.velocity_m_s;
    const double radius_m = math::norm(position_m);
    const Vector3 angular_momentum_m2_s = math::cross(position_m, velocity_m_s);
    const double angular_momentum_norm_m2_s = math::norm(angular_momentum_m2_s);
    if (radius_m == 0.0 || angular_momentum_norm_m2_s <= 1e-12 * radius_m * math::norm(velocity_m_s)) {
        return core::fail(ErrorCode::InvalidArgument,
                          "degenerate (radial or zero) state has no orbital plane");
    }

    const double speed_squared_m2_s2 = math::squared_norm(velocity_m_s);
    const double specific_energy_m2_s2 = 0.5 * speed_squared_m2_s2 - mu_m3_s2 / radius_m;
    if (specific_energy_m2_s2 >= 0.0) {
        return core::fail(ErrorCode::OutOfRange,
                          "state is unbound (e ≥ 1); only elliptic orbits are supported");
    }

    const Vector3 normal_unit = angular_momentum_m2_s / angular_momentum_norm_m2_s;
    const Vector3 eccentricity_vector = ((speed_squared_m2_s2 - mu_m3_s2 / radius_m) * position_m
                                         - math::dot(position_m, velocity_m_s) * velocity_m_s)
                                        / mu_m3_s2;
    const double eccentricity = math::norm(eccentricity_vector);

    KeplerianElements elements;
    elements.semi_major_axis_m = -mu_m3_s2 / (2.0 * specific_energy_m2_s2);
    elements.eccentricity = eccentricity;
    elements.inclination_rad = std::atan2(std::hypot(normal_unit.x, normal_unit.y), normal_unit.z);

    // Node line n = ẑ × h.
    const Vector3 node_vector{-normal_unit.y, normal_unit.x, 0.0};
    const bool equatorial = math::norm(node_vector) < k_equatorial_sine;
    const bool circular = eccentricity < k_circular_eccentricity;
    // For equatorial orbits angles are measured from +x, with the sense of the motion.
    const double motion_sign = normal_unit.z >= 0.0 ? 1.0 : -1.0;

    elements.longitude_of_ascending_node_rad =
        equatorial ? 0.0 : wrap_angle_rad(std::atan2(node_vector.y, node_vector.x));

    double true_anomaly_rad = 0.0;
    if (!circular) {
        elements.argument_of_periapsis_rad =
            equatorial
                ? wrap_angle_rad(std::atan2(motion_sign * eccentricity_vector.y, eccentricity_vector.x))
                : wrap_angle_rad(signed_angle_rad(node_vector, eccentricity_vector, normal_unit));
        true_anomaly_rad = signed_angle_rad(eccentricity_vector, position_m, normal_unit);
    } else {
        elements.argument_of_periapsis_rad = 0.0;
        true_anomaly_rad = equatorial ? std::atan2(motion_sign * position_m.y, position_m.x)
                                      : signed_angle_rad(node_vector, position_m, normal_unit);
    }

    const double eccentric_anomaly_rad =
        2.0
        * std::atan2(std::sqrt(1.0 - eccentricity) * std::sin(0.5 * true_anomaly_rad),
                     std::sqrt(1.0 + eccentricity) * std::cos(0.5 * true_anomaly_rad));
    elements.mean_anomaly_rad =
        wrap_angle_rad(eccentric_anomaly_rad - eccentricity * std::sin(eccentric_anomaly_rad));
    return elements;
}

Vector3 two_body_acceleration_m_s2(const Vector3& position_m, double gravitational_parameter_m3_s2) noexcept {
    const double radius_m = math::norm(position_m);
    return position_m * (-gravitational_parameter_m3_s2 / (radius_m * radius_m * radius_m));
}

} // namespace helios::orbital
