#include "helios/orbital/kepler.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

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

double stumpff_c(double z) noexcept {
    if (std::abs(z) < 0.1) {
        // Σ (−z)^k / (2k + 2)!
        double term = 0.5;
        double sum = term;
        for (int k = 1; k < 9; ++k) {
            term *= -z / static_cast<double>((2 * k + 1) * (2 * k + 2));
            sum += term;
        }
        return sum;
    }
    if (z > 0.0) {
        const double root = std::sqrt(z);
        return (1.0 - std::cos(root)) / z;
    }
    const double root = std::sqrt(-z);
    return (std::cosh(root) - 1.0) / -z;
}

double stumpff_s(double z) noexcept {
    if (std::abs(z) < 0.1) {
        // Σ (−z)^k / (2k + 3)!
        double term = 1.0 / 6.0;
        double sum = term;
        for (int k = 1; k < 9; ++k) {
            term *= -z / static_cast<double>((2 * k + 2) * (2 * k + 3));
            sum += term;
        }
        return sum;
    }
    if (z > 0.0) {
        const double root = std::sqrt(z);
        return (root - std::sin(root)) / (root * root * root);
    }
    const double root = std::sqrt(-z);
    return (std::sinh(root) - root) / (root * root * root);
}

core::Result<StateVector> propagate_conic(const StateVector& initial_state,
                                          double gravitational_parameter_m3_s2, double elapsed_s) noexcept {
    const double mu_m3_s2 = gravitational_parameter_m3_s2;
    if (!all_finite(initial_state.position_m) || !all_finite(initial_state.velocity_m_s)
        || !std::isfinite(mu_m3_s2) || !std::isfinite(elapsed_s)) {
        return core::fail(ErrorCode::NotFinite, "conic propagation input is not finite");
    }
    if (mu_m3_s2 <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "gravitational parameter must be positive");
    }
    const Vector3& position_m = initial_state.position_m;
    const Vector3& velocity_m_s = initial_state.velocity_m_s;
    const double radius_m = math::norm(position_m);
    if (radius_m == 0.0) {
        return core::fail(ErrorCode::InvalidArgument, "cannot propagate from the attracting centre");
    }
    if (elapsed_s == 0.0) {
        return initial_state;
    }

    const double sqrt_mu = std::sqrt(mu_m3_s2);
    const double radial_speed_term = math::dot(position_m, velocity_m_s) / sqrt_mu; // r·v/√μ
    const double inverse_semi_major_axis_per_m =
        2.0 / radius_m - math::squared_norm(velocity_m_s) / mu_m3_s2; // α

    // Bound orbits: reduce to less than one period; the solution is periodic.
    double time_s = elapsed_s;
    if (inverse_semi_major_axis_per_m > 0.0) {
        const double semi_major_axis_m = 1.0 / inverse_semi_major_axis_per_m;
        const double period_s =
            k_two_pi * std::sqrt(semi_major_axis_m * semi_major_axis_m * semi_major_axis_m / mu_m3_s2);
        time_s = std::fmod(elapsed_s, period_s);
        if (time_s == 0.0) {
            return initial_state;
        }
    }

    // Universal anomaly χ [√m]; starters from Vallado (2013) §2.3.
    double chi = sqrt_mu * time_s / radius_m;
    if (inverse_semi_major_axis_per_m > 1e-12 / radius_m) {
        chi = sqrt_mu * inverse_semi_major_axis_per_m * time_s;
    } else if (inverse_semi_major_axis_per_m < -1e-12 / radius_m) {
        const double semi_major_axis_m = 1.0 / inverse_semi_major_axis_per_m; // negative
        const double direction = time_s >= 0.0 ? 1.0 : -1.0;
        const double argument = (-2.0 * mu_m3_s2 * inverse_semi_major_axis_per_m * time_s)
                                / (math::dot(position_m, velocity_m_s)
                                   + direction * std::sqrt(-mu_m3_s2 * semi_major_axis_m)
                                         * (1.0 - radius_m * inverse_semi_major_axis_per_m));
        if (argument > 0.0 && std::isfinite(argument)) {
            chi = direction * std::sqrt(-semi_major_axis_m) * std::log(argument);
        }
    }
    const double one_minus_alpha_r = 1.0 - inverse_semi_major_axis_per_m * radius_m;
    constexpr int k_max_iterations = 100;
    constexpr double k_laguerre_order = 5.0;
    bool converged = false;
    double last_relative_residual = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < k_max_iterations; ++iteration) {
        const double chi_squared = chi * chi;
        const double z = inverse_semi_major_axis_per_m * chi_squared;
        const double c_value = stumpff_c(z);
        const double s_value = stumpff_s(z);
        const double function = radial_speed_term * chi_squared * c_value
                                + one_minus_alpha_r * chi_squared * chi * s_value + radius_m * chi
                                - sqrt_mu * time_s;
        const double first = radial_speed_term * chi * (1.0 - z * s_value)
                             + one_minus_alpha_r * chi_squared * c_value + radius_m;
        const double second =
            radial_speed_term * (1.0 - z * c_value) + one_minus_alpha_r * chi * (1.0 - z * s_value);
        const double discriminant =
            std::abs(((k_laguerre_order - 1.0) * (k_laguerre_order - 1.0) * first * first)
                     - (k_laguerre_order * (k_laguerre_order - 1.0) * function * second));
        const double denominator = first + std::copysign(std::sqrt(discriminant), first);
        const double step = k_laguerre_order * function / denominator;
        if (!std::isfinite(function) || !std::isfinite(step)) {
            // Overshot into cosh/sinh overflow: back off towards zero and retry.
            chi *= 0.5;
            continue;
        }
        // Rationale: two convergence tests, because the rounding floor depends on the platform's
        // libm (Apple's sinh/cosh differ from glibc's in the last bits):
        //  * the residual is within a few ulps of the magnitude of its terms, or
        //  * the (cubically convergent) step is below 1e-13·|χ|, so the step being applied now
        //    brings χ to rounding level.
        const double residual_scale = std::abs(radial_speed_term * chi_squared * c_value)
                                      + std::abs(one_minus_alpha_r * chi_squared * chi * s_value)
                                      + std::abs(radius_m * chi) + std::abs(sqrt_mu * time_s);
        last_relative_residual = std::abs(function) / residual_scale;
        if (last_relative_residual <= 64.0 * std::numeric_limits<double>::epsilon()) {
            converged = true;
            break;
        }
        chi -= step;
        if (std::abs(step) <= 1e-13 * std::max(1.0, std::abs(chi))) {
            converged = true;
            break;
        }
    }
    if (!converged || !std::isfinite(chi)) {
        return core::fail(
            ErrorCode::OutOfRange,
            std::format("universal Kepler equation did not converge (Δt = {} s, χ = {}, relative "
                        "residual {:.3g})",
                        time_s, chi, last_relative_residual));
    }

    const double chi_squared = chi * chi;
    const double z = inverse_semi_major_axis_per_m * chi_squared;
    const double c_value = stumpff_c(z);
    const double s_value = stumpff_s(z);
    const double f = 1.0 - chi_squared / radius_m * c_value;
    const double g = time_s - chi_squared * chi / sqrt_mu * s_value;
    const Vector3 new_position_m = f * position_m + g * velocity_m_s;
    const double new_radius_m = math::norm(new_position_m);
    const double f_dot = sqrt_mu / (new_radius_m * radius_m) * (z * s_value - 1.0) * chi;
    const double g_dot = 1.0 - chi_squared / new_radius_m * c_value;
    return StateVector{.position_m = new_position_m,
                       .velocity_m_s = f_dot * position_m + g_dot * velocity_m_s};
}

Vector3 two_body_acceleration_m_s2(const Vector3& position_m, double gravitational_parameter_m3_s2) noexcept {
    const double radius_m = math::norm(position_m);
    return position_m * (-gravitational_parameter_m3_s2 / (radius_m * radius_m * radius_m));
}

} // namespace helios::orbital
