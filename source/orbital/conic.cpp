#include "helios/orbital/conic.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace helios::orbital {

namespace {

using core::ErrorCode;
using math::Vector3;

// Below this eccentricity the orbit has no meaningful periapsis direction.
constexpr double k_circular_eccentricity = 1e-9;
// Within this of e = 1 the elliptic/hyperbolic forms of Kepler's equation lose accuracy and the
// parabolic (Barker) form is used instead.
constexpr double k_parabolic_band = 1e-8;

[[nodiscard]] bool all_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool is_parabolic(double eccentricity) noexcept {
    return std::abs(eccentricity - 1.0) < k_parabolic_band;
}

// |a|, the (absolute) semi-major axis; +inf for a parabola.
[[nodiscard]] double semi_major_axis_magnitude_m(const ConicGeometry& conic) noexcept {
    return conic.semi_latus_rectum_m / std::abs(1.0 - conic.eccentricity * conic.eccentricity);
}

[[nodiscard]] double eccentric_anomaly_from_true_rad(double eccentricity, double true_anomaly_rad) noexcept {
    return 2.0
           * std::atan2(std::sqrt(1.0 - eccentricity) * std::sin(0.5 * true_anomaly_rad),
                        std::sqrt(1.0 + eccentricity) * std::cos(0.5 * true_anomaly_rad));
}

[[nodiscard]] double hyperbolic_anomaly_from_true(double eccentricity, double true_anomaly_rad) noexcept {
    return 2.0
           * std::atanh(std::sqrt((eccentricity - 1.0) / (eccentricity + 1.0))
                        * std::tan(0.5 * true_anomaly_rad));
}

[[nodiscard]] Vector3 perifocal_to_frame(const ConicGeometry& conic, double along_periapsis_m,
                                         double along_in_plane_m) noexcept {
    return along_periapsis_m * conic.periapsis_unit + along_in_plane_m * conic.in_plane_unit;
}

} // namespace

core::Result<ConicGeometry> conic_geometry(const StateVector& state,
                                           double gravitational_parameter_m3_s2) noexcept {
    const double mu_m3_s2 = gravitational_parameter_m3_s2;
    if (!all_finite(state.position_m) || !all_finite(state.velocity_m_s) || !std::isfinite(mu_m3_s2)) {
        return core::fail(ErrorCode::NotFinite, "conic geometry input is not finite");
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

    const Vector3 eccentricity_vector = ((math::squared_norm(velocity_m_s) - mu_m3_s2 / radius_m) * position_m
                                         - math::dot(position_m, velocity_m_s) * velocity_m_s)
                                        / mu_m3_s2;

    ConicGeometry conic;
    conic.gravitational_parameter_m3_s2 = mu_m3_s2;
    conic.eccentricity = math::norm(eccentricity_vector);
    conic.semi_latus_rectum_m = angular_momentum_norm_m2_s * angular_momentum_norm_m2_s / mu_m3_s2;
    conic.periapsis_radius_m = conic.semi_latus_rectum_m / (1.0 + conic.eccentricity);
    conic.apoapsis_radius_m = conic.eccentricity < 1.0
                                  ? conic.semi_latus_rectum_m / (1.0 - conic.eccentricity)
                                  : std::numeric_limits<double>::infinity();
    conic.normal_unit = angular_momentum_m2_s / angular_momentum_norm_m2_s;
    conic.periapsis_unit = conic.eccentricity > k_circular_eccentricity
                               ? eccentricity_vector / conic.eccentricity
                               : position_m / radius_m;
    conic.in_plane_unit = math::cross(conic.normal_unit, conic.periapsis_unit);
    conic.true_anomaly_rad =
        std::atan2(math::dot(position_m, conic.in_plane_unit), math::dot(position_m, conic.periapsis_unit));
    return conic;
}

Vector3 conic_position_at_true_anomaly(const ConicGeometry& conic, double true_anomaly_rad) noexcept {
    const double radius_m =
        conic.semi_latus_rectum_m / (1.0 + conic.eccentricity * std::cos(true_anomaly_rad));
    return perifocal_to_frame(conic, radius_m * std::cos(true_anomaly_rad),
                              radius_m * std::sin(true_anomaly_rad));
}

double true_anomaly_limit_rad(double eccentricity) noexcept {
    return eccentricity < 1.0 ? k_pi : std::acos(-1.0 / eccentricity);
}

double time_since_periapsis_s(const ConicGeometry& conic, double true_anomaly_rad) noexcept {
    const double eccentricity = conic.eccentricity;
    const double mu_m3_s2 = conic.gravitational_parameter_m3_s2;
    if (is_parabolic(eccentricity)) {
        // Barker's equation.
        const double tangent = std::tan(0.5 * true_anomaly_rad);
        const double semi_latus_rectum_m = conic.semi_latus_rectum_m;
        return 0.5 * std::sqrt(semi_latus_rectum_m * semi_latus_rectum_m * semi_latus_rectum_m / mu_m3_s2)
               * (tangent + tangent * tangent * tangent / 3.0);
    }
    const double semi_major_axis_m = semi_major_axis_magnitude_m(conic);
    const double mean_motion_rad_s =
        std::sqrt(mu_m3_s2 / (semi_major_axis_m * semi_major_axis_m * semi_major_axis_m));
    if (eccentricity < 1.0) {
        const double eccentric_anomaly_rad = eccentric_anomaly_from_true_rad(eccentricity, true_anomaly_rad);
        return (eccentric_anomaly_rad - eccentricity * std::sin(eccentric_anomaly_rad)) / mean_motion_rad_s;
    }
    const double hyperbolic_anomaly = hyperbolic_anomaly_from_true(eccentricity, true_anomaly_rad);
    return (eccentricity * std::sinh(hyperbolic_anomaly) - hyperbolic_anomaly) / mean_motion_rad_s;
}

double orbital_period_s(const ConicGeometry& conic) noexcept {
    if (conic.eccentricity >= 1.0) {
        return std::numeric_limits<double>::infinity();
    }
    const double semi_major_axis_m = semi_major_axis_magnitude_m(conic);
    return k_two_pi
           * std::sqrt(semi_major_axis_m * semi_major_axis_m * semi_major_axis_m
                       / conic.gravitational_parameter_m3_s2);
}

std::optional<double> time_to_periapsis_s(const ConicGeometry& conic) noexcept {
    if (conic.eccentricity < k_circular_eccentricity) {
        return std::nullopt;
    }
    const double since_periapsis_s = time_since_periapsis_s(conic, conic.true_anomaly_rad);
    if (!conic.is_bound()) {
        return since_periapsis_s < 0.0 ? std::optional<double>(-since_periapsis_s) : std::nullopt;
    }
    const double period_s = orbital_period_s(conic);
    double until_s = -since_periapsis_s;
    if (until_s <= 0.0) {
        until_s += period_s;
    }
    return until_s;
}

std::optional<double> time_to_apoapsis_s(const ConicGeometry& conic) noexcept {
    if (conic.eccentricity < k_circular_eccentricity || !conic.is_bound()) {
        return std::nullopt;
    }
    const double period_s = orbital_period_s(conic);
    double until_s = 0.5 * period_s - time_since_periapsis_s(conic, conic.true_anomaly_rad);
    if (until_s <= 0.0) {
        until_s += period_s;
    }
    return until_s;
}

std::optional<double> time_to_radius_s(const ConicGeometry& conic, double radius_m,
                                       RadialDirection direction) noexcept {
    if (!(radius_m > 0.0) || conic.eccentricity < k_circular_eccentricity
        || radius_m < conic.periapsis_radius_m || radius_m > conic.apoapsis_radius_m) {
        return std::nullopt;
    }
    const double cosine =
        std::clamp((conic.semi_latus_rectum_m / radius_m - 1.0) / conic.eccentricity, -1.0, 1.0);
    const double crossing_rad = std::acos(cosine);
    const double target_rad = direction == RadialDirection::Outbound ? crossing_rad : -crossing_rad;
    double until_s =
        time_since_periapsis_s(conic, target_rad) - time_since_periapsis_s(conic, conic.true_anomaly_rad);
    if (!conic.is_bound()) {
        return until_s > 0.0 ? std::optional<double>(until_s) : std::nullopt;
    }
    const double period_s = orbital_period_s(conic);
    until_s = std::fmod(until_s, period_s);
    if (until_s <= 0.0) {
        until_s += period_s;
    }
    return until_s;
}

core::Result<ConicArc> sample_conic(const ConicGeometry& conic, double clip_radius_m,
                                    std::size_t point_count) {
    if (point_count < 4) {
        return core::fail(ErrorCode::InvalidArgument, "a conic polyline needs at least 4 points");
    }
    if (!(clip_radius_m >= conic.periapsis_radius_m)) {
        return core::fail(ErrorCode::OutOfRange, std::format("clip radius {} m is below the periapsis {} m",
                                                             clip_radius_m, conic.periapsis_radius_m));
    }
    const double eccentricity = conic.eccentricity;
    const auto count = static_cast<double>(point_count);
    ConicArc arc;
    arc.points_m.reserve(point_count);

    if (conic.is_bound() && !is_parabolic(eccentricity) && conic.apoapsis_radius_m <= clip_radius_m) {
        const double semi_major_axis_m = semi_major_axis_magnitude_m(conic);
        const double semi_minor_axis_m =
            semi_major_axis_m * std::sqrt((1.0 - eccentricity) * (1.0 + eccentricity));
        for (std::size_t index = 0; index < point_count; ++index) {
            const double eccentric_anomaly_rad = k_two_pi * static_cast<double>(index) / count;
            arc.points_m.push_back(perifocal_to_frame(
                conic, semi_major_axis_m * (std::cos(eccentric_anomaly_rad) - eccentricity),
                semi_minor_axis_m * std::sin(eccentric_anomaly_rad)));
        }
        arc.closed = true;
        return arc;
    }

    // An arc symmetric about periapsis, from where the conic enters the clip sphere to where it
    // leaves it.
    const double clip_cosine =
        std::clamp((conic.semi_latus_rectum_m / clip_radius_m - 1.0) / eccentricity, -1.0, 1.0);
    const double clip_true_anomaly_rad = std::acos(clip_cosine);
    const double last = count - 1.0;
    for (std::size_t index = 0; index < point_count; ++index) {
        const double fraction = 2.0 * static_cast<double>(index) / last - 1.0; // −1 … 1
        if (is_parabolic(eccentricity)) {
            const double tangent = fraction * std::tan(0.5 * clip_true_anomaly_rad);
            const double semi_latus_rectum_m = conic.semi_latus_rectum_m;
            arc.points_m.push_back(perifocal_to_frame(
                conic, 0.5 * semi_latus_rectum_m * (1.0 - tangent * tangent), semi_latus_rectum_m * tangent));
        } else if (eccentricity < 1.0) {
            const double semi_major_axis_m = semi_major_axis_magnitude_m(conic);
            const double semi_minor_axis_m =
                semi_major_axis_m * std::sqrt((1.0 - eccentricity) * (1.0 + eccentricity));
            const double eccentric_anomaly_rad =
                fraction * eccentric_anomaly_from_true_rad(eccentricity, clip_true_anomaly_rad);
            arc.points_m.push_back(perifocal_to_frame(
                conic, semi_major_axis_m * (std::cos(eccentric_anomaly_rad) - eccentricity),
                semi_minor_axis_m * std::sin(eccentric_anomaly_rad)));
        } else {
            const double semi_major_axis_m = semi_major_axis_magnitude_m(conic);
            const double semi_minor_axis_m =
                semi_major_axis_m * std::sqrt((eccentricity - 1.0) * (eccentricity + 1.0));
            const double hyperbolic_anomaly =
                fraction * hyperbolic_anomaly_from_true(eccentricity, clip_true_anomaly_rad);
            arc.points_m.push_back(
                perifocal_to_frame(conic, semi_major_axis_m * (eccentricity - std::cosh(hyperbolic_anomaly)),
                                   semi_minor_axis_m * std::sinh(hyperbolic_anomaly)));
        }
    }
    return arc;
}

core::Result<ManeuverBasis> maneuver_basis(const StateVector& state) noexcept {
    const double speed_m_s = math::norm(state.velocity_m_s);
    const Vector3 angular_momentum_m2_s = math::cross(state.position_m, state.velocity_m_s);
    const double angular_momentum_norm_m2_s = math::norm(angular_momentum_m2_s);
    if (!(speed_m_s > 0.0) || !std::isfinite(speed_m_s)
        || angular_momentum_norm_m2_s <= 1e-12 * speed_m_s * math::norm(state.position_m)) {
        return core::fail(ErrorCode::InvalidArgument, "no manoeuvre frame for a zero or radial velocity");
    }
    ManeuverBasis basis;
    basis.prograde_unit = state.velocity_m_s / speed_m_s;
    basis.normal_unit = angular_momentum_m2_s / angular_momentum_norm_m2_s;
    basis.radial_out_unit = math::cross(basis.prograde_unit, basis.normal_unit);
    return basis;
}

Vector3 maneuver_to_inertial(const ManeuverBasis& basis, double prograde_m_s, double normal_m_s,
                             double radial_out_m_s) noexcept {
    return prograde_m_s * basis.prograde_unit + normal_m_s * basis.normal_unit
           + radial_out_m_s * basis.radial_out_unit;
}

} // namespace helios::orbital
