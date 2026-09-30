#include "helios/ephemeris/keplerian_ephemeris.hpp"

#include <array>
#include <cmath>
#include <format>

namespace helios::ephemeris {

namespace {
using core::ErrorCode;
} // namespace

core::Result<KeplerianEphemeris> KeplerianEphemeris::make(const SecularElements& elements) noexcept {
    const std::array<double, 12> values{elements.semi_major_axis_m,
                                        elements.semi_major_axis_rate_m_s,
                                        elements.eccentricity,
                                        elements.eccentricity_rate_per_s,
                                        elements.inclination_rad,
                                        elements.inclination_rate_rad_s,
                                        elements.longitude_of_ascending_node_rad,
                                        elements.longitude_of_ascending_node_rate_rad_s,
                                        elements.longitude_of_periapsis_rad,
                                        elements.longitude_of_periapsis_rate_rad_s,
                                        elements.mean_longitude_rad,
                                        elements.mean_longitude_rate_rad_s};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return core::fail(ErrorCode::NotFinite, "secular elements contain a non-finite value");
        }
    }
    if (elements.semi_major_axis_m <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "semi-major axis must be positive");
    }
    if (elements.eccentricity < 0.0 || elements.eccentricity >= 1.0) {
        return core::fail(ErrorCode::OutOfRange,
                          std::format("eccentricity {} is not in [0, 1)", elements.eccentricity));
    }
    if (elements.mean_longitude_rate_rad_s <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "mean motion (mean-longitude rate) must be positive");
    }
    return KeplerianEphemeris{elements};
}

double KeplerianEphemeris::effective_gravitational_parameter_m3_s2(double semi_major_axis_m) const noexcept {
    const double mean_motion_rad_s = elements_.mean_longitude_rate_rad_s;
    return mean_motion_rad_s * mean_motion_rad_s * semi_major_axis_m * semi_major_axis_m * semi_major_axis_m;
}

core::Result<orbital::KeplerianElements>
KeplerianEphemeris::elements_at(const time::Epoch& instant) const noexcept {
    // TODO: for |Δt| ≫ 1e3 years, n·Δt loses sub-metre precision; switch to a split product.
    const double elapsed_s = time::seconds_between(elements_.reference_epoch, instant);
    const SecularElements& mean = elements_;

    const double semi_major_axis_m = mean.semi_major_axis_m + mean.semi_major_axis_rate_m_s * elapsed_s;
    const double eccentricity = mean.eccentricity + mean.eccentricity_rate_per_s * elapsed_s;
    if (semi_major_axis_m <= 0.0 || eccentricity < 0.0 || eccentricity >= 1.0) {
        return core::fail(ErrorCode::OutOfRange,
                          "secular rates drive the orbit out of the elliptic regime at this epoch");
    }
    const double node_rad =
        mean.longitude_of_ascending_node_rad + mean.longitude_of_ascending_node_rate_rad_s * elapsed_s;
    const double periapsis_longitude_rad =
        mean.longitude_of_periapsis_rad + mean.longitude_of_periapsis_rate_rad_s * elapsed_s;
    const double mean_longitude_rad = mean.mean_longitude_rad + mean.mean_longitude_rate_rad_s * elapsed_s;

    return orbital::KeplerianElements{
        .semi_major_axis_m = semi_major_axis_m,
        .eccentricity = eccentricity,
        .inclination_rad = mean.inclination_rad + mean.inclination_rate_rad_s * elapsed_s,
        .longitude_of_ascending_node_rad = orbital::wrap_angle_rad(node_rad),
        .argument_of_periapsis_rad = orbital::wrap_angle_rad(periapsis_longitude_rad - node_rad),
        .mean_anomaly_rad = orbital::wrap_angle_rad(mean_longitude_rad - periapsis_longitude_rad),
    };
}

core::Result<orbital::StateVector> KeplerianEphemeris::state_at(const time::Epoch& instant) const noexcept {
    return elements_at(instant).and_then([&](const orbital::KeplerianElements& osculating) {
        return orbital::state_from_elements(
            osculating, effective_gravitational_parameter_m3_s2(osculating.semi_major_axis_m));
    });
}

core::Result<math::Vector3> KeplerianEphemeris::acceleration_at(const time::Epoch& instant) const noexcept {
    return elements_at(instant).and_then([&](const orbital::KeplerianElements& osculating) {
        const double mu_m3_s2 = effective_gravitational_parameter_m3_s2(osculating.semi_major_axis_m);
        return orbital::state_from_elements(osculating, mu_m3_s2)
            .transform([&](const orbital::StateVector& state) {
                return orbital::two_body_acceleration_m_s2(state.position_m, mu_m3_s2);
            });
    });
}

} // namespace helios::ephemeris
