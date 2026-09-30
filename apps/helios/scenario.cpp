#include "scenario.hpp"

#include "helios/orbital/conic.hpp"
#include "helios/orbital/kepler.hpp"

#include <chrono>
#include <cmath>
#include <format>

namespace helios::app {

namespace {

using core::ErrorCode;
using math::Vector3;

constexpr double k_j2000_unix_utc_s = 946'727'935.816; // 2000-01-01T11:58:55.816 UTC = J2000.0 TT
constexpr double k_leap_seconds_since_j2000 = 5.0;     // TAI − UTC: 32 s in 2000, 37 s since 2017

[[nodiscard]] Vector3 unit(const Vector3& vector) {
    return vector / math::norm(vector);
}

// Circular orbit of radius r in the plane spanned by the unit vectors `first` and `second`
// (orthonormal, motion from first towards second), at angle `phase` from `first`.
[[nodiscard]] orbital::StateVector circular_state(double radius_m, double mu_m3_s2, const Vector3& first,
                                                  const Vector3& second, double phase_rad) {
    const double speed_m_s = std::sqrt(mu_m3_s2 / radius_m);
    return {.position_m = radius_m * (std::cos(phase_rad) * first + std::sin(phase_rad) * second),
            .velocity_m_s = speed_m_s * (-std::sin(phase_rad) * first + std::cos(phase_rad) * second)};
}

} // namespace

core::Result<time::Epoch> epoch_from_unix_utc(double unix_seconds) {
    const double since_j2000_s = unix_seconds - k_j2000_unix_utc_s + k_leap_seconds_since_j2000;
    const double whole_s = std::floor(since_j2000_s);
    return time::Epoch::from_parts(static_cast<std::int64_t>(whole_s), since_j2000_s - whole_s);
}

std::string format_epoch_utc(const time::Epoch& epoch) {
    const double unix_seconds = static_cast<double>(epoch.whole_seconds()) + epoch.fraction()
                                + k_j2000_unix_utc_s - k_leap_seconds_since_j2000;
    const auto instant =
        std::chrono::sys_seconds(std::chrono::seconds(static_cast<std::int64_t>(std::floor(unix_seconds))));
    return std::format("{:%Y-%m-%d %H:%M:%S} UTC", instant);
}

core::VoidResult add_demo_vessels(sim::Simulation& simulation) {
    const auto& catalog = simulation.catalog();
    const auto earth_id = catalog.find("Earth");
    const auto moon_id = catalog.find("Moon");
    if (!earth_id || !moon_id) {
        return core::fail(ErrorCode::InvalidArgument, "the demo needs an Earth and a Moon");
    }
    const bodies::Body& earth = catalog.body(*earth_id)->get();
    const bodies::Body& moon = catalog.body(*moon_id)->get();
    const double mu_m3_s2 = earth.gravitational_parameter_m3_s2;
    const time::Epoch now = simulation.now();

    // Station: 420 km, 51.6° to the equator.
    Vector3 pole{0.0, 0.0, 1.0};
    if (earth.rotation.has_value()) {
        const math::Matrix3 to_body = earth.rotation->universe_to_body_fixed(now);
        pole = Vector3{to_body(2, 0), to_body(2, 1), to_body(2, 2)}; // body z in universe axes
    }
    const Vector3 node = unit(math::cross(pole, Vector3{0.0, 0.0, 1.0}));
    const Vector3 in_equator = math::cross(pole, node);
    constexpr double k_inclination_rad = 51.6 * orbital::k_pi / 180.0;
    const Vector3 along_orbit = std::cos(k_inclination_rad) * in_equator + std::sin(k_inclination_rad) * pole;
    const auto station = simulation.add_vessel(
        "Station", circular_state(earth.mean_radius_m + 420e3, mu_m3_s2, node, along_orbit, 0.3), *earth_id);
    if (!station) {
        return std::unexpected(station.error());
    }

    // Lunar probe: 200 km parking orbit in the Moon's plane; Hohmann-like injection.
    const auto moon_now = simulation.tree().relative_state(moon.frame, earth.frame, now);
    if (!moon_now) {
        return std::unexpected(moon_now.error());
    }
    const Vector3 moon_normal = unit(math::cross(moon_now->position_m, moon_now->velocity_m_s));
    const double parking_radius_m = earth.mean_radius_m + 200e3;
    const double moon_distance_m = math::norm(moon_now->position_m);
    const double transfer_semi_major_axis_m = 0.5 * (parking_radius_m + moon_distance_m);
    const double transfer_time_s = orbital::k_pi
                                   * std::sqrt(transfer_semi_major_axis_m * transfer_semi_major_axis_m
                                               * transfer_semi_major_axis_m / mu_m3_s2);
    constexpr double k_burn_delay_s = 600.0;
    const auto burn_epoch = now.advanced_by(k_burn_delay_s);
    const auto arrival_epoch = now.advanced_by(k_burn_delay_s + transfer_time_s);
    if (!burn_epoch || !arrival_epoch) {
        return core::fail(ErrorCode::OutOfRange, "demo epochs out of range");
    }
    const auto moon_at_arrival = simulation.tree().relative_state(moon.frame, earth.frame, *arrival_epoch);
    if (!moon_at_arrival) {
        return std::unexpected(moon_at_arrival.error());
    }
    // The burn point is opposite the Moon's arrival position; back off along the parking orbit
    // by the ten minutes before the burn.
    const Vector3 first =
        unit(moon_at_arrival->position_m - math::dot(moon_at_arrival->position_m, moon_normal) * moon_normal);
    const Vector3 second = math::cross(moon_normal, first);
    const double parking_rate_rad_s =
        std::sqrt(mu_m3_s2 / (parking_radius_m * parking_radius_m * parking_radius_m));
    const double start_phase_rad = orbital::k_pi - parking_rate_rad_s * k_burn_delay_s;
    const auto probe = simulation.add_vessel(
        "Lunar probe", circular_state(parking_radius_m, mu_m3_s2, first, second, start_phase_rad), *earth_id);
    if (!probe) {
        return std::unexpected(probe.error());
    }
    const double parking_speed_m_s = std::sqrt(mu_m3_s2 / parking_radius_m);
    const double transfer_speed_m_s =
        std::sqrt(mu_m3_s2 * (2.0 / parking_radius_m - 1.0 / transfer_semi_major_axis_m));
    return simulation.schedule_maneuver(*probe, *burn_epoch, transfer_speed_m_s - parking_speed_m_s, 0.0,
                                        0.0);
}

} // namespace helios::app
