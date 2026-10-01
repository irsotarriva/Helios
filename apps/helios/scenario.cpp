#include "scenario.hpp"

#include "helios/orbital/conic.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/vessel/blueprint.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <string_view>
#include <utility>

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
    // Rationale: std::chrono's calendar types hold the year in 16 bits, and a slow interstellar
    // trip ends tens of thousands of years from now. This is the usual days-to-civil algorithm
    // (proleptic Gregorian calendar) on 64-bit integers.
    constexpr std::int64_t k_seconds_per_day = 86'400;
    constexpr std::int64_t k_days_per_era = 146'097; // 400 years
    const auto whole_s = static_cast<std::int64_t>(std::floor(unix_seconds));
    const std::int64_t days =
        (whole_s >= 0 ? whole_s : whole_s - (k_seconds_per_day - 1)) / k_seconds_per_day;
    const std::int64_t second_of_day = whole_s - days * k_seconds_per_day;
    const std::int64_t shifted = days + 719'468; // days since 0000-03-01
    const std::int64_t era = (shifted >= 0 ? shifted : shifted - (k_days_per_era - 1)) / k_days_per_era;
    const std::int64_t day_of_era = shifted - era * k_days_per_era;
    const std::int64_t year_of_era =
        (day_of_era - day_of_era / 1'460 + day_of_era / 36'524 - day_of_era / 146'096) / 365;
    const std::int64_t day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    const std::int64_t shifted_month = (5 * day_of_year + 2) / 153; // 0 = March
    const std::int64_t day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
    const std::int64_t month = shifted_month < 10 ? shifted_month + 3 : shifted_month - 9;
    const std::int64_t year = year_of_era + era * 400 + (month <= 2 ? 1 : 0);
    return std::format("{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC", year, month, day, second_of_day / 3'600,
                       second_of_day / 60 % 60, second_of_day % 60);
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
    if (core::VoidResult burn = simulation.schedule_maneuver(
            *probe, *burn_epoch, transfer_speed_m_s - parking_speed_m_s, 0.0, 0.0);
        !burn) {
        return burn;
    }

    // Interstellar probe: already past the planets and leaving for good, roughly where and how
    // fast Voyager 1 is (160 au, 17 km/s, 35° above the ecliptic). Something to watch at the
    // highest warp levels.
    if (const auto sun_id = catalog.find("Sun")) {
        constexpr double k_astronomical_unit_m = 1.495978707e11;
        constexpr double k_latitude_rad = 35.0 * orbital::k_pi / 180.0;
        constexpr double k_longitude_rad = 255.0 * orbital::k_pi / 180.0;
        const Vector3 outward{std::cos(k_latitude_rad) * std::cos(k_longitude_rad),
                              std::cos(k_latitude_rad) * std::sin(k_longitude_rad), std::sin(k_latitude_rad)};
        const Vector3 sideways = unit(math::cross(Vector3{0.0, 0.0, 1.0}, outward));
        const auto interstellar =
            simulation.add_vessel("Interstellar probe",
                                  {.position_m = 160.0 * k_astronomical_unit_m * outward,
                                   .velocity_m_s = 16'900.0 * outward + 1'000.0 * sideways},
                                  *sun_id);
        if (!interstellar) {
            return std::unexpected(interstellar.error());
        }
    }
    return {};
}

namespace {

constexpr double k_degree_rad = orbital::k_pi / 180.0;

struct ParkingOrbit {
    std::string_view vessel;
    double altitude_m = 0.0; // above the Earth
    double phase_rad = 0.0;
};
constexpr std::array k_parking_orbits{ParkingOrbit{"Kestrel", 300e3, 2.0},
                                      ParkingOrbit{"Firefly", 1000e3, 4.0},
                                      ParkingOrbit{"Merlin", 420e3, 3.0}};

// On the Moon, in Mare Tranquillitatis. Osprey stands 60 m south of Heron, and its pilot faces
// north: Heron is in the window.
struct LandingSite {
    std::string_view vessel;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};
constexpr std::array k_landing_sites{LandingSite{"Heron", 0.674, 23.473},
                                     LandingSite{"Osprey", 0.672, 23.473}};

} // namespace

core::VoidResult add_demo_craft(sim::Simulation& simulation, const std::filesystem::path& data_root) {
    vessel::PartCatalog parts;
    if (core::VoidResult loaded = parts.load_directory(data_root / "parts"); !loaded) {
        return loaded;
    }
    auto blueprints = vessel::load_blueprints(data_root / "vessels" / "demo.toml", parts);
    if (!blueprints) {
        return std::unexpected(blueprints.error());
    }
    const auto earth_id = simulation.catalog().find("Earth");
    if (!earth_id) {
        return std::unexpected(earth_id.error());
    }
    const bodies::Body& earth = simulation.catalog().body(*earth_id)->get();
    const auto moon_id = simulation.catalog().find("Moon");
    if (!moon_id) {
        return std::unexpected(moon_id.error());
    }
    // Orbits are circular, in the plane of the universe's x and y axes. A vessel that is not
    // listed gets an orbit of its own, above the ones before it.
    double altitude_m = 1700e3;
    double phase_rad = 0.0;
    for (vessel::VesselBlueprint& blueprint : *blueprints) {
        auto systems = vessel::VesselSystems::make(std::move(blueprint.assembly), parts.interfaces(),
                                                   std::move(blueprint.stages), simulation.now());
        if (!systems) {
            return std::unexpected(systems.error());
        }
        const auto site = std::ranges::find(k_landing_sites, blueprint.name, &LandingSite::vessel);
        if (site != k_landing_sites.end()) {
            const auto landed =
                simulation.add_landed_vessel(blueprint.name, *moon_id, site->latitude_deg * k_degree_rad,
                                             site->longitude_deg * k_degree_rad, std::move(*systems));
            if (!landed) {
                return std::unexpected(landed.error());
            }
            continue;
        }
        const auto listed = std::ranges::find(k_parking_orbits, blueprint.name, &ParkingOrbit::vessel);
        const bool is_listed = listed != k_parking_orbits.end();
        const auto added = simulation.add_vessel(
            blueprint.name,
            circular_state(earth.mean_radius_m + (is_listed ? listed->altitude_m : altitude_m),
                           earth.gravitational_parameter_m3_s2, Vector3{1.0, 0.0, 0.0},
                           Vector3{0.0, 1.0, 0.0}, is_listed ? listed->phase_rad : phase_rad),
            *earth_id, std::move(*systems));
        if (!added) {
            return std::unexpected(added.error());
        }
        if (!is_listed) {
            altitude_m += 700e3;
            phase_rad += 2.0;
        }
    }
    return {};
}

} // namespace helios::app
