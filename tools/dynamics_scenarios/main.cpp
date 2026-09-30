// helios_dynamics_scenarios — runs the Phase 1 dynamics validation scenarios through the real
// propagator and writes one CSV per scenario, for tools/dynamics/plot_dynamics.py.
//
//   helios_dynamics_scenarios <ephemeris.hce> <bodies.csv> <output directory>
//
// The ephemeris must cover 2020-01-02 .. 2020-01-23 TDB (the committed test excerpt does).

#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/dynamics/gravity_model.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "../support/universes.hpp"

namespace {

using helios::core::ErrorCode;
using helios::core::Result;
using helios::core::VoidResult;
using helios::dynamics::EnckePropagator;
using helios::dynamics::GravityModel;
using helios::dynamics::PropagatorOptions;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::StateVector;
using helios::time::Epoch;
using helios::tools::EarthMoonCr3bp;

constexpr double k_day_s = 86'400.0;
constexpr double k_hour_s = 3'600.0;

// Collects CSV rows in memory and writes them in one go (I/O wrapped at the boundary).
class CsvWriter {
public:
    explicit CsvWriter(std::string header) { text_ = std::move(header) + "\n"; }
    template <typename... Values>
    void row(const Values&... values) {
        std::string line;
        ((line += std::format("{:.17g},", static_cast<double>(values))), ...);
        line.back() = '\n';
        text_ += line;
    }
    [[nodiscard]] VoidResult save(const std::filesystem::path& path) const {
        return helios::core::try_call(ErrorCode::IoFailure, "writing CSV",
                                      [&]() -> VoidResult {
                                          std::ofstream stream(path);
                                          if (!stream) {
                                              return helios::core::fail(
                                                  ErrorCode::IoFailure,
                                                  std::format("cannot write '{}'", path.string()));
                                          }
                                          stream << text_;
                                          return {};
                                      })
            .and_then([](VoidResult result) { return result; });
    }

private:
    std::string text_;
};

[[nodiscard]] Epoch at(const Epoch& start, double offset_s) {
    return start.advanced_by(offset_s).value();
}

[[nodiscard]] double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// ── A: Encke vs Cowell in the real Solar System, and a tolerance sweep ───────────────────────
[[nodiscard]] VoidResult encke_vs_cowell(const helios::tools::SolarSystem& solar_system,
                                         const std::filesystem::path& output) {
    const auto earth = solar_system.catalog->find("Earth").value();
    const double earth_mu_m3_s2 =
        solar_system.catalog->body(earth).value().get().gravitational_parameter_m3_s2;
    const Epoch start = Epoch::from_parts(631'238'400, 0.0).value();
    const GravityModel gravity =
        GravityModel::make(*solar_system.tree, *solar_system.catalog, {}, start).value();
    const StateVector initial{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 10'200.0, 1'000.0}};
    constexpr double k_duration_s = 20.0 * k_day_s;
    PropagatorOptions options;
    options.switch_domains = false;

    CsvWriter history("time_days,encke_minus_cowell_m,kepler_minus_cowell_m,radius_m");
    EnckePropagator encke = EnckePropagator::make(gravity, {earth, start, initial}, options).value();
    StateVector cowell = initial;
    for (int hour = 1; hour <= static_cast<int>(k_duration_s / k_hour_s); ++hour) {
        const double time_s = k_hour_s * hour;
        cowell = helios::dynamics::propagate_cowell(gravity, earth, at(start, time_s - k_hour_s), cowell,
                                                    k_hour_s, 1e-13, 300.0)
                     .value();
        const StateVector encke_state = encke.state_at(at(start, time_s)).value().state_in_domain;
        const StateVector kepler = helios::orbital::propagate_conic(initial, earth_mu_m3_s2, time_s).value();
        history.row(time_s / k_day_s, norm(encke_state.position_m - cowell.position_m),
                    norm(kepler.position_m - cowell.position_m), norm(cowell.position_m));
    }
    if (VoidResult saved = history.save(output / "encke_vs_cowell.csv"); !saved) {
        return saved;
    }

    // Reference for the sweep: Cowell at a tighter tolerance and a short maximum step.
    const StateVector reference =
        helios::dynamics::propagate_cowell(gravity, earth, start, initial, k_duration_s, 2e-14, 120.0)
            .value();
    CsvWriter sweep(
        "relative_tolerance,encke_error_m,encke_steps,encke_seconds,cowell_error_m,cowell_seconds");
    for (const double tolerance : {1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 1e-13}) {
        PropagatorOptions sweep_options = options;
        sweep_options.relative_tolerance = tolerance;
        const auto encke_clock = std::chrono::steady_clock::now();
        EnckePropagator sweep_encke =
            EnckePropagator::make(gravity, {earth, start, initial}, sweep_options).value();
        const StateVector encke_final = sweep_encke.state_at(at(start, k_duration_s)).value().state_in_domain;
        const double encke_seconds = seconds_since(encke_clock);
        const auto cowell_clock = std::chrono::steady_clock::now();
        const StateVector cowell_final =
            helios::dynamics::propagate_cowell(gravity, earth, start, initial, k_duration_s, tolerance, 1e9)
                .value();
        const double cowell_seconds = seconds_since(cowell_clock);
        sweep.row(tolerance, norm(encke_final.position_m - reference.position_m),
                  static_cast<double>(sweep_encke.statistics().accepted_steps), encke_seconds,
                  norm(cowell_final.position_m - reference.position_m), cowell_seconds);
    }
    return sweep.save(output / "tolerance_sweep.csv");
}

// ── B: warp invariance ──────────────────────────────────────────────────────────────────────
[[nodiscard]] VoidResult warp_invariance(const std::filesystem::path& output) {
    const EarthMoonCr3bp system = helios::tools::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {4.2e7, 0.0, 0.0}, .velocity_m_s = {0.0, 3'080.0, 100.0}};
    PropagatorOptions options;
    options.switch_domains = false;
    constexpr double k_span_s = 30.0 * k_day_s;

    // The finest cadence is the reference; every coarser run is compared at its own samples.
    EnckePropagator finest =
        EnckePropagator::make(gravity, {system.earth, Epoch{}, initial}, options).value();
    std::vector<Vector3> reference_positions;
    constexpr double k_finest_s = 60.0;
    for (int sample = 1; sample <= static_cast<int>(k_span_s / k_finest_s); ++sample) {
        reference_positions.push_back(
            finest.state_at(at(Epoch{}, k_finest_s * sample)).value().state_in_domain.position_m);
    }
    CsvWriter table("cadence_s,equivalent_warp,samples,max_position_difference_m,accepted_steps");
    for (const double cadence_s : {60.0, 3'600.0, 86'400.0, 30.0 * 86'400.0}) {
        EnckePropagator run =
            EnckePropagator::make(gravity, {system.earth, Epoch{}, initial}, options).value();
        double worst_m = 0.0;
        const int samples = static_cast<int>(k_span_s / cadence_s);
        for (int sample = 1; sample <= samples; ++sample) {
            const double time_s = cadence_s * sample;
            const Vector3 position_m = run.state_at(at(Epoch{}, time_s)).value().state_in_domain.position_m;
            const auto reference_index = static_cast<std::size_t>(std::llround(time_s / k_finest_s)) - 1;
            worst_m = std::max(worst_m, norm(position_m - reference_positions.at(reference_index)));
        }
        // "Equivalent warp" if the renderer samples once per 1/60 s frame.
        table.row(cadence_s, cadence_s * 60.0, samples, worst_m,
                  static_cast<double>(run.statistics().accepted_steps));
    }
    return table.save(output / "warp_invariance.csv");
}

[[nodiscard]] helios::frames::FrameId frame_of(const EarthMoonCr3bp& system, helios::bodies::BodyId body) {
    return system.catalog->body(body).value().get().frame;
}

[[nodiscard]] StateVector barycentric(const EarthMoonCr3bp& system,
                                      const helios::dynamics::VesselState& state) {
    const auto earth =
        system.tree
            ->relative_state(frame_of(system, system.earth), helios::frames::FrameTree::root(), state.epoch)
            .value();
    return StateVector{.position_m = state.state_in_domain.position_m + earth.position_m,
                       .velocity_m_s = state.state_in_domain.velocity_m_s + earth.velocity_m_s};
}

[[nodiscard]] Vector3 co_rotating(const Vector3& position_m, double time_s) {
    const double angle_rad = -EarthMoonCr3bp::mean_motion_rad_s() * time_s;
    return {position_m.x * std::cos(angle_rad) - position_m.y * std::sin(angle_rad),
            position_m.x * std::sin(angle_rad) + position_m.y * std::cos(angle_rad), position_m.z};
}

[[nodiscard]] StateVector co_rotating_rest_state(const EarthMoonCr3bp& system, const Vector3& point_m) {
    const double mean_motion_rad_s = EarthMoonCr3bp::mean_motion_rad_s();
    const auto earth =
        system.tree
            ->relative_state(frame_of(system, system.earth), helios::frames::FrameTree::root(), Epoch{})
            .value();
    return StateVector{.position_m = point_m - earth.position_m,
                       .velocity_m_s =
                           Vector3{-mean_motion_rad_s * point_m.y, mean_motion_rad_s * point_m.x, 0.0}
                           - earth.velocity_m_s};
}

[[nodiscard]] double jacobi_constant(const EarthMoonCr3bp& system, const StateVector& vessel,
                                     const Epoch& instant) {
    const Vector3 earth_m =
        system.tree
            ->relative_state(frame_of(system, system.earth), helios::frames::FrameTree::root(), instant)
            .value()
            .position_m;
    const Vector3 moon_m =
        system.tree->relative_state(frame_of(system, system.moon), helios::frames::FrameTree::root(), instant)
            .value()
            .position_m;
    return 0.5 * helios::math::dot(vessel.velocity_m_s, vessel.velocity_m_s)
           - EarthMoonCr3bp::mean_motion_rad_s()
                 * helios::math::cross(vessel.position_m, vessel.velocity_m_s).z
           - EarthMoonCr3bp::k_earth_mu_m3_s2 / norm(vessel.position_m - earth_m)
           - EarthMoonCr3bp::k_moon_mu_m3_s2 / norm(vessel.position_m - moon_m);
}

// ── C: L4 libration and Jacobi conservation over five years ─────────────────────────────────
[[nodiscard]] VoidResult l4_libration(const std::filesystem::path& output) {
    const EarthMoonCr3bp system = helios::tools::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const double separation_m = EarthMoonCr3bp::k_separation_m;
    const Vector3 l4_m{separation_m * (0.5 - EarthMoonCr3bp::mass_ratio()),
                       separation_m * std::numbers::sqrt3 / 2.0, 0.0};
    PropagatorOptions options;
    options.switch_domains = false;
    const StateVector initial = co_rotating_rest_state(system, l4_m + Vector3{1.0e6, 0.0, 0.0});
    EnckePropagator propagator =
        EnckePropagator::make(gravity, {system.earth, Epoch{}, initial}, options).value();
    const double initial_jacobi =
        jacobi_constant(system, barycentric(system, {system.earth, Epoch{}, initial}), Epoch{});

    CsvWriter table("time_days,rotating_x_m,rotating_y_m,jacobi_relative_drift");
    for (int step = 0; step <= 5 * 365 * 4; ++step) {
        const double time_s = step * k_day_s / 4.0;
        const StateVector vessel = barycentric(system, propagator.state_at(at(Epoch{}, time_s)).value());
        const Vector3 rotating_m = co_rotating(vessel.position_m, time_s);
        table.row(time_s / k_day_s, rotating_m.x, rotating_m.y,
                  jacobi_constant(system, vessel, at(Epoch{}, time_s)) / initial_jacobi - 1.0);
    }
    return table.save(output / "l4_libration.csv");
}

// ── D: L1 divergence ────────────────────────────────────────────────────────────────────────
[[nodiscard]] double l1_gamma(double mass_ratio) {
    double gamma = std::cbrt(mass_ratio / 3.0);
    for (int iteration = 0; iteration < 50; ++iteration) {
        const double value = std::pow(gamma, 5) - (3.0 - mass_ratio) * std::pow(gamma, 4)
                             + (3.0 - 2.0 * mass_ratio) * std::pow(gamma, 3) - mass_ratio * gamma * gamma
                             + 2.0 * mass_ratio * gamma - mass_ratio;
        const double slope = 5.0 * std::pow(gamma, 4) - 4.0 * (3.0 - mass_ratio) * std::pow(gamma, 3)
                             + 3.0 * (3.0 - 2.0 * mass_ratio) * gamma * gamma - 2.0 * mass_ratio * gamma
                             + 2.0 * mass_ratio;
        gamma -= value / slope;
    }
    return gamma;
}

[[nodiscard]] VoidResult l1_divergence(const std::filesystem::path& output) {
    const EarthMoonCr3bp system = helios::tools::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const double mass_ratio = EarthMoonCr3bp::mass_ratio();
    const double gamma = l1_gamma(mass_ratio);
    const Vector3 l1_m{(1.0 - mass_ratio - gamma) * EarthMoonCr3bp::k_separation_m, 0.0, 0.0};
    const double c2 = mass_ratio / std::pow(gamma, 3) + (1.0 - mass_ratio) / std::pow(1.0 - gamma, 3);
    const double expected_rate_per_s = std::sqrt((c2 - 2.0 + std::sqrt(9.0 * c2 * c2 - 8.0 * c2)) / 2.0)
                                       * EarthMoonCr3bp::mean_motion_rad_s();

    CsvWriter table("seed_offset_m,time_days,distance_from_l1_m,expected_rate_per_day");
    PropagatorOptions options;
    options.switch_domains = false;
    for (const double seed_m : {1e-3, 1.0, 1e3}) {
        EnckePropagator propagator =
            EnckePropagator::make(
                gravity,
                {system.earth, Epoch{}, co_rotating_rest_state(system, l1_m + Vector3{seed_m, 0.0, 0.0})},
                options)
                .value();
        for (int hour = 0; hour <= 60 * 24; hour += 2) {
            const double time_s = k_hour_s * hour;
            const StateVector vessel = barycentric(system, propagator.state_at(at(Epoch{}, time_s)).value());
            table.row(seed_m, time_s / k_day_s, norm(co_rotating(vessel.position_m, time_s) - l1_m),
                      expected_rate_per_s * k_day_s);
        }
    }
    return table.save(output / "l1_divergence.csv");
}

// ── E: domain switch Moon → Earth ───────────────────────────────────────────────────────────
[[nodiscard]] VoidResult domain_switch(const std::filesystem::path& output) {
    const EarthMoonCr3bp system = helios::tools::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {6.7e6, 0.0, 0.0}, .velocity_m_s = {3'000.0, 500.0, 0.0}};
    EnckePropagator propagator = EnckePropagator::make(gravity, {system.moon, Epoch{}, initial}, {}).value();
    const auto moon_in_earth =
        system.tree->relative_state(frame_of(system, system.moon), frame_of(system, system.earth), Epoch{})
            .value();
    StateVector cowell{.position_m = initial.position_m + moon_in_earth.position_m,
                       .velocity_m_s = initial.velocity_m_s + moon_in_earth.velocity_m_s};

    CsvWriter table("time_hours,domain_is_moon,earth_frame_x_m,earth_frame_y_m,distance_from_moon_m,"
                    "encke_minus_cowell_m");
    constexpr double k_sample_s = 600.0;
    for (int sample = 1; sample <= static_cast<int>(3.0 * k_day_s / k_sample_s); ++sample) {
        const double time_s = k_sample_s * sample;
        const Epoch instant = at(Epoch{}, time_s);
        const auto state = propagator.state_at(instant).value();
        const auto domain_frame = frame_of(system, state.domain);
        const auto domain_in_earth =
            system.tree->relative_state(domain_frame, frame_of(system, system.earth), instant).value();
        const Vector3 in_earth_m = state.state_in_domain.position_m + domain_in_earth.position_m;
        const auto moon =
            system.tree
                ->relative_state(frame_of(system, system.moon), frame_of(system, system.earth), instant)
                .value();
        cowell = helios::dynamics::propagate_cowell(gravity, system.earth, at(Epoch{}, time_s - k_sample_s),
                                                    cowell, k_sample_s, 1e-13, 60.0)
                     .value();
        table.row(time_s / k_hour_s, state.domain == system.moon ? 1.0 : 0.0, in_earth_m.x, in_earth_m.y,
                  norm(in_earth_m - moon.position_m), norm(in_earth_m - cowell.position_m));
    }
    return table.save(output / "domain_switch.csv");
}

// ── F: fidelity vs cost of the Barnes–Hut opening angle ────────────────────────────────────
[[nodiscard]] VoidResult opening_angle(const helios::tools::SolarSystem& solar_system,
                                       const std::filesystem::path& output) {
    const Epoch instant = Epoch::from_parts(631'238'400, 0.0).value();
    const auto earth = solar_system.catalog->find("Earth").value();
    const auto sun = solar_system.catalog->find("Sun").value();
    const auto earth_from_sun =
        solar_system.tree
            ->relative_state(solar_system.catalog->body(earth).value().get().frame,
                             solar_system.catalog->body(sun).value().get().frame, instant)
            .value();
    const Vector3 anti_sun_unit = earth_from_sun.position_m / norm(earth_from_sun.position_m);
    struct Probe {
        std::string_view name;
        helios::bodies::BodyId domain;
        Vector3 position_m;
    };
    const std::array probes{
        Probe{"LEO", earth, {7.0e6, 0.0, 0.0}},
        Probe{"GEO", earth, {4.2164e7, 0.0, 0.0}},
        Probe{"Cislunar 450000 km", earth, {0.0, 4.5e8, 0.0}},
        Probe{"Sun-Earth L2", sun, earth_from_sun.position_m + anti_sun_unit * 1.5e9},
        Probe{"Heliocentric 1.5 AU", sun, earth_from_sun.position_m * 1.5},
        Probe{"Heliocentric 5 AU", sun, earth_from_sun.position_m * 5.0},
    };
    CsvWriter table("probe_index,opening_angle,sources,relative_error_vs_theta0,microseconds_per_evaluation");
    for (std::size_t probe_index = 0; probe_index < probes.size(); ++probe_index) {
        const Probe& probe = probes.at(probe_index);
        const GravityModel exact =
            GravityModel::make(*solar_system.tree, *solar_system.catalog, {.opening_angle = 0.0}, instant)
                .value();
        const Vector3 exact_m_s2 =
            exact.total_acceleration_m_s2(probe.domain, probe.position_m, instant).value();
        for (const double theta : {0.0, 0.1, 0.25, 0.5, 1.0, 2.0}) {
            const GravityModel model = GravityModel::make(*solar_system.tree, *solar_system.catalog,
                                                          {.opening_angle = theta}, instant)
                                           .value();
            const Vector3 approximate_m_s2 =
                model.total_acceleration_m_s2(probe.domain, probe.position_m, instant).value();
            constexpr int k_repetitions = 2'000;
            const auto clock = std::chrono::steady_clock::now();
            // Summing the results keeps the optimiser from discarding the timed work.
            double checksum_m_s2 = 0.0;
            for (int repetition = 0; repetition < k_repetitions; ++repetition) {
                const auto timed = model.total_acceleration_m_s2(probe.domain, probe.position_m, instant);
                if (!timed) {
                    return std::unexpected(timed.error());
                }
                checksum_m_s2 += timed->x;
            }
            const double microseconds = seconds_since(clock) * 1e6 / k_repetitions;
            if (!std::isfinite(checksum_m_s2)) {
                return helios::core::fail(ErrorCode::NotFinite, "gravity benchmark produced a non-finite value");
            }
            const auto sources = model.selected_sources(probe.domain, probe.position_m, instant).value();
            table.row(static_cast<double>(probe_index), theta, static_cast<double>(sources.size()),
                      norm(approximate_m_s2 - exact_m_s2) / norm(exact_m_s2), microseconds);
        }
    }
    std::string names = "probe_index,name\n";
    for (std::size_t index = 0; index < probes.size(); ++index) {
        names += std::format("{},{}\n", index, probes.at(index).name);
    }
    if (auto written =
            helios::core::try_call(ErrorCode::IoFailure, "writing probe names",
                                   [&] { std::ofstream(output / "opening_angle_probes.csv") << names; });
        !written) {
        return std::unexpected(written.error());
    }
    return table.save(output / "opening_angle.csv");
}

[[nodiscard]] VoidResult run(std::span<char*> arguments) {
    if (arguments.size() != 4) {
        return helios::core::fail(
            ErrorCode::InvalidArgument,
            "usage: helios_dynamics_scenarios <ephemeris.hce> <bodies.csv> <output directory>");
    }
    const std::filesystem::path output{arguments[3]};
    auto solar_system = helios::tools::load_solar_system(arguments[1], arguments[2]);
    if (!solar_system) {
        return std::unexpected(solar_system.error());
    }
    const std::array<std::pair<std::string_view, std::function<VoidResult()>>, 6> scenarios{{
        {"encke vs cowell", [&] { return encke_vs_cowell(*solar_system, output); }},
        {"warp invariance", [&] { return warp_invariance(output); }},
        {"L4 libration", [&] { return l4_libration(output); }},
        {"L1 divergence", [&] { return l1_divergence(output); }},
        {"domain switch", [&] { return domain_switch(output); }},
        {"opening angle", [&] { return opening_angle(*solar_system, output); }},
    }};
    for (const auto& [name, scenario] : scenarios) {
        const auto clock = std::chrono::steady_clock::now();
        if (VoidResult result = scenario(); !result) {
            return helios::core::fail(ErrorCode::Unknown,
                                      std::format("{}: {}", name, helios::core::describe(result.error())));
        }
        std::fputs(std::format("{:<16} {:6.1f} s\n", name, seconds_since(clock)).c_str(), stderr);
    }
    return {};
}

} // namespace

int main(int argc, char** argv) {
    const VoidResult result = run(std::span<char*>{argv, static_cast<std::size_t>(argc)});
    if (!result) {
        std::fputs(
            std::format("helios_dynamics_scenarios: {}\n", helios::core::describe(result.error())).c_str(),
            stderr);
        return 1;
    }
    return 0;
}
