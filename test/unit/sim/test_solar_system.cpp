#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/solar_system.hpp"

#include <gtest/gtest.h>

#include "../support/universes.hpp"

namespace {

using helios::math::norm;
using helios::time::Epoch;

TEST(StockSolarSystem, MeanElementUniverseAgreesWithDe440) {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    const auto reference = helios::test::load_solar_system();
    const Epoch epoch = Epoch::from_parts(631'238'400, 0.0).value(); // 2020-01-02
    for (const char* name : {"Earth", "Moon", "Mars", "Jupiter"}) {
        const auto& body = universe.catalog->body(universe.catalog->find(name).value())->get();
        const auto& sun = universe.catalog->body(universe.catalog->find("Sun").value())->get();
        const auto& reference_body = reference.catalog->body(reference.catalog->find(name).value())->get();
        const auto& reference_sun = reference.catalog->body(reference.catalog->find("Sun").value())->get();
        const auto heliocentric = universe.tree->relative_state(body.frame, sun.frame, epoch).value();
        const auto expected =
            reference.tree->relative_state(reference_body.frame, reference_sun.frame, epoch).value();
        // Mean elements: ~1e-4 relative for the inner planets (plus the Sun–barycentre offset).
        EXPECT_LT(norm(heliocentric.position_m - expected.position_m) / norm(expected.position_m), 5e-3)
            << name;
    }
    const auto& earth = universe.catalog->body(universe.catalog->find("Earth").value())->get();
    const auto& moon = universe.catalog->body(universe.catalog->find("Moon").value())->get();
    const auto& reference_earth = reference.catalog->body(reference.catalog->find("Earth").value())->get();
    const auto& reference_moon = reference.catalog->body(reference.catalog->find("Moon").value())->get();
    const auto geocentric = universe.tree->relative_state(moon.frame, earth.frame, epoch).value();
    const auto expected =
        reference.tree->relative_state(reference_moon.frame, reference_earth.frame, epoch).value();
    EXPECT_LT(norm(geocentric.position_m - expected.position_m) / norm(expected.position_m), 0.03);
}

TEST(StockSolarSystem, BuildsASnapshotWithEveryBodyAndOrbit) {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    const std::size_t body_count = universe.catalog->bodies().size();
    const auto earth = universe.catalog->find("Earth").value();
    auto simulation = helios::sim::Simulation::make(std::move(universe.tree), std::move(universe.catalog),
                                                    Epoch::from_parts(820'000'000, 0.0).value(), {})
                          .value();
    const auto snapshot = helios::sim::build_snapshot(simulation, helios::sim::Focus::body(earth)).value();
    EXPECT_EQ(snapshot.bodies.size(), body_count);
    EXPECT_EQ(snapshot.lines.size(), body_count - 1); // every body but the Sun has an orbit
}

} // namespace
