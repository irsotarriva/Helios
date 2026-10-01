#ifndef HELIOS_TEST_SUPPORT_UNIVERSES_HPP
#define HELIOS_TEST_SUPPORT_UNIVERSES_HPP

#include <filesystem>

#include "../../../tools/support/universes.hpp"

namespace helios::test {

using tools::EarthMoonCr3bp;
using tools::make_earth_moon_cr3bp;
using tools::SolarSystem;

// The real Solar System over the committed DE440 excerpt (2020-01-01 .. 2020-02-25 TDB).
[[nodiscard]] inline SolarSystem load_solar_system() {
    return tools::load_solar_system(std::filesystem::path{HELIOS_TEST_DATA_DIR} / "de440_2020_excerpt.hce",
                                    std::filesystem::path{HELIOS_DATA_DIR} / "solar_system" / "bodies.csv")
        .value();
}

} // namespace helios::test

#endif // HELIOS_TEST_SUPPORT_UNIVERSES_HPP
