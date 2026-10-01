#ifndef HELIOS_APPS_HELIOS_SCENARIO_HPP
#define HELIOS_APPS_HELIOS_SCENARIO_HPP

#include "helios/core/error.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/time/epoch.hpp"

#include <string>

namespace helios::app {

// Universe time from Unix time (UTC). Approximate: TDB ≈ TT = UTC + 69.184 s, which assumes
// TAI − UTC = 37 s (true since 2017-01-01) and neglects TDB − TT (< 2 ms).
[[nodiscard]] core::Result<time::Epoch> epoch_from_unix_utc(double unix_seconds);

// "YYYY-MM-DD HH:MM:SS UTC", with the same approximation.
[[nodiscard]] std::string format_epoch_utc(const time::Epoch& epoch);

// The Phase 2 demo: a space station in a 420 km, 51.6° orbit, and a probe in a 200 km parking
// orbit in the Moon's plane with a translunar injection burn scheduled ten minutes after the
// start, timed so the transfer arrives where the Moon will be.
[[nodiscard]] core::VoidResult add_demo_vessels(sim::Simulation& simulation);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_SCENARIO_HPP
