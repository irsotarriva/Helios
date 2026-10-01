#ifndef HELIOS_SIM_SOLAR_SYSTEM_HPP
#define HELIOS_SIM_SOLAR_SYSTEM_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/frames/frame_tree.hpp"

#include <filesystem>
#include <memory>
#include <optional>

namespace helios::sim {

struct Universe {
    std::unique_ptr<frames::FrameTree> tree;
    std::unique_ptr<bodies::BodyCatalog> catalog;
};

// The stock Solar System from `data_dir` (data/solar_system).
//
// Body motion comes from `chebyshev_path` when given: a Helios Chebyshev export of JPL DE440
// (tools/ephemeris/export_de_chebyshev.py; ~110 MB for 1550–2650, not committed), exact to
// millimetres within its coverage. Otherwise from the committed mean elements
// (mean_elements.csv), usable at any epoch but only good to 0.3″–310″ for the planets and ~1°
// for the Moon inside their fit window (2000–2200), and degrading outside it (Jupiter and
// Saturn reach degrees within a few centuries): plenty for play, not for validation
// (docs/validation/ephemeris).
//
// The mean-element table gives the Moon about the Earth and the Earth–Moon barycentre about
// the Sun; the Earth is placed about the barycentre as −μ_Moon/(μ_Earth + μ_Moon) times the
// Moon's position, which in elements is the Moon's orbit scaled and turned by π.
[[nodiscard]] core::Result<Universe>
load_stock_solar_system(const std::filesystem::path& data_dir,
                        const std::optional<std::filesystem::path>& chebyshev_path);

} // namespace helios::sim

#endif // HELIOS_SIM_SOLAR_SYSTEM_HPP
