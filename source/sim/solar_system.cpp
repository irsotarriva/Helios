#include "helios/sim/solar_system.hpp"

#include "helios/core/parse.hpp"
#include "helios/ephemeris/ephemeris_file.hpp"
#include "helios/frames/body_rotation.hpp"
#include "helios/orbital/kepler.hpp"

#include <format>
#include <string>
#include <string_view>

namespace helios::sim {

namespace {

using core::ErrorCode;

constexpr std::string_view k_root_name = "Solar System barycentre";

// μ of one body, read straight from bodies.csv (needed before the catalogue can be built).
[[nodiscard]] core::Result<double> gravitational_parameter_m3_s2(std::string_view bodies_csv,
                                                                 std::string_view name) {
    std::size_t start = 0;
    while (start < bodies_csv.size()) {
        std::size_t end = bodies_csv.find('\n', start);
        if (end == std::string_view::npos) {
            end = bodies_csv.size();
        }
        const std::string_view line = core::trim(bodies_csv.substr(start, end - start));
        start = end + 1;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto fields = core::split_csv_line(line);
        if (fields.size() > 2 && core::trim(fields[0]) == name) {
            return core::parse_double(core::trim(fields[2]));
        }
    }
    return core::fail(ErrorCode::ParseFailure, std::format("bodies.csv has no row for '{}'", name));
}

[[nodiscard]] core::Result<frames::FrameTree> mean_element_tree(const std::filesystem::path& data_dir,
                                                                std::string_view bodies_csv) {
    auto rows = ephemeris::load_secular_elements_csv(data_dir / "mean_elements.csv");
    if (!rows) {
        return std::unexpected(rows.error());
    }
    const auto earth_mu = gravitational_parameter_m3_s2(bodies_csv, "Earth");
    const auto moon_mu = gravitational_parameter_m3_s2(bodies_csv, "Moon");
    if (!earth_mu || !moon_mu) {
        return std::unexpected(!earth_mu ? earth_mu.error() : moon_mu.error());
    }
    const ephemeris::NamedSecularElements* moon = nullptr;
    for (const auto& row : *rows) {
        if (row.target_name == "Moon" && row.center_name == "Earth") {
            moon = &row;
        }
    }
    if (moon == nullptr) {
        return core::fail(ErrorCode::ParseFailure, "mean_elements.csv has no Moon (about the Earth) row");
    }

    frames::FrameTree tree{std::string(k_root_name)};
    // Rationale: the Sun–barycentre offset (up to ~2 solar radii) is below the mean elements'
    // own error for the planets fitted about the Sun, so the Sun sits at the barycentre here.
    if (auto sun = tree.add_frame("Sun", frames::FrameTree::root(), frames::FixedOffset{}); !sun) {
        return std::unexpected(sun.error());
    }
    for (const auto& row : *rows) {
        if (&row == moon) {
            continue;
        }
        auto added = tree.find(row.center_name).and_then([&](frames::FrameId center) {
            return tree.add_frame(row.target_name, center, row.ephemeris);
        });
        if (!added) {
            return std::unexpected(added.error());
        }
    }
    // Earth about the Earth–Moon barycentre: the Moon's orbit scaled by −μ_M/(μ_E + μ_M).
    ephemeris::SecularElements earth = moon->ephemeris.elements();
    const double moon_fraction = *moon_mu / (*earth_mu + *moon_mu);
    earth.semi_major_axis_m *= moon_fraction;
    earth.semi_major_axis_rate_m_s *= moon_fraction;
    earth.longitude_of_periapsis_rad += orbital::k_pi;
    earth.mean_longitude_rad += orbital::k_pi;
    auto earth_ephemeris = ephemeris::KeplerianEphemeris::make(earth);
    if (!earth_ephemeris) {
        return std::unexpected(earth_ephemeris.error());
    }
    auto added = tree.find("Earth-Moon barycentre")
                     .and_then([&](frames::FrameId barycentre) {
                         return tree.add_frame("Earth", barycentre, *earth_ephemeris);
                     })
                     .and_then([&](frames::FrameId earth_frame) {
                         return tree.add_frame("Moon", earth_frame, moon->ephemeris);
                     });
    if (!added) {
        return std::unexpected(added.error());
    }
    return tree;
}

[[nodiscard]] core::Result<frames::FrameTree> chebyshev_tree(const std::filesystem::path& path) {
    auto segments = ephemeris::load_chebyshev_file(path);
    if (!segments) {
        return std::unexpected(segments.error());
    }
    if (segments->empty()) {
        return core::fail(ErrorCode::InvalidArgument, "ephemeris file has no segments");
    }
    frames::FrameTree tree(segments->front().center_name);
    for (auto& segment : *segments) {
        auto added = tree.find(segment.center_name).and_then([&](frames::FrameId center) {
            return tree.add_frame(segment.target_name, center, std::move(segment.ephemeris));
        });
        if (!added) {
            return std::unexpected(added.error());
        }
    }
    return tree;
}

} // namespace

core::Result<Universe> load_stock_solar_system(const std::filesystem::path& data_dir,
                                               const std::optional<std::filesystem::path>& chebyshev_path) {
    const auto bodies_csv = core::read_text_file(data_dir / "bodies.csv");
    if (!bodies_csv) {
        return std::unexpected(bodies_csv.error());
    }
    auto tree = chebyshev_path.has_value() ? chebyshev_tree(*chebyshev_path)
                                           : mean_element_tree(data_dir, *bodies_csv);
    if (!tree) {
        return std::unexpected(tree.error());
    }
    auto owned_tree = std::make_unique<frames::FrameTree>(std::move(*tree));
    auto catalog = bodies::parse_bodies_csv(*bodies_csv, *owned_tree, frames::icrf_to_j2000_ecliptic());
    if (!catalog) {
        return std::unexpected(catalog.error());
    }
    return Universe{.tree = std::move(owned_tree),
                    .catalog = std::make_unique<bodies::BodyCatalog>(std::move(*catalog))};
}

} // namespace helios::sim
