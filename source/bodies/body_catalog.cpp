#include "helios/bodies/body_catalog.hpp"

#include "helios/core/parse.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <iterator>
#include <numbers>
#include <utility>

namespace helios::bodies {

namespace {

using core::ErrorCode;

constexpr double k_degree_to_rad = std::numbers::pi / 180.0;
constexpr double k_seconds_per_julian_century = 36'525.0 * 86'400.0;
constexpr std::size_t k_column_count = 12;

[[nodiscard]] core::Result<double> parse_column(std::string_view field, std::string_view column,
                                                std::size_t line_number) {
    return core::parse_double(field).transform_error([&](core::Error error) {
        error.context = std::format("line {}, {}: {}", line_number, column, error.context);
        return error;
    });
}

} // namespace

core::Result<BodyId> BodyCatalog::add(Body body) {
    if (body.name.empty() || find(body.name).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("body name '{}' is empty or duplicated", body.name));
    }
    if (body_on_frame(body.frame).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("frame of '{}' already has a body", body.name));
    }
    if (!std::isfinite(body.gravitational_parameter_m3_s2) || body.gravitational_parameter_m3_s2 < 0.0
        || !std::isfinite(body.mean_radius_m) || body.mean_radius_m <= 0.0) {
        return core::fail(ErrorCode::OutOfRange,
                          std::format("body '{}' has an invalid μ or radius", body.name));
    }
    if (body.domain_parent.has_value()) {
        if (body.domain_parent->index >= bodies_.size()) {
            return core::fail(ErrorCode::InvalidArgument,
                              std::format("unknown domain parent for '{}'", body.name));
        }
        if (!std::isfinite(body.domain_radius_m) || body.domain_radius_m <= body.mean_radius_m) {
            return core::fail(
                ErrorCode::OutOfRange,
                std::format("body '{}' needs a finite domain radius above its surface", body.name));
        }
    }
    bodies_.push_back(std::move(body));
    return BodyId{static_cast<std::uint32_t>(bodies_.size() - 1)};
}

core::Result<BodyId> BodyCatalog::find(std::string_view name) const noexcept {
    const auto match = std::ranges::find(bodies_, name, &Body::name);
    if (match == bodies_.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("no body named '{}'", name));
    }
    return BodyId{static_cast<std::uint32_t>(std::distance(bodies_.begin(), match))};
}

core::Result<std::reference_wrapper<const Body>> BodyCatalog::body(BodyId id) const noexcept {
    if (id.index >= bodies_.size()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("unknown body id {}", id.index));
    }
    return std::cref(bodies_[id.index]);
}

std::optional<BodyId> BodyCatalog::body_on_frame(frames::FrameId frame) const noexcept {
    const auto match = std::ranges::find(bodies_, frame, &Body::frame);
    if (match == bodies_.end()) {
        return std::nullopt;
    }
    return BodyId{static_cast<std::uint32_t>(std::distance(bodies_.begin(), match))};
}

std::vector<BodyId> BodyCatalog::domain_children(BodyId parent) const {
    std::vector<BodyId> children;
    for (std::uint32_t index = 0; index < bodies_.size(); ++index) {
        if (bodies_[index].domain_parent == parent) {
            children.push_back(BodyId{index});
        }
    }
    return children;
}

core::Result<BodyCatalog> parse_bodies_csv(std::string_view csv_text, const frames::FrameTree& tree,
                                           const math::Matrix3& reference_to_universe) {
    BodyCatalog catalog;
    bool header_seen = false;
    std::size_t line_number = 0;
    while (!csv_text.empty()) {
        const std::size_t line_end = csv_text.find('\n');
        const std::string_view line = core::trim(csv_text.substr(0, line_end));
        csv_text = line_end == std::string_view::npos ? std::string_view{} : csv_text.substr(line_end + 1);
        ++line_number;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        if (!header_seen) {
            header_seen = true;
            continue;
        }
        const std::vector<std::string_view> fields = core::split_csv_line(line);
        if (fields.size() != k_column_count) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: expected {} columns, found {}", line_number,
                                          k_column_count, fields.size()));
        }

        Body body;
        body.name = std::string{fields[0]};
        const auto frame = tree.find(fields[1]);
        const auto mu = parse_column(fields[2], "gravitational_parameter_m3_s2", line_number);
        const auto radius = parse_column(fields[3], "mean_radius_m", line_number);
        if (!frame) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: {}", line_number, core::describe(frame.error())));
        }
        if (!mu) {
            return std::unexpected(mu.error());
        }
        if (!radius) {
            return std::unexpected(radius.error());
        }
        body.frame = *frame;
        body.gravitational_parameter_m3_s2 = *mu;
        body.mean_radius_m = *radius;

        if (!fields[4].empty()) {
            const auto parent = catalog.find(fields[4]);
            const auto domain_radius = parse_column(fields[5], "domain_radius_m", line_number);
            if (!parent || !domain_radius) {
                return core::fail(ErrorCode::ParseFailure,
                                  std::format("line {}: bad domain parent or radius", line_number));
            }
            body.domain_parent = *parent;
            body.domain_radius_m = *domain_radius;
        }

        const bool has_rotation = std::ranges::any_of(std::span{fields}.subspan(6),
                                                      [](std::string_view field) { return !field.empty(); });
        if (has_rotation) {
            std::array<double, 6> values{};
            for (std::size_t column = 0; column < values.size(); ++column) {
                const auto value = parse_column(fields[6 + column], "rotation", line_number);
                if (!value) {
                    return std::unexpected(value.error());
                }
                values.at(column) = *value;
            }
            const frames::RotationElements elements{
                .pole_right_ascension_rad = values[0] * k_degree_to_rad,
                .pole_right_ascension_rate_rad_s = values[1] * k_degree_to_rad / k_seconds_per_julian_century,
                .pole_declination_rad = values[2] * k_degree_to_rad,
                .pole_declination_rate_rad_s = values[3] * k_degree_to_rad / k_seconds_per_julian_century,
                .prime_meridian_rad = values[4] * k_degree_to_rad,
                .rotation_rate_rad_s = values[5] * k_degree_to_rad / 86'400.0};
            auto rotation = frames::BodyRotation::make(elements, reference_to_universe);
            if (!rotation) {
                return std::unexpected(rotation.error());
            }
            body.rotation = *rotation;
        }

        if (auto added = catalog.add(std::move(body)); !added) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: {}", line_number, core::describe(added.error())));
        }
    }
    if (!header_seen) {
        return core::fail(ErrorCode::ParseFailure, "bodies CSV has no header row");
    }
    return catalog;
}

core::Result<BodyCatalog> load_bodies_csv(const std::filesystem::path& path, const frames::FrameTree& tree,
                                          const math::Matrix3& reference_to_universe) {
    auto contents =
        core::try_call(ErrorCode::IoFailure, "reading bodies CSV", [&]() -> core::Result<std::string> {
            std::ifstream stream(path);
            if (!stream) {
                return core::fail(ErrorCode::FileNotFound, std::format("cannot open '{}'", path.string()));
            }
            return std::string{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
        });
    if (!contents) {
        return std::unexpected(contents.error());
    }
    return contents->and_then(
        [&](const std::string& text) { return parse_bodies_csv(text, tree, reference_to_universe); });
}

} // namespace helios::bodies
