#ifndef HELIOS_BODIES_BODY_CATALOG_HPP
#define HELIOS_BODIES_BODY_CATALOG_HPP

#include "helios/core/error.hpp"
#include "helios/frames/body_rotation.hpp"
#include "helios/frames/frame_tree.hpp"
#include "helios/math/matrix3.hpp"

#include <compare>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::bodies {

struct BodyId {
    std::uint32_t index = 0;

    [[nodiscard]] friend constexpr auto operator<=>(const BodyId&, const BodyId&) noexcept = default;
};

// A massive, on-rails body: an anchor for vessel coordinates (BRIEFING §4.1) and a gravity source.
struct Body {
    std::string name;
    frames::FrameId frame; // the body's inertial frame (origin = its centre)
    double gravitational_parameter_m3_s2 = 0.0;
    double mean_radius_m = 0.0;
    // The domain (sphere of influence) hierarchy: where a vessel's coordinates are anchored.
    // It can differ from the frame tree (DE421 hangs Earth under the Earth–Moon barycentre,
    // but Earth's domain parent is the Sun).
    std::optional<BodyId> domain_parent;
    double domain_radius_m = std::numeric_limits<double>::infinity();
    std::optional<frames::BodyRotation> rotation;
};

class BodyCatalog {
public:
    // Fails on a duplicate name or frame, an unknown domain parent, non-finite or negative
    // μ / radius, or a finite domain radius missing for a body that has a parent.
    [[nodiscard]] core::Result<BodyId> add(Body body);

    [[nodiscard]] core::Result<BodyId> find(std::string_view name) const noexcept;
    [[nodiscard]] core::Result<std::reference_wrapper<const Body>> body(BodyId id) const noexcept;
    [[nodiscard]] std::optional<BodyId> body_on_frame(frames::FrameId frame) const noexcept;
    [[nodiscard]] std::vector<BodyId> domain_children(BodyId parent) const;
    [[nodiscard]] std::span<const Body> bodies() const noexcept { return bodies_; }

private:
    std::vector<Body> bodies_;
};

// Parses data/solar_system/bodies.csv (format documented in its header comment). Frames are
// resolved by name in `tree`; rotation angles are converted from the ICRF with
// `reference_to_universe`. Rows must list domain parents before their children.
[[nodiscard]] core::Result<BodyCatalog> parse_bodies_csv(std::string_view csv_text,
                                                         const frames::FrameTree& tree,
                                                         const math::Matrix3& reference_to_universe);

[[nodiscard]] core::Result<BodyCatalog> load_bodies_csv(const std::filesystem::path& path,
                                                        const frames::FrameTree& tree,
                                                        const math::Matrix3& reference_to_universe);

} // namespace helios::bodies

#endif // HELIOS_BODIES_BODY_CATALOG_HPP
