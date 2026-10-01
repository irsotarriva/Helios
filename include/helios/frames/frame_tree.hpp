#ifndef HELIOS_FRAMES_FRAME_TREE_HPP
#define HELIOS_FRAMES_FRAME_TREE_HPP

#include "helios/core/error.hpp"
#include "helios/ephemeris/chebyshev_ephemeris.hpp"
#include "helios/ephemeris/keplerian_ephemeris.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/time/epoch.hpp"

#include <compare>
#include <concepts>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace helios::frames {

// A frame whose origin sits at a constant offset from its parent's origin (e.g. a
// barycentre-coincident body such as Mercury in DE440, or a test fixture).
struct FixedOffset {
    math::Vector3 position_in_parent_m;

    [[nodiscard]] core::Result<orbital::StateVector> state_at(const time::Epoch& /*instant*/) const noexcept {
        return orbital::StateVector{.position_m = position_in_parent_m, .velocity_m_s = {}};
    }
    // NOLINTBEGIN(readability-convert-member-functions-to-static): EphemerisModel interface.
    [[nodiscard]] core::Result<math::Vector3> acceleration_at(const time::Epoch& /*instant*/) const noexcept {
        return math::Vector3{};
    }
    [[nodiscard]] std::optional<time::Interval> valid_range() const noexcept { return std::nullopt; }
    // NOLINTEND(readability-convert-member-functions-to-static)
};

// What every on-rails body model must provide (BRIEFING §5.1): a pure function of time,
// independent of every other body, with an acceleration that is the true second derivative.
template <typename Model>
concept EphemerisModel = requires(const Model& model, const time::Epoch& instant) {
    { model.state_at(instant) } -> std::same_as<core::Result<orbital::StateVector>>;
    { model.acceleration_at(instant) } -> std::same_as<core::Result<math::Vector3>>;
    { model.valid_range() } -> std::same_as<std::optional<time::Interval>>;
};

static_assert(EphemerisModel<FixedOffset>);
static_assert(EphemerisModel<ephemeris::KeplerianEphemeris>);
static_assert(EphemerisModel<ephemeris::ChebyshevEphemeris>);

// Rationale: a closed set chosen at compile time, dispatched with std::visit.
// TODO: add a type-erased alternative for models supplied by native mod plugins.
using AnyEphemeris = std::variant<FixedOffset, ephemeris::KeplerianEphemeris, ephemeris::ChebyshevEphemeris>;

struct FrameId {
    std::uint32_t index = 0;

    [[nodiscard]] friend constexpr auto operator<=>(const FrameId&, const FrameId&) noexcept = default;
};

// The hierarchy of non-rotating reference frames anchored on large bodies (BRIEFING §4).
// All frames share the same inertial axes (the J2000 ecliptic for the stock Solar System);
// only their origins move. Frames are added parent-first, so the tree is acyclic by construction.
class FrameTree {
public:
    explicit FrameTree(std::string root_name);

    [[nodiscard]] static constexpr FrameId root() noexcept { return FrameId{0}; }
    [[nodiscard]] std::size_t size() const noexcept { return nodes_.size(); }

    // Fails if `parent` is unknown, `name` is empty or already used.
    [[nodiscard]] core::Result<FrameId> add_frame(std::string name, FrameId parent,
                                                  AnyEphemeris motion_in_parent);

    [[nodiscard]] core::Result<FrameId> find(std::string_view name) const noexcept;
    [[nodiscard]] core::Result<std::string_view> name(FrameId frame) const noexcept;
    // The root has no parent: returns an error.
    [[nodiscard]] core::Result<FrameId> parent(FrameId frame) const noexcept;
    [[nodiscard]] core::Result<std::uint32_t> depth(FrameId frame) const noexcept;

    // Origin of `frame` relative to its parent's origin.
    [[nodiscard]] core::Result<orbital::StateVector>
    state_in_parent(FrameId frame, const time::Epoch& instant) const noexcept;

    // Origin of `target` relative to the origin of `observer`.
    //
    // Rationale: the two chains are summed only up to their lowest common ancestor, never to
    // the root. Relative states of nearby frames therefore stay exact even when the root is
    // 1e20 m away (the "hierarchical coordinates" requirement of BRIEFING §4 and §3.1).
    [[nodiscard]] core::Result<orbital::StateVector>
    relative_state(FrameId target, FrameId observer, const time::Epoch& instant) const noexcept;

    // Acceleration of `target`'s origin relative to `observer`'s origin; the observer's term
    // is exactly the indirect acceleration of BRIEFING §4.3.
    [[nodiscard]] core::Result<math::Vector3>
    relative_acceleration_m_s2(FrameId target, FrameId observer, const time::Epoch& instant) const noexcept;

    [[nodiscard]] core::Result<FrameId> lowest_common_ancestor(FrameId first, FrameId second) const noexcept;

private:
    struct Node {
        std::string name;
        FrameId parent; // the root is its own parent; depth 0 identifies it
        std::uint32_t depth = 0;
        AnyEphemeris motion_in_parent;
    };

    [[nodiscard]] core::VoidResult check(FrameId frame) const noexcept;
    [[nodiscard]] const Node& node(FrameId frame) const noexcept { return nodes_[frame.index]; }
    [[nodiscard]] core::Result<orbital::StateVector> motion_state(FrameId frame,
                                                                  const time::Epoch& instant) const noexcept;
    [[nodiscard]] core::Result<math::Vector3> motion_acceleration(FrameId frame,
                                                                  const time::Epoch& instant) const noexcept;

    // Sum of `frame`'s chain of parent-relative states up to (excluding) `ancestor`.
    [[nodiscard]] core::Result<orbital::StateVector>
    state_relative_to_ancestor(FrameId frame, FrameId ancestor, const time::Epoch& instant) const noexcept;
    [[nodiscard]] core::Result<math::Vector3>
    acceleration_relative_to_ancestor(FrameId frame, FrameId ancestor,
                                      const time::Epoch& instant) const noexcept;

    std::vector<Node> nodes_;
};

} // namespace helios::frames

#endif // HELIOS_FRAMES_FRAME_TREE_HPP
