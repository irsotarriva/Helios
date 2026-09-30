#include "helios/frames/frame_tree.hpp"

#include <algorithm>
#include <format>
#include <utility>

namespace helios::frames {

namespace {

using core::ErrorCode;

// Rationale: std::get_if instead of std::visit, which may throw bad_variant_access.
template <typename Function>
[[nodiscard]] auto visit_motion(const AnyEphemeris& motion, const Function& function) noexcept
    -> decltype(function(std::declval<const FixedOffset&>())) {
    if (const auto* fixed = std::get_if<FixedOffset>(&motion)) {
        return function(*fixed);
    }
    if (const auto* keplerian = std::get_if<ephemeris::KeplerianEphemeris>(&motion)) {
        return function(*keplerian);
    }
    if (const auto* chebyshev = std::get_if<ephemeris::ChebyshevEphemeris>(&motion)) {
        return function(*chebyshev);
    }
    return core::fail(ErrorCode::Unknown, "frame motion model is valueless");
}

} // namespace

FrameTree::FrameTree(std::string root_name) {
    nodes_.push_back(Node{.name = std::move(root_name),
                          .parent = root(),
                          .depth = 0,
                          .motion_in_parent = FixedOffset{.position_in_parent_m = {}}});
}

core::VoidResult FrameTree::check(FrameId frame) const noexcept {
    if (frame.index >= nodes_.size()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("unknown frame id {}", frame.index));
    }
    return {};
}

core::Result<FrameId> FrameTree::add_frame(std::string name, FrameId parent, AnyEphemeris motion_in_parent) {
    if (core::VoidResult valid = check(parent); !valid) {
        return std::unexpected(valid.error());
    }
    if (name.empty()) {
        return core::fail(ErrorCode::InvalidArgument, "frame name must not be empty");
    }
    if (find(name).has_value()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("a frame named '{}' already exists", name));
    }
    const FrameId frame{static_cast<std::uint32_t>(nodes_.size())};
    const std::uint32_t parent_depth = node(parent).depth;
    nodes_.push_back(Node{.name = std::move(name),
                          .parent = parent,
                          .depth = parent_depth + 1,
                          .motion_in_parent = std::move(motion_in_parent)});
    return frame;
}

core::Result<FrameId> FrameTree::find(std::string_view name) const noexcept {
    const auto match = std::ranges::find(nodes_, name, &Node::name);
    if (match == nodes_.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("no frame named '{}'", name));
    }
    return FrameId{static_cast<std::uint32_t>(std::distance(nodes_.begin(), match))};
}

core::Result<std::string_view> FrameTree::name(FrameId frame) const noexcept {
    return check(frame).transform([&] { return std::string_view{node(frame).name}; });
}

core::Result<FrameId> FrameTree::parent(FrameId frame) const noexcept {
    return check(frame).and_then([&]() -> core::Result<FrameId> {
        if (node(frame).depth == 0) {
            return core::fail(ErrorCode::InvalidArgument, "the root frame has no parent");
        }
        return node(frame).parent;
    });
}

core::Result<std::uint32_t> FrameTree::depth(FrameId frame) const noexcept {
    return check(frame).transform([&] { return node(frame).depth; });
}

core::Result<orbital::StateVector> FrameTree::motion_state(FrameId frame,
                                                           const time::Epoch& instant) const noexcept {
    return visit_motion(node(frame).motion_in_parent,
                        [&](const auto& model) { return model.state_at(instant); });
}

core::Result<math::Vector3> FrameTree::motion_acceleration(FrameId frame,
                                                           const time::Epoch& instant) const noexcept {
    return visit_motion(node(frame).motion_in_parent,
                        [&](const auto& model) { return model.acceleration_at(instant); });
}

core::Result<orbital::StateVector> FrameTree::state_in_parent(FrameId frame,
                                                              const time::Epoch& instant) const noexcept {
    return check(frame).and_then([&] { return motion_state(frame, instant); });
}

core::Result<FrameId> FrameTree::lowest_common_ancestor(FrameId first, FrameId second) const noexcept {
    if (core::VoidResult valid = check(first).and_then([&] { return check(second); }); !valid) {
        return std::unexpected(valid.error());
    }
    while (node(first).depth > node(second).depth) {
        first = node(first).parent;
    }
    while (node(second).depth > node(first).depth) {
        second = node(second).parent;
    }
    while (first != second) {
        first = node(first).parent;
        second = node(second).parent;
    }
    return first;
}

core::Result<orbital::StateVector>
FrameTree::state_relative_to_ancestor(FrameId frame, FrameId ancestor,
                                      const time::Epoch& instant) const noexcept {
    orbital::StateVector sum{};
    for (FrameId current = frame; current != ancestor; current = node(current).parent) {
        const auto link = motion_state(current, instant);
        if (!link) {
            return core::fail(ErrorCode::OutOfRange, std::format("frame '{}': {}", node(current).name,
                                                                 core::describe(link.error())));
        }
        sum.position_m += link->position_m;
        sum.velocity_m_s += link->velocity_m_s;
    }
    return sum;
}

core::Result<math::Vector3>
FrameTree::acceleration_relative_to_ancestor(FrameId frame, FrameId ancestor,
                                             const time::Epoch& instant) const noexcept {
    math::Vector3 sum_m_s2{};
    for (FrameId current = frame; current != ancestor; current = node(current).parent) {
        const auto link = motion_acceleration(current, instant);
        if (!link) {
            return core::fail(ErrorCode::OutOfRange, std::format("frame '{}': {}", node(current).name,
                                                                 core::describe(link.error())));
        }
        sum_m_s2 += *link;
    }
    return sum_m_s2;
}

core::Result<orbital::StateVector> FrameTree::relative_state(FrameId target, FrameId observer,
                                                             const time::Epoch& instant) const noexcept {
    return lowest_common_ancestor(target, observer)
        .and_then([&](FrameId ancestor) -> core::Result<orbital::StateVector> {
            const auto target_state = state_relative_to_ancestor(target, ancestor, instant);
            const auto observer_state = target_state.and_then([&](const orbital::StateVector&) {
                return state_relative_to_ancestor(observer, ancestor, instant);
            });
            if (!observer_state) {
                return std::unexpected(observer_state.error());
            }
            return orbital::StateVector{.position_m = target_state->position_m - observer_state->position_m,
                                        .velocity_m_s =
                                            target_state->velocity_m_s - observer_state->velocity_m_s};
        });
}

core::Result<math::Vector3> FrameTree::relative_acceleration_m_s2(FrameId target, FrameId observer,
                                                                  const time::Epoch& instant) const noexcept {
    return lowest_common_ancestor(target, observer)
        .and_then([&](FrameId ancestor) -> core::Result<math::Vector3> {
            const auto target_acceleration = acceleration_relative_to_ancestor(target, ancestor, instant);
            const auto observer_acceleration = target_acceleration.and_then([&](const math::Vector3&) {
                return acceleration_relative_to_ancestor(observer, ancestor, instant);
            });
            if (!observer_acceleration) {
                return std::unexpected(observer_acceleration.error());
            }
            return *target_acceleration - *observer_acceleration;
        });
}

} // namespace helios::frames
