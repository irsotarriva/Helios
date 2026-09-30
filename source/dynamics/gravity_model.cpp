#include "helios/dynamics/gravity_model.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace helios::dynamics {

namespace {

using core::ErrorCode;
using math::Vector3;

constexpr double k_extent_safety_factor = 1.25;

[[nodiscard]] Vector3 point_mass_acceleration_m_s2(const Vector3& source_minus_point_m,
                                                   double gravitational_parameter_m3_s2) noexcept {
    const double distance_m = math::norm(source_minus_point_m);
    return source_minus_point_m * (gravitational_parameter_m3_s2 / (distance_m * distance_m * distance_m));
}

} // namespace

core::Result<GravityModel> GravityModel::make(const frames::FrameTree& tree,
                                              const bodies::BodyCatalog& catalog, GravityOptions options,
                                              const time::Epoch& extent_epoch) {
    if (!std::isfinite(options.opening_angle) || options.opening_angle < 0.0) {
        return core::fail(ErrorCode::OutOfRange, "opening angle must be finite and non-negative");
    }
    std::vector<Node> nodes(tree.size());
    for (const bodies::Body& body : catalog.bodies()) {
        nodes.at(body.frame.index).own_gravitational_parameter_m3_s2 = body.gravitational_parameter_m3_s2;
    }
    for (std::uint32_t index = 1; index < tree.size(); ++index) {
        const auto parent = tree.parent(frames::FrameId{index});
        if (!parent) {
            return std::unexpected(parent.error());
        }
        nodes.at(parent->index).children.push_back(frames::FrameId{index});
    }
    // Children are always added after their parents, so a reverse sweep sees children first.
    for (auto index = static_cast<std::uint32_t>(tree.size()); index-- > 0;) {
        Node& node = nodes.at(index);
        node.subtree_gravitational_parameter_m3_s2 = node.own_gravitational_parameter_m3_s2;
        for (const frames::FrameId child : node.children) {
            const Node& child_node = nodes.at(child.index);
            node.subtree_gravitational_parameter_m3_s2 += child_node.subtree_gravitational_parameter_m3_s2;
            const auto offset = tree.state_in_parent(child, extent_epoch);
            if (!offset) {
                return core::fail(ErrorCode::OutOfRange, std::format("cannot measure subsystem extents: {}",
                                                                     core::describe(offset.error())));
            }
            node.extent_m = std::max(node.extent_m, (k_extent_safety_factor * math::norm(offset->position_m))
                                                        + child_node.extent_m);
        }
    }
    return GravityModel{tree, catalog, options, std::move(nodes)};
}

bool GravityModel::is_ancestor_or_self(frames::FrameId ancestor, frames::FrameId frame) const noexcept {
    for (;;) {
        if (frame == ancestor) {
            return true;
        }
        const auto parent = tree().parent(frame);
        if (!parent) {
            return false;
        }
        frame = *parent;
    }
}

template <typename Visitor>
core::VoidResult GravityModel::walk(frames::FrameId root, frames::FrameId anchor_frame,
                                    const Vector3& position_m, const time::Epoch& instant,
                                    const Visitor& visit) const {
    // Rationale: an explicit stack rather than recursion; the order of visits (and therefore
    // the floating-point summation order) is still fixed, so results stay deterministic.
    std::vector<frames::FrameId> pending{root};
    while (!pending.empty()) {
        const frames::FrameId node_id = pending.back();
        pending.pop_back();
        const Node& node = nodes_[node_id.index];
        if (node.subtree_gravitational_parameter_m3_s2 == 0.0) {
            continue;
        }
        const bool contains_anchor = is_ancestor_or_self(node_id, anchor_frame);
        const auto origin = tree().relative_state(node_id, anchor_frame, instant);
        if (!origin) {
            return std::unexpected(origin.error());
        }
        if (!contains_anchor && !node.children.empty()) {
            // Monopole of the unopened subsystem. For a barycentre node the origin *is* the centre
            // of mass; for a body with satellites, weight in the children's positions (one level).
            Vector3 centre_of_mass_m = origin->position_m * node.own_gravitational_parameter_m3_s2;
            for (const frames::FrameId child : node.children) {
                const auto child_origin = tree().relative_state(child, anchor_frame, instant);
                if (!child_origin) {
                    return std::unexpected(child_origin.error());
                }
                centre_of_mass_m +=
                    child_origin->position_m * nodes_[child.index].subtree_gravitational_parameter_m3_s2;
            }
            centre_of_mass_m = centre_of_mass_m / node.subtree_gravitational_parameter_m3_s2;
            if (node.extent_m < options_.opening_angle * math::norm(centre_of_mass_m - position_m)) {
                visit(node_id, centre_of_mass_m, node.subtree_gravitational_parameter_m3_s2, true);
                continue;
            }
        }
        if (node.own_gravitational_parameter_m3_s2 > 0.0 && node_id != anchor_frame) {
            visit(node_id, origin->position_m, node.own_gravitational_parameter_m3_s2, false);
        }
        pending.insert(pending.end(), node.children.rbegin(), node.children.rend());
    }
    return {};
}

core::Result<Vector3> GravityModel::perturbing_acceleration_m_s2(bodies::BodyId anchor,
                                                                 const Vector3& position_m,
                                                                 const time::Epoch& instant) const noexcept {
    const auto anchor_body = catalog().body(anchor);
    if (!anchor_body) {
        return std::unexpected(anchor_body.error());
    }
    const frames::FrameId anchor_frame = anchor_body->get().frame;

    Vector3 acceleration_m_s2{};
    const auto accumulate = [&](frames::FrameId /*frame*/, const Vector3& source_position_m, double mu_m3_s2,
                                bool /*is_aggregate*/) {
        acceleration_m_s2 += point_mass_acceleration_m_s2(source_position_m - position_m, mu_m3_s2);
    };
    if (core::VoidResult walked =
            walk(frames::FrameTree::root(), anchor_frame, position_m, instant, accumulate);
        !walked) {
        return std::unexpected(walked.error());
    }
    return tree()
        .relative_acceleration_m_s2(anchor_frame, frames::FrameTree::root(), instant)
        .transform([&](const Vector3& anchor_acceleration_m_s2) {
            return acceleration_m_s2 - anchor_acceleration_m_s2;
        });
}

core::Result<Vector3> GravityModel::total_acceleration_m_s2(bodies::BodyId anchor, const Vector3& position_m,
                                                            const time::Epoch& instant) const noexcept {
    const auto anchor_body = catalog().body(anchor);
    if (!anchor_body) {
        return std::unexpected(anchor_body.error());
    }
    const double central_mu_m3_s2 = anchor_body->get().gravitational_parameter_m3_s2;
    return perturbing_acceleration_m_s2(anchor, position_m, instant)
        .transform([&](const Vector3& perturbation_m_s2) {
            return perturbation_m_s2 + orbital::two_body_acceleration_m_s2(position_m, central_mu_m3_s2);
        });
}

core::Result<std::vector<GravitySource>> GravityModel::selected_sources(bodies::BodyId anchor,
                                                                        const Vector3& position_m,
                                                                        const time::Epoch& instant) const {
    const auto anchor_body = catalog().body(anchor);
    if (!anchor_body) {
        return std::unexpected(anchor_body.error());
    }
    std::vector<GravitySource> sources;
    const auto record = [&](frames::FrameId frame, const Vector3& /*position*/, double mu_m3_s2,
                            bool is_aggregate) {
        sources.push_back(GravitySource{
            .frame = frame, .gravitational_parameter_m3_s2 = mu_m3_s2, .is_aggregate = is_aggregate});
    };
    return walk(frames::FrameTree::root(), anchor_body->get().frame, position_m, instant, record)
        .transform([&] { return sources; });
}

} // namespace helios::dynamics
