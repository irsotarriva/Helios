#include "helios/sim/scene_snapshot.hpp"

#include "helios/orbital/conic.hpp"

#include <functional>
#include <limits>

namespace helios::sim {

namespace {

using core::ErrorCode;
using math::Vector3;

// The focus as a frame plus an offset within it.
struct Origin {
    frames::FrameId frame;
    Vector3 offset_m;
};

class Placer {
public:
    Placer(const Simulation& simulation, const Origin& origin) noexcept
        : simulation_(simulation), origin_(origin) {}

    // Origin of `frame` relative to the focus.
    [[nodiscard]] core::Result<Vector3> frame_position_m(frames::FrameId frame) const noexcept {
        return simulation_.get()
            .tree()
            .relative_state(frame, origin_.frame, simulation_.get().now())
            .transform(
                [&](const orbital::StateVector& state) { return state.position_m - origin_.offset_m; });
    }

    [[nodiscard]] core::Result<Vector3> body_position_m(bodies::BodyId id) const noexcept {
        return simulation_.get().catalog().body(id).and_then(
            [&](const bodies::Body& body) { return frame_position_m(body.frame); });
    }

private:
    std::reference_wrapper<const Simulation> simulation_;
    Origin origin_;
};

[[nodiscard]] core::Result<Origin> origin_of(const Simulation& simulation, const Focus& focus,
                                             std::string& name, double& radius_m) {
    if (focus.kind == Focus::Kind::Body) {
        return simulation.catalog()
            .body(bodies::BodyId{focus.index})
            .transform([&](const bodies::Body& body) {
                name = body.name;
                radius_m = body.mean_radius_m;
                return Origin{.frame = body.frame, .offset_m = {}};
            });
    }
    const auto vessel = simulation.vessel(VesselId{focus.index});
    if (!vessel) {
        return std::unexpected(vessel.error());
    }
    name = vessel->get().name;
    radius_m = 0.0;
    return simulation.catalog().body(vessel->get().state.domain).transform([&](const bodies::Body& body) {
        return Origin{.frame = body.frame, .offset_m = vessel->get().state.state_in_domain.position_m};
    });
}

[[nodiscard]] core::VoidResult add_body_orbit(const Simulation& simulation, const Placer& placer,
                                              bodies::BodyId id, const bodies::Body& body,
                                              const SnapshotOptions& options, std::vector<LineView>& lines) {
    if (!body.domain_parent.has_value()) {
        return {};
    }
    const auto parent = simulation.catalog().body(*body.domain_parent);
    if (!parent) {
        return std::unexpected(parent.error());
    }
    const auto relative = simulation.tree().relative_state(body.frame, parent->get().frame, simulation.now());
    if (!relative) {
        return std::unexpected(relative.error());
    }
    // The relative two-body orbit uses μ_parent + μ_body.
    const auto conic = orbital::conic_geometry(*relative, parent->get().gravitational_parameter_m3_s2
                                                              + body.gravitational_parameter_m3_s2);
    if (!conic) {
        return {}; // e.g. a fixed offset: no orbit to draw
    }
    const double clip_m = std::isfinite(parent->get().domain_radius_m)
                              ? std::max(parent->get().domain_radius_m, conic->periapsis_radius_m)
                              : std::numeric_limits<double>::max();
    const auto arc = orbital::sample_conic(*conic, clip_m, options.body_orbit_points);
    const auto centre_m = placer.frame_position_m(parent->get().frame);
    if (!arc || !centre_m) {
        return std::unexpected(!arc ? arc.error() : centre_m.error());
    }
    LineView line{.kind = LineKind::BodyOrbit,
                  .owner = id.index,
                  .frame_body = *body.domain_parent,
                  .points_m = {},
                  .closed = arc->closed};
    line.points_m.reserve(arc->points_m.size());
    for (const Vector3& point_m : arc->points_m) {
        line.points_m.push_back(*centre_m + point_m);
    }
    lines.push_back(std::move(line));
    return {};
}

[[nodiscard]] core::VoidResult add_vessel_trajectory(const Simulation& simulation, const Placer& placer,
                                                     VesselId id, const Vessel& vessel,
                                                     const Vector3& vessel_position_m,
                                                     std::vector<LineView>& lines) {
    bool first = true;
    for (const dynamics::TrajectorySegment& segment : vessel.prediction) {
        const auto domain = simulation.catalog().body(segment.domain);
        if (!domain) {
            return std::unexpected(domain.error());
        }
        const auto centre_m = placer.frame_position_m(domain->get().frame);
        if (!centre_m) {
            return std::unexpected(centre_m.error());
        }
        LineView line{.kind = LineKind::VesselTrajectory,
                      .owner = id.index,
                      .frame_body = segment.domain,
                      .points_m = {},
                      .closed = false};
        line.points_m.reserve(segment.samples.size() + 1);
        if (first && segment.domain == vessel.state.domain) {
            line.points_m.push_back(vessel_position_m);
        }
        for (const dynamics::TrajectorySample& sample : segment.samples) {
            if (sample.epoch > simulation.now()) {
                line.points_m.push_back(*centre_m + sample.position_m);
            }
        }
        first = false;
        if (line.points_m.size() >= 2) {
            lines.push_back(std::move(line));
        }
    }
    return {};
}

} // namespace

core::Result<SceneSnapshot> build_snapshot(const Simulation& simulation, const Focus& focus,
                                           const SnapshotOptions& options) {
    if (options.body_orbit_points < 4) {
        return core::fail(ErrorCode::InvalidArgument, "orbit lines need at least 4 points");
    }
    SceneSnapshot snapshot;
    const auto origin = origin_of(simulation, focus, snapshot.focus_name, snapshot.focus_radius_m);
    if (!origin) {
        return std::unexpected(origin.error());
    }
    const Placer placer(simulation, *origin);
    const time::Epoch& now = simulation.now();
    snapshot.epoch = now;
    snapshot.warp_factor = simulation.effective_warp();
    snapshot.requested_warp_factor = simulation.time_warp().requested_factor();
    snapshot.paused = simulation.time_warp().paused();
    snapshot.focus = focus;

    const auto all_bodies = simulation.catalog().bodies();
    snapshot.bodies.reserve(all_bodies.size());
    for (std::uint32_t index = 0; index < all_bodies.size(); ++index) {
        const bodies::Body& body = all_bodies[index];
        const auto position_m = placer.frame_position_m(body.frame);
        if (!position_m) {
            return std::unexpected(position_m.error());
        }
        snapshot.bodies.push_back(
            BodyView{.id = {index},
                     .name = body.name,
                     .position_m = *position_m,
                     .radius_m = body.mean_radius_m,
                     .domain_parent = body.domain_parent,
                     .body_to_universe = body.rotation.has_value()
                                             ? math::transpose(body.rotation->universe_to_body_fixed(now))
                                             : math::Matrix3{}});
        if (core::VoidResult added =
                add_body_orbit(simulation, placer, {index}, body, options, snapshot.lines);
            !added) {
            return std::unexpected(added.error());
        }
    }

    const auto vessels = simulation.vessels();
    for (std::uint32_t index = 0; index < vessels.size(); ++index) {
        const Vessel& vessel = vessels[index];
        const auto domain_m = placer.body_position_m(vessel.state.domain);
        if (!domain_m) {
            return std::unexpected(domain_m.error());
        }
        const Vector3 position_m = *domain_m + vessel.state.state_in_domain.position_m;
        const bodies::Body& domain = simulation.catalog().body(vessel.state.domain)->get();
        const auto conic =
            orbital::conic_geometry(vessel.state.state_in_domain, domain.gravitational_parameter_m3_s2);
        snapshot.vessels.push_back(
            VesselView{.id = {index},
                       .name = vessel.name,
                       .status = vessel.status,
                       .position_m = position_m,
                       .velocity_in_domain_m_s = vessel.state.state_in_domain.velocity_m_s,
                       .domain = vessel.state.domain,
                       .domain_name = domain.name,
                       .domain_radius_m = domain.mean_radius_m,
                       .osculating = conic ? std::optional(*conic) : std::nullopt,
                       .pending_burns = vessel.propagator.pending_impulses().size()});
        if (vessel.status == VesselStatus::Flying) {
            if (core::VoidResult added =
                    add_vessel_trajectory(simulation, placer, {index}, vessel, position_m, snapshot.lines);
                !added) {
                return std::unexpected(added.error());
            }
        }
    }

    if (const auto limit = simulation.warp_limit()) {
        snapshot.next_event = EventView{.vessel = limit->vessel,
                                        .kind = limit->kind,
                                        .body = limit->body,
                                        .time_to_event_s = time::seconds_between(now, limit->epoch)};
    }
    return snapshot;
}

void SnapshotExchange::publish(std::shared_ptr<const SceneSnapshot> snapshot) {
    const std::scoped_lock lock(mutex_);
    latest_ = std::move(snapshot);
}

std::shared_ptr<const SceneSnapshot> SnapshotExchange::latest() const {
    const std::scoped_lock lock(mutex_);
    return latest_;
}

} // namespace helios::sim
