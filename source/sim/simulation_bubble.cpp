// The physics bubble (BRIEFING §5.3, D25): the part of Simulation that flies one vessel as a
// rigid body.
//
// Division of labour. The Encke propagator stays the owner of the vessel's place and
// velocity, because nothing integrates gravity better. The rigid-body engine works in a small
// frame that starts each tick at the vessel's centre of mass, and supplies what the propagator
// cannot: the turning of the vessel under torques, and the pushes that are not gravity.
//
//  - Away from the ground the frame falls with the vessel, so in it there is no gravity at
//    all. The engine turns the body and reports the change of velocity the thrust gave it,
//    which goes to the propagator as an impulse at the middle of the tick.
//  - Near the ground the frame moves with the ground under the vessel, the ground is a plane,
//    and the engine also integrates the fall (under the apparent gravity of that frame) and
//    resolves the contact. The propagator is then restarted from the engine's result every
//    tick, until the vessel is clear of the ground or has come to rest on it.
//
// A vessel at rest on the ground is taken out of the bubble and pinned to the body-fixed frame
// (VesselStatus::Landed): closed-form like everything else on rails, so it costs nothing and
// survives any warp.

#include "helios/core/logging.hpp"
#include "helios/sim/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <string_view>
#include <utility>

#include "bubble.hpp"

namespace helios::sim {

namespace {

using core::ErrorCode;
using math::Matrix3;
using math::Quaternion;
using math::Vector3;

// Below this altitude of the centre of mass the ground is in the bubble.
constexpr double k_contact_altitude_m = 200.0;
// A vessel touching the ground this slowly, for this long, with nothing pushing it, has landed.
constexpr double k_rest_speed_m_s = 0.05;
constexpr double k_rest_spin_rad_s = 0.01;
constexpr int k_rest_ticks = 64;
// The rigid body is rebuilt when the centre of mass has moved this far in the vessel.
constexpr double k_body_rebuild_shift_m = 0.01;
constexpr double k_ground_friction = 0.8;
// While something pushes the vessel, its predicted path is redrawn this often (simulated time).
constexpr double k_pushed_prediction_period_s = 0.25;
// A flying vessel this near the active one is a rigid body with it, and stays one until it is
// further than the second distance (so that it does not come and go at the edge).
constexpr double k_bubble_join_distance_m = 400.0;
constexpr double k_bubble_leave_distance_m = 500.0;
// Smaller displacements than this are the engine's rounding, not its parting of two bodies.
constexpr double k_contact_correction_m = 1e-4;

// Attitude hold: the turn rate asked for per radian of error, how hard a rate error is
// corrected, and the resolution of the commands (so a steady hold repeats the same command).
constexpr double k_hold_rate_per_s = 1.0;
constexpr double k_hold_rate_gain_per_s = 4.0;
constexpr double k_hold_command_steps = 1024.0;
constexpr std::array<std::string_view, 3> k_attitude_signals{"attitude/roll", "attitude/pitch",
                                                             "attitude/yaw"};

// The rotating frame of a body at an instant; a body without a rotation model does not turn.
struct BodyFrame {
    Matrix3 to_fixed; // universe axes → body-fixed axes
    Vector3 angular_velocity_rad_s;
};

[[nodiscard]] BodyFrame body_frame(const bodies::Body& body, const time::Epoch& instant) noexcept {
    if (!body.rotation.has_value()) {
        return {};
    }
    return {.to_fixed = body.rotation->universe_to_body_fixed(instant),
            .angular_velocity_rad_s = body.rotation->angular_velocity_rad_s(instant)};
}

[[nodiscard]] Vector3 unit_or(const Vector3& vector, const Vector3& fallback) noexcept {
    const double length = math::norm(vector);
    return length > 0.0 && std::isfinite(length) ? vector / length : fallback;
}

// The orientation whose x axis is `nose`, with its z axis as close to `towards` as that allows.
[[nodiscard]] Quaternion orientation_with_nose(const Vector3& nose, const Vector3& towards) noexcept {
    Vector3 side = math::cross(towards, nose);
    if (math::norm(side) < 1e-9) {
        // `towards` is along the nose: any direction across it will do.
        side = math::cross(std::abs(nose.x) < 0.9 ? Vector3{1.0, 0.0, 0.0} : Vector3{0.0, 1.0, 0.0}, nose);
    }
    const Vector3 y_axis = unit_or(side, {0.0, 1.0, 0.0});
    const Vector3 z_axis = math::cross(nose, y_axis);
    return math::from_matrix(
        Matrix3{{nose.x, y_axis.x, z_axis.x, nose.y, y_axis.y, z_axis.y, nose.z, y_axis.z, z_axis.z}});
}

// The pieces of the rigid body: one per part that has a shape, about the centre of mass.
[[nodiscard]] std::vector<physics::ShapePiece> shape_pieces(const vessel::Assembly& assembly,
                                                            const Vector3& centre_of_mass_m) {
    std::vector<physics::ShapePiece> pieces;
    const auto parts = assembly.parts();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cylinder>& shape = parts[index].datasheet->shape;
        if (!shape.has_value()) {
            continue;
        }
        const vessel::PartPose& pose = assembly.poses()[index];
        pieces.push_back(physics::ShapePiece{.radius_m = shape->radius_m,
                                             .half_length_m = 0.5 * shape->length_m,
                                             .position_m = pose.position_m - centre_of_mass_m,
                                             .orientation = math::from_matrix(pose.orientation)});
    }
    return pieces;
}

// How far below the centre of mass the vessel's lowest point is when it stands nose up.
[[nodiscard]] double standing_height_m(const vessel::Assembly& assembly, const Vector3& centre_of_mass_m) {
    double lowest_m = std::numeric_limits<double>::infinity();
    const auto parts = assembly.parts();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cylinder>& shape = parts[index].datasheet->shape;
        if (!shape.has_value()) {
            continue;
        }
        const vessel::PartPose& pose = assembly.poses()[index];
        // The reach of a cylinder along the vessel's x: its half length along its own axis and
        // its radius across it.
        const double along = (pose.orientation * Vector3{1.0, 0.0, 0.0}).x;
        const double reach_m = std::abs(along) * 0.5 * shape->length_m
                               + shape->radius_m * std::sqrt(std::max(0.0, 1.0 - along * along));
        lowest_m = std::min(lowest_m, pose.position_m.x - reach_m);
    }
    return std::isfinite(lowest_m) ? centre_of_mass_m.x - lowest_m : 0.0;
}

struct LandedPose {
    orbital::StateVector state; // about the body, universe axes
    Attitude attitude;
};

// Where a place on the turning body is at `instant`, and how a vessel standing there is turned.
[[nodiscard]] LandedPose landed_pose(const bodies::Body& body, const LandedPlace& place,
                                     const time::Epoch& instant) noexcept {
    const BodyFrame frame = body_frame(body, instant);
    const Matrix3 to_universe = math::transpose(frame.to_fixed);
    const Vector3 position_m = to_universe * place.position_m;
    return LandedPose{
        .state = {.position_m = position_m,
                  .velocity_m_s = math::cross(frame.angular_velocity_rad_s, position_m)},
        .attitude = {.orientation = math::normalized(math::from_matrix(to_universe) * place.orientation),
                     .angular_velocity_rad_s = frame.angular_velocity_rad_s}};
}

[[nodiscard]] double least_impact_tolerance_m_s(const vessel::Assembly& assembly) {
    double tolerance_m_s = std::numeric_limits<double>::infinity();
    for (const vessel::PartInstance& part : assembly.parts()) {
        tolerance_m_s = std::min(tolerance_m_s, part.datasheet->impact_tolerance_m_s);
    }
    return tolerance_m_s;
}

// Withdraws the attitude hold's commands, so that the wheels stop: on rails and on the ground
// nothing is to be turned.
[[nodiscard]] core::VoidResult release_attitude_hold(Vessel& vessel, const time::Epoch& instant) {
    if (!vessel.systems.has_value()) {
        return {};
    }
    vessel::VesselSystems& systems = *vessel.systems;
    std::vector<vessel::Separation> separations;
    for (const std::string_view signal : k_attitude_signals) {
        if (!systems.bus().find(signal).has_value()) {
            continue;
        }
        if (core::VoidResult released =
                systems.release(signal, vessel::ControlSource::Autopilot,
                                std::max(instant, systems.propulsion().epoch), separations);
            !released) {
            return released;
        }
    }
    return {};
}

} // namespace

Attitude detail::pointing_attitude(const vessel::Pointing& pointing,
                                   const orbital::StateVector& state) noexcept {
    const auto nose = dynamics::thrust_unit_vector(to_thrust_direction(pointing), state);
    // Roll is not commanded: the vessel's z axis is kept towards the orbit's normal.
    const Vector3 normal = unit_or(math::cross(state.position_m, state.velocity_m_s), Vector3{0.0, 0.0, 1.0});
    return Attitude{.orientation = orientation_with_nose(nose ? *nose : Vector3{1.0, 0.0, 0.0}, normal),
                    .angular_velocity_rad_s = {}};
}

// Rationale: defined here, where Bubble is a complete type for the unique_ptr.
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;
Simulation::~Simulation() = default;

core::VoidResult Simulation::set_active_vessel(std::optional<VesselId> id) {
    if (id.has_value() && id->index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    active_vessel_ = id;
    return {};
}

bool Simulation::in_bubble(VesselId id) const noexcept {
    return bubble_ != nullptr && bubble_->find(id.index) != nullptr;
}

core::Result<VesselId> Simulation::add_landed_vessel(std::string name, bodies::BodyId body,
                                                     double latitude_rad, double longitude_rad,
                                                     vessel::VesselSystems systems) {
    const auto found = catalog_->body(body);
    if (!found) {
        return std::unexpected(found.error());
    }
    if (!std::isfinite(latitude_rad) || !std::isfinite(longitude_rad)) {
        return core::fail(ErrorCode::NotFinite, "a landing site needs a finite latitude and longitude");
    }
    if (systems.propulsion().epoch > now_) {
        return core::fail(ErrorCode::OutOfRange, "the vessel's systems are ahead of the simulation clock");
    }
    std::vector<vessel::Separation> separations;
    // Standing still means pointing up, not along an orbit the vessel is not on.
    for (const auto& [signal, value] :
         {std::pair{vessel::k_signal_pointing_frame, 2.0}, std::pair{vessel::k_signal_pointing_x, 0.0},
          std::pair{vessel::k_signal_pointing_y, 0.0}, std::pair{vessel::k_signal_pointing_z, 1.0}}) {
        if (core::VoidResult pointed =
                systems.command(signal, vessel::ControlSource::Sequencer, value, now_, separations);
            !pointed) {
            return std::unexpected(pointed.error());
        }
    }
    if (!separations.empty() || systems.propulsion().thrust_n != 0.0) {
        return core::fail(ErrorCode::InvalidArgument,
                          "a vessel cannot be put down while it is separating or under thrust");
    }
    const auto mass = systems.mass_properties(now_);
    if (!mass) {
        return std::unexpected(mass.error());
    }
    // Body-fixed axes: z along the pole, x through the prime meridian.
    const Vector3 up{std::cos(latitude_rad) * std::cos(longitude_rad),
                     std::cos(latitude_rad) * std::sin(longitude_rad), std::sin(latitude_rad)};
    const double height_m = standing_height_m(systems.assembly(), mass->centre_of_mass_m);
    const LandedPlace place{.position_m = (found->get().mean_radius_m + height_m) * up,
                            .orientation = orientation_with_nose(up, Vector3{0.0, 0.0, 1.0})};

    const LandedPose pose = landed_pose(found->get(), place, now_);
    const dynamics::VesselState state{.domain = body, .epoch = now_, .state_in_domain = pose.state};
    // Rationale: a landed vessel has no trajectory, but a Vessel always has a propagator; this
    // one is replaced when the vessel lifts off.
    auto propagator = dynamics::EnckePropagator::make(*gravity_, state, options_.propagator);
    if (!propagator) {
        return std::unexpected(propagator.error());
    }
    if (core::VoidResult reported = systems.report_telemetry(now_); !reported) {
        return std::unexpected(reported.error());
    }
    vessels_.push_back(Vessel{.name = std::move(name),
                              .status = VesselStatus::Landed,
                              .propagator = std::move(*propagator),
                              .state = state,
                              .systems = std::move(systems),
                              .attitude = pose.attitude,
                              .landed = place,
                              .events = {},
                              .events_epoch = now_,
                              .prediction = {},
                              .prediction_start = now_,
                              .prediction_horizon_s = 0.0,
                              .events_stale = false,
                              .prediction_stale = false});
    if (core::VoidResult reported = report_navigation(); !reported) {
        return std::unexpected(reported.error());
    }
    return VesselId{static_cast<std::uint32_t>(vessels_.size() - 1)};
}

// The state of a landed vessel at `instant`: where its place on the turning body is then.
void Simulation::place_landed(Vessel& vessel, const time::Epoch& instant) const {
    if (!vessel.landed.has_value()) {
        return;
    }
    const auto body = catalog_->body(vessel.state.domain);
    if (!body) {
        return;
    }
    const LandedPose pose = landed_pose(body->get(), *vessel.landed, instant);
    vessel.state.epoch = instant;
    vessel.state.state_in_domain = pose.state;
    vessel.attitude = pose.attitude;
}

core::VoidResult Simulation::advance_landed(std::size_t index, const time::Epoch& instant) {
    std::vector<vessel::Separation> separations;
    if (std::optional<vessel::VesselSystems>& systems = vessels_[index].systems; systems.has_value()) {
        if (core::VoidResult advanced = systems->advance_to(instant, separations); !advanced) {
            return advanced;
        }
    }
    for (vessel::Separation& separation : separations) {
        if (core::VoidResult separated = separate_landed(index, std::move(separation)); !separated) {
            return separated;
        }
    }
    Vessel& vessel = vessels_[index];
    place_landed(vessel, instant);
    // The ground holds the vessel up against gravity, less what its turning with the body
    // takes care of: that push is what is felt aboard.
    const Vector3& position_m = vessel.state.state_in_domain.position_m;
    const Vector3& spin_rad_s = vessel.attitude.angular_velocity_rad_s;
    if (const auto gravity = gravity_->total_acceleration_m_s2(vessel.state.domain, position_m, instant)) {
        vessel.proper_acceleration_m_s2 =
            math::cross(spin_rad_s, math::cross(spin_rad_s, position_m)) - *gravity;
    }
    if (vessel.systems.has_value()) {
        return vessel.systems->report_telemetry(instant);
    }
    return {};
}

// Parts that separate from a landed vessel stay where they are, as a landed vessel of their own.
core::VoidResult Simulation::separate_landed(std::size_t parent, vessel::Separation separation) {
    std::string name =
        std::format("{} ({})", vessels_[parent].name, separation.systems.assembly().parts().front().name);
    Vessel separated{.name = std::move(name),
                     .status = VesselStatus::Landed,
                     .propagator = vessels_[parent].propagator,
                     .state = vessels_[parent].state,
                     .systems = std::move(separation.systems),
                     .attitude = vessels_[parent].attitude,
                     .landed = vessels_[parent].landed,
                     .events = {},
                     .events_epoch = separation.epoch,
                     .prediction = {},
                     .prediction_start = separation.epoch,
                     .prediction_horizon_s = 0.0,
                     .events_stale = false,
                     .prediction_stale = false};
    if (separated.systems.has_value()) {
        if (core::VoidResult reported = separated.systems->report_telemetry(separation.epoch); !reported) {
            return reported;
        }
    }
    LOG_INFO("'{}' separated from '{}' on the ground", separated.name, vessels_[parent].name)
        .tag("subsystem", "sim");
    vessels_.push_back(std::move(separated));
    return {};
}

// A landed vessel whose engines push leaves its place on the ground and flies again.
core::VoidResult Simulation::lift_off(std::size_t index) {
    Vessel& vessel = vessels_[index];
    place_landed(vessel, now_);
    auto propagator = dynamics::EnckePropagator::make(*gravity_, vessel.state, options_.propagator);
    if (!propagator) {
        return std::unexpected(propagator.error());
    }
    vessel.propagator = std::move(*propagator);
    vessel.status = VesselStatus::Flying;
    vessel.landed.reset();
    vessel.events_stale = true;
    vessel.prediction_stale = true;
    return {};
}

bool Simulation::bubble_wanted(std::size_t index) const noexcept {
    if (!active_vessel_.has_value() || active_vessel_->index != index || index >= vessels_.size()
        || !vessels_[index].systems.has_value() || effective_warp_ > options_.max_physics_warp) {
        return false;
    }
    // A vessel standing on the ground costs nothing where it is; it is the anchor of a bubble
    // only while something flies near it, which it can then be touched by.
    return vessels_[index].status == VesselStatus::Flying
           || (vessels_[index].status == VesselStatus::Landed
               && flown_near(index,
                             bubble_ != nullptr ? k_bubble_leave_distance_m : k_bubble_join_distance_m));
}

bool Simulation::flown_near(std::size_t index, double distance_m) const noexcept {
    const Vessel& centre = vessels_[index];
    for (std::size_t other = 0; other < vessels_.size(); ++other) {
        const Vessel& vessel = vessels_[other];
        if (other != index && vessel.systems.has_value() && vessel.status == VesselStatus::Flying
            && vessel.state.domain == centre.state.domain
            && math::norm(vessel.state.state_in_domain.position_m - centre.state.state_in_domain.position_m)
                   < distance_m) {
            return true;
        }
    }
    return false;
}

bool Simulation::within_bubble(std::size_t index, double distance_m) const noexcept {
    const Vessel& vessel = vessels_[index];
    const Vessel& anchor = vessels_[bubble_->anchor];
    return vessel.systems.has_value()
           && (vessel.status == VesselStatus::Flying || vessel.status == VesselStatus::Landed)
           && vessel.state.domain == anchor.state.domain
           && math::norm(vessel.state.state_in_domain.position_m - anchor.state.state_in_domain.position_m)
                  < distance_m;
}

core::VoidResult Simulation::update_bubble_membership() {
    // Rationale: a landed vessel stays pinned to the ground until its engines push it; only
    // the bubble can take it from there, so thrust on a landed vessel at high warp does nothing.
    if (active_vessel_.has_value() && active_vessel_->index < vessels_.size()) {
        const std::size_t index = active_vessel_->index;
        if (const Vessel& vessel = vessels_[index];
            vessel.status == VesselStatus::Landed && vessel.systems.has_value()
            && vessel.systems->propulsion().thrust_n != 0.0 && effective_warp_ <= options_.max_physics_warp) {
            if (core::VoidResult lifted = lift_off(index); !lifted) {
                return lifted;
            }
            // Its new propagator starts now, and only moves forward: the bubble it was the
            // fixed anchor of skips the part of a tick it was behind.
            if (BubbleMember* member = bubble_ != nullptr ? bubble_->find(index) : nullptr;
                member != nullptr) {
                bubble_->epoch = now_;
                member->since = now_;
                member->ticks_at_rest = 0;
            }
        }
    }
    if (bubble_ != nullptr && !bubble_wanted(bubble_->anchor)) {
        if (core::VoidResult left = leave_bubble(); !left) {
            return left;
        }
    }
    if (bubble_ == nullptr) {
        if (!active_vessel_.has_value() || active_vessel_->index >= vessels_.size()) {
            return {};
        }
        const std::size_t index = active_vessel_->index;
        if (!bubble_wanted(index)) {
            return {};
        }
        if (core::VoidResult entered = enter_bubble(index); !entered) {
            return entered;
        }
    }
    // Who else is a body of it: the vessels near the anchor, flying or landed (BRIEFING D30).
    for (std::size_t index = 0; index < vessels_.size(); ++index) {
        if (index == bubble_->anchor) {
            continue;
        }
        if (const BubbleMember* member = bubble_->find(index); member != nullptr) {
            if (!within_bubble(index, k_bubble_leave_distance_m)) {
                if (core::VoidResult released =
                        release_from_bubble(index, std::max(bubble_->epoch, member->since));
                    !released) {
                    return released;
                }
            }
        } else if (within_bubble(index, k_bubble_join_distance_m)) {
            if (core::VoidResult joined = join_bubble(index, now_); !joined) {
                return joined;
            }
        }
    }
    return {};
}

core::VoidResult Simulation::enter_bubble(std::size_t index) {
    auto world = physics::World::make();
    if (!world) {
        return std::unexpected(world.error());
    }
    bubble_ = std::make_unique<Bubble>(
        Bubble{.world = std::move(*world), .anchor = index, .epoch = now_, .members = {}, .ground = {}});
    return join_bubble(index, now_);
}

core::VoidResult Simulation::join_bubble(std::size_t index, const time::Epoch& since) {
    Vessel& vessel = vessels_[index];
    // The bubble applies the thrust the vessel really has, tick by tick; what was planned for
    // the rails is withdrawn.
    std::vector<dynamics::ThrustChange> coast;
    if (vessel.propagator.is_thrusting()) {
        coast.push_back(dynamics::ThrustChange{.epoch = since});
    }
    if (core::VoidResult cleared = vessel.propagator.set_thrust_plan(std::move(coast)); !cleared) {
        return cleared;
    }
    bubble_->members.push_back(BubbleMember{.vessel = index, .since = since});
    LOG_DEBUG("'{}' enters the physics bubble", vessel.name).tag("subsystem", "sim");
    return {};
}

core::VoidResult Simulation::release_from_bubble(std::size_t index, const time::Epoch& epoch) {
    const auto member = std::ranges::find(bubble_->members, index, &BubbleMember::vessel);
    if (member == bubble_->members.end()) {
        return {};
    }
    const Vector3 shift_m = member->pending_shift_m;
    if (member->body.has_value()) {
        if (core::VoidResult removed = bubble_->world.remove(*member->body); !removed) {
            return removed;
        }
    }
    bubble_->members.erase(member);
    Vessel& vessel = vessels_[index];
    // On rails nothing turns the vessel, so the wheels stop too.
    if (core::VoidResult released = release_attitude_hold(vessel, epoch); !released) {
        return released;
    }
    vessel.events_stale = true;
    vessel.prediction_stale = true;
    if (vessel.status != VesselStatus::Flying) {
        return {};
    }
    if (shift_m != Vector3{}) {
        auto state = vessel.propagator.state_at(epoch);
        if (!state) {
            return std::unexpected(state.error());
        }
        state->state_in_domain.position_m += shift_m;
        if (core::VoidResult restarted = restart_propagator(vessel, *state); !restarted) {
            return restarted;
        }
    }
    LOG_DEBUG("'{}' leaves the physics bubble", vessel.name).tag("subsystem", "sim");
    return detail::sync_propulsion(vessel, epoch, true);
}

core::VoidResult Simulation::leave_bubble() {
    const time::Epoch epoch = bubble_->epoch;
    core::VoidResult result;
    while (!bubble_->members.empty()) {
        const BubbleMember& member = bubble_->members.back();
        if (core::VoidResult released = release_from_bubble(member.vessel, std::max(epoch, member.since));
            !released) {
            result = released;
            break;
        }
    }
    bubble_.reset();
    return result;
}

// Starts the vessel's propagator again from `state`. Manoeuvres still to come carry over.
core::VoidResult Simulation::restart_propagator(Vessel& vessel, const dynamics::VesselState& state) {
    auto restarted = dynamics::EnckePropagator::make(*gravity_, state, options_.propagator);
    if (!restarted) {
        return std::unexpected(restarted.error());
    }
    for (dynamics::Impulse impulse : vessel.propagator.pending_impulses()) {
        impulse.epoch = std::max(impulse.epoch, state.epoch);
        if (core::VoidResult scheduled = restarted->schedule_impulse(impulse); !scheduled) {
            return scheduled;
        }
    }
    vessel.propagator = std::move(*restarted);
    return {};
}

core::VoidResult Simulation::advance_in_bubble(const time::Epoch& instant) {
    while (bubble_ != nullptr) {
        const auto tick_end = bubble_->epoch.advanced_by(options_.physics_step_s);
        if (!tick_end) {
            return std::unexpected(tick_end.error());
        }
        if (*tick_end > instant) {
            break;
        }
        if (core::VoidResult ticked = bubble_tick(); !ticked) {
            return ticked;
        }
    }
    if (bubble_ == nullptr) {
        // The anchor landed or was destroyed on the way; whoever was with it is on rails again.
        return {};
    }
    // Rationale: by position, because parts that separate from a landed member add a vessel.
    for (std::size_t slot = 0; slot < bubble_->members.size(); ++slot) {
        BubbleMember& member = bubble_->members[slot];
        if (vessels_[member.vessel].status == VesselStatus::Landed) {
            if (core::VoidResult advanced = advance_landed(member.vessel, instant); !advanced) {
                return advanced;
            }
            continue;
        }
        Vessel& vessel = vessels_[member.vessel];
        // Between the latest tick and `instant` the vessel is shown coasting.
        dynamics::EnckePropagator ahead = vessel.propagator;
        if (const auto state = ahead.state_at(instant)) {
            vessel.state = *state;
            vessel.state.state_in_domain.position_m += member.pending_shift_m;
        }
        if (vessel.systems.has_value()) {
            if (core::VoidResult reported = vessel.systems->report_telemetry(instant); !reported) {
                return reported;
            }
        }
        const bool event_passed = !vessel.events.empty() && vessel.events.front().epoch <= instant;
        const bool events_aging =
            time::seconds_between(vessel.events_epoch, instant) > 0.25 * options_.event_horizon_s;
        const double prediction_age_s = time::seconds_between(vessel.prediction_start, instant);
        vessel.events_stale = vessel.events_stale || member.pushed || event_passed || events_aging;
        vessel.prediction_stale = vessel.prediction_stale
                                  || prediction_age_s > 0.25 * vessel.prediction_horizon_s
                                  || (member.pushed && prediction_age_s > k_pushed_prediction_period_s);
        if (vessel.prediction_stale) {
            member.pushed = false;
        }
        refresh_caches(vessel);
    }
    return {};
}

// Keeps the nose on the commanded pointing with whatever turns the vessel (reaction wheels),
// through the control bus like any other autopilot. It runs once per physics tick, a fixed
// physical period (BRIEFING §7.1 rule 5).
core::VoidResult Simulation::hold_attitude(std::size_t index, const dynamics::VesselState& state,
                                           const vessel::MassProperties& mass, bool touching,
                                           std::vector<vessel::Separation>& separations) {
    Vessel& vessel = vessels_[index];
    if (!vessel.systems.has_value()) {
        return {};
    }
    vessel::VesselSystems& systems = *vessel.systems;
    const auto target = dynamics::thrust_unit_vector(
        detail::to_thrust_direction(systems.propulsion().pointing), state.state_in_domain);
    // Rationale: on the ground the wheels would only fight the ground; and without a direction
    // to hold (a vessel at rest has no orbital frame) there is nothing to do.
    if (!systems.attitude_hold() || touching || !target) {
        return release_attitude_hold(vessel, state.epoch);
    }
    const Quaternion& orientation = vessel.attitude.orientation;
    const Quaternion to_vessel = math::conjugate(orientation);
    const Vector3 nose = math::rotate(orientation, {1.0, 0.0, 0.0});
    // The rotation that would bring the nose onto the target, as axis × angle.
    const Vector3 turn = math::cross(nose, *target);
    const double sine = math::norm(turn);
    const double angle_rad = std::atan2(sine, math::dot(nose, *target));
    const Vector3 axis = sine > 1e-12 ? turn / sine : math::rotate(orientation, {0.0, 1.0, 0.0});
    const Vector3 error_rad = math::rotate(to_vessel, angle_rad * axis);
    const Vector3 rate_rad_s = math::rotate(to_vessel, vessel.attitude.angular_velocity_rad_s);
    const Vector3 authority_n_m = systems.torque_authority_n_m();

    const std::array<double, 3> errors{error_rad.x, error_rad.y, error_rad.z};
    const std::array<double, 3> rates{rate_rad_s.x, rate_rad_s.y, rate_rad_s.z};
    const std::array<double, 3> authorities{authority_n_m.x, authority_n_m.y, authority_n_m.z};
    const std::array<double, 3> inertias{mass.inertia_kg_m2(0, 0), mass.inertia_kg_m2(1, 1),
                                         mass.inertia_kg_m2(2, 2)};
    for (std::size_t axis_index = 0; axis_index < k_attitude_signals.size(); ++axis_index) {
        const std::string_view signal = k_attitude_signals.at(axis_index);
        if (!systems.bus().find(signal).has_value()) {
            continue;
        }
        double command = 0.0;
        if (authorities.at(axis_index) > 0.0 && inertias.at(axis_index) > 0.0) {
            // Turn towards the target, never faster than the vessel can still stop from: the
            // rate from which the available torque brakes to rest over the remaining angle.
            const double acceleration_rad_s2 = authorities.at(axis_index) / inertias.at(axis_index);
            const double error = errors.at(axis_index);
            const double wanted_rate_rad_s =
                std::copysign(std::min(k_hold_rate_per_s * std::abs(error),
                                       std::sqrt(acceleration_rad_s2 * std::abs(error))),
                              error);
            command = std::clamp(k_hold_rate_gain_per_s * (wanted_rate_rad_s - rates.at(axis_index))
                                     / acceleration_rad_s2,
                                 -1.0, 1.0);
            command = std::round(command * k_hold_command_steps) / k_hold_command_steps;
        }
        if (core::VoidResult commanded =
                systems.command(signal, vessel::ControlSource::Autopilot, command,
                                std::max(state.epoch, systems.propulsion().epoch), separations);
            !commanded) {
            return commanded;
        }
    }
    return {};
}

core::VoidResult Simulation::bubble_tick() {
    const double step_s = options_.physics_step_s;
    const time::Epoch start = bubble_->epoch;
    const auto end = start.advanced_by(step_s);
    const auto middle = start.advanced_by(0.5 * step_s);
    if (!end || !middle) {
        return core::fail(ErrorCode::OutOfRange, "the physics tick is out of the range of epochs");
    }

    // Every member's systems up to the tick, then the attitude hold's commands for it.
    // Rationale: by position and looked up again each time, because a separation adds a vessel
    // and a member, and may move them all.
    for (std::size_t slot = 0; slot < bubble_->members.size(); ++slot) {
        const std::size_t index = bubble_->members[slot].vessel;
        // A landed member's systems are advanced with the frame, like any landed vessel's.
        if (bubble_->members[slot].since > start || vessels_[index].status != VesselStatus::Flying) {
            continue;
        }
        const auto state = vessels_[index].propagator.state_at(start);
        if (!state) {
            return std::unexpected(state.error());
        }
        std::vector<vessel::Separation> separations;
        const auto systems = systems_of(VesselId{static_cast<std::uint32_t>(index)});
        if (!systems) {
            return std::unexpected(systems.error());
        }
        if (core::VoidResult advanced = systems->get().advance_to(start, separations); !advanced) {
            return advanced;
        }
        const auto mass = systems->get().mass_properties(start);
        if (!mass) {
            return std::unexpected(mass.error());
        }
        if (core::VoidResult held =
                hold_attitude(index, *state, *mass, bubble_->members[slot].touching, separations);
            !held) {
            return held;
        }
        for (vessel::Separation& separation : separations) {
            // The parts leave at the tick, where the propagator stands, not a fraction before it.
            separation.epoch = start;
            if (core::VoidResult separated = separate(index, std::move(separation)); !separated) {
                return separated;
            }
        }
    }

    // The members that take part in this tick, each where its own propagator has it.
    struct Flown {
        std::size_t vessel = 0;
        dynamics::VesselState state;   // at the start of the tick
        Vector3 frame_velocity_m_s;    // in the frame of the tick
        Vector3 apparent_gravity_m_s2; // there
        physics::BodyId body;
        physics::BodyState before; // as the engine holds it, in single precision
        Attitude attitude;         // at the start of the tick
        bool fixed = false;        // landed: a body that stays where its place on the ground is
        bool loaded = false;       // by its own systems
        bool pushed_from_cabin = false;
    };
    std::vector<Flown> flown;
    for (BubbleMember& member : bubble_->members) {
        Vessel& vessel = vessels_[member.vessel];
        if (vessel.status == VesselStatus::Landed && vessel.landed.has_value()) {
            const auto body = catalog_->body(vessel.state.domain);
            if (!body) {
                return std::unexpected(body.error());
            }
            const LandedPose pose = landed_pose(body->get(), *vessel.landed, start);
            flown.push_back(
                Flown{.vessel = member.vessel,
                      .state = {.domain = vessel.state.domain, .epoch = start, .state_in_domain = pose.state},
                      .attitude = pose.attitude,
                      .fixed = true});
            continue;
        }
        if (member.since > start) {
            continue;
        }
        auto state = vessel.propagator.state_at(start);
        if (!state) {
            return std::unexpected(state.error());
        }
        if (member.pending_shift_m != Vector3{}) {
            state->state_in_domain.position_m += member.pending_shift_m;
            member.pending_shift_m = {};
            if (core::VoidResult restarted = restart_propagator(vessel, *state); !restarted) {
                return restarted;
            }
        }
        flown.push_back(Flown{.vessel = member.vessel, .state = *state, .attitude = vessel.attitude});
    }
    const auto anchored = std::ranges::find(flown, bubble_->anchor, &Flown::vessel);
    if (anchored == flown.end()) {
        return core::fail(ErrorCode::InvalidArgument, "the physics bubble has lost its anchor");
    }
    const dynamics::VesselState reference = anchored->state;
    // A member that is no longer about the same body as the anchor has no place in its frame.
    for (auto one = flown.begin(); one != flown.end();) {
        if (one->state.domain == reference.domain) {
            ++one;
            continue;
        }
        if (core::VoidResult released = release_from_bubble(one->vessel, start); !released) {
            return released;
        }
        one = flown.erase(one);
    }
    const auto domain = catalog_->body(reference.domain);
    if (!domain) {
        return std::unexpected(domain.error());
    }
    Bubble& bubble = *bubble_;
    physics::World& world = bubble.world;

    const Vector3& position_m = reference.state_in_domain.position_m;
    const Vector3& velocity_m_s = reference.state_in_domain.velocity_m_s;
    const double radius_m = math::norm(position_m);
    const double altitude_m = radius_m - domain->get().mean_radius_m;
    const bool near_ground = std::ranges::any_of(flown, [&](const Flown& one) {
        return math::norm(one.state.state_in_domain.position_m) - domain->get().mean_radius_m
               < k_contact_altitude_m;
    });
    const BodyFrame frame = near_ground ? body_frame(domain->get(), start) : BodyFrame{};
    const Vector3& spin_rad_s = frame.angular_velocity_rad_s;

    // The frame of this tick: its origin starts at the anchor's centre of mass. Away from the
    // ground it falls with the anchor; near the ground every velocity is taken relative to the
    // ground under the body it belongs to.
    Vector3 gravity_m_s2;
    if (near_ground) {
        const Vector3 up = position_m / radius_m;
        const auto gravity = gravity_->total_acceleration_m_s2(reference.domain, position_m, start);
        if (!gravity) {
            return std::unexpected(gravity.error());
        }
        gravity_m_s2 = *gravity;
        const Vector3 ground_m = -altitude_m * up;
        if (!bubble.ground.has_value()) {
            const auto ground = world.add_ground(ground_m, up, k_ground_friction);
            if (!ground) {
                return std::unexpected(ground.error());
            }
            bubble.ground = *ground;
        } else if (core::VoidResult moved = world.set_ground(*bubble.ground, ground_m, up); !moved) {
            return moved;
        }
    } else if (bubble.ground.has_value()) {
        if (core::VoidResult removed = world.remove(*bubble.ground); !removed) {
            return removed;
        }
        bubble.ground.reset();
    }
    for (Flown& one : flown) {
        const orbital::StateVector& state = one.state.state_in_domain;
        if (near_ground) {
            // The origin moves at Ω × r, with axes that do not turn: the apparent acceleration
            // is gravity less one Coriolis term and the centripetal acceleration of the origin.
            one.frame_velocity_m_s = state.velocity_m_s - math::cross(spin_rad_s, state.position_m);
            one.apparent_gravity_m_s2 = gravity_m_s2 - math::cross(spin_rad_s, one.frame_velocity_m_s)
                                        - math::cross(spin_rad_s, math::cross(spin_rad_s, state.position_m));
        } else {
            one.frame_velocity_m_s = state.velocity_m_s - velocity_m_s;
        }
    }
    // Rationale: the engine has one gravity for all, the anchor's. Its difference from a
    // member's own apparent gravity (the turning of the ground; the tide is left out, 1e-3 of
    // the weight at 500 m) is given to that member as a force.
    const Vector3 anchor_gravity_m_s2 =
        std::ranges::find(flown, bubble.anchor, &Flown::vessel)->apparent_gravity_m_s2;
    world.set_gravity(anchor_gravity_m_s2);

    for (Flown& one : flown) {
        BubbleMember& member = *bubble.find(one.vessel);
        const auto systems = systems_of(VesselId{static_cast<std::uint32_t>(one.vessel)});
        if (!systems) {
            return std::unexpected(systems.error());
        }
        const auto mass = systems->get().mass_properties(start);
        if (!mass) {
            return std::unexpected(mass.error());
        }
        const vessel::Assembly& assembly = systems->get().assembly();

        // The rigid body, rebuilt when the vessel changed shape, its centre of mass moved, or
        // it came to stand on the ground or left it.
        if (member.body.has_value()
            && (member.body_part_count != assembly.parts().size() || member.body_fixed != one.fixed
                || math::norm(mass->centre_of_mass_m - member.body_centre_of_mass_m)
                       > k_body_rebuild_shift_m)) {
            if (core::VoidResult removed = world.remove(*member.body); !removed) {
                return removed;
            }
            member.body.reset();
        }
        if (!member.body.has_value()) {
            const auto body = world.add_body(
                physics::BodyDescription{.pieces = shape_pieces(assembly, mass->centre_of_mass_m),
                                         .mass_kg = mass->mass_kg,
                                         .inertia_kg_m2 = mass->inertia_kg_m2,
                                         .state = {},
                                         .friction = k_ground_friction,
                                         .fixed = one.fixed});
            if (!body) {
                return std::unexpected(body.error());
            }
            member.body = *body;
            member.body_centre_of_mass_m = mass->centre_of_mass_m;
            member.body_part_count = assembly.parts().size();
            member.body_fixed = one.fixed;
            member.impact_tolerance_m_s = least_impact_tolerance_m_s(assembly);
        } else if (!one.fixed) {
            if (core::VoidResult weighed = world.set_mass(*member.body, mass->mass_kg, mass->inertia_kg_m2);
                !weighed) {
                return weighed;
            }
        }
        one.body = *member.body;

        const Quaternion& orientation = one.attitude.orientation;
        if (core::VoidResult placed = world.set_state(
                one.body, {.position_m = one.state.state_in_domain.position_m - position_m,
                           .orientation = orientation,
                           .velocity_m_s = one.frame_velocity_m_s,
                           .angular_velocity_rad_s = one.attitude.angular_velocity_rad_s - spin_rad_s});
            !placed) {
            return placed;
        }
        // Rationale: what the tick did to the body is the difference from what the engine
        // holds now, not from the doubles it was given, so a body nothing touches gains nothing.
        const auto before = world.state(one.body);
        if (!before) {
            return std::unexpected(before.error());
        }
        one.before = *before;
        if (one.fixed) {
            continue; // nothing moves it
        }
        const Vector3& centre_m = one.before.position_m;

        const std::vector<vessel::AppliedLoad> loads = systems->get().loads();
        for (const vessel::AppliedLoad& load : loads) {
            if (load.force_n != Vector3{}) {
                if (core::VoidResult applied = world.add_force(
                        one.body, math::rotate(orientation, load.force_n),
                        centre_m + math::rotate(orientation, load.position_m - mass->centre_of_mass_m));
                    !applied) {
                    return applied;
                }
            }
            if (load.torque_n_m != Vector3{}) {
                if (core::VoidResult applied =
                        world.add_torque(one.body, math::rotate(orientation, load.torque_n_m));
                    !applied) {
                    return applied;
                }
            }
        }
        if (const Vector3 difference_m_s2 = one.apparent_gravity_m_s2 - anchor_gravity_m_s2;
            difference_m_s2 != Vector3{}) {
            if (core::VoidResult applied =
                    world.add_force(one.body, mass->mass_kg * difference_m_s2, centre_m);
                !applied) {
                return applied;
            }
        }
        // Whatever moves about in the cabin (the pilot, loose items) pushes the vessel the
        // other way: a force through the centre of mass, and the torque about it.
        const bool pilot_pushes =
            cabin_reaction_.has_value() && pilot_.has_value() && pilot_->vessel.index == one.vessel
            && (cabin_reaction_->force_n != Vector3{} || cabin_reaction_->torque_n_m != Vector3{});
        if (pilot_pushes) {
            const Vector3 about_centre_n_m =
                cabin_reaction_->torque_n_m - math::cross(mass->centre_of_mass_m, cabin_reaction_->force_n);
            if (const auto error = core::first_error(
                    world.add_force(one.body, math::rotate(orientation, cabin_reaction_->force_n), centre_m),
                    world.add_torque(one.body, math::rotate(orientation, about_centre_n_m)))) {
                return std::unexpected(*error);
            }
        }
        one.loaded = !loads.empty();
        one.pushed_from_cabin = pilot_pushes;
    }

    if (core::VoidResult stepped = world.step(step_s); !stepped) {
        return stepped;
    }
    bubble.epoch = *end;

    // Vessels that end the tick destroyed are bodies no longer.
    std::vector<std::size_t> gone;
    const auto destroy = [&](Vessel& vessel, const dynamics::VesselState& state, bool by_the_ground) {
        vessel.state = state;
        vessel.status = VesselStatus::Crashed;
        vessel.events.clear();
        vessel.prediction.clear();
        if (by_the_ground) {
            LOG_INFO("vessel '{}' hit the surface of {} too hard", vessel.name, domain->get().name)
                .tag("subsystem", "sim");
        } else {
            LOG_INFO("vessel '{}' was hit too hard", vessel.name).tag("subsystem", "sim");
        }
    };
    for (const Flown& one : flown) {
        BubbleMember& member = *bubble.find(one.vessel);
        Vessel& vessel = vessels_[one.vessel];
        const auto after = world.state(one.body);
        if (!after) {
            return std::unexpected(after.error());
        }
        member.touching = world.touching(one.body);
        member.pushed = member.pushed || member.touching || one.loaded || one.pushed_from_cabin;
        const Vector3 gained_m_s = after->velocity_m_s - one.before.velocity_m_s;
        const Vector3 moved_m = after->position_m - one.before.position_m;
        std::optional<bool> hit_too_hard; // whether by the ground
        for (const physics::Contact& contact : world.contacts_begun()) {
            if ((contact.first == one.body || contact.second == one.body)
                && contact.closing_speed_m_s > member.impact_tolerance_m_s) {
                hit_too_hard = bubble.ground.has_value()
                               && (contact.first == *bubble.ground || contact.second == *bubble.ground);
            }
        }
        if (one.fixed) {
            if (hit_too_hard.has_value()) {
                destroy(vessel, one.state, *hit_too_hard);
                gone.push_back(one.vessel);
            }
            continue;
        }

        if (!near_ground) {
            // Only what is not gravity changed the velocity in the falling frame; the propagator
            // takes it as an impulse and integrates gravity as it always does. Thrust acts all
            // through the tick, so its impulse is in the middle; a contact is resolved by the
            // engine as a jump at the start, and is placed there for the two to agree on where
            // the bodies end up.
            if (gained_m_s != Vector3{}) {
                if (core::VoidResult pushed = vessel.propagator.schedule_impulse(
                        {.epoch = member.touching ? start : *middle, .delta_v_m_s = gained_m_s});
                    !pushed) {
                    return pushed;
                }
            }
            auto moved = vessel.propagator.state_at(*end);
            if (!moved) {
                return std::unexpected(moved.error());
            }
            // What the engine moved a touching body by beyond its velocity is its pushing
            // bodies out of each other, which the propagator has to be told.
            const Vector3 corrected_m = member.touching ? moved_m - step_s * after->velocity_m_s : Vector3{};
            if (math::norm(corrected_m) > k_contact_correction_m) {
                moved->state_in_domain.position_m += corrected_m;
                if (core::VoidResult restarted = restart_propagator(vessel, *moved); !restarted) {
                    return restarted;
                }
            }
            vessel.state = *moved;
            vessel.attitude = {.orientation = after->orientation,
                               .angular_velocity_rad_s = after->angular_velocity_rad_s};
            vessel.proper_acceleration_m_s2 = gained_m_s / step_s;
            member.ticks_at_rest = 0;
            if (hit_too_hard.has_value()) {
                destroy(vessel, *moved, false);
                gone.push_back(one.vessel);
            }
            continue;
        }
        // What the engines and what it touches did to the vessel during the tick, gravity apart.
        vessel.proper_acceleration_m_s2 = gained_m_s / step_s - one.apparent_gravity_m_s2;

        // Near the ground the engine's result is the vessel's state: back to the universe's
        // frame, and the propagator starts again from it.
        const Vector3& start_position_m = one.state.state_in_domain.position_m;
        const Vector3 end_position_m =
            start_position_m + step_s * math::cross(spin_rad_s, start_position_m) + moved_m;
        const dynamics::VesselState end_state{
            .domain = one.state.domain,
            .epoch = *end,
            .state_in_domain = {.position_m = end_position_m,
                                .velocity_m_s = one.frame_velocity_m_s + gained_m_s
                                                + math::cross(spin_rad_s, end_position_m)}};
        vessel.attitude = {.orientation = math::normalized(math::from_rotation_vector(step_s * spin_rad_s)
                                                           * after->orientation),
                           .angular_velocity_rad_s = after->angular_velocity_rad_s + spin_rad_s};

        if (hit_too_hard.has_value()) {
            destroy(vessel, end_state, *hit_too_hard);
            gone.push_back(one.vessel);
            continue;
        }
        const Vector3 frame_velocity_m_s = one.frame_velocity_m_s + gained_m_s;
        const bool at_rest = member.touching && !one.loaded
                             && math::norm(frame_velocity_m_s) < k_rest_speed_m_s
                             && math::norm(after->angular_velocity_rad_s) < k_rest_spin_rad_s;
        member.ticks_at_rest = at_rest ? member.ticks_at_rest + 1 : 0;
        if (member.ticks_at_rest >= k_rest_ticks) {
            const BodyFrame end_frame = body_frame(domain->get(), *end);
            vessel.state = end_state;
            vessel.status = VesselStatus::Landed;
            vessel.landed = LandedPlace{.position_m = end_frame.to_fixed * end_position_m,
                                        .orientation = math::normalized(math::from_matrix(end_frame.to_fixed)
                                                                        * vessel.attitude.orientation)};
            vessel.events.clear();
            vessel.prediction.clear();
            LOG_INFO("vessel '{}' landed on {}", vessel.name, domain->get().name).tag("subsystem", "sim");
            // It stays a body, a fixed one from the next tick on, for as long as others fly here.
            member.ticks_at_rest = 0;
            if (core::VoidResult released = release_attitude_hold(vessel, *end); !released) {
                return released;
            }
            continue;
        }
        vessel.state = end_state;
        if (core::VoidResult restarted = restart_propagator(vessel, end_state); !restarted) {
            return restarted;
        }
    }

    for (const std::size_t index : gone) {
        const auto member = std::ranges::find(bubble.members, index, &BubbleMember::vessel);
        if (member->body.has_value()) {
            if (core::VoidResult removed = world.remove(*member->body); !removed) {
                return removed;
            }
        }
        bubble.members.erase(member);
    }
    // Rationale: the frame of the bubble goes with its anchor. Without it the other members go
    // back on rails; a new bubble forms about the active vessel when it flies again.
    // Nor is there anything to simulate once nothing in it flies: every vessel left is pinned to
    // its place on the ground, which costs nothing.
    const bool flying = std::ranges::any_of(bubble.members, [&](const BubbleMember& member) {
        return vessels_[member.vessel].status == VesselStatus::Flying;
    });
    if (std::ranges::find(gone, bubble.anchor) != gone.end() || !flying) {
        return leave_bubble();
    }
    return {};
}

} // namespace helios::sim
