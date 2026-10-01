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
    return bubble_ != nullptr && bubble_->occupant == id.index;
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
    return active_vessel_.has_value() && active_vessel_->index == index && index < vessels_.size()
           && vessels_[index].systems.has_value() && vessels_[index].status == VesselStatus::Flying
           && effective_warp_ <= options_.max_physics_warp;
}

core::VoidResult Simulation::update_bubble_membership() {
    if (bubble_ != nullptr && !bubble_wanted(bubble_->occupant)) {
        if (core::VoidResult left = leave_bubble(); !left) {
            return left;
        }
    }
    if (bubble_ != nullptr || !active_vessel_.has_value() || active_vessel_->index >= vessels_.size()) {
        return {};
    }
    const std::size_t index = active_vessel_->index;
    // Rationale: a landed vessel stays pinned to the ground until its engines push it; only
    // the bubble can take it from there, so thrust on a landed vessel at high warp does nothing.
    if (const Vessel& vessel = vessels_[index];
        vessel.status == VesselStatus::Landed && vessel.systems.has_value()
        && vessel.systems->propulsion().thrust_n != 0.0 && effective_warp_ <= options_.max_physics_warp) {
        if (core::VoidResult lifted = lift_off(index); !lifted) {
            return lifted;
        }
    }
    return bubble_wanted(index) ? enter_bubble(index) : core::VoidResult{};
}

core::VoidResult Simulation::enter_bubble(std::size_t index) {
    auto world = physics::World::make();
    if (!world) {
        return std::unexpected(world.error());
    }
    Vessel& vessel = vessels_[index];
    // The bubble applies the thrust the vessel really has, tick by tick; what was planned for
    // the rails is withdrawn.
    std::vector<dynamics::ThrustChange> coast;
    if (vessel.propagator.is_thrusting()) {
        coast.push_back(dynamics::ThrustChange{.epoch = now_});
    }
    if (core::VoidResult cleared = vessel.propagator.set_thrust_plan(std::move(coast)); !cleared) {
        return cleared;
    }
    bubble_ = std::make_unique<Bubble>(Bubble{.world = std::move(*world), .occupant = index, .epoch = now_});
    LOG_DEBUG("'{}' enters the physics bubble", vessel.name).tag("subsystem", "sim");
    return {};
}

core::VoidResult Simulation::leave_bubble() {
    const time::Epoch epoch = bubble_->epoch;
    Vessel& vessel = vessels_[bubble_->occupant];
    bubble_.reset();
    // On rails nothing turns the vessel, so the wheels stop too.
    if (core::VoidResult released = release_attitude_hold(vessel, epoch); !released) {
        return released;
    }
    vessel.events_stale = true;
    vessel.prediction_stale = true;
    if (vessel.status != VesselStatus::Flying) {
        return {};
    }
    LOG_DEBUG("'{}' leaves the physics bubble", vessel.name).tag("subsystem", "sim");
    return detail::sync_propulsion(vessel, epoch, true);
}

core::VoidResult Simulation::advance_in_bubble(const time::Epoch& instant) {
    const std::size_t index = bubble_->occupant;
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
    Vessel& vessel = vessels_[index];
    if (bubble_ == nullptr) {
        // The vessel landed or was destroyed on the way; a landed one still has to reach `instant`.
        return vessel.status == VesselStatus::Landed ? advance_landed(index, instant) : core::VoidResult{};
    }
    // Between the latest tick and `instant` the vessel is shown coasting.
    dynamics::EnckePropagator ahead = vessel.propagator;
    if (const auto state = ahead.state_at(instant)) {
        vessel.state = *state;
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
    vessel.events_stale = vessel.events_stale || bubble_->pushed || event_passed || events_aging;
    vessel.prediction_stale = vessel.prediction_stale || prediction_age_s > 0.25 * vessel.prediction_horizon_s
                              || (bubble_->pushed && prediction_age_s > k_pushed_prediction_period_s);
    if (vessel.prediction_stale) {
        bubble_->pushed = false;
    }
    refresh_caches(vessel);
    return {};
}

// Keeps the nose on the commanded pointing with whatever turns the vessel (reaction wheels),
// through the control bus like any other autopilot. It runs once per physics tick, a fixed
// physical period (BRIEFING §7.1 rule 5).
core::VoidResult Simulation::hold_attitude(std::size_t index, const dynamics::VesselState& state,
                                           const vessel::MassProperties& mass,
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
    if (!systems.attitude_hold() || bubble_->touching || !target) {
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
    const std::size_t index = bubble_->occupant;
    const double step_s = options_.physics_step_s;
    const time::Epoch start = bubble_->epoch;
    const auto end = start.advanced_by(step_s);
    const auto middle = start.advanced_by(0.5 * step_s);
    if (!end || !middle) {
        return core::fail(ErrorCode::OutOfRange, "the physics tick is out of the range of epochs");
    }
    const auto state = vessels_[index].propagator.state_at(start);
    if (!state) {
        return std::unexpected(state.error());
    }
    const auto domain = catalog_->body(state->domain);
    if (!domain) {
        return std::unexpected(domain.error());
    }

    // The vessel's systems up to the tick, then the attitude hold's commands for it.
    std::vector<vessel::Separation> separations;
    {
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
        if (core::VoidResult held = hold_attitude(index, *state, *mass, separations); !held) {
            return held;
        }
    }
    for (vessel::Separation& separation : separations) {
        // The parts leave at the tick, where the propagator stands, not a fraction before it.
        separation.epoch = start;
        if (core::VoidResult separated = separate(index, std::move(separation)); !separated) {
            return separated;
        }
    }

    // Rationale: looked up again, because a separation adds a vessel and may move them all.
    Vessel& vessel = vessels_[index];
    const auto systems = systems_of(VesselId{static_cast<std::uint32_t>(index)});
    if (!systems) {
        return std::unexpected(systems.error());
    }
    const auto mass = systems->get().mass_properties(start);
    if (!mass) {
        return std::unexpected(mass.error());
    }
    const vessel::Assembly& assembly = systems->get().assembly();
    Bubble& bubble = *bubble_;
    physics::World& world = bubble.world;

    // The rigid body, rebuilt when the vessel changed shape or its centre of mass moved.
    if (bubble.body.has_value()
        && (bubble.body_part_count != assembly.parts().size()
            || math::norm(mass->centre_of_mass_m - bubble.body_centre_of_mass_m) > k_body_rebuild_shift_m)) {
        if (core::VoidResult removed = world.remove(*bubble.body); !removed) {
            return removed;
        }
        bubble.body.reset();
    }
    if (!bubble.body.has_value()) {
        const auto body =
            world.add_body(physics::BodyDescription{.pieces = shape_pieces(assembly, mass->centre_of_mass_m),
                                                    .mass_kg = mass->mass_kg,
                                                    .inertia_kg_m2 = mass->inertia_kg_m2,
                                                    .state = {},
                                                    .friction = k_ground_friction});
        if (!body) {
            return std::unexpected(body.error());
        }
        bubble.body = *body;
        bubble.body_centre_of_mass_m = mass->centre_of_mass_m;
        bubble.body_part_count = assembly.parts().size();
        bubble.impact_tolerance_m_s = least_impact_tolerance_m_s(assembly);
    } else if (core::VoidResult weighed = world.set_mass(*bubble.body, mass->mass_kg, mass->inertia_kg_m2);
               !weighed) {
        return weighed;
    }
    const physics::BodyId body = *bubble.body;

    const Vector3& position_m = state->state_in_domain.position_m;
    const Vector3& velocity_m_s = state->state_in_domain.velocity_m_s;
    const double radius_m = math::norm(position_m);
    const double altitude_m = radius_m - domain->get().mean_radius_m;
    const bool near_ground = altitude_m < k_contact_altitude_m;
    const BodyFrame frame = near_ground ? body_frame(domain->get(), start) : BodyFrame{};
    const Vector3& spin_rad_s = frame.angular_velocity_rad_s;

    // The frame of this tick: its origin starts at the centre of mass and moves with the
    // ground under the vessel (near the ground) or falls with the vessel (away from it).
    Vector3 start_velocity_m_s; // of the vessel in the frame
    Vector3 apparent_gravity_m_s2;
    if (near_ground) {
        const Vector3 up = position_m / radius_m;
        const auto gravity = gravity_->total_acceleration_m_s2(state->domain, position_m, start);
        if (!gravity) {
            return std::unexpected(gravity.error());
        }
        // The origin moves at Ω × r, with axes that do not turn: the apparent acceleration is
        // gravity less one Coriolis term and the centripetal acceleration of the origin.
        start_velocity_m_s = velocity_m_s - math::cross(spin_rad_s, position_m);
        apparent_gravity_m_s2 = *gravity - math::cross(spin_rad_s, start_velocity_m_s)
                                - math::cross(spin_rad_s, math::cross(spin_rad_s, position_m));
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
    world.set_gravity(apparent_gravity_m_s2);
    const Quaternion& orientation = vessel.attitude.orientation;
    if (core::VoidResult placed = world.set_state(
            body, {.position_m = {},
                   .orientation = orientation,
                   .velocity_m_s = start_velocity_m_s,
                   .angular_velocity_rad_s = vessel.attitude.angular_velocity_rad_s - spin_rad_s});
        !placed) {
        return placed;
    }
    const std::vector<vessel::AppliedLoad> loads = systems->get().loads();
    for (const vessel::AppliedLoad& load : loads) {
        if (load.force_n != Vector3{}) {
            if (core::VoidResult applied =
                    world.add_force(body, math::rotate(orientation, load.force_n),
                                    math::rotate(orientation, load.position_m - mass->centre_of_mass_m));
                !applied) {
                return applied;
            }
        }
        if (load.torque_n_m != Vector3{}) {
            if (core::VoidResult applied = world.add_torque(body, math::rotate(orientation, load.torque_n_m));
                !applied) {
                return applied;
            }
        }
    }
    if (core::VoidResult stepped = world.step(step_s); !stepped) {
        return stepped;
    }
    const auto after = world.state(body);
    if (!after) {
        return std::unexpected(after.error());
    }
    bubble.touching = world.touching(body);
    bubble.pushed = bubble.pushed || bubble.touching || !loads.empty();
    bubble.epoch = *end;

    if (!near_ground) {
        // Only the thrust changed the velocity in the falling frame; the propagator takes it as
        // an impulse in the middle of the tick and integrates gravity as it always does.
        if (after->velocity_m_s != Vector3{}) {
            if (core::VoidResult pushed = vessel.propagator.schedule_impulse(
                    {.epoch = *middle, .delta_v_m_s = after->velocity_m_s});
                !pushed) {
                return pushed;
            }
        }
        if (const auto moved = vessel.propagator.state_at(*end); !moved) {
            return std::unexpected(moved.error());
        }
        vessel.attitude = {.orientation = after->orientation,
                           .angular_velocity_rad_s = after->angular_velocity_rad_s};
        bubble.ticks_at_rest = 0;
        return {};
    }

    // Near the ground the engine's result is the vessel's state: back to the universe's frame,
    // and the propagator starts again from it.
    const Vector3 end_position_m =
        position_m + step_s * math::cross(spin_rad_s, position_m) + after->position_m;
    const dynamics::VesselState end_state{
        .domain = state->domain,
        .epoch = *end,
        .state_in_domain = {.position_m = end_position_m,
                            .velocity_m_s = after->velocity_m_s + math::cross(spin_rad_s, end_position_m)}};
    vessel.attitude = {
        .orientation = math::normalized(math::from_rotation_vector(step_s * spin_rad_s) * after->orientation),
        .angular_velocity_rad_s = after->angular_velocity_rad_s + spin_rad_s};

    const bool destroyed = std::ranges::any_of(world.contacts_begun(), [&](const physics::Contact& contact) {
        return (contact.first == body || contact.second == body)
               && contact.closing_speed_m_s > bubble.impact_tolerance_m_s;
    });
    if (destroyed) {
        vessel.state = end_state;
        vessel.status = VesselStatus::Crashed;
        vessel.events.clear();
        vessel.prediction.clear();
        LOG_INFO("vessel '{}' hit the surface of {} too hard", vessel.name, domain->get().name)
            .tag("subsystem", "sim");
        bubble_.reset();
        return {};
    }

    const bool at_rest = bubble.touching && loads.empty()
                         && math::norm(after->velocity_m_s) < k_rest_speed_m_s
                         && math::norm(after->angular_velocity_rad_s) < k_rest_spin_rad_s;
    bubble.ticks_at_rest = at_rest ? bubble.ticks_at_rest + 1 : 0;
    if (bubble.ticks_at_rest >= k_rest_ticks) {
        const BodyFrame end_frame = body_frame(domain->get(), *end);
        vessel.state = end_state;
        vessel.status = VesselStatus::Landed;
        vessel.landed = LandedPlace{.position_m = end_frame.to_fixed * end_position_m,
                                    .orientation = math::normalized(math::from_matrix(end_frame.to_fixed)
                                                                    * vessel.attitude.orientation)};
        vessel.events.clear();
        vessel.prediction.clear();
        LOG_INFO("vessel '{}' landed on {}", vessel.name, domain->get().name).tag("subsystem", "sim");
        bubble_.reset();
        return release_attitude_hold(vessel, *end);
    }

    // Manoeuvres still to come carry over to the new propagator.
    auto restarted = dynamics::EnckePropagator::make(*gravity_, end_state, options_.propagator);
    if (!restarted) {
        return std::unexpected(restarted.error());
    }
    for (dynamics::Impulse impulse : vessel.propagator.pending_impulses()) {
        impulse.epoch = std::max(impulse.epoch, *end);
        if (core::VoidResult scheduled = restarted->schedule_impulse(impulse); !scheduled) {
            return scheduled;
        }
    }
    vessel.propagator = std::move(*restarted);
    return {};
}

} // namespace helios::sim
