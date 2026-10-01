// The pilot in the cabin (BRIEFING §12, D27): the part of Simulation that moves the player's
// body about the inside of a vessel.
//
// The cabin is a small physics world of its own, in the vessel's axes and at rest with the
// vessel: its walls do not move, and the one body in it is the pilot. What the vessel does is
// felt as the forces of its frame. Its proper acceleration (thrust, or the ground holding it
// up) is the cabin's "gravity", so the pilot floats in free fall, stands on the Moon, and is
// pressed to the aft wall under thrust, all by the same rule; a turning vessel adds the
// centrifugal, Coriolis and Euler terms.
//
// The other way round: every push the cabin gives the pilot (a floor under the feet, a wall
// kicked off from, a handhold) the pilot gives back to the vessel, at the place where the pilot
// is. While the vessel is in the physics bubble it feels that as a force like any other, so a
// pilot who kicks off moves the vessel a little the other way and one who runs round a
// spinning cabin makes it wobble. The cabin exists only while the pilot is out of the seat;
// otherwise the vessel is the one rigid body it is to everything else (BRIEFING D27).
//
// Nothing here moves the pilot except contact: legs when there is weight to stand against, and
// a hand that holds on to a surface. A body that floats free keeps its velocity until it
// touches something.

#include "helios/core/logging.hpp"
#include "helios/sim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

#include "bubble.hpp"

namespace helios::sim {

namespace {

using core::ErrorCode;
using math::Matrix3;
using math::Quaternion;
using math::Vector3;

// The body: a ball about the chest, with the eyes a little above its centre.
constexpr double k_body_radius_m = 0.3;
constexpr double k_body_mass_kg = 80.0;
constexpr double k_body_friction = 0.5;
constexpr double k_wall_friction = 0.8;
// Legs: they hold the body's centre this far above what is under it, as a damped spring, when
// the weight felt is at least the threshold.
constexpr double k_leg_length_m = 1.3;
constexpr double k_leg_slack_m = 0.2;
constexpr double k_leg_stiffness_per_s2 = 150.0;
constexpr double k_leg_damping_per_s = 24.0;
constexpr double k_weight_threshold_m_s2 = 0.5;
constexpr double k_walk_speed_m_s = 1.5;
constexpr double k_walk_gain_per_s = 8.0;
constexpr double k_jump_speed_m_s = 2.5;
// The hand: it takes hold of a surface within reach of the eyes, and the arm then moves the
// body relative to it, no further than its length.
constexpr double k_reach_m = 0.9;
constexpr double k_arm_length_m = 1.1;
constexpr double k_arm_least_m = 0.3;
constexpr double k_grip_clearance_m = 0.05; // between the body and the surface the hand holds
constexpr double k_arm_speed_m_s = 1.2;
constexpr double k_grip_stiffness_per_s2 = 120.0;
constexpr double k_grip_damping_per_s = 22.0;
constexpr double k_grip_strength_m_s2 = 10.0;
constexpr double k_seat_reach_m = 1.0;
// A pilot this far from the seat has left the cabin through a gap in its walls.
constexpr double k_lost_distance_m = 100.0;

struct Seat {
    Vector3 eye_m;          // vessel axes, from the vessel's origin
    Matrix3 seat_to_vessel; // columns: forward, left, up
    bool walkable = false;
};

// The seat of the first crewed part.
[[nodiscard]] std::optional<Seat> seat_of(const vessel::Assembly& assembly) {
    const auto parts = assembly.parts();
    const auto poses = assembly.poses();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cockpit>& cockpit = parts[index].datasheet->cockpit;
        if (cockpit.has_value()) {
            return Seat{.eye_m = poses[index].position_m + poses[index].orientation * cockpit->eye_m,
                        .seat_to_vessel = poses[index].orientation * cockpit->seat_to_part(),
                        .walkable = cockpit->walkable};
        }
    }
    return std::nullopt;
}

[[nodiscard]] Vector3 column(const Matrix3& matrix, std::size_t index) noexcept {
    return {matrix(0, index), matrix(1, index), matrix(2, index)};
}

// Where the body's centre is when the pilot sits.
[[nodiscard]] Vector3 seated_position_m(const Seat& seat) noexcept {
    return seat.eye_m - k_pilot_eye_height_m * column(seat.seat_to_vessel, 2);
}

[[nodiscard]] Vector3 limited(const Vector3& vector, double greatest) noexcept {
    const double length = math::norm(vector);
    return length > greatest ? vector * (greatest / length) : vector;
}

} // namespace

core::VoidResult Simulation::board(VesselId id) {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    const Vessel& vessel = vessels_[id.index];
    if (vessel.status == VesselStatus::Crashed || !vessel.systems.has_value()
        || !seat_of(vessel.systems->assembly()).has_value()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("'{}' has no seat to take", vessel.name));
    }
    pilot_ = Pilot{.vessel = id};
    seat_pilot();
    return {};
}

void Simulation::seat_pilot() {
    cabin_.reset();
    pilot_reaction_.reset();
    if (!pilot_.has_value() || pilot_->vessel.index >= vessels_.size()) {
        return;
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    const auto seat = vessel.systems.has_value() ? seat_of(vessel.systems->assembly()) : std::nullopt;
    if (!seat.has_value()) {
        pilot_.reset(); // the part with the seat is gone
        return;
    }
    *pilot_ = Pilot{.vessel = pilot_->vessel,
                    .seated = true,
                    .position_m = seated_position_m(*seat),
                    .velocity_m_s = {},
                    .view = math::from_matrix(seat->seat_to_vessel)};
}

core::VoidResult Simulation::leave_seat() {
    if (!pilot_.has_value() || !pilot_->seated) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not in a seat");
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    const auto seat = vessel.systems.has_value() ? seat_of(vessel.systems->assembly()) : std::nullopt;
    if (vessel.status == VesselStatus::Crashed || !seat.has_value() || !seat->walkable) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("'{}' has no cabin to move about in", vessel.name));
    }
    pilot_->seated = false;
    return build_cabin();
}

core::VoidResult Simulation::take_seat() {
    if (!pilot_.has_value() || pilot_->seated) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not out of the seat");
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    const auto seat = vessel.systems.has_value() ? seat_of(vessel.systems->assembly()) : std::nullopt;
    if (!seat.has_value() || math::norm(pilot_->position_m - seated_position_m(*seat)) > k_seat_reach_m) {
        return core::fail(ErrorCode::InvalidArgument, "the seat is out of reach");
    }
    seat_pilot();
    return {};
}

// The cabin's walls, from the cockpits of the vessel's parts, and the pilot's body where the
// pilot is.
core::VoidResult Simulation::build_cabin() {
    if (!pilot_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is no pilot");
    }
    const std::optional<vessel::VesselSystems>& systems = vessels_[pilot_->vessel.index].systems;
    if (!systems.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot's vessel has no cabin");
    }
    auto world = physics::World::make();
    if (!world) {
        return std::unexpected(world.error());
    }
    const vessel::Assembly& assembly = systems->assembly();
    const auto parts = assembly.parts();
    const auto poses = assembly.poses();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cockpit>& cockpit = parts[index].datasheet->cockpit;
        if (!cockpit.has_value()) {
            continue;
        }
        const Matrix3 seat_to_vessel = poses[index].orientation * cockpit->seat_to_part();
        const Vector3 eye_m = poses[index].position_m + poses[index].orientation * cockpit->eye_m;
        for (const vessel::CockpitBox& box : cockpit->boxes) {
            if (const auto wall = world->add_static_box({.centre_m = eye_m + seat_to_vessel * box.centre_m,
                                                         .half_extents_m = 0.5 * box.size_m,
                                                         .orientation = math::from_matrix(seat_to_vessel),
                                                         .friction = k_wall_friction});
                !wall) {
                return std::unexpected(wall.error());
            }
        }
    }
    const double inertia_kg_m2 = 0.4 * k_body_mass_kg * k_body_radius_m * k_body_radius_m;
    const auto body = world->add_body(
        {.pieces = {physics::ShapePiece{.radius_m = k_body_radius_m}},
         .mass_kg = k_body_mass_kg,
         .inertia_kg_m2 =
             Matrix3{{inertia_kg_m2, 0.0, 0.0, 0.0, inertia_kg_m2, 0.0, 0.0, 0.0, inertia_kg_m2}},
         .state = {.position_m = pilot_->position_m, .velocity_m_s = pilot_->velocity_m_s},
         .friction = k_body_friction});
    if (!body) {
        return std::unexpected(body.error());
    }
    cabin_ = std::make_unique<Cabin>(
        Cabin{.world = std::move(*world), .body = *body, .part_count = parts.size(), .epoch = now_});
    pilot_reaction_.reset();
    return {};
}

core::VoidResult Simulation::advance_pilot(const time::Epoch& instant) {
    if (!pilot_.has_value()) {
        return {};
    }
    if (pilot_->vessel.index >= vessels_.size()) {
        pilot_.reset();
        cabin_.reset();
        return {};
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    if (pilot_->seated || vessel.status == VesselStatus::Crashed || !vessel.systems.has_value()) {
        if (!pilot_->seated) {
            seat_pilot(); // nowhere left to move about
        }
        return {};
    }
    // The vessel lost or gained parts: the cabin is built again around the pilot.
    if (cabin_ == nullptr || cabin_->part_count != vessel.systems->assembly().parts().size()) {
        if (!seat_of(vessel.systems->assembly()).has_value()) {
            seat_pilot();
            return {};
        }
        if (core::VoidResult built = build_cabin(); !built) {
            return built;
        }
    }
    const double step_s = options_.physics_step_s;
    const double behind_s = time::seconds_between(cabin_->epoch, instant);
    // Rationale: above physics warp, or after a stall, the pilot is taken to stay put relative
    // to the vessel; a cabin is not something to integrate over hours.
    if (effective_warp_ > options_.max_physics_warp
        || behind_s > step_s * options_.max_cabin_ticks_per_frame) {
        cabin_->epoch = instant;
        cabin_->grab_held = false;
        pilot_->grabbing = false;
        pilot_reaction_.reset();
        return {};
    }
    cabin_->impulse_n_s = {};
    cabin_->place_m_s = {};
    cabin_->ticked_s = 0.0;
    while (true) {
        const auto tick_end = cabin_->epoch.advanced_by(step_s);
        if (!tick_end) {
            return std::unexpected(tick_end.error());
        }
        if (*tick_end > instant) {
            break;
        }
        if (core::VoidResult ticked = cabin_tick(step_s); !ticked) {
            return ticked;
        }
        if (cabin_ == nullptr) {
            break; // the pilot was put back in the seat
        }
        cabin_->epoch = *tick_end;
    }
    // Rationale: a frame without a cabin tick keeps the reaction of the one before; the vessel's
    // own ticks do not line up with the frames either.
    if (cabin_ != nullptr && cabin_->ticked_s > 0.0) {
        pilot_reaction_ = PilotReaction{.force_n = -1.0 * cabin_->impulse_n_s / cabin_->ticked_s,
                                        .position_m = cabin_->place_m_s / cabin_->ticked_s};
    }
    return {};
}

core::VoidResult Simulation::cabin_tick(double step_s) {
    Pilot& pilot = *pilot_;
    Cabin& cabin = *cabin_;
    const Vessel& vessel = vessels_[pilot.vessel.index];
    if (!vessel.systems.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot's vessel has no cabin");
    }
    const vessel::VesselSystems& systems = *vessel.systems;
    physics::World& world = cabin.world;
    const auto before = world.state(cabin.body);
    const auto mass = systems.mass_properties(now_);
    if (!before || !mass) {
        return std::unexpected(!before ? before.error() : mass.error());
    }
    const Vector3& position_m = before->position_m;
    const Vector3& velocity_m_s = before->velocity_m_s;

    // The vessel's frame: what it feels is the cabin's gravity, and its turning the rest.
    const Quaternion to_vessel = math::conjugate(vessel.attitude.orientation);
    const Vector3 weight_m_s2 = -1.0 * math::rotate(to_vessel, vessel.proper_acceleration_m_s2);
    const Vector3 spin_rad_s = math::rotate(to_vessel, vessel.attitude.angular_velocity_rad_s);
    const Vector3 spin_rate_rad_s2 = cabin.has_spin ? (spin_rad_s - cabin.spin_rad_s) / step_s : Vector3{};
    cabin.spin_rad_s = spin_rad_s;
    cabin.has_spin = true;
    const Vector3 arm_m = position_m - mass->centre_of_mass_m;
    // What the frame alone does to a free body in it; everything else is a push from the vessel.
    const Vector3 frame_m_s2 = -1.0 * math::cross(spin_rad_s, math::cross(spin_rad_s, arm_m))
                               - 2.0 * math::cross(spin_rad_s, velocity_m_s)
                               - math::cross(spin_rate_rad_s2, arm_m);
    Vector3 acceleration_m_s2 = frame_m_s2;

    const PilotInput& input = pilot_input_;
    const Quaternion view = math::normalized(input.view);
    const Vector3 forward = math::rotate(view, {1.0, 0.0, 0.0});
    const Vector3 move{std::clamp(input.move.x, -1.0, 1.0), std::clamp(input.move.y, -1.0, 1.0),
                       std::clamp(input.move.z, -1.0, 1.0)};
    const double weight = math::norm(weight_m_s2);
    const bool heavy = weight >= k_weight_threshold_m_s2;
    const Vector3 up = heavy ? weight_m_s2 * (-1.0 / weight) : Vector3{};

    // The hand.
    const Vector3 eye_m = position_m + k_pilot_eye_height_m * math::rotate(view, {0.0, 0.0, 1.0});
    const auto within_reach = world.cast_ray(eye_m, forward, k_reach_m, cabin.body);
    const bool grab_pressed = input.grab && !cabin.grab_held;
    cabin.grab_held = input.grab;
    if (!input.grab) {
        pilot.grabbing = false;
    } else if (grab_pressed && within_reach.has_value()) {
        pilot.grabbing = true;
        pilot.grip_m = within_reach->point_m;
        cabin.grip_offset_m = position_m - within_reach->point_m;
        cabin.grip_normal = within_reach->normal;
    }
    if (pilot.grabbing) {
        // The arm moves the body relative to the hand, between bent and stretched.
        const Vector3 wanted = math::rotate(view, move);
        Vector3 offset_m = cabin.grip_offset_m + (k_arm_speed_m_s * step_s) * wanted;
        const double length_m = math::norm(offset_m);
        if (length_m > 0.0) {
            offset_m = offset_m * (std::clamp(length_m, k_arm_least_m, k_arm_length_m) / length_m);
        }
        // The arm does not pull the body into the surface it holds.
        const double clear_m = math::dot(offset_m, cabin.grip_normal);
        if (clear_m < k_body_radius_m + k_grip_clearance_m) {
            offset_m = offset_m + (k_body_radius_m + k_grip_clearance_m - clear_m) * cabin.grip_normal;
        }
        cabin.grip_offset_m = offset_m;
        // An arm pushed out straight has pushed off: the hand lets go, and the body keeps the
        // speed the push gave it. (Holding on at arm's length without pushing keeps the grip.)
        if (length_m > k_arm_length_m && math::dot(wanted, offset_m) > 0.5 * k_arm_length_m) {
            pilot.grabbing = false;
        }
    }
    if (pilot.grabbing) {
        const Vector3& offset_m = cabin.grip_offset_m;
        acceleration_m_s2 = acceleration_m_s2
                            + limited(k_grip_stiffness_per_s2 * (pilot.grip_m + offset_m - position_m)
                                          - k_grip_damping_per_s * velocity_m_s,
                                      k_grip_strength_m_s2);
    }

    // The legs.
    const bool jump_pressed = input.jump && !cabin.jump_held;
    cabin.jump_held = input.jump;
    pilot.standing = false;
    if (heavy) {
        const auto floor = world.cast_ray(position_m, -1.0 * up, k_leg_length_m + k_leg_slack_m, cabin.body);
        if (floor.has_value()) {
            pilot.standing = true;
            const double rising_m_s = math::dot(velocity_m_s, up);
            const double push_m_s2 = k_leg_stiffness_per_s2 * (k_leg_length_m - floor->distance_m)
                                     - k_leg_damping_per_s * rising_m_s;
            acceleration_m_s2 = acceleration_m_s2 + std::max(push_m_s2, 0.0) * up;
            if (jump_pressed) {
                acceleration_m_s2 = acceleration_m_s2 + (k_jump_speed_m_s / step_s) * up;
            }
            if (!pilot.grabbing) {
                // Walking: towards the wanted speed over the floor, no harder than the weight
                // on the feet lets them push.
                const Vector3 level = forward - math::dot(forward, up) * up;
                const double level_length = math::norm(level);
                const Vector3 ahead =
                    level_length > 1e-6
                        ? level / level_length
                        : math::rotate(view, {0.0, 0.0, math::dot(forward, up) > 0.0 ? -1.0 : 1.0});
                const Vector3 left = math::cross(up, ahead);
                const Vector3 wanted_m_s = k_walk_speed_m_s * (move.x * ahead + move.y * left);
                const Vector3 over_floor_m_s = velocity_m_s - rising_m_s * up;
                acceleration_m_s2 =
                    acceleration_m_s2
                    + limited(k_walk_gain_per_s * (wanted_m_s - over_floor_m_s), 0.6 * weight + 0.5);
            }
        }
    }

    world.set_gravity(weight_m_s2);
    if (core::VoidResult pushed = world.add_force(cabin.body, k_body_mass_kg * acceleration_m_s2, position_m);
        !pushed) {
        return pushed;
    }
    if (core::VoidResult stepped = world.step(step_s); !stepped) {
        return stepped;
    }
    const auto after = world.state(cabin.body);
    if (!after) {
        return std::unexpected(after.error());
    }
    // The push of the vessel on the pilot during the tick: legs, hand and walls together.
    const Vector3 pushed_m_s2 = (after->velocity_m_s - velocity_m_s) / step_s - frame_m_s2 - weight_m_s2;
    cabin.impulse_n_s += (k_body_mass_kg * step_s) * pushed_m_s2;
    cabin.place_m_s += step_s * position_m;
    cabin.ticked_s += step_s;
    pilot.position_m = after->position_m;
    pilot.velocity_m_s = after->velocity_m_s;
    pilot.view = view;
    pilot.up = up;
    pilot.in_reach = within_reach.has_value();

    const auto seat = seat_of(systems.assembly());
    if (!seat.has_value() || math::norm(pilot.position_m - seat->eye_m) > k_lost_distance_m) {
        LOG_WARN("the pilot left the cabin of '{}' and is put back in the seat", vessel.name)
            .tag("subsystem", "sim");
        seat_pilot();
    }
    return {};
}

} // namespace helios::sim
