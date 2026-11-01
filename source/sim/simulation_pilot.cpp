// The pilot in the cabin (BRIEFING §12, D27): the part of Simulation that moves the player's
// body, and the loose things around it, about the inside of a vessel.
//
// The cabin is a small physics world of its own, in the vessel's axes and at rest with the
// vessel: its walls do not move, and the bodies in it are the pilot (while out of the seat) and
// the loose items. What the vessel does is felt as the forces of its frame. Its proper
// acceleration (thrust, or the ground holding it up) is the cabin's "gravity", so everything
// floats in free fall, stands on the Moon, and is pressed to the aft wall under thrust, all by
// the same rule; a turning vessel adds the centrifugal, Coriolis and Euler terms.
//
// The other way round: every push the cabin gives a body in it (a floor under the feet, a wall
// kicked off from, a handhold, a crate hitting the ceiling) is given back to the vessel. While
// the vessel is in the physics bubble it feels that as a force and a torque like any other, so
// a pilot who kicks off moves the vessel a little the other way and one who runs round a
// spinning cabin makes it wobble. A vessel without a cabin, or with nobody aboard, is the one
// rigid body it is to everything else.
//
// Nothing here moves the pilot except contact and what the pilot throws: legs when there is
// weight to stand against, a hand that holds on to a surface, and the recoil of an item thrown.
// A body that floats free keeps its velocity until it touches something.
//
// Outside (BRIEFING D31) there is no cabin: the pilot is in a suit, and the suit is a vessel
// like any other, with its own parts, its own propagator and its place in the physics bubble.
// The airlock is where the one is exchanged for the other, behind the time it takes to put the
// suit on and let the air out.

#include "helios/core/logging.hpp"
#include "helios/sim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <span>
#include <utility>
#include <vector>

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
constexpr double k_item_friction = 0.6;
// Legs: they hold the body's centre this far above what is under it, as a damped spring, when
// the weight felt is at least the threshold.
constexpr double k_leg_length_m = 1.3;
constexpr double k_leg_slack_m = 0.2;
constexpr double k_leg_stiffness_per_s2 = 150.0;
constexpr double k_leg_damping_per_s = 24.0;
constexpr double k_weight_threshold_m_s2 = 0.5;
// Weight has to last this long before the legs take it: a jolt (something thrown that hits a
// wall) is not a floor to stand on.
constexpr double k_weight_settle_s = 0.25;
constexpr double k_walk_speed_m_s = 1.5;
constexpr double k_walk_gain_per_s = 8.0;
constexpr double k_jump_speed_m_s = 2.5;
// The hand: it takes hold of a surface within reach of the eyes, and the arm then moves the
// body relative to it, no further than its length.
constexpr double k_reach_m = 0.9;
// A suit left turning damps its own spin at this rate per second (D34): a suit has nobody
// steadying it, and a slow turn is one the pilot can live with. Against the pack's 15 N m it
// caps a free spin, but it does not noticeably hold back a turn the pilot is making.
constexpr double k_suit_spin_damping_per_s = 0.3;
// A loose item can be picked up from further off: the pilot bends and stretches for it, down
// to the floor from standing.
constexpr double k_item_reach_m = 1.6;
constexpr double k_arm_length_m = 1.1;
constexpr double k_arm_least_m = 0.3;
constexpr double k_grip_clearance_m = 0.05; // between the body and the surface the hand holds
constexpr double k_arm_speed_m_s = 1.2;
constexpr double k_grip_stiffness_per_s2 = 120.0;
constexpr double k_grip_damping_per_s = 22.0;
constexpr double k_grip_strength_m_s2 = 10.0;
constexpr double k_seat_reach_m = 1.0;
// An item in hand is carried this far ahead of the eyes, and let go of there.
constexpr double k_hold_distance_m = 0.5;
// A throw gives the item this much momentum relative to the pilot, within these speeds: a
// heavy crate leaves slowly and pushes the thrower back hard, a spanner leaves fast and hardly
// does.
constexpr double k_throw_impulse_n_s = 40.0;
constexpr double k_throw_least_m_s = 1.0;
constexpr double k_throw_most_m_s = 5.0;
// A pilot this far from the seat has left the cabin through a gap in its walls.
constexpr double k_lost_distance_m = 100.0;
// An airlock is used from this near: inside, by the body's centre; outside, by the suit's.
constexpr double k_airlock_reach_m = 1.2;
constexpr double k_hatch_reach_m = 2.5;
constexpr std::string_view k_suit_name = "EVA suit";

// On foot outside (BRIEFING D32). The legs hold the suit's lowest point this far off the ground,
// so that it is the legs and not the suit that the ground pushes.
constexpr double k_suit_clearance_m = 0.1;
// The hardest the legs push, as an acceleration of the suit and its wearer.
constexpr double k_leg_strength_m_s2 = 20.0;
// Balance: how the legs turn the body upright and to where it is to face.
constexpr double k_balance_stiffness_per_s2 = 60.0;
constexpr double k_balance_damping_per_s = 14.0;
constexpr double k_balance_strength_rad_s2 = 12.0;
// Leaning less than this (the sine of the angle) is standing straight.
constexpr double k_upright_sine = 0.05;
// A body that stands still is turned to where the pilot looks only once the look is this far
// off, and then until it is within the second angle: the head turns in the helmet first.
constexpr double k_turn_begin_rad = 0.6;
constexpr double k_turn_end_rad = 0.05;
// The input of a crewed part that says somebody is in it.
constexpr std::string_view k_worn_input = "worn";
// On a ladder (BRIEFING D35). The suit's centre hangs this far out from the ladder's line, and
// a ladder is in reach when the place the suit would hang at is this near.
constexpr double k_ladder_hang_m = 0.5;
constexpr double k_ladder_reach_m = 1.0;
// How fast the hands go up and down it, and across it.
constexpr double k_climb_speed_m_s = 0.6;
// The arm holds the suit where the hands are as a damped spring, no harder than this on top of
// carrying its weight, and turns it to face the hull.
constexpr double k_ladder_stiffness_per_s2 = 40.0;
constexpr double k_ladder_damping_per_s = 12.0;
constexpr double k_ladder_strength_m_s2 = 6.0;

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

[[nodiscard]] std::optional<Seat> seat_in(const Vessel& vessel) {
    const std::optional<vessel::VesselSystems>& systems = vessel.systems;
    return systems.has_value() ? seat_of(systems->assembly()) : std::nullopt;
}

// An airlock, in the vessel's axes from the vessel's origin.
struct Hatch {
    Vector3 inside_m;
    Vector3 outside_m;
    Vector3 head; // of someone standing in it, unit
    double cycle_s = 0.0;
};

// The airlock of the first cabin that has one.
[[nodiscard]] std::optional<Hatch> hatch_in(const Vessel& vessel) {
    if (!vessel.systems.has_value() || vessel.status == VesselStatus::Crashed
        || vessel.status == VesselStatus::Stowed) {
        return std::nullopt;
    }
    const vessel::Assembly& assembly = vessel.systems->assembly();
    const auto parts = assembly.parts();
    const auto poses = assembly.poses();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cockpit>& cockpit = parts[index].datasheet->cockpit;
        if (!cockpit.has_value() || !cockpit->airlock.has_value()) {
            continue;
        }
        const Matrix3 seat_to_vessel = poses[index].orientation * cockpit->seat_to_part();
        const Vector3 eye_m = poses[index].position_m + poses[index].orientation * cockpit->eye_m;
        return Hatch{.inside_m = eye_m + seat_to_vessel * cockpit->airlock->inside_m,
                     .outside_m = eye_m + seat_to_vessel * cockpit->airlock->outside_m,
                     .head = seat_to_vessel * Vector3{0.0, 0.0, 1.0},
                     .cycle_s = cockpit->airlock->cycle_s};
    }
    return std::nullopt;
}

// A ladder, with its axes worked out: `along` from the foot up to the top, `out` away from the
// hull and square to `along`, `across` to the left of someone facing the hull with their head
// up the ladder.
struct LadderAxes {
    Vector3 from_m;
    Vector3 along;
    Vector3 out;
    Vector3 across;
    double length_m = 0.0;
    double side_m = 0.0;

    [[nodiscard]] Vector3 hand_m(double along_m, double across_m) const noexcept {
        return from_m + along_m * along + across_m * across;
    }
    // Where the suit's centre hangs while the hands are there.
    [[nodiscard]] Vector3 hang_m(double along_m, double across_m) const noexcept {
        return hand_m(along_m, across_m) + k_ladder_hang_m * out;
    }
};

[[nodiscard]] std::optional<LadderAxes> ladder_axes(const Vector3& from_m, const Vector3& to_m,
                                                    const Vector3& out, double side_m) noexcept {
    const double length_m = math::norm(to_m - from_m);
    if (!(length_m > 0.0)) {
        return std::nullopt;
    }
    const Vector3 along = (to_m - from_m) / length_m;
    const Vector3 square = out - math::dot(out, along) * along;
    const double square_norm = math::norm(square);
    if (!(square_norm > 1.0e-9)) {
        return std::nullopt;
    }
    const Vector3 away = square / square_norm;
    return LadderAxes{.from_m = from_m,
                      .along = along,
                      .out = away,
                      .across = math::cross(away, along),
                      .length_m = length_m,
                      .side_m = side_m};
}

// The ladder of the first cabin that has one, in the vessel's axes from its origin, on a vessel
// that is still there.
[[nodiscard]] std::optional<LadderAxes> ladder_in(const Vessel& vessel) {
    if (!vessel.systems.has_value() || vessel.status == VesselStatus::Crashed
        || vessel.status == VesselStatus::Stowed) {
        return std::nullopt;
    }
    const vessel::Assembly& assembly = vessel.systems->assembly();
    const auto parts = assembly.parts();
    const auto poses = assembly.poses();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cockpit>& cockpit = parts[index].datasheet->cockpit;
        if (!cockpit.has_value() || !cockpit->ladder.has_value()) {
            continue;
        }
        const vessel::Ladder& ladder = *cockpit->ladder;
        const Matrix3 seat_to_vessel = poses[index].orientation * cockpit->seat_to_part();
        const Vector3 eye_m = poses[index].position_m + poses[index].orientation * cockpit->eye_m;
        return ladder_axes(eye_m + seat_to_vessel * ladder.from_m, eye_m + seat_to_vessel * ladder.to_m,
                           seat_to_vessel * ladder.out_m, ladder.side_m);
    }
    return std::nullopt;
}

// The signal that says whether someone is in a vessel's crewed part (D33): the `worn` input of
// the first crewed part, by the name its instance in the vessel gives it. Nothing where the
// crewed part has no such input: then there is nothing to say.
[[nodiscard]] std::optional<std::string> worn_signal(const vessel::Assembly& assembly) {
    for (const vessel::PartInstance& part : assembly.parts()) {
        if (!part.datasheet->cockpit.has_value()) {
            continue;
        }
        const auto& inputs = part.datasheet->inputs;
        if (std::ranges::find(inputs, k_worn_input, &vessel::InputPort::name) == inputs.end()) {
            return std::nullopt;
        }
        return std::format("{}/{}", part.name, k_worn_input);
    }
    return std::nullopt;
}

// Puts the pilot into `systems`' suit, or out of it again: the suit breathes only while someone
// is in it, and holds its breath in the locker (D33).
[[nodiscard]] core::VoidResult set_worn(vessel::VesselSystems& systems, bool worn,
                                        const time::Epoch& instant) {
    const std::optional<std::string> signal = worn_signal(systems.assembly());
    if (!signal.has_value()) {
        return {};
    }
    std::vector<vessel::Separation> separations;
    return systems.command(*signal, vessel::ControlSource::Sequencer, worn ? 1.0 : 0.0,
                           std::max(instant, systems.propulsion().epoch), separations);
}

// Where a point of a vessel (its axes, from its origin) is about the vessel's domain body.
[[nodiscard]] std::optional<Vector3> place_about_domain(const Vessel& vessel, const Vector3& point_m,
                                                        const time::Epoch& instant) {
    if (!vessel.systems.has_value()) {
        return std::nullopt;
    }
    const auto mass = vessel.systems->mass_properties(instant);
    if (!mass) {
        return std::nullopt;
    }
    return vessel.state.state_in_domain.position_m
           + math::rotate(vessel.attitude.orientation, point_m - mass->centre_of_mass_m);
}

// The other way: where a point about the vessel's domain body is in the vessel's axes.
[[nodiscard]] std::optional<Vector3> place_in_vessel(const Vessel& vessel, const Vector3& about_domain_m,
                                                     const time::Epoch& instant) {
    if (!vessel.systems.has_value()) {
        return std::nullopt;
    }
    const auto mass = vessel.systems->mass_properties(instant);
    if (!mass) {
        return std::nullopt;
    }
    return mass->centre_of_mass_m
           + math::rotate(math::conjugate(vessel.attitude.orientation),
                          about_domain_m - vessel.state.state_in_domain.position_m);
}

// The loose items of every cabin of the vessel, where their datasheets put them.
[[nodiscard]] std::vector<CabinItem> stowed_items(const vessel::Assembly& assembly) {
    std::vector<CabinItem> items;
    const auto parts = assembly.parts();
    const auto poses = assembly.poses();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const std::optional<vessel::Cockpit>& cockpit = parts[index].datasheet->cockpit;
        if (!cockpit.has_value()) {
            continue;
        }
        const Matrix3 seat_to_vessel = poses[index].orientation * cockpit->seat_to_part();
        const Vector3 eye_m = poses[index].position_m + poses[index].orientation * cockpit->eye_m;
        for (const vessel::CockpitItem& item : cockpit->items) {
            items.push_back(CabinItem{.name = item.name,
                                      .size_m = item.size_m,
                                      .mass_kg = item.mass_kg,
                                      .colour = item.colour,
                                      .position_m = eye_m + seat_to_vessel * item.position_m,
                                      .orientation = math::from_matrix(seat_to_vessel),
                                      .velocity_m_s = {}});
        }
    }
    return items;
}

[[nodiscard]] Vector3 column(const Matrix3& matrix, std::size_t index) noexcept {
    return {matrix(0, index), matrix(1, index), matrix(2, index)};
}

// Where the body's centre is when the pilot sits.
[[nodiscard]] Vector3 seated_position_m(const Seat& seat) noexcept {
    return seat.eye_m - k_pilot_eye_height_m * column(seat.seat_to_vessel, 2);
}

// Universe axes to the body-fixed ones of `body` at `instant`: the ground's.
[[nodiscard]] Matrix3 to_ground_axes(const bodies::BodyCatalog& catalog, bodies::BodyId body,
                                     const time::Epoch& instant) noexcept {
    const auto found = catalog.body(body);
    if (!found) {
        return Matrix3{};
    }
    const auto& rotation = found->get().rotation;
    return rotation.has_value() ? rotation->universe_to_body_fixed(instant) : Matrix3{};
}

// `vector` without its part along the unit vector `up`, at unit length; `fallback` if nothing
// is left of it.
[[nodiscard]] Vector3 levelled(const Vector3& vector, const Vector3& up, const Vector3& fallback) noexcept {
    const Vector3 level = vector - math::dot(vector, up) * up;
    const double length = math::norm(level);
    return length > 1e-6 ? level / length : fallback;
}

// The angle from one level direction to another, about `up`.
[[nodiscard]] double heading_between_rad(const Vector3& from, const Vector3& to, const Vector3& up) noexcept {
    return std::atan2(math::dot(math::cross(from, to), up), math::dot(from, to));
}

// The look of someone standing in a suit and looking straight ahead: `seat_to_frame` takes the
// helmet's axes (forward, left, up) to those the look is wanted in, where `up` is.
[[nodiscard]] Quaternion look_ahead(const Matrix3& seat_to_frame, const Vector3& up) noexcept {
    // Lying on its back the suit faces nowhere level: towards where its feet are, then.
    const Vector3 ahead = levelled(
        column(seat_to_frame, 0), up,
        levelled(-1.0 * column(seat_to_frame, 2), up, levelled({1.0, 0.0, 0.0}, up, {0.0, 1.0, 0.0})));
    const Vector3 left = math::cross(up, ahead);
    return math::from_matrix(Matrix3{{ahead.x, left.x, up.x, ahead.y, left.y, up.y, ahead.z, left.z, up.z}});
}

[[nodiscard]] Vector3 limited(const Vector3& vector, double greatest) noexcept {
    const double length = math::norm(vector);
    return length > greatest ? vector * (greatest / length) : vector;
}

[[nodiscard]] Matrix3 ball_inertia_kg_m2(double mass_kg) noexcept {
    const double moment = 0.4 * mass_kg * k_body_radius_m * k_body_radius_m;
    return Matrix3{{moment, 0.0, 0.0, 0.0, moment, 0.0, 0.0, 0.0, moment}};
}

[[nodiscard]] physics::BodyDescription item_body(const CabinItem& item) {
    const Vector3& size_m = item.size_m;
    const double twelfth_kg = item.mass_kg / 12.0;
    return {.pieces = {physics::ShapePiece{.box_half_extents_m = 0.5 * size_m}},
            .mass_kg = item.mass_kg,
            .inertia_kg_m2 = Matrix3{{twelfth_kg * (size_m.y * size_m.y + size_m.z * size_m.z), 0.0, 0.0, 0.0,
                                      twelfth_kg * (size_m.x * size_m.x + size_m.z * size_m.z), 0.0, 0.0, 0.0,
                                      twelfth_kg * (size_m.x * size_m.x + size_m.y * size_m.y)}},
            .state = {.position_m = item.position_m,
                      .orientation = item.orientation,
                      .velocity_m_s = item.velocity_m_s},
            .friction = k_item_friction};
}

[[nodiscard]] physics::BodyDescription pilot_body(const Pilot& pilot, double mass_kg) {
    return {.pieces = {physics::ShapePiece{.radius_m = k_body_radius_m}},
            .mass_kg = mass_kg,
            .inertia_kg_m2 = ball_inertia_kg_m2(mass_kg),
            .state = {.position_m = pilot.position_m, .velocity_m_s = pilot.velocity_m_s},
            .friction = k_body_friction};
}

// The item in the pilot's hand, as an index into `items`; none if the hand is empty.
[[nodiscard]] std::optional<std::size_t> item_in_hand(const Pilot& pilot,
                                                      std::span<const CabinItem> items) noexcept {
    const std::optional<std::size_t> held = pilot.held_item;
    if (!held.has_value() || *held >= items.size()) {
        return std::nullopt;
    }
    return held;
}

// The pilot's own mass and that of whatever is in hand.
[[nodiscard]] double carried_mass_kg(const Pilot& pilot, std::span<const CabinItem> items) noexcept {
    const std::optional<std::size_t> held = item_in_hand(pilot, items);
    return k_body_mass_kg + (held.has_value() ? items[*held].mass_kg : 0.0);
}

} // namespace

core::VoidResult Simulation::board(VesselId id) {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    const Vessel& vessel = vessels_[id.index];
    const std::optional<vessel::VesselSystems>& systems = vessel.systems;
    const std::optional<Seat> seat =
        vessel.status != VesselStatus::Crashed && vessel.status != VesselStatus::Stowed ? seat_in(vessel)
                                                                                        : std::nullopt;
    if (!seat.has_value() || !systems.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("'{}' has no seat to take", vessel.name));
    }
    cabin_.reset();
    cabin_reaction_.reset();
    cabin_items_.clear();
    passage_.reset();
    ladder_latch_.reset();
    pilot_ = Pilot{.vessel = id,
                   .seated = true,
                   .position_m = seated_position_m(*seat),
                   .view = math::from_matrix(seat->seat_to_vessel)};
    if (seat->walkable) {
        cabin_items_ = stowed_items(systems->assembly());
        if (core::VoidResult built = build_cabin(); !built) {
            return built;
        }
    }
    update_offer();
    return {};
}

// Back into the seat from wherever the pilot is; what was in hand is left where it was.
void Simulation::seat_pilot() {
    if (!pilot_.has_value()) {
        return;
    }
    passage_.reset();
    const std::optional<Seat> seat =
        pilot_->vessel.index < vessels_.size() ? seat_in(vessels_[pilot_->vessel.index]) : std::nullopt;
    if (!seat.has_value()) {
        // The part with the seat is gone, and the pilot with it.
        pilot_.reset();
        cabin_.reset();
        cabin_items_.clear();
        cabin_reaction_.reset();
        return;
    }
    if (pilot_->held_item.has_value()) {
        if (core::VoidResult released = release_item(0.0); !released) {
            LOG_WARN("{}", core::describe(released.error())).tag("subsystem", "sim");
        }
    }
    if (const std::optional<physics::BodyId> body = cabin_ != nullptr ? cabin_->body : std::nullopt;
        body.has_value()) {
        if (core::VoidResult removed = cabin_->world.remove(*body); !removed) {
            LOG_WARN("{}", core::describe(removed.error())).tag("subsystem", "sim");
        }
        cabin_->body.reset();
        cabin_->grab_held = false;
        cabin_->jump_held = false;
        cabin_->item_in_reach.reset();
    }
    *pilot_ = Pilot{.vessel = pilot_->vessel,
                    .seated = true,
                    .position_m = seated_position_m(*seat),
                    .view = math::from_matrix(seat->seat_to_vessel)};
    update_offer();
}

core::VoidResult Simulation::leave_seat() {
    if (!pilot_.has_value() || !pilot_->seated) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not in a seat");
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    if (vessel.status == VesselStatus::Crashed || cabin_ == nullptr) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("'{}' has no cabin to move about in", vessel.name));
    }
    const auto body = cabin_->world.add_body(pilot_body(*pilot_, k_body_mass_kg));
    if (!body) {
        return std::unexpected(body.error());
    }
    cabin_->body = *body;
    pilot_->seated = false;
    update_offer();
    return {};
}

core::VoidResult Simulation::take_seat() {
    if (!pilot_.has_value() || pilot_->seated || passage_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not out of the seat");
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    const std::optional<Seat> seat = seat_in(vessel);
    if (!seat.has_value() || math::norm(pilot_->position_m - seated_position_m(*seat)) > k_seat_reach_m) {
        return core::fail(ErrorCode::InvalidArgument, "the seat is out of reach");
    }
    seat_pilot();
    return {};
}

core::VoidResult Simulation::interact() {
    if (!pilot_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is no pilot");
    }
    switch (pilot_->offer) {
    case PilotOffer::LeaveSeat:  return leave_seat();
    case PilotOffer::TakeSeat:   return take_seat();
    case PilotOffer::PutDown:    return release_item(0.0);
    case PilotOffer::GoOutside:  return go_outside();
    case PilotOffer::ComeInside: return come_inside();
    case PilotOffer::TakeLadder: return grab_ladder();
    case PilotOffer::LetGo:      return let_go_ladder();
    case PilotOffer::PickUp:
        if (const std::optional<std::size_t> item = cabin_ != nullptr ? cabin_->item_in_reach : std::nullopt;
            item.has_value()) {
            return pick_up(*item);
        }
        break;
    case PilotOffer::None: break;
    }
    return core::fail(ErrorCode::InvalidArgument, "there is nothing to do here");
}

// The item joins the pilot: one body from then on, with the momentum of the two.
core::VoidResult Simulation::pick_up(std::size_t item) {
    if (!pilot_.has_value() || cabin_ == nullptr || pilot_->held_item.has_value()
        || item >= cabin_items_.size() || item >= cabin_->items.size()) {
        return core::fail(ErrorCode::InvalidArgument, "there is nothing to pick up");
    }
    const std::optional<physics::BodyId> own = cabin_->body;
    const std::optional<physics::BodyId> lying = cabin_->items[item];
    if (!own.has_value() || !lying.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is nothing to pick up");
    }
    physics::World& world = cabin_->world;
    const physics::BodyId taken = *lying;
    const auto body = world.state(*own);
    const auto loose = world.state(taken);
    if (!body || !loose) {
        return std::unexpected(!body ? body.error() : loose.error());
    }
    const double item_kg = cabin_items_[item].mass_kg;
    const double together_kg = k_body_mass_kg + item_kg;
    physics::BodyState joined = *body;
    joined.velocity_m_s = (k_body_mass_kg * body->velocity_m_s + item_kg * loose->velocity_m_s) / together_kg;
    if (const auto error =
            core::first_error(world.remove(taken), world.set_state(*own, joined),
                              world.set_mass(*own, together_kg, ball_inertia_kg_m2(together_kg)))) {
        return std::unexpected(*error);
    }
    cabin_->items[item].reset();
    cabin_->item_in_reach.reset();
    cabin_items_[item].held = true;
    pilot_->held_item = item;
    pilot_->velocity_m_s = joined.velocity_m_s;
    update_offer();
    return {};
}

core::VoidResult Simulation::release_item(double speed_m_s) {
    if (!pilot_.has_value() || cabin_ == nullptr) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot holds nothing");
    }
    const std::optional<std::size_t> held = item_in_hand(*pilot_, cabin_items_);
    if (!held.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot holds nothing");
    }
    const std::size_t index = *held;
    CabinItem& item = cabin_items_[index];
    physics::World& world = cabin_->world;
    const Vector3 forward = math::rotate(pilot_->view, {1.0, 0.0, 0.0});
    const Vector3 eye_m =
        pilot_->position_m + k_pilot_eye_height_m * math::rotate(pilot_->view, {0.0, 0.0, 1.0});
    // As far ahead as it was carried, or as far as there is room for it.
    const double item_radius_m = 0.5 * math::norm(item.size_m);
    const auto ahead = world.cast_ray(eye_m, forward, k_hold_distance_m + item_radius_m, cabin_->body);
    const double distance_m = ahead.has_value()
                                  ? std::clamp(ahead->distance_m - item_radius_m, 0.0, k_hold_distance_m)
                                  : k_hold_distance_m;
    // The two part with `speed_m_s` between them and the momentum they had together.
    const double together_kg = k_body_mass_kg + item.mass_kg;
    const Vector3 pilot_m_s = pilot_->velocity_m_s - (item.mass_kg / together_kg * speed_m_s) * forward;
    item.position_m = eye_m + distance_m * forward;
    item.orientation = pilot_->view;
    item.velocity_m_s = pilot_m_s + speed_m_s * forward;
    item.held = false;
    pilot_->held_item.reset();
    pilot_->velocity_m_s = pilot_m_s;
    const auto body = world.add_body(item_body(item));
    if (!body) {
        return std::unexpected(body.error());
    }
    cabin_->items[index] = *body;
    if (const std::optional<physics::BodyId> own = cabin_->body; own.has_value()) {
        auto state = world.state(*own);
        if (!state) {
            return std::unexpected(state.error());
        }
        state->velocity_m_s = pilot_m_s;
        if (const auto error =
                core::first_error(world.set_state(*own, *state),
                                  world.set_mass(*own, k_body_mass_kg, ball_inertia_kg_m2(k_body_mass_kg)))) {
            return std::unexpected(*error);
        }
    }
    update_offer();
    return {};
}

core::VoidResult Simulation::throw_item() {
    const std::optional<std::size_t> held =
        pilot_.has_value() ? item_in_hand(*pilot_, cabin_items_) : std::nullopt;
    if (!held.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot holds nothing to throw");
    }
    const double mass_kg = cabin_items_[*held].mass_kg;
    return release_item(std::clamp(k_throw_impulse_n_s / mass_kg, k_throw_least_m_s, k_throw_most_m_s));
}

// What the one key would do now (see PilotOffer).
void Simulation::update_offer() {
    if (!pilot_.has_value()) {
        return;
    }
    Pilot& pilot = *pilot_;
    pilot.offer = PilotOffer::None;
    pilot.offer_item.clear();
    if (passage_.has_value()) {
        return; // in the airlock there is only the wait
    }
    if (pilot.outside) {
        // The hatch first: at the top of the ladder that is the way on.
        if (hatch_within_reach().has_value()) {
            pilot.offer = PilotOffer::ComeInside;
        } else if (ladder_latch_.has_value()) {
            pilot.offer = PilotOffer::LetGo;
        } else if (ladder_near().has_value()) {
            pilot.offer = PilotOffer::TakeLadder;
        }
        return;
    }
    if (pilot.seated) {
        pilot.offer = cabin_ != nullptr ? PilotOffer::LeaveSeat : PilotOffer::None;
        return;
    }
    if (const std::optional<std::size_t> held = item_in_hand(pilot, cabin_items_); held.has_value()) {
        pilot.offer = PilotOffer::PutDown;
        pilot.offer_item = cabin_items_[*held].name;
        return;
    }
    if (const std::optional<std::size_t> item = cabin_ != nullptr ? cabin_->item_in_reach : std::nullopt;
        item.has_value() && *item < cabin_items_.size()) {
        pilot.offer = PilotOffer::PickUp;
        pilot.offer_item = cabin_items_[*item].name;
        return;
    }
    // The seat or the airlock, whichever is nearer of those in reach.
    const Vessel& vessel = vessels_[pilot.vessel.index];
    const std::optional<Seat> seat = seat_in(vessel);
    const std::optional<Hatch> hatch = suit_.has_value() ? hatch_in(vessel) : std::nullopt;
    const double to_seat_m = seat.has_value() ? math::norm(pilot.position_m - seated_position_m(*seat)) : 0.0;
    const double to_hatch_m = hatch.has_value() ? math::norm(pilot.position_m - hatch->inside_m) : 0.0;
    const bool seat_in_reach = seat.has_value() && to_seat_m <= k_seat_reach_m;
    const bool hatch_in_reach = hatch.has_value() && to_hatch_m <= k_airlock_reach_m;
    if (hatch_in_reach && (!seat_in_reach || to_hatch_m < to_seat_m)) {
        pilot.offer = PilotOffer::GoOutside;
    } else if (seat_in_reach) {
        pilot.offer = PilotOffer::TakeSeat;
    }
}

void Simulation::face_as_suit(const Vessel& suit) {
    const std::optional<Seat> helmet = seat_in(suit);
    const double radius_m = math::norm(suit.state.state_in_domain.position_m);
    if (!pilot_.has_value() || !helmet.has_value() || radius_m <= 0.0) {
        return;
    }
    const Matrix3 to_ground = to_ground_axes(*catalog_, suit.state.domain, now_);
    const Quaternion view =
        look_ahead(to_ground * math::to_matrix(suit.attitude.orientation) * helmet->seat_to_vessel,
                   to_ground * (suit.state.state_in_domain.position_m / radius_m));
    pilot_input_ = PilotInput{.view = view};
    pilot_->view = view;
    legs_ = {};
}

// The legs of the suited pilot, for one tick of the bubble. They work as in a cabin (D27): a
// spring under the body while there is weight and something below the feet, a push along the
// ground no harder than the weight on them lets the feet give, and a jump. Outside there is no
// vessel to be upright in, so the legs also keep the body's head end up and turn it to where
// the pilot looks.
LegPush Simulation::suit_legs(const LegsTick& tick) {
    LegPush legs;
    if (!pilot_.has_value() || pilot_->vessel.index >= vessels_.size()) {
        return legs;
    }
    if (tick.ladder.has_value() && ladder_latch_.has_value()) {
        return ladder_hold(tick);
    }
    Pilot& pilot = *pilot_;
    const std::optional<Seat> helmet = seat_in(vessels_[pilot.vessel.index]);
    const double weight = math::norm(tick.apparent_gravity_m_s2);
    pilot.standing = false;
    if (weight < k_weight_threshold_m_s2 || !helmet.has_value() || tick.world == nullptr) {
        // Nothing to stand with: the suit is flown, if it can be. A turn that is not being
        // made settles at once: the suit steadies itself at 0.3 per second (D34), which
        // caps a tumble without holding back a turn the pilot wants.
        pilot.seated = true;
        pilot.up = {};
        legs_ = {};
        const Vector3 settled_rad_s2 = -k_suit_spin_damping_per_s * tick.before.angular_velocity_rad_s;
        const Quaternion& orientation = tick.before.orientation;
        legs.torque_n_m = math::rotate(
            orientation, tick.inertia_kg_m2 * math::rotate(math::conjugate(orientation), settled_rad_s2));
        return legs;
    }
    const Vector3 up = tick.apparent_gravity_m_s2 * (-1.0 / weight);
    const Matrix3 seat_to_world = math::to_matrix(tick.before.orientation) * helmet->seat_to_vessel;
    const Matrix3 to_ground = math::transpose(tick.ground_to_world);
    if (pilot.seated) {
        // Coming to the ground in a suit that was flown: looking where the suit faces.
        pilot.seated = false;
        pilot_input_ = PilotInput{.view = look_ahead(to_ground * seat_to_world, to_ground * up)};
        legs_ = {};
    }
    const PilotInput& input = pilot_input_;
    const Quaternion view = math::normalized(input.view);
    pilot.up = to_ground * up;
    pilot.view = view;
    const bool jump_pressed = input.jump && !legs_.jump_held;
    legs_.jump_held = input.jump;

    const Vector3& position_m = tick.before.position_m;
    const double leg_length_m = tick.standing_height_m + k_suit_clearance_m;
    const auto floor = tick.world->cast_ray(position_m, -1.0 * up, leg_length_m + k_leg_slack_m, tick.body);
    if (!floor.has_value()) {
        return legs; // in the air nothing moves the body (D27)
    }
    pilot.standing = true;
    legs.standing = true;
    legs.footing = floor->body;
    legs.foot_m = floor->point_m;
    // What is stood on may move itself: another vessel that has not come to rest.
    Vector3 footing_m_s;
    Vector3 footing_spin_rad_s;
    if (const auto footing = tick.world->state(floor->body)) {
        footing_spin_rad_s = footing->angular_velocity_rad_s;
        footing_m_s =
            footing->velocity_m_s + math::cross(footing_spin_rad_s, floor->point_m - footing->position_m);
    }
    const Vector3 velocity_m_s = tick.before.velocity_m_s - footing_m_s;
    const double rising_m_s = math::dot(velocity_m_s, up);
    const double push_m_s2 =
        k_leg_stiffness_per_s2 * (leg_length_m - floor->distance_m) - k_leg_damping_per_s * rising_m_s;
    Vector3 acceleration_m_s2 = std::clamp(push_m_s2, 0.0, k_leg_strength_m_s2) * up;
    if (jump_pressed) {
        // The same legs as in the cabin, with the suit to lift as well.
        const double speed_m_s =
            k_jump_speed_m_s * std::sqrt(k_body_mass_kg / std::max(tick.mass_kg, k_body_mass_kg));
        acceleration_m_s2 = acceleration_m_s2 + (speed_m_s / tick.step_s) * up;
    }

    // Where the body faces, and where the pilot looks.
    const Vector3 head = column(seat_to_world, 2);
    const Vector3 gaze = tick.ground_to_world * math::rotate(view, {1.0, 0.0, 0.0});
    const Vector3 facing = levelled(column(seat_to_world, 0), up, levelled(-1.0 * head, up, gaze));
    const Vector3 ahead = levelled(gaze, up, facing);
    const double off_rad = std::abs(heading_between_rad(facing, ahead, up));
    const Vector3 move{std::clamp(input.move.x, -1.0, 1.0), std::clamp(input.move.y, -1.0, 1.0), 0.0};
    const bool walking = move.x != 0.0 || move.y != 0.0;
    if (walking || off_rad > k_turn_begin_rad) {
        legs_.turning = true;
    } else if (off_rad < k_turn_end_rad) {
        legs_.turning = false;
    }

    // Walking: towards the wanted speed over the ground.
    const Vector3 left = math::cross(up, ahead);
    const Vector3 wanted_m_s = k_walk_speed_m_s * (move.x * ahead + move.y * left);
    const Vector3 over_ground_m_s = velocity_m_s - rising_m_s * up;
    acceleration_m_s2 =
        acceleration_m_s2 + limited(k_walk_gain_per_s * (wanted_m_s - over_ground_m_s), 0.6 * weight + 0.5);
    legs.force_n = tick.mass_kg * acceleration_m_s2;

    // Balance: the turn that takes the body's axes to upright and facing where it should, as
    // an axis times the sine of the angle.
    // Rationale: towards a look straight behind, the turn below is nothing (the cross product
    // of opposite directions); a quarter turn to that side gets the body going.
    Vector3 wanted_facing = legs_.turning ? ahead : facing;
    if (legs_.turning && math::dot(ahead, facing) < 0.0) {
        const Vector3 to_left = math::cross(up, facing);
        wanted_facing = heading_between_rad(facing, ahead, up) >= 0.0 ? to_left : -1.0 * to_left;
    }
    const Vector3 wanted_left = math::cross(up, wanted_facing);
    const Vector3 lean = 0.5
                         * (math::cross(head, up) + math::cross(column(seat_to_world, 0), wanted_facing)
                            + math::cross(column(seat_to_world, 1), wanted_left));
    const Vector3 turn_rad_s2 =
        limited(k_balance_stiffness_per_s2 * lean
                    - k_balance_damping_per_s * (tick.before.angular_velocity_rad_s - footing_spin_rad_s),
                k_balance_strength_rad_s2);
    const Quaternion& orientation = tick.before.orientation;
    legs.torque_n_m = math::rotate(
        orientation, tick.inertia_kg_m2 * math::rotate(math::conjugate(orientation), turn_rad_s2));
    legs.busy = walking || jump_pressed || legs_.turning || math::norm(lean) > k_upright_sine;
    return legs;
}

// Whether the pilot, standing still on the ground in a suit that is pinned there like any
// landed vessel, asks the legs for something.
bool Simulation::legs_wanted() const noexcept {
    if (!pilot_.has_value() || !pilot_->outside || pilot_->seated || passage_.has_value()
        || pilot_->vessel.index >= vessels_.size()) {
        return false;
    }
    const Vessel& suit = vessels_[pilot_->vessel.index];
    if (suit.status != VesselStatus::Landed || !suit.landed.has_value() || !suit.systems.has_value()) {
        return false;
    }
    if (ladder_latch_.has_value()) {
        return true; // hands on a ladder: the arm holds the suit, not the ground
    }
    const PilotInput& input = pilot_input_;
    if (input.move.x != 0.0 || input.move.y != 0.0 || (input.jump && !legs_.jump_held)) {
        return true;
    }
    const std::optional<Seat> helmet = seat_of(suit.systems->assembly());
    const double radius_m = math::norm(suit.landed->position_m);
    if (!helmet.has_value() || radius_m <= 0.0) {
        return false;
    }
    const Vector3 up = suit.landed->position_m / radius_m;
    const Matrix3 seat_to_ground = math::to_matrix(suit.landed->orientation) * helmet->seat_to_vessel;
    const Vector3 gaze = math::rotate(math::normalized(input.view), {1.0, 0.0, 0.0});
    const Vector3 facing = levelled(column(seat_to_ground, 0), up, gaze);
    return std::abs(heading_between_rad(facing, levelled(gaze, up, facing), up)) > k_turn_begin_rad;
}

std::optional<Simulation::LadderLine> Simulation::held_ladder() const {
    if (!ladder_latch_.has_value() || ladder_latch_->host.index >= vessels_.size()) {
        return std::nullopt;
    }
    const std::optional<LadderAxes> ladder = ladder_in(vessels_[ladder_latch_->host.index]);
    if (!ladder.has_value()) {
        return std::nullopt;
    }
    return LadderLine{.host = ladder_latch_->host,
                      .from_m = ladder->from_m,
                      .to_m = ladder->from_m + ladder->length_m * ladder->along,
                      .out_m = ladder->out,
                      .side_m = ladder->side_m};
}

// The nearest ladder the suit could hang from, of the vessels about the same body.
std::optional<Simulation::LadderLatch> Simulation::ladder_near() const {
    if (!pilot_.has_value() || !pilot_->outside || passage_.has_value()
        || pilot_->vessel.index >= vessels_.size()) {
        return std::nullopt;
    }
    const Vessel& suit = vessels_[pilot_->vessel.index];
    if (suit.status != VesselStatus::Flying && suit.status != VesselStatus::Landed) {
        return std::nullopt;
    }
    std::optional<LadderLatch> nearest;
    double nearest_m = k_ladder_reach_m;
    for (std::uint32_t index = 0; index < vessels_.size(); ++index) {
        const Vessel& vessel = vessels_[index];
        if (index == pilot_->vessel.index || vessel.state.domain != suit.state.domain) {
            continue;
        }
        const std::optional<LadderAxes> ladder = ladder_in(vessel);
        if (!ladder.has_value()) {
            continue;
        }
        const std::optional<Vector3> suit_m =
            place_in_vessel(vessel, suit.state.state_in_domain.position_m, now_);
        if (!suit_m.has_value()) {
            continue;
        }
        const Vector3 offset_m = *suit_m - ladder->from_m;
        if (math::dot(offset_m, ladder->out) <= 0.0) {
            continue; // on the other side of the hull
        }
        const double along_m = std::clamp(math::dot(offset_m, ladder->along), 0.0, ladder->length_m);
        const double across_m =
            std::clamp(math::dot(offset_m, ladder->across), -ladder->side_m, ladder->side_m);
        const double distance_m = math::norm(*suit_m - ladder->hang_m(along_m, across_m));
        if (distance_m <= nearest_m) {
            nearest_m = distance_m;
            nearest = LadderLatch{.host = VesselId{index}, .along_m = along_m, .side_m = across_m};
        }
    }
    return nearest;
}

bool Simulation::ladder_in_reach() const {
    return ladder_near().has_value();
}

core::VoidResult Simulation::grab_ladder() {
    if (!pilot_.has_value() || !pilot_->outside || passage_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not outside");
    }
    if (ladder_latch_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot already holds a ladder");
    }
    const std::optional<LadderLatch> near = ladder_near();
    if (!near.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is no ladder within reach");
    }
    ladder_latch_ = near;
    LOG_INFO("the pilot takes hold of the ladder of '{}'", vessels_[near->host.index].name)
        .tag("subsystem", "sim");
    update_offer();
    return {};
}

core::VoidResult Simulation::let_go_ladder() {
    if (!ladder_latch_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot holds no ladder");
    }
    ladder_latch_.reset();
    legs_ = {};
    update_offer();
    return {};
}

// Holding on to a ladder, for one tick of the bubble (D35): the hands move up, down and across
// it as the walking keys say, and the arm holds the suit's centre out from them as a damped
// spring that also carries its weight. Whatever the arm does to the suit it does the other way
// to the hull at the hands. The body is turned to face the hull, head up the ladder.
LegPush Simulation::ladder_hold(const LegsTick& tick) {
    LegPush hold;
    if (!tick.ladder.has_value() || !ladder_latch_.has_value() || !pilot_.has_value()) {
        return hold;
    }
    const LadderTick& ladder = *tick.ladder;
    LadderLatch& latch = *ladder_latch_;
    Pilot& pilot = *pilot_;
    const std::optional<Seat> helmet = seat_in(vessels_[pilot.vessel.index]);
    const std::optional<LadderAxes> axes =
        ladder_axes(ladder.from_m, ladder.to_m, ladder.out_m, ladder.side_m);
    if (!helmet.has_value() || !axes.has_value()) {
        return hold;
    }
    const Matrix3 seat_to_world = math::to_matrix(tick.before.orientation) * helmet->seat_to_vessel;
    const Matrix3 to_ground = to_ground_axes(*catalog_, vessels_[pilot.vessel.index].state.domain, now_);
    if (pilot.seated) {
        // From flying the suit: looking where it faces, head up the ladder.
        pilot_input_ = PilotInput{.view = look_ahead(to_ground * seat_to_world, to_ground * axes->along)};
    }
    if (pilot.seated || pilot.standing) {
        pilot.seated = false;
        pilot.standing = false;
        legs_ = {};
    }
    pilot.up = to_ground * axes->along;
    pilot.view = math::normalized(pilot_input_.view);

    // The hands move first, along the ladder and no further than its ends.
    const PilotInput& input = pilot_input_;
    const double climb = std::clamp(input.move.x, -1.0, 1.0);
    const double sideways = std::clamp(input.move.y, -1.0, 1.0);
    latch.along_m = std::clamp(latch.along_m + climb * k_climb_speed_m_s * tick.step_s, 0.0, axes->length_m);
    latch.side_m =
        std::clamp(latch.side_m + sideways * k_climb_speed_m_s * tick.step_s, -axes->side_m, axes->side_m);
    const Vector3 hand_m = axes->hand_m(latch.along_m, latch.side_m);
    const Vector3 wanted_m = axes->hang_m(latch.along_m, latch.side_m);

    // The arm: towards where it holds the body, relative to the hull it holds.
    const physics::BodyState& host = ladder.host_state;
    const Vector3 host_spin_rad_s = host.angular_velocity_rad_s;
    const Vector3 hull_m_s = host.velocity_m_s + math::cross(host_spin_rad_s, wanted_m - host.position_m);
    const Vector3 relative_m_s = tick.before.velocity_m_s - hull_m_s;
    const Vector3 pull_m_s2 = limited(k_ladder_stiffness_per_s2 * (wanted_m - tick.before.position_m)
                                          - k_ladder_damping_per_s * relative_m_s,
                                      k_ladder_strength_m_s2);
    hold.force_n = tick.mass_kg * (pull_m_s2 - tick.apparent_gravity_m_s2);

    // Facing the hull, head up the ladder: the turn that takes the body's axes there.
    const Vector3 facing = -1.0 * axes->out;
    const Vector3 lean =
        0.5
        * (math::cross(column(seat_to_world, 2), axes->along) + math::cross(column(seat_to_world, 0), facing)
           + math::cross(column(seat_to_world, 1), axes->across));
    const Vector3 turn_rad_s2 =
        limited(k_balance_stiffness_per_s2 * lean
                    - k_balance_damping_per_s * (tick.before.angular_velocity_rad_s - host_spin_rad_s),
                k_balance_strength_rad_s2);
    const Quaternion& orientation = tick.before.orientation;
    hold.torque_n_m = math::rotate(
        orientation, tick.inertia_kg_m2 * math::rotate(math::conjugate(orientation), turn_rad_s2));
    hold.grip_body = ladder.host;
    hold.grip_m = hand_m;
    hold.busy = true;
    return hold;
}

void Simulation::provide_suit(vessel::VesselSystems suit) {
    suit_ = std::move(suit);
}

std::optional<VesselId> Simulation::hatch_within_reach() const {
    if (!pilot_.has_value() || !pilot_->outside || pilot_->vessel.index >= vessels_.size()) {
        return std::nullopt;
    }
    const Vessel& suit = vessels_[pilot_->vessel.index];
    if (suit.status != VesselStatus::Flying && suit.status != VesselStatus::Landed) {
        return std::nullopt;
    }
    for (std::uint32_t index = 0; index < vessels_.size(); ++index) {
        const Vessel& vessel = vessels_[index];
        if (index == pilot_->vessel.index || vessel.state.domain != suit.state.domain) {
            continue;
        }
        const std::optional<Hatch> hatch = hatch_in(vessel);
        if (!hatch.has_value()) {
            continue;
        }
        const std::optional<Vector3> outside_m = place_about_domain(vessel, hatch->outside_m, now_);
        if (outside_m.has_value()
            && math::norm(*outside_m - suit.state.state_in_domain.position_m) <= k_hatch_reach_m) {
            return VesselId{index};
        }
    }
    return std::nullopt;
}

core::VoidResult Simulation::go_outside() {
    if (!pilot_.has_value() || pilot_->seated || pilot_->outside || passage_.has_value()
        || cabin_ == nullptr) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not moving about a cabin");
    }
    if (!suit_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is no suit to go outside in");
    }
    const std::optional<Hatch> hatch = hatch_in(vessels_[pilot_->vessel.index]);
    if (!hatch.has_value() || math::norm(pilot_->position_m - hatch->inside_m) > k_airlock_reach_m) {
        return core::fail(ErrorCode::InvalidArgument, "there is no airlock within reach");
    }
    const auto ends = now_.advanced_by(hatch->cycle_s);
    if (!ends) {
        return std::unexpected(ends.error());
    }
    if (pilot_->held_item.has_value()) {
        if (core::VoidResult released = release_item(0.0); !released) {
            return released;
        }
    }
    // In the airlock the pilot is out of the cabin's way: no body to bump into, nothing to hold.
    if (const std::optional<physics::BodyId> body = cabin_->body; body.has_value()) {
        if (core::VoidResult removed = cabin_->world.remove(*body); !removed) {
            return removed;
        }
        cabin_->body.reset();
    }
    cabin_->grab_held = false;
    cabin_->jump_held = false;
    cabin_->item_in_reach.reset();
    pilot_->grabbing = false;
    pilot_->standing = false;
    pilot_->in_reach = false;
    pilot_->position_m = hatch->inside_m;
    pilot_->velocity_m_s = {};
    pilot_->airlock_cycle_s = hatch->cycle_s;
    pilot_->airlock_left_s = hatch->cycle_s;
    pilot_->airlock_outwards = true;
    passage_ = Passage{.outwards = true, .ends = *ends};
    update_offer();
    return {};
}

core::VoidResult Simulation::come_inside() {
    if (!pilot_.has_value() || !pilot_->outside || passage_.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot is not outside");
    }
    const std::optional<VesselId> host = hatch_within_reach();
    const std::optional<Hatch> hatch = host.has_value() ? hatch_in(vessels_[host->index]) : std::nullopt;
    if (!host.has_value() || !hatch.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "there is no airlock within reach");
    }
    const auto ends = now_.advanced_by(hatch->cycle_s);
    if (!ends) {
        return std::unexpected(ends.error());
    }
    ladder_latch_.reset();
    // The suit is put away: it is no longer a body, nor anywhere in the world.
    const VesselId suit = pilot_->vessel;
    if (bubble_ != nullptr) {
        if (const BubbleMember* member = bubble_->find(suit.index); bubble_->anchor == suit.index) {
            if (core::VoidResult left = leave_bubble(); !left) {
                return left;
            }
        } else if (member != nullptr) {
            if (core::VoidResult released =
                    release_from_bubble(suit.index, std::max(bubble_->epoch, member->since));
                !released) {
                return released;
            }
        }
    }
    Vessel& stowed = vessels_[suit.index];
    // Nobody is in the suit now: it breathes no more while it is put away (D33).
    if (std::optional<vessel::VesselSystems>& systems = stowed.systems; systems.has_value()) {
        if (core::VoidResult breathing = set_worn(*systems, false, now_); !breathing) {
            return breathing;
        }
    }
    stowed.status = VesselStatus::Stowed;
    stowed.landed.reset();
    stowed.events.clear();
    stowed.prediction.clear();
    suit_vessel_ = suit;
    if (active_vessel_ == suit) {
        active_vessel_ = host;
    }
    LOG_INFO("the pilot comes inside '{}'", vessels_[host->index].name).tag("subsystem", "sim");
    // Into the cabin by the airlock, with nothing to do there until it has cycled.
    if (core::VoidResult boarded = board(*host); !boarded) {
        return boarded;
    }
    if (core::VoidResult left = leave_seat(); !left) {
        return left;
    }
    if (const std::optional<physics::BodyId> body = cabin_ != nullptr ? cabin_->body : std::nullopt;
        body.has_value()) {
        if (core::VoidResult removed = cabin_->world.remove(*body); !removed) {
            return removed;
        }
        cabin_->body.reset();
    }
    pilot_->position_m = hatch->inside_m;
    pilot_->airlock_cycle_s = hatch->cycle_s;
    pilot_->airlock_left_s = hatch->cycle_s;
    // What was asked of the legs outside was in the ground's axes; in here it means nothing.
    pilot_input_ = PilotInput{.view = pilot_->view};
    legs_ = {};
    passage_ = Passage{.outwards = false, .ends = *ends};
    update_offer();
    return {};
}

// The airlock has cycled: outwards, the pilot is the suit from here on, a vessel just clear of
// the hull and moving with it; inwards, the pilot is free to move about the cabin.
core::VoidResult Simulation::finish_passage() {
    if (!passage_.has_value() || !pilot_.has_value()) {
        return {};
    }
    const bool outwards = passage_->outwards;
    passage_.reset();
    pilot_->airlock_left_s = 0.0;
    pilot_->airlock_cycle_s = 0.0;
    if (!outwards) {
        if (cabin_ == nullptr) {
            return {};
        }
        const auto body = cabin_->world.add_body(pilot_body(*pilot_, k_body_mass_kg));
        if (!body) {
            return std::unexpected(body.error());
        }
        cabin_->body = *body;
        update_offer();
        return {};
    }

    const VesselId host_id = pilot_->vessel;
    // Rationale: copies, because adding the suit to the vessels may move them all.
    const std::optional<Hatch> hatch = hatch_in(vessels_[host_id.index]);
    const std::optional<Vector3> place_m =
        hatch.has_value() ? place_about_domain(vessels_[host_id.index], hatch->outside_m, now_)
                          : std::nullopt;
    if (!hatch.has_value() || !place_m.has_value() || !suit_.has_value()) {
        seat_pilot(); // the airlock is gone: nowhere to go
        return {};
    }
    const std::string host_name = vessels_[host_id.index].name;
    const Attitude attitude = vessels_[host_id.index].attitude;
    const dynamics::VesselState host_state = vessels_[host_id.index].state;
    const Vector3 arm_m = *place_m - host_state.state_in_domain.position_m;
    // The suit worn before, as it was put away, or a new one.
    vessel::VesselSystems systems{*suit_};
    bool worn_before = false;
    if (suit_vessel_.has_value() && vessels_[suit_vessel_->index].status == VesselStatus::Stowed) {
        if (std::optional<vessel::VesselSystems>& kept = vessels_[suit_vessel_->index].systems;
            kept.has_value()) {
            systems = std::move(*kept);
            worn_before = true;
        }
    }
    std::vector<vessel::Separation> separations;
    // Nobody asked the suit to point anywhere: the wearer turns it.
    if (core::VoidResult free =
            systems.command(vessel::k_signal_attitude_hold, vessel::ControlSource::Sequencer, 0.0,
                            std::max(now_, systems.propulsion().epoch), separations);
        !free) {
        return free;
    }
    // The pilot is about to be in the suit: it breathes from now on (D33).
    if (core::VoidResult breathing = set_worn(systems, true, now_); !breathing) {
        return breathing;
    }
    const auto added = add_vessel(std::string{k_suit_name},
                                  {.position_m = *place_m,
                                   .velocity_m_s = host_state.state_in_domain.velocity_m_s
                                                   + math::cross(attitude.angular_velocity_rad_s, arm_m)},
                                  host_state.domain, std::move(systems));
    if (!added) {
        seat_pilot();
        return std::unexpected(added.error());
    }
    VesselId suit = *added;
    if (worn_before) {
        vessels_[suit_vessel_->index] = std::move(vessels_.back());
        vessels_.pop_back();
        suit = *suit_vessel_;
    }
    suit_vessel_ = suit;

    // Facing away from the hull, head where it was in the cabin. The suit's axes: +x towards
    // the head, +z where the wearer faces.
    const Vector3 outward =
        (hatch->outside_m - hatch->inside_m) / math::norm(hatch->outside_m - hatch->inside_m);
    Vector3 head = hatch->head - math::dot(hatch->head, outward) * outward;
    if (math::norm(head) < 1e-6) {
        head =
            math::cross(outward, std::abs(outward.x) < 0.9 ? Vector3{1.0, 0.0, 0.0} : Vector3{0.0, 1.0, 0.0});
    }
    head = head / math::norm(head);
    const Vector3 side = math::cross(outward, head);
    const Quaternion suit_to_host = math::from_matrix(
        Matrix3{{head.x, side.x, outward.x, head.y, side.y, outward.y, head.z, side.z, outward.z}});
    Vessel& worn = vessels_[suit.index];
    worn.attitude = {.orientation = math::normalized(attitude.orientation * suit_to_host),
                     .angular_velocity_rad_s = attitude.angular_velocity_rad_s};
    const std::optional<Seat> helmet = seat_in(worn);
    if (!helmet.has_value()) {
        worn.status = VesselStatus::Stowed;
        seat_pilot();
        return core::fail(ErrorCode::InvalidArgument, "the suit has nobody's place in it: no crewed part");
    }
    // The pilot's body is what the player flies now.
    if (active_vessel_ == host_id) {
        active_vessel_ = suit;
    }
    cabin_.reset();
    cabin_items_.clear();
    cabin_reaction_.reset();
    pilot_ = Pilot{.vessel = suit,
                   .seated = true,
                   .position_m = seated_position_m(*helmet),
                   .view = math::from_matrix(helmet->seat_to_vessel),
                   .outside = true};
    // What was asked of the pilot in the cabin was in the cabin's axes; out here it means nothing.
    pilot_input_ = {};
    legs_ = {};
    LOG_INFO("the pilot goes outside '{}'", host_name).tag("subsystem", "sim");
    update_offer();
    return {};
}

// The cabin's walls, from the cockpits of the vessel's parts, with the loose items and (out of
// the seat) the pilot's body where they are.
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
    Cabin cabin{.world = std::move(*world), .part_count = parts.size(), .epoch = now_};
    for (const CabinItem& item : cabin_items_) {
        if (item.held) {
            cabin.items.emplace_back();
            continue;
        }
        const auto body = cabin.world.add_body(item_body(item));
        if (!body) {
            return std::unexpected(body.error());
        }
        cabin.items.emplace_back(*body);
    }
    if (!pilot_->seated && !passage_.has_value()) {
        const auto body = cabin.world.add_body(pilot_body(*pilot_, carried_mass_kg(*pilot_, cabin_items_)));
        if (!body) {
            return std::unexpected(body.error());
        }
        cabin.body = *body;
    }
    cabin_ = std::make_unique<Cabin>(std::move(cabin));
    cabin_reaction_.reset();
    return {};
}

core::VoidResult Simulation::advance_pilot(const time::Epoch& instant) {
    if (!pilot_.has_value()) {
        return {};
    }
    if (pilot_->vessel.index >= vessels_.size()) {
        pilot_.reset();
        cabin_.reset();
        cabin_items_.clear();
        passage_.reset();
        return {};
    }
    if (passage_.has_value()) {
        pilot_->airlock_left_s = std::max(0.0, time::seconds_between(instant, passage_->ends));
        if (instant >= passage_->ends) {
            if (core::VoidResult finished = finish_passage(); !finished) {
                return finished;
            }
            if (!pilot_.has_value()) {
                return {};
            }
        }
    }
    if (pilot_->outside) {
        // The suit is a vessel like any other; in the bubble near the ground its legs are part
        // of the tick (suit_legs). Here: what the pilot is doing when the suit is in no tick.
        Pilot& pilot = *pilot_;
        const Vessel& suit = vessels_[pilot.vessel.index];
        if (suit.status == VesselStatus::Landed) {
            // Standing still, pinned to the ground: on foot, if there is weight to stand with.
            const double weight = math::norm(suit.proper_acceleration_m_s2);
            const bool was_seated = pilot.seated;
            pilot.seated = weight < k_weight_threshold_m_s2;
            pilot.standing = !pilot.seated;
            pilot.up = pilot.seated ? Vector3{}
                                    : to_ground_axes(*catalog_, suit.state.domain, instant)
                                          * (suit.proper_acceleration_m_s2 / weight);
            if (was_seated && !pilot.seated) {
                face_as_suit(suit); // getting to its feet: looking where the suit faces
            }
            if (!pilot.seated) {
                pilot.view = math::normalized(pilot_input_.view);
            }
            if (!pilot_input_.jump) {
                legs_.jump_held = false;
            }
        } else if (!in_bubble(pilot.vessel)) {
            pilot.seated = true; // on rails: nothing is touched
            pilot.standing = false;
            pilot.up = {};
            ladder_latch_.reset();
        }
        if (ladder_latch_.has_value() && !held_ladder().has_value()) {
            ladder_latch_.reset(); // what it hung on is gone
        }
        update_offer();
        return {};
    }
    const Vessel& vessel = vessels_[pilot_->vessel.index];
    if (vessel.status == VesselStatus::Crashed || !vessel.systems.has_value()) {
        if (!pilot_->seated) {
            seat_pilot(); // nowhere left to move about
        }
        return {};
    }
    if (cabin_ == nullptr) {
        return {}; // a vessel without a cabin: the pilot sits
    }
    // The vessel lost or gained parts: the cabin is built again around what is in it.
    if (cabin_->part_count != vessel.systems->assembly().parts().size()) {
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
    // Rationale: above physics warp, or after a stall, everything in the cabin is taken to stay
    // put relative to the vessel; a cabin is not something to integrate over hours.
    if (effective_warp_ > options_.max_physics_warp
        || behind_s > step_s * options_.max_cabin_ticks_per_frame) {
        cabin_->epoch = instant;
        cabin_->grab_held = false;
        pilot_->grabbing = false;
        cabin_reaction_.reset();
        return {};
    }
    cabin_->impulse_n_s = {};
    cabin_->turning_impulse_n_m_s = {};
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
            return {}; // the pilot is gone
        }
        cabin_->epoch = *tick_end;
    }
    // Rationale: a frame without a cabin tick keeps the reaction of the one before; the vessel's
    // own ticks do not line up with the frames either.
    if (cabin_->ticked_s > 0.0) {
        cabin_reaction_ =
            CabinReaction{.force_n = -1.0 * cabin_->impulse_n_s / cabin_->ticked_s,
                          .torque_n_m = -1.0 * cabin_->turning_impulse_n_m_s / cabin_->ticked_s};
    }
    update_offer();
    return {};
}

core::VoidResult Simulation::cabin_tick(double step_s) {
    if (!pilot_.has_value() || cabin_ == nullptr) {
        return core::fail(ErrorCode::InvalidArgument, "there is no pilot aboard a cabin");
    }
    Pilot& pilot = *pilot_;
    Cabin& cabin = *cabin_;
    const Vessel& vessel = vessels_[pilot.vessel.index];
    if (!vessel.systems.has_value()) {
        return core::fail(ErrorCode::InvalidArgument, "the pilot's vessel has no cabin");
    }
    const vessel::VesselSystems& systems = *vessel.systems;
    physics::World& world = cabin.world;
    const auto mass = systems.mass_properties(now_);
    if (!mass) {
        return std::unexpected(mass.error());
    }

    // The vessel's frame: what it feels is the cabin's gravity, and its turning the rest.
    const Quaternion to_vessel = math::conjugate(vessel.attitude.orientation);
    const Vector3 weight_m_s2 = -1.0 * math::rotate(to_vessel, vessel.proper_acceleration_m_s2);
    const Vector3 spin_rad_s = math::rotate(to_vessel, vessel.attitude.angular_velocity_rad_s);
    const Vector3 spin_rate_rad_s2 = cabin.has_spin ? (spin_rad_s - cabin.spin_rad_s) / step_s : Vector3{};
    cabin.spin_rad_s = spin_rad_s;
    cabin.has_spin = true;
    // What the frame alone does to a free body in it; everything else is a push from the vessel.
    const auto frame_m_s2 = [&](const physics::BodyState& state) {
        const Vector3 arm_m = state.position_m - mass->centre_of_mass_m;
        return -1.0 * math::cross(spin_rad_s, math::cross(spin_rad_s, arm_m))
               - 2.0 * math::cross(spin_rad_s, state.velocity_m_s) - math::cross(spin_rate_rad_s2, arm_m);
    };
    const double weight = math::norm(weight_m_s2);
    cabin.heavy_s = weight >= k_weight_threshold_m_s2 ? cabin.heavy_s + step_s : 0.0;
    const bool heavy = cabin.heavy_s >= k_weight_settle_s;
    const Vector3 up = heavy ? weight_m_s2 * (-1.0 / weight) : Vector3{};

    // Everything that moves, as it is before the tick.
    struct Moving {
        physics::BodyId body;
        double mass_kg = 0.0;
        physics::BodyState before;
        Vector3 frame_m_s2;
    };
    std::vector<Moving> moving;
    moving.reserve(cabin.items.size() + 1);
    for (std::size_t index = 0; index < cabin.items.size(); ++index) {
        const std::optional<physics::BodyId> loose = cabin.items[index];
        if (!loose.has_value()) {
            continue;
        }
        const auto state = world.state(*loose);
        if (!state) {
            return std::unexpected(state.error());
        }
        const Moving item{.body = *loose,
                          .mass_kg = cabin_items_[index].mass_kg,
                          .before = *state,
                          .frame_m_s2 = frame_m_s2(*state)};
        if (core::VoidResult pushed =
                world.add_force(item.body, item.mass_kg * item.frame_m_s2, state->position_m);
            !pushed) {
            return pushed;
        }
        moving.push_back(item);
    }

    const Quaternion view = pilot.seated ? pilot.view : math::normalized(pilot_input_.view);
    bool surface_in_reach = false;
    const std::optional<physics::BodyId> own = cabin.body;
    if (own.has_value()) {
        const physics::BodyId body = *own;
        const auto before = world.state(body);
        if (!before) {
            return std::unexpected(before.error());
        }
        const Vector3& position_m = before->position_m;
        const Vector3& velocity_m_s = before->velocity_m_s;
        const double body_mass_kg = carried_mass_kg(pilot, cabin_items_);
        const Vector3 own_frame_m_s2 = frame_m_s2(*before);
        Vector3 acceleration_m_s2 = own_frame_m_s2;

        const PilotInput& input = pilot_input_;
        const Vector3 forward = math::rotate(view, {1.0, 0.0, 0.0});
        const Vector3 move{std::clamp(input.move.x, -1.0, 1.0), std::clamp(input.move.y, -1.0, 1.0),
                           std::clamp(input.move.z, -1.0, 1.0)};

        // The hand: what is in reach along the line of sight is a loose item to pick up, or a
        // surface to hold on to.
        const Vector3 eye_m = position_m + k_pilot_eye_height_m * math::rotate(view, {0.0, 0.0, 1.0});
        const auto within_reach = world.cast_ray(eye_m, forward, k_item_reach_m, body);
        cabin.item_in_reach.reset();
        if (within_reach.has_value()) {
            const auto item = std::ranges::find(cabin.items, std::optional{within_reach->body});
            if (item != cabin.items.end()) {
                cabin.item_in_reach = static_cast<std::size_t>(item - cabin.items.begin());
            }
        }
        surface_in_reach = within_reach.has_value() && !cabin.item_in_reach.has_value()
                           && within_reach->distance_m <= k_reach_m;
        const bool grab_pressed = input.grab && !cabin.grab_held;
        cabin.grab_held = input.grab;
        if (!input.grab) {
            pilot.grabbing = false;
        } else if (grab_pressed && surface_in_reach && within_reach.has_value()) {
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
            // An arm pushed out straight has pushed off: the hand lets go, and the body keeps
            // the speed the push gave it. (Holding on at arm's length without pushing keeps
            // the grip.)
            if (length_m > k_arm_length_m && math::dot(wanted, offset_m) > 0.5 * k_arm_length_m) {
                pilot.grabbing = false;
            }
        }
        if (pilot.grabbing) {
            acceleration_m_s2 =
                acceleration_m_s2
                + limited(k_grip_stiffness_per_s2 * (pilot.grip_m + cabin.grip_offset_m - position_m)
                              - k_grip_damping_per_s * velocity_m_s,
                          k_grip_strength_m_s2);
        }

        // The legs.
        const bool jump_pressed = input.jump && !cabin.jump_held;
        cabin.jump_held = input.jump;
        pilot.standing = false;
        if (heavy) {
            const auto floor = world.cast_ray(position_m, -1.0 * up, k_leg_length_m + k_leg_slack_m, body);
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
                    // Walking: towards the wanted speed over the floor, no harder than the
                    // weight on the feet lets them push.
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
        if (core::VoidResult pushed = world.add_force(body, body_mass_kg * acceleration_m_s2, position_m);
            !pushed) {
            return pushed;
        }
        moving.push_back(
            Moving{.body = body, .mass_kg = body_mass_kg, .before = *before, .frame_m_s2 = own_frame_m_s2});
    }

    world.set_gravity(weight_m_s2);
    if (core::VoidResult stepped = world.step(step_s); !stepped) {
        return stepped;
    }

    // The push of the vessel on each body during the tick (legs, hand, walls and floor
    // together): what changed its velocity that was not the frame. Pushes between the bodies
    // themselves cancel in the sums.
    for (const Moving& body : moving) {
        const auto after = world.state(body.body);
        if (!after) {
            return std::unexpected(after.error());
        }
        const Vector3 pushed_n =
            body.mass_kg
            * ((after->velocity_m_s - body.before.velocity_m_s) / step_s - body.frame_m_s2 - weight_m_s2);
        cabin.impulse_n_s += step_s * pushed_n;
        cabin.turning_impulse_n_m_s += step_s * math::cross(body.before.position_m, pushed_n);
    }
    cabin.ticked_s += step_s;

    pilot.up = up;
    if (own.has_value()) {
        const auto after = world.state(*own);
        if (!after) {
            return std::unexpected(after.error());
        }
        pilot.position_m = after->position_m;
        pilot.velocity_m_s = after->velocity_m_s;
        pilot.view = view;
        pilot.in_reach = surface_in_reach;
    }
    for (std::size_t index = 0; index < cabin.items.size(); ++index) {
        CabinItem& item = cabin_items_[index];
        const std::optional<physics::BodyId> loose = cabin.items[index];
        if (!loose.has_value()) {
            // In hand: ahead of the eyes, going where the pilot goes.
            item.position_m = pilot.position_m + k_pilot_eye_height_m * math::rotate(view, {0.0, 0.0, 1.0})
                              + k_hold_distance_m * math::rotate(view, {1.0, 0.0, 0.0});
            item.orientation = view;
            item.velocity_m_s = pilot.velocity_m_s;
            continue;
        }
        const auto after = world.state(*loose);
        if (!after) {
            return std::unexpected(after.error());
        }
        item.position_m = after->position_m;
        item.orientation = after->orientation;
        item.velocity_m_s = after->velocity_m_s;
    }

    const auto seat = seat_of(systems.assembly());
    if (!pilot.seated
        && (!seat.has_value() || math::norm(pilot.position_m - seat->eye_m) > k_lost_distance_m)) {
        LOG_WARN("the pilot left the cabin of '{}' and is put back in the seat", vessel.name)
            .tag("subsystem", "sim");
        seat_pilot();
    }
    return {};
}

} // namespace helios::sim
