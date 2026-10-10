#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <utility>

namespace helios::vessel {

namespace {

using core::ErrorCode;
using math::Vector3;

// A store closer than this to empty or full (as a fraction of its capacity, or as the time its
// current rate needs to get there) is empty or full: the instant computed for the change and
// the amount recomputed at that instant differ by rounding.
constexpr double k_snap_fraction = 1e-9;
constexpr double k_snap_time_s = 1e-9;
// More changes than this at one call means the systems are not making progress.
constexpr int k_max_changes_per_call = 100'000;
constexpr double k_switch_threshold = 0.5;

// The thrust of a process at `level`, with the sign of the level.
[[nodiscard]] double thrust_at(const Process& process, double level) noexcept {
    const double magnitude = std::abs(level);
    const double fraction = process.thrust_curve.has_value() ? (*process.thrust_curve)(magnitude) : magnitude;
    return std::copysign(process.thrust_n * fraction, level);
}

[[nodiscard]] std::string port_signal_name(std::string_view owner, std::string_view port) {
    return std::format("{}/{}", owner, port);
}

} // namespace

core::Result<VesselSystems> VesselSystems::make(Assembly assembly,
                                                std::span<const InterfaceDefinition> interfaces,
                                                std::vector<Stage> stages, const time::Epoch& start) {
    return build(std::move(assembly), std::vector<InterfaceDefinition>(interfaces.begin(), interfaces.end()),
                 std::move(stages), start);
}

core::Result<VesselSystems> VesselSystems::build(Assembly assembly,
                                                 std::vector<InterfaceDefinition> interfaces,
                                                 std::vector<Stage> stages, const time::Epoch& start) {
    if (assembly.parts().empty()) {
        return core::fail(ErrorCode::InvalidArgument, "a vessel needs at least one part");
    }
    VesselSystems systems;
    systems.assembly_ = std::move(assembly);
    systems.interfaces_ = std::move(interfaces);
    systems.stages_ = std::move(stages);
    systems.segment_start_ = start;
    ControlBus& bus = systems.bus_;
    const auto parts = systems.assembly_.parts();

    // Signals of the vessel itself.
    const double stage_count = static_cast<double>(std::max<std::size_t>(systems.stages_.size(), 1));
    const auto stage =
        bus.add_command(std::string{k_signal_stage}, "", {.minimum = 0.0, .maximum = stage_count});
    const auto frame = bus.add_command(std::string{k_signal_pointing_frame}, "",
                                       {.minimum = 0.0, .maximum = 2.0, .default_value = 1.0});
    const auto pointing_x = bus.add_command(std::string{k_signal_pointing_x}, "",
                                            {.minimum = -1.0, .maximum = 1.0, .default_value = 1.0});
    const auto pointing_y =
        bus.add_command(std::string{k_signal_pointing_y}, "", {.minimum = -1.0, .maximum = 1.0});
    const auto pointing_z =
        bus.add_command(std::string{k_signal_pointing_z}, "", {.minimum = -1.0, .maximum = 1.0});
    const auto hold = bus.add_command(std::string{k_signal_attitude_hold}, "",
                                      {.minimum = 0.0, .maximum = 1.0, .default_value = 1.0, .toggle = true});
    const auto mass = bus.add_telemetry(std::string{k_signal_mass}, "kg");
    const auto thrust = bus.add_telemetry(std::string{k_signal_thrust}, "N");
    const auto mass_flow = bus.add_telemetry(std::string{k_signal_mass_flow}, "kg/s");
    const auto altitude = bus.add_telemetry(std::string{k_signal_altitude}, "m");
    const auto vertical_speed = bus.add_telemetry(std::string{k_signal_vertical_speed}, "m/s");
    const auto surface_speed = bus.add_telemetry(std::string{k_signal_surface_speed}, "m/s");
    const auto speed = bus.add_telemetry(std::string{k_signal_speed}, "m/s");
    for (const auto& signal : {stage, frame, pointing_x, pointing_y, pointing_z, hold, mass, thrust,
                               mass_flow, altitude, vertical_speed, surface_speed, speed}) {
        if (!signal) {
            return std::unexpected(signal.error());
        }
    }
    systems.stage_signal_ = *stage;
    systems.pointing_frame_signal_ = *frame;
    systems.pointing_x_signal_ = *pointing_x;
    systems.pointing_y_signal_ = *pointing_y;
    systems.pointing_z_signal_ = *pointing_z;
    systems.attitude_hold_signal_ = *hold;
    systems.mass_signal_ = *mass;
    systems.thrust_signal_ = *thrust;
    systems.mass_flow_signal_ = *mass_flow;
    systems.altitude_signal_ = *altitude;
    systems.vertical_speed_signal_ = *vertical_speed;
    systems.surface_speed_signal_ = *surface_speed;
    systems.speed_signal_ = *speed;

    // Resources flow through an attachment unless the attached part says otherwise.
    std::vector<std::size_t> feed_groups(parts.size(), 0);
    std::size_t feed_group_count = 1;
    for (std::size_t index = 1; index < parts.size(); ++index) {
        feed_groups[index] =
            parts[index].datasheet->crossfeed ? feed_groups[*parts[index].parent] : feed_group_count++;
    }
    const auto pool_of = [&](std::size_t part, std::string_view resource) -> std::optional<std::size_t> {
        const auto match = std::ranges::find_if(systems.pools_, [&](const Pool& pool) {
            return pool.feed_group == feed_groups[part] && pool.resource == resource;
        });
        if (match == systems.pools_.end()) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(match - systems.pools_.begin());
    };

    std::vector<std::vector<SignalId>> input_signals(parts.size());
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const PartInstance& part = parts[index];
        const PartDatasheet& datasheet = *part.datasheet;
        for (const InputPort& input : datasheet.inputs) {
            const auto signal = bus.add_command(port_signal_name(part.name, input.name), input.unit,
                                                {.minimum = input.minimum,
                                                 .maximum = input.maximum,
                                                 .default_value = input.default_value,
                                                 .toggle = input.toggle});
            if (!signal) {
                return std::unexpected(signal.error());
            }
            input_signals[index].push_back(*signal);
        }
        for (std::size_t output = 0; output < datasheet.outputs.size(); ++output) {
            const auto signal = bus.add_telemetry(port_signal_name(part.name, datasheet.outputs[output].name),
                                                  datasheet.outputs[output].unit);
            if (!signal) {
                return std::unexpected(signal.error());
            }
            systems.outputs_.push_back(OutputBinding{.part = index, .output = output, .signal = *signal});
        }
        for (std::size_t store = 0; store < datasheet.stores.size(); ++store) {
            const Store& definition = datasheet.stores[store];
            std::optional<std::size_t> pool = pool_of(index, definition.resource);
            if (!pool.has_value()) {
                systems.pools_.push_back(
                    Pool{.resource = definition.resource, .feed_group = feed_groups[index], .stores = {}});
                pool = systems.pools_.size() - 1;
            }
            systems.pools_[*pool].stores.push_back(systems.stores_.size());
            systems.stores_.push_back(StoreState{.part = index,
                                                 .store = store,
                                                 .pool = *pool,
                                                 .base_amount = definition.initial_amount,
                                                 .rate_per_s = 0.0});
        }
        if (datasheet.separator.has_value() && index != 0) {
            systems.separators_.push_back(
                SeparatorBinding{.part = index, .signal = input_signals[index][datasheet.separator->input]});
        }
    }
    // Processes are bound once every store exists: a process may be fed by a part added later.
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const PartDatasheet& datasheet = *parts[index].datasheet;
        for (std::size_t process = 0; process < datasheet.processes.size(); ++process) {
            const Process& definition = datasheet.processes[process];
            ProcessState state{.part = index,
                               .process = process,
                               .level_signal = {},
                               .enable_signal = {},
                               .consumed_pools = {},
                               .produced_pools = {},
                               .level = 0.0};
            if (definition.level_input.has_value()) {
                state.level_signal = input_signals[index][*definition.level_input];
            }
            if (definition.enable_input.has_value()) {
                state.enable_signal = input_signals[index][*definition.enable_input];
            }
            for (const ResourceFlow& flow : definition.consumes) {
                state.consumed_pools.push_back(pool_of(index, flow.resource));
            }
            for (const ResourceFlow& flow : definition.produces) {
                state.produced_pools.push_back(pool_of(index, flow.resource));
            }
            systems.processes_.push_back(std::move(state));
        }
    }

    // One command per input of each interface in use, which the members' inputs follow.
    for (const InterfaceDefinition& definition : systems.interfaces_) {
        for (const std::string& input_name : definition.inputs) {
            std::optional<SignalId> leader;
            for (std::size_t index = 0; index < parts.size(); ++index) {
                const PartDatasheet& datasheet = *parts[index].datasheet;
                const auto port = std::ranges::find(datasheet.inputs, input_name, &InputPort::name);
                if (std::ranges::find(datasheet.interfaces, definition.id) == datasheet.interfaces.end()
                    || port == datasheet.inputs.end()) {
                    continue;
                }
                if (!leader.has_value()) {
                    const auto added =
                        bus.add_command(port_signal_name(definition.id, input_name), port->unit,
                                        {.minimum = port->minimum,
                                         .maximum = port->maximum,
                                         .default_value = port->default_value,
                                         .toggle = port->toggle});
                    if (!added) {
                        return std::unexpected(added.error());
                    }
                    leader = *added;
                }
                const SignalId follower =
                    input_signals[index][static_cast<std::size_t>(port - datasheet.inputs.begin())];
                if (core::VoidResult linked = bus.link(*leader, follower); !linked) {
                    return std::unexpected(linked.error());
                }
            }
        }
    }
    systems.evaluate();
    return systems;
}

const Store& VesselSystems::store_definition(const StoreState& state) const noexcept {
    return assembly_.parts()[state.part].datasheet->stores[state.store];
}

double VesselSystems::store_amount(const StoreState& state, const time::Epoch& instant) const noexcept {
    const double elapsed_s = std::max(time::seconds_between(segment_start_, instant), 0.0);
    return std::clamp(state.base_amount + state.rate_per_s * elapsed_s, 0.0,
                      store_definition(state).capacity);
}

double VesselSystems::amount(std::size_t part, std::size_t store, const time::Epoch& instant) const noexcept {
    const auto match = std::ranges::find_if(
        stores_, [&](const StoreState& state) { return state.part == part && state.store == store; });
    return match == stores_.end() ? 0.0 : store_amount(*match, instant);
}

double VesselSystems::mass_kg(const time::Epoch& instant) const noexcept {
    double total_kg = 0.0;
    for (const PartInstance& part : assembly_.parts()) {
        total_kg += part.datasheet->dry_mass_kg;
    }
    for (const StoreState& state : stores_) {
        total_kg += store_amount(state, instant) * store_definition(state).mass_per_unit_kg;
    }
    return total_kg;
}

core::Result<MassProperties> VesselSystems::mass_properties(const time::Epoch& instant) const {
    std::vector<double> part_masses_kg;
    part_masses_kg.reserve(assembly_.parts().size());
    for (const PartInstance& part : assembly_.parts()) {
        part_masses_kg.push_back(part.datasheet->dry_mass_kg);
    }
    for (const StoreState& state : stores_) {
        part_masses_kg[state.part] += store_amount(state, instant) * store_definition(state).mass_per_unit_kg;
    }
    return assembly_.mass_properties(part_masses_kg);
}

std::vector<double> VesselSystems::command_values() const {
    std::vector<double> values;
    const auto signals = bus_.signals();
    values.reserve(signals.size());
    for (std::uint32_t index = 0; index < signals.size(); ++index) {
        if (signals[index].kind == Signal::Kind::Command) {
            values.push_back(bus_.value(SignalId{index}));
        }
    }
    return values;
}

Pointing VesselSystems::pointing() const noexcept {
    const Vector3 direction{bus_.value(pointing_x_signal_), bus_.value(pointing_y_signal_),
                            bus_.value(pointing_z_signal_)};
    const double length = math::norm(direction);
    const double frame = bus_.value(pointing_frame_signal_);
    Pointing::Frame kind = Pointing::Frame::Orbital;
    if (frame < k_switch_threshold) {
        kind = Pointing::Frame::Inertial;
    } else if (frame >= 1.0 + k_switch_threshold) {
        kind = Pointing::Frame::Local;
    }
    return Pointing{.frame = kind, .direction = length > 0.0 ? direction / length : Vector3{1.0, 0.0, 0.0}};
}

void VesselSystems::rebase(const time::Epoch& instant) noexcept {
    for (StoreState& state : stores_) {
        const double capacity = store_definition(state).capacity;
        const double margin =
            std::max(k_snap_fraction * capacity, std::abs(state.rate_per_s) * k_snap_time_s);
        double amount = store_amount(state, instant);
        if (amount <= margin) {
            amount = 0.0;
        } else if (amount >= capacity - margin) {
            amount = capacity;
        }
        state.base_amount = amount;
    }
    segment_start_ = instant;
}

// From the commands on the bus and the stores as they are at the latest change: the level of
// every process, the rate of every store, the propulsion state and the next store change.
void VesselSystems::evaluate() {
    const auto parts = assembly_.parts();
    const auto definition_of = [&](const ProcessState& state) -> const Process& {
        return parts[state.part].datasheet->processes[state.process];
    };

    std::vector<double> held(pools_.size(), 0.0);
    std::vector<double> room(pools_.size(), 0.0);
    for (const StoreState& state : stores_) {
        held[state.pool] += state.base_amount;
        room[state.pool] += store_definition(state).capacity - state.base_amount;
    }

    for (ProcessState& state : processes_) {
        const Process& definition = definition_of(state);
        const bool enabled =
            !state.enable_signal.has_value() || bus_.value(*state.enable_signal) >= k_switch_threshold;
        const double requested = state.level_signal.has_value() ? bus_.value(*state.level_signal) : 1.0;
        const bool fed = std::ranges::all_of(
            state.consumed_pools, [](const std::optional<std::size_t>& pool) { return pool.has_value(); });
        const double magnitude = std::abs(requested);
        state.level = enabled && fed && magnitude > 0.0
                          ? std::copysign(std::clamp(std::max(magnitude, definition.minimum_level), 0.0, 1.0),
                                          requested)
                          : 0.0;
    }

    // A process stops when a pool it draws from is empty and is not being refilled as fast as
    // it is drawn. Stopping one process can starve another, so repeat until nothing changes.
    std::vector<double> consumed(pools_.size());
    std::vector<double> produced(pools_.size());
    for (bool starved = true; starved;) {
        std::ranges::fill(consumed, 0.0);
        std::ranges::fill(produced, 0.0);
        for (const ProcessState& state : processes_) {
            if (state.level == 0.0) {
                continue;
            }
            const Process& definition = definition_of(state);
            const double rate_scale = std::abs(state.level);
            for (std::size_t flow = 0; flow < definition.consumes.size(); ++flow) {
                consumed[*state.consumed_pools[flow]] += rate_scale * definition.consumes[flow].rate_per_s;
            }
            for (std::size_t flow = 0; flow < definition.produces.size(); ++flow) {
                if (state.produced_pools[flow].has_value()) {
                    produced[*state.produced_pools[flow]] +=
                        rate_scale * definition.produces[flow].rate_per_s;
                }
            }
        }
        starved = false;
        for (ProcessState& state : processes_) {
            if (state.level != 0.0
                && std::ranges::any_of(state.consumed_pools, [&](const std::optional<std::size_t>& pool) {
                       return held[*pool] <= 0.0 && consumed[*pool] > produced[*pool];
                   })) {
                state.level = 0.0;
                starved = true;
            }
        }
    }

    // A pool that is drawn down empties its stores together (each in proportion to what it
    // holds); one that is filled fills them together (in proportion to the room left).
    double soonest_s = std::numeric_limits<double>::infinity();
    for (std::size_t pool = 0; pool < pools_.size(); ++pool) {
        const double net_per_s = produced[pool] - consumed[pool];
        for (const std::size_t index : pools_[pool].stores) {
            StoreState& state = stores_[index];
            if (net_per_s < 0.0 && held[pool] > 0.0) {
                state.rate_per_s = net_per_s * state.base_amount / held[pool];
            } else if (net_per_s > 0.0 && room[pool] > 0.0) {
                state.rate_per_s =
                    net_per_s * (store_definition(state).capacity - state.base_amount) / room[pool];
            } else {
                state.rate_per_s = 0.0;
            }
        }
        if (net_per_s < 0.0 && held[pool] > 0.0) {
            soonest_s = std::min(soonest_s, held[pool] / -net_per_s);
        } else if (net_per_s > 0.0 && room[pool] > 0.0) {
            soonest_s = std::min(soonest_s, room[pool] / net_per_s);
        }
    }
    store_change_.reset();
    if (std::isfinite(soonest_s)) {
        if (const auto change = segment_start_.advanced_by(soonest_s)) {
            store_change_ = *change;
        }
    }

    Vector3 force_n;
    for (const ProcessState& state : processes_) {
        const Process& definition = definition_of(state);
        if (state.level != 0.0 && definition.thrust_n > 0.0) {
            force_n += thrust_at(definition, state.level)
                       * (assembly_.poses()[state.part].orientation * definition.thrust_direction);
        }
    }
    double mass_rate_kg_s = 0.0;
    for (const StoreState& state : stores_) {
        mass_rate_kg_s += state.rate_per_s * store_definition(state).mass_per_unit_kg;
    }
    // Rationale: on rails only the thrust along the nose acts. The sideways part would turn the
    // vessel, which the rails do not model; real designs cancel it by symmetry.
    double dry_mass_kg = 0.0;
    for (const PartInstance& part : parts) {
        dry_mass_kg += part.datasheet->dry_mass_kg;
    }
    propulsion_ = PropulsionState{.epoch = segment_start_,
                                  .thrust_n = force_n.x,
                                  .mass_flow_kg_s = 0.0 - mass_rate_kg_s, // not a negative zero
                                  .mass_kg = mass_kg(segment_start_),
                                  .dry_mass_kg = dry_mass_kg,
                                  .pointing = pointing()};
}

std::vector<AppliedLoad> VesselSystems::loads() const {
    std::vector<AppliedLoad> loads;
    const auto parts = assembly_.parts();
    for (const ProcessState& state : processes_) {
        const Process& definition = parts[state.part].datasheet->processes[state.process];
        const double thrust_n = thrust_at(definition, state.level);
        const Vector3 torque_n_m = state.level * definition.torque_n_m;
        if (thrust_n == 0.0 && torque_n_m == Vector3{}) {
            continue;
        }
        const PartPose& pose = assembly_.poses()[state.part];
        loads.push_back(
            AppliedLoad{.force_n = thrust_n * (pose.orientation * definition.thrust_direction),
                        .position_m = pose.position_m + pose.orientation * definition.thrust_position_m,
                        .torque_n_m = pose.orientation * torque_n_m});
    }
    return loads;
}

math::Vector3 VesselSystems::torque_authority_n_m() const noexcept {
    Vector3 authority_n_m;
    const auto parts = assembly_.parts();
    for (const ProcessState& state : processes_) {
        const Vector3 torque_n_m = assembly_.poses()[state.part].orientation
                                   * parts[state.part].datasheet->processes[state.process].torque_n_m;
        authority_n_m += Vector3{std::abs(torque_n_m.x), std::abs(torque_n_m.y), std::abs(torque_n_m.z)};
    }
    return authority_n_m;
}

bool VesselSystems::attitude_hold() const noexcept {
    return bus_.value(attitude_hold_signal_) >= k_switch_threshold;
}

core::VoidResult VesselSystems::command(std::string_view signal, ControlSource source, double value,
                                        const time::Epoch& instant, std::vector<Separation>& separations) {
    return change_command(signal, source, value, instant, separations);
}

core::VoidResult VesselSystems::release(std::string_view signal, ControlSource source,
                                        const time::Epoch& instant, std::vector<Separation>& separations) {
    return change_command(signal, source, std::nullopt, instant, separations);
}

core::VoidResult VesselSystems::change_command(std::string_view signal, ControlSource source,
                                               std::optional<double> value, const time::Epoch& instant,
                                               std::vector<Separation>& separations) {
    if (instant < segment_start_) {
        return core::fail(ErrorCode::OutOfRange, "a command cannot precede the vessel's latest change");
    }
    if (core::VoidResult advanced = advance_to(instant, separations); !advanced) {
        return advanced;
    }
    // After advancing: a separation on the way may have taken the signal with it.
    const auto id = bus_.find(signal);
    if (!id) {
        return std::unexpected(id.error());
    }
    const std::vector<double> before = command_values();
    if (core::VoidResult changed =
            value.has_value() ? bus_.publish(*id, source, *value) : bus_.release(*id, source);
        !changed) {
        return changed;
    }
    // Rationale: a command that leaves every value as it was (a held key repeating itself) must
    // not start a new closed-form segment, or the stores would depend on how often it repeats.
    if (command_values() == before) {
        return {};
    }
    rebase(instant);
    return settle_with_separated(instant, separations);
}

core::VoidResult VesselSystems::schedule(TimedCommand command) {
    if (command.epoch < segment_start_) {
        return core::fail(ErrorCode::OutOfRange, "a command cannot be scheduled before the latest change");
    }
    if (!std::isfinite(command.value)) {
        return core::fail(ErrorCode::NotFinite, "a scheduled command needs a finite value");
    }
    const auto position =
        std::ranges::upper_bound(program_, command.epoch, std::less{}, &TimedCommand::epoch);
    program_.insert(position, std::move(command));
    return {};
}

std::optional<time::Epoch> VesselSystems::next_change() const noexcept {
    if (program_.empty()) {
        return store_change_;
    }
    return store_change_.has_value() ? std::min(*store_change_, program_.front().epoch)
                                     : program_.front().epoch;
}

core::VoidResult VesselSystems::advance_to(const time::Epoch& instant, std::vector<Separation>& separations) {
    for (int changes = 0;; ++changes) {
        const std::optional<time::Epoch> next = next_change();
        if (!next.has_value() || *next > instant) {
            return {};
        }
        if (changes >= k_max_changes_per_call) {
            return core::fail(ErrorCode::Timeout, "the vessel's systems keep changing without advancing");
        }
        if (core::VoidResult applied = apply_change(*next, separations); !applied) {
            return applied;
        }
    }
}

core::VoidResult VesselSystems::apply_change(const time::Epoch& instant,
                                             std::vector<Separation>& separations) {
    rebase(instant);
    while (!program_.empty() && program_.front().epoch <= instant) {
        const TimedCommand command = std::move(program_.front());
        program_.erase(program_.begin());
        // A signal that left with a dropped stage is no longer there to command.
        if (const auto id = bus_.find(command.signal)) {
            if (core::VoidResult published = bus_.publish(*id, command.source, command.value); !published) {
                return published;
            }
        }
    }
    return settle_with_separated(instant, separations);
}

// Brings this vessel and every vessel that separates from it (and from those) into a
// consistent state at `instant`.
core::VoidResult VesselSystems::settle_with_separated(const time::Epoch& instant,
                                                      std::vector<Separation>& separations) {
    std::size_t next_unsettled = separations.size();
    if (core::VoidResult settled = settle(instant, separations); !settled) {
        return settled;
    }
    // Rationale: by index, not by reference, because settling appends to the vector.
    for (; next_unsettled < separations.size(); ++next_unsettled) {
        VesselSystems separated = std::move(separations[next_unsettled].systems);
        const core::VoidResult settled = separated.settle(instant, separations);
        separations[next_unsettled].systems = std::move(separated);
        if (!settled) {
            return settled;
        }
    }
    return {};
}

// Acts on the commands as they are now: activates stages, fires separators, then re-evaluates.
core::VoidResult VesselSystems::settle(const time::Epoch& instant, std::vector<Separation>& separations) {
    while (stages_activated_ < stages_.size()
           && bus_.value(stage_signal_) >= static_cast<double>(stages_activated_ + 1)) {
        for (const StageCommand& command : stages_[stages_activated_].commands) {
            if (const auto id = bus_.find(command.signal)) {
                if (core::VoidResult published = bus_.publish(*id, ControlSource::Sequencer, command.value);
                    !published) {
                    return published;
                }
            }
        }
        ++stages_activated_;
    }
    for (bool fired = true; fired;) {
        fired = false;
        for (const SeparatorBinding& separator : separators_) {
            if (bus_.value(separator.signal) >= k_switch_threshold) {
                // Detaching rebuilds this vessel, so the list of separators starts over.
                if (core::VoidResult detached = detach(separator.part, instant, separations); !detached) {
                    return detached;
                }
                fired = true;
                break;
            }
        }
    }
    evaluate();
    return {};
}

core::VoidResult VesselSystems::detach(std::size_t part, const time::Epoch& instant,
                                       std::vector<Separation>& separations) {
    auto split = assembly_.split_at(part);
    if (!split) {
        return std::unexpected(split.error());
    }
    const auto before = mass_properties(instant);
    if (!before) {
        return std::unexpected(before.error());
    }
    const std::optional<Separator>& separator = assembly_.parts()[part].datasheet->separator;
    const double impulse_n_s = separator.has_value() ? separator->impulse_n_s : 0.0;
    // The parts that leave keep the axes of the detached part.
    const PartPose detached = assembly_.poses()[part];
    auto kept = build(std::move(split->kept), interfaces_, stages_, instant);
    auto separated = build(std::move(split->separated), interfaces_, {}, instant);
    if (!kept || !separated) {
        return std::unexpected(!kept ? kept.error() : separated.error());
    }
    // Each new vessel takes over the contents of its stores and the commands of its signals.
    const auto take_over = [&](VesselSystems& target, const std::vector<std::size_t>& original_parts) {
        for (StoreState& state : target.stores_) {
            const auto original = std::ranges::find_if(stores_, [&](const StoreState& candidate) {
                return candidate.part == original_parts[state.part] && candidate.store == state.store;
            });
            state.base_amount = original->base_amount;
        }
        target.bus_.adopt_commands(bus_);
    };
    take_over(*kept, split->kept_parts);
    take_over(*separated, split->separated_parts);
    kept->stages_activated_ = stages_activated_;
    kept->program_ = std::move(program_);

    // The separator pushes the two sides apart along the nose, each by impulse / mass.
    const double kept_mass_kg = kept->mass_kg(instant);
    const double separated_mass_kg = separated->mass_kg(instant);
    const auto kept_properties = kept->mass_properties(instant);
    const auto separated_properties = separated->mass_properties(instant);
    if (!kept_properties || !separated_properties) {
        return std::unexpected(!kept_properties ? kept_properties.error() : separated_properties.error());
    }
    // The kept side is ahead if its centre of mass is ahead of the whole vessel's.
    const double side = kept_properties->centre_of_mass_m.x >= before->centre_of_mass_m.x ? 1.0 : -1.0;
    separations.push_back(
        Separation{.epoch = instant,
                   .systems = std::move(*separated),
                   .pointing = pointing(),
                   .kept_delta_v_m_s = side * impulse_n_s / kept_mass_kg,
                   .separated_delta_v_m_s = -side * impulse_n_s / separated_mass_kg,
                   .kept_offset_m = kept_properties->centre_of_mass_m - before->centre_of_mass_m,
                   .separated_offset_m = detached.position_m
                                         + detached.orientation * separated_properties->centre_of_mass_m
                                         - before->centre_of_mass_m,
                   .separated_orientation = math::from_matrix(detached.orientation)});
    *this = std::move(*kept);
    return {};
}

core::Result<std::vector<PropulsionState>> VesselSystems::forecast(std::size_t max_states) const {
    std::vector<PropulsionState> states{propulsion_};
    VesselSystems ahead = *this;
    std::vector<Separation> ignored;
    for (int changes = 0; states.size() < max_states; ++changes) {
        const std::optional<time::Epoch> next = ahead.next_change();
        if (!next.has_value()) {
            break;
        }
        if (changes >= k_max_changes_per_call) {
            return core::fail(ErrorCode::Timeout, "the vessel's systems keep changing without advancing");
        }
        if (core::VoidResult applied = ahead.apply_change(*next, ignored); !applied) {
            return std::unexpected(applied.error());
        }
        ignored.clear();
        if (states.back().thrust_n != 0.0 || ahead.propulsion_.thrust_n != 0.0) {
            states.push_back(ahead.propulsion_);
        }
    }
    return states;
}

core::VoidResult VesselSystems::report_telemetry(const time::Epoch& instant) {
    const auto parts = assembly_.parts();
    for (const OutputBinding& binding : outputs_) {
        const PartDatasheet& datasheet = *parts[binding.part].datasheet;
        const OutputPort& output = datasheet.outputs[binding.output];
        double value = 0.0;
        if (output.quantity == OutputPort::Quantity::Amount
            || output.quantity == OutputPort::Quantity::Fraction) {
            value = amount(binding.part, output.index, instant);
            if (output.quantity == OutputPort::Quantity::Fraction) {
                value /= datasheet.stores[output.index].capacity;
            }
        } else {
            const auto state = std::ranges::find_if(processes_, [&](const ProcessState& candidate) {
                return candidate.part == binding.part && candidate.process == output.index;
            });
            const Process& definition = datasheet.processes[output.index];
            const double level = state->level;
            switch (output.quantity) {
            case OutputPort::Quantity::Thrust: value = thrust_at(definition, level); break;
            case OutputPort::Quantity::MassFlow:
                for (const ResourceFlow& flow : definition.consumes) {
                    value += std::abs(level) * flow.rate_per_s * flow.mass_per_unit_kg;
                }
                break;
            case OutputPort::Quantity::Curve: value = (*output.curve)(std::abs(level)); break;
            default:                          value = level; break;
            }
        }
        if (core::VoidResult reported = bus_.report(binding.signal, value); !reported) {
            return reported;
        }
    }
    if (const auto error = core::first_error(bus_.report(mass_signal_, mass_kg(instant)),
                                             bus_.report(thrust_signal_, propulsion_.thrust_n),
                                             bus_.report(mass_flow_signal_, propulsion_.mass_flow_kg_s))) {
        return std::unexpected(*error);
    }
    return {};
}

core::VoidResult VesselSystems::report_navigation(const Navigation& navigation) {
    if (const auto error =
            core::first_error(bus_.report(altitude_signal_, navigation.altitude_m),
                              bus_.report(vertical_speed_signal_, navigation.vertical_speed_m_s),
                              bus_.report(surface_speed_signal_, navigation.surface_speed_m_s),
                              bus_.report(speed_signal_, navigation.speed_m_s))) {
        return std::unexpected(*error);
    }
    return {};
}

} // namespace helios::vessel
