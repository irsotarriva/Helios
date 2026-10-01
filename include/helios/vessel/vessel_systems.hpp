#ifndef HELIOS_VESSEL_VESSEL_SYSTEMS_HPP
#define HELIOS_VESSEL_VESSEL_SYSTEMS_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/time/epoch.hpp"
#include "helios/vessel/assembly.hpp"
#include "helios/vessel/control_bus.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::vessel {

// Where the vessel's nose (+x) points. On rails a vessel has no attitude dynamics: it holds
// this direction exactly (BRIEFING D22).
struct Pointing {
    enum class Frame : std::uint8_t {
        Inertial, // universe axes
        Orbital,  // (prograde, normal, radial-out) of the orbit about the domain body
    };

    Frame frame = Frame::Orbital;
    math::Vector3 direction{1.0, 0.0, 0.0}; // unit length

    [[nodiscard]] friend constexpr bool operator==(const Pointing&, const Pointing&) noexcept = default;
};

// What the trajectory needs to know about the vessel, constant from `epoch` until the next
// state: the thrust along the nose and the mass, which falls at a constant rate.
struct PropulsionState {
    time::Epoch epoch;
    double thrust_n = 0.0;
    double mass_flow_kg_s = 0.0; // mass leaving the vessel
    double mass_kg = 0.0;        // at `epoch`
    double dry_mass_kg = 0.0;    // with every store empty
    Pointing pointing;
};

struct StageCommand {
    std::string signal;
    double value = 0.0;
};

// What activating one stage does: a set of commands, published by the sequencer.
struct Stage {
    std::vector<StageCommand> commands;
};

struct TimedCommand {
    time::Epoch epoch;
    std::string signal;
    ControlSource source = ControlSource::Autopilot;
    double value = 0.0;
};

struct Separation;

// Names of the signals every vessel has, besides "<part>/<port>" for each port of each part and
// "<interface>/<input>" for each input of each interface one of its parts offers.
inline constexpr std::string_view k_signal_stage = "staging/stage";           // number of stages activated
inline constexpr std::string_view k_signal_pointing_frame = "guidance/frame"; // 0 inertial, 1 orbital
inline constexpr std::string_view k_signal_pointing_x = "guidance/x";
inline constexpr std::string_view k_signal_pointing_y = "guidance/y";
inline constexpr std::string_view k_signal_pointing_z = "guidance/z";
inline constexpr std::string_view k_signal_mass = "vessel/mass_kg";
inline constexpr std::string_view k_signal_thrust = "vessel/thrust_n";
inline constexpr std::string_view k_signal_mass_flow = "vessel/mass_flow_kg_s";

// The systems of one vessel in flight: its parts, their stores and processes, and the control
// bus that commands them.
//
// Warp invariance (BRIEFING §7.1, D14). Between two changes every process runs at a constant
// level, so every store changes at a constant rate and its amount is a closed-form function of
// time: asking at 1 s or at 1e6 s cadence gives the same amounts and the same instants at
// which a tank runs dry. A change is a command, or a store running out or filling up; both
// happen at instants that do not depend on when anyone looked.
class VesselSystems {
public:
    [[nodiscard]] static core::Result<VesselSystems> make(Assembly assembly,
                                                          std::span<const InterfaceDefinition> interfaces,
                                                          std::vector<Stage> stages,
                                                          const time::Epoch& start);

    [[nodiscard]] const Assembly& assembly() const noexcept { return assembly_; }
    [[nodiscard]] const ControlBus& bus() const noexcept { return bus_; }
    [[nodiscard]] std::span<const Stage> stages() const noexcept { return stages_; }

    // Publishes (or releases) a command at `instant`, which must not precede the latest change,
    // after bringing the systems up to that instant. Parts that separate as a result are
    // appended to `separations`.
    [[nodiscard]] core::VoidResult command(std::string_view signal, ControlSource source, double value,
                                           const time::Epoch& instant, std::vector<Separation>& separations);
    [[nodiscard]] core::VoidResult release(std::string_view signal, ControlSource source,
                                           const time::Epoch& instant, std::vector<Separation>& separations);

    // A command to be published when its epoch is reached.
    [[nodiscard]] core::VoidResult schedule(TimedCommand command);
    [[nodiscard]] std::span<const TimedCommand> program() const noexcept { return program_; }

    // Applies every change up to and including `instant`, in time order.
    [[nodiscard]] core::VoidResult advance_to(const time::Epoch& instant,
                                              std::vector<Separation>& separations);

    // The instant of the next change, if one is coming: a scheduled command, or a store running
    // out or filling up.
    [[nodiscard]] std::optional<time::Epoch> next_change() const noexcept;

    // The propulsion state since the latest change, and the states that will follow it as far
    // as the scheduled commands and the stores determine them (at most `max_states`). A state
    // that only continues a coast is left out: it would not alter the trajectory.
    [[nodiscard]] const PropulsionState& propulsion() const noexcept { return propulsion_; }
    [[nodiscard]] core::Result<std::vector<PropulsionState>> forecast(std::size_t max_states) const;

    // In the unit of the resource; 0 for an unknown part or store.
    [[nodiscard]] double amount(std::size_t part, std::size_t store,
                                const time::Epoch& instant) const noexcept;
    [[nodiscard]] double mass_kg(const time::Epoch& instant) const noexcept;
    [[nodiscard]] core::Result<MassProperties> mass_properties(const time::Epoch& instant) const;

    // Writes every telemetry signal of the bus for `instant`.
    [[nodiscard]] core::VoidResult report_telemetry(const time::Epoch& instant);

private:
    struct StoreState {
        std::size_t part = 0;
        std::size_t store = 0; // in the part's datasheet
        std::size_t pool = 0;
        double base_amount = 0.0; // at the latest change
        double rate_per_s = 0.0;
    };
    // The stores of one resource that feed each other.
    struct Pool {
        std::string resource;
        std::size_t feed_group = 0;
        std::vector<std::size_t> stores;
    };
    struct ProcessState {
        std::size_t part = 0;
        std::size_t process = 0; // in the part's datasheet
        std::optional<SignalId> level_signal;
        std::optional<SignalId> enable_signal;
        std::vector<std::optional<std::size_t>> consumed_pools; // per flow; none: nothing to draw from
        std::vector<std::optional<std::size_t>> produced_pools;
        double level = 0.0;
    };
    struct OutputBinding {
        std::size_t part = 0;
        std::size_t output = 0;
        SignalId signal;
    };
    struct SeparatorBinding {
        std::size_t part = 0;
        SignalId signal;
    };

    VesselSystems() = default;

    [[nodiscard]] static core::Result<VesselSystems> build(Assembly assembly,
                                                           std::vector<InterfaceDefinition> interfaces,
                                                           std::vector<Stage> stages,
                                                           const time::Epoch& start);

    [[nodiscard]] double store_amount(const StoreState& state, const time::Epoch& instant) const noexcept;
    [[nodiscard]] const Store& store_definition(const StoreState& state) const noexcept;
    [[nodiscard]] std::vector<double> command_values() const;
    [[nodiscard]] Pointing pointing() const noexcept;
    void rebase(const time::Epoch& instant) noexcept;
    void evaluate();
    [[nodiscard]] core::VoidResult change_command(std::string_view signal, ControlSource source,
                                                  std::optional<double> value, const time::Epoch& instant,
                                                  std::vector<Separation>& separations);
    [[nodiscard]] core::VoidResult apply_change(const time::Epoch& instant,
                                                std::vector<Separation>& separations);
    [[nodiscard]] core::VoidResult settle(const time::Epoch& instant, std::vector<Separation>& separations);
    [[nodiscard]] core::VoidResult settle_with_separated(const time::Epoch& instant,
                                                         std::vector<Separation>& separations);
    [[nodiscard]] core::VoidResult detach(std::size_t part, const time::Epoch& instant,
                                          std::vector<Separation>& separations);

    Assembly assembly_;
    std::vector<InterfaceDefinition> interfaces_;
    ControlBus bus_;
    std::vector<Stage> stages_;
    std::size_t stages_activated_ = 0;
    std::vector<TimedCommand> program_; // sorted by epoch

    std::vector<StoreState> stores_;
    std::vector<Pool> pools_;
    std::vector<ProcessState> processes_;
    std::vector<OutputBinding> outputs_;
    std::vector<SeparatorBinding> separators_;
    SignalId stage_signal_;
    SignalId pointing_frame_signal_;
    SignalId pointing_x_signal_;
    SignalId pointing_y_signal_;
    SignalId pointing_z_signal_;
    SignalId mass_signal_;
    SignalId thrust_signal_;
    SignalId mass_flow_signal_;

    time::Epoch segment_start_;               // the latest change
    std::optional<time::Epoch> store_change_; // when a store next runs out or fills up
    PropulsionState propulsion_;
};

// Parts that left a vessel, as a vessel of their own. The separator pushes the two apart along
// the nose of the vessel they left.
struct Separation {
    time::Epoch epoch;
    VesselSystems systems; // of the parts that left
    Pointing pointing;     // of the vessel they left, at the separation
    double kept_delta_v_m_s = 0.0;
    double separated_delta_v_m_s = 0.0;
};

} // namespace helios::vessel

#endif // HELIOS_VESSEL_VESSEL_SYSTEMS_HPP
