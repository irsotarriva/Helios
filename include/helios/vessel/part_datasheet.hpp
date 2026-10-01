#ifndef HELIOS_VESSEL_PART_DATASHEET_HPP
#define HELIOS_VESSEL_PART_DATASHEET_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/vessel/curve.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Datasheets: what the flight simulation knows about a part (BRIEFING §10, D22).
//
// A part is designed in detail elsewhere (blade angles, nozzle contours, …) and characterised
// into a datasheet: its mass properties, the interface it offers (inputs it accepts, outputs it
// reports, resources it holds and exchanges) and the curves that relate them. Flight consumes
// only the datasheet, so it never depends on how the part was designed.
namespace helios::vessel {

// Something parts hold and exchange: a propellant (amounts in kg), electric charge (J), …
struct ResourceDefinition {
    std::string id;
    std::string name;
    std::string unit;              // of one unit of amount, for display
    double mass_per_unit_kg = 0.0; // 0 for massless resources such as electric charge
};

// A standard interface: the ports every part claiming it must have. Controls, autopilots and
// cockpit gauges bind to interfaces ("every engine's throttle") instead of to particular parts,
// so a new part works with them as soon as it declares the interface.
struct InterfaceDefinition {
    std::string id;
    std::vector<std::string> inputs;
    std::vector<std::string> outputs;
};

// A control the part accepts.
struct InputPort {
    std::string name;
    std::string unit;
    double minimum = 0.0;
    double maximum = 1.0;
    double default_value = 0.0;
    bool toggle = false; // a switch (off = 0, on = 1) rather than a lever
};

struct Store {
    std::string resource;
    double capacity = 0.0; // in the resource's unit
    double initial_amount = 0.0;
    double mass_per_unit_kg = 0.0; // filled in from the resource definition
};

struct ResourceFlow {
    std::string resource;
    double rate_per_s = 0.0;       // resource units per second at level 1
    double mass_per_unit_kg = 0.0; // filled in from the resource definition
};

// Something the part does while it runs: it exchanges resources and may push the vessel.
// Rates scale with the process level (0 = off, 1 = full), which follows one of the part's
// inputs. A process stops when a resource it consumes runs out.
struct Process {
    std::string name;
    std::optional<std::size_t> level_input;  // index into the inputs; none: level 1 whenever enabled
    std::optional<std::size_t> enable_input; // must be on (≥ 0.5) for the process to run; none: always
    double minimum_level = 0.0;              // a running process never goes below this level
    std::vector<ResourceFlow> consumes;
    std::vector<ResourceFlow> produces;
    double thrust_n = 0.0;                         // at level 1, in vacuum
    std::optional<Curve> thrust_curve;             // fraction of thrust_n against level; none: the level
    math::Vector3 thrust_direction{1.0, 0.0, 0.0}; // of the force on the vessel, part axes, unit length
    math::Vector3 thrust_position_m;               // where it acts, part axes
};

// Something the part reports.
struct OutputPort {
    enum class Quantity : std::uint8_t {
        Level,    // of a process
        Thrust,   // of a process, N
        MassFlow, // consumed by a process, kg/s
        Curve,    // a characterised curve of a process's level (a temperature, a pressure, …)
        Amount,   // held in a store, in the resource's unit
        Fraction, // of a store's capacity
    };

    std::string name;
    std::string unit;
    Quantity quantity = Quantity::Level;
    std::size_t index = 0; // the process, or the store for Amount and Fraction
    std::optional<Curve> curve;
};

// The part lets go of its parent, taking everything attached below it along.
struct Separator {
    std::size_t input = 0;    // the input that fires it
    double impulse_n_s = 0.0; // pushes the two sides apart
};

struct PartDatasheet {
    std::string id;
    std::string name;
    double dry_mass_kg = 0.0;
    math::Vector3 centre_of_mass_m; // part axes
    // Principal moments of inertia about the centre of mass per unit of mass (part axes). The
    // contents of the stores are taken to be distributed like the part itself.
    math::Vector3 gyration_m2;
    bool crossfeed = true; // resources flow between this part and its parent
    std::vector<std::string> interfaces;
    std::vector<InputPort> inputs;
    std::vector<OutputPort> outputs;
    std::vector<Store> stores;
    std::vector<Process> processes;
    std::optional<Separator> separator;
};

class PartCatalog {
public:
    [[nodiscard]] core::VoidResult add_resource(ResourceDefinition resource);
    [[nodiscard]] core::VoidResult add_interface(InterfaceDefinition definition);
    // Validates the datasheet against the resources and interfaces already in the catalogue.
    [[nodiscard]] core::VoidResult add_part(PartDatasheet part);

    [[nodiscard]] core::Result<std::shared_ptr<const PartDatasheet>> part(std::string_view id) const;
    [[nodiscard]] std::span<const ResourceDefinition> resources() const noexcept { return resources_; }
    [[nodiscard]] std::span<const InterfaceDefinition> interfaces() const noexcept { return interfaces_; }
    [[nodiscard]] std::span<const std::shared_ptr<const PartDatasheet>> parts() const noexcept {
        return parts_;
    }

    // Adds the [[resource]], [[interface]] and [[part]] tables of a TOML document (the format is
    // described in data/parts/README.md). Nothing is added if any of them is invalid.
    [[nodiscard]] core::VoidResult load_toml(std::string_view text);
    [[nodiscard]] core::VoidResult load_file(const std::filesystem::path& path);
    // Every .toml file of a directory, in name order.
    [[nodiscard]] core::VoidResult load_directory(const std::filesystem::path& directory);

private:
    std::vector<ResourceDefinition> resources_;
    std::vector<InterfaceDefinition> interfaces_;
    // Rationale: shared so a vessel keeps its datasheets alive on its own, whatever happens to
    // the catalogue afterwards (mods reloaded, another save loaded).
    std::vector<std::shared_ptr<const PartDatasheet>> parts_;
};

} // namespace helios::vessel

#endif // HELIOS_VESSEL_PART_DATASHEET_HPP
