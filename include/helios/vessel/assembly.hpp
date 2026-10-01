#ifndef HELIOS_VESSEL_ASSEMBLY_HPP
#define HELIOS_VESSEL_ASSEMBLY_HPP

#include "helios/core/error.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::vessel {

struct PartInstance {
    std::string name; // unique in the vessel: its signals are addressed as "<name>/<port>"
    std::shared_ptr<const PartDatasheet> datasheet;
    std::optional<std::size_t> parent;   // none for the root part
    math::Vector3 position_in_parent_m;  // of this part's origin, in the parent's axes
    math::Matrix3 orientation_in_parent; // takes this part's axes to the parent's
};

// Where a part is in the vessel: the vessel's axes and origin are those of its root part, with
// +x towards the nose (the direction the vessel is pointed in).
struct PartPose {
    math::Vector3 position_m;
    math::Matrix3 orientation; // takes the part's axes to the vessel's
};

struct MassProperties {
    double mass_kg = 0.0;
    math::Vector3 centre_of_mass_m; // vessel axes
    math::Matrix3 inertia_kg_m2;    // about the centre of mass, vessel axes
};

struct AssemblySplit;

// The part tree of a vessel (BRIEFING §10.4): one rigid body, whose topology changes only by
// explicit operations such as split_at().
class Assembly {
public:
    // The first part is the root and has no parent; every other part names a parent already added.
    [[nodiscard]] core::Result<std::size_t> add_part(PartInstance part);

    [[nodiscard]] std::span<const PartInstance> parts() const noexcept { return parts_; }
    [[nodiscard]] std::span<const PartPose> poses() const noexcept { return poses_; }
    [[nodiscard]] core::Result<std::size_t> find(std::string_view name) const;

    // `part` and everything attached below it, parents before children.
    [[nodiscard]] std::vector<std::size_t> subtree(std::size_t part) const;

    // Composite mass properties, given the mass of every part including its contents.
    [[nodiscard]] core::Result<MassProperties> mass_properties(std::span<const double> part_masses_kg) const;

    // Detaches `part` (not the root) from its parent.
    [[nodiscard]] core::Result<AssemblySplit> split_at(std::size_t part) const;

private:
    std::vector<PartInstance> parts_;
    std::vector<PartPose> poses_;
};

struct AssemblySplit {
    Assembly kept;      // what stays with the root
    Assembly separated; // the detached part, as the root of its own vessel, and its subtree
    // For each part of the new assemblies, its index in the original one.
    std::vector<std::size_t> kept_parts;
    std::vector<std::size_t> separated_parts;
};

// Rotation by `angle_rad` about `axis` (right-handed): for placing a part in its parent.
[[nodiscard]] core::Result<math::Matrix3> rotation_about(const math::Vector3& axis, double angle_rad);

} // namespace helios::vessel

#endif // HELIOS_VESSEL_ASSEMBLY_HPP
