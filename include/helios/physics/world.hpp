#ifndef HELIOS_PHYSICS_WORLD_HPP
#define HELIOS_PHYSICS_WORLD_HPP

#include "helios/core/error.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"

#include <compare>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// Rigid bodies in a local frame (BRIEFING §5.3, D23, D25). This is the only place that knows
// the rigid-body engine (Jolt Physics): nothing of it appears in this header, so the rest of
// Helios depends on these few types and not on the engine.
//
// The world is small and local: single-precision inside, metres from an origin the caller
// chooses and keeps near the bodies. It knows nothing of orbits; whoever drives it supplies
// the apparent gravity of its frame.
namespace helios::physics {

struct BodyId {
    std::uint32_t value = 0xFFFFFFFFU;

    [[nodiscard]] friend constexpr auto operator<=>(const BodyId&, const BodyId&) noexcept = default;
};

// A convex piece of a body's shape: a solid cylinder about its own x axis, or a sphere when
// its half length is zero.
struct ShapePiece {
    double radius_m = 0.0;
    double half_length_m = 0.0;
    math::Vector3 position_m;     // of its centre from the body's centre of mass, body axes
    math::Quaternion orientation; // takes the piece's axes to the body's
};

struct BodyState {
    math::Vector3 position_m;             // of the centre of mass
    math::Quaternion orientation;         // takes the body's axes to the world's
    math::Vector3 velocity_m_s;           // of the centre of mass
    math::Vector3 angular_velocity_rad_s; // world axes
};

struct BodyDescription {
    std::vector<ShapePiece> pieces; // none: the body touches nothing
    double mass_kg = 0.0;
    math::Matrix3 inertia_kg_m2; // about the centre of mass, body axes
    BodyState state;
    double friction = 0.8;
};

// Two bodies that came into contact during a step.
struct Contact {
    BodyId first;
    BodyId second;
    double closing_speed_m_s = 0.0; // along the contact normal, at the contact point
};

class World {
public:
    [[nodiscard]] static core::Result<World> make();

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept;
    World& operator=(World&&) noexcept;
    ~World();

    [[nodiscard]] core::Result<BodyId> add_body(const BodyDescription& description);
    // Solid, immovable ground: everything behind the plane through `point_m` whose outward
    // normal is `unit_normal`.
    [[nodiscard]] core::Result<BodyId> add_ground(const math::Vector3& point_m,
                                                  const math::Vector3& unit_normal, double friction);
    // Puts the ground somewhere else: the frame of the world moves over the terrain.
    [[nodiscard]] core::VoidResult set_ground(BodyId id, const math::Vector3& point_m,
                                              const math::Vector3& unit_normal);
    [[nodiscard]] core::VoidResult remove(BodyId id);

    [[nodiscard]] core::Result<BodyState> state(BodyId id) const;
    [[nodiscard]] core::VoidResult set_state(BodyId id, const BodyState& state);
    // For a body whose mass changes (propellant burnt); its shape stays as it was made.
    [[nodiscard]] core::VoidResult set_mass(BodyId id, double mass_kg, const math::Matrix3& inertia_kg_m2);

    // Loads for the next step only. The point and the vectors are in world axes.
    [[nodiscard]] core::VoidResult add_force(BodyId id, const math::Vector3& force_n,
                                             const math::Vector3& point_m);
    [[nodiscard]] core::VoidResult add_torque(BodyId id, const math::Vector3& torque_n_m);

    // The acceleration every body has in this frame when nothing touches or pushes it.
    void set_gravity(const math::Vector3& acceleration_m_s2);

    [[nodiscard]] core::VoidResult step(double duration_s);

    // Contacts that began during the latest step.
    [[nodiscard]] std::span<const Contact> contacts_begun() const noexcept;
    // Whether the body was touching another during the latest step.
    [[nodiscard]] bool touching(BodyId id) const noexcept;

private:
    struct Implementation;

    explicit World(std::unique_ptr<Implementation> implementation) noexcept;

    std::unique_ptr<Implementation> implementation_;
};

} // namespace helios::physics

#endif // HELIOS_PHYSICS_WORLD_HPP
