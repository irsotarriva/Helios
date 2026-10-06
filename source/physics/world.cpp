#include "helios/physics/world.hpp"

#include "helios/core/logging.hpp"

// clang-format off: Jolt.h must come first, it configures every other Jolt header.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <format>
#include <numbers>
#include <utility>

namespace helios::physics {

namespace {

using core::ErrorCode;
using math::Matrix3;
using math::Quaternion;
using math::Vector3;

constexpr JPH::ObjectLayer k_layer_static = 0;
constexpr JPH::ObjectLayer k_layer_moving = 1;
constexpr JPH::uint k_layer_count = 2;
constexpr JPH::uint k_max_bodies = 256;
constexpr JPH::uint k_max_body_pairs = 1024;
constexpr JPH::uint k_max_contact_constraints = 1024;
constexpr std::size_t k_temporary_memory_bytes = std::size_t{4} * 1024 * 1024;
// A piece is never thinner than this: the engine needs a margin inside each convex shape.
constexpr double k_min_piece_size_m = 0.02;
constexpr float k_max_speed_m_s = 1.0e4F;
constexpr float k_max_spin_rad_s = 100.0F;
constexpr float k_ground_half_extent_m = 5.0e4F;

[[nodiscard]] JPH::Vec3 to_jolt(const Vector3& vector) noexcept {
    return {static_cast<float>(vector.x), static_cast<float>(vector.y), static_cast<float>(vector.z)};
}
[[nodiscard]] JPH::Quat to_jolt(const Quaternion& rotation) noexcept {
    return {static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z),
            static_cast<float>(rotation.w)};
}
[[nodiscard]] Vector3 from_jolt(JPH::Vec3Arg vector) noexcept {
    return {static_cast<double>(vector.GetX()), static_cast<double>(vector.GetY()),
            static_cast<double>(vector.GetZ())};
}
[[nodiscard]] Quaternion from_jolt(JPH::QuatArg rotation) noexcept {
    return math::normalized({static_cast<double>(rotation.GetW()), static_cast<double>(rotation.GetX()),
                             static_cast<double>(rotation.GetY()), static_cast<double>(rotation.GetZ())});
}

[[nodiscard]] bool is_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool is_finite(const BodyState& state) noexcept {
    const Quaternion& rotation = state.orientation;
    return is_finite(state.position_m) && is_finite(state.velocity_m_s)
           && is_finite(state.angular_velocity_rad_s) && std::isfinite(rotation.w)
           && std::isfinite(rotation.x) && std::isfinite(rotation.y) && std::isfinite(rotation.z);
}

[[nodiscard]] core::Result<JPH::MassProperties> mass_properties(double mass_kg,
                                                                const Matrix3& inertia_kg_m2) {
    bool finite = std::isfinite(mass_kg);
    for (const double element : inertia_kg_m2.elements) {
        finite = finite && std::isfinite(element);
    }
    if (!finite || !(mass_kg > 0.0) || !(inertia_kg_m2(0, 0) > 0.0) || !(inertia_kg_m2(1, 1) > 0.0)
        || !(inertia_kg_m2(2, 2) > 0.0)) {
        return core::fail(ErrorCode::OutOfRange, "a body needs a positive mass and inertia");
    }
    JPH::MassProperties properties;
    properties.mMass = static_cast<float>(mass_kg);
    properties.mInertia = JPH::Mat44::sIdentity();
    for (JPH::uint row = 0; row < 3; ++row) {
        for (JPH::uint column = 0; column < 3; ++column) {
            properties.mInertia(row, column) = static_cast<float>(inertia_kg_m2(row, column));
        }
    }
    return properties;
}

// Rationale for the suppressions: the signature is Jolt's, va_start is what initialises a
// va_list, and where a va_list is an array it is meant to be passed as one.
// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg, cert-dcl50-cpp, cppcoreguidelines-init-variables)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
void trace(const char* format, ...) {
    std::array<char, 1024> buffer{};
    std::va_list arguments;
    va_start(arguments, format);
    static_cast<void>(std::vsnprintf(buffer.data(), buffer.size(), format, arguments));
    va_end(arguments);
    LOG_DEBUG("{}", buffer.data()).tag("subsystem", "physics");
}
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
// NOLINTEND(cppcoreguidelines-pro-type-vararg, cert-dcl50-cpp, cppcoreguidelines-init-variables)

#ifdef JPH_ENABLE_ASSERTS
// Reports a failed assertion of the engine and carries on (returning true would break into
// the debugger).
bool assert_failed(const char* expression, const char* message, const char* file, JPH::uint line) {
    LOG_ERROR("physics engine assertion failed: {} ({}) at {}:{}", expression,
              message != nullptr ? message : "", file, line)
        .tag("subsystem", "physics");
    return false;
}
#endif

// The engine's process-wide registrations, made once and kept for the life of the process.
void register_engine_once() {
    static const bool registered = [] {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = trace;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assert_failed;)
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): owned by the engine through this global.
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
        return true;
    }();
    static_cast<void>(registered);
}

class ContactRecorder final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold& manifold,
                        JPH::ContactSettings& /*settings*/) override {
        // The normal points from the first body to the second; they close when the second
        // moves against it relative to the first.
        double closing_speed_m_s = 0.0;
        if (!manifold.mRelativeContactPointsOn1.empty()) {
            const JPH::RVec3 point = manifold.GetWorldSpaceContactPointOn1(0);
            const JPH::Vec3 relative = second.GetPointVelocity(point) - first.GetPointVelocity(point);
            closing_speed_m_s = std::max(0.0, -static_cast<double>(relative.Dot(manifold.mWorldSpaceNormal)));
        }
        begun_.push_back(Contact{.first = {first.GetID().GetIndexAndSequenceNumber()},
                                 .second = {second.GetID().GetIndexAndSequenceNumber()},
                                 .closing_speed_m_s = closing_speed_m_s});
        touch(first, second);
    }

    void OnContactPersisted(const JPH::Body& first, const JPH::Body& second,
                            const JPH::ContactManifold& /*manifold*/,
                            JPH::ContactSettings& /*settings*/) override {
        touch(first, second);
    }

    void begin_step() {
        begun_.clear();
        touching_.clear();
    }

    [[nodiscard]] const std::vector<Contact>& begun() const noexcept { return begun_; }
    [[nodiscard]] bool touching(BodyId id) const noexcept {
        return std::ranges::find(touching_, id) != touching_.end();
    }

private:
    void touch(const JPH::Body& first, const JPH::Body& second) {
        touching_.push_back({first.GetID().GetIndexAndSequenceNumber()});
        touching_.push_back({second.GetID().GetIndexAndSequenceNumber()});
    }

    std::vector<Contact> begun_;
    std::vector<BodyId> touching_;
};

// NOLINTBEGIN(cppcoreguidelines-owning-memory): Jolt's shapes are reference-counted; the Ref or
// the settings object a new shape is handed to owns it from then on.
[[nodiscard]] core::Result<JPH::ShapeRefC> make_shape(const std::vector<ShapePiece>& pieces) {
    // Jolt's cylinder stands along y; ours lies along x.
    const Quaternion y_to_x = math::from_axis_angle({0.0, 0.0, 1.0}, -0.5 * std::numbers::pi);
    JPH::StaticCompoundShapeSettings compound;
    for (const ShapePiece& piece : pieces) {
        const Vector3& half_m = piece.box_half_extents_m;
        if (half_m != Vector3{}) {
            if (!is_finite(half_m) || !(half_m.x > 0.0) || !(half_m.y > 0.0) || !(half_m.z > 0.0)
                || !is_finite(piece.position_m)) {
                return core::fail(ErrorCode::OutOfRange,
                                  "a box piece needs positive half extents and a finite place");
            }
            const Vector3 clamped_m{std::max(half_m.x, k_min_piece_size_m),
                                    std::max(half_m.y, k_min_piece_size_m),
                                    std::max(half_m.z, k_min_piece_size_m)};
            const double margin_m = std::min({0.05, 0.5 * clamped_m.x, 0.5 * clamped_m.y, 0.5 * clamped_m.z});
            compound.AddShape(to_jolt(piece.position_m), to_jolt(math::normalized(piece.orientation)),
                              new JPH::BoxShape(to_jolt(clamped_m), static_cast<float>(margin_m)));
            continue;
        }
        if (!std::isfinite(piece.radius_m) || !std::isfinite(piece.half_length_m) || !(piece.radius_m > 0.0)
            || piece.half_length_m < 0.0 || !is_finite(piece.position_m)) {
            return core::fail(ErrorCode::OutOfRange,
                              "a shape piece needs a positive radius and a finite place");
        }
        const double radius_m = std::max(piece.radius_m, k_min_piece_size_m);
        JPH::ShapeRefC shape;
        if (piece.half_length_m > 0.0) {
            const double half_length_m = std::max(piece.half_length_m, k_min_piece_size_m);
            const double margin_m = std::min({0.05, 0.5 * radius_m, 0.5 * half_length_m});
            shape = new JPH::CylinderShape(static_cast<float>(half_length_m), static_cast<float>(radius_m),
                                           static_cast<float>(margin_m));
        } else {
            shape = new JPH::SphereShape(static_cast<float>(radius_m));
        }
        compound.AddShape(to_jolt(piece.position_m), to_jolt(math::normalized(piece.orientation * y_to_x)),
                          shape);
    }
    if (pieces.empty()) {
        // Jolt needs a shape; a body that should touch nothing gets one too small to matter.
        compound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(),
                          new JPH::SphereShape(static_cast<float>(k_min_piece_size_m)));
    }
    const JPH::ShapeSettings::ShapeResult built = compound.Create();
    if (built.HasError()) {
        return core::fail(ErrorCode::ExternalLibraryFailure,
                          std::format("cannot build the body's shape: {}", built.GetError()));
    }
    // The pieces are placed about the centre of mass the caller computed, which the engine
    // must use instead of the one it derives from their volumes.
    const JPH::ShapeRefC& shape = built.Get();
    const JPH::ShapeSettings::ShapeResult centred =
        JPH::OffsetCenterOfMassShapeSettings(-shape->GetCenterOfMass(), shape).Create();
    if (centred.HasError()) {
        return core::fail(ErrorCode::ExternalLibraryFailure,
                          std::format("cannot build the body's shape: {}", centred.GetError()));
    }
    return centred.Get();
}
// NOLINTEND(cppcoreguidelines-owning-memory)

} // namespace

// Rationale: Jolt is built without exceptions and reports failures through return values and
// its assertion hook, so its calls are not wrapped in core::try_call.
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding): the members are in order of lifetime.
struct World::Implementation {
    Implementation()
        : temporary_memory(k_temporary_memory_bytes), jobs(JPH::cMaxPhysicsJobs),
          broad_phase_layers(k_layer_count, k_layer_count), layer_pairs(k_layer_count) {
        broad_phase_layers.MapObjectToBroadPhaseLayer(k_layer_static, JPH::BroadPhaseLayer(0));
        broad_phase_layers.MapObjectToBroadPhaseLayer(k_layer_moving, JPH::BroadPhaseLayer(1));
        layer_pairs.EnableCollision(k_layer_moving, k_layer_static);
        layer_pairs.EnableCollision(k_layer_moving, k_layer_moving);
    }

    [[nodiscard]] bool knows(BodyId id) const noexcept {
        return std::ranges::find(bodies, id) != bodies.end();
    }

    JPH::TempAllocatorImpl temporary_memory;
    JPH::JobSystemSingleThreaded jobs;
    JPH::BroadPhaseLayerInterfaceTable broad_phase_layers;
    JPH::ObjectLayerPairFilterTable layer_pairs;
    std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> layer_filter;
    JPH::PhysicsSystem system;
    ContactRecorder contacts;
    std::vector<BodyId> bodies;
};

core::Result<World> World::make() {
    register_engine_once();
    auto implementation = std::make_unique<Implementation>();
    implementation->layer_filter = std::make_unique<JPH::ObjectVsBroadPhaseLayerFilterTable>(
        implementation->broad_phase_layers, k_layer_count, implementation->layer_pairs, k_layer_count);
    implementation->system.Init(k_max_bodies, 0, k_max_body_pairs, k_max_contact_constraints,
                                implementation->broad_phase_layers, *implementation->layer_filter,
                                implementation->layer_pairs);
    implementation->system.SetGravity(JPH::Vec3::sZero());
    implementation->system.SetContactListener(&implementation->contacts);
    return World(std::move(implementation));
}

World::World(std::unique_ptr<Implementation> implementation) noexcept
    : implementation_(std::move(implementation)) {
}
World::World(World&&) noexcept = default;
World& World::operator=(World&&) noexcept = default;
World::~World() {
    if (!implementation_) {
        return;
    }
    JPH::BodyInterface& bodies = implementation_->system.GetBodyInterfaceNoLock();
    for (const BodyId id : implementation_->bodies) {
        bodies.RemoveBody(JPH::BodyID(id.value));
        bodies.DestroyBody(JPH::BodyID(id.value));
    }
}

core::Result<BodyId> World::add_body(const BodyDescription& description) {
    const auto properties = mass_properties(description.mass_kg, description.inertia_kg_m2);
    if (!properties) {
        return std::unexpected(properties.error());
    }
    if (!is_finite(description.state) || !(description.friction >= 0.0)) {
        return core::fail(ErrorCode::NotFinite, "a body needs a finite state and a friction of 0 or more");
    }
    const auto shape = make_shape(description.pieces);
    if (!shape) {
        return std::unexpected(shape.error());
    }
    JPH::BodyCreationSettings settings(*shape, JPH::RVec3(to_jolt(description.state.position_m)),
                                       to_jolt(math::normalized(description.state.orientation)),
                                       JPH::EMotionType::Dynamic, k_layer_moving);
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    settings.mMassPropertiesOverride = *properties;
    settings.mLinearVelocity = to_jolt(description.state.velocity_m_s);
    settings.mAngularVelocity = to_jolt(description.state.angular_velocity_rad_s);
    settings.mFriction = static_cast<float>(description.friction);
    settings.mRestitution = 0.0F;
    // Nothing in space slows a body down by itself, and the caller decides when one is at rest.
    settings.mLinearDamping = 0.0F;
    settings.mAngularDamping = 0.0F;
    settings.mAllowSleeping = false;
    settings.mMaxLinearVelocity = k_max_speed_m_s;
    settings.mMaxAngularVelocity = k_max_spin_rad_s;
    const JPH::BodyID id = implementation_->system.GetBodyInterfaceNoLock().CreateAndAddBody(
        settings, JPH::EActivation::Activate);
    if (id.IsInvalid()) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the physics world is full");
    }
    implementation_->bodies.push_back({id.GetIndexAndSequenceNumber()});
    return implementation_->bodies.back();
}

core::Result<BodyId> World::add_ground(const Vector3& point_m, const Vector3& unit_normal, double friction) {
    if (!is_finite(point_m) || !is_finite(unit_normal) || std::abs(math::norm(unit_normal) - 1.0) > 1e-6
        || !(friction >= 0.0)) {
        return core::fail(ErrorCode::InvalidArgument, "the ground needs a finite point and a unit normal");
    }
    // The plane's own normal is +y; the body's rotation turns it to the one asked for, so the
    // ground can be turned later without a new shape.
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): owned by the reference-counted pointer.
    const JPH::ShapeRefC shape =
        new JPH::PlaneShape(JPH::Plane(JPH::Vec3::sAxisY(), 0.0F), nullptr, k_ground_half_extent_m);
    JPH::BodyCreationSettings settings(shape, JPH::RVec3(to_jolt(point_m)),
                                       JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), to_jolt(unit_normal)),
                                       JPH::EMotionType::Static, k_layer_static);
    settings.mFriction = static_cast<float>(friction);
    settings.mRestitution = 0.0F;
    const JPH::BodyID id = implementation_->system.GetBodyInterfaceNoLock().CreateAndAddBody(
        settings, JPH::EActivation::DontActivate);
    if (id.IsInvalid()) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the physics world is full");
    }
    implementation_->bodies.push_back({id.GetIndexAndSequenceNumber()});
    return implementation_->bodies.back();
}

core::Result<BodyId> World::add_static_box(const BoxDescription& box) {
    const Vector3& half_m = box.half_extents_m;
    if (!is_finite(box.centre_m) || !is_finite(half_m) || !(half_m.x > 0.0) || !(half_m.y > 0.0)
        || !(half_m.z > 0.0) || !(box.friction >= 0.0)) {
        return core::fail(ErrorCode::InvalidArgument,
                          "a box needs a finite centre, positive half extents and a friction of 0 or more");
    }
    const Vector3 clamped_m{std::max(half_m.x, k_min_piece_size_m), std::max(half_m.y, k_min_piece_size_m),
                            std::max(half_m.z, k_min_piece_size_m)};
    const double margin_m = std::min({0.05, 0.5 * clamped_m.x, 0.5 * clamped_m.y, 0.5 * clamped_m.z});
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): owned by the reference-counted pointer.
    const JPH::ShapeRefC shape = new JPH::BoxShape(to_jolt(clamped_m), static_cast<float>(margin_m));
    JPH::BodyCreationSettings settings(shape, JPH::RVec3(to_jolt(box.centre_m)),
                                       to_jolt(math::normalized(box.orientation)), JPH::EMotionType::Static,
                                       k_layer_static);
    settings.mFriction = static_cast<float>(box.friction);
    settings.mRestitution = 0.0F;
    const JPH::BodyID id = implementation_->system.GetBodyInterfaceNoLock().CreateAndAddBody(
        settings, JPH::EActivation::DontActivate);
    if (id.IsInvalid()) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the physics world is full");
    }
    implementation_->bodies.push_back({id.GetIndexAndSequenceNumber()});
    return implementation_->bodies.back();
}

core::VoidResult World::set_ground(BodyId id, const Vector3& point_m, const Vector3& unit_normal) {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    if (!is_finite(point_m) || !is_finite(unit_normal) || std::abs(math::norm(unit_normal) - 1.0) > 1e-6) {
        return core::fail(ErrorCode::InvalidArgument, "the ground needs a finite point and a unit normal");
    }
    implementation_->system.GetBodyInterfaceNoLock().SetPositionAndRotation(
        JPH::BodyID(id.value), JPH::RVec3(to_jolt(point_m)),
        JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), to_jolt(unit_normal)), JPH::EActivation::DontActivate);
    return {};
}

core::VoidResult World::remove(BodyId id) {
    const auto known = std::ranges::find(implementation_->bodies, id);
    if (known == implementation_->bodies.end()) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    JPH::BodyInterface& bodies = implementation_->system.GetBodyInterfaceNoLock();
    bodies.RemoveBody(JPH::BodyID(id.value));
    bodies.DestroyBody(JPH::BodyID(id.value));
    implementation_->bodies.erase(known);
    return {};
}

core::Result<BodyState> World::state(BodyId id) const {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    const JPH::BodyInterface& bodies = implementation_->system.GetBodyInterfaceNoLock();
    const JPH::BodyID body(id.value);
    return BodyState{.position_m = from_jolt(JPH::Vec3(bodies.GetCenterOfMassPosition(body))),
                     .orientation = from_jolt(bodies.GetRotation(body)),
                     .velocity_m_s = from_jolt(bodies.GetLinearVelocity(body)),
                     .angular_velocity_rad_s = from_jolt(bodies.GetAngularVelocity(body))};
}

core::VoidResult World::set_state(BodyId id, const BodyState& state) {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    if (!is_finite(state)) {
        return core::fail(ErrorCode::NotFinite, "a body's state must be finite");
    }
    JPH::BodyInterface& bodies = implementation_->system.GetBodyInterfaceNoLock();
    const JPH::BodyID body(id.value);
    // The shape is centred on the centre of mass, so the body's position is that of its centre.
    bodies.SetPositionAndRotation(body, JPH::RVec3(to_jolt(state.position_m)),
                                  to_jolt(math::normalized(state.orientation)), JPH::EActivation::Activate);
    bodies.SetLinearAndAngularVelocity(body, to_jolt(state.velocity_m_s),
                                       to_jolt(state.angular_velocity_rad_s));
    return {};
}

core::VoidResult World::set_mass(BodyId id, double mass_kg, const Matrix3& inertia_kg_m2) {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    const auto properties = mass_properties(mass_kg, inertia_kg_m2);
    if (!properties) {
        return std::unexpected(properties.error());
    }
    const JPH::BodyLockWrite lock(implementation_->system.GetBodyLockInterfaceNoLock(),
                                  JPH::BodyID(id.value));
    if (!lock.Succeeded() || !lock.GetBody().IsDynamic()) {
        return core::fail(ErrorCode::InvalidArgument, "only a moving body has a mass to set");
    }
    lock.GetBody().GetMotionProperties()->SetMassProperties(JPH::EAllowedDOFs::All, *properties);
    return {};
}

core::VoidResult World::add_force(BodyId id, const Vector3& force_n, const Vector3& point_m) {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    if (!is_finite(force_n) || !is_finite(point_m)) {
        return core::fail(ErrorCode::NotFinite, "a force must be finite");
    }
    implementation_->system.GetBodyInterfaceNoLock().AddForce(JPH::BodyID(id.value), to_jolt(force_n),
                                                              JPH::RVec3(to_jolt(point_m)));
    return {};
}

core::VoidResult World::add_torque(BodyId id, const Vector3& torque_n_m) {
    if (!implementation_->knows(id)) {
        return core::fail(ErrorCode::InvalidArgument, "unknown body");
    }
    if (!is_finite(torque_n_m)) {
        return core::fail(ErrorCode::NotFinite, "a torque must be finite");
    }
    implementation_->system.GetBodyInterfaceNoLock().AddTorque(JPH::BodyID(id.value), to_jolt(torque_n_m));
    return {};
}

void World::set_gravity(const Vector3& acceleration_m_s2) {
    implementation_->system.SetGravity(to_jolt(acceleration_m_s2));
}

core::VoidResult World::step(double duration_s) {
    if (!(duration_s > 0.0) || !std::isfinite(duration_s)) {
        return core::fail(ErrorCode::OutOfRange, "a physics step needs a positive duration");
    }
    implementation_->contacts.begin_step();
    const JPH::EPhysicsUpdateError error = implementation_->system.Update(
        static_cast<float>(duration_s), 1, &implementation_->temporary_memory, &implementation_->jobs);
    if (error != JPH::EPhysicsUpdateError::None) {
        return core::fail(
            ErrorCode::ExternalLibraryFailure,
            std::format("the physics step overflowed a buffer (code {})", static_cast<unsigned>(error)));
    }
    return {};
}

std::optional<RayHit> World::cast_ray(const Vector3& origin_m, const Vector3& unit_direction,
                                      double max_distance_m, std::optional<BodyId> ignored) const {
    if (!is_finite(origin_m) || !is_finite(unit_direction) || !std::isfinite(max_distance_m)
        || !(max_distance_m > 0.0) || std::abs(math::norm(unit_direction) - 1.0) > 1e-6) {
        return std::nullopt;
    }
    const JPH::RRayCast ray(JPH::RVec3(to_jolt(origin_m)), to_jolt(max_distance_m * unit_direction));
    JPH::RayCastResult result;
    // Rationale: an invalid id matches no body, so the filter then lets every body through.
    const JPH::IgnoreSingleBodyFilter body_filter(ignored.has_value() ? JPH::BodyID(ignored->value)
                                                                      : JPH::BodyID());
    if (!implementation_->system.GetNarrowPhaseQueryNoLock().CastRay(ray, result, {}, {}, body_filter)) {
        return std::nullopt;
    }
    const JPH::RVec3 point = ray.GetPointOnRay(result.mFraction);
    const JPH::BodyLockRead lock(implementation_->system.GetBodyLockInterfaceNoLock(), result.mBodyID);
    if (!lock.Succeeded()) {
        return std::nullopt;
    }
    return RayHit{.body = {result.mBodyID.GetIndexAndSequenceNumber()},
                  .distance_m = max_distance_m * static_cast<double>(result.mFraction),
                  .point_m = from_jolt(JPH::Vec3(point)),
                  .normal = from_jolt(lock.GetBody().GetWorldSpaceSurfaceNormal(result.mSubShapeID2, point))};
}

std::span<const Contact> World::contacts_begun() const noexcept {
    return implementation_->contacts.begun();
}

bool World::touching(BodyId id) const noexcept {
    return implementation_->contacts.touching(id);
}

} // namespace helios::physics
