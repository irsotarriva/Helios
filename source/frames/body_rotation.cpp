#include "helios/frames/body_rotation.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace helios::frames {

namespace {

using core::ErrorCode;

constexpr double k_arcsec_to_rad = std::numbers::pi / (180.0 * 3600.0);
constexpr double k_j2000_obliquity_rad = 84'381.448 * k_arcsec_to_rad;

} // namespace

math::Matrix3 icrf_to_j2000_ecliptic() noexcept {
    return math::frame_rotation_x(k_j2000_obliquity_rad);
}

core::Result<BodyRotation> BodyRotation::make(const RotationElements& elements,
                                              const math::Matrix3& reference_to_universe) noexcept {
    const std::array<double, 6> values{
        elements.pole_right_ascension_rad, elements.pole_right_ascension_rate_rad_s,
        elements.pole_declination_rad,     elements.pole_declination_rate_rad_s,
        elements.prime_meridian_rad,       elements.rotation_rate_rad_s};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return core::fail(ErrorCode::NotFinite, "rotation elements contain a non-finite value");
        }
    }
    return BodyRotation{elements, reference_to_universe};
}

math::Matrix3 BodyRotation::universe_to_body_fixed(const time::Epoch& instant) const noexcept {
    const double elapsed_s = time::seconds_between(time::Epoch{}, instant);
    const double right_ascension_rad =
        elements_.pole_right_ascension_rad + elements_.pole_right_ascension_rate_rad_s * elapsed_s;
    const double declination_rad =
        elements_.pole_declination_rad + elements_.pole_declination_rate_rad_s * elapsed_s;
    // TODO: W·Δt loses sub-arcsecond precision after ~1e4 years of Earth rotation; split the product.
    const double prime_meridian_rad = std::remainder(
        elements_.prime_meridian_rad + elements_.rotation_rate_rad_s * elapsed_s, orbital::k_two_pi);

    // IAU convention: R3(W) · R1(90° − δ₀) · R3(90° + α₀), from the reference (ICRF) axes.
    const math::Matrix3 reference_to_body =
        math::frame_rotation_z(prime_meridian_rad)
        * math::frame_rotation_x(RotationElements::k_half_pi - declination_rad)
        * math::frame_rotation_z(RotationElements::k_half_pi + right_ascension_rad);
    return reference_to_body * math::transpose(reference_to_universe_);
}

math::Vector3 BodyRotation::angular_velocity_rad_s(const time::Epoch& instant) const noexcept {
    // The body's +z axis (the pole) expressed in universe axes is the third row of R.
    const math::Matrix3 rotation = universe_to_body_fixed(instant);
    const math::Vector3 pole_unit{rotation(2, 0), rotation(2, 1), rotation(2, 2)};
    return pole_unit * elements_.rotation_rate_rad_s;
}

orbital::StateVector BodyRotation::to_body_fixed(const orbital::StateVector& inertial,
                                                 const time::Epoch& instant) const noexcept {
    const math::Matrix3 rotation = universe_to_body_fixed(instant);
    const math::Vector3 omega_rad_s = angular_velocity_rad_s(instant);
    return orbital::StateVector{
        .position_m = rotation * inertial.position_m,
        .velocity_m_s = rotation * (inertial.velocity_m_s - math::cross(omega_rad_s, inertial.position_m))};
}

orbital::StateVector BodyRotation::from_body_fixed(const orbital::StateVector& body_fixed,
                                                   const time::Epoch& instant) const noexcept {
    const math::Matrix3 inverse = math::transpose(universe_to_body_fixed(instant));
    const math::Vector3 omega_rad_s = angular_velocity_rad_s(instant);
    const math::Vector3 position_m = inverse * body_fixed.position_m;
    return orbital::StateVector{.position_m = position_m,
                                .velocity_m_s =
                                    inverse * body_fixed.velocity_m_s + math::cross(omega_rad_s, position_m)};
}

double BodyRotation::sidereal_period_s() const noexcept {
    return orbital::k_two_pi / std::abs(elements_.rotation_rate_rad_s);
}

SphericalPosition to_spherical(const math::Vector3& body_fixed_position_m) noexcept {
    const double radius_m = math::norm(body_fixed_position_m);
    return SphericalPosition{
        .latitude_rad =
            std::atan2(body_fixed_position_m.z, std::hypot(body_fixed_position_m.x, body_fixed_position_m.y)),
        .longitude_rad = std::atan2(body_fixed_position_m.y, body_fixed_position_m.x),
        .radius_m = radius_m};
}

} // namespace helios::frames
