#include "helios/vessel/assembly.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <utility>

namespace helios::vessel {

namespace {

using core::ErrorCode;
using math::Matrix3;
using math::Vector3;

constexpr double k_orthonormality_tolerance = 1e-9;

[[nodiscard]] bool is_rotation(const Matrix3& matrix) noexcept {
    const Matrix3 product = matrix * math::transpose(matrix);
    const Matrix3 identity;
    for (std::size_t index = 0; index < product.elements.size(); ++index) {
        if (!(std::abs(product.elements.at(index) - identity.elements.at(index))
              < k_orthonormality_tolerance)) {
            return false;
        }
    }
    return true;
}

// The inertia about the origin of a point mass at `offset_m`: the parallel-axis term.
[[nodiscard]] Matrix3 displaced_point_inertia(double mass_kg, const Vector3& offset_m) noexcept {
    const double squared_m2 = math::squared_norm(offset_m);
    const std::array<double, 3> offset{offset_m.x, offset_m.y, offset_m.z};
    Matrix3 inertia;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            inertia(row, column) =
                mass_kg * ((row == column ? squared_m2 : 0.0) - offset.at(row) * offset.at(column));
        }
    }
    return inertia;
}

} // namespace

core::Result<std::size_t> Assembly::add_part(PartInstance part) {
    if (!part.datasheet) {
        return core::fail(ErrorCode::InvalidArgument, std::format("part '{}' has no datasheet", part.name));
    }
    if (part.name.empty() || part.name.contains('/') || find(part.name).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("part name '{}' is empty, duplicated or contains '/'", part.name));
    }
    if (parts_.empty() == part.parent.has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("part '{}': only the first part of a vessel has no parent", part.name));
    }
    if (part.parent.has_value() && *part.parent >= parts_.size()) {
        return core::fail(ErrorCode::OutOfRange, std::format("part '{}' has an unknown parent", part.name));
    }
    const Vector3& position_m = part.position_in_parent_m;
    if (!std::isfinite(position_m.x) || !std::isfinite(position_m.y) || !std::isfinite(position_m.z)
        || !is_rotation(part.orientation_in_parent)) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("part '{}' has an invalid position or orientation", part.name));
    }
    PartPose pose;
    if (part.parent.has_value()) {
        const PartPose& parent = poses_[*part.parent];
        pose.position_m = parent.position_m + parent.orientation * part.position_in_parent_m;
        pose.orientation = parent.orientation * part.orientation_in_parent;
    }
    parts_.push_back(std::move(part));
    poses_.push_back(pose);
    return parts_.size() - 1;
}

core::Result<std::size_t> Assembly::find(std::string_view name) const {
    const auto match = std::ranges::find(parts_, name, &PartInstance::name);
    if (match == parts_.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("the vessel has no part named '{}'", name));
    }
    return static_cast<std::size_t>(match - parts_.begin());
}

std::vector<std::size_t> Assembly::subtree(std::size_t part) const {
    // Parents always precede their children, so one pass in index order finds every descendant.
    std::vector<bool> inside(parts_.size(), false);
    std::vector<std::size_t> members;
    for (std::size_t index = part; index < parts_.size(); ++index) {
        const std::optional<std::size_t>& parent = parts_[index].parent;
        if (index == part || (parent.has_value() && inside[*parent])) {
            inside[index] = true;
            members.push_back(index);
        }
    }
    return members;
}

core::Result<MassProperties> Assembly::mass_properties(std::span<const double> part_masses_kg) const {
    if (part_masses_kg.size() != parts_.size() || parts_.empty()) {
        return core::fail(ErrorCode::InvalidArgument, "one mass per part of a non-empty vessel is needed");
    }
    MassProperties properties;
    Vector3 moment_kg_m;
    std::vector<Vector3> centres_m(parts_.size());
    for (std::size_t index = 0; index < parts_.size(); ++index) {
        centres_m[index] =
            poses_[index].position_m + poses_[index].orientation * parts_[index].datasheet->centre_of_mass_m;
        properties.mass_kg += part_masses_kg[index];
        moment_kg_m += part_masses_kg[index] * centres_m[index];
    }
    if (!(properties.mass_kg > 0.0)) {
        return core::fail(ErrorCode::OutOfRange, "a vessel needs a positive mass");
    }
    properties.centre_of_mass_m = moment_kg_m / properties.mass_kg;
    Matrix3 inertia{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    for (std::size_t index = 0; index < parts_.size(); ++index) {
        const double mass_kg = part_masses_kg[index];
        const Vector3& gyration_m2 = parts_[index].datasheet->gyration_m2;
        const Matrix3 own{{mass_kg * gyration_m2.x, 0.0, 0.0, 0.0, mass_kg * gyration_m2.y, 0.0, 0.0, 0.0,
                           mass_kg * gyration_m2.z}};
        const Matrix3& orientation = poses_[index].orientation;
        const Matrix3 own_in_vessel = orientation * own * math::transpose(orientation);
        const Matrix3 displaced =
            displaced_point_inertia(mass_kg, centres_m[index] - properties.centre_of_mass_m);
        for (std::size_t element = 0; element < inertia.elements.size(); ++element) {
            inertia.elements.at(element) +=
                own_in_vessel.elements.at(element) + displaced.elements.at(element);
        }
    }
    properties.inertia_kg_m2 = inertia;
    return properties;
}

core::Result<AssemblySplit> Assembly::split_at(std::size_t part) const {
    if (part == 0 || part >= parts_.size()) {
        return core::fail(ErrorCode::OutOfRange, "only a part other than the root can be detached");
    }
    AssemblySplit split;
    split.separated_parts = subtree(part);
    std::vector<std::optional<std::size_t>> new_index(parts_.size());
    for (std::size_t index = 0; index < parts_.size(); ++index) {
        const bool leaves = std::ranges::binary_search(split.separated_parts, index);
        Assembly& destination = leaves ? split.separated : split.kept;
        PartInstance instance = parts_[index];
        if (index == part) {
            // The detached part becomes the root of its own vessel, which keeps its axes.
            instance.parent.reset();
            instance.position_in_parent_m = {};
            instance.orientation_in_parent = {};
        } else if (instance.parent.has_value()) {
            instance.parent = new_index[*instance.parent];
        }
        const auto added = destination.add_part(std::move(instance));
        if (!added) {
            return std::unexpected(added.error());
        }
        new_index[index] = *added;
        if (!leaves) {
            split.kept_parts.push_back(index);
        }
    }
    return split;
}

core::Result<Matrix3> rotation_about(const Vector3& axis, double angle_rad) {
    const double length = math::norm(axis);
    if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(angle_rad)) {
        return core::fail(ErrorCode::InvalidArgument, "a rotation needs a non-zero axis and a finite angle");
    }
    // Rodrigues' rotation formula.
    const Vector3 unit = axis / length;
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    const double versine = 1.0 - cosine;
    return Matrix3{{cosine + unit.x * unit.x * versine, unit.x * unit.y * versine - unit.z * sine,
                    unit.x * unit.z * versine + unit.y * sine, unit.y * unit.x * versine + unit.z * sine,
                    cosine + unit.y * unit.y * versine, unit.y * unit.z * versine - unit.x * sine,
                    unit.z * unit.x * versine - unit.y * sine, unit.z * unit.y * versine + unit.x * sine,
                    cosine + unit.z * unit.z * versine}};
}

} // namespace helios::vessel
