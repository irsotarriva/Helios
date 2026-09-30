#include "helios/ephemeris/chebyshev_ephemeris.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <utility>

namespace helios::ephemeris {

namespace {
using core::ErrorCode;
using math::Vector3;
} // namespace

ChebyshevEphemeris::ChebyshevEphemeris(const time::Epoch& start, const time::Interval& range,
                                       double record_duration_s, std::size_t record_count,
                                       std::size_t coefficients_per_axis,
                                       std::vector<double> coefficients_m) noexcept
    : start_(start), range_(range), record_duration_s_(record_duration_s), record_count_(record_count),
      coefficients_per_axis_(coefficients_per_axis), coefficients_m_(std::move(coefficients_m)) {
}

core::Result<ChebyshevEphemeris> ChebyshevEphemeris::make(const time::Epoch& start, double record_duration_s,
                                                          std::size_t coefficients_per_axis,
                                                          std::vector<double> coefficients_m) noexcept {
    if (!std::isfinite(record_duration_s) || record_duration_s <= 0.0) {
        return core::fail(ErrorCode::OutOfRange, "Chebyshev record duration must be positive and finite");
    }
    if (coefficients_per_axis == 0) {
        return core::fail(ErrorCode::InvalidArgument,
                          "Chebyshev series needs at least one coefficient per axis");
    }
    const std::size_t values_per_record = 3 * coefficients_per_axis;
    if (coefficients_m.empty() || coefficients_m.size() % values_per_record != 0) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("{} coefficients is not a whole number of records of {} values",
                                      coefficients_m.size(), values_per_record));
    }
    for (const double coefficient_m : coefficients_m) {
        if (!std::isfinite(coefficient_m)) {
            return core::fail(ErrorCode::NotFinite, "Chebyshev coefficients contain a non-finite value");
        }
    }
    const std::size_t record_count = coefficients_m.size() / values_per_record;
    const double span_s = static_cast<double>(record_count) * record_duration_s;

    return start.advanced_by(span_s)
        .and_then([&](const time::Epoch& end) { return time::Interval::make(start, end); })
        .transform([&](const time::Interval& range) {
            return ChebyshevEphemeris{start,
                                      range,
                                      record_duration_s,
                                      record_count,
                                      coefficients_per_axis,
                                      std::move(coefficients_m)};
        });
}

core::Result<ChebyshevEphemeris::Derivatives>
ChebyshevEphemeris::evaluate(const time::Epoch& instant) const noexcept {
    if (instant < range_.begin() || instant > range_.end()) {
        return core::fail(ErrorCode::OutOfRange, "epoch is outside the Chebyshev ephemeris coverage");
    }

    const double offset_from_start_s = time::seconds_between(start_, instant);
    const auto last_record_index = static_cast<double>(record_count_ - 1);
    auto record_index = static_cast<std::size_t>(
        std::clamp(std::floor(offset_from_start_s / record_duration_s_), 0.0, last_record_index));

    // Rationale: the offset inside the record is measured from the record's own start epoch.
    // Using `offset_from_start_s − index·duration` instead would inherit the ~1e-6 s rounding
    // of a 150-year offset — about a millimetre of lunar motion.
    const auto local_offset_s = [&](std::size_t index) -> core::Result<double> {
        return start_.advanced_by(static_cast<double>(index) * record_duration_s_)
            .transform([&](const time::Epoch& record_start) {
                return time::seconds_between(record_start, instant);
            });
    };
    core::Result<double> offset_in_record_s = local_offset_s(record_index);
    if (!offset_in_record_s) {
        return std::unexpected(offset_in_record_s.error());
    }
    // The floor above used a rounded offset; correct an off-by-one at a record boundary.
    if (*offset_in_record_s < 0.0 && record_index > 0) {
        --record_index;
        offset_in_record_s = local_offset_s(record_index);
    } else if (*offset_in_record_s > record_duration_s_ && record_index + 1 < record_count_) {
        ++record_index;
        offset_in_record_s = local_offset_s(record_index);
    }
    if (!offset_in_record_s) {
        return std::unexpected(offset_in_record_s.error());
    }

    // Map the record to τ ∈ [−1, 1]; dτ/dt = 2/duration.
    const double tau = 2.0 * (*offset_in_record_s / record_duration_s_) - 1.0;
    const double tau_rate_per_s = 2.0 / record_duration_s_;

    // Chebyshev recurrences for T_k(τ), T'_k(τ) and T''_k(τ).
    const std::size_t count = coefficients_per_axis_;
    const std::size_t record_base = record_index * 3 * count;
    std::array<double, 3> position{};
    std::array<double, 3> first{};
    std::array<double, 3> second{};

    double value_previous = 1.0;
    double value_current = tau;
    double slope_previous = 0.0;
    double slope_current = 1.0;
    double curvature_previous = 0.0;
    double curvature_current = 0.0;
    for (std::size_t k = 0; k < count; ++k) {
        double value = 1.0;
        double slope = 0.0;
        double curvature = 0.0;
        if (k == 1) {
            value = tau;
            slope = 1.0;
        } else if (k >= 2) {
            value = 2.0 * tau * value_current - value_previous;
            slope = 2.0 * value_current + 2.0 * tau * slope_current - slope_previous;
            curvature = 4.0 * slope_current + 2.0 * tau * curvature_current - curvature_previous;
            value_previous = value_current;
            value_current = value;
            slope_previous = slope_current;
            slope_current = slope;
            curvature_previous = curvature_current;
            curvature_current = curvature;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double coefficient_m = coefficients_m_[record_base + axis * count + k];
            position.at(axis) += coefficient_m * value;
            first.at(axis) += coefficient_m * slope;
            second.at(axis) += coefficient_m * curvature;
        }
    }

    const double tau_rate_squared = tau_rate_per_s * tau_rate_per_s;
    return Derivatives{
        .position_m = {position[0], position[1], position[2]},
        .velocity_m_s = Vector3{first[0], first[1], first[2]} * tau_rate_per_s,
        .acceleration_m_s2 = Vector3{second[0], second[1], second[2]} * tau_rate_squared,
    };
}

core::Result<orbital::StateVector> ChebyshevEphemeris::state_at(const time::Epoch& instant) const noexcept {
    return evaluate(instant).transform([](const Derivatives& derivatives) {
        return orbital::StateVector{.position_m = derivatives.position_m,
                                    .velocity_m_s = derivatives.velocity_m_s};
    });
}

core::Result<Vector3> ChebyshevEphemeris::acceleration_at(const time::Epoch& instant) const noexcept {
    return evaluate(instant).transform(
        [](const Derivatives& derivatives) { return derivatives.acceleration_m_s2; });
}

} // namespace helios::ephemeris
