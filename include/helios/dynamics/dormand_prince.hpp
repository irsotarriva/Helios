#ifndef HELIOS_DYNAMICS_DORMAND_PRINCE_HPP
#define HELIOS_DYNAMICS_DORMAND_PRINCE_HPP

#include "helios/core/error.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>

namespace helios::dynamics {

// A first-order state: position (3) and velocity (3), or their Encke deviations.
using State6 = std::array<double, 6>;

// y' = f(t, y); may fail (e.g. an ephemeris evaluated outside its coverage).
template <typename Function>
concept StateDerivative = requires(const Function& function, double time_s, const State6& state) {
    { function(time_s, state) } -> std::same_as<core::Result<State6>>;
};

struct DormandPrinceStep {
    State6 state;            // 5th-order solution at t + h
    State6 derivative;       // f(t + h, state): first-same-as-last for the next step
    double error_norm = 0.0; // RMS of the embedded error estimate / `scale`; accept if ≤ 1
};

// Dormand–Prince 5(4) Butcher tableau (Dormand & Prince 1980; Hairer, Nørsett & Wanner,
// "Solving ODEs I", Table 5.2). Row i of k_stage_weights builds the input of stage i + 1.
inline constexpr std::array<double, 7> k_dormand_prince_nodes{0.0,       1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0,
                                                              8.0 / 9.0, 1.0,       1.0};
inline constexpr std::array<std::array<double, 6>, 6> k_dormand_prince_stage_weights{{
    {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0},
    {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0},
    {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0, 0.0},
    {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0, 0.0},
    {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0}, // = 5th-order solution
}};
// 5th-order minus embedded 4th-order weights, for all seven stages (the 7th is FSAL).
inline constexpr std::array<double, 7> k_dormand_prince_error_weights{
    71.0 / 57600.0, 0.0, -71.0 / 16695.0, 71.0 / 1920.0, -17253.0 / 339200.0, 22.0 / 525.0, -1.0 / 40.0};

// One Dormand–Prince 5(4) step. `derivative` must be f(time_s, state).
template <StateDerivative Function>
[[nodiscard]] core::Result<DormandPrinceStep>
dormand_prince_step(const Function& function, double time_s, const State6& state, const State6& derivative,
                    double step_s, const State6& scale) noexcept {
    std::array<State6, 7> slopes{};
    slopes[0] = derivative;
    for (std::size_t stage = 1; stage < slopes.size(); ++stage) {
        State6 stage_state = state;
        const std::array<double, 6>& weights = k_dormand_prince_stage_weights.at(stage - 1);
        for (std::size_t previous = 0; previous < stage; ++previous) {
            for (std::size_t component = 0; component < stage_state.size(); ++component) {
                stage_state.at(component) +=
                    step_s * weights.at(previous) * slopes.at(previous).at(component);
            }
        }
        auto slope = function(time_s + k_dormand_prince_nodes.at(stage) * step_s, stage_state);
        if (!slope) {
            return std::unexpected(slope.error());
        }
        slopes.at(stage) = *slope;
        if (stage == slopes.size() - 1) {
            // The last stage is evaluated at the 5th-order solution itself (first same as last).
            double squared_sum = 0.0;
            for (std::size_t component = 0; component < state.size(); ++component) {
                double error = 0.0;
                for (std::size_t index = 0; index < slopes.size(); ++index) {
                    error += k_dormand_prince_error_weights.at(index) * slopes.at(index).at(component);
                }
                const double scaled = step_s * error / scale.at(component);
                squared_sum += scaled * scaled;
            }
            return DormandPrinceStep{.state = stage_state,
                                     .derivative = slopes.at(stage),
                                     .error_norm =
                                         std::sqrt(squared_sum / static_cast<double>(state.size()))};
        }
    }
    return core::fail(core::ErrorCode::Unknown, "unreachable: Dormand–Prince stage loop ended early");
}

// Standard step-size controller for a 5th-order method: 0.9·err^(−1/5), clamped to [0.2, 5].
[[nodiscard]] inline double next_step_factor(double error_norm) noexcept {
    if (error_norm == 0.0) {
        return 5.0;
    }
    return std::clamp(0.9 * std::pow(error_norm, -0.2), 0.2, 5.0);
}

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_DORMAND_PRINCE_HPP
