#ifndef HELIOS_DYNAMICS_TRAJECTORY_HPP
#define HELIOS_DYNAMICS_TRAJECTORY_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/math/vector3.hpp"
#include "helios/time/epoch.hpp"

#include <cstddef>
#include <vector>

namespace helios::dynamics {

struct TrajectorySample {
    time::Epoch epoch;
    math::Vector3 position_m; // in the segment domain's non-rotating frame
};

// A run of samples in one domain. Drawn relative to the domain body's *current* position, as in
// any map view: the line shows the path relative to the body, not through inertial space.
struct TrajectorySegment {
    bodies::BodyId domain;
    std::vector<TrajectorySample> samples;
};

struct PredictionOptions {
    double horizon_s = 0.0;
    // Sampling density: at most this much true anomaly between samples (dν/dt = |h|/r²).
    double max_true_anomaly_step_rad = 0.035;
    double min_step_s = 1.0;
    std::size_t max_samples = 4096;
};

// Samples the future trajectory of `propagator` from `from` to `from + horizon`, running a copy so
// the caller's propagator is untouched. Because propagation is warp-invariant and deterministic
// (D14), this is exactly the path the vessel will follow, scheduled impulses included, and not
// an approximation of it. Stops early, without error, when the vessel reaches the surface of its
// domain body or the propagator fails after at least one sample (e.g. a collision singularity).
[[nodiscard]] core::Result<std::vector<TrajectorySegment>>
predict_trajectory(EnckePropagator propagator, const time::Epoch& from, const PredictionOptions& options);

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_TRAJECTORY_HPP
