#ifndef HELIOS_DYNAMICS_GRAVITY_MODEL_HPP
#define HELIOS_DYNAMICS_GRAVITY_MODEL_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/frames/frame_tree.hpp"
#include "helios/math/vector3.hpp"
#include "helios/time/epoch.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace helios::dynamics {

struct GravityOptions {
    // Barnes–Hut opening angle θ (BRIEFING §5.2): a subsystem whose extent / distance < θ is one
    // point mass at its barycentre. Smaller is more accurate; 0 opens everything.
    double opening_angle = 0.5;
};

// One gravity source the tree walk used, for diagnostics and tests.
struct GravitySource {
    frames::FrameId frame;
    double gravitational_parameter_m3_s2 = 0.0;
    bool is_aggregate = false; // an unopened subsystem treated as its barycentre
};

// Tree-code gravity over the frame hierarchy, for massless vessels.
//
// A vessel anchored on body A (its domain) moves in A's non-rotating frame, which accelerates.
// Its equation of motion there is r̈ = −μ_A r/|r|³ + a_perturbing with
//   a_perturbing = Σ_k μ_k (d_k − r)/|d_k − r|³ − a_A      (BRIEFING §4.3)
// where the sum runs over the sources selected by the opening criterion (never A itself) and
// a_A is A's acceleration relative to the inertial root, taken from the ephemerides.
class GravityModel {
public:
    // `extent_epoch` is where subsystem extents (max child distance × 1.25) are measured; the
    // opening criterion only needs their order of magnitude. `tree` and `catalog` must outlive
    // the model.
    [[nodiscard]] static core::Result<GravityModel> make(const frames::FrameTree& tree,
                                                         const bodies::BodyCatalog& catalog,
                                                         GravityOptions options,
                                                         const time::Epoch& extent_epoch);

    [[nodiscard]] core::Result<math::Vector3>
    perturbing_acceleration_m_s2(bodies::BodyId anchor, const math::Vector3& position_m,
                                 const time::Epoch& instant) const noexcept;

    // Central term plus perturbations.
    [[nodiscard]] core::Result<math::Vector3>
    total_acceleration_m_s2(bodies::BodyId anchor, const math::Vector3& position_m,
                            const time::Epoch& instant) const noexcept;

    // The sources the walk selects for this configuration (excludes the anchor itself).
    [[nodiscard]] core::Result<std::vector<GravitySource>> selected_sources(bodies::BodyId anchor,
                                                                            const math::Vector3& position_m,
                                                                            const time::Epoch& instant) const;

    [[nodiscard]] const frames::FrameTree& tree() const noexcept { return tree_.get(); }
    [[nodiscard]] const bodies::BodyCatalog& catalog() const noexcept { return catalog_.get(); }

private:
    struct Node {
        double own_gravitational_parameter_m3_s2 = 0.0;
        double subtree_gravitational_parameter_m3_s2 = 0.0;
        double extent_m = 0.0;
        std::vector<frames::FrameId> children;
    };

    GravityModel(const frames::FrameTree& tree, const bodies::BodyCatalog& catalog, GravityOptions options,
                 std::vector<Node> nodes) noexcept
        : tree_(tree), catalog_(catalog), options_(options), nodes_(std::move(nodes)) {}

    template <typename Visitor>
    [[nodiscard]] core::VoidResult walk(frames::FrameId root, frames::FrameId anchor_frame,
                                        const math::Vector3& position_m, const time::Epoch& instant,
                                        const Visitor& visit) const;

    [[nodiscard]] bool is_ancestor_or_self(frames::FrameId ancestor, frames::FrameId frame) const noexcept;

    std::reference_wrapper<const frames::FrameTree> tree_;
    std::reference_wrapper<const bodies::BodyCatalog> catalog_;
    GravityOptions options_;
    std::vector<Node> nodes_;
};

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_GRAVITY_MODEL_HPP
