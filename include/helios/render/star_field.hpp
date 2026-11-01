#ifndef HELIOS_RENDER_STAR_FIELD_HPP
#define HELIOS_RENDER_STAR_FIELD_HPP

#include "helios/math/vector3.hpp"
#include "helios/render/scene_geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// The stars behind everything (BRIEFING D37): a fixed sky, the same in every run, so that a
// pilot facing away from every vessel and body still sees something, and sees it turn. They are
// made up, not a catalogue of real stars (which would be a data source to approve), but drawn
// the way the sky looks: few bright stars and many faint ones, crowded along the Milky Way.
namespace helios::render {

struct Star {
    math::Vector3 direction; // unit, universe axes (the J2000 ecliptic for the stock Solar System)
    float brightness = 0.0F; // 0 to 1
    std::uint32_t abgr = 0;  // its colour at that brightness
};

inline constexpr std::size_t k_star_count = 4000;

// The same stars for the same count and seed, on every platform.
[[nodiscard]] std::vector<Star> make_star_field(std::size_t count = k_star_count, std::uint64_t seed = 1);

// Each star as a short line of about a pixel, far away along its direction: a line list for a
// camera at the origin, as the map renderer draws (rotation-only view, reversed depth).
[[nodiscard]] std::vector<LineVertex> star_lines(std::span<const Star> stars);

// How far away the stars are drawn: beyond everything in the Solar System, so that whatever is
// drawn in front of them is nearer, and at a depth that is still above the cleared value.
inline constexpr double k_star_distance_m = 1.0e14;
// The length of a star's line, as an angle: about two pixels at 1080 lines over 0.9 rad.
inline constexpr double k_star_length_rad = 1.7e-3;

} // namespace helios::render

#endif // HELIOS_RENDER_STAR_FIELD_HPP
