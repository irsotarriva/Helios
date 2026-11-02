#ifndef HELIOS_RENDER_STROKE_FONT_HPP
#define HELIOS_RENDER_STROKE_FONT_HPP

#include "helios/math/vector3.hpp"
#include "helios/render/scene_geometry.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

// Text drawn in the world (BRIEFING D38): the labels and readings of a cockpit's panels, as
// painted or lit lettering on the panel itself, so that it is seen from wherever the eye is and
// in a headset as on a flat screen. The letters are strokes on a small grid, the plain, narrow
// lettering of an instrument panel, made here rather than taken from a font file.
namespace helios::render {

// A straight stroke of a letter, in cap heights: the baseline is y = 0, the capitals reach
// y = 1, and the text starts at x = 0. A stroke of no length is a dot.
struct TextStroke {
    float x0 = 0.0F;
    float y0 = 0.0F;
    float x1 = 0.0F;
    float y1 = 0.0F;
};

// From one letter to the next, and the width of a letter, in cap heights. Every letter is as wide.
inline constexpr float k_text_advance = 6.0F / 7.0F;
inline constexpr float k_letter_width = 4.0F / 7.0F;
// The width of a stroke, in cap heights.
inline constexpr float k_stroke_width = 0.11F;

// The width of a line of text, in cap heights, from the left of its first letter to the right
// of its last.
[[nodiscard]] float text_width(std::string_view text) noexcept;

// The strokes of a line of text. Printable ASCII is drawn; anything else (a byte of a UTF-8
// sequence included) is drawn as '?'.
[[nodiscard]] std::vector<TextStroke> text_strokes(std::string_view text);

// Where a line of text goes: its centre (the middle of the line, halfway up the capitals) and the
// directions of its right and its up, each as long as a cap height, all in camera space.
struct TextPlacement {
    math::Vector3 centre_m;
    math::Vector3 right_m;
    math::Vector3 up_m;
};

// Appends the text to a triangle list, two triangles to a stroke, in one colour.
void append_text(std::vector<LineVertex>& triangles, std::string_view text, const TextPlacement& placement,
                 std::uint32_t abgr);

} // namespace helios::render

#endif // HELIOS_RENDER_STROKE_FONT_HPP
