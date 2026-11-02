#include "helios/render/stroke_font.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace helios::render {

namespace {

// The letters on a grid four wide and nine high: the baseline at 2, the capitals and the
// ascenders at 9, the small letters at 6, the descenders down to 0. Each letter is strokes
// separated by spaces; a stroke is the points it passes through, each as two digits, x and y.
// Curves are cut at the corners, as a stencil or a panel engraver would.
constexpr std::string_view k_unknown = "08193948472524 2222"; // '?'
constexpr std::array<std::string_view, 95> k_glyphs{
    "",                                 // space
    "2924 2222",                        // !
    "1917 3937",                        // "
    "1218 3238 0444 0646",              // #
    "483919080716364543321203 2921",    // $
    "0249 0818170708 3343423233",       // %
    k_unknown,                          // &
    "2927",                             // '
    "39171331",                         // (
    "19373311",                         // )
    "2327 0644 0446",                   // *
    "0545 2327",                        // +
    "2311",                             // ,
    "1535",                             // -
    "2222",                             // .
    "0249",                             // /
    "123243483919080312 1437",          // 0
    "182922 1232",                      // 1
    "08193948460242",                   // 2
    "0819394846354443321203 1535",      // 3
    "32390444",                         // 4
    "4909063645433202",                 // 5
    "39190803123243453606",             // 6
    "09494822",                         // 7
    "16070819394847361605031232434536", // 8
    "12324348391908061545",             // 9
    "2323 2626",                        // :
    "2626 2311",                        // ;
    "470543",                           // <
    "0444 0646",                        // =
    "074503",                           // >
    k_unknown,                          // ?
    k_unknown,                          // @
    "0206294642 0444",                  // A
    "02093948473606 3645433202",        // B
    "4839190803123243",                 // C
    "02092947442202",                   // D
    "49090242 0636",                    // E
    "490902 0636",                      // F
    "48391908031232434525",             // G
    "0209 4942 0646",                   // H
    "1939 2922 1232",                   // I
    "494332120304",                     // J
    "0209 4905 1642",                   // K
    "090242",                           // L
    "0209264942",                       // M
    "02094249",                         // N
    "123243483919080312",               // O
    "02093948463505",                   // P
    "123243483919080312 2441",          // Q
    "02093948463505 2542",              // R
    "483919080716364543321203",         // S
    "0949 2922",                        // T
    "090312324349",                     // U
    "092249",                           // V
    "0912253249",                       // W
    "0942 4902",                        // X
    "092649 2622",                      // Y
    "09490242",                         // Z
    "39191131",                         // [
    "0942",                             // backslash
    "19393111",                         // ]
    "062946",                           // ^
    "0141",                             // _
    "1928",                             // `
    "4642 4536160503123243",            // a
    "0902 0516364543321203",            // b
    "4536160503123243",                 // c
    "4942 4536160503123243",            // d
    "040445361605031242",               // e
    "4839291812 0636",                  // f
    "4641301001 4536160503123243",      // g
    "0902 0516364542",                  // h
    "162622 2828",                      // i
    "26211000 2828",                    // j
    "0902 3603 1442",                   // k
    "19292332",                         // l
    "0602 05162522 25364542",           // m
    "0602 0516364542",                  // n
    "123243453616050312",               // o
    "0600 0516364543321203",            // p
    "4640 4536160503123243",            // q
    "0602 042646",                      // r
    "45361605143443321203",             // s
    "18132232 0636",                    // t
    "0603123243 4642",                  // u
    "062246",                           // v
    "0612243246",                       // w
    "0642 4602",                        // x
    "0622 4610",                        // y
    "06460242",                         // z
    k_unknown,                          // {
    "2921",                             // |
    k_unknown,                          // }
    "05163546",                         // ~
};

constexpr float k_grid_per_cap = 7.0F; // grid steps from the baseline to the top of a capital
constexpr float k_baseline = 2.0F;     // on the grid

[[nodiscard]] std::string_view glyph(char letter) noexcept {
    const auto code = static_cast<unsigned char>(letter);
    if (code < 32U || code > 126U) {
        return k_unknown;
    }
    return k_glyphs[code - 32U];
}

[[nodiscard]] float grid_x(char digit, float left) noexcept {
    return left + (static_cast<float>(digit - '0') / k_grid_per_cap);
}

[[nodiscard]] float grid_y(char digit) noexcept {
    return (static_cast<float>(digit - '0') - k_baseline) / k_grid_per_cap;
}

} // namespace

float text_width(std::string_view text) noexcept {
    if (text.empty()) {
        return 0.0F;
    }
    return (static_cast<float>(text.size() - 1U) * k_text_advance) + k_letter_width;
}

std::vector<TextStroke> text_strokes(std::string_view text) {
    std::vector<TextStroke> strokes;
    float left = 0.0F;
    for (const char letter : text) {
        const std::string_view shape = glyph(letter);
        std::size_t start = 0;
        while (start < shape.size()) {
            std::size_t end = shape.find(' ', start);
            if (end == std::string_view::npos) {
                end = shape.size();
            }
            const std::string_view points = shape.substr(start, end - start);
            if (points.size() == 2U) {
                const float x = grid_x(points[0], left);
                const float y = grid_y(points[1]);
                strokes.push_back(TextStroke{.x0 = x, .y0 = y, .x1 = x, .y1 = y});
            }
            for (std::size_t at = 2; at + 1U < points.size(); at += 2U) {
                strokes.push_back(TextStroke{.x0 = grid_x(points[at - 2U], left),
                                             .y0 = grid_y(points[at - 1U]),
                                             .x1 = grid_x(points[at], left),
                                             .y1 = grid_y(points[at + 1U])});
            }
            start = end + 1U;
        }
        left += k_text_advance;
    }
    return strokes;
}

void append_text(std::vector<LineVertex>& triangles, std::string_view text, const TextPlacement& placement,
                 std::uint32_t abgr) {
    const double half_width = 0.5 * static_cast<double>(text_width(text));
    const double half_stroke = 0.5 * static_cast<double>(k_stroke_width);
    const auto vertex = [&](double x, double y) {
        const math::Vector3 at_m =
            placement.centre_m + (x - half_width) * placement.right_m + (y - 0.5) * placement.up_m;
        return LineVertex{
            .position = {static_cast<float>(at_m.x), static_cast<float>(at_m.y), static_cast<float>(at_m.z)},
            .abgr = abgr};
    };
    for (const TextStroke& stroke : text_strokes(text)) {
        const double x0 = stroke.x0;
        const double y0 = stroke.y0;
        double along_x = static_cast<double>(stroke.x1) - x0;
        double along_y = static_cast<double>(stroke.y1) - y0;
        const double length = std::hypot(along_x, along_y);
        // A dot is a square, as a stroke of no length with square ends.
        along_x = length > 0.0 ? along_x / length : 1.0;
        along_y = length > 0.0 ? along_y / length : 0.0;
        // Each end goes on by half a stroke, so that strokes meeting at a corner close it.
        const double ax = x0 - (half_stroke * along_x);
        const double ay = y0 - (half_stroke * along_y);
        const double bx = stroke.x1 + (half_stroke * along_x);
        const double by = stroke.y1 + (half_stroke * along_y);
        const double across_x = -half_stroke * along_y;
        const double across_y = half_stroke * along_x;
        const LineVertex a_left = vertex(ax + across_x, ay + across_y);
        const LineVertex a_right = vertex(ax - across_x, ay - across_y);
        const LineVertex b_left = vertex(bx + across_x, by + across_y);
        const LineVertex b_right = vertex(bx - across_x, by - across_y);
        triangles.insert(triangles.end(), {a_right, b_right, b_left, a_right, b_left, a_left});
    }
}

} // namespace helios::render
