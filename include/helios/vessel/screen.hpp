#ifndef HELIOS_VESSEL_SCREEN_HPP
#define HELIOS_VESSEL_SCREEN_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/math/vector3.hpp"
#include "helios/vessel/controls.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// A cockpit's screens (BRIEFING D46): displays drawn at their own resolution, as a multi-function
// display is, each showing one of its pages. A page is laid out from elements (text, readouts,
// bars, graphs, boxes and lines), each placed in the screen's pixels and bound to signals of the
// vessel's control bus, as a panel's indicators are. Which page is up is a signal too, set by
// ordinary controls (the buttons round the screen), an autopilot or a script: the screen holds
// no logic. The format is in data/parts/README.md.
namespace helios::vessel {

struct ScreenElement {
    enum class Type : std::uint8_t {
        Text,    // words that do not change
        Readout, // a signal's value, written out
        Bar,     // a signal's value as the filled part of a box
        Graph,   // a signal's value over the last `span_s` seconds
        Box,     // a frame, or a filled box
        Line,    // from `at_px` to `to_px`
    };
    enum class Align : std::uint8_t { Left, Centre, Right };

    Type type = Type::Text;
    // In the screen's pixels, from its top left corner: where the element's box starts (a text's
    // or a readout's top left, as aligned), and how big it is.
    std::array<double, 2> at_px{};
    std::array<double, 2> size_px{};
    std::array<double, 2> to_px{}; // a line's other end
    std::string text;              // a text's words; written before a readout's value
    std::string signal;            // a readout's, a bar's or a graph's
    // A bar's and a graph's range.
    double minimum = 0.0;
    double maximum = 1.0;
    // A readout's number is the value times `scale`, followed by `unit`; `decimals` fixed, or
    // as many as the number's size leaves room for.
    double scale = 1.0;
    std::string unit;
    std::optional<int> decimals;
    double height_px = 12.0; // of a text's or a readout's capitals
    Align align = Align::Left;
    double span_s = 60.0;                // what a graph shows of the past
    double line_px = 1.0;                // the width of a line, a frame, a graph's trace
    bool fill = false;                   // a box: filled rather than framed
    std::optional<math::Vector3> colour; // none: the screen's accent
};

struct ScreenPage {
    std::string name;
    std::vector<ScreenElement> elements;
};

struct Screen {
    std::string id; // unique in the cockpit, among its controls and indicators
    std::string label;
    std::vector<std::string> classes;
    PanelPlace place;                             // its centre, facing out of the glass
    std::array<double, 2> size_m{};               // of the glass, across and up
    std::array<std::uint16_t, 2> resolution_px{}; // across and down
    // The signal whose value (rounded, and kept within the pages) is the page shown; empty:
    // the first page, always.
    std::string page_signal;
    // The signal that powers the screen: dark below one half. Empty: always on.
    std::string power_signal;
    std::vector<ScreenPage> pages; // at least one
    LookPatch own_look;
    Look look; // resolved from the style sheets: the bezel, and the screen's accent
};

inline constexpr std::string_view k_screen_element = "screen";
inline constexpr std::uint16_t k_least_screen_px = 16;
inline constexpr std::uint16_t k_most_screen_px = 2048;

[[nodiscard]] core::Result<Screen> parse_screen(const core::TomlValue& table);

// The page shown for a value of the page signal: rounded, and kept within the pages.
[[nodiscard]] std::size_t page_shown(const Screen& screen, double page_value) noexcept;

} // namespace helios::vessel

#endif // HELIOS_VESSEL_SCREEN_HPP
