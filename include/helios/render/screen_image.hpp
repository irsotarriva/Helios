#ifndef HELIOS_RENDER_SCREEN_IMAGE_HPP
#define HELIOS_RENDER_SCREEN_IMAGE_HPP

#include "helios/render/scene_geometry.hpp"
#include "helios/sim/scene_snapshot.hpp"
#include "helios/time/epoch.hpp"
#include "helios/vessel/screen.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// What a cockpit's screen shows (BRIEFING D46), drawn in its own pixels. GPU-free, like the rest
// of the flight geometry: a page goes in, triangles in the screen's pixels come out, and the
// client draws them into the screen's texture.
namespace helios::render {

// A signal's value at a time, as a graph remembers it.
struct GraphSample {
    double time_s = 0.0; // from whenever the memory began
    double value = 0.0;
};

// Which graph of which screen: a vessel's part, its cockpit's screen, a page and an element.
struct GraphKey {
    sim::VesselId vessel;
    std::size_t part = 0;
    std::size_t screen = 0;
    std::size_t page = 0;
    std::size_t element = 0;

    [[nodiscard]] friend bool operator==(const GraphKey&, const GraphKey&) = default;
};

// What the screens' graphs remember of their signals: the screens' own memory, as a display
// computer has one. Whoever draws keeps it and gives it every snapshot. Every graph remembers,
// on whatever page, so a page brought up shows its past.
class ScreenMemory {
public:
    // The most samples a graph keeps; they are spread over its span.
    static constexpr std::size_t k_samples = 240;

    // Takes the values of the graphs' signals on the screens of every vessel with a cockpit. A
    // snapshot earlier than the last forgets everything.
    void record(const sim::SceneSnapshot& snapshot);

    // Oldest first; empty for a graph not remembered.
    [[nodiscard]] std::span<const GraphSample> samples(const GraphKey& key) const noexcept;
    // When the last snapshot was, on the samples' clock.
    [[nodiscard]] double now_s() const noexcept { return now_s_; }

private:
    struct Trace {
        GraphKey key;
        std::vector<GraphSample> samples;
    };

    std::vector<Trace> traces_;
    std::optional<time::Epoch> origin_;
    double now_s_ = 0.0;
};

// A screen's picture: its pixels' background and what is drawn over it.
struct ScreenImage {
    std::uint16_t width_px = 0;
    std::uint16_t height_px = 0;
    bool on = false;                   // powered, and with every signal it needs to show a page
    std::uint32_t background = 0;      // abgr
    std::vector<LineVertex> triangles; // positions in pixels from the top left (z = 0), unlit
};

// The page the screen shows now, drawn from `vessel`'s signals. `memory` (if any) has the graphs'
// past, under `vessel`'s id, `part` and `screen`.
[[nodiscard]] ScreenImage draw_screen(const vessel::Screen& screen, const sim::VesselView& vessel,
                                      const ScreenMemory* memory = nullptr, std::size_t part = 0,
                                      std::size_t screen_index = 0);

} // namespace helios::render

#endif // HELIOS_RENDER_SCREEN_IMAGE_HPP
