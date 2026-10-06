#ifndef HELIOS_APPS_HELIOS_MAP_VIEW_UI_HPP
#define HELIOS_APPS_HELIOS_MAP_VIEW_UI_HPP

#include "helios/render/camera.hpp"
#include "helios/render/flight_geometry.hpp"
#include "helios/render/scene_geometry.hpp"
#include "helios/sim/scene_snapshot.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "flight_controls.hpp"

namespace helios::app {

struct BurnRequest {
    sim::VesselId vessel;
    enum class When : std::uint8_t { InSeconds, NextApoapsis, NextPeriapsis } when = When::InSeconds;
    double delay_s = 0.0;
    double prograde_m_s = 0.0;
    double normal_m_s = 0.0;
    double radial_out_m_s = 0.0;
};

// What the player asked for this frame.
struct UiActions {
    std::optional<sim::Focus> focus;
    std::optional<std::size_t> warp_level;
    std::optional<bool> paused;
    std::optional<BurnRequest> burn;
    std::vector<SignalCommand> commands;
};

// Widget state that persists between frames.
struct UiState {
    float prograde_m_s = 0.0F;
    float normal_m_s = 0.0F;
    float radial_out_m_s = 0.0F;
    float burn_delay_s = 60.0F;
    bool show_help = true;
    bool show_panels = true; // the focus and vessel panels (in the cockpit they cover the view)
    // Levers being dragged, by "<vessel>:<signal>": the snapshot lags the hand by a tick or two.
    std::unordered_map<std::string, float> held_levers;
};

// The overlay's projection, in ImGui display units (points; pixels × framebuffer scale on HiDPI).
struct ScreenProjection {
    render::Matrix4f view_projection{};
    float width_px = 0.0F;
    float height_px = 0.0F;
    float focal_length_px = 0.0F; // display units per unit of (size / distance)
};

// Labels and markers over the 3-D view, then the panels. Call between ImGui::NewFrame and
// ImGui::Render.
[[nodiscard]] UiActions draw_map_view_ui(const sim::SceneSnapshot& snapshot,
                                         const render::FrameGeometry& geometry,
                                         const math::Vector3& camera_from_focus_m,
                                         const ScreenProjection& screen, UiState& state,
                                         double frames_per_second);

// The same for the flight view: the names and readings of the cockpit's instruments (drawn
// over the picture for now; they belong on the panel, BRIEFING §12.1 rule 3), markers for
// other vessels, then the panels. Seen from inside, the picture also says what the interact
// key would do now, and with the pilot out of the seat its middle is marked: it is where the
// hand reaches.
[[nodiscard]] UiActions draw_flight_view_ui(const sim::SceneSnapshot& snapshot,
                                            const render::FlightGeometry& geometry,
                                            const math::Vector3& camera_from_focus_m,
                                            const ScreenProjection& screen, UiState& state,
                                            double frames_per_second, bool from_inside);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_MAP_VIEW_UI_HPP
