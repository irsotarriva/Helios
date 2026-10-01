#include "map_view_ui.hpp"

#include "helios/orbital/conic.hpp"
#include "helios/sim/time_warp.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <imgui.h>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "scenario.hpp"

namespace helios::app {

namespace {

using math::Vector3;

constexpr float k_marker_radius_px = 3.5F;
constexpr float k_label_gap_px = 6.0F;
constexpr float k_moon_label_min_separation_px = 24.0F;

// Type-safe text: ImGui's own Text() is printf-style.
template <typename... Arguments>
void text(std::format_string<Arguments...> format, Arguments&&... arguments) {
    const std::string line = std::format(format, std::forward<Arguments>(arguments)...);
    ImGui::TextUnformatted(line.c_str());
}

[[nodiscard]] std::string format_distance(double metres) {
    const double magnitude = std::abs(metres);
    if (magnitude >= 1e9) {
        return std::format("{:.3f} Gm", metres / 1e9);
    }
    if (magnitude >= 1e6) {
        return std::format("{:.3f} Mm", metres / 1e6);
    }
    if (magnitude >= 1e3) {
        return std::format("{:.2f} km", metres / 1e3);
    }
    return std::format("{:.1f} m", metres);
}

[[nodiscard]] std::string format_duration(double seconds) {
    if (!std::isfinite(seconds)) {
        return "inf";
    }
    const long long total = std::llround(std::abs(seconds));
    const long long days = total / 86'400;
    const long long hours = (total % 86'400) / 3'600;
    const long long minutes = (total % 3'600) / 60;
    const long long secs = total % 60;
    const char* sign = seconds < 0.0 ? "-" : "";
    if (days > 0) {
        return std::format("{}{}d {:02}h {:02}m", sign, days, hours, minutes);
    }
    return std::format("{}{:02}:{:02}:{:02}", sign, hours, minutes, secs);
}

[[nodiscard]] std::string format_warp(double factor) {
    if (factor >= 1e3) {
        return std::format("{:.2g}×", factor);
    }
    return std::format("{:g}×", factor);
}

[[nodiscard]] std::string_view event_name(const std::optional<dynamics::EventKind>& kind) {
    return kind.has_value() ? dynamics::to_string(*kind) : std::string_view("burn");
}

[[nodiscard]] std::optional<ImVec2> project(const render::Float3& point, const ScreenProjection& screen) {
    const auto projected =
        render::project_to_screen(point, screen.view_projection, screen.width_px, screen.height_px);
    if (!projected.has_value()) {
        return std::nullopt;
    }
    return ImVec2(projected->x, projected->y);
}

void draw_label(ImDrawList& list, const ImVec2& at, std::uint32_t abgr, const std::string& text,
                bool marker) {
    if (marker) {
        list.AddCircleFilled(at, k_marker_radius_px, abgr);
    }
    const ImVec2 text_at(at.x + k_label_gap_px, at.y - (0.5F * ImGui::GetFontSize()));
    list.AddText(ImVec2(text_at.x + 1.0F, text_at.y + 1.0F), IM_COL32(0, 0, 0, 200), text.c_str());
    list.AddText(text_at, abgr, text.c_str());
}

void draw_overlay(const sim::SceneSnapshot& snapshot, const render::FrameGeometry& geometry,
                  const Vector3& camera_from_focus_m, const ScreenProjection& screen) {
    ImDrawList& list = *ImGui::GetBackgroundDrawList();
    const std::size_t body_count = snapshot.bodies.size();
    std::vector<std::optional<ImVec2>> body_screen(body_count);
    for (std::size_t index = 0; index < body_count; ++index) {
        body_screen[index] = project(geometry.bodies[index].centre, screen);
    }
    for (std::size_t index = 0; index < body_count; ++index) {
        const sim::BodyView& body = snapshot.bodies[index];
        const render::BodyInstance& instance = geometry.bodies[index];
        const std::optional<ImVec2>& at = body_screen[index];
        if (!at.has_value()) {
            continue;
        }
        // A moon too close on screen to its planet would only clutter its label.
        if (body.domain_parent.has_value() && body.domain_parent->index < body_count) {
            const std::optional<ImVec2>& parent = body_screen[body.domain_parent->index];
            if (parent.has_value()
                && std::hypot(parent->x - at->x, parent->y - at->y) < k_moon_label_min_separation_px) {
                continue;
            }
        }
        const double apparent_radius_px =
            instance.radius_m / std::max(instance.distance_m, 1.0) * screen.focal_length_px;
        draw_label(list, *at, instance.abgr, body.name, apparent_radius_px < k_marker_radius_px);
    }

    // Anything drawn around a body is only worth labelling when it is visibly apart from it.
    const auto crowds_body = [&](const ImVec2& at, bodies::BodyId body) {
        if (body.index >= body_count || !body_screen[body.index].has_value()) {
            return false;
        }
        const ImVec2& centre = *body_screen[body.index];
        return std::hypot(centre.x - at.x, centre.y - at.y) < k_moon_label_min_separation_px;
    };
    for (const sim::VesselView& vessel : snapshot.vessels) {
        const auto at = project(render::to_camera_space(vessel.position_m, camera_from_focus_m), screen);
        if (at.has_value() && !crowds_body(*at, vessel.domain)) {
            const std::uint32_t colour = vessel.status == sim::VesselStatus::Crashed
                                             ? IM_COL32(255, 80, 80, 255)
                                             : IM_COL32(255, 255, 255, 255);
            list.AddQuadFilled(ImVec2(at->x, at->y - 5), ImVec2(at->x + 5, at->y), ImVec2(at->x, at->y + 5),
                               ImVec2(at->x - 5, at->y), colour);
            draw_label(list, *at, colour, vessel.name, false);
        }
        // Apsis markers on the osculating conic.
        if (!vessel.osculating.has_value() || vessel.status != sim::VesselStatus::Flying
            || vessel.domain.index >= body_count) {
            continue;
        }
        const orbital::ConicGeometry& conic = *vessel.osculating;
        const Vector3& centre_m = snapshot.bodies[vessel.domain.index].position_m;
        const auto mark = [&](double true_anomaly_rad, const char* name, double radius_m) {
            const Vector3 point_m =
                centre_m + orbital::conic_position_at_true_anomaly(conic, true_anomaly_rad);
            const auto marker = project(render::to_camera_space(point_m, camera_from_focus_m), screen);
            if (marker.has_value() && !crowds_body(*marker, vessel.domain)) {
                list.AddCircle(*marker, 4.0F, IM_COL32(120, 200, 255, 255), 0, 1.5F);
                draw_label(list, *marker, IM_COL32(120, 200, 255, 255),
                           std::format("{} {}", name, format_distance(radius_m - vessel.domain_radius_m)),
                           false);
            }
        };
        if (conic.eccentricity > 1e-4) {
            mark(0.0, "Pe", conic.periapsis_radius_m);
            if (conic.is_bound()) {
                mark(orbital::k_pi, "Ap", conic.apoapsis_radius_m);
            }
        }
    }
}

void clock_panel(const sim::SceneSnapshot& snapshot, UiActions& actions, double frames_per_second) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::Begin("Time", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextUnformatted(format_epoch_utc(snapshot.epoch).c_str());
    text("warp {} (requested {}){}", format_warp(snapshot.warp_factor),
         format_warp(snapshot.requested_warp_factor), snapshot.paused ? "  PAUSED" : "");
    constexpr std::size_t k_levels_per_row = 6;
    for (std::size_t level = 0;
         level < sim::k_warp_levels.size() && sim::k_warp_levels.at(level) <= snapshot.max_warp_factor;
         ++level) {
        if (level % k_levels_per_row != 0) {
            ImGui::SameLine();
        }
        const bool selected = sim::k_warp_levels.at(level) == snapshot.requested_warp_factor;
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(60, 120, 200, 255));
        }
        if (ImGui::SmallButton(format_warp(sim::k_warp_levels.at(level)).c_str())) {
            actions.warp_level = level;
        }
        if (selected) {
            ImGui::PopStyleColor();
        }
    }
    if (ImGui::Button(snapshot.paused ? "Resume" : "Pause")) {
        actions.paused = !snapshot.paused;
    }
    if (snapshot.next_event.has_value()) {
        const auto& event = *snapshot.next_event;
        const std::string vessel_name =
            event.vessel.index < snapshot.vessels.size() ? snapshot.vessels[event.vessel.index].name : "?";
        const std::string body_name =
            event.body.index < snapshot.bodies.size() ? snapshot.bodies[event.body.index].name : "?";
        text("next: {} {} ({}) in {}", vessel_name, event_name(event.kind), body_name,
             format_duration(event.time_to_event_s));
    }
    ImGui::BeginDisabled();
    text("{:.0f} FPS", frames_per_second);
    ImGui::EndDisabled();
    ImGui::End();
}

void focus_panel(const sim::SceneSnapshot& snapshot, UiActions& actions) {
    ImGui::SetNextWindowPos(ImVec2(10, 170), ImGuiCond_FirstUseEver);
    ImGui::Begin("Focus", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    // Bodies as the domain (sphere-of-influence) hierarchy.
    const auto depth_of = [&](const sim::BodyView& body) {
        int depth = 0;
        std::optional<bodies::BodyId> parent = body.domain_parent;
        while (parent.has_value() && parent->index < snapshot.bodies.size() && depth < 16) {
            ++depth;
            parent = snapshot.bodies[parent->index].domain_parent;
        }
        return depth;
    };
    // Depth-first order with an explicit stack (children pushed in reverse to keep file order).
    std::vector<const sim::BodyView*> stack;
    for (const sim::BodyView& body : std::views::reverse(snapshot.bodies)) {
        if (!body.domain_parent.has_value()) {
            stack.push_back(&body);
        }
    }
    while (!stack.empty()) {
        const sim::BodyView& body = *stack.back();
        stack.pop_back();
        const float indent = static_cast<float>(depth_of(body)) * 8.0F;
        if (indent > 0.0F) {
            ImGui::Indent(indent);
        }
        if (ImGui::Selectable(body.name.c_str(), snapshot.focus == sim::Focus::body(body.id))) {
            actions.focus = sim::Focus::body(body.id);
        }
        if (indent > 0.0F) {
            ImGui::Unindent(indent);
        }
        for (const sim::BodyView& child : std::views::reverse(snapshot.bodies)) {
            if (child.domain_parent == body.id) {
                stack.push_back(&child);
            }
        }
    }
    ImGui::Separator();
    for (const sim::VesselView& vessel : snapshot.vessels) {
        const bool selected = snapshot.focus == sim::Focus::vessel(vessel.id);
        const std::string label =
            std::format("{}{}", vessel.name, vessel.status == sim::VesselStatus::Crashed ? " (crashed)" : "");
        if (ImGui::Selectable(label.c_str(), selected)) {
            actions.focus = sim::Focus::vessel(vessel.id);
        }
    }
    ImGui::End();
}

[[nodiscard]] std::optional<std::reference_wrapper<const sim::SignalView>>
find_signal(const sim::VesselView& vessel, std::string_view name) {
    const auto match = std::ranges::find(vessel.signals, name, &sim::SignalView::name);
    if (match == vessel.signals.end()) {
        return std::nullopt;
    }
    return std::cref(*match);
}

// A switch or a lever for one command signal. `label` is what is shown; the signal's own name
// keeps the widget's identity.
void signal_control(const sim::VesselView& vessel, const sim::SignalView& signal, const char* label,
                    UiState& state, UiActions& actions) {
    const std::string widget = std::format("{}##{}", label, signal.name);
    if (signal.toggle) {
        bool on = signal.value >= 0.5;
        if (ImGui::Checkbox(widget.c_str(), &on)) {
            actions.commands.push_back({.vessel = vessel.id, .signal = signal.name, .value = on ? 1.0 : 0.0});
        }
        return;
    }
    const std::string key = std::format("{}:{}", vessel.id.index, signal.name);
    const auto held = state.held_levers.find(key);
    auto value = held != state.held_levers.end() ? held->second : static_cast<float>(signal.value);
    ImGui::SetNextItemWidth(160.0F);
    if (ImGui::SliderFloat(widget.c_str(), &value, static_cast<float>(signal.minimum),
                           static_cast<float>(signal.maximum), "%.2f")) {
        actions.commands.push_back(
            {.vessel = vessel.id, .signal = signal.name, .value = static_cast<double>(value)});
        state.held_levers[key] = value;
    }
    if (!ImGui::IsItemActive()) {
        state.held_levers.erase(key);
    }
}

// The vessel's systems, drawn from its control bus alone (BRIEFING §8.3, D22): the flight
// controls from the signals every vessel has, then every signal its parts offer.
void systems_section(const sim::VesselView& vessel, UiState& state, UiActions& actions) {
    ImGui::SeparatorText("Systems");
    const auto reading = [&](std::string_view name) {
        const auto signal = find_signal(vessel, name);
        return signal.has_value() ? signal->get().value : 0.0;
    };
    text("mass {:.0f} kg   thrust {:.1f} kN   flow {:.3f} kg/s", reading(vessel::k_signal_mass),
         reading(vessel::k_signal_thrust) / 1e3, reading(vessel::k_signal_mass_flow));
    if (const auto throttle = find_signal(vessel, "engine/throttle")) {
        signal_control(vessel, throttle->get(), "throttle", state, actions);
    }
    if (const auto stage = find_signal(vessel, vessel::k_signal_stage)) {
        const int activated = static_cast<int>(std::lround(stage->get().value));
        const int count = static_cast<int>(std::lround(stage->get().maximum));
        ImGui::BeginDisabled(activated >= count);
        if (ImGui::Button(std::format("Stage ({} of {})", activated, count).c_str())) {
            actions.commands.push_back({.vessel = vessel.id,
                                        .signal = std::string{vessel::k_signal_stage},
                                        .value = static_cast<double>(activated + 1)});
        }
        ImGui::EndDisabled();
    }
    // Where the nose should point: in the orbit's prograde / normal / radial-out frame (1), or
    // straight up or down from the body below (the local frame, 2).
    struct Heading {
        const char* label = "";
        double frame = 1.0;
        Vector3 direction;
    };
    static constexpr std::array<Heading, 8> k_headings{{{"Prograde", 1.0, {1.0, 0.0, 0.0}},
                                                        {"Retrograde", 1.0, {-1.0, 0.0, 0.0}},
                                                        {"Normal", 1.0, {0.0, 1.0, 0.0}},
                                                        {"Anti-normal", 1.0, {0.0, -1.0, 0.0}},
                                                        {"Radial out", 1.0, {0.0, 0.0, 1.0}},
                                                        {"Radial in", 1.0, {0.0, 0.0, -1.0}},
                                                        {"Up", 2.0, {0.0, 0.0, 1.0}},
                                                        {"Down", 2.0, {0.0, 0.0, -1.0}}}};
    if (const auto hold = find_signal(vessel, vessel::k_signal_attitude_hold)) {
        signal_control(vessel, hold->get(), "hold attitude", state, actions);
        ImGui::SameLine();
    }
    constexpr double k_rad_to_deg = 57.29577951308232;
    text("turning {:.2f} deg/s{}", vessel.turn_rate_rad_s * k_rad_to_deg,
         vessel.in_bubble ? "" : "  (on rails)");
    const Vector3 pointing{reading(vessel::k_signal_pointing_x), reading(vessel::k_signal_pointing_y),
                           reading(vessel::k_signal_pointing_z)};
    for (std::size_t index = 0; index < k_headings.size(); ++index) {
        const Heading& heading = k_headings.at(index);
        if (index % 3 != 0) {
            ImGui::SameLine();
        }
        const bool selected =
            reading(vessel::k_signal_pointing_frame) == heading.frame && pointing == heading.direction;
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(60, 120, 200, 255));
        }
        if (ImGui::SmallButton(heading.label)) {
            const auto set = [&](std::string_view signal, double value) {
                actions.commands.push_back(
                    {.vessel = vessel.id, .signal = std::string{signal}, .value = value});
            };
            set(vessel::k_signal_pointing_frame, heading.frame);
            set(vessel::k_signal_pointing_x, heading.direction.x);
            set(vessel::k_signal_pointing_y, heading.direction.y);
            set(vessel::k_signal_pointing_z, heading.direction.z);
        }
        if (selected) {
            ImGui::PopStyleColor();
        }
    }
    if (ImGui::TreeNode("All signals")) {
        for (const sim::SignalView& signal : vessel.signals) {
            if (signal.command) {
                signal_control(vessel, signal, signal.name.c_str(), state, actions);
            } else {
                text("{}  {:.6g} {}", signal.name, signal.value, signal.unit);
            }
        }
        ImGui::TreePop();
    }
}

void vessel_panel(const sim::SceneSnapshot& snapshot, UiState& state, UiActions& actions) {
    if (snapshot.vessels.empty()) {
        return;
    }
    const sim::VesselView* vessel = &snapshot.vessels.front();
    if (snapshot.focus.kind == sim::Focus::Kind::Vessel && snapshot.focus.index < snapshot.vessels.size()) {
        vessel = &snapshot.vessels[snapshot.focus.index];
    }
    // Beside the focus list, which grows as stages separate.
    ImGui::SetNextWindowPos(ImVec2(170, 170), ImGuiCond_FirstUseEver);
    ImGui::Begin("Vessel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    text("{}  ({})", vessel->name, vessel->domain_name);
    const double radius_m = math::norm(vessel->position_m - snapshot.bodies[vessel->domain.index].position_m);
    text("altitude {}   speed {:.1f} m/s", format_distance(radius_m - vessel->domain_radius_m),
         math::norm(vessel->velocity_in_domain_m_s));
    // Below the precision shown, a vertical speed is zero rather than "-0.0".
    text("over the surface {:.1f} m/s   vertical {:+.1f} m/s", vessel->surface_speed_m_s,
         std::abs(vessel->vertical_speed_m_s) < 0.05 ? 0.0 : vessel->vertical_speed_m_s);
    if (vessel->osculating.has_value() && vessel->status == sim::VesselStatus::Flying) {
        const orbital::ConicGeometry& conic = *vessel->osculating;
        text("Pe {}   Ap {}   e {:.4f}", format_distance(conic.periapsis_radius_m - vessel->domain_radius_m),
             conic.is_bound() ? format_distance(conic.apoapsis_radius_m - vessel->domain_radius_m)
                              : std::string("escape"),
             conic.eccentricity);
        if (conic.is_bound()) {
            text("period {}", format_duration(orbital::orbital_period_s(conic)));
        }
    }
    if (vessel->status == sim::VesselStatus::Crashed) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 100, 100, 255));
        ImGui::TextUnformatted("crashed");
        ImGui::PopStyleColor();
        ImGui::End();
        return;
    }
    if (vessel->status == sim::VesselStatus::Landed) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(120, 220, 140, 255));
        ImGui::TextUnformatted("landed");
        ImGui::PopStyleColor();
    }
    if (!vessel->signals.empty()) {
        systems_section(*vessel, state, actions);
    }
    if (vessel->status != sim::VesselStatus::Flying) {
        ImGui::End();
        return; // manoeuvres are planned on a trajectory, and a landed vessel has none
    }
    ImGui::SeparatorText("Manoeuvre");
    ImGui::DragFloat("prograde m/s", &state.prograde_m_s, 1.0F, -5000.0F, 5000.0F, "%.1f");
    ImGui::DragFloat("normal m/s", &state.normal_m_s, 1.0F, -5000.0F, 5000.0F, "%.1f");
    ImGui::DragFloat("radial-out m/s", &state.radial_out_m_s, 1.0F, -5000.0F, 5000.0F, "%.1f");
    ImGui::DragFloat("in (s)", &state.burn_delay_s, 1.0F, 0.0F, 1e7F, "%.0f");
    const auto request = [&](BurnRequest::When when) {
        actions.burn = BurnRequest{.vessel = vessel->id,
                                   .when = when,
                                   .delay_s = state.burn_delay_s,
                                   .prograde_m_s = state.prograde_m_s,
                                   .normal_m_s = state.normal_m_s,
                                   .radial_out_m_s = state.radial_out_m_s};
    };
    if (ImGui::Button("Burn in")) {
        request(BurnRequest::When::InSeconds);
    }
    ImGui::SameLine();
    if (ImGui::Button("at Ap")) {
        request(BurnRequest::When::NextApoapsis);
    }
    ImGui::SameLine();
    if (ImGui::Button("at Pe")) {
        request(BurnRequest::When::NextPeriapsis);
    }
    text("scheduled burns: {}", vessel->pending_burns);
    ImGui::End();
}

void help_panel(UiState& state, float width_px) {
    if (!state.show_help) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(width_px - 330.0F, 10), ImGuiCond_FirstUseEver);
    ImGui::Begin("Controls", &state.show_help, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextUnformatted("drag: orbit camera   wheel: zoom\n"
                           ", / . : slower / faster warp\n"
                           "space: pause   tab: next focus   F1: this help\n"
                           "esc: quit");
    ImGui::End();
}

} // namespace

UiActions draw_map_view_ui(const sim::SceneSnapshot& snapshot, const render::FrameGeometry& geometry,
                           const Vector3& camera_from_focus_m, const ScreenProjection& screen, UiState& state,
                           double frames_per_second) {
    UiActions actions;
    draw_overlay(snapshot, geometry, camera_from_focus_m, screen);
    clock_panel(snapshot, actions, frames_per_second);
    focus_panel(snapshot, actions);
    vessel_panel(snapshot, state, actions);
    help_panel(state, screen.width_px);
    return actions;
}

} // namespace helios::app
