#include "application.hpp"

#include "helios/core/logging.hpp"
#include "helios/orbital/conic.hpp"
#include "helios/render/camera.hpp"
#include "helios/render/flight_geometry.hpp"
#include "helios/render/gpu/imgui_renderer.hpp"
#include "helios/render/gpu/map_renderer.hpp"
#include "helios/render/gpu/solid_renderer.hpp"
#include "helios/render/scene_geometry.hpp"
#include "helios/sim/solar_system.hpp"
#include "helios/sim/time_warp.hpp"
#include "helios/xr/session.hpp"

#include <SDL3/SDL.h>
#include <algorithm>
#include <array>
#include <bgfx/bgfx.h>
#include <chrono>
#include <cmath>
#include <format>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <memory>
#include <numbers>
#include <string_view>

#include "flight_controls.hpp"
#include "headset.hpp"
#include "map_view_ui.hpp"
#include "pilot_controls.hpp"
#include "platform.hpp"
#include "scenario.hpp"
#include "screenshot.hpp"
#include "simulation_host.hpp"

namespace helios::app {

namespace {

using core::ErrorCode;

constexpr bgfx::ViewId k_map_view = 0;
constexpr bgfx::ViewId k_ui_view = 1;
// The eyes of a headset, each into its own image.
constexpr std::array<bgfx::ViewId, xr::k_eyes> k_eye_views{2, 3};
// How often the frame rate in VR is written to the log.
constexpr double k_vr_report_s = 5.0;
constexpr double k_simulation_tick_hz = 120.0;
constexpr double k_fixed_frame_s = 1.0 / 60.0;
constexpr double k_orbit_sensitivity_rad_per_px = 0.005;
constexpr double k_zoom_per_wheel_step = 0.85;
constexpr double k_look_sensitivity_rad_per_px = 0.004;
constexpr double k_cockpit_field_of_view_rad = 1.2;
constexpr double k_cockpit_near_m = 0.05;
constexpr double k_head_pitch_deg = -12.0;
constexpr double k_chase_distance_m = 25.0;
constexpr double k_least_chase_distance_m = 3.0;
constexpr double k_roll_rate_rad_s = 1.0; // of a floating pilot, on the roll keys
// Out of the seat the keys turn the view and the mouse is the hand (D27): the view turns at
// this rate on the keys, a drag moves the body this far per pixel of the mouse, smoothed over
// the second, and a throw is fully wound up after the third.
constexpr double k_turn_rate_rad_s = 1.2;
constexpr double k_arm_m_per_px = 0.004;
constexpr double k_arm_smoothing_s = 0.05;
constexpr double k_arm_key_m_s = 1.0; // the arrows move the body against the hand this fast
// Letting go of a ladder takes the interact key held this long.
constexpr double k_let_go_hold_s = 1.0;

enum class ViewMode : std::uint8_t {
    Map,
    Outside, // the vessel being flown, from a camera that circles it
    Cockpit, // from its seat
};

[[nodiscard]] core::Result<ViewMode> view_mode(std::string_view name) {
    if (name == "map") {
        return ViewMode::Map;
    }
    if (name == "outside") {
        return ViewMode::Outside;
    }
    if (name == "cockpit") {
        return ViewMode::Cockpit;
    }
    return core::fail(ErrorCode::InvalidArgument, std::format("unknown view '{}'", name));
}

// The keys of the flight actions (BRIEFING §12.1 rule 4): by position on the keyboard, so that
// they are the same keys on every layout.
struct KeyBinding {
    SDL_Scancode key;
    Action action;
};
constexpr std::array k_key_bindings{
    KeyBinding{SDL_SCANCODE_LSHIFT, Action::ThrottleUp}, KeyBinding{SDL_SCANCODE_LCTRL, Action::ThrottleDown},
    KeyBinding{SDL_SCANCODE_Z, Action::ThrottleFull}, KeyBinding{SDL_SCANCODE_X, Action::ThrottleCut},
    KeyBinding{SDL_SCANCODE_S, Action::PitchUp}, KeyBinding{SDL_SCANCODE_W, Action::PitchDown},
    KeyBinding{SDL_SCANCODE_A, Action::YawLeft}, KeyBinding{SDL_SCANCODE_D, Action::YawRight},
    KeyBinding{SDL_SCANCODE_Q, Action::RollLeft}, KeyBinding{SDL_SCANCODE_E, Action::RollRight},
    KeyBinding{SDL_SCANCODE_RETURN, Action::Stage}, KeyBinding{SDL_SCANCODE_T, Action::ToggleAttitudeHold},
    // Moving, for every vessel and the pilot alike: the arrows,
    // with the right shift and ctrl for up and down.
    KeyBinding{SDL_SCANCODE_UP, Action::TranslateForward},
    KeyBinding{SDL_SCANCODE_DOWN, Action::TranslateBack},
    KeyBinding{SDL_SCANCODE_LEFT, Action::TranslateLeft},
    KeyBinding{SDL_SCANCODE_RIGHT, Action::TranslateRight},
    KeyBinding{SDL_SCANCODE_RSHIFT, Action::TranslateUp},
    KeyBinding{SDL_SCANCODE_RCTRL, Action::TranslateDown}};

// The vessel being flown: the focus, when it is a vessel.
[[nodiscard]] std::optional<std::size_t> flown_vessel(const sim::SceneSnapshot& snapshot) noexcept {
    if (snapshot.focus.kind != sim::Focus::Kind::Vessel || snapshot.focus.index >= snapshot.vessels.size()) {
        return std::nullopt;
    }
    return snapshot.focus.index;
}

// The view that can be shown of what was asked for.
[[nodiscard]] ViewMode available_view(ViewMode wanted, const sim::SceneSnapshot& snapshot) noexcept {
    const auto flown = flown_vessel(snapshot);
    if (!flown.has_value()) {
        return ViewMode::Map;
    }
    if (wanted == ViewMode::Cockpit && !render::seat_part(snapshot.vessels[*flown]).has_value()) {
        return ViewMode::Outside;
    }
    return wanted;
}

// The pilot moving about the cabin of the vessel being flown, if that is what is going on.
[[nodiscard]] std::optional<sim::PilotView> pilot_afoot(const sim::SceneSnapshot& snapshot) {
    const auto flown = flown_vessel(snapshot);
    if (!flown.has_value() || !snapshot.pilot.has_value() || snapshot.pilot->seated
        || snapshot.pilot->vessel.index != *flown) {
        return std::nullopt;
    }
    return snapshot.pilot;
}

// What the previous frame showed of the flight view: it is what the pointer points at.
struct FlightFrame {
    render::FlightGeometry geometry;
    render::ViewPoint view;
    render::Matrix4f view_projection{};
};

// A lever held by the pointer.
struct Grab {
    std::size_t instrument = 0;
    double fraction = 0.0;
};

// The cockpit the pilot sits in, if there is one.
[[nodiscard]] std::optional<std::reference_wrapper<const vessel::Cockpit>>
cockpit_of(const sim::VesselView& vessel) noexcept {
    const auto seat = render::seat_part(vessel);
    if (!seat.has_value()) {
        return std::nullopt;
    }
    return std::cref(*vessel.parts[*seat].datasheet->cockpit);
}

void post_commands(SimulationHost& host, const std::vector<SignalCommand>& commands) {
    for (const SignalCommand& command : commands) {
        // Rationale: an init-capture, because copying `command` itself would make the
        // closure's member const and its move a copy that can throw.
        host.post([posted = command](sim::Simulation& simulation) {
            core::VoidResult done =
                posted.release
                    ? simulation.release_command(posted.vessel, posted.signal, vessel::ControlSource::Pilot)
                    : simulation.command(posted.vessel, posted.signal, vessel::ControlSource::Pilot,
                                         posted.value);
            if (!done) {
                LOG_WARN("command rejected: {}", core::describe(done.error())).tag("subsystem", "ui");
            }
        });
    }
}

[[nodiscard]] core::Result<bgfx::RendererType::Enum> renderer_type(std::string_view name) {
    if (name == "auto") {
        return bgfx::RendererType::Count;
    }
    if (name == "metal") {
        return bgfx::RendererType::Metal;
    }
    if (name == "vulkan") {
        return bgfx::RendererType::Vulkan;
    }
    if (name == "opengl") {
        return bgfx::RendererType::OpenGL;
    }
    if (name == "d3d11") {
        return bgfx::RendererType::Direct3D11;
    }
    if (name == "d3d12") {
        return bgfx::RendererType::Direct3D12;
    }
    return core::fail(ErrorCode::InvalidArgument, std::format("unknown renderer '{}'", name));
}

// Tears the platform layers down in reverse order, whatever path leaves run().
struct PlatformScope {
    SDL_Window* window = nullptr;
    bool bgfx_ready = false;
    bool imgui_ready = false;

    PlatformScope() = default;
    PlatformScope(const PlatformScope&) = delete;
    PlatformScope& operator=(const PlatformScope&) = delete;
    PlatformScope(PlatformScope&&) = delete;
    PlatformScope& operator=(PlatformScope&&) = delete;
    ~PlatformScope() {
        if (imgui_ready) {
            ImGui_ImplSDL3_Shutdown();
            ImGui::DestroyContext();
        }
        if (bgfx_ready) {
            bgfx::shutdown();
        }
        if (window != nullptr) {
            SDL_DestroyWindow(window);
        }
        SDL_Quit();
    }
};

[[nodiscard]] core::Result<sim::Focus> find_focus(const sim::Simulation& simulation, std::string_view name) {
    if (const auto body = simulation.catalog().find(name)) {
        return sim::Focus::body(*body);
    }
    const auto vessels = simulation.vessels();
    for (std::uint32_t index = 0; index < vessels.size(); ++index) {
        if (vessels[index].name == name) {
            return sim::Focus::vessel(sim::VesselId{index});
        }
    }
    return core::fail(ErrorCode::InvalidArgument, std::format("nothing called '{}' to focus on", name));
}

[[nodiscard]] double framing_distance_m(const sim::SceneSnapshot& snapshot) {
    if (snapshot.focus.kind == sim::Focus::Kind::Body) {
        return std::max(8.0 * snapshot.focus_radius_m, 1e5);
    }
    if (snapshot.focus.index < snapshot.vessels.size()) {
        return 2.5 * snapshot.vessels[snapshot.focus.index].domain_radius_m;
    }
    return 1e7;
}

void post_burn(SimulationHost& host, const BurnRequest& burn) {
    host.post([burn](sim::Simulation& simulation) {
        const auto vessel = simulation.vessel(burn.vessel);
        if (!vessel) {
            return;
        }
        const dynamics::VesselState& state = vessel->get().state;
        std::optional<double> delay_s = burn.delay_s;
        if (burn.when != BurnRequest::When::InSeconds) {
            const auto domain = simulation.catalog().body(state.domain);
            const auto conic = orbital::conic_geometry(
                state.state_in_domain, domain ? domain->get().gravitational_parameter_m3_s2 : 0.0);
            if (!conic) {
                delay_s.reset();
            } else if (burn.when == BurnRequest::When::NextApoapsis) {
                delay_s = orbital::time_to_apoapsis_s(*conic);
            } else {
                delay_s = orbital::time_to_periapsis_s(*conic);
            }
        }
        if (!delay_s.has_value()) {
            LOG_WARN("no such apsis to burn at").tag("subsystem", "ui");
            return;
        }
        const auto epoch = simulation.now().advanced_by(*delay_s);
        if (!epoch) {
            return;
        }
        if (core::VoidResult scheduled = simulation.schedule_maneuver(burn.vessel, *epoch, burn.prograde_m_s,
                                                                      burn.normal_m_s, burn.radial_out_m_s);
            !scheduled) {
            LOG_WARN("burn rejected: {}", core::describe(scheduled.error())).tag("subsystem", "ui");
        }
    });
}

[[nodiscard]] sim::Focus next_focus(const sim::SceneSnapshot& snapshot) {
    const std::size_t bodies = snapshot.bodies.size();
    const std::size_t total = bodies + snapshot.vessels.size();
    const std::size_t current =
        snapshot.focus.index + (snapshot.focus.kind == sim::Focus::Kind::Vessel ? bodies : 0);
    if (total == 0) {
        return snapshot.focus;
    }
    // A vessel put away aboard another is nowhere to look at.
    std::size_t next = (current + 1) % total;
    while (next >= bodies && next != current
           && snapshot.vessels[next - bodies].status == sim::VesselStatus::Stowed) {
        next = (next + 1) % total;
    }
    if (next < bodies) {
        return sim::Focus::body(bodies::BodyId{static_cast<std::uint32_t>(next)});
    }
    return sim::Focus::vessel(sim::VesselId{static_cast<std::uint32_t>(next - bodies)});
}

struct BgfxSetup {
    bgfx::RendererType::Enum type = bgfx::RendererType::Count;
    int width_px = 0;
    int height_px = 0;
    std::uint32_t reset_flags = BGFX_RESET_VSYNC;
};

[[nodiscard]] bool init_bgfx(const NativeWindow& native, const BgfxSetup& setup, BgfxCallback& callback,
                             bgfx::TextureFormat::Enum depth_format) {
    bgfx::Init init;
    init.type = setup.type;
    init.fallback = setup.type == bgfx::RendererType::Count;
    init.swapChain.nwh = native.window_handle;
    init.swapChain.ndt = native.display_handle;
    init.platformData.type = native.type;
    init.swapChain.width = static_cast<std::uint32_t>(setup.width_px);
    init.swapChain.height = static_cast<std::uint32_t>(setup.height_px);
    init.swapChain.flags = BGFX_SWAP_CHAIN_MSAA_X4;
    init.swapChain.formatDepthStencil = depth_format;
    init.reset = setup.reset_flags;
    init.callback = &callback;
    return bgfx::init(init);
}

} // namespace

core::VoidResult run(const Options& options) {
    // --- Universe -----------------------------------------------------------------------------
    auto universe = sim::load_stock_solar_system(options.data_dir, options.ephemeris);
    if (!universe) {
        return std::unexpected(universe.error());
    }
    const double start_unix_s = options.start_unix_utc_s.value_or(
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto start = epoch_from_unix_utc(start_unix_s);
    if (!start) {
        return std::unexpected(start.error());
    }
    auto universe_simulation =
        sim::Simulation::make(std::move(universe->tree), std::move(universe->catalog), *start, {});
    if (!universe_simulation) {
        return std::unexpected(universe_simulation.error());
    }
    if (options.demo_vessels) {
        if (core::VoidResult added = add_demo_vessels(*universe_simulation); !added) {
            return added;
        }
        // The parts live next to the Solar System data; a data directory without them still runs.
        if (core::VoidResult added = add_demo_craft(*universe_simulation, options.data_dir.parent_path());
            !added) {
            LOG_WARN("no demo craft: {}", core::describe(added.error())).tag("subsystem", "app");
        }
    }
    universe_simulation->time_warp().request_level(options.warp_level);
    const auto focus = find_focus(*universe_simulation, options.focus);
    if (!focus) {
        return std::unexpected(focus.error());
    }
    // Rationale: Direct3D 11 is the one renderer whose device OpenXR is given so far (D29).
    const auto requested_renderer =
        renderer_type(options.vr && options.renderer == "auto" ? "d3d11" : options.renderer);
    if (!requested_renderer) {
        return std::unexpected(requested_renderer.error());
    }
    const auto requested_view = view_mode(options.view);
    if (!requested_view) {
        return std::unexpected(requested_view.error());
    }

    // --- Window, bgfx, ImGui -------------------------------------------------------------------
    // Declared before the platform scope: bgfx calls it until bgfx::shutdown, in ~PlatformScope.
    BgfxCallback callback;
    PlatformScope platform;
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        return core::fail(ErrorCode::ExternalLibraryFailure, std::format("SDL_Init: {}", SDL_GetError()));
    }
    platform.window = SDL_CreateWindow("Helios", options.width_px, options.height_px,
                                       SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (platform.window == nullptr) {
        return core::fail(ErrorCode::ExternalLibraryFailure,
                          std::format("SDL_CreateWindow: {}", SDL_GetError()));
    }
    const auto native = native_window(platform.window);
    if (!native) {
        return std::unexpected(native.error());
    }
    int width_px = 0;
    int height_px = 0;
    SDL_GetWindowSizeInPixels(platform.window, &width_px, &height_px);

    // Rationale: calling renderFrame before init makes bgfx render on this thread (no render
    // thread), which Metal requires when the window is driven by SDL's main-thread event loop.
    bgfx::renderFrame();
    // A 32-bit float depth buffer is what makes reversed-Z uniform in log distance; fall back to
    // 24-bit where the back end or driver refuses it.
#ifdef _WIN32
    // Rationale: on WGL, bgfx searches for a window pixel format with the requested depth bits
    // and, when none exists (NVIDIA has no 32-bit depth window format), retries forever without
    // relaxing them, so init never returns and the fallback below is never reached.
    const bool try_float_depth = *requested_renderer != bgfx::RendererType::OpenGL;
#else
    const bool try_float_depth = true;
#endif
    // Rationale: in VR the headset paces the frames (Headset::begin_frame); waiting for the
    // window's refresh as well would hold the headset to the monitor's rate.
    const std::uint32_t reset_flags = options.vr ? BGFX_RESET_NONE : BGFX_RESET_VSYNC;
    const BgfxSetup setup{.type = *requested_renderer,
                          .width_px = width_px,
                          .height_px = height_px,
                          .reset_flags = reset_flags};
    platform.bgfx_ready = (try_float_depth && init_bgfx(*native, setup, callback, bgfx::TextureFormat::D32F))
                          || init_bgfx(*native, setup, callback, bgfx::TextureFormat::D24S8);
    if (!platform.bgfx_ready) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "bgfx::init failed");
    }
    const bgfx::Caps& caps = *bgfx::getCaps();
    LOG_INFO("renderer: {}", bgfx::getRendererName(bgfx::getRendererType())).tag("subsystem", "render");

    ImGui::CreateContext();
    platform.imgui_ready = true;
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOther(platform.window);

    auto map_renderer = render::gpu::MapRenderer::make();
    if (!map_renderer) {
        return std::unexpected(map_renderer.error());
    }
    auto solid_renderer = render::gpu::SolidRenderer::make();
    if (!solid_renderer) {
        return std::unexpected(solid_renderer.error());
    }
    auto ui_renderer = render::gpu::ImGuiRenderer::make(k_ui_view);
    if (!ui_renderer) {
        return std::unexpected(ui_renderer.error());
    }

    // Declared after the platform scope and the renderers: the headset goes before bgfx does.
    std::optional<Headset> headset;
    if (options.vr) {
        auto opened = Headset::open();
        if (!opened) {
            return std::unexpected(opened.error());
        }
        headset.emplace(std::move(*opened));
    }
    double vr_report_s = 0.0;
    int vr_frames = 0;

    // --- Simulation ----------------------------------------------------------------------------
    const bool deterministic = options.frames.has_value();
    SimulationHost host(std::move(*universe_simulation), *focus);
    if (!deterministic) {
        host.start(k_simulation_tick_hz);
    }

    render::OrbitCamera camera;
    if (const auto first = host.latest()) {
        camera.distance_m = options.camera_distance_m.value_or(framing_distance_m(*first));
    }
    if (options.camera_yaw_deg.has_value()) {
        camera.yaw_rad = *options.camera_yaw_deg * std::numbers::pi / 180.0;
    }
    if (options.camera_pitch_deg.has_value()) {
        camera.pitch_rad = *options.camera_pitch_deg * std::numbers::pi / 180.0;
    }
    // The flight views: the pilot's head in the cockpit, and a camera circling the vessel.
    ViewMode wanted_view =
        options.vr && *requested_view == ViewMode::Map ? ViewMode::Cockpit : *requested_view;
    ViewMode flight_view = wanted_view == ViewMode::Outside ? ViewMode::Outside : ViewMode::Cockpit;
    render::HeadPose head;
    // Looking a little down, at the horizon over the top of the panel.
    head.turn(options.camera_yaw_deg.value_or(0.0) * std::numbers::pi / 180.0,
              options.camera_pitch_deg.value_or(k_head_pitch_deg) * std::numbers::pi / 180.0);
    render::OrbitCamera chase{.yaw_rad = camera.yaw_rad, .pitch_rad = camera.pitch_rad};
    chase.distance_m = wanted_view == ViewMode::Map ? k_chase_distance_m
                                                    : options.camera_distance_m.value_or(k_chase_distance_m);
    std::optional<FlightFrame> flight_frame;
    std::optional<Grab> grab;
    // The pilot out of the seat: where they look, and the hand the mouse drives: whether it is
    // closed, how far the mouse has dragged it this frame, and the arm's smoothed speed.
    PilotLook look;
    bool hand_closed = false;
    bool latch_closed = false;
    // Since when the interact key has been held to let go of a ladder.
    std::optional<std::chrono::steady_clock::time_point> letting_go_since;
    double arm_dx_px = 0.0;
    double arm_dy_px = 0.0;
    math::Vector3 arm_m_s;
    bool was_afoot = false;
    // The vessel the pilot was last seen in; it changes by itself in an airlock.
    std::optional<sim::VesselId> pilot_vessel;
    bool leave_seat_wanted = options.leave_seat;
    FlightControlState control_state;
    UiState ui_state;
    ui_state.show_help = !deterministic;
    ui_state.show_panels = wanted_view != ViewMode::Cockpit;
    double frames_per_second = 60.0;
    auto previous_frame = std::chrono::steady_clock::now();
    bool running = true;
    int frame = 0;
    bool dragging = false;
    double last_frame_s = k_fixed_frame_s;

    while (running) {
        // The headset first: it holds the frame back until it wants one, and what is drawn
        // should be as fresh as can be when it does.
        xr::Frame headset_frame;
        if (headset.has_value()) {
            const auto wanted = headset->poll();
            if (!wanted) {
                return std::unexpected(wanted.error());
            }
            auto begun = *wanted ? headset->begin_frame() : core::Result<xr::Frame>(xr::Frame{});
            if (!begun) {
                return std::unexpected(begun.error());
            }
            headset_frame = *begun;
            running = *wanted;
        }
        // Carries the frame out, and hands the eyes' images back to the headset.
        const auto present = [&]() -> core::VoidResult {
            bgfx::frame();
            return headset.has_value() ? headset->end_frame(headset_frame) : core::VoidResult{};
        };
        SDL_Event event;
        std::shared_ptr<const sim::SceneSnapshot> snapshot = host.latest();
        const ViewMode shown = snapshot ? available_view(wanted_view, *snapshot) : ViewMode::Map;
        const sim::VesselView* flown = nullptr;
        if (snapshot) {
            if (const auto index = flown_vessel(*snapshot)) {
                flown = &snapshot->vessels[*index];
            }
        }
        // Out of the seat (and looking through the pilot's eyes) the mouse turns the pilot and
        // the keys move them; the vessel's flight keys are out of reach.
        const std::optional<sim::PilotView> walker =
            snapshot && shown == ViewMode::Cockpit ? pilot_afoot(*snapshot) : std::nullopt;
        const bool afoot = walker.has_value();
        // On their feet without having left a seat here (out of an airlock, or come down to the
        // ground in a suit that was flown): the pilot looks where the body faces. On foot
        // outside, the look is in the ground's axes.
        const std::optional<sim::VesselId> pilot_now =
            snapshot && snapshot->pilot.has_value() ? std::optional(snapshot->pilot->vessel) : std::nullopt;
        if (afoot && flown != nullptr && (pilot_now != pilot_vessel || (!was_afoot && walker->outside))) {
            const math::Matrix3 seat_axes = render::seat_to_vessel(*flown).value_or(math::Matrix3{});
            look = walker->ground.has_value()
                       ? look_from_suit(math::to_matrix(math::conjugate(*walker->ground) * flown->orientation)
                                        * seat_axes)
                       : look_from_seat(seat_axes, 0.0, 0.0);
        }
        pilot_vessel = pilot_now;
        if (afoot != was_afoot) {
            // Rationale: a deterministic run has no one at the mouse, and must not take it.
            SDL_SetWindowRelativeMouseMode(platform.window, afoot && !deterministic);
            if (!afoot) {
                // The keys are the vessel's again: the legs are asked nothing more.
                host.post([view = view_of(look)](sim::Simulation& simulation) {
                    simulation.set_pilot_input({.view = view});
                });
            }
            was_afoot = afoot;
            grab.reset();
            hand_closed = false;
            latch_closed = false;
            arm_m_s = {};
            ui_state.hand_x_px = -1.0F; // the hand starts in the middle of the picture
            ui_state.hand_y_px = -1.0F;
        }
        double look_yaw_rad = 0.0;
        double look_pitch_rad = 0.0;
        if (leave_seat_wanted && flown != nullptr && (shown == ViewMode::Cockpit || options.go_outside)) {
            leave_seat_wanted = false;
            look = look_from_seat(render::seat_to_vessel(*flown).value_or(math::Matrix3{}), head.yaw_rad,
                                  head.pitch_rad);
            host.post([outside = options.go_outside](sim::Simulation& simulation) {
                if (core::VoidResult left = simulation.leave_seat(); !left) {
                    LOG_WARN("{}", core::describe(left.error())).tag("subsystem", "ui");
                    return;
                }
                if (!outside) {
                    return;
                }
                if (core::VoidResult out = simulation.go_outside(); !out) {
                    LOG_WARN("{}", core::describe(out.error())).tag("subsystem", "ui");
                }
            });
        }
        ActionState flight_actions;
        std::vector<SignalCommand> commands;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            switch (event.type) {
            case SDL_EVENT_QUIT:
            case SDL_EVENT_WINDOW_CLOSE_REQUESTED:    running = false; break;
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED: {
                SDL_GetWindowSizeInPixels(platform.window, &width_px, &height_px);
                bgfx::SwapChain swap_chain;
                swap_chain.width = static_cast<std::uint32_t>(width_px);
                swap_chain.height = static_cast<std::uint32_t>(height_px);
                swap_chain.flags = BGFX_SWAP_CHAIN_MSAA_X4;
                bgfx::reset(reset_flags, &swap_chain);
                break;
            }
            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                dragging = !io.WantCaptureMouse;
                if (!dragging || shown != ViewMode::Cockpit || !flight_frame.has_value()
                    || flown == nullptr) {
                    break;
                }
                // Out of the seat the right button is the latching hand (D27): it takes hold of
                // what is under the hand, drags like the left one, and stays anchored after.
                if (afoot && event.button.button == SDL_BUTTON_RIGHT) {
                    latch_closed = !walker->outside;
                    break;
                }
                // In the cockpit the pointer is a hand: on an instrument it handles it, anywhere
                // else it turns the head.
                const auto cockpit = cockpit_of(*flown);
                // Afoot the hand is where the mouse has it (the middle of the picture at first).
                float pointer_x = event.button.x;
                float pointer_y = event.button.y;
                if (afoot) {
                    const bool moved = ui_state.hand_x_px >= 0.0F;
                    pointer_x = moved ? ui_state.hand_x_px : 0.5F * io.DisplaySize.x;
                    pointer_y = moved ? ui_state.hand_y_px : 0.5F * io.DisplaySize.y;
                }
                const auto picked = render::pick_instrument(
                    flight_frame->geometry.instruments,
                    render::ray_through_pixel(pointer_x, pointer_y, io.DisplaySize.x, io.DisplaySize.y,
                                              k_cockpit_field_of_view_rad,
                                              flight_frame->view.universe_to_camera));
                if (!cockpit.has_value() || !picked.has_value()) {
                    // Not on an instrument: the grabbing hand closes on what is under it, a wall
                    // or a loose item, for as long as the button is held.
                    hand_closed = afoot && !walker->outside && event.button.button == SDL_BUTTON_LEFT;
                    break;
                }
                const std::size_t index = flight_frame->geometry.instruments[*picked].instrument;
                if (index >= cockpit->get().instruments.size()) {
                    break;
                }
                dragging = false;
                const vessel::Instrument& instrument = cockpit->get().instruments[index];
                if (instrument.kind == vessel::Instrument::Kind::Lever) {
                    grab = Grab{.instrument = index,
                                .fraction = lever_fraction(*flown, instrument).value_or(0.0)};
                } else {
                    std::ranges::copy(press_instrument(*flown, instrument), std::back_inserter(commands));
                }
                break;
            }
            case SDL_EVENT_KEY_UP:
                if (event.key.key == SDLK_F) {
                    letting_go_since.reset(); // released too soon: still holding on
                }
                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                dragging = false;
                grab.reset();
                if (event.button.button == SDL_BUTTON_LEFT) {
                    hand_closed = false; // the hand opens: the body and what it held keep their speeds
                }
                if (event.button.button == SDL_BUTTON_RIGHT) {
                    latch_closed = false; // the hand keeps its hold, and the arm stops at once
                    arm_m_s = {};
                }
                break;
            case SDL_EVENT_MOUSE_MOTION:
                if (grab.has_value() && flight_frame.has_value() && flown != nullptr) {
                    const auto cockpit = cockpit_of(*flown);
                    const auto handle =
                        std::ranges::find(flight_frame->geometry.instruments, grab->instrument,
                                          &render::InstrumentHandle::instrument);
                    if (cockpit.has_value() && grab->instrument < cockpit->get().instruments.size()
                        && handle != flight_frame->geometry.instruments.end()) {
                        grab->fraction =
                            std::clamp(grab->fraction
                                           + render::lever_travel_fraction(
                                               *handle, flight_frame->view_projection, io.DisplaySize.x,
                                               io.DisplaySize.y, event.motion.xrel, event.motion.yrel),
                                       0.0, 1.0);
                        if (const auto moved = move_lever(
                                *flown, cockpit->get().instruments[grab->instrument], grab->fraction)) {
                            commands.push_back(*moved);
                        }
                    }
                } else if (afoot && deterministic) {
                    // Nobody is at the mouse of a deterministic run: the pilot looks ahead.
                } else if (afoot && (hand_closed || latch_closed) && walker->grabbing) {
                    // Holding on, the mouse is the arm: what it drags moves the body.
                    arm_dx_px += event.motion.xrel;
                    arm_dy_px += event.motion.yrel;
                } else if (afoot) {
                    // Otherwise it moves the hand over the picture; the view stays where it is.
                    const float x_px =
                        ui_state.hand_x_px >= 0.0F ? ui_state.hand_x_px : 0.5F * io.DisplaySize.x;
                    const float y_px =
                        ui_state.hand_y_px >= 0.0F ? ui_state.hand_y_px : 0.5F * io.DisplaySize.y;
                    ui_state.hand_x_px = std::clamp(x_px + event.motion.xrel, 0.0F, io.DisplaySize.x);
                    ui_state.hand_y_px = std::clamp(y_px + event.motion.yrel, 0.0F, io.DisplaySize.y);
                } else if (dragging && shown == ViewMode::Map) {
                    camera.orbit(-event.motion.xrel * k_orbit_sensitivity_rad_per_px,
                                 event.motion.yrel * k_orbit_sensitivity_rad_per_px);
                } else if (dragging && shown == ViewMode::Outside) {
                    chase.orbit(-event.motion.xrel * k_orbit_sensitivity_rad_per_px,
                                event.motion.yrel * k_orbit_sensitivity_rad_per_px);
                } else if (dragging) {
                    head.turn(-event.motion.xrel * k_look_sensitivity_rad_per_px,
                              -event.motion.yrel * k_look_sensitivity_rad_per_px);
                }
                break;
            case SDL_EVENT_MOUSE_WHEEL:
                if (io.WantCaptureMouse || !snapshot) {
                    break;
                }
                if (shown == ViewMode::Map) {
                    camera.zoom(std::pow(k_zoom_per_wheel_step, event.wheel.y),
                                1.02 * snapshot->focus_radius_m);
                } else if (shown == ViewMode::Outside) {
                    chase.zoom(std::pow(k_zoom_per_wheel_step, event.wheel.y), k_least_chase_distance_m);
                }
                break;
            case SDL_EVENT_KEY_DOWN:
                if (io.WantCaptureKeyboard) {
                    break;
                }
                for (const KeyBinding& binding : k_key_bindings) {
                    // A key held down repeats; an action is pressed once.
                    if (binding.key == event.key.scancode && !event.key.repeat && !afoot) {
                        flight_actions.set(binding.action, true, true);
                    }
                }
                if (event.key.key == SDLK_ESCAPE) {
                    running = false;
                } else if (event.key.key == SDLK_PERIOD) {
                    host.post([](sim::Simulation& simulation) { simulation.time_warp().increase(); });
                } else if (event.key.key == SDLK_COMMA) {
                    host.post([](sim::Simulation& simulation) { simulation.time_warp().decrease(); });
                } else if (event.key.key == SDLK_P || (event.key.key == SDLK_SPACE && !afoot)) {
                    // Rationale: out of the seat the space bar is the pilot's (jump, up).
                    host.post([](sim::Simulation& simulation) {
                        simulation.time_warp().set_paused(!simulation.time_warp().paused());
                    });
                } else if (event.key.key == SDLK_F1) {
                    ui_state.show_help = !ui_state.show_help;
                } else if (event.key.key == SDLK_F && !event.key.repeat && flown != nullptr
                           && shown == ViewMode::Cockpit && afoot
                           && walker->offer == sim::PilotOffer::LetGo) {
                    // Letting go of a ladder can leave the pilot drifting off for good: it takes
                    // the key held down a while, never a tap (D35).
                    letting_go_since = std::chrono::steady_clock::now();
                } else if (event.key.key == SDLK_F && !event.key.repeat && flown != nullptr
                           && shown == ViewMode::Cockpit) {
                    // One key for whatever there is to do here: the simulation says what
                    // (PilotOffer), and the picture says it to the player.
                    if (!afoot) {
                        look = look_from_seat(render::seat_to_vessel(*flown).value_or(math::Matrix3{}),
                                              head.yaw_rad, head.pitch_rad);
                    }
                    host.post([reset_input = !afoot](sim::Simulation& simulation) {
                        if (reset_input) {
                            simulation.set_pilot_input({});
                        }
                        if (core::VoidResult done = simulation.interact(); !done) {
                            LOG_INFO("{}", core::describe(done.error())).tag("subsystem", "ui");
                        }
                    });
                } else if (event.key.key == SDLK_F2) {
                    ui_state.show_panels = !ui_state.show_panels;
                } else if (event.key.key == SDLK_M) {
                    wanted_view = wanted_view == ViewMode::Map ? flight_view : ViewMode::Map;
                    ui_state.show_panels = wanted_view != ViewMode::Cockpit;
                } else if (event.key.key == SDLK_C && wanted_view != ViewMode::Map) {
                    flight_view = wanted_view == ViewMode::Cockpit ? ViewMode::Outside : ViewMode::Cockpit;
                    wanted_view = flight_view;
                    ui_state.show_panels = wanted_view != ViewMode::Cockpit;
                } else if (event.key.key == SDLK_TAB && snapshot) {
                    const sim::Focus next = next_focus(*snapshot);
                    host.set_focus(next);
                }
                break;
            default: break;
            }
        }
        const double input_dt_s = deterministic ? k_fixed_frame_s : last_frame_s;
        ui_state.letting_go = -1.0F;
        if (letting_go_since.has_value()) {
            const double held_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - *letting_go_since).count();
            if (!afoot || walker->offer != sim::PilotOffer::LetGo) {
                letting_go_since.reset();
            } else if (held_s >= k_let_go_hold_s) {
                letting_go_since.reset();
                host.post([](sim::Simulation& simulation) {
                    if (core::VoidResult done = simulation.interact(); !done) {
                        LOG_INFO("{}", core::describe(done.error())).tag("subsystem", "ui");
                    }
                });
            } else {
                ui_state.letting_go = static_cast<float>(held_s / k_let_go_hold_s);
            }
        }
        if (afoot) {
            flight_actions = {};
            // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): SDL's key table.
            const bool* keys = SDL_GetKeyboardState(nullptr);
            const auto axis = [&](SDL_Scancode positive, SDL_Scancode negative) {
                return io.WantCaptureKeyboard ? 0.0
                                              : (keys[positive] ? 1.0 : 0.0) - (keys[negative] ? 1.0 : 0.0);
            };
            const bool jump = !io.WantCaptureKeyboard && keys[SDL_SCANCODE_SPACE];
            // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            // The keys as for a vessel (D27): W A S D Q E turn, never the mouse; the arrows (with
            // the right shift and ctrl for up and down) move: walking, climbing, or the arm.
            const math::Vector3 arrows{axis(SDL_SCANCODE_UP, SDL_SCANCODE_DOWN),
                                       axis(SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT),
                                       axis(SDL_SCANCODE_RSHIFT, SDL_SCANCODE_RCTRL)};
            const double turn_rad = k_turn_rate_rad_s * input_dt_s;
            look_yaw_rad += axis(SDL_SCANCODE_A, SDL_SCANCODE_D) * turn_rad;
            look_pitch_rad += axis(SDL_SCANCODE_S, SDL_SCANCODE_W) * turn_rad;
            // Rationale: with weight the body stays upright; the head only turns about the up.
            const double roll_rad =
                walker->standing ? 0.0
                                 : axis(SDL_SCANCODE_E, SDL_SCANCODE_Q) * k_roll_rate_rad_s * input_dt_s;
            const math::Vector3 move = walker->standing || walker->on_ladder ? arrows : math::Vector3{};
            turn(look, look_yaw_rad, look_pitch_rad, roll_rad, walker->up, input_dt_s);
            // With a headset on, the hand goes where the head is turned.
            const math::Quaternion gaze =
                headset_frame.drawn ? view_of(look) * headset_frame.head.orientation : view_of(look);
            // Where the hand reaches: through its place on the picture, in the pilot's axes
            // (forward, left, up).
            math::Vector3 hand;
            if (!headset_frame.drawn && ui_state.hand_x_px >= 0.0F && io.DisplaySize.x > 0.0F
                && io.DisplaySize.y > 0.0F) {
                const double half_height = std::tan(0.5 * k_cockpit_field_of_view_rad);
                const double aspect = io.DisplaySize.x / io.DisplaySize.y;
                hand = {1.0, -((2.0 * ui_state.hand_x_px / io.DisplaySize.x) - 1.0) * half_height * aspect,
                        (1.0 - (2.0 * ui_state.hand_y_px / io.DisplaySize.y)) * half_height};
            }
            // The arm: the mouse's drag this frame as a speed, smoothed, as the hand moving over
            // the surface it holds (drag_to_arm); the arrows as well, at a steady pace.
            const math::Vector3 normal = math::rotate(math::conjugate(gaze), walker->grip_normal);
            const math::Vector3 dragged_m_s =
                input_dt_s > 0.0 ? (k_arm_m_per_px / input_dt_s) * drag_to_arm(arm_dx_px, arm_dy_px, normal)
                                 : math::Vector3{};
            arm_dx_px = 0.0;
            arm_dy_px = 0.0;
            const double blend = std::min(input_dt_s / k_arm_smoothing_s, 1.0);
            // Rationale: the mouse drags only while its button is held; the arrows work the arm
            // of whichever hand holds, the latched one too.
            const math::Vector3 wanted_m_s =
                (hand_closed || latch_closed ? dragged_m_s : math::Vector3{}) + k_arm_key_m_s * arrows;
            arm_m_s = walker->grabbing ? arm_m_s + blend * (wanted_m_s - arm_m_s) : math::Vector3{};
            const sim::PilotInput input{.view = gaze,
                                        .move = move,
                                        .hand = hand,
                                        .arm_m_s = arm_m_s,
                                        .grab = hand_closed,
                                        .latch = latch_closed,
                                        .jump = jump};
            host.post([input](sim::Simulation& simulation) { simulation.set_pilot_input(input); });
        } else if (!io.WantCaptureKeyboard) {
            const bool* keys = SDL_GetKeyboardState(nullptr);
            for (const KeyBinding& binding : k_key_bindings) {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): SDL's key table.
                if (keys[binding.key]) {
                    flight_actions.set(binding.action, true, flight_actions.was_pressed(binding.action));
                }
            }
        }

        if (flown != nullptr) {
            const math::Matrix3 seat_axes = shown == ViewMode::Cockpit
                                                ? render::seat_to_vessel(*flown).value_or(math::Matrix3{})
                                                : math::Matrix3{};
            std::ranges::copy(flight_commands(*flown, flight_actions, seat_axes, input_dt_s, control_state),
                              std::back_inserter(commands));
        }
        post_commands(host, commands);

        if (options.hold_warp) {
            host.post([level = options.warp_level](sim::Simulation& simulation) {
                simulation.time_warp().request_level(level);
            });
        }
        if (deterministic) {
            host.step(k_fixed_frame_s);
            snapshot = host.latest();
        }
        const auto now = std::chrono::steady_clock::now();
        const double frame_s = std::chrono::duration<double>(now - previous_frame).count();
        previous_frame = now;
        if (frame_s > 0.0) {
            frames_per_second += 0.05 * (1.0 / frame_s - frames_per_second);
        }
        last_frame_s = frame_s;
        if (!snapshot) {
            bgfx::touch(k_map_view);
            if (core::VoidResult presented = present(); !presented) {
                return presented;
            }
            continue;
        }

        // --- Camera and the 3-D view --------------------------------------------------------
        const double aspect = static_cast<double>(width_px) / std::max(height_px, 1);
        const render::gpu::ViewTarget target{.view_id = k_map_view,
                                             .width = static_cast<std::uint16_t>(width_px),
                                             .height = static_cast<std::uint16_t>(height_px)};
        const auto flown_now = flown_vessel(*snapshot);
        const ViewMode drawn = available_view(wanted_view, *snapshot);
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        const float display_height = io.DisplaySize.y;
        UiActions actions;
        if (drawn == ViewMode::Map || !flown_now.has_value()) {
            flight_frame.reset();
            camera.zoom(1.0, 1.02 * snapshot->focus_radius_m);
            const math::Vector3 camera_m = camera.offset_from_focus_m();
            const double near_m = std::max(1.0, 0.5 * (camera.distance_m - snapshot->focus_radius_m));
            const render::gpu::ViewCamera view_camera{
                .view = render::view_matrix(camera.universe_to_camera()),
                .projection = render::reversed_infinite_projection(camera.vertical_field_of_view_rad, aspect,
                                                                   near_m, caps.homogeneousDepth)};
            const render::FrameGeometry geometry = render::build_frame_geometry(*snapshot, camera_m);
            map_renderer->draw(geometry, view_camera, target);
            const ScreenProjection screen{
                .view_projection = render::multiply(view_camera.projection, view_camera.view),
                .width_px = io.DisplaySize.x,
                .height_px = display_height,
                .focal_length_px = static_cast<float>(0.5 * display_height
                                                      / std::tan(0.5 * camera.vertical_field_of_view_rad))};
            actions = draw_map_view_ui(*snapshot, geometry, camera_m, screen, ui_state, frames_per_second);
        } else {
            const sim::VesselView& vessel = snapshot->vessels[*flown_now];
            const bool from_seat = drawn == ViewMode::Cockpit;
            const std::optional<sim::PilotView> walking = from_seat ? pilot_afoot(*snapshot) : std::nullopt;
            // With a headset on, the head is where the headset is; otherwise where the mouse
            // has turned it.
            const bool tracked = from_seat && headset_frame.drawn;
            const auto seen_from = [&](const render::TrackedPose& pose) -> std::optional<render::ViewPoint> {
                if (walking.has_value()) {
                    return render::pilot_view_point(vessel, *walking, view_of(look), pose);
                }
                return render::seat_view_point(vessel, pose);
            };
            std::optional<render::ViewPoint> eyes;
            if (tracked) {
                eyes = seen_from(headset_frame.head);
            } else if (walking.has_value()) {
                eyes = render::pilot_view_point(vessel, *walking, view_of(look));
            } else if (from_seat) {
                eyes = render::seat_view_point(vessel, head);
            }
            const render::ViewPoint view = eyes.value_or(render::chase_view_point(*snapshot, vessel, chase));
            const double field_of_view_rad =
                from_seat ? k_cockpit_field_of_view_rad : chase.vertical_field_of_view_rad;
            const double near_m = from_seat ? k_cockpit_near_m : std::max(0.1, 0.05 * chase.distance_m);
            const render::gpu::ViewCamera view_camera{
                .view = render::view_matrix(view.universe_to_camera),
                .projection = render::reversed_infinite_projection(field_of_view_rad, aspect, near_m,
                                                                   caps.homogeneousDepth)};
            render::FlightGeometry geometry =
                render::build_flight_geometry(*snapshot, *flown_now, view, from_seat);
            map_renderer->draw(geometry.scene, view_camera, target);
            solid_renderer->draw(geometry, k_map_view);
            // The same geometry once more for each eye (BRIEFING 12.1 rule 1): it was built
            // about the head, and an eye is a few centimetres from there.
            for (std::size_t eye = 0; tracked && headset.has_value() && eye < xr::k_eyes; ++eye) {
                const std::optional<render::ViewPoint> eye_point = seen_from(headset_frame.eyes.at(eye).pose);
                if (!eye_point.has_value()) {
                    break;
                }
                const render::gpu::ViewCamera eye_camera{
                    .view =
                        render::view_matrix(eye_point->universe_to_camera,
                                            eye_point->position_from_focus_m - view.position_from_focus_m),
                    .projection = render::reversed_infinite_projection(
                        headset_frame.eyes.at(eye).field_of_view, k_cockpit_near_m, caps.homogeneousDepth)};
                const render::gpu::ViewTarget eye_target =
                    headset->target(headset_frame, eye, k_eye_views.at(eye));
                map_renderer->draw(geometry.scene, eye_camera, eye_target);
                solid_renderer->draw(geometry, eye_target.view_id);
            }
            const ScreenProjection screen{
                .view_projection = render::multiply(view_camera.projection, view_camera.view),
                .width_px = io.DisplaySize.x,
                .height_px = display_height,
                .focal_length_px =
                    static_cast<float>(0.5 * display_height / std::tan(0.5 * field_of_view_rad))};
            actions = draw_flight_view_ui(*snapshot, view.position_from_focus_m, screen, ui_state,
                                          frames_per_second, from_seat);
            flight_frame = FlightFrame{
                .geometry = std::move(geometry), .view = view, .view_projection = screen.view_projection};
        }
        ImGui::Render();
        ui_renderer->render(ImGui::GetDrawData());

        if (actions.focus.has_value()) {
            host.set_focus(*actions.focus);
            sim::SceneSnapshot framed = *snapshot;
            framed.focus = *actions.focus;
            framed.focus_radius_m = actions.focus->kind == sim::Focus::Kind::Body
                                        ? snapshot->bodies[actions.focus->index].radius_m
                                        : 0.0;
            camera.distance_m = framing_distance_m(framed);
        }
        if (actions.warp_level.has_value()) {
            host.post([level = *actions.warp_level](sim::Simulation& simulation) {
                simulation.time_warp().request_level(level);
            });
        }
        if (actions.paused.has_value()) {
            host.post([paused = *actions.paused](sim::Simulation& simulation) {
                simulation.time_warp().set_paused(paused);
            });
        }
        if (actions.burn.has_value()) {
            post_burn(host, *actions.burn);
        }
        post_commands(host, actions.commands);

        ++frame;
        if (deterministic && frame >= *options.frames) {
            running = false;
            if (options.screenshot.has_value()) {
                bgfx::requestScreenShot(BGFX_INVALID_HANDLE, options.screenshot->string().c_str());
            }
        }
        if (core::VoidResult presented = present(); !presented) {
            return presented;
        }
        if (headset.has_value()) {
            vr_report_s += frame_s;
            vr_frames += headset_frame.drawn ? 1 : 0;
            if (vr_report_s >= k_vr_report_s) {
                LOG_INFO("VR: {:.1f} frames/s drawn; the headset asks for {:.1f}", vr_frames / vr_report_s,
                         headset->frame_period_s() > 0.0 ? 1.0 / headset->frame_period_s() : 0.0)
                    .tag("subsystem", "xr");
                vr_report_s = 0.0;
                vr_frames = 0;
            }
        }
    }
    if (options.screenshot.has_value()) {
        // The capture is read back during the next frames.
        for (int flush = 0; flush < 3 && !callback.screenshot_written(); ++flush) {
            bgfx::frame();
        }
        if (!callback.screenshot_written()) {
            return core::fail(ErrorCode::IoFailure, "the screenshot was not written");
        }
    }
    return {};
}

} // namespace helios::app
