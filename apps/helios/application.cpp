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
constexpr std::array k_key_bindings{KeyBinding{SDL_SCANCODE_LSHIFT, Action::ThrottleUp},
                                    KeyBinding{SDL_SCANCODE_LCTRL, Action::ThrottleDown},
                                    KeyBinding{SDL_SCANCODE_Z, Action::ThrottleFull},
                                    KeyBinding{SDL_SCANCODE_X, Action::ThrottleCut},
                                    KeyBinding{SDL_SCANCODE_S, Action::PitchUp},
                                    KeyBinding{SDL_SCANCODE_W, Action::PitchDown},
                                    KeyBinding{SDL_SCANCODE_A, Action::YawLeft},
                                    KeyBinding{SDL_SCANCODE_D, Action::YawRight},
                                    KeyBinding{SDL_SCANCODE_Q, Action::RollLeft},
                                    KeyBinding{SDL_SCANCODE_E, Action::RollRight},
                                    KeyBinding{SDL_SCANCODE_RETURN, Action::Stage},
                                    KeyBinding{SDL_SCANCODE_T, Action::ToggleAttitudeHold}};

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
[[nodiscard]] std::optional<sim::PilotView> pilot_afoot(const sim::SceneSnapshot& snapshot) noexcept {
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
    const std::size_t next = (current + 1) % total;
    if (next < bodies) {
        return sim::Focus::body(bodies::BodyId{static_cast<std::uint32_t>(next)});
    }
    return sim::Focus::vessel(sim::VesselId{static_cast<std::uint32_t>(next - bodies)});
}

[[nodiscard]] bool init_bgfx(const NativeWindow& native, bgfx::RendererType::Enum type, int width_px,
                             int height_px, BgfxCallback& callback, bgfx::TextureFormat::Enum depth_format) {
    bgfx::Init init;
    init.type = type;
    init.fallback = type == bgfx::RendererType::Count;
    init.swapChain.nwh = native.window_handle;
    init.swapChain.ndt = native.display_handle;
    init.platformData.type = native.type;
    init.swapChain.width = static_cast<std::uint32_t>(width_px);
    init.swapChain.height = static_cast<std::uint32_t>(height_px);
    init.swapChain.flags = BGFX_SWAP_CHAIN_MSAA_X4;
    init.swapChain.formatDepthStencil = depth_format;
    init.reset = BGFX_RESET_VSYNC;
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
    const auto requested_renderer = renderer_type(options.renderer);
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
    platform.bgfx_ready =
        (try_float_depth
         && init_bgfx(*native, *requested_renderer, width_px, height_px, callback, bgfx::TextureFormat::D32F))
        || init_bgfx(*native, *requested_renderer, width_px, height_px, callback, bgfx::TextureFormat::D24S8);
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
    ViewMode wanted_view = *requested_view;
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
    // The pilot out of the seat: where they look, and whether the hand is closed.
    PilotLook look;
    bool hand_closed = false;
    bool was_afoot = false;
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
        if (afoot != was_afoot) {
            // Rationale: a deterministic run has no one at the mouse, and must not take it.
            SDL_SetWindowRelativeMouseMode(platform.window, afoot && !deterministic);
            was_afoot = afoot;
            hand_closed = false;
            grab.reset();
        }
        double look_yaw_rad = 0.0;
        double look_pitch_rad = 0.0;
        if (leave_seat_wanted && flown != nullptr && shown == ViewMode::Cockpit) {
            leave_seat_wanted = false;
            look = look_from_seat(render::seat_to_vessel(*flown).value_or(math::Matrix3{}), head.yaw_rad,
                                  head.pitch_rad);
            host.post([](sim::Simulation& simulation) {
                if (core::VoidResult left = simulation.leave_seat(); !left) {
                    LOG_WARN("{}", core::describe(left.error())).tag("subsystem", "ui");
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
                bgfx::reset(BGFX_RESET_VSYNC, &swap_chain);
                break;
            }
            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                dragging = !io.WantCaptureMouse;
                if (!dragging || shown != ViewMode::Cockpit || !flight_frame.has_value()
                    || flown == nullptr) {
                    break;
                }
                // In the cockpit the pointer is a hand: on an instrument it handles it, anywhere
                // else it turns the head.
                const auto cockpit = cockpit_of(*flown);
                // Afoot the hand is where the pilot looks: the middle of the picture.
                const float pointer_x = afoot ? 0.5F * io.DisplaySize.x : event.button.x;
                const float pointer_y = afoot ? 0.5F * io.DisplaySize.y : event.button.y;
                const auto picked = render::pick_instrument(
                    flight_frame->geometry.instruments,
                    render::ray_through_pixel(pointer_x, pointer_y, io.DisplaySize.x, io.DisplaySize.y,
                                              k_cockpit_field_of_view_rad,
                                              flight_frame->view.universe_to_camera));
                if (!cockpit.has_value() || !picked.has_value()) {
                    hand_closed = afoot; // not on an instrument: take hold of what is there
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
            case SDL_EVENT_MOUSE_BUTTON_UP:
                dragging = false;
                hand_closed = false;
                grab.reset();
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
                } else if (afoot) {
                    look_yaw_rad -= event.motion.xrel * k_look_sensitivity_rad_per_px;
                    look_pitch_rad -= event.motion.yrel * k_look_sensitivity_rad_per_px;
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
                } else if (event.key.key == SDLK_SPACE) {
                    host.post([](sim::Simulation& simulation) {
                        simulation.time_warp().set_paused(!simulation.time_warp().paused());
                    });
                } else if (event.key.key == SDLK_F1) {
                    ui_state.show_help = !ui_state.show_help;
                } else if (event.key.key == SDLK_F && !event.key.repeat && flown != nullptr
                           && shown == ViewMode::Cockpit) {
                    if (afoot) {
                        host.post([](sim::Simulation& simulation) {
                            if (core::VoidResult seated = simulation.take_seat(); !seated) {
                                LOG_INFO("{}", core::describe(seated.error())).tag("subsystem", "ui");
                            }
                        });
                    } else {
                        look = look_from_seat(render::seat_to_vessel(*flown).value_or(math::Matrix3{}),
                                              head.yaw_rad, head.pitch_rad);
                        host.post([](sim::Simulation& simulation) {
                            simulation.set_pilot_input({});
                            if (core::VoidResult left = simulation.leave_seat(); !left) {
                                LOG_INFO("{}", core::describe(left.error())).tag("subsystem", "ui");
                            }
                        });
                    }
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
            turn(look, look_yaw_rad, look_pitch_rad,
                 axis(SDL_SCANCODE_E, SDL_SCANCODE_Q) * k_roll_rate_rad_s * input_dt_s, walker->up,
                 input_dt_s);
            const sim::PilotInput input{.view = view_of(look),
                                        .move = {axis(SDL_SCANCODE_W, SDL_SCANCODE_S),
                                                 axis(SDL_SCANCODE_A, SDL_SCANCODE_D),
                                                 axis(SDL_SCANCODE_R, SDL_SCANCODE_V)},
                                        .grab = hand_closed,
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
            bgfx::frame();
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
            std::optional<render::ViewPoint> eyes;
            if (walking.has_value()) {
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
            const ScreenProjection screen{
                .view_projection = render::multiply(view_camera.projection, view_camera.view),
                .width_px = io.DisplaySize.x,
                .height_px = display_height,
                .focal_length_px =
                    static_cast<float>(0.5 * display_height / std::tan(0.5 * field_of_view_rad))};
            actions = draw_flight_view_ui(*snapshot, geometry, view.position_from_focus_m, screen, ui_state,
                                          frames_per_second, walking.has_value());
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
        bgfx::frame();
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
