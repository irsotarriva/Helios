#ifndef HELIOS_APPS_HELIOS_APPLICATION_HPP
#define HELIOS_APPS_HELIOS_APPLICATION_HPP

#include "helios/core/error.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace helios::app {

struct Options {
    std::filesystem::path data_dir;                 // data/solar_system
    std::optional<std::filesystem::path> ephemeris; // full DE440 .hce; mean elements otherwise
    std::string renderer = "auto";                  // auto | metal | vulkan | opengl | d3d11 | d3d12
    int width_px = 1600;
    int height_px = 900;
    std::optional<double> start_unix_utc_s; // default: now
    std::string focus = "Earth";
    // map | outside | cockpit. The flight views need a vessel as the focus, and the cockpit a
    // vessel with a seat; otherwise the next one down is shown.
    std::string view = "map";
    bool leave_seat = false; // start out of the seat, in the cabin (where the vessel has one)
    bool go_outside = false; // ... and from there out through the airlock, in a suit
    // Draw the pilot's view for a headset too (OpenXR; Direct3D 11 only). Starts in the cockpit.
    bool vr = false;
    std::size_t warp_level = 2; // index into k_warp_levels (2 = real time)
    bool demo_vessels = true;
    // Deterministic run: fixed 1/60 s ticks on the main thread, stop after `frames`, optionally
    // writing a PNG of the last frame.
    std::optional<int> frames;
    bool hold_warp = false; // keep re-requesting warp_level (burns normally drop it to real time)
    std::optional<std::filesystem::path> screenshot;
    // Initial camera, when given (otherwise framed on the focus). In the cockpit the yaw and
    // pitch are those of the pilot's head.
    std::optional<double> camera_distance_m;
    std::optional<double> camera_yaw_deg;
    std::optional<double> camera_pitch_deg;
};

// Opens the window and runs the game until the player quits (or `frames` have run).
[[nodiscard]] core::VoidResult run(const Options& options);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_APPLICATION_HPP
