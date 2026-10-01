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
    std::size_t warp_level = 2; // index into k_warp_levels (2 = real time)
    bool demo_vessels = true;
    // Deterministic run: fixed 1/60 s ticks on the main thread, stop after `frames`, optionally
    // writing a PNG of the last frame.
    std::optional<int> frames;
    bool hold_warp = false; // keep re-requesting warp_level (burns normally drop it to real time)
    std::optional<std::filesystem::path> screenshot;
    // Initial camera, when given (otherwise framed on the focus).
    std::optional<double> camera_distance_m;
    std::optional<double> camera_yaw_deg;
    std::optional<double> camera_pitch_deg;
};

// Opens the window and runs the map view until the player quits (or `frames` have run).
[[nodiscard]] core::VoidResult run(const Options& options);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_APPLICATION_HPP
