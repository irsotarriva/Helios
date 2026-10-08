// Helios: the map view (Phase 2) and the flight view (Phase 4).
//
//   helios [--data <dir>] [--ephemeris <de440.hce>] [--renderer auto|metal|vulkan|opengl|d3d11|d3d12]
//          [--size <w>x<h>] [--start-unix <seconds>] [--focus <name>] [--warp <level 0-8>] [--no-demo]
//          [--view map|outside|cockpit] [--leave-seat] [--vr]
//          [--frames <n>] [--screenshot <file.png>] [--hold-warp]
//          [--camera-distance <m>] [--camera-yaw <deg>] [--camera-pitch <deg>]

#include "helios/core/logging.hpp"
#include "helios/core/parse.hpp"

#include <cstdio>
#include <format>
#include <print>
#include <span>
#include <string_view>

#include "application.hpp"

namespace {

using helios::core::ErrorCode;
using helios::core::Result;

[[nodiscard]] Result<helios::app::Options> parse_options(std::span<char*> arguments) {
    helios::app::Options options;
    options.data_dir = HELIOS_DATA_DIR "/solar_system";
    for (std::size_t index = 1; index < arguments.size(); ++index) {
        const std::string_view flag = arguments[index];
        if (flag == "--no-demo") {
            options.demo_vessels = false;
            continue;
        }
        if (flag == "--leave-seat") {
            options.leave_seat = true;
            continue;
        }
        if (flag == "--vr") {
            options.vr = true;
            continue;
        }
        if (flag == "--hold-warp") {
            options.hold_warp = true;
            continue;
        }
        if (index + 1 >= arguments.size()) {
            return helios::core::fail(ErrorCode::InvalidArgument, std::format("'{}' needs a value", flag));
        }
        const std::string_view value = arguments[++index];
        const auto number = [&]() { return helios::core::parse_double(value); };
        const auto store = [&](auto& target) -> Result<bool> {
            const auto parsed = number();
            if (!parsed) {
                return std::unexpected(parsed.error());
            }
            target = static_cast<std::remove_reference_t<decltype(target)>>(*parsed);
            return true;
        };
        Result<bool> stored = true;
        if (flag == "--data") {
            options.data_dir = value;
        } else if (flag == "--ephemeris") {
            options.ephemeris = value;
        } else if (flag == "--renderer") {
            options.renderer = value;
        } else if (flag == "--focus") {
            options.focus = value;
        } else if (flag == "--view") {
            options.view = value;
        } else if (flag == "--screenshot") {
            options.screenshot = value;
        } else if (flag == "--size") {
            const std::size_t cross = value.find('x');
            const auto width = helios::core::parse_int64(value.substr(0, cross));
            const auto height =
                cross == std::string_view::npos ? width : helios::core::parse_int64(value.substr(cross + 1));
            if (!width || !height) {
                return helios::core::fail(ErrorCode::InvalidArgument, "--size expects <width>x<height>");
            }
            options.width_px = static_cast<int>(*width);
            options.height_px = static_cast<int>(*height);
        } else if (flag == "--start-unix") {
            double seconds = 0.0;
            stored = store(seconds);
            options.start_unix_utc_s = seconds;
        } else if (flag == "--warp") {
            stored = store(options.warp_level);
        } else if (flag == "--frames") {
            int frames = 0;
            stored = store(frames);
            options.frames = frames;
        } else if (flag == "--camera-distance") {
            double distance_m = 0.0;
            stored = store(distance_m);
            options.camera_distance_m = distance_m;
        } else if (flag == "--camera-yaw") {
            double yaw_deg = 0.0;
            stored = store(yaw_deg);
            options.camera_yaw_deg = yaw_deg;
        } else if (flag == "--camera-pitch") {
            double pitch_deg = 0.0;
            stored = store(pitch_deg);
            options.camera_pitch_deg = pitch_deg;
        } else {
            return helios::core::fail(ErrorCode::InvalidArgument, std::format("unknown option '{}'", flag));
        }
        if (!stored) {
            return std::unexpected(stored.error());
        }
    }
    if (options.screenshot.has_value() && !options.frames.has_value()) {
        options.frames = 120;
    }
    return options;
}

// Rationale: not std::println, which some Apple toolchains' libc++ still lacks.
void report(const std::string& message) {
    std::fputs(std::format("helios: {}\n", message).c_str(), stderr);
}

[[nodiscard]] int run_program(std::span<char*> arguments) {
    auto sinks = helios::core::initialise_logging(helios::core::logging_options_from_environment({}));
    if (!sinks) {
        report(helios::core::describe(sinks.error()));
        return 1;
    }
    const auto options = parse_options(arguments);
    helios::core::VoidResult result =
        options ? helios::app::run(*options) : helios::core::VoidResult(std::unexpected(options.error()));
    if (!result) {
        LOG_ERROR("{}", helios::core::describe(result.error()));
    }
    if (helios::core::VoidResult flushed = helios::core::shutdown_logging(*sinks); !flushed) {
        report(helios::core::describe(flushed.error()));
    }
    if (!result) {
        report(helios::core::describe(result.error()));
        return 1;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::span arguments(argv, static_cast<std::size_t>(argc));
    // The exception boundary: nothing in Helios throws, but the standard library may.
    const auto exit_code = helios::core::try_call(helios::core::ErrorCode::Unknown, "helios",
                                                  [&] { return run_program(arguments); });
    return exit_code.value_or(1);
}
