#pragma once

#include "helios/core/error.hpp"

#include <lumen/lumen.h>

#include <filesystem>
#include <optional>
#include <vector>

// Usage constraint (LumenLog @ d3ae5d5): records are dispatched asynchronously and store
// their message and string tags as std::string_view. Only pass strings that outlive the
// dispatch (string literals, interned names). Put runtime values in numeric tags:
//
//     LOG_INFO("domain change").tag("vessel_id", std::int64_t{id}).tag("altitude_m", altitude);
//
// Never LOG_INFO(std::format(...)): the temporary is freed before the sink reads it.
// TODO: lift once LumenLog copies messages into its arena (upstream issue).

namespace helios::core {

struct LoggingOptions {
    bool enable_terminal = true;
    lumen::LogLevel terminal_minimum_level = lumen::LogLevel::INFO;
    // Every record and metric, as JSON lines, for offline analysis (e.g. pandas).
    std::optional<std::filesystem::path> json_path;
};

// Sinks registered by initialise_logging, so a caller (or a test) can remove them.
struct InstalledSinks {
    std::vector<lumen::SinkId> ids;
};

// Registers Helios' default sinks with the global Lumen core and sets the process tags
// (`app`, `version`). Call once at startup, before the simulation thread starts.
[[nodiscard]] Result<InstalledSinks> initialise_logging(const LoggingOptions& options);

// Removes the sinks and flushes everything still queued.
[[nodiscard]] VoidResult shutdown_logging(InstalledSinks& sinks);

} // namespace helios::core
