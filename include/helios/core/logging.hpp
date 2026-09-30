#ifndef HELIOS_CORE_LOGGING_HPP
#define HELIOS_CORE_LOGGING_HPP

#include "helios/core/error.hpp"

#include <lumen/lumen.h>

#include <filesystem>
#include <optional>
#include <vector>

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

#endif // HELIOS_CORE_LOGGING_HPP
