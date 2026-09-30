#ifndef HELIOS_CORE_LOGGING_HPP
#define HELIOS_CORE_LOGGING_HPP

#include "helios/core/error.hpp"

#include <lumen/lumen.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace helios::core {

// Which records reach which sink is decided by LumenLog query strings, so filters can be
// changed from a config file, the environment or the dev console without recompiling:
//   "level >= WARN || subsystem == orbital"      "vessel_id == 42 && altitude_m < 70000"
// Grammar: lumen/predicate.h (parse_predicate).
struct LoggingOptions {
    bool enable_terminal = true;
    std::string terminal_query = "level >= INFO";
    // Records and metrics as JSON lines, for offline analysis (e.g. pandas).
    std::optional<std::filesystem::path> json_path;
    std::string json_query = "true";
};

// Sinks registered by initialise_logging, so a caller (or a test) can remove them.
struct InstalledSinks {
    std::vector<lumen::SinkId> ids;
};

// `defaults` overridden by HELIOS_LOG_TERMINAL (terminal query, or "off"), HELIOS_LOG_JSON
// (JSON-lines path) and HELIOS_LOG_JSON_QUERY. Read the environment before starting threads.
[[nodiscard]] LoggingOptions logging_options_from_environment(LoggingOptions defaults);

// Parses a query, reporting a bad one with the offending column marked:
//   invalid log query: expected ')' at column 19
//     level >= WARN && (x
//                       ^
[[nodiscard]] Result<lumen::Predicate> parse_log_query(std::string_view query);

// Registers Helios' default sinks with the global Lumen core and sets the process tags
// (`app`, `version`). Call once at startup, before the simulation thread starts.
// Fails without installing anything if a query is invalid or the JSON directory is missing.
[[nodiscard]] Result<InstalledSinks> initialise_logging(const LoggingOptions& options);

// Flushes every queued record, then removes the sinks.
[[nodiscard]] VoidResult shutdown_logging(InstalledSinks& sinks);

} // namespace helios::core

#endif // HELIOS_CORE_LOGGING_HPP
