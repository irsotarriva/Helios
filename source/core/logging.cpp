#include "helios/core/logging.hpp"

#include <cstdlib>
#include <format>
#include <memory>
#include <string>
#include <utility>

namespace helios::core {

namespace {

[[nodiscard]] VoidResult validate_json_path(const std::filesystem::path& json_path) {
    const std::filesystem::path parent =
        json_path.has_parent_path() ? json_path.parent_path() : std::filesystem::path{"."};
    std::error_code filesystem_error;
    const bool parent_is_directory = std::filesystem::is_directory(parent, filesystem_error);
    if (filesystem_error || !parent_is_directory) {
        return fail(ErrorCode::FileNotFound,
                    std::format("log directory '{}' does not exist", parent.string()));
    }
    return {};
}

[[nodiscard]] std::optional<std::string> environment_variable(const char* name) {
    // NOLINTNEXTLINE(concurrency-mt-unsafe): documented as startup-only, before threads exist.
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string{value};
}

} // namespace

LoggingOptions logging_options_from_environment(LoggingOptions defaults) {
    if (const auto terminal = environment_variable("HELIOS_LOG_TERMINAL")) {
        defaults.enable_terminal = *terminal != "off";
        if (defaults.enable_terminal) {
            defaults.terminal_query = *terminal;
        }
    }
    if (const auto json_path = environment_variable("HELIOS_LOG_JSON")) {
        defaults.json_path = std::filesystem::path{*json_path};
    }
    if (const auto json_query = environment_variable("HELIOS_LOG_JSON_QUERY")) {
        defaults.json_query = *json_query;
    }
    return defaults;
}

Result<lumen::Predicate> parse_log_query(std::string_view query) {
    auto parsed = try_call(ErrorCode::ExternalLibraryFailure, "parsing log query",
                           [&] { return lumen::parse_predicate(query); });
    if (!parsed) {
        return std::unexpected(parsed.error());
    }
    if (!*parsed) {
        const lumen::PredicateParseError& error = parsed->error();
        return fail(ErrorCode::ParseFailure,
                    std::format("invalid log query: {} at column {}\n  {}\n  {}^", error.message,
                                error.position + 1, query, std::string(error.position, ' ')));
    }
    return std::move(**parsed);
}

Result<InstalledSinks> initialise_logging(const LoggingOptions& options) {
    // Validate everything first so a bad option never leaves half the sinks installed.
    std::optional<lumen::Predicate> terminal_filter;
    if (options.enable_terminal) {
        auto parsed = parse_log_query(options.terminal_query);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        terminal_filter = std::move(*parsed);
    }
    std::optional<lumen::Predicate> json_filter;
    if (options.json_path.has_value()) {
        if (VoidResult valid = validate_json_path(*options.json_path); !valid) {
            return std::unexpected(valid.error());
        }
        auto parsed = parse_log_query(options.json_query);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        json_filter = std::move(*parsed);
    }

    InstalledSinks installed;
    // Rationale: Lumen's sink constructors start writer threads and may throw.
    return try_call(ErrorCode::ExternalLibraryFailure, "installing Lumen sinks",
                    [&] {
                        lumen::Core& lumen_core = lumen::core();
                        lumen_core.set_process_tag("app", "helios");
                        lumen_core.set_process_tag("version", HELIOS_VERSION);

                        if (terminal_filter.has_value()) {
                            installed.ids.push_back(lumen_core.add_sink(
                                std::make_unique<lumen::TerminalSink>(lumen::TerminalSink::default_config()),
                                std::move(*terminal_filter)));
                        }
                        if (json_filter.has_value()) {
                            const std::string path_string = options.json_path->string();
                            installed.ids.push_back(lumen_core.add_sink(
                                std::make_unique<lumen::JsonSink>(path_string), std::move(*json_filter)));
                        }
                    })
        .transform([&] { return std::move(installed); });
}

VoidResult shutdown_logging(InstalledSinks& sinks) {
    return try_call(ErrorCode::ExternalLibraryFailure, "removing Lumen sinks", [&] {
        lumen::Core& lumen_core = lumen::core();
        // Blocks until every record emitted so far has reached the sinks.
        lumen_core.flush();
        for (const lumen::SinkId id : sinks.ids) {
            // A sink id that is already gone is harmless here; nothing to report.
            [[maybe_unused]] const auto removed = lumen_core.remove_sink(id);
        }
        sinks.ids.clear();
    });
}

} // namespace helios::core
