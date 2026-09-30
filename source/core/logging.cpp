#include "helios/core/logging.hpp"

#include <memory>
#include <string>

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

} // namespace

Result<InstalledSinks> initialise_logging(const LoggingOptions& options) {
    InstalledSinks installed;

    if (options.json_path.has_value()) {
        if (VoidResult valid = validate_json_path(*options.json_path); !valid) {
            return std::unexpected(valid.error());
        }
    }

    // Rationale: Lumen's sink constructors open files and allocate; they are third-party
    // code and may throw, so everything that touches them goes through try_call.
    return try_call(ErrorCode::ExternalLibraryFailure, "installing Lumen sinks",
                    [&] {
                        lumen::Core& lumen_core = lumen::core();
                        lumen_core.set_process_tag("app", "helios");
                        lumen_core.set_process_tag("version", HELIOS_VERSION);

                        if (options.enable_terminal) {
                            installed.ids.push_back(lumen_core.add_sink(
                                std::make_unique<lumen::TerminalSink>(lumen::TerminalSink::default_config()),
                                lumen::level_at_least(options.terminal_minimum_level)));
                        }
                        if (options.json_path.has_value()) {
                            const std::string path_string = options.json_path->string();
                            installed.ids.push_back(lumen_core.add_sink(
                                std::make_unique<lumen::JsonSink>(path_string), lumen::always()));
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
