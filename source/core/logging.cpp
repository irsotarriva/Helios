#include "helios/core/logging.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

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

// Waits until Lumen's dispatch thread has emptied its ring buffers.
// TODO: replace with lumen::Core::flush() once it drains synchronously upstream; today it
// only wakes the dispatch thread, so records emitted just before shutdown can be lost.
[[nodiscard]] VoidResult wait_for_lumen_dispatch(lumen::Core& lumen_core) {
    using namespace std::chrono_literals;
    constexpr std::chrono::milliseconds k_drain_timeout{2000};
    // Lumen's dispatch loop polls every 10 ms; one extra period lets a record that was
    // popped but not yet handed to the sinks finish dispatching.
    constexpr std::chrono::milliseconds k_dispatch_period{20};

    const auto deadline = std::chrono::steady_clock::now() + k_drain_timeout;
    lumen_core.flush();
    while (!lumen_core.log_buffer().empty() || !lumen_core.metric_buffer().empty()
           || !lumen_core.progress_buffer().empty()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return fail(ErrorCode::Timeout, "Lumen dispatch did not drain its buffers");
        }
        lumen_core.flush();
        std::this_thread::sleep_for(1ms);
    }
    std::this_thread::sleep_for(k_dispatch_period);
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
    return try_call(ErrorCode::ExternalLibraryFailure, "removing Lumen sinks",
                    [&]() -> VoidResult {
                        lumen::Core& lumen_core = lumen::core();
                        const VoidResult drained = wait_for_lumen_dispatch(lumen_core);
                        for (const lumen::SinkId id : sinks.ids) {
                            auto removed = lumen_core.remove_sink(id);
                            // A sink id that is already gone is harmless here; nothing to flush.
                            if (removed.has_value() && *removed != nullptr) {
                                (*removed)->flush();
                            }
                        }
                        sinks.ids.clear();
                        return drained;
                    })
        .and_then([](VoidResult drained) { return drained; });
}

} // namespace helios::core
