#include "helios/core/logging.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>

namespace {

using helios::core::ErrorCode;
using helios::core::initialise_logging;
using helios::core::LoggingOptions;
using helios::core::parse_log_query;
using helios::core::shutdown_logging;

[[nodiscard]] std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

[[nodiscard]] std::filesystem::path fresh_log_path(const std::string& name) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(path);
    return path;
}

TEST(Logging, JsonSinkReceivesFormattedRecordsWithProcessTags) {
    const std::filesystem::path json_path = fresh_log_path("helios_test_logging.jsonl");
    auto installed = initialise_logging({.enable_terminal = false, .json_path = json_path});
    ASSERT_TRUE(installed.has_value()) << helios::core::describe(installed.error());
    EXPECT_EQ(installed->ids.size(), 1U);

    {
        // The formatted temporary dies at the end of this scope, before dispatch may run.
        const std::string vessel_name = "Pathfinder";
        LOG_INFO("vessel {} reached epoch {}", vessel_name, 42).tag("subsystem", "time");
    }
    ASSERT_TRUE(shutdown_logging(*installed).has_value());
    EXPECT_TRUE(installed->ids.empty());

    const std::string contents = read_file(json_path);
    EXPECT_NE(contents.find("vessel Pathfinder reached epoch 42"), std::string::npos) << contents;
    EXPECT_NE(contents.find("helios"), std::string::npos) << contents;
    EXPECT_NE(contents.find("subsystem"), std::string::npos) << contents;
}

TEST(Logging, QueryFiltersRecordsByLevelAndNumericTags) {
    const std::filesystem::path json_path = fresh_log_path("helios_test_logging_query.jsonl");
    auto installed = initialise_logging({.enable_terminal = false,
                                         .json_path = json_path,
                                         .json_query = "level >= WARN && altitude_m < 70000"});
    ASSERT_TRUE(installed.has_value()) << helios::core::describe(installed.error());

    LOG_WARN("low and warned").tag("altitude_m", 69'999.97);
    LOG_WARN("high and warned").tag("altitude_m", 120'000.0);
    LOG_INFO("low but only info").tag("altitude_m", 1'000.0);
    LOG_WARN("no altitude at all");
    ASSERT_TRUE(shutdown_logging(*installed).has_value());

    const std::string contents = read_file(json_path);
    EXPECT_NE(contents.find("low and warned"), std::string::npos) << contents;
    EXPECT_EQ(contents.find("high and warned"), std::string::npos) << contents;
    EXPECT_EQ(contents.find("low but only info"), std::string::npos) << contents;
    EXPECT_EQ(contents.find("no altitude at all"), std::string::npos) << contents;
}

TEST(Logging, BadQueryIsReportedWithItsColumnAndInstallsNothing) {
    const auto parsed = parse_log_query("level >= WARN && (x");
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error().code, ErrorCode::ParseFailure);
    EXPECT_NE(parsed.error().context.find("column"), std::string::npos) << parsed.error().context;
    EXPECT_NE(parsed.error().context.find('^'), std::string::npos) << parsed.error().context;

    const auto installed = initialise_logging({.terminal_query = "level >>= WARN"});
    ASSERT_FALSE(installed.has_value());
    EXPECT_EQ(installed.error().code, ErrorCode::ParseFailure);
}

TEST(Logging, RejectsJsonPathInMissingDirectory) {
    const auto installed = initialise_logging(
        {.enable_terminal = false,
         .json_path = std::filesystem::temp_directory_path() / "helios_no_such_dir" / "log.jsonl"});
    ASSERT_FALSE(installed.has_value());
    EXPECT_EQ(installed.error().code, ErrorCode::FileNotFound);
}

TEST(Logging, EnvironmentOverridesDefaults) {
    // NOLINTBEGIN(concurrency-mt-unsafe): single-threaded test setup.
    ::setenv("HELIOS_LOG_TERMINAL", "level >= ERROR", 1);
    ::setenv("HELIOS_LOG_JSON", "/tmp/helios_env.jsonl", 1);
    const LoggingOptions options = helios::core::logging_options_from_environment({});
    ::setenv("HELIOS_LOG_TERMINAL", "off", 1);
    const LoggingOptions silenced = helios::core::logging_options_from_environment({});
    ::unsetenv("HELIOS_LOG_TERMINAL");
    ::unsetenv("HELIOS_LOG_JSON");
    // NOLINTEND(concurrency-mt-unsafe)

    EXPECT_EQ(options.terminal_query, "level >= ERROR");
    ASSERT_TRUE(options.json_path.has_value());
    EXPECT_EQ(*options.json_path, std::filesystem::path{"/tmp/helios_env.jsonl"});
    EXPECT_FALSE(silenced.enable_terminal);
}

} // namespace
