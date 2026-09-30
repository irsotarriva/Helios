#include "helios/core/logging.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>

namespace {

using helios::core::ErrorCode;
using helios::core::initialise_logging;
using helios::core::LoggingOptions;
using helios::core::shutdown_logging;

[[nodiscard]] std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

TEST(Logging, JsonSinkReceivesFormattedRecordsWithProcessTags) {
    const std::filesystem::path json_path =
        std::filesystem::temp_directory_path() / "helios_test_logging.jsonl";
    std::filesystem::remove(json_path);

    const LoggingOptions options{.enable_terminal = false, .json_path = json_path};
    auto installed = initialise_logging(options);
    ASSERT_TRUE(installed.has_value());
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

TEST(Logging, RejectsJsonPathInMissingDirectory) {
    const LoggingOptions options{.enable_terminal = false,
                                 .json_path = std::filesystem::temp_directory_path() / "helios_no_such_dir"
                                              / "log.jsonl"};
    const auto installed = initialise_logging(options);
    ASSERT_FALSE(installed.has_value());
    EXPECT_EQ(installed.error().code, ErrorCode::FileNotFound);
}

} // namespace
