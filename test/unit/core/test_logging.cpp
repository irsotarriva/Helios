#include "helios/core/logging.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

using helios::core::ErrorCode;
using helios::core::initialise_logging;
using helios::core::LoggingOptions;
using helios::core::shutdown_logging;

TEST_CASE("JSON sink receives records with Helios process tags", "[core][logging]") {
    const std::filesystem::path json_path =
        std::filesystem::temp_directory_path() / "helios_test_logging.jsonl";
    std::filesystem::remove(json_path);

    const LoggingOptions options{.enable_terminal = false, .json_path = json_path};
    auto installed = initialise_logging(options);
    REQUIRE(installed.has_value());
    CHECK(installed->ids.size() == 1);

    LOG_INFO("epoch reached").tag("subsystem", "time").tag("epoch_seconds", std::int64_t{42});
    REQUIRE(shutdown_logging(*installed).has_value());
    CHECK(installed->ids.empty());

    std::ifstream stream(json_path);
    REQUIRE(stream.is_open());
    const std::string contents{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
    CHECK(contents.find("epoch reached") != std::string::npos);
    CHECK(contents.find("epoch_seconds") != std::string::npos);
    CHECK(contents.find("helios") != std::string::npos);
    CHECK(contents.find("subsystem") != std::string::npos);
}

TEST_CASE("A JSON path in a missing directory is rejected", "[core][logging]") {
    const LoggingOptions options{.enable_terminal = false,
                                 .json_path = std::filesystem::temp_directory_path() / "helios_no_such_dir"
                                              / "log.jsonl"};
    const auto installed = initialise_logging(options);
    REQUIRE_FALSE(installed.has_value());
    CHECK(installed.error().code == ErrorCode::FileNotFound);
}
