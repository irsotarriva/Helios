#include "helios/core/error.hpp"

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>

using helios::core::describe;
using helios::core::ErrorCode;
using helios::core::fail;
using helios::core::Result;
using helios::core::try_call;
using helios::core::VoidResult;

namespace {

[[nodiscard]] Result<int> parse_positive(int value) {
    if (value <= 0) {
        return fail(ErrorCode::InvalidArgument, "value must be positive");
    }
    return value;
}

} // namespace

TEST_CASE("ErrorCode converts to std::error_code in the helios category", "[core][error]") {
    const std::error_code code = ErrorCode::Overflow;
    CHECK(code.category().name() == std::string{"helios"});
    CHECK(code.message() == "arithmetic overflow");
    CHECK(code == ErrorCode::Overflow);
    CHECK(static_cast<bool>(code));
}

TEST_CASE("Results propagate through monadic chains", "[core][error]") {
    const Result<int> doubled = parse_positive(21).transform([](int value) { return value * 2; });
    REQUIRE(doubled.has_value());
    CHECK(*doubled == 42);

    const Result<int> failed = parse_positive(-1).transform([](int value) { return value * 2; });
    REQUIRE_FALSE(failed.has_value());
    CHECK(failed.error().code == ErrorCode::InvalidArgument);
    CHECK(describe(failed.error()) == "helios: invalid argument (value must be positive)");
}

TEST_CASE("try_call converts exceptions into errors", "[core][error]") {
    SECTION("value-returning success") {
        const Result<int> value = try_call(ErrorCode::ExternalLibraryFailure, "ok", [] { return 7; });
        REQUIRE(value.has_value());
        CHECK(*value == 7);
    }
    SECTION("void success") {
        bool called = false;
        const VoidResult result = try_call(ErrorCode::ExternalLibraryFailure, "ok", [&] { called = true; });
        CHECK(result.has_value());
        CHECK(called);
    }
    SECTION("std::exception") {
        const Result<int> value = try_call(ErrorCode::ParseFailure, "gltf",
                                           []() -> int { throw std::runtime_error("bad header"); });
        REQUIRE_FALSE(value.has_value());
        CHECK(value.error().code == ErrorCode::ParseFailure);
        CHECK(value.error().context == "gltf: bad header");
    }
    SECTION("non-standard exception") {
        const VoidResult result = try_call(ErrorCode::Unknown, "legacy", [] { throw 42; });
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().context == "legacy: non-standard exception");
    }
}
