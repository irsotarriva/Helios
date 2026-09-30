#include "helios/core/error.hpp"

#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

namespace {

using helios::core::describe;
using helios::core::ErrorCode;
using helios::core::fail;
using helios::core::Result;
using helios::core::try_call;
using helios::core::VoidResult;

[[nodiscard]] Result<int> parse_positive(int value) {
    if (value <= 0) {
        return fail(ErrorCode::InvalidArgument, "value must be positive");
    }
    return value;
}

TEST(ErrorCode, ConvertsToErrorCodeInTheHeliosCategory) {
    const std::error_code code = ErrorCode::Overflow;
    EXPECT_STREQ(code.category().name(), "helios");
    EXPECT_EQ(code.message(), "arithmetic overflow");
    EXPECT_EQ(code, ErrorCode::Overflow);
    EXPECT_TRUE(static_cast<bool>(code));
}

TEST(Result, SuccessPropagatesThroughMonadicChain) {
    const Result<int> doubled = parse_positive(21).transform([](int value) { return value * 2; });
    ASSERT_TRUE(doubled.has_value());
    EXPECT_EQ(*doubled, 42);
}

TEST(Result, FailurePropagatesThroughMonadicChain) {
    const Result<int> failed = parse_positive(-1).transform([](int value) { return value * 2; });
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(describe(failed.error()), "helios: invalid argument (value must be positive)");
}

TEST(TryCall, ReturnsValueOnSuccess) {
    const Result<int> value = try_call(ErrorCode::ExternalLibraryFailure, "ok", [] { return 7; });
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(*value, 7);
}

TEST(TryCall, SupportsVoidCallables) {
    bool called = false;
    const VoidResult result = try_call(ErrorCode::ExternalLibraryFailure, "ok", [&] { called = true; });
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(called);
}

TEST(TryCall, ConvertsStdExceptionToError) {
    const Result<int> value =
        try_call(ErrorCode::ParseFailure, "gltf", []() -> int { throw std::runtime_error("bad header"); });
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(value.error().context, "gltf: bad header");
}

TEST(TryCall, ConvertsNonStandardExceptionToError) {
    const VoidResult result = try_call(ErrorCode::Unknown, "legacy", [] { throw 42; });
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().context, "legacy: non-standard exception");
}

} // namespace
