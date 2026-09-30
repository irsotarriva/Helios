#include "helios/core/parse.hpp"

#include <gtest/gtest.h>

namespace {

using helios::core::ErrorCode;
using helios::core::parse_double;
using helios::core::parse_int64;
using helios::core::trim;

TEST(Parse, DoublesAreLocaleIndependentAndExact) {
    EXPECT_EQ(parse_double(" 1.5e-3 ").value(), 1.5e-3);
    EXPECT_EQ(parse_double("-2.2794e11").value(), -2.2794e11);
    EXPECT_EQ(parse_double("0.10000000000000001").value(), 0.1);
}

TEST(Parse, RejectsPartialOrEmptyFields) {
    EXPECT_EQ(parse_double("1.5x").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(parse_double("").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(parse_double("1,5").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(parse_int64("12.0").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(parse_int64("").error().code, ErrorCode::ParseFailure);
}

TEST(Parse, IntegersAndTrim) {
    EXPECT_EQ(parse_int64(" -3169195200\r").value(), -3'169'195'200);
    EXPECT_EQ(trim("\t a b \r"), "a b");
    EXPECT_EQ(trim("   "), "");
}

} // namespace
