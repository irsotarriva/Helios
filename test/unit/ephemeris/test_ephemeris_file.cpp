#include "helios/ephemeris/ephemeris_file.hpp"

#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

using helios::core::ErrorCode;
using helios::ephemeris::parse_chebyshev_file;
using helios::ephemeris::parse_secular_elements_csv;
using helios::time::Epoch;

// Builds a little-endian .hce image in memory (tests run on little-endian hosts).
class FileBuilder {
public:
    FileBuilder() { append_bytes(std::as_bytes(std::span{std::string_view{"HLSCHEB1"}})); }

    template <typename Scalar>
    FileBuilder& append(Scalar value) {
        append_bytes(std::as_bytes(std::span{&value, 1}));
        return *this;
    }
    FileBuilder& append_name(const std::string& name) {
        append(static_cast<std::uint32_t>(name.size()));
        append_bytes(std::as_bytes(std::span{name}));
        return *this;
    }
    FileBuilder& append_segment(const std::string& target, const std::string& center,
                                std::uint32_t record_count) {
        append_name(target).append_name(center);
        append(std::int64_t{-100})
            .append(0.25)
            .append(86'400.0)
            .append(record_count)
            .append(std::uint32_t{2});
        for (std::uint32_t value = 0; value < record_count * 6; ++value) {
            append(static_cast<double>(value));
        }
        return *this;
    }
    [[nodiscard]] std::span<const std::byte> bytes() const { return bytes_; }
    void truncate(std::size_t byte_count) { bytes_.resize(bytes_.size() - byte_count); }

private:
    void append_bytes(std::span<const std::byte> data) {
        bytes_.insert(bytes_.end(), data.begin(), data.end());
    }
    std::vector<std::byte> bytes_;
};

TEST(ChebyshevFile, ParsesSegmentsInOrder) {
    FileBuilder builder;
    builder.append(std::uint32_t{2})
        .append_segment("Earth-Moon barycentre", "Solar System barycentre", 3)
        .append_segment("Moon", "Earth-Moon barycentre", 1);

    const auto segments = parse_chebyshev_file(builder.bytes());
    ASSERT_TRUE(segments.has_value()) << helios::core::describe(segments.error());
    ASSERT_EQ(segments->size(), 2U);
    EXPECT_EQ((*segments)[0].target_name, "Earth-Moon barycentre");
    EXPECT_EQ((*segments)[0].center_name, "Solar System barycentre");
    EXPECT_EQ((*segments)[0].ephemeris.record_count(), 3U);
    EXPECT_EQ((*segments)[1].ephemeris.coefficients_per_axis(), 2U);
    const auto coverage = (*segments)[0].ephemeris.valid_range();
    ASSERT_TRUE(coverage.has_value());
    EXPECT_EQ(coverage->begin(), Epoch::from_parts(-100, 0.25).value());
}

TEST(ChebyshevFile, RejectsBadMagic) {
    std::vector<std::byte> bytes(16, std::byte{0});
    EXPECT_EQ(parse_chebyshev_file(bytes).error().code, ErrorCode::ParseFailure);
}

TEST(ChebyshevFile, RejectsTruncatedData) {
    FileBuilder builder;
    builder.append(std::uint32_t{1}).append_segment("Moon", "Earth", 2);
    builder.truncate(8);
    EXPECT_EQ(parse_chebyshev_file(builder.bytes()).error().code, ErrorCode::ParseFailure);
}

TEST(ChebyshevFile, RejectsRecordCountLargerThanTheFile) {
    FileBuilder builder;
    builder.append(std::uint32_t{1}).append_name("Moon").append_name("Earth");
    builder.append(std::int64_t{0})
        .append(0.0)
        .append(86'400.0)
        .append(std::uint32_t{4'000'000'000U})
        .append(std::uint32_t{13});
    EXPECT_EQ(parse_chebyshev_file(builder.bytes()).error().code, ErrorCode::ParseFailure);
}

TEST(ChebyshevFile, RejectsTrailingBytes) {
    FileBuilder builder;
    builder.append(std::uint32_t{1}).append_segment("Moon", "Earth", 1).append(std::uint8_t{7});
    EXPECT_EQ(parse_chebyshev_file(builder.bytes()).error().code, ErrorCode::ParseFailure);
}

constexpr std::string_view k_header = "target,center,reference_epoch_tdb_s,a_m,a_rate,e,e_rate,i,i_rate,node,"
                                      "node_rate,peri,peri_rate,L,L_rate\n";

TEST(SecularElementsCsv, ParsesRowsAndSkipsComments) {
    const std::string csv =
        std::string{"# fitted to DE421\n"} + std::string{k_header}
        + "Mars, Sun, 0, 2.2794e11, 0, 0.0934, 0, 0.0323, 0, 0.865, 0, 5.866, 0, 6.204, 1.0586e-7\n";
    const auto bodies = parse_secular_elements_csv(csv);
    ASSERT_TRUE(bodies.has_value()) << helios::core::describe(bodies.error());
    ASSERT_EQ(bodies->size(), 1U);
    EXPECT_EQ((*bodies)[0].target_name, "Mars");
    EXPECT_EQ((*bodies)[0].center_name, "Sun");
    EXPECT_DOUBLE_EQ((*bodies)[0].ephemeris.elements().eccentricity, 0.0934);
}

TEST(SecularElementsCsv, RejectsWrongColumnCountAndBadNumbers) {
    EXPECT_EQ(parse_secular_elements_csv(std::string{k_header} + "Mars,Sun,0,1\n").error().code,
              ErrorCode::ParseFailure);
    EXPECT_EQ(
        parse_secular_elements_csv(std::string{k_header} + "Mars,Sun,0,abc,0,0.1,0,0,0,0,0,0,0,0,1e-7\n")
            .error()
            .code,
        ErrorCode::ParseFailure);
    EXPECT_EQ(
        parse_secular_elements_csv(std::string{k_header} + "Mars,Sun,0,1e11,0,1.5,0,0,0,0,0,0,0,0,1e-7\n")
            .error()
            .code,
        ErrorCode::ParseFailure);
    EXPECT_EQ(parse_secular_elements_csv("").error().code, ErrorCode::ParseFailure);
}

} // namespace
