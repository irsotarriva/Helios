#include "helios/ephemeris/ephemeris_file.hpp"

#include "helios/core/parse.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <format>
#include <string_view>
#include <utility>

namespace helios::ephemeris {

namespace {

using core::ErrorCode;

constexpr std::string_view k_magic = "HLSCHEB1";
constexpr std::uint32_t k_max_name_length = 256;
constexpr std::uint32_t k_max_coefficients_per_axis = 64;

// Bounds-checked little-endian reader over the file bytes.
class ByteReader {
public:
    explicit ByteReader(std::span<const std::byte> bytes) noexcept : bytes_(bytes) {}

    [[nodiscard]] std::size_t remaining() const noexcept { return bytes_.size() - offset_; }

    template <typename Scalar>
        requires std::is_arithmetic_v<Scalar>
    [[nodiscard]] core::Result<Scalar> read() noexcept {
        if (remaining() < sizeof(Scalar)) {
            return core::fail(ErrorCode::ParseFailure, "ephemeris file is truncated");
        }
        Scalar value{};
        std::memcpy(&value, bytes_.subspan(offset_, sizeof(Scalar)).data(), sizeof(Scalar));
        offset_ += sizeof(Scalar);
        if constexpr (std::endian::native == std::endian::big && sizeof(Scalar) > 1) {
            // The format is little-endian on disk; swap on big-endian hosts.
            using Bits = std::conditional_t<sizeof(Scalar) == 8, std::uint64_t, std::uint32_t>;
            value = std::bit_cast<Scalar>(std::byteswap(std::bit_cast<Bits>(value)));
        }
        return value;
    }

    [[nodiscard]] core::Result<std::string> read_string(std::uint32_t length) noexcept {
        if (length > k_max_name_length || remaining() < length) {
            return core::fail(ErrorCode::ParseFailure, "ephemeris file has an invalid name length");
        }
        const std::span<const std::byte> raw = bytes_.subspan(offset_, length);
        offset_ += length;
        std::string text(length, '\0');
        std::memcpy(text.data(), raw.data(), length);
        return text;
    }

private:
    std::span<const std::byte> bytes_;
    std::size_t offset_ = 0;
};

[[nodiscard]] core::Result<std::string> read_name(ByteReader& reader) noexcept {
    return reader.read<std::uint32_t>().and_then(
        [&](std::uint32_t length) { return reader.read_string(length); });
}

[[nodiscard]] core::Result<NamedChebyshevSegment> read_segment(ByteReader& reader) noexcept {
    auto target_name = read_name(reader);
    auto center_name = target_name.and_then([&](const std::string&) { return read_name(reader); });
    if (!center_name) {
        return std::unexpected(center_name.error());
    }
    const auto start_whole_seconds = reader.read<std::int64_t>();
    const auto start_fraction_s = reader.read<double>();
    const auto record_duration_s = reader.read<double>();
    const auto record_count = reader.read<std::uint32_t>();
    const auto coefficients_per_axis = reader.read<std::uint32_t>();
    if (!start_whole_seconds || !start_fraction_s || !record_duration_s || !record_count
        || !coefficients_per_axis) {
        return core::fail(ErrorCode::ParseFailure, "ephemeris segment header is truncated");
    }
    if (*coefficients_per_axis == 0 || *coefficients_per_axis > k_max_coefficients_per_axis
        || *record_count == 0) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("segment '{}' has an invalid record layout", *target_name));
    }
    // Check the size against the bytes actually present before allocating anything.
    const std::uint64_t value_count =
        std::uint64_t{*record_count} * 3U * std::uint64_t{*coefficients_per_axis};
    if (value_count > reader.remaining() / sizeof(double)) {
        return core::fail(ErrorCode::ParseFailure, std::format("segment '{}' is truncated", *target_name));
    }
    std::vector<double> coefficients_m;
    coefficients_m.reserve(static_cast<std::size_t>(value_count));
    for (std::uint64_t index = 0; index < value_count; ++index) {
        const auto coefficient_m = reader.read<double>();
        if (!coefficient_m) {
            return std::unexpected(coefficient_m.error());
        }
        coefficients_m.push_back(*coefficient_m);
    }

    return time::Epoch::from_parts(*start_whole_seconds, *start_fraction_s)
        .and_then([&](const time::Epoch& start) {
            return ChebyshevEphemeris::make(start, *record_duration_s, *coefficients_per_axis,
                                            std::move(coefficients_m));
        })
        .transform([&](ChebyshevEphemeris ephemeris) {
            return NamedChebyshevSegment{.target_name = std::move(*target_name),
                                         .center_name = std::move(*center_name),
                                         .ephemeris = std::move(ephemeris)};
        });
}

[[nodiscard]] core::Result<double> parse_field(std::string_view field, std::size_t line_number) noexcept {
    return core::parse_double(field).transform_error([&](core::Error error) {
        error.context = std::format("line {}: {}", line_number, error.context);
        return error;
    });
}

constexpr std::size_t k_secular_column_count = 15;

} // namespace

core::Result<std::vector<NamedChebyshevSegment>>
parse_chebyshev_file(std::span<const std::byte> file_bytes) noexcept {
    if (file_bytes.size() < k_magic.size()
        || std::memcmp(file_bytes.data(), k_magic.data(), k_magic.size()) != 0) {
        return core::fail(ErrorCode::ParseFailure, "not a Helios Chebyshev ephemeris file (bad magic)");
    }
    ByteReader reader(file_bytes.subspan(k_magic.size()));
    const auto segment_count = reader.read<std::uint32_t>();
    if (!segment_count) {
        return std::unexpected(segment_count.error());
    }

    std::vector<NamedChebyshevSegment> segments;
    for (std::uint32_t index = 0; index < *segment_count; ++index) {
        auto segment = read_segment(reader);
        if (!segment) {
            return std::unexpected(segment.error());
        }
        segments.push_back(std::move(*segment));
    }
    if (reader.remaining() != 0) {
        return core::fail(ErrorCode::ParseFailure, "ephemeris file has trailing bytes");
    }
    return segments;
}

core::Result<std::vector<NamedChebyshevSegment>>
load_chebyshev_file(const std::filesystem::path& path) noexcept {
    return core::read_text_file(path).and_then([](const std::string& contents) {
        return parse_chebyshev_file(std::as_bytes(std::span{contents.data(), contents.size()}));
    });
}

core::Result<std::vector<NamedSecularElements>>
parse_secular_elements_csv(std::string_view csv_text) noexcept {
    std::vector<NamedSecularElements> bodies;
    bool header_seen = false;
    std::size_t line_number = 0;

    while (!csv_text.empty()) {
        const std::size_t line_end = csv_text.find('\n');
        const std::string_view line = core::trim(csv_text.substr(0, line_end));
        csv_text = line_end == std::string_view::npos ? std::string_view{} : csv_text.substr(line_end + 1);
        ++line_number;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        if (!header_seen) {
            header_seen = true;
            continue;
        }

        std::array<std::string_view, k_secular_column_count> fields{};
        std::size_t field_count = 0;
        std::string_view rest = line;
        while (field_count < k_secular_column_count) {
            const std::size_t comma = rest.find(',');
            fields.at(field_count++) = core::trim(rest.substr(0, comma));
            if (comma == std::string_view::npos) {
                rest = {};
                break;
            }
            rest = rest.substr(comma + 1);
        }
        if (field_count != k_secular_column_count || !rest.empty()) {
            return core::fail(ErrorCode::ParseFailure, std::format("line {}: expected {} columns",
                                                                   line_number, k_secular_column_count));
        }

        std::array<double, k_secular_column_count - 2> numbers{};
        for (std::size_t column = 2; column < k_secular_column_count; ++column) {
            const auto number = parse_field(fields.at(column), line_number);
            if (!number) {
                return std::unexpected(number.error());
            }
            numbers.at(column - 2) = *number;
        }

        const double reference_epoch_s = numbers[0];
        auto body =
            time::Epoch::from_seconds(reference_epoch_s).and_then([&](const time::Epoch& reference_epoch) {
                return KeplerianEphemeris::make(SecularElements{
                    .reference_epoch = reference_epoch,
                    .semi_major_axis_m = numbers[1],
                    .semi_major_axis_rate_m_s = numbers[2],
                    .eccentricity = numbers[3],
                    .eccentricity_rate_per_s = numbers[4],
                    .inclination_rad = numbers[5],
                    .inclination_rate_rad_s = numbers[6],
                    .longitude_of_ascending_node_rad = numbers[7],
                    .longitude_of_ascending_node_rate_rad_s = numbers[8],
                    .longitude_of_periapsis_rad = numbers[9],
                    .longitude_of_periapsis_rate_rad_s = numbers[10],
                    .mean_longitude_rad = numbers[11],
                    .mean_longitude_rate_rad_s = numbers[12],
                });
            });
        if (!body) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: {}", line_number, core::describe(body.error())));
        }
        bodies.push_back(NamedSecularElements{.target_name = std::string{fields[0]},
                                              .center_name = std::string{fields[1]},
                                              .ephemeris = *body});
    }
    if (!header_seen) {
        return core::fail(ErrorCode::ParseFailure, "secular elements CSV has no header row");
    }
    return bodies;
}

core::Result<std::vector<NamedSecularElements>>
load_secular_elements_csv(const std::filesystem::path& path) noexcept {
    return core::read_text_file(path).and_then(
        [](const std::string& contents) { return parse_secular_elements_csv(contents); });
}

} // namespace helios::ephemeris
