#include "helios/core/parse.hpp"

#include <charconv>
#include <format>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>

namespace helios::core {

std::string_view trim(std::string_view text) noexcept {
    constexpr std::string_view k_whitespace = " \t\r";
    const std::size_t first = text.find_first_not_of(k_whitespace);
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(k_whitespace);
    return text.substr(first, last - first + 1);
}

std::vector<std::string_view> split_csv_line(std::string_view line) {
    std::vector<std::string_view> fields;
    while (true) {
        const std::size_t comma = line.find(',');
        fields.push_back(trim(line.substr(0, comma)));
        if (comma == std::string_view::npos) {
            return fields;
        }
        line = line.substr(comma + 1);
    }
}

Result<double> parse_double(std::string_view text) noexcept {
    const std::string_view field = trim(text);
    if (field.empty()) {
        return fail(ErrorCode::ParseFailure, "empty numeric field");
    }
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
    double value = 0.0;
    const auto [end, error] = std::from_chars(field.data(), field.data() + field.size(), value);
    if (error != std::errc{} || end != field.data() + field.size()) {
        return fail(ErrorCode::ParseFailure, std::format("'{}' is not a number", field));
    }
    return value;
#else
    // Rationale: some standard libraries (older Apple libc++) lack floating-point from_chars.
    // A classic-locale stream is the portable, locale-independent fallback.
    return try_call(ErrorCode::ParseFailure, "parsing a number",
                    [&]() -> Result<double> {
                        std::istringstream stream{std::string{field}};
                        stream.imbue(std::locale::classic());
                        double value = 0.0;
                        stream >> value;
                        if (stream.fail() || stream.peek() != std::char_traits<char>::eof()) {
                            return fail(ErrorCode::ParseFailure, std::format("'{}' is not a number", field));
                        }
                        return value;
                    })
        .and_then([](Result<double> value) { return value; });
#endif
}

Result<std::int64_t> parse_int64(std::string_view text) noexcept {
    const std::string_view field = trim(text);
    std::int64_t value = 0;
    const auto [end, error] = std::from_chars(field.data(), field.data() + field.size(), value);
    if (field.empty() || error != std::errc{} || end != field.data() + field.size()) {
        return fail(ErrorCode::ParseFailure, std::format("'{}' is not an integer", field));
    }
    return value;
}

} // namespace helios::core
