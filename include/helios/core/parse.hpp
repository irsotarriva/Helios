#ifndef HELIOS_CORE_PARSE_HPP
#define HELIOS_CORE_PARSE_HPP

#include "helios/core/error.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace helios::core {

// Whole file as a string (binary-safe). Fails if missing, unreadable or larger than 1 GiB.
[[nodiscard]] Result<std::string> read_text_file(const std::filesystem::path& path) noexcept;

// Locale-independent parsing of a whole field (surrounding whitespace allowed, nothing else).
[[nodiscard]] Result<double> parse_double(std::string_view text) noexcept;
[[nodiscard]] Result<std::int64_t> parse_int64(std::string_view text) noexcept;

// Comma-separated fields of one line, each trimmed. No quoting (Helios data files never need it).
[[nodiscard]] std::vector<std::string_view> split_csv_line(std::string_view line);

// `text` without leading/trailing spaces, tabs and carriage returns.
[[nodiscard]] std::string_view trim(std::string_view text) noexcept;

} // namespace helios::core

#endif // HELIOS_CORE_PARSE_HPP
