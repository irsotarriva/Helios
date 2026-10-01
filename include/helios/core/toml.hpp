#ifndef HELIOS_CORE_TOML_HPP
#define HELIOS_CORE_TOML_HPP

#include "helios/core/error.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::core {

// One value of a parsed TOML document; the document itself is a Table.
//
// The typed lookups are for reading data definitions: each failure names the key and the line
// of the table it was looked up in, so a modder can find the mistake.
struct TomlValue {
    enum class Kind : std::uint8_t { Boolean, Number, String, Array, Table };

    Kind kind = Kind::Table;
    std::size_t line = 0; // where the value (or the table's header) starts, 1-based
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<std::string> keys; // Table: the keys, parallel to `items`
    std::vector<TomlValue> items;  // Array: the elements. Table: the values

    [[nodiscard]] bool contains(std::string_view key) const noexcept;
    [[nodiscard]] Result<std::reference_wrapper<const TomlValue>> at(std::string_view key) const;

    [[nodiscard]] Result<double> number_at(std::string_view key) const;
    [[nodiscard]] Result<double> number_or(std::string_view key, double fallback) const;
    [[nodiscard]] Result<std::string> string_at(std::string_view key) const;
    [[nodiscard]] Result<std::string> string_or(std::string_view key, std::string fallback) const;
    [[nodiscard]] Result<bool> boolean_or(std::string_view key, bool fallback) const;
    // The elements of an array member; empty when the key is absent.
    [[nodiscard]] Result<std::span<const TomlValue>> array_or_empty(std::string_view key) const;
    // An array of numbers; empty when the key is absent.
    [[nodiscard]] Result<std::vector<double>> numbers_or_empty(std::string_view key) const;
    // An array of strings; empty when the key is absent.
    [[nodiscard]] Result<std::vector<std::string>> strings_or_empty(std::string_view key) const;

    // Fails if the table has a key outside `allowed`: catches misspelt keys, which would
    // otherwise silently fall back to their defaults.
    [[nodiscard]] VoidResult expect_keys(std::initializer_list<std::string_view> allowed) const;
};

// Parses the subset of TOML 1.0 that Helios data files use: comments, bare and quoted keys,
// `[table]` and `[[array of tables]]` headers with dotted paths, basic ("…", with \" \\ \n \t
// escapes) and literal ('…') strings, integers and floats (with `_` separators), booleans,
// arrays (possibly spanning lines) and inline tables. Dates, multi-line strings and dotted keys
// in assignments are rejected with an error rather than misread, so every accepted file is also
// valid TOML for a complete parser.
[[nodiscard]] Result<TomlValue> parse_toml(std::string_view text);

} // namespace helios::core

#endif // HELIOS_CORE_TOML_HPP
