#include "helios/core/toml.hpp"

#include "helios/core/parse.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

namespace helios::core {

namespace {

[[nodiscard]] bool is_bare_key_character(char character) noexcept {
    return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z')
           || (character >= '0' && character <= '9') || character == '_' || character == '-';
}

[[nodiscard]] std::string_view kind_name(TomlValue::Kind kind) noexcept {
    switch (kind) {
    case TomlValue::Kind::Boolean: return "a boolean";
    case TomlValue::Kind::Number:  return "a number";
    case TomlValue::Kind::String:  return "a string";
    case TomlValue::Kind::Array:   return "an array";
    case TomlValue::Kind::Table:   return "a table";
    }
    return "a value";
}

[[nodiscard]] bool is_array_of_tables(const TomlValue& value) noexcept {
    return value.kind == TomlValue::Kind::Array && !value.items.empty()
           && std::ranges::all_of(value.items,
                                  [](const TomlValue& item) { return item.kind == TomlValue::Kind::Table; });
}

class Parser {
public:
    explicit Parser(std::string_view text) noexcept : text_(text) {}

    [[nodiscard]] Result<TomlValue> parse() {
        TomlValue root;
        root.line = 1;
        std::reference_wrapper<TomlValue> current = root;
        for (;;) {
            skip_blank();
            if (at_end()) {
                return root;
            }
            if (peek() == '[') {
                auto table = parse_header(root);
                if (!table) {
                    return std::unexpected(table.error());
                }
                current = *table;
            } else if (VoidResult assigned = parse_assignment(current.get()); !assigned) {
                return std::unexpected(assigned.error());
            }
            if (VoidResult ended = end_of_line(); !ended) {
                return std::unexpected(ended.error());
            }
        }
    }

private:
    [[nodiscard]] bool at_end() const noexcept { return position_ >= text_.size(); }
    [[nodiscard]] char peek() const noexcept { return at_end() ? '\0' : text_[position_]; }

    void advance() noexcept {
        if (peek() == '\n') {
            ++line_;
        }
        ++position_;
    }

    [[nodiscard]] std::unexpected<Error> error(std::string_view message) const {
        return fail(ErrorCode::ParseFailure, std::format("line {}: {}", line_, message));
    }

    void skip_spaces() noexcept {
        while (peek() == ' ' || peek() == '\t' || peek() == '\r') {
            advance();
        }
    }

    void skip_comment() noexcept {
        if (peek() == '#') {
            while (!at_end() && peek() != '\n') {
                advance();
            }
        }
    }

    // Whitespace, line ends and comments: between statements and inside arrays.
    void skip_blank() noexcept {
        for (;;) {
            skip_spaces();
            skip_comment();
            if (peek() != '\n') {
                return;
            }
            advance();
        }
    }

    [[nodiscard]] VoidResult end_of_line() {
        skip_spaces();
        skip_comment();
        if (at_end()) {
            return {};
        }
        if (peek() != '\n') {
            return error(std::format("unexpected '{}' after the value", peek()));
        }
        advance();
        return {};
    }

    [[nodiscard]] Result<std::string> parse_key() {
        skip_spaces();
        if (peek() == '"' || peek() == '\'') {
            return parse_string(peek()).transform([](TomlValue value) { return std::move(value.text); });
        }
        const std::size_t start = position_;
        while (is_bare_key_character(peek())) {
            advance();
        }
        if (position_ == start) {
            return error("expected a key");
        }
        return std::string{text_.substr(start, position_ - start)};
    }

    // Follows `path` from the root, creating tables on the way; an array of tables stands for
    // its last element. Returns the table that holds the last key of the path.
    [[nodiscard]] Result<std::reference_wrapper<TomlValue>> descend(TomlValue& root,
                                                                    std::span<const std::string> path) {
        std::reference_wrapper<TomlValue> table = root;
        for (const std::string& key : path) {
            const auto match = std::ranges::find(table.get().keys, key);
            if (match == table.get().keys.end()) {
                table.get().keys.push_back(key);
                TomlValue& created = table.get().items.emplace_back();
                created.line = line_;
                table = created;
                continue;
            }
            TomlValue& existing =
                table.get().items[static_cast<std::size_t>(match - table.get().keys.begin())];
            if (existing.kind == TomlValue::Kind::Table) {
                table = existing;
            } else if (is_array_of_tables(existing)) {
                table = existing.items.back();
            } else {
                return error(std::format("'{}' is already {}", key, kind_name(existing.kind)));
            }
        }
        return table;
    }

    [[nodiscard]] Result<std::reference_wrapper<TomlValue>> parse_header(TomlValue& root) {
        advance(); // '['
        const bool is_array = peek() == '[';
        if (is_array) {
            advance();
        }
        std::vector<std::string> path;
        std::string joined;
        for (;;) {
            auto key = parse_key();
            if (!key) {
                return std::unexpected(key.error());
            }
            joined += std::format("{}{}", path.empty() ? "" : ".", *key);
            path.push_back(std::move(*key));
            skip_spaces();
            if (peek() != '.') {
                break;
            }
            advance();
        }
        if (peek() != ']') {
            return error("expected ']' to close the table header");
        }
        advance();
        if (is_array) {
            if (peek() != ']') {
                return error("expected ']]' to close the array-of-tables header");
            }
            advance();
        }

        const auto parent = descend(root, std::span{path}.first(path.size() - 1));
        if (!parent) {
            return std::unexpected(parent.error());
        }
        TomlValue& holder = parent->get();
        const std::string& name = path.back();
        const auto match = std::ranges::find(holder.keys, name);
        if (match == holder.keys.end()) {
            holder.keys.push_back(name);
            TomlValue& created = holder.items.emplace_back();
            created.line = line_;
            if (!is_array) {
                defined_tables_.push_back(std::move(joined));
                return created;
            }
            created.kind = TomlValue::Kind::Array;
            TomlValue& element = created.items.emplace_back();
            element.line = line_;
            return element;
        }
        TomlValue& existing = holder.items[static_cast<std::size_t>(match - holder.keys.begin())];
        if (is_array) {
            if (!is_array_of_tables(existing)) {
                return error(std::format("'{}' is already {}", name, kind_name(existing.kind)));
            }
            TomlValue& element = existing.items.emplace_back();
            element.line = line_;
            return element;
        }
        // A table created on the way to a deeper header may be opened once afterwards.
        if (existing.kind != TomlValue::Kind::Table
            || std::ranges::find(defined_tables_, joined) != defined_tables_.end()) {
            return error(std::format("table '{}' is defined twice", joined));
        }
        defined_tables_.push_back(std::move(joined));
        return existing;
    }

    // NOLINTBEGIN(misc-no-recursion): recursive descent; parse_value() limits the depth.
    [[nodiscard]] VoidResult parse_assignment(TomlValue& table) {
        auto key = parse_key();
        if (!key) {
            return std::unexpected(key.error());
        }
        skip_spaces();
        if (peek() == '.') {
            return error("dotted keys are not supported; use a [table] header");
        }
        if (peek() != '=') {
            return error(std::format("expected '=' after the key '{}'", *key));
        }
        advance();
        if (table.contains(*key)) {
            return error(std::format("the key '{}' is set twice", *key));
        }
        auto value = parse_value();
        if (!value) {
            return std::unexpected(value.error());
        }
        table.keys.push_back(std::move(*key));
        table.items.push_back(std::move(*value));
        return {};
    }

    [[nodiscard]] Result<TomlValue> parse_value() {
        skip_spaces();
        if (depth_ >= k_max_nesting) {
            return error("arrays and inline tables are nested too deeply");
        }
        ++depth_;
        auto value = parse_value_here();
        --depth_;
        return value;
    }

    [[nodiscard]] Result<TomlValue> parse_value_here() {
        switch (peek()) {
        case '"':
        case '\'': return parse_string(peek());
        case '[':  return parse_array();
        case '{':  return parse_inline_table();
        default:   return parse_scalar();
        }
    }

    [[nodiscard]] Result<TomlValue> parse_string(char quote) {
        TomlValue value;
        value.kind = TomlValue::Kind::String;
        value.line = line_;
        advance(); // the opening quote
        if (peek() == quote && position_ + 1 < text_.size() && text_[position_ + 1] == quote) {
            return error("multi-line strings are not supported");
        }
        for (;;) {
            if (at_end() || peek() == '\n') {
                return error("unterminated string");
            }
            const char character = peek();
            advance();
            if (character == quote) {
                return value;
            }
            if (character != '\\' || quote == '\'') {
                value.text += character;
                continue;
            }
            const char escaped = peek();
            advance();
            switch (escaped) {
            case '"':
            case '\\': value.text += escaped; break;
            case 'n':  value.text += '\n'; break;
            case 't':  value.text += '\t'; break;
            default:   return error(std::format("unsupported escape '\\{}'", escaped));
            }
        }
    }

    [[nodiscard]] Result<TomlValue> parse_array() {
        TomlValue value;
        value.kind = TomlValue::Kind::Array;
        value.line = line_;
        advance(); // '['
        for (;;) {
            skip_blank();
            if (peek() == ']') {
                advance();
                return value;
            }
            if (at_end()) {
                return error("unterminated array");
            }
            auto element = parse_value();
            if (!element) {
                return element;
            }
            value.items.push_back(std::move(*element));
            skip_blank();
            if (peek() == ',') {
                advance();
            } else if (peek() != ']') {
                return error("expected ',' or ']' in the array");
            }
        }
    }

    [[nodiscard]] Result<TomlValue> parse_inline_table() {
        TomlValue value;
        value.line = line_;
        advance(); // '{'
        skip_spaces();
        if (peek() == '}') {
            advance();
            return value;
        }
        for (;;) {
            if (VoidResult assigned = parse_assignment(value); !assigned) {
                return std::unexpected(assigned.error());
            }
            skip_spaces();
            if (peek() == '}') {
                advance();
                return value;
            }
            if (peek() != ',') {
                return error("expected ',' or '}' in the inline table");
            }
            advance();
        }
    }
    // NOLINTEND(misc-no-recursion)

    [[nodiscard]] Result<TomlValue> parse_scalar() {
        const std::size_t start = position_;
        while (!at_end() && std::string_view{" \t\r\n,]}#"}.find(peek()) == std::string_view::npos) {
            advance();
        }
        const std::string_view token = text_.substr(start, position_ - start);
        TomlValue value;
        value.line = line_;
        if (token == "true" || token == "false") {
            value.kind = TomlValue::Kind::Boolean;
            value.boolean = token == "true";
            return value;
        }
        std::string digits;
        for (const char character : token) {
            if (character != '_') {
                digits += character;
            }
        }
        const std::string_view unsigned_digits =
            digits.starts_with('+') ? std::string_view{digits}.substr(1) : std::string_view{digits};
        // Rationale: the decimal form is checked here because what parse_double() accepts beyond
        // it differs between standard libraries (hexadecimal floats, "inf", "nan").
        const std::size_t first_digit = unsigned_digits.find_first_of("0123456789");
        const bool decimal = unsigned_digits.find_first_not_of("0123456789+-.eE") == std::string_view::npos
                             && (first_digit == 0 || (first_digit == 1 && unsigned_digits.front() == '-'));
        const auto number = parse_double(unsigned_digits);
        if (!decimal || !number || !std::isfinite(*number)) {
            return error(std::format("'{}' is not a value this reader supports", token));
        }
        value.kind = TomlValue::Kind::Number;
        value.number = *number;
        return value;
    }

    static constexpr int k_max_nesting = 32;

    int depth_ = 0;
    std::string_view text_;
    std::size_t position_ = 0;
    std::size_t line_ = 1;
    std::vector<std::string> defined_tables_;
};

[[nodiscard]] std::unexpected<Error> wrong_kind(const TomlValue& table, std::string_view key,
                                                TomlValue::Kind expected) {
    return fail(ErrorCode::ParseFailure,
                std::format("'{}' in the table at line {} must be {}", key, table.line, kind_name(expected)));
}

} // namespace

bool TomlValue::contains(std::string_view key) const noexcept {
    return std::ranges::find(keys, key) != keys.end();
}

Result<std::reference_wrapper<const TomlValue>> TomlValue::at(std::string_view key) const {
    const auto match = std::ranges::find(keys, key);
    if (kind != Kind::Table || match == keys.end()) {
        return fail(ErrorCode::ParseFailure, std::format("the table at line {} has no '{}'", line, key));
    }
    return std::cref(items[static_cast<std::size_t>(match - keys.begin())]);
}

Result<double> TomlValue::number_at(std::string_view key) const {
    return at(key).and_then([&](const TomlValue& value) -> Result<double> {
        if (value.kind != Kind::Number) {
            return wrong_kind(*this, key, Kind::Number);
        }
        return value.number;
    });
}

Result<double> TomlValue::number_or(std::string_view key, double fallback) const {
    return contains(key) ? number_at(key) : Result<double>{fallback};
}

Result<std::string> TomlValue::string_at(std::string_view key) const {
    return at(key).and_then([&](const TomlValue& value) -> Result<std::string> {
        if (value.kind != Kind::String) {
            return wrong_kind(*this, key, Kind::String);
        }
        return value.text;
    });
}

Result<std::string> TomlValue::string_or(std::string_view key, std::string fallback) const {
    return contains(key) ? string_at(key) : Result<std::string>{std::move(fallback)};
}

Result<bool> TomlValue::boolean_or(std::string_view key, bool fallback) const {
    if (!contains(key)) {
        return fallback;
    }
    return at(key).and_then([&](const TomlValue& value) -> Result<bool> {
        if (value.kind != Kind::Boolean) {
            return wrong_kind(*this, key, Kind::Boolean);
        }
        return value.boolean;
    });
}

Result<std::span<const TomlValue>> TomlValue::array_or_empty(std::string_view key) const {
    if (!contains(key)) {
        return std::span<const TomlValue>{};
    }
    return at(key).and_then([&](const TomlValue& value) -> Result<std::span<const TomlValue>> {
        if (value.kind != Kind::Array) {
            return wrong_kind(*this, key, Kind::Array);
        }
        return std::span<const TomlValue>{value.items};
    });
}

Result<std::vector<double>> TomlValue::numbers_or_empty(std::string_view key) const {
    return array_or_empty(key).and_then(
        [&](std::span<const TomlValue> elements) -> Result<std::vector<double>> {
            std::vector<double> numbers;
            numbers.reserve(elements.size());
            for (const TomlValue& element : elements) {
                if (element.kind != Kind::Number) {
                    return fail(ErrorCode::ParseFailure,
                                std::format("'{}' at line {} must hold only numbers", key, element.line));
                }
                numbers.push_back(element.number);
            }
            return numbers;
        });
}

Result<std::vector<std::string>> TomlValue::strings_or_empty(std::string_view key) const {
    return array_or_empty(key).and_then(
        [&](std::span<const TomlValue> elements) -> Result<std::vector<std::string>> {
            std::vector<std::string> strings;
            strings.reserve(elements.size());
            for (const TomlValue& element : elements) {
                if (element.kind != Kind::String) {
                    return fail(ErrorCode::ParseFailure,
                                std::format("'{}' at line {} must hold only strings", key, element.line));
                }
                strings.push_back(element.text);
            }
            return strings;
        });
}

VoidResult TomlValue::expect_keys(std::initializer_list<std::string_view> allowed) const {
    for (const std::string& key : keys) {
        if (std::ranges::find(allowed, key) == allowed.end()) {
            return fail(ErrorCode::ParseFailure,
                        std::format("unknown key '{}' in the table at line {}", key, line));
        }
    }
    return {};
}

Result<TomlValue> parse_toml(std::string_view text) {
    return Parser(text).parse();
}

} // namespace helios::core
