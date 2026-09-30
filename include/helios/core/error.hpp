#ifndef HELIOS_CORE_ERROR_HPP
#define HELIOS_CORE_ERROR_HPP

#include <concepts>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <functional>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>

namespace helios::core {

// Values start at 1: std::error_code treats 0 as "success".
enum class ErrorCode : std::int32_t {
    InvalidArgument = 1,
    NotFinite,
    OutOfRange,
    Overflow,
    FileNotFound,
    IoFailure,
    ParseFailure,
    ExternalLibraryFailure,
    Timeout,
    Unknown,
};

// The std::error_category for ErrorCode ("helios").
[[nodiscard]] const std::error_category& helios_category() noexcept;

// Found by ADL; lets ErrorCode convert implicitly to std::error_code.
[[nodiscard]] std::error_code make_error_code(ErrorCode code) noexcept;

// Every failure in Helios. `code` may belong to any category, so errors from
// external libraries that already speak std::error_code pass through unchanged.
struct Error {
    std::error_code code;
    std::string context;
};

template <typename T>
using Result = std::expected<T, Error>;

using VoidResult = std::expected<void, Error>;

[[nodiscard]] Error make_error(std::error_code code, std::string context);

// Shorthand for `return std::unexpected(make_error(...))`.
[[nodiscard]] std::unexpected<Error> fail(ErrorCode code, std::string context);

// Human-readable "category: message (context)".
[[nodiscard]] std::string describe(const Error& error);

template <typename Callable>
concept ValueReturningCallable =
    std::invocable<Callable> && !std::is_reference_v<std::invoke_result_t<Callable>>;

// Boundary adapter for third-party calls that may throw (CODING_STANDARDS §1.2).
// Any exception becomes an Error carrying `code_on_exception`; nothing propagates.
template <ValueReturningCallable Callable>
[[nodiscard]] Result<std::invoke_result_t<Callable>>
try_call(ErrorCode code_on_exception, std::string_view context, Callable&& callable) noexcept {
    using ValueType = std::invoke_result_t<Callable>;
    // Rationale: noexcept is deliberate. Only a failed allocation while building the
    // error message can escape the handlers below, and running out of memory there is
    // not recoverable anyway.
    try {
        if constexpr (std::is_void_v<ValueType>) {
            std::invoke(std::forward<Callable>(callable));
            return {};
        } else {
            return std::invoke(std::forward<Callable>(callable));
        }
    } catch (const std::exception& exception) {
        return std::unexpected(
            make_error(make_error_code(code_on_exception), std::format("{}: {}", context, exception.what())));
    } catch (...) {
        return std::unexpected(make_error(make_error_code(code_on_exception),
                                          std::format("{}: non-standard exception", context)));
    }
}

} // namespace helios::core

template <>
struct std::is_error_code_enum<helios::core::ErrorCode> : std::true_type {};

#endif // HELIOS_CORE_ERROR_HPP
