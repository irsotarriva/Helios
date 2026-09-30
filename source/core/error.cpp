#include "helios/core/error.hpp"

namespace helios::core {

namespace {

class HeliosErrorCategory final : public std::error_category {
public:
    [[nodiscard]] const char* name() const noexcept override { return "helios"; }

    [[nodiscard]] std::string message(int value) const override {
        switch (static_cast<ErrorCode>(value)) {
        case ErrorCode::InvalidArgument:        return "invalid argument";
        case ErrorCode::NotFinite:              return "value is not finite";
        case ErrorCode::OutOfRange:             return "value out of range";
        case ErrorCode::Overflow:               return "arithmetic overflow";
        case ErrorCode::FileNotFound:           return "file not found";
        case ErrorCode::IoFailure:              return "I/O failure";
        case ErrorCode::ParseFailure:           return "parse failure";
        case ErrorCode::ExternalLibraryFailure: return "external library failure";
        case ErrorCode::Timeout:                return "operation timed out";
        case ErrorCode::Unknown:                return "unknown error";
        }
        return std::format("unrecognised helios error {}", value);
    }
};

} // namespace

const std::error_category& helios_category() noexcept {
    static const HeliosErrorCategory category;
    return category;
}

std::error_code make_error_code(ErrorCode code) noexcept {
    return {static_cast<int>(code), helios_category()};
}

Error make_error(std::error_code code, std::string context) {
    return Error{.code = code, .context = std::move(context)};
}

std::unexpected<Error> fail(ErrorCode code, std::string context) {
    return std::unexpected(make_error(make_error_code(code), std::move(context)));
}

std::string describe(const Error& error) {
    if (error.context.empty()) {
        return std::format("{}: {}", error.code.category().name(), error.code.message());
    }
    return std::format("{}: {} ({})", error.code.category().name(), error.code.message(), error.context);
}

} // namespace helios::core
