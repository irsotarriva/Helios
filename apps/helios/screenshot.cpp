#include "screenshot.hpp"

#include "helios/core/logging.hpp"

#include <array>
#include <bimg/bimg.h>
#include <bx/file.h>
#include <cstdio>
#include <cstdlib>

namespace helios::app {

void BgfxCallback::fatal(const char* file_path, std::uint16_t line, bgfx::Fatal::Enum code,
                         const char* message) {
    if (code == bgfx::Fatal::DebugCheck) {
        LOG_WARN("bgfx debug check at {}:{}: {}", file_path, line, message).tag("subsystem", "render");
        return;
    }
    LOG_ERROR("bgfx fatal error {} at {}:{}: {}", static_cast<int>(code), file_path, line, message)
        .tag("subsystem", "render");
    // Rationale: bgfx cannot continue after a fatal error (its documentation requires the
    // callback not to return), and there is no caller to hand a Result to.
    std::abort();
}

void BgfxCallback::traceVargs(const char* /*file_path*/, std::uint16_t /*line*/, const char* format,
                              va_list arguments) {
    std::array<char, 1024> buffer{};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): bgfx's C-style trace interface.
    std::vsnprintf(buffer.data(), buffer.size(), format, arguments);
    LOG_DEBUG("bgfx: {}", buffer.data()).tag("subsystem", "render");
}

void BgfxCallback::screenShot(const char* file_path, std::uint32_t width, std::uint32_t height,
                              std::uint32_t pitch, bgfx::TextureFormat::Enum format, const void* data,
                              std::uint32_t /*size*/, bool y_flip) {
    bx::FileWriter writer;
    bx::Error error;
    if (!bx::open(&writer, file_path, false, &error)) {
        LOG_ERROR("cannot open screenshot file '{}'", file_path).tag("subsystem", "render");
        return;
    }
    bimg::imageWritePng(&writer, width, height, pitch, data, static_cast<bimg::TextureFormat::Enum>(format),
                        y_flip, &error);
    bx::close(&writer);
    if (!error.isOk()) {
        LOG_ERROR("cannot write screenshot '{}'", file_path).tag("subsystem", "render");
        return;
    }
    LOG_INFO("screenshot written to {} ({}×{})", file_path, width, height).tag("subsystem", "render");
    screenshot_written_ = true;
}

} // namespace helios::app
