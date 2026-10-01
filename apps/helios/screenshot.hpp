#ifndef HELIOS_APPS_HELIOS_SCREENSHOT_HPP
#define HELIOS_APPS_HELIOS_SCREENSHOT_HPP

#include <atomic>
#include <bgfx/bgfx.h>
#include <cstdarg>
#include <cstdint>

namespace helios::app {

// bgfx callbacks: screenshots are written as PNG to the requested path, everything else is
// logged or ignored.
class BgfxCallback final : public bgfx::CallbackI {
public:
    [[nodiscard]] bool screenshot_written() const noexcept { return screenshot_written_; }

    void fatal(const char* file_path, std::uint16_t line, bgfx::Fatal::Enum code,
               const char* message) override;
    void traceVargs(const char* file_path, std::uint16_t line, const char* format,
                    va_list arguments) override;
    void profilerBegin(const char* /*name*/, std::uint32_t /*abgr*/, const char* /*file*/,
                       std::uint16_t /*line*/) override {}
    void profilerBeginLiteral(const char* /*name*/, std::uint32_t /*abgr*/, const char* /*file*/,
                              std::uint16_t /*line*/) override {}
    void profilerEnd() override {}
    std::uint32_t cacheReadSize(std::uint64_t /*id*/) override { return 0; }
    bool cacheRead(std::uint64_t /*id*/, void* /*data*/, std::uint32_t /*size*/) override { return false; }
    void cacheWrite(std::uint64_t /*id*/, const void* /*data*/, std::uint32_t /*size*/) override {}
    void screenShot(const char* file_path, std::uint32_t width, std::uint32_t height, std::uint32_t pitch,
                    bgfx::TextureFormat::Enum format, const void* data, std::uint32_t size,
                    bool y_flip) override;
    void captureBegin(std::uint32_t /*width*/, std::uint32_t /*height*/, std::uint32_t /*pitch*/,
                      bgfx::TextureFormat::Enum /*format*/, bool /*y_flip*/) override {}
    void captureEnd() override {}
    void captureFrame(const void* /*data*/, std::uint32_t /*size*/) override {}

private:
    std::atomic<bool> screenshot_written_{false};
};

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_SCREENSHOT_HPP
