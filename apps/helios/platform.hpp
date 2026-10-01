#ifndef HELIOS_APPS_HELIOS_PLATFORM_HPP
#define HELIOS_APPS_HELIOS_PLATFORM_HPP

#include "helios/core/error.hpp"

#include <bgfx/bgfx.h>

struct SDL_Window;

namespace helios::app {

struct NativeWindow {
    void* window_handle = nullptr;  // NSWindow*, HWND or an X11 Window id
    void* display_handle = nullptr; // X11 Display*; null elsewhere
    bgfx::NativeWindowHandleType::Enum type = bgfx::NativeWindowHandleType::Default;
};

// The native handles bgfx needs to create its swap chain on an SDL window.
[[nodiscard]] core::Result<NativeWindow> native_window(SDL_Window* window);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_PLATFORM_HPP
