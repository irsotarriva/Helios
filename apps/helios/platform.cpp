#include "platform.hpp"

#include <SDL3/SDL.h>
#include <cstdint>
#include <string_view>

namespace helios::app {

core::Result<NativeWindow> native_window(SDL_Window* window) {
    const SDL_PropertiesID properties = SDL_GetWindowProperties(window);
    NativeWindow native;
#if defined(SDL_PLATFORM_MACOS)
    native.window_handle = SDL_GetPointerProperty(properties, SDL_PROP_WINDOW_COCOA_WINDOW_POINTER, nullptr);
#elif defined(SDL_PLATFORM_WIN32)
    native.window_handle = SDL_GetPointerProperty(properties, SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr);
#elif defined(SDL_PLATFORM_LINUX)
    if (std::string_view(SDL_GetCurrentVideoDriver()) == "x11") {
        native.display_handle =
            SDL_GetPointerProperty(properties, SDL_PROP_WINDOW_X11_DISPLAY_POINTER, nullptr);
        const auto window_id = static_cast<std::uintptr_t>(
            SDL_GetNumberProperty(properties, SDL_PROP_WINDOW_X11_WINDOW_NUMBER, 0));
        // X11 window ids travel as pointers in bgfx's API.
        // NOLINTNEXTLINE(performance-no-int-to-ptr,cppcoreguidelines-pro-type-reinterpret-cast)
        native.window_handle = reinterpret_cast<void*>(window_id);
    }
#endif
    if (native.window_handle == nullptr) {
        return core::fail(core::ErrorCode::ExternalLibraryFailure,
                          "no native window handle for this platform / video driver");
    }
    return native;
}

} // namespace helios::app
