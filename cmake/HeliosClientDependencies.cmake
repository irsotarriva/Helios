include(FetchContent)

# Third-party libraries of the game client (the renderer, windowing and debug UI). The headless
# simulation never links any of them. Approved by the maintainer for Phase 2 (BRIEFING D19).
#
# Rationale for FetchContent over vcpkg here: bgfx needs its shader compiler (shaderc) built for
# the host and its CMake helpers (bgfx_compile_shaders) at configure time, which bgfx.cmake
# provides directly; pins are commits, like the LumenLog submodule. Bump the pins
# deliberately; never track a moving branch. To use a local checkout of any of them, pass
# -DFETCHCONTENT_SOURCE_DIR_<NAME>=<path> (e.g. FETCHCONTENT_SOURCE_DIR_BGFX).
set(HELIOS_BGFX_CMAKE_COMMIT "de08a6080b39994ab8a9eddb82e79e18bc3df7bd") # bgfx.cmake v1.161.9510-579
set(HELIOS_SDL3_COMMIT "fa2c02bb6e21974a89ea9824bc53c9932abe5f9c")       # SDL release-3.4.16
set(HELIOS_IMGUI_COMMIT "f1cc2ae15e53a861a874c3034aae6798fde194ab")      # Dear ImGui v1.92.9b
set(HELIOS_OPENXR_COMMIT "f2448a8797c85814aa892efc1ab8707900fbcc78")     # OpenXR-SDK release-1.1.63

# bgfx: only the libraries and shaderc.
set(BGFX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BGFX_INSTALL OFF CACHE BOOL "" FORCE)
set(BGFX_CUSTOM_TARGETS OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS ON CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS_SHADER ON CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS_BIN2C ON CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS_GEOMETRY OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS_TEXTURE OFF CACHE BOOL "" FORCE)
set(BGFX_CONFIG_VIDEO OFF CACHE BOOL "" FORCE)
# X11 only on Linux for now; Wayland sessions run it through XWayland.
set(BGFX_WITH_WAYLAND OFF CACHE BOOL "" FORCE)

FetchContent_Declare(bgfx
  GIT_REPOSITORY https://github.com/bkaradzic/bgfx.cmake.git
  GIT_TAG        ${HELIOS_BGFX_CMAKE_COMMIT}
  GIT_SUBMODULES_RECURSE ON
  GIT_SHALLOW    OFF
  SYSTEM
)

# SDL3: a static library with only what the client uses.
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
set(SDL_WAYLAND OFF CACHE BOOL "" FORCE)
set(SDL_AUDIO OFF CACHE BOOL "" FORCE)
set(SDL_CAMERA OFF CACHE BOOL "" FORCE)
set(SDL_GPU OFF CACHE BOOL "" FORCE)
set(SDL_RENDER OFF CACHE BOOL "" FORCE)
set(SDL_HAPTIC OFF CACHE BOOL "" FORCE)
set(SDL_SENSOR OFF CACHE BOOL "" FORCE)
set(SDL_POWER OFF CACHE BOOL "" FORCE)
set(SDL_DIALOG OFF CACHE BOOL "" FORCE)
set(SDL_X11_XTEST OFF CACHE BOOL "" FORCE)
set(SDL_X11_XSCRNSAVER OFF CACHE BOOL "" FORCE)

FetchContent_Declare(SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG        ${HELIOS_SDL3_COMMIT}
  SYSTEM
)

# Dear ImGui has no CMake build of its own; helios_imgui below compiles it.
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        ${HELIOS_IMGUI_COMMIT}
)

# Rationale: bgfx, bx and dawn sources are UTF-8. Without /utf-8, MSVC reads them in the
# system code page (C4819), and on a double-byte code page such as 932 (Japanese) a multi-byte
# character can swallow the end of a comment line and silently change the code that follows.
if(MSVC)
  add_compile_options(/utf-8)
endif()

FetchContent_MakeAvailable(bgfx SDL3 imgui)

# OpenXR: the Khronos headers and loader, linked statically. The loader finds the headset's
# runtime when a session is asked for; nothing of it runs otherwise.
if(HELIOS_BUILD_XR)
  set(DYNAMIC_LOADER OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
  set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(openxr
    GIT_REPOSITORY https://github.com/KhronosGroup/OpenXR-SDK.git
    GIT_TAG        ${HELIOS_OPENXR_COMMIT}
    SYSTEM
  )
  FetchContent_MakeAvailable(openxr)
endif()

add_library(helios_imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
)
target_include_directories(helios_imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(helios_imgui PUBLIC SDL3::SDL3-static)
target_compile_definitions(helios_imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
add_library(helios::imgui ALIAS helios_imgui)

# Third-party code is not ours to lint: keep clang-tidy away from it even when a caller sets
# CMAKE_CXX_CLANG_TIDY globally.
foreach(third_party_target helios_imgui bgfx bx bimg shaderc openxr_loader)
  if(TARGET ${third_party_target})
    set_target_properties(${third_party_target} PROPERTIES CXX_CLANG_TIDY "")
  endif()
endforeach()
