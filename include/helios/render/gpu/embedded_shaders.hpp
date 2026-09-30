#ifndef HELIOS_RENDER_GPU_EMBEDDED_SHADERS_HPP
#define HELIOS_RENDER_GPU_EMBEDDED_SHADERS_HPP

#include "helios/core/error.hpp"

#include <bgfx/bgfx.h>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace helios::render::gpu {

// A compiled shader for one back-end, embedded in the executable at build time.
struct EmbeddedShader {
    std::string_view name;
    bgfx::RendererType::Enum renderer;
    const std::uint8_t* data;
    std::size_t size;
};

// Creates the program `vertex_name` + `fragment_name` for the running back-end. Fails if the
// shaders were not compiled for it (see source/render/gpu/CMakeLists.txt).
[[nodiscard]] core::Result<bgfx::ProgramHandle> load_program(std::string_view vertex_name,
                                                             std::string_view fragment_name);

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_EMBEDDED_SHADERS_HPP
