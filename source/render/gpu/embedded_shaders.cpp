#include "helios/render/gpu/embedded_shaders.hpp"

#include <format>

// NOLINTBEGIN: generated shader blobs (static arrays emitted by bgfx's bin2c).
#include "embedded_shaders.inc"
// NOLINTEND

namespace helios::render::gpu {

namespace {

[[nodiscard]] core::Result<bgfx::ShaderHandle> load_shader(std::string_view name) {
    const bgfx::RendererType::Enum renderer = bgfx::getRendererType();
    for (const EmbeddedShader& shader : k_embedded_shaders) {
        if (shader.name == name && shader.renderer == renderer) {
            const bgfx::ShaderHandle handle =
                bgfx::createShader(bgfx::makeRef(shader.data, static_cast<std::uint32_t>(shader.size)));
            if (!bgfx::isValid(handle)) {
                return core::fail(core::ErrorCode::ExternalLibraryFailure,
                                  std::format("bgfx rejected shader '{}'", name));
            }
            bgfx::setName(handle, shader.name.data(), static_cast<std::int32_t>(shader.name.size()));
            return handle;
        }
    }
    return core::fail(core::ErrorCode::InvalidArgument,
                      std::format("shader '{}' was not compiled for the {} back-end", name,
                                  bgfx::getRendererName(renderer)));
}

} // namespace

core::Result<bgfx::ProgramHandle> load_program(std::string_view vertex_name, std::string_view fragment_name) {
    const auto vertex = load_shader(vertex_name);
    if (!vertex) {
        return std::unexpected(vertex.error());
    }
    const auto fragment = load_shader(fragment_name);
    if (!fragment) {
        bgfx::destroy(*vertex);
        return std::unexpected(fragment.error());
    }
    const bgfx::ProgramHandle program = bgfx::createProgram(*vertex, *fragment, true);
    if (!bgfx::isValid(program)) {
        return core::fail(core::ErrorCode::ExternalLibraryFailure,
                          std::format("bgfx could not link '{}' + '{}'", vertex_name, fragment_name));
    }
    return program;
}

} // namespace helios::render::gpu
