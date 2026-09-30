#ifndef HELIOS_RENDER_GPU_IMGUI_RENDERER_HPP
#define HELIOS_RENDER_GPU_IMGUI_RENDERER_HPP

#include "helios/core/error.hpp"

#include <bgfx/bgfx.h>

struct ImDrawData;
struct ImTextureData;

namespace helios::render::gpu {

// Dear ImGui renderer back-end for bgfx (ImGui ships none). Supports ImGui's texture protocol
// (ImGuiBackendFlags_RendererHasTextures) so the font atlas can grow at run time. Create after
// ImGui::CreateContext and bgfx::init; destroy before either is shut down.
class ImGuiRenderer {
public:
    [[nodiscard]] static core::Result<ImGuiRenderer> make(bgfx::ViewId view_id);

    ImGuiRenderer(const ImGuiRenderer&) = delete;
    ImGuiRenderer& operator=(const ImGuiRenderer&) = delete;
    ImGuiRenderer(ImGuiRenderer&& other) noexcept;
    ImGuiRenderer& operator=(ImGuiRenderer&&) = delete;
    ~ImGuiRenderer();

    // Draws ImGui::GetDrawData() output on top of everything, in window pixels.
    void render(ImDrawData* draw_data);

private:
    explicit ImGuiRenderer(bgfx::ViewId view_id) noexcept : view_id_(view_id) {}
    static void update_texture(ImTextureData& texture);

    bgfx::ViewId view_id_;
    bgfx::ProgramHandle program_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle sampler_ = BGFX_INVALID_HANDLE;
    bgfx::VertexLayout layout_;
    bool owns_context_state_ = false;
};

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_IMGUI_RENDERER_HPP
