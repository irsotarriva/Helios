#include "helios/render/gpu/imgui_renderer.hpp"

#include "helios/render/gpu/embedded_shaders.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <imgui.h>
#include <span>
#include <utility>
#include <vector>

namespace helios::render::gpu {

namespace {

// ImTextureID 0 means "invalid", but bgfx handle index 0 is valid: store index + 1.
[[nodiscard]] ImTextureID to_texture_id(bgfx::TextureHandle handle) noexcept {
    return static_cast<ImTextureID>(handle.idx) + 1U;
}

[[nodiscard]] bgfx::TextureHandle from_texture_id(ImTextureID id) noexcept {
    return bgfx::TextureHandle{static_cast<std::uint16_t>(id - 1U)};
}

constexpr std::uint64_t k_sampler_flags = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

// ImGui's "reset render state" request. Every draw here sets its full state, so it is a no-op;
// its address is what identifies the request.
void reset_render_state(const ImDrawList* /*list*/, const ImDrawCmd* /*command*/) {
}

} // namespace

core::Result<ImGuiRenderer> ImGuiRenderer::make(bgfx::ViewId view_id) {
    ImGuiRenderer renderer(view_id);
    auto program = load_program("vs_imgui", "fs_imgui");
    if (!program) {
        return std::unexpected(program.error());
    }
    renderer.program_ = *program;
    renderer.sampler_ = bgfx::createUniform("s_texture", bgfx::UniformType::Sampler);
    renderer.layout_.begin()
        .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
        .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
        .end();

    ImGuiIO& io = ImGui::GetIO();
    io.BackendRendererName = "helios_bgfx";
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset | ImGuiBackendFlags_RendererHasTextures;
    ImGui::GetPlatformIO().DrawCallback_ResetRenderState = &reset_render_state;
    renderer.owns_context_state_ = true;
    return renderer;
}

ImGuiRenderer::ImGuiRenderer(ImGuiRenderer&& other) noexcept
    : view_id_(other.view_id_), program_(std::exchange(other.program_, BGFX_INVALID_HANDLE)),
      sampler_(std::exchange(other.sampler_, BGFX_INVALID_HANDLE)), layout_(other.layout_),
      owns_context_state_(std::exchange(other.owns_context_state_, false)) {
}

ImGuiRenderer::~ImGuiRenderer() {
    if (owns_context_state_ && ImGui::GetCurrentContext() != nullptr) {
        for (ImTextureData* texture : ImGui::GetPlatformIO().Textures) {
            if (texture->RefCount == 1 && texture->TexID != ImTextureID_Invalid) {
                bgfx::destroy(from_texture_id(texture->TexID));
                texture->SetTexID(ImTextureID_Invalid);
                texture->SetStatus(ImTextureStatus_Destroyed);
            }
        }
        ImGuiIO& io = ImGui::GetIO();
        io.BackendRendererName = nullptr;
        io.BackendFlags &= ~(ImGuiBackendFlags_RendererHasVtxOffset | ImGuiBackendFlags_RendererHasTextures);
    }
    if (bgfx::isValid(program_)) {
        bgfx::destroy(program_);
    }
    if (bgfx::isValid(sampler_)) {
        bgfx::destroy(sampler_);
    }
}

void ImGuiRenderer::update_texture(ImTextureData& texture) {
    if (texture.Status == ImTextureStatus_WantCreate) {
        // Rationale: RGBA32 only; ImGui's default atlas format. The texture is created without
        // data and then filled, because bgfx makes a texture created *with* data immutable, and
        // ImGui adds glyphs to the atlas at run time.
        const auto width = static_cast<std::uint16_t>(texture.Width);
        const auto height = static_cast<std::uint16_t>(texture.Height);
        const bgfx::TextureHandle handle = bgfx::createTexture2D(
            width, height, false, 1, bgfx::TextureFormat::RGBA8, k_sampler_flags, nullptr);
        bgfx::updateTexture2D(
            handle, 0, 0, 0, 0, width, height,
            bgfx::copy(texture.GetPixels(), static_cast<std::uint32_t>(texture.GetSizeInBytes())));
        texture.SetTexID(to_texture_id(handle));
        texture.SetStatus(ImTextureStatus_OK);
    } else if (texture.Status == ImTextureStatus_WantUpdates) {
        const bgfx::TextureHandle handle = from_texture_id(texture.TexID);
        for (const ImTextureRect& rect : texture.Updates) {
            const auto row_bytes =
                static_cast<std::uint32_t>(rect.w) * static_cast<std::uint32_t>(texture.BytesPerPixel);
            const bgfx::Memory* memory = bgfx::alloc(row_bytes * rect.h);
            const std::span<std::uint8_t> destination(memory->data, memory->size);
            for (int row = 0; row < rect.h; ++row) {
                std::memcpy(destination.subspan(static_cast<std::size_t>(row) * row_bytes).data(),
                            texture.GetPixelsAt(rect.x, rect.y + row), row_bytes);
            }
            bgfx::updateTexture2D(handle, 0, 0, rect.x, rect.y, rect.w, rect.h, memory);
        }
        texture.SetStatus(ImTextureStatus_OK);
    } else if (texture.Status == ImTextureStatus_WantDestroy && texture.UnusedFrames > 0) {
        bgfx::destroy(from_texture_id(texture.TexID));
        texture.SetTexID(ImTextureID_Invalid);
        texture.SetStatus(ImTextureStatus_Destroyed);
    }
}

void ImGuiRenderer::render(ImDrawData* draw_data) {
    if (draw_data == nullptr) {
        return;
    }
    if (draw_data->Textures != nullptr) {
        for (ImTextureData* texture : *draw_data->Textures) {
            if (texture->Status != ImTextureStatus_OK) {
                update_texture(*texture);
            }
        }
    }
    const float width = draw_data->DisplaySize.x * draw_data->FramebufferScale.x;
    const float height = draw_data->DisplaySize.y * draw_data->FramebufferScale.y;
    if (width <= 0.0F || height <= 0.0F) {
        return;
    }

    bgfx::setViewName(view_id_, "ImGui");
    bgfx::setViewMode(view_id_, bgfx::ViewMode::Sequential);
    bgfx::setViewRect(view_id_, 0, 0, static_cast<std::uint16_t>(width), static_cast<std::uint16_t>(height));
    // Orthographic projection in ImGui's display coordinates (y down).
    const float left = draw_data->DisplayPos.x;
    const float right = left + draw_data->DisplaySize.x;
    const float top = draw_data->DisplayPos.y;
    const float bottom = top + draw_data->DisplaySize.y;
    const bool homogeneous = bgfx::getCaps()->homogeneousDepth;
    const std::array<float, 16> ortho{2.0F / (right - left),
                                      0.0F,
                                      0.0F,
                                      0.0F,
                                      0.0F,
                                      2.0F / (top - bottom),
                                      0.0F,
                                      0.0F,
                                      0.0F,
                                      0.0F,
                                      homogeneous ? -1.0F : -0.5F,
                                      0.0F,
                                      (right + left) / (left - right),
                                      (top + bottom) / (bottom - top),
                                      homogeneous ? 0.0F : 0.5F,
                                      1.0F};
    bgfx::setViewTransform(view_id_, nullptr, ortho.data());
    bgfx::touch(view_id_);

    const ImVec2 clip_offset = draw_data->DisplayPos;
    const ImVec2 clip_scale = draw_data->FramebufferScale;
    for (const ImDrawList* list : draw_data->CmdLists) {
        const auto vertex_count = static_cast<std::uint32_t>(list->VtxBuffer.size());
        const auto index_count = static_cast<std::uint32_t>(list->IdxBuffer.size());
        if (bgfx::getAvailTransientVertexBuffer(vertex_count, layout_) < vertex_count
            || bgfx::getAvailTransientIndexBuffer(index_count, sizeof(ImDrawIdx) == 4) < index_count) {
            break; // out of transient memory: drop the rest of the UI this frame
        }
        bgfx::TransientVertexBuffer vertices{};
        bgfx::TransientIndexBuffer indices{};
        bgfx::allocTransientVertexBuffer(&vertices, vertex_count, layout_);
        bgfx::allocTransientIndexBuffer(&indices, index_count, sizeof(ImDrawIdx) == 4);
        std::memcpy(vertices.data, list->VtxBuffer.Data, vertex_count * sizeof(ImDrawVert));
        std::memcpy(indices.data, list->IdxBuffer.Data, index_count * sizeof(ImDrawIdx));

        for (const ImDrawCmd& command : list->CmdBuffer) {
            if (command.UserCallback != nullptr) {
                if (command.UserCallback != &reset_render_state) {
                    command.UserCallback(list, &command);
                }
                continue;
            }
            if (command.ElemCount == 0) {
                continue;
            }
            const float clip_left = (command.ClipRect.x - clip_offset.x) * clip_scale.x;
            const float clip_top = (command.ClipRect.y - clip_offset.y) * clip_scale.y;
            const float clip_right = (command.ClipRect.z - clip_offset.x) * clip_scale.x;
            const float clip_bottom = (command.ClipRect.w - clip_offset.y) * clip_scale.y;
            if (clip_right <= clip_left || clip_bottom <= clip_top || clip_left >= width
                || clip_top >= height) {
                continue;
            }
            const auto scissor_x = static_cast<std::uint16_t>(std::max(clip_left, 0.0F));
            const auto scissor_y = static_cast<std::uint16_t>(std::max(clip_top, 0.0F));
            bgfx::setScissor(
                scissor_x, scissor_y,
                static_cast<std::uint16_t>(std::min(clip_right, 65535.0F) - static_cast<float>(scissor_x)),
                static_cast<std::uint16_t>(std::min(clip_bottom, 65535.0F) - static_cast<float>(scissor_y)));
            bgfx::setState(
                BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_MSAA
                | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA));
            bgfx::setTexture(0, sampler_, from_texture_id(command.GetTexID()));
            bgfx::setVertexBuffer(0, &vertices, command.VtxOffset, vertex_count - command.VtxOffset);
            bgfx::setIndexBuffer(&indices, command.IdxOffset, command.ElemCount);
            bgfx::submit(view_id_, program_);
        }
    }
}

} // namespace helios::render::gpu
