#include "helios/render/gpu/screen_renderer.hpp"

#include "helios/render/gpu/embedded_shaders.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace helios::render::gpu {

namespace {

constexpr std::array<float, 16> k_identity{1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                                           0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};

struct GlassVertex {
    Float3 position;
    float u = 0.0F;
    float v = 0.0F;
};
static_assert(sizeof(GlassVertex) == 20, "GlassVertex must match the bgfx glass layout");

template <typename Handle>
void destroy_if_valid(Handle& handle) noexcept {
    if (bgfx::isValid(handle)) {
        bgfx::destroy(handle);
        handle = BGFX_INVALID_HANDLE;
    }
}

// From a screen's pixels (from the top left, y down) to clip space, as an orthographic
// projection in bgfx's column-major order. Where the back end's render targets start at the
// bottom (OpenGL), the picture is drawn upside down, so that the texture's first row is the
// screen's top on every back end.
[[nodiscard]] std::array<float, 16> pixels_to_clip(std::uint16_t width_px, std::uint16_t height_px,
                                                   bool origin_bottom_left) noexcept {
    const float down = origin_bottom_left ? 1.0F : -1.0F;
    std::array<float, 16> matrix{};
    matrix[0] = 2.0F / static_cast<float>(width_px);
    matrix[5] = down * 2.0F / static_cast<float>(height_px);
    matrix[10] = 1.0F;
    matrix[12] = -1.0F;
    matrix[13] = -down;
    matrix[15] = 1.0F;
    return matrix;
}

// abgr, as the geometry gives colours, to the rgba bgfx clears with.
[[nodiscard]] constexpr std::uint32_t clear_colour(std::uint32_t abgr) noexcept {
    return ((abgr & 0xFFU) << 24U) | (((abgr >> 8U) & 0xFFU) << 16U) | (((abgr >> 16U) & 0xFFU) << 8U)
           | ((abgr >> 24U) & 0xFFU);
}

} // namespace

core::Result<ScreenRenderer> ScreenRenderer::make() {
    ScreenRenderer renderer;
    auto ink = load_program("vs_line", "fs_line");
    if (!ink) {
        return std::unexpected(ink.error());
    }
    renderer.ink_program_ = *ink;
    auto glass = load_program("vs_screen", "fs_screen");
    if (!glass) {
        return std::unexpected(glass.error());
    }
    renderer.glass_program_ = *glass;
    renderer.sampler_ = bgfx::createUniform("s_screen", bgfx::UniformType::Sampler);
    renderer.ink_layout_.begin()
        .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
        .end();
    renderer.glass_layout_.begin()
        .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
        .end();
    return renderer;
}

ScreenRenderer::ScreenRenderer(ScreenRenderer&& other) noexcept
    : ink_program_(std::exchange(other.ink_program_, BGFX_INVALID_HANDLE)),
      glass_program_(std::exchange(other.glass_program_, BGFX_INVALID_HANDLE)),
      sampler_(std::exchange(other.sampler_, BGFX_INVALID_HANDLE)), ink_layout_(other.ink_layout_),
      glass_layout_(other.glass_layout_), targets_(std::exchange(other.targets_, {})) {
}

ScreenRenderer& ScreenRenderer::operator=(ScreenRenderer&& other) noexcept {
    if (this != &other) {
        release();
        ink_program_ = std::exchange(other.ink_program_, BGFX_INVALID_HANDLE);
        glass_program_ = std::exchange(other.glass_program_, BGFX_INVALID_HANDLE);
        sampler_ = std::exchange(other.sampler_, BGFX_INVALID_HANDLE);
        ink_layout_ = other.ink_layout_;
        glass_layout_ = other.glass_layout_;
        targets_ = std::exchange(other.targets_, {});
    }
    return *this;
}

ScreenRenderer::~ScreenRenderer() {
    release();
}

void ScreenRenderer::release() noexcept {
    destroy_if_valid(ink_program_);
    destroy_if_valid(glass_program_);
    destroy_if_valid(sampler_);
    for (Target& target : targets_) {
        destroy_if_valid(target.frame);
    }
    targets_.clear();
}

const ScreenRenderer::Target* ScreenRenderer::target_of(const ScreenSurface& surface) const noexcept {
    const auto match = std::ranges::find_if(targets_, [&](const Target& target) {
        return target.part == surface.part && target.screen == surface.screen
               && target.width_px == surface.image.width_px && target.height_px == surface.image.height_px;
    });
    return match != targets_.end() ? &*match : nullptr;
}

void ScreenRenderer::paint(std::span<const ScreenSurface> screens, bgfx::ViewId first_view) {
    const std::span<const ScreenSurface> painted = screens.first(std::min(screens.size(), k_most_screens));
    // Let go of the textures of screens no longer shown.
    std::erase_if(targets_, [&](Target& target) {
        const bool shown = std::ranges::any_of(painted, [&](const ScreenSurface& surface) {
            return target.part == surface.part && target.screen == surface.screen
                   && target.width_px == surface.image.width_px
                   && target.height_px == surface.image.height_px;
        });
        if (!shown) {
            destroy_if_valid(target.frame);
        }
        return !shown;
    });
    const bool origin_bottom_left = bgfx::getCaps()->originBottomLeft;
    for (std::size_t index = 0; index < painted.size(); ++index) {
        const ScreenSurface& surface = painted[index];
        const ScreenImage& image = surface.image;
        if (target_of(surface) == nullptr) {
            Target target{.part = surface.part,
                          .screen = surface.screen,
                          .width_px = image.width_px,
                          .height_px = image.height_px,
                          .frame = bgfx::createFrameBuffer(image.width_px, image.height_px,
                                                           bgfx::TextureFormat::RGBA8,
                                                           BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP),
                          .texture = BGFX_INVALID_HANDLE};
            if (!bgfx::isValid(target.frame)) {
                continue;
            }
            target.texture = bgfx::getTexture(target.frame);
            targets_.push_back(target);
        }
        const Target& target = *target_of(surface);
        const auto view = static_cast<bgfx::ViewId>(first_view + index);
        bgfx::setViewName(view, "screen");
        bgfx::setViewFrameBuffer(view, target.frame);
        bgfx::setViewRect(view, 0, 0, image.width_px, image.height_px);
        bgfx::setViewClear(view, BGFX_CLEAR_COLOR, clear_colour(image.background));
        const std::array<float, 16> projection =
            pixels_to_clip(image.width_px, image.height_px, origin_bottom_left);
        bgfx::setViewTransform(view, k_identity.data(), projection.data());
        bgfx::touch(view);
        const auto count = static_cast<std::uint32_t>(image.triangles.size());
        // Rationale: rebuilt every frame; if the transient buffers are full the screen shows
        // only its background for a frame.
        if (count == 0U || bgfx::getAvailTransientVertexBuffer(count, ink_layout_) != count) {
            continue;
        }
        bgfx::TransientVertexBuffer vertices{};
        bgfx::allocTransientVertexBuffer(&vertices, count, ink_layout_);
        std::memcpy(vertices.data, image.triangles.data(), image.triangles.size() * sizeof(LineVertex));
        bgfx::setVertexBuffer(0, &vertices);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
        bgfx::submit(view, ink_program_);
    }
}

void ScreenRenderer::draw(std::span<const ScreenSurface> screens, bgfx::ViewId view) const {
    constexpr std::uint32_t k_vertices = 6;
    for (const ScreenSurface& surface : screens.first(std::min(screens.size(), k_most_screens))) {
        const Target* target = target_of(surface);
        if (target == nullptr
            || bgfx::getAvailTransientVertexBuffer(k_vertices, glass_layout_) != k_vertices) {
            continue;
        }
        const std::array<Float3, 4>& corner = surface.corners;
        const std::array<GlassVertex, k_vertices> quad{{
            {corner[0], 0.0F, 0.0F},
            {corner[1], 1.0F, 0.0F},
            {corner[2], 1.0F, 1.0F},
            {corner[0], 0.0F, 0.0F},
            {corner[2], 1.0F, 1.0F},
            {corner[3], 0.0F, 1.0F},
        }};
        bgfx::TransientVertexBuffer vertices{};
        bgfx::allocTransientVertexBuffer(&vertices, k_vertices, glass_layout_);
        std::memcpy(vertices.data, quad.data(), sizeof(quad));
        bgfx::setTransform(k_identity.data());
        bgfx::setVertexBuffer(0, &vertices);
        bgfx::setTexture(0, sampler_, target->texture);
        // Not culled: seen from the front, and the bezel hides it from behind.
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                       | BGFX_STATE_DEPTH_TEST_GREATER | BGFX_STATE_MSAA);
        bgfx::submit(view, glass_program_);
    }
}

} // namespace helios::render::gpu
