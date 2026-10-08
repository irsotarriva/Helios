#ifndef HELIOS_XR_SESSION_HPP
#define HELIOS_XR_SESSION_HPP

#include "helios/core/error.hpp"
#include "helios/render/camera.hpp"
#include "helios/render/flight_geometry.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// VR (BRIEFING §12, D29): a headset through OpenXR. The session hands out the images of each
// eye as textures of the renderer's own graphics device, tells where the eyes are every frame,
// and takes the drawn images back. No OpenXR type appears here; the renderer wraps the textures
// (render/gpu/eye_target.hpp) and the client draws its ordinary views into them.
namespace helios::xr {

inline constexpr std::size_t k_eyes = 2; // left, right

enum class GraphicsApi : std::uint8_t {
    Direct3D11, // `device` is an ID3D11Device*
};

// The graphics device the renderer already draws with. The headset's images are created on it.
struct Graphics {
    GraphicsApi api = GraphicsApi::Direct3D11;
    void* device = nullptr;
};

enum class ColourFormat : std::uint8_t {
    Rgba8,
    Bgra8,
};

// The images one eye is drawn into, in turn: textures of the graphics device (for Direct3D 11,
// ID3D11Texture2D*). The headset shows what is written as sRGB-encoded colour, which is what
// the window shows too.
struct EyeImages {
    std::uint32_t width_px = 0;
    std::uint32_t height_px = 0;
    ColourFormat format = ColourFormat::Rgba8;
    std::vector<std::uintptr_t> textures;
};

struct Eye {
    // Relative to where the head was when the session began (or was last re-centred), which
    // is taken to be the eye point of the seat.
    render::TrackedPose pose;
    render::FieldOfView field_of_view;
    std::size_t image = 0; // which of the eye's images to draw into this frame
};

// One frame of the headset. Between begin_frame() and end_frame() the client draws each eye
// into its image and has the renderer carry the drawing out.
struct Frame {
    bool drawn = false; // false: the headset shows nothing of this frame, and `eyes` means nothing
    std::int64_t display_time_ns = 0;
    std::array<Eye, k_eyes> eyes;
    render::TrackedPose head; // midway between the eyes
};

class Session {
public:
    // Fails when there is no OpenXR runtime, no headset, or the runtime cannot use the device.
    [[nodiscard]] static core::Result<Session> make(const Graphics& graphics);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&& other) noexcept;
    Session& operator=(Session&& other) noexcept;
    ~Session();

    [[nodiscard]] const std::string& runtime_name() const noexcept;
    [[nodiscard]] const std::string& system_name() const noexcept;
    [[nodiscard]] const std::array<EyeImages, k_eyes>& images() const noexcept;
    // The time between two frames of the headset, as it last said; 0 before the first frame.
    [[nodiscard]] double frame_period_s() const noexcept;

    // Takes what the runtime has to say (the headset put on or taken off, the session ending).
    // False once the runtime wants the session over.
    [[nodiscard]] core::Result<bool> poll();

    // Waits until the headset wants the next frame, which is what paces the client in VR.
    // While the headset is not in use this returns at once with a frame that is not drawn.
    [[nodiscard]] core::Result<Frame> begin_frame();
    [[nodiscard]] core::VoidResult end_frame(const Frame& frame);

private:
    struct State;
    explicit Session(std::unique_ptr<State> state) noexcept;

    std::unique_ptr<State> state_;
};

} // namespace helios::xr

#endif // HELIOS_XR_SESSION_HPP
