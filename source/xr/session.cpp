#include "helios/xr/session.hpp"

#include "helios/core/logging.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"

#include <algorithm>
#include <cstring>
#include <format>
#include <string_view>
#include <utility>

// Rationale: OpenXR's platform header declares the Direct3D 11 binding only when these are
// defined, and needs the Direct3D header before it.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <d3d11.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

namespace helios::xr {

using core::ErrorCode;

namespace {

constexpr XrViewConfigurationType k_view_configuration = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
constexpr double k_nanoseconds = 1e-9;

// DXGI_FORMAT values, in the order they are wanted. Rationale for sRGB first: the renderer
// writes display-encoded colour, as it does to the window; the runtime must read it as such.
struct KnownFormat {
    std::int64_t dxgi;
    ColourFormat format;
    bool srgb;
};
constexpr std::array<KnownFormat, 4> k_known_formats{
    KnownFormat{.dxgi = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, .format = ColourFormat::Rgba8, .srgb = true},
    KnownFormat{.dxgi = DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, .format = ColourFormat::Bgra8, .srgb = true},
    KnownFormat{.dxgi = DXGI_FORMAT_R8G8B8A8_UNORM, .format = ColourFormat::Rgba8, .srgb = false},
    KnownFormat{.dxgi = DXGI_FORMAT_B8G8R8A8_UNORM, .format = ColourFormat::Bgra8, .srgb = false},
};

[[nodiscard]] std::string_view meaning(XrResult result) noexcept {
    switch (result) {
    case XR_ERROR_RUNTIME_UNAVAILABLE:          return "no OpenXR runtime is installed or active";
    case XR_ERROR_FORM_FACTOR_UNAVAILABLE:      return "the headset is not connected";
    case XR_ERROR_FORM_FACTOR_UNSUPPORTED:      return "the runtime has no headset";
    case XR_ERROR_GRAPHICS_DEVICE_INVALID:      return "the runtime cannot use the renderer's graphics device";
    case XR_ERROR_EXTENSION_NOT_PRESENT:        return "the runtime lacks an extension";
    case XR_ERROR_API_VERSION_UNSUPPORTED:      return "the runtime does not speak this OpenXR version";
    case XR_ERROR_RUNTIME_FAILURE:              return "the runtime failed";
    case XR_ERROR_SESSION_LOST:                 return "the session was lost";
    case XR_ERROR_INSTANCE_LOST:                return "the runtime went away";
    case XR_ERROR_SESSION_NOT_RUNNING:          return "the session is not running";
    case XR_ERROR_LIMIT_REACHED:                return "a limit of the runtime was reached";
    case XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED: return "the image format is not supported";
    default:                                    return "OpenXR error";
    }
}

[[nodiscard]] core::VoidResult check(XrResult result, std::string_view call) {
    if (XR_SUCCEEDED(result)) {
        return {};
    }
    return core::fail(ErrorCode::ExternalLibraryFailure,
                      std::format("{}: {} ({})", call, meaning(result), static_cast<int>(result)));
}

// For what is being taken down: there is no one to hand a failure to, so it is logged.
void gone(XrResult result, std::string_view call) {
    if (core::VoidResult destroyed = check(result, call); !destroyed) {
        LOG_WARN("{}", core::describe(destroyed.error())).tag("subsystem", "xr");
    }
}

// OpenXR's axes are right, up and backward; ours are forward, left and up. The same change of
// axes on both sides of the rotation, so a quaternion's vector part changes like a vector.
[[nodiscard]] math::Vector3 to_seat_axes(const XrVector3f& vector) noexcept {
    return {-static_cast<double>(vector.z), -static_cast<double>(vector.x), static_cast<double>(vector.y)};
}

[[nodiscard]] render::TrackedPose to_tracked(const XrPosef& pose) noexcept {
    return {.position_m = to_seat_axes(pose.position),
            .orientation = math::normalized(math::Quaternion{.w = static_cast<double>(pose.orientation.w),
                                                             .x = -static_cast<double>(pose.orientation.z),
                                                             .y = -static_cast<double>(pose.orientation.x),
                                                             .z = static_cast<double>(pose.orientation.y)})};
}

[[nodiscard]] render::FieldOfView to_field_of_view(const XrFovf& fov) noexcept {
    return {.left_rad = static_cast<double>(fov.angleLeft),
            .right_rad = static_cast<double>(fov.angleRight),
            .up_rad = static_cast<double>(fov.angleUp),
            .down_rad = static_cast<double>(fov.angleDown)};
}

// Into a fixed character array of an OpenXR structure that is still all zeros.
template <std::size_t Size>
void copy_name(char (&target)[Size], std::string_view name) noexcept { // NOLINT(*-avoid-c-arrays)
    std::memcpy(static_cast<char*>(target), name.data(), std::min(name.size(), Size - 1));
}

// A fixed character array of an OpenXR structure, up to its terminator.
template <std::size_t Size>
[[nodiscard]] std::string to_string(const char (&text)[Size]) { // NOLINT(*-avoid-c-arrays): OpenXR's.
    const std::string_view view(static_cast<const char*>(text), Size);
    return std::string(view.substr(0, view.find('\0')));
}

[[nodiscard]] core::Result<bool> has_extension(std::string_view name) {
    std::uint32_t count = 0;
    if (core::VoidResult counted =
            check(xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr), "OpenXR extensions");
        !counted) {
        return std::unexpected(counted.error());
    }
    std::vector<XrExtensionProperties> extensions(
        count, XrExtensionProperties{.type = XR_TYPE_EXTENSION_PROPERTIES});
    if (core::VoidResult listed =
            check(xrEnumerateInstanceExtensionProperties(nullptr, count, &count, extensions.data()),
                  "OpenXR extensions");
        !listed) {
        return std::unexpected(listed.error());
    }
    return std::ranges::any_of(extensions, [&](const XrExtensionProperties& extension) {
        return name == static_cast<const char*>(extension.extensionName);
    });
}

} // namespace

struct Session::State {
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace space = XR_NULL_HANDLE;
    std::array<XrSwapchain, k_eyes> swapchains{};
    std::array<EyeImages, k_eyes> images;
    std::array<XrView, k_eyes> views{};
    std::string runtime_name;
    std::string system_name;
    double frame_period_s = 0.0;
    bool running = false;     // between xrBeginSession and xrEndSession
    bool over = false;        // the runtime wants the session ended
    bool frame_begun = false; // between xrBeginFrame and xrEndFrame

    State() = default;
    State(const State&) = delete;
    State& operator=(const State&) = delete;
    State(State&&) = delete;
    State& operator=(State&&) = delete;

    ~State() {
        for (XrSwapchain swapchain : swapchains) {
            if (swapchain != XR_NULL_HANDLE) {
                gone(xrDestroySwapchain(swapchain), "xrDestroySwapchain");
            }
        }
        if (space != XR_NULL_HANDLE) {
            gone(xrDestroySpace(space), "xrDestroySpace");
        }
        if (session != XR_NULL_HANDLE) {
            gone(xrDestroySession(session), "xrDestroySession");
        }
        if (instance != XR_NULL_HANDLE) {
            gone(xrDestroyInstance(instance), "xrDestroyInstance");
        }
    }

    [[nodiscard]] core::VoidResult open_instance();
    [[nodiscard]] core::VoidResult open_session(const Graphics& graphics);
    [[nodiscard]] core::VoidResult make_images();
    [[nodiscard]] core::VoidResult state_changed(XrSessionState state);
};

core::VoidResult Session::State::open_instance() {
    const auto direct3d = has_extension(XR_KHR_D3D11_ENABLE_EXTENSION_NAME);
    if (!direct3d) {
        return std::unexpected(direct3d.error());
    }
    if (!*direct3d) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the OpenXR runtime has no Direct3D 11 binding");
    }
    const std::array<const char*, 1> extensions{XR_KHR_D3D11_ENABLE_EXTENSION_NAME};
    XrInstanceCreateInfo create{.type = XR_TYPE_INSTANCE_CREATE_INFO};
    copy_name(create.applicationInfo.applicationName, "Helios");
    copy_name(create.applicationInfo.engineName, "Helios");
    create.applicationInfo.applicationVersion = 1;
    // Rationale: nothing of OpenXR 1.1 is used, and 1.0 is what every runtime speaks.
    create.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    create.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    create.enabledExtensionNames = extensions.data();
    if (core::VoidResult created = check(xrCreateInstance(&create, &instance), "xrCreateInstance");
        !created) {
        return created;
    }
    XrInstanceProperties properties{.type = XR_TYPE_INSTANCE_PROPERTIES};
    if (XR_SUCCEEDED(xrGetInstanceProperties(instance, &properties))) {
        runtime_name = std::format(
            "{} {}.{}.{}", to_string(properties.runtimeName), XR_VERSION_MAJOR(properties.runtimeVersion),
            XR_VERSION_MINOR(properties.runtimeVersion), XR_VERSION_PATCH(properties.runtimeVersion));
    }
    const XrSystemGetInfo wanted{.type = XR_TYPE_SYSTEM_GET_INFO,
                                 .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    if (core::VoidResult found = check(xrGetSystem(instance, &wanted, &system), "xrGetSystem"); !found) {
        return found;
    }
    XrSystemProperties system_properties{.type = XR_TYPE_SYSTEM_PROPERTIES};
    if (XR_SUCCEEDED(xrGetSystemProperties(instance, system, &system_properties))) {
        system_name = to_string(system_properties.systemName);
    }
    return {};
}

core::VoidResult Session::State::open_session(const Graphics& graphics) {
    if (graphics.api != GraphicsApi::Direct3D11 || graphics.device == nullptr) {
        return core::fail(ErrorCode::InvalidArgument, "VR needs the Direct3D 11 renderer");
    }
    // The runtime must be asked what it needs of the device before a session is made on it.
    PFN_xrGetD3D11GraphicsRequirementsKHR requirements_of = nullptr;
    if (core::VoidResult found =
            check(xrGetInstanceProcAddr(
                      instance, "xrGetD3D11GraphicsRequirementsKHR",
                      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): OpenXR's way.
                      reinterpret_cast<PFN_xrVoidFunction*>(&requirements_of)),
                  "xrGetInstanceProcAddr");
        !found) {
        return found;
    }
    XrGraphicsRequirementsD3D11KHR requirements{.type = XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    if (core::VoidResult asked =
            check(requirements_of(instance, system, &requirements), "xrGetD3D11GraphicsRequirementsKHR");
        !asked) {
        return asked;
    }
    auto* const device = static_cast<ID3D11Device*>(graphics.device);
    if (device->GetFeatureLevel() < requirements.minFeatureLevel) {
        return core::fail(ErrorCode::ExternalLibraryFailure,
                          "the graphics device is below the feature level the headset needs");
    }
    const XrGraphicsBindingD3D11KHR binding{.type = XR_TYPE_GRAPHICS_BINDING_D3D11_KHR, .device = device};
    const XrSessionCreateInfo create{
        .type = XR_TYPE_SESSION_CREATE_INFO, .next = &binding, .systemId = system};
    if (core::VoidResult created = check(xrCreateSession(instance, &create, &session), "xrCreateSession");
        !created) {
        return created;
    }
    // The seat's eye point: where the head is when the session begins, level with gravity.
    XrReferenceSpaceCreateInfo local{.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
                                     .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL};
    local.poseInReferenceSpace.orientation.w = 1.0F;
    return check(xrCreateReferenceSpace(session, &local, &space), "xrCreateReferenceSpace");
}

core::VoidResult Session::State::make_images() {
    std::array<XrViewConfigurationView, k_eyes> sizes{
        XrViewConfigurationView{.type = XR_TYPE_VIEW_CONFIGURATION_VIEW},
        XrViewConfigurationView{.type = XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    std::uint32_t count = 0;
    if (core::VoidResult listed = check(
            xrEnumerateViewConfigurationViews(instance, system, k_view_configuration,
                                              static_cast<std::uint32_t>(sizes.size()), &count, sizes.data()),
            "xrEnumerateViewConfigurationViews");
        !listed) {
        return listed;
    }
    if (count != k_eyes) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the headset does not have two views");
    }

    std::uint32_t format_count = 0;
    if (core::VoidResult counted = check(xrEnumerateSwapchainFormats(session, 0, &format_count, nullptr),
                                         "xrEnumerateSwapchainFormats");
        !counted) {
        return counted;
    }
    std::vector<std::int64_t> offered(format_count);
    if (core::VoidResult listed =
            check(xrEnumerateSwapchainFormats(session, format_count, &format_count, offered.data()),
                  "xrEnumerateSwapchainFormats");
        !listed) {
        return listed;
    }
    const KnownFormat* chosen = nullptr;
    for (const KnownFormat& known : k_known_formats) {
        if (std::ranges::find(offered, known.dxgi) != offered.end()) {
            chosen = &known;
            break;
        }
    }
    if (chosen == nullptr) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "the headset offers no 8-bit colour format");
    }
    if (!chosen->srgb) {
        LOG_WARN("the headset offers no sRGB image format; the picture will look washed out")
            .tag("subsystem", "xr");
    }

    for (std::size_t eye = 0; eye < k_eyes; ++eye) {
        const XrSwapchainCreateInfo create{.type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                                           .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
                                           .format = chosen->dxgi,
                                           .sampleCount = 1,
                                           .width = sizes.at(eye).recommendedImageRectWidth,
                                           .height = sizes.at(eye).recommendedImageRectHeight,
                                           .faceCount = 1,
                                           .arraySize = 1,
                                           .mipCount = 1};
        if (core::VoidResult created =
                check(xrCreateSwapchain(session, &create, &swapchains.at(eye)), "xrCreateSwapchain");
            !created) {
            return created;
        }
        std::uint32_t image_count = 0;
        if (core::VoidResult counted =
                check(xrEnumerateSwapchainImages(swapchains.at(eye), 0, &image_count, nullptr),
                      "xrEnumerateSwapchainImages");
            !counted) {
            return counted;
        }
        std::vector<XrSwapchainImageD3D11KHR> textures(
            image_count, XrSwapchainImageD3D11KHR{.type = XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR});
        if (core::VoidResult listed =
                check(xrEnumerateSwapchainImages(
                          swapchains.at(eye), image_count, &image_count,
                          // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): OpenXR's way.
                          reinterpret_cast<XrSwapchainImageBaseHeader*>(textures.data())),
                      "xrEnumerateSwapchainImages");
            !listed) {
            return listed;
        }
        EyeImages& target = images.at(eye);
        target.width_px = create.width;
        target.height_px = create.height;
        target.format = chosen->format;
        for (const XrSwapchainImageD3D11KHR& texture : textures) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): a handle for the renderer.
            target.textures.push_back(reinterpret_cast<std::uintptr_t>(texture.texture));
        }
        if (eye == 0 && !textures.empty()) {
            D3D11_TEXTURE2D_DESC description{};
            textures.front().texture->GetDesc(&description);
            LOG_INFO("eye images: {}x{}, {} per eye, DXGI format {} asked, {} given", target.width_px,
                     target.height_px, textures.size(), chosen->dxgi, static_cast<int>(description.Format))
                .tag("subsystem", "xr");
        }
    }
    return {};
}

core::VoidResult Session::State::state_changed(XrSessionState state) {
    switch (state) {
    case XR_SESSION_STATE_READY: {
        const XrSessionBeginInfo begin{.type = XR_TYPE_SESSION_BEGIN_INFO,
                                       .primaryViewConfigurationType = k_view_configuration};
        if (core::VoidResult begun = check(xrBeginSession(session, &begin), "xrBeginSession"); !begun) {
            return begun;
        }
        running = true;
        LOG_INFO("the headset is in use").tag("subsystem", "xr");
        break;
    }
    case XR_SESSION_STATE_STOPPING:
        running = false;
        LOG_INFO("the headset is no longer in use").tag("subsystem", "xr");
        return check(xrEndSession(session), "xrEndSession");
    case XR_SESSION_STATE_EXITING:
    case XR_SESSION_STATE_LOSS_PENDING: over = true; break;
    default:                            break;
    }
    return {};
}

core::Result<Session> Session::make(const Graphics& graphics) {
    auto state = std::make_unique<State>();
    if (core::VoidResult opened = state->open_instance(); !opened) {
        return std::unexpected(opened.error());
    }
    if (core::VoidResult opened = state->open_session(graphics); !opened) {
        return std::unexpected(opened.error());
    }
    if (core::VoidResult made = state->make_images(); !made) {
        return std::unexpected(made.error());
    }
    LOG_INFO("VR: {} on {}", state->system_name, state->runtime_name).tag("subsystem", "xr");
    return Session(std::move(state));
}

Session::Session(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {
}
Session::Session(Session&& other) noexcept = default;
Session& Session::operator=(Session&& other) noexcept = default;
Session::~Session() = default;

const std::string& Session::runtime_name() const noexcept {
    return state_->runtime_name;
}

const std::string& Session::system_name() const noexcept {
    return state_->system_name;
}

const std::array<EyeImages, k_eyes>& Session::images() const noexcept {
    return state_->images;
}

double Session::frame_period_s() const noexcept {
    return state_->frame_period_s;
}

core::Result<bool> Session::poll() {
    State& state = *state_;
    for (;;) {
        XrEventDataBuffer event{.type = XR_TYPE_EVENT_DATA_BUFFER};
        const XrResult polled = xrPollEvent(state.instance, &event);
        if (polled == XR_EVENT_UNAVAILABLE) {
            break;
        }
        if (core::VoidResult taken = check(polled, "xrPollEvent"); !taken) {
            return std::unexpected(taken.error());
        }
        if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            state.over = true;
        } else if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            XrEventDataSessionStateChanged changed{};
            std::memcpy(&changed, &event, sizeof(changed));
            if (core::VoidResult handled = state.state_changed(changed.state); !handled) {
                return std::unexpected(handled.error());
            }
        }
    }
    return !state.over;
}

core::Result<Frame> Session::begin_frame() {
    State& state = *state_;
    Frame frame;
    if (!state.running) {
        return frame;
    }
    XrFrameState timing{.type = XR_TYPE_FRAME_STATE};
    if (core::VoidResult waited = check(xrWaitFrame(state.session, nullptr, &timing), "xrWaitFrame");
        !waited) {
        return std::unexpected(waited.error());
    }
    if (core::VoidResult begun = check(xrBeginFrame(state.session, nullptr), "xrBeginFrame"); !begun) {
        return std::unexpected(begun.error());
    }
    state.frame_begun = true;
    state.frame_period_s = static_cast<double>(timing.predictedDisplayPeriod) * k_nanoseconds;
    frame.display_time_ns = timing.predictedDisplayTime;
    if (timing.shouldRender == XR_FALSE) {
        return frame;
    }

    const XrViewLocateInfo locate{.type = XR_TYPE_VIEW_LOCATE_INFO,
                                  .viewConfigurationType = k_view_configuration,
                                  .displayTime = timing.predictedDisplayTime,
                                  .space = state.space};
    XrViewState located{.type = XR_TYPE_VIEW_STATE};
    state.views = {XrView{.type = XR_TYPE_VIEW}, XrView{.type = XR_TYPE_VIEW}};
    std::uint32_t count = 0;
    if (core::VoidResult found =
            check(xrLocateViews(state.session, &locate, &located,
                                static_cast<std::uint32_t>(state.views.size()), &count, state.views.data()),
                  "xrLocateViews");
        !found) {
        return std::unexpected(found.error());
    }
    constexpr XrViewStateFlags k_tracked =
        XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;
    if (count != k_eyes || (located.viewStateFlags & k_tracked) != k_tracked) {
        return frame; // not tracked yet: nothing to draw from
    }

    for (std::size_t eye = 0; eye < k_eyes; ++eye) {
        std::uint32_t image = 0;
        if (core::VoidResult acquired =
                check(xrAcquireSwapchainImage(state.swapchains.at(eye), nullptr, &image),
                      "xrAcquireSwapchainImage");
            !acquired) {
            return std::unexpected(acquired.error());
        }
        const XrSwapchainImageWaitInfo wait{.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
                                            .timeout = XR_INFINITE_DURATION};
        if (core::VoidResult ready =
                check(xrWaitSwapchainImage(state.swapchains.at(eye), &wait), "xrWaitSwapchainImage");
            !ready) {
            return std::unexpected(ready.error());
        }
        frame.eyes.at(eye) = Eye{.pose = to_tracked(state.views.at(eye).pose),
                                 .field_of_view = to_field_of_view(state.views.at(eye).fov),
                                 .image = image};
    }
    frame.head = frame.eyes.front().pose;
    frame.head.position_m = 0.5 * (frame.eyes.front().pose.position_m + frame.eyes.back().pose.position_m);
    frame.drawn = true;
    return frame;
}

core::VoidResult Session::end_frame(const Frame& frame) {
    State& state = *state_;
    if (!state.frame_begun) {
        return {};
    }
    state.frame_begun = false;
    XrFrameEndInfo end{.type = XR_TYPE_FRAME_END_INFO,
                       .displayTime = frame.display_time_ns,
                       .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE};
    if (!frame.drawn) {
        return check(xrEndFrame(state.session, &end), "xrEndFrame");
    }
    std::array<XrCompositionLayerProjectionView, k_eyes> views{};
    for (std::size_t eye = 0; eye < k_eyes; ++eye) {
        if (core::VoidResult released =
                check(xrReleaseSwapchainImage(state.swapchains.at(eye), nullptr), "xrReleaseSwapchainImage");
            !released) {
            return released;
        }
        XrCompositionLayerProjectionView& view = views.at(eye);
        view.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        view.pose = state.views.at(eye).pose;
        view.fov = state.views.at(eye).fov;
        view.subImage.swapchain = state.swapchains.at(eye);
        view.subImage.imageRect.extent = {.width = static_cast<std::int32_t>(state.images.at(eye).width_px),
                                          .height =
                                              static_cast<std::int32_t>(state.images.at(eye).height_px)};
    }
    const XrCompositionLayerProjection layer{.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
                                             .space = state.space,
                                             .viewCount = static_cast<std::uint32_t>(views.size()),
                                             .views = views.data()};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): OpenXR's way of listing layers.
    const auto* const listed = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer);
    const std::array<const XrCompositionLayerBaseHeader*, 1> layers{listed};
    end.layerCount = static_cast<std::uint32_t>(layers.size());
    end.layers = layers.data();
    return check(xrEndFrame(state.session, &end), "xrEndFrame");
}

} // namespace helios::xr
