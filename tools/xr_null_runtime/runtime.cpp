// A headset that is not there: the least of an OpenXR runtime that Helios' VR path needs
// (BRIEFING §12.2, D29), for checking that path on a machine without a headset or a simulator.
// It hands out Direct3D 11 images, keeps the head still, asks for 90 frames a second, and can
// write what each eye was sent to a file. It shows nothing and checks little: it is a test
// double, and passing here does not replace a run on a real runtime.
//
//   set XR_RUNTIME_JSON=<build>/tools/xr_null_runtime/helios_xr_null.json
//   set HELIOS_XR_NULL_DUMP=<prefix>        writes <prefix>_left.bmp and <prefix>_right.bmp ...
//   set HELIOS_XR_NULL_DUMP_FRAME=<n>       ... of the n-th frame (20 if not given)
//   set HELIOS_XR_NULL_YAW_DEG=<degrees>    the head turned to the left by so much
//   set HELIOS_XR_NULL_SIZE=<pixels>        the side of an eye's image (1024 if not given)

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <d3d11.h>
#include <windows.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <numbers>
#include <openxr/openxr.h>
#include <openxr/openxr_loader_negotiation.h>
#include <openxr/openxr_platform.h>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t k_eyes = 2;
constexpr std::uint32_t k_images = 3;
constexpr XrDuration k_period_ns = 11'111'111; // 90 Hz
constexpr float k_half_eye_distance_m = 0.032F;
constexpr float k_outward_rad = 0.8727F; // 50°
constexpr float k_inward_rad = 0.6981F;  // 40°
constexpr float k_vertical_rad = 0.7854F;
constexpr std::array<std::int64_t, 2> k_formats{DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_R8G8B8A8_UNORM};

struct Swapchain {
    std::vector<ID3D11Texture2D*> textures;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t next = 0;
    std::uint32_t last = 0; // the image released last: the one a frame shows
};

struct Runtime {
    bool instance = false;
    ID3D11Device* device = nullptr;
    std::deque<XrSessionState> events;
    std::vector<std::unique_ptr<Swapchain>> swapchains;
    std::chrono::steady_clock::time_point next_frame = std::chrono::steady_clock::now();
    int frames = 0;
};

Runtime g_runtime; // one application, one session

[[nodiscard]] std::optional<std::string> environment(const char* name) {
    std::array<char, 512> buffer{};
    const DWORD length = GetEnvironmentVariableA(name, buffer.data(), static_cast<DWORD>(buffer.size()));
    if (length == 0 || length >= buffer.size()) {
        return std::nullopt;
    }
    return std::string(buffer.data(), length);
}

[[nodiscard]] double number_from(const char* name, double fallback) {
    const std::optional<std::string> text = environment(name);
    double value = fallback;
    if (text.has_value()) {
        std::from_chars(text->data(), text->data() + text->size(), value);
    }
    return value;
}

template <typename Handle, typename Object>
[[nodiscard]] Handle to_handle(Object* object) {
    return reinterpret_cast<Handle>(object);
}

// The fill-or-count convention of OpenXR's enumerations.
template <typename Item, typename Source>
XrResult enumerate(const Source& source, std::uint32_t capacity, std::uint32_t* count, Item* items) {
    *count = static_cast<std::uint32_t>(source.size());
    if (capacity == 0) {
        return XR_SUCCESS;
    }
    if (capacity < source.size()) {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }
    for (std::size_t index = 0; index < source.size(); ++index) {
        items[index] = source[index];
    }
    return XR_SUCCESS;
}

// A 32-bit picture, top row first, from a texture of the application's device.
void write_picture(ID3D11Texture2D* texture, const std::string& path) {
    D3D11_TEXTURE2D_DESC description{};
    texture->GetDesc(&description);
    D3D11_TEXTURE2D_DESC staging_description = description;
    staging_description.Usage = D3D11_USAGE_STAGING;
    staging_description.BindFlags = 0;
    staging_description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    staging_description.MiscFlags = 0;
    staging_description.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    ID3D11Texture2D* staging = nullptr;
    ID3D11DeviceContext* context = nullptr;
    g_runtime.device->GetImmediateContext(&context);
    if (FAILED(g_runtime.device->CreateTexture2D(&staging_description, nullptr, &staging))) {
        context->Release();
        return;
    }
    context->CopyResource(staging, texture);
    D3D11_MAPPED_SUBRESOURCE mapped{};
    if (SUCCEEDED(context->Map(staging, 0, D3D11_MAP_READ, 0, &mapped))) {
        BITMAPFILEHEADER file{};
        BITMAPINFOHEADER info{};
        const std::uint32_t bytes = description.Width * description.Height * 4;
        file.bfType = 0x4D42; // "BM"
        file.bfOffBits = sizeof(file) + sizeof(info);
        file.bfSize = file.bfOffBits + bytes;
        info.biSize = sizeof(info);
        info.biWidth = static_cast<LONG>(description.Width);
        info.biHeight = -static_cast<LONG>(description.Height); // top row first
        info.biPlanes = 1;
        info.biBitCount = 32;
        std::vector<std::uint8_t> pixels(bytes);
        for (std::uint32_t row = 0; row < description.Height; ++row) {
            const auto* source = static_cast<const std::uint8_t*>(mapped.pData) + row * mapped.RowPitch;
            std::uint8_t* target = pixels.data() + static_cast<std::size_t>(row) * description.Width * 4;
            for (std::uint32_t column = 0; column < description.Width; ++column) {
                target[column * 4 + 0] = source[column * 4 + 2]; // a .bmp is blue first
                target[column * 4 + 1] = source[column * 4 + 1];
                target[column * 4 + 2] = source[column * 4 + 0];
                target[column * 4 + 3] = 255;
            }
        }
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(&file), sizeof(file));
        out.write(reinterpret_cast<const char*>(&info), sizeof(info));
        out.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
        context->Unmap(staging, 0);
    }
    staging->Release();
    context->Release();
}

// --- The functions of OpenXR that Helios calls --------------------------------------------------

XrResult XRAPI_CALL enumerate_extensions(const char* /*layer*/, std::uint32_t capacity, std::uint32_t* count,
                                         XrExtensionProperties* properties) {
    XrExtensionProperties direct3d{.type = XR_TYPE_EXTENSION_PROPERTIES};
    std::strncpy(direct3d.extensionName, XR_KHR_D3D11_ENABLE_EXTENSION_NAME,
                 sizeof(direct3d.extensionName) - 1);
    direct3d.extensionVersion = XR_KHR_D3D11_enable_SPEC_VERSION;
    return enumerate(std::array{direct3d}, capacity, count, properties);
}

XrResult XRAPI_CALL create_instance(const XrInstanceCreateInfo* /*info*/, XrInstance* instance) {
    g_runtime.instance = true;
    *instance = to_handle<XrInstance>(&g_runtime);
    return XR_SUCCESS;
}

XrResult XRAPI_CALL destroy_instance(XrInstance /*instance*/) {
    g_runtime.instance = false;
    return XR_SUCCESS;
}

XrResult XRAPI_CALL instance_properties(XrInstance /*instance*/, XrInstanceProperties* properties) {
    properties->runtimeVersion = XR_MAKE_VERSION(0, 1, 0);
    std::strncpy(properties->runtimeName, "Helios null headset", sizeof(properties->runtimeName) - 1);
    return XR_SUCCESS;
}

XrResult XRAPI_CALL get_system(XrInstance /*instance*/, const XrSystemGetInfo* info, XrSystemId* system) {
    if (info->formFactor != XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY) {
        return XR_ERROR_FORM_FACTOR_UNSUPPORTED;
    }
    *system = 1;
    return XR_SUCCESS;
}

XrResult XRAPI_CALL system_properties(XrInstance /*instance*/, XrSystemId /*system*/,
                                      XrSystemProperties* properties) {
    properties->systemId = 1;
    std::strncpy(properties->systemName, "a headset that is not there", sizeof(properties->systemName) - 1);
    properties->graphicsProperties = {
        .maxSwapchainImageHeight = 4096, .maxSwapchainImageWidth = 4096, .maxLayerCount = 1};
    properties->trackingProperties = {.orientationTracking = XR_TRUE, .positionTracking = XR_TRUE};
    return XR_SUCCESS;
}

XrResult XRAPI_CALL graphics_requirements(XrInstance /*instance*/, XrSystemId /*system*/,
                                          XrGraphicsRequirementsD3D11KHR* requirements) {
    requirements->adapterLuid = {};
    requirements->minFeatureLevel = D3D_FEATURE_LEVEL_11_0;
    return XR_SUCCESS;
}

XrResult XRAPI_CALL create_session(XrInstance /*instance*/, const XrSessionCreateInfo* info,
                                   XrSession* session) {
    const auto* binding = static_cast<const XrGraphicsBindingD3D11KHR*>(info->next);
    if (binding == nullptr || binding->type != XR_TYPE_GRAPHICS_BINDING_D3D11_KHR
        || binding->device == nullptr) {
        return XR_ERROR_GRAPHICS_DEVICE_INVALID;
    }
    g_runtime.device = binding->device;
    g_runtime.device->AddRef();
    g_runtime.events = {XR_SESSION_STATE_IDLE, XR_SESSION_STATE_READY};
    g_runtime.frames = 0;
    *session = to_handle<XrSession>(&g_runtime);
    return XR_SUCCESS;
}

XrResult XRAPI_CALL destroy_swapchain(XrSwapchain handle) {
    std::erase_if(g_runtime.swapchains, [&](const std::unique_ptr<Swapchain>& swapchain) {
        if (to_handle<XrSwapchain>(swapchain.get()) != handle) {
            return false;
        }
        for (ID3D11Texture2D* texture : swapchain->textures) {
            texture->Release();
        }
        return true;
    });
    return XR_SUCCESS;
}

XrResult XRAPI_CALL destroy_session(XrSession /*session*/) {
    while (!g_runtime.swapchains.empty()) {
        destroy_swapchain(to_handle<XrSwapchain>(g_runtime.swapchains.back().get()));
    }
    if (g_runtime.device != nullptr) {
        g_runtime.device->Release();
        g_runtime.device = nullptr;
    }
    return XR_SUCCESS;
}

XrResult XRAPI_CALL create_space(XrSession /*session*/, const XrReferenceSpaceCreateInfo* /*info*/,
                                 XrSpace* space) {
    *space = to_handle<XrSpace>(&g_runtime);
    return XR_SUCCESS;
}

XrResult XRAPI_CALL destroy_space(XrSpace /*space*/) {
    return XR_SUCCESS;
}

XrResult XRAPI_CALL view_configuration_views(XrInstance /*instance*/, XrSystemId /*system*/,
                                             XrViewConfigurationType type, std::uint32_t capacity,
                                             std::uint32_t* count, XrViewConfigurationView* views) {
    if (type != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) {
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }
    const auto side = static_cast<std::uint32_t>(number_from("HELIOS_XR_NULL_SIZE", 1024.0));
    const XrViewConfigurationView view{.type = XR_TYPE_VIEW_CONFIGURATION_VIEW,
                                       .recommendedImageRectWidth = side,
                                       .maxImageRectWidth = 4096,
                                       .recommendedImageRectHeight = side,
                                       .maxImageRectHeight = 4096,
                                       .recommendedSwapchainSampleCount = 1,
                                       .maxSwapchainSampleCount = 1};
    return enumerate(std::array{view, view}, capacity, count, views);
}

XrResult XRAPI_CALL swapchain_formats(XrSession /*session*/, std::uint32_t capacity, std::uint32_t* count,
                                      std::int64_t* formats) {
    return enumerate(k_formats, capacity, count, formats);
}

XrResult XRAPI_CALL create_swapchain(XrSession /*session*/, const XrSwapchainCreateInfo* info,
                                     XrSwapchain* handle) {
    if (info->format != DXGI_FORMAT_R8G8B8A8_UNORM_SRGB && info->format != DXGI_FORMAT_R8G8B8A8_UNORM) {
        return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    }
    auto swapchain = std::make_unique<Swapchain>();
    swapchain->width = info->width;
    swapchain->height = info->height;
    // Typeless, as real runtimes make them: the application chooses how to write into it.
    D3D11_TEXTURE2D_DESC description{};
    description.Width = info->width;
    description.Height = info->height;
    description.MipLevels = 1;
    description.ArraySize = 1;
    description.Format = DXGI_FORMAT_R8G8B8A8_TYPELESS;
    description.SampleDesc.Count = 1;
    description.Usage = D3D11_USAGE_DEFAULT;
    description.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    for (std::uint32_t image = 0; image < k_images; ++image) {
        ID3D11Texture2D* texture = nullptr;
        if (FAILED(g_runtime.device->CreateTexture2D(&description, nullptr, &texture))) {
            return XR_ERROR_RUNTIME_FAILURE;
        }
        swapchain->textures.push_back(texture);
    }
    *handle = to_handle<XrSwapchain>(swapchain.get());
    g_runtime.swapchains.push_back(std::move(swapchain));
    return XR_SUCCESS;
}

XrResult XRAPI_CALL swapchain_images(XrSwapchain handle, std::uint32_t capacity, std::uint32_t* count,
                                     XrSwapchainImageBaseHeader* images) {
    const auto* swapchain = reinterpret_cast<const Swapchain*>(handle);
    *count = static_cast<std::uint32_t>(swapchain->textures.size());
    if (capacity == 0) {
        return XR_SUCCESS;
    }
    if (capacity < swapchain->textures.size()) {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }
    auto* textures = reinterpret_cast<XrSwapchainImageD3D11KHR*>(images);
    for (std::size_t image = 0; image < swapchain->textures.size(); ++image) {
        textures[image].texture = swapchain->textures[image];
    }
    return XR_SUCCESS;
}

XrResult XRAPI_CALL poll_event(XrInstance /*instance*/, XrEventDataBuffer* buffer) {
    if (g_runtime.events.empty()) {
        return XR_EVENT_UNAVAILABLE;
    }
    XrEventDataSessionStateChanged changed{.type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED};
    changed.session = to_handle<XrSession>(&g_runtime);
    changed.state = g_runtime.events.front();
    g_runtime.events.pop_front();
    std::memcpy(buffer, &changed, sizeof(changed));
    return XR_SUCCESS;
}

XrResult XRAPI_CALL begin_session(XrSession /*session*/, const XrSessionBeginInfo* /*info*/) {
    g_runtime.events.insert(g_runtime.events.end(), {XR_SESSION_STATE_SYNCHRONIZED, XR_SESSION_STATE_VISIBLE,
                                                     XR_SESSION_STATE_FOCUSED});
    g_runtime.next_frame = std::chrono::steady_clock::now();
    return XR_SUCCESS;
}

XrResult XRAPI_CALL end_session(XrSession /*session*/) {
    return XR_SUCCESS;
}

XrResult XRAPI_CALL wait_frame(XrSession /*session*/, const XrFrameWaitInfo* /*info*/, XrFrameState* state) {
    // The pace of a 90 Hz headset: the application is held until the next frame is due.
    const auto now = std::chrono::steady_clock::now();
    g_runtime.next_frame = std::max(g_runtime.next_frame + std::chrono::nanoseconds(k_period_ns), now);
    std::this_thread::sleep_until(g_runtime.next_frame);
    state->predictedDisplayTime =
        std::chrono::duration_cast<std::chrono::nanoseconds>(g_runtime.next_frame.time_since_epoch()).count()
        + 2 * k_period_ns;
    state->predictedDisplayPeriod = k_period_ns;
    state->shouldRender = XR_TRUE;
    return XR_SUCCESS;
}

XrResult XRAPI_CALL begin_frame(XrSession /*session*/, const XrFrameBeginInfo* /*info*/) {
    return XR_SUCCESS;
}

XrResult XRAPI_CALL locate_views(XrSession /*session*/, const XrViewLocateInfo* /*info*/, XrViewState* state,
                                 std::uint32_t capacity, std::uint32_t* count, XrView* views) {
    *count = k_eyes;
    if (capacity == 0) {
        return XR_SUCCESS;
    }
    if (capacity < k_eyes) {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }
    state->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT
                            | XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT;
    // The head at the origin, turned about the vertical (+y); a turn to the left is positive.
    const auto yaw_rad =
        static_cast<float>(number_from("HELIOS_XR_NULL_YAW_DEG", 0.0) * std::numbers::pi / 180.0);
    const XrQuaternionf turned{
        .x = 0.0F, .y = std::sin(0.5F * yaw_rad), .z = 0.0F, .w = std::cos(0.5F * yaw_rad)};
    const XrVector3f right{.x = std::cos(yaw_rad), .y = 0.0F, .z = -std::sin(yaw_rad)};
    for (std::size_t eye = 0; eye < k_eyes; ++eye) {
        const float side = eye == 0 ? -1.0F : 1.0F;
        views[eye].pose.orientation = turned;
        views[eye].pose.position = {.x = side * k_half_eye_distance_m * right.x,
                                    .y = 0.0F,
                                    .z = side * k_half_eye_distance_m * right.z};
        views[eye].fov = {.angleLeft = eye == 0 ? -k_outward_rad : -k_inward_rad,
                          .angleRight = eye == 0 ? k_inward_rad : k_outward_rad,
                          .angleUp = k_vertical_rad,
                          .angleDown = -k_vertical_rad};
    }
    return XR_SUCCESS;
}

XrResult XRAPI_CALL acquire_image(XrSwapchain handle, const XrSwapchainImageAcquireInfo* /*info*/,
                                  std::uint32_t* index) {
    auto* swapchain = reinterpret_cast<Swapchain*>(handle);
    *index = swapchain->next;
    swapchain->last = swapchain->next;
    swapchain->next = (swapchain->next + 1) % static_cast<std::uint32_t>(swapchain->textures.size());
    return XR_SUCCESS;
}

XrResult XRAPI_CALL wait_image(XrSwapchain /*handle*/, const XrSwapchainImageWaitInfo* /*info*/) {
    return XR_SUCCESS;
}

XrResult XRAPI_CALL release_image(XrSwapchain /*handle*/, const XrSwapchainImageReleaseInfo* /*info*/) {
    return XR_SUCCESS;
}

XrResult XRAPI_CALL end_frame(XrSession /*session*/, const XrFrameEndInfo* info) {
    ++g_runtime.frames;
    const std::optional<std::string> prefix = environment("HELIOS_XR_NULL_DUMP");
    if (!prefix.has_value() || info->layerCount == 0
        || g_runtime.frames != static_cast<int>(number_from("HELIOS_XR_NULL_DUMP_FRAME", 20.0))) {
        return XR_SUCCESS;
    }
    const auto* layer = reinterpret_cast<const XrCompositionLayerProjection*>(info->layers[0]);
    if (layer->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION || layer->viewCount != k_eyes) {
        return XR_ERROR_LAYER_INVALID;
    }
    constexpr std::array<std::string_view, k_eyes> k_names{"_left.bmp", "_right.bmp"};
    for (std::size_t eye = 0; eye < k_eyes; ++eye) {
        const auto* swapchain = reinterpret_cast<const Swapchain*>(layer->views[eye].subImage.swapchain);
        write_picture(swapchain->textures[swapchain->last], *prefix + std::string(k_names[eye]));
    }
    return XR_SUCCESS;
}

struct Entry {
    std::string_view name;
    PFN_xrVoidFunction function;
};

template <typename Function>
[[nodiscard]] Entry entry(std::string_view name, Function function) {
    return {name, reinterpret_cast<PFN_xrVoidFunction>(function)};
}

XrResult XRAPI_CALL get_instance_proc_addr(XrInstance /*instance*/, const char* name,
                                           PFN_xrVoidFunction* function);

const std::array k_functions{
    entry("xrGetInstanceProcAddr", get_instance_proc_addr),
    entry("xrEnumerateInstanceExtensionProperties", enumerate_extensions),
    entry("xrCreateInstance", create_instance),
    entry("xrDestroyInstance", destroy_instance),
    entry("xrGetInstanceProperties", instance_properties),
    entry("xrGetSystem", get_system),
    entry("xrGetSystemProperties", system_properties),
    entry("xrGetD3D11GraphicsRequirementsKHR", graphics_requirements),
    entry("xrCreateSession", create_session),
    entry("xrDestroySession", destroy_session),
    entry("xrCreateReferenceSpace", create_space),
    entry("xrDestroySpace", destroy_space),
    entry("xrEnumerateViewConfigurationViews", view_configuration_views),
    entry("xrEnumerateSwapchainFormats", swapchain_formats),
    entry("xrCreateSwapchain", create_swapchain),
    entry("xrDestroySwapchain", destroy_swapchain),
    entry("xrEnumerateSwapchainImages", swapchain_images),
    entry("xrPollEvent", poll_event),
    entry("xrBeginSession", begin_session),
    entry("xrEndSession", end_session),
    entry("xrWaitFrame", wait_frame),
    entry("xrBeginFrame", begin_frame),
    entry("xrLocateViews", locate_views),
    entry("xrAcquireSwapchainImage", acquire_image),
    entry("xrWaitSwapchainImage", wait_image),
    entry("xrReleaseSwapchainImage", release_image),
    entry("xrEndFrame", end_frame),
};

XrResult XRAPI_CALL get_instance_proc_addr(XrInstance /*instance*/, const char* name,
                                           PFN_xrVoidFunction* function) {
    for (const Entry& known : k_functions) {
        if (known.name == name) {
            *function = known.function;
            return XR_SUCCESS;
        }
    }
    *function = nullptr;
    return XR_ERROR_FUNCTION_UNSUPPORTED;
}

} // namespace

// What the OpenXR loader looks for in a runtime's library.
extern "C" __declspec(dllexport) XrResult XRAPI_CALL
xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo* loader, XrNegotiateRuntimeRequest* request) {
    if (loader == nullptr || request == nullptr
        || loader->minInterfaceVersion > XR_CURRENT_LOADER_RUNTIME_VERSION
        || loader->maxInterfaceVersion < XR_CURRENT_LOADER_RUNTIME_VERSION) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    request->runtimeInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    request->runtimeApiVersion = XR_CURRENT_API_VERSION;
    request->getInstanceProcAddr = get_instance_proc_addr;
    return XR_SUCCESS;
}
