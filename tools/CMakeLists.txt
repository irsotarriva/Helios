add_executable(helios_ephemeris_probe ephemeris_probe/main.cpp)
target_link_libraries(helios_ephemeris_probe PRIVATE helios::frames helios::compile_options)

add_executable(helios_dynamics_scenarios dynamics_scenarios/main.cpp)
target_link_libraries(helios_dynamics_scenarios PRIVATE helios::dynamics helios::ephemeris helios::compile_options)

# A stand-in for a headset: an OpenXR runtime that hands out Direct3D 11 images and shows
# nothing (xr_null_runtime/runtime.cpp), for checking the VR path where there is no headset.
if(HELIOS_BUILD_CLIENT AND HELIOS_BUILD_XR AND WIN32)
  add_library(helios_xr_null MODULE xr_null_runtime/runtime.cpp)
  target_link_libraries(helios_xr_null PRIVATE OpenXR::headers d3d11)
  target_compile_definitions(helios_xr_null PRIVATE _CRT_SECURE_NO_WARNINGS)
  # The manifest the OpenXR loader is pointed at with XR_RUNTIME_JSON.
  file(GENERATE OUTPUT $<TARGET_FILE_DIR:helios_xr_null>/helios_xr_null.json CONTENT
"{
  \"file_format_version\": \"1.0.0\",
  \"runtime\": { \"name\": \"Helios null headset\", \"library_path\": \"./$<TARGET_FILE_NAME:helios_xr_null>\" }
}
")
endif()
