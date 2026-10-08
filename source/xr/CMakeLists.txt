# VR: an OpenXR session for the client (BRIEFING 12, D29). Only built with HELIOS_BUILD_CLIENT
# and HELIOS_BUILD_XR. No OpenXR type appears in a header under include/.
add_library(helios_xr
  session.cpp
)
add_library(helios::xr ALIAS helios_xr)
target_include_directories(helios_xr PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_xr
  PUBLIC  helios::render_core
  PRIVATE OpenXR::openxr_loader OpenXR::headers helios::compile_options
)
