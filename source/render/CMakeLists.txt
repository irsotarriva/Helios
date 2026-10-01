# GPU-free parts of the renderer (camera, meshes, snapshot → camera-space geometry). Unit-tested
# in every CI job; the bgfx renderer below is only built with HELIOS_BUILD_CLIENT.
add_library(helios_render_core
  camera.cpp
  mesh.cpp
  scene_geometry.cpp
)
add_library(helios::render_core ALIAS helios_render_core)

target_include_directories(helios_render_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_render_core
  PUBLIC  helios::sim
  PRIVATE helios::compile_options
)

if(HELIOS_BUILD_CLIENT)
  add_subdirectory(gpu)
endif()
