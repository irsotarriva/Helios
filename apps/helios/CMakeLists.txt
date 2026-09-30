# The GPU-free part of the game (scenarios, the simulation host): always built and unit-tested.
add_library(helios_app_headless
  scenario.cpp
  simulation_host.cpp
)
add_library(helios::app_headless ALIAS helios_app_headless)
target_include_directories(helios_app_headless PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(helios_app_headless
  PUBLIC  helios::sim
  PRIVATE helios::compile_options
)

if(HELIOS_BUILD_CLIENT)
  add_executable(helios
    application.cpp
    main.cpp
    map_view_ui.cpp
    platform.cpp
    screenshot.cpp
  )
  target_link_libraries(helios PRIVATE
    helios::app_headless helios::render_gpu helios::imgui SDL3::SDL3-static helios::compile_options
  )
  target_compile_definitions(helios PRIVATE HELIOS_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
endif()
