add_library(helios_sim
  scene_snapshot.cpp
  simulation.cpp
  time_warp.cpp
)
add_library(helios::sim ALIAS helios_sim)

target_include_directories(helios_sim PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_sim
  PUBLIC  helios::dynamics
  PRIVATE helios::compile_options
)
