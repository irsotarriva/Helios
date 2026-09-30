add_library(helios_dynamics
  encke_propagator.cpp
  events.cpp
  gravity_model.cpp
  trajectory.cpp
)
add_library(helios::dynamics ALIAS helios_dynamics)

target_include_directories(helios_dynamics PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_dynamics
  PUBLIC  helios::bodies
  PRIVATE helios::compile_options
)
