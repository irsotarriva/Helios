add_library(helios_physics
  world.cpp
)
add_library(helios::physics ALIAS helios_physics)

target_include_directories(helios_physics PUBLIC ${PROJECT_SOURCE_DIR}/include)
# Rationale: Jolt is PRIVATE so that its compile definitions (and its instruction-set options,
# where the package exports them) stay inside this one library; world.hpp shows none of it.
target_link_libraries(helios_physics
  PUBLIC  helios::core helios::math
  PRIVATE Jolt::Jolt helios::compile_options
)
