add_library(helios_vessel
  assembly.cpp
  blueprint.cpp
  cockpit.cpp
  control_bus.cpp
  curve.cpp
  part_datasheet.cpp
  vessel_systems.cpp
)
add_library(helios::vessel ALIAS helios_vessel)

target_include_directories(helios_vessel PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_vessel
  PUBLIC  helios::time helios::math
  PRIVATE helios::compile_options
)
