add_library(helios_orbital
  conic.cpp
  kepler.cpp
)
add_library(helios::orbital ALIAS helios_orbital)

target_include_directories(helios_orbital PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_orbital
  PUBLIC  helios::core helios::math
  PRIVATE helios::compile_options
)
