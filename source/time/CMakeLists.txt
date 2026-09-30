add_library(helios_time
  epoch.cpp
)
add_library(helios::time ALIAS helios_time)

target_include_directories(helios_time PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_time
  PUBLIC  helios::core
  PRIVATE helios::compile_options
)
