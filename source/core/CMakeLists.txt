add_library(helios_core
  error.cpp
  logging.cpp
  parse.cpp
)
add_library(helios::core ALIAS helios_core)

target_include_directories(helios_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(helios_core PRIVATE HELIOS_VERSION="${PROJECT_VERSION}")
target_link_libraries(helios_core
  PUBLIC  lumen::lumen
  PRIVATE helios::compile_options
)
