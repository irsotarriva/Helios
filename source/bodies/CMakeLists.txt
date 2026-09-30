add_library(helios_bodies
  body_catalog.cpp
)
add_library(helios::bodies ALIAS helios_bodies)

target_include_directories(helios_bodies PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_bodies
  PUBLIC  helios::frames
  PRIVATE helios::compile_options
)
