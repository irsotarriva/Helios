add_library(helios_frames
  body_rotation.cpp
  frame_tree.cpp
)
add_library(helios::frames ALIAS helios_frames)

target_include_directories(helios_frames PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_frames
  PUBLIC  helios::ephemeris
  PRIVATE helios::compile_options
)
