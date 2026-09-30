add_library(helios_ephemeris
  chebyshev_ephemeris.cpp
  ephemeris_file.cpp
  keplerian_ephemeris.cpp
)
add_library(helios::ephemeris ALIAS helios_ephemeris)

target_include_directories(helios_ephemeris PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(helios_ephemeris
  PUBLIC  helios::orbital helios::time
  PRIVATE helios::compile_options
)
