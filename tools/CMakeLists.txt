add_executable(helios_ephemeris_probe ephemeris_probe/main.cpp)
target_link_libraries(helios_ephemeris_probe PRIVATE helios::frames helios::compile_options)
