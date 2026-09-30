add_executable(helios_ephemeris_probe ephemeris_probe/main.cpp)
target_link_libraries(helios_ephemeris_probe PRIVATE helios::frames helios::compile_options)

add_executable(helios_dynamics_scenarios dynamics_scenarios/main.cpp)
target_link_libraries(helios_dynamics_scenarios PRIVATE helios::dynamics helios::ephemeris helios::compile_options)
