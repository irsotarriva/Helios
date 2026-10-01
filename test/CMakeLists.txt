include(GoogleTest)

function(helios_add_unit_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LINKS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LINKS} GTest::gtest_main helios::compile_options)
  # Rationale: by default the tests are listed by running the executable right after it is
  # linked, with a 5 s timeout. In a parallel build on a loaded Windows runner several test
  # executables link at once while their GoogleTest DLLs are still being copied next to them,
  # and the listing timed out intermittently, failing the build. Listing when ctest starts
  # keeps it out of the build, and the timeout leaves room for a slow machine.
  gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 60)
endfunction()

helios_add_unit_test(test_core
  SOURCES unit/core/test_error.cpp unit/core/test_logging.cpp unit/core/test_parse.cpp
          unit/core/test_toml.cpp
  LINKS   helios::core)

helios_add_unit_test(test_time
  SOURCES unit/time/test_epoch.cpp
  LINKS   helios::time)

helios_add_unit_test(test_math
  SOURCES unit/math/test_vector3.cpp unit/math/test_quaternion.cpp
  LINKS   helios::math)

helios_add_unit_test(test_orbital
  SOURCES unit/orbital/test_kepler.cpp unit/orbital/test_conic.cpp
  LINKS   helios::orbital)

helios_add_unit_test(test_ephemeris
  SOURCES unit/ephemeris/test_keplerian_ephemeris.cpp
          unit/ephemeris/test_chebyshev_ephemeris.cpp
          unit/ephemeris/test_ephemeris_file.cpp
  LINKS   helios::ephemeris)

helios_add_unit_test(test_frames
  SOURCES unit/frames/test_frame_tree.cpp unit/frames/test_de440_golden.cpp unit/frames/test_body_rotation.cpp
  LINKS   helios::frames)

helios_add_unit_test(test_bodies
  SOURCES unit/bodies/test_body_catalog.cpp
  LINKS   helios::bodies helios::ephemeris)

helios_add_unit_test(test_dynamics
  SOURCES unit/dynamics/test_gravity_model.cpp unit/dynamics/test_encke_propagator.cpp
          unit/dynamics/test_events.cpp
  LINKS   helios::dynamics)

helios_add_unit_test(test_vessel
  SOURCES unit/vessel/test_part_datasheet.cpp unit/vessel/test_assembly.cpp
          unit/vessel/test_control_bus.cpp unit/vessel/test_vessel_systems.cpp
          unit/vessel/test_cockpit.cpp
  LINKS   helios::vessel)

helios_add_unit_test(test_physics
  SOURCES unit/physics/test_world.cpp
  LINKS   helios::physics)

helios_add_unit_test(test_sim
  SOURCES unit/sim/test_time_warp.cpp unit/sim/test_simulation.cpp unit/sim/test_solar_system.cpp
          unit/sim/test_flight.cpp unit/sim/test_bubble.cpp
          unit/sim/test_pilot.cpp
  LINKS   helios::sim)

foreach(test_target test_frames test_bodies test_dynamics test_vessel test_sim)
  target_compile_definitions(${test_target} PRIVATE
    HELIOS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
    HELIOS_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
endforeach()

helios_add_unit_test(test_render_core
  SOURCES unit/render/test_camera.cpp unit/render/test_scene_geometry.cpp
          unit/render/test_flight_geometry.cpp
  LINKS   helios::render_core)

helios_add_unit_test(test_app
  SOURCES unit/app/test_scenario.cpp unit/app/test_flight_controls.cpp
          unit/app/test_pilot_controls.cpp
  LINKS   helios::app_headless)
target_compile_definitions(test_app PRIVATE HELIOS_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
