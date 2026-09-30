include(GoogleTest)

function(helios_add_unit_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LINKS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LINKS} GTest::gtest_main helios::compile_options)
  gtest_discover_tests(${name})
endfunction()

helios_add_unit_test(test_core
  SOURCES unit/core/test_error.cpp unit/core/test_logging.cpp unit/core/test_parse.cpp
  LINKS   helios::core)

helios_add_unit_test(test_time
  SOURCES unit/time/test_epoch.cpp
  LINKS   helios::time)

helios_add_unit_test(test_math
  SOURCES unit/math/test_vector3.cpp
  LINKS   helios::math)

helios_add_unit_test(test_orbital
  SOURCES unit/orbital/test_kepler.cpp
  LINKS   helios::orbital)

helios_add_unit_test(test_ephemeris
  SOURCES unit/ephemeris/test_keplerian_ephemeris.cpp
          unit/ephemeris/test_chebyshev_ephemeris.cpp
          unit/ephemeris/test_ephemeris_file.cpp
  LINKS   helios::ephemeris)

helios_add_unit_test(test_frames
  SOURCES unit/frames/test_frame_tree.cpp unit/frames/test_de421_golden.cpp
  LINKS   helios::frames)
target_compile_definitions(test_frames PRIVATE HELIOS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
