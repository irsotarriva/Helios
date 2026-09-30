include(Catch)

function(helios_add_unit_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LINKS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LINKS} Catch2::Catch2WithMain helios::compile_options)
  catch_discover_tests(${name})
endfunction()

helios_add_unit_test(test_core
  SOURCES unit/core/test_error.cpp unit/core/test_logging.cpp
  LINKS   helios::core)

helios_add_unit_test(test_time
  SOURCES unit/time/test_epoch.cpp
  LINKS   helios::time)
