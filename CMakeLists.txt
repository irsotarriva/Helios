cmake_minimum_required(VERSION 3.28)
project(helios VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(HELIOS_BUILD_TESTS "Build the unit tests" ON)
option(HELIOS_ENABLE_SANITIZERS "Enable ASan + UBSan (GCC/Clang)" OFF)
option(HELIOS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(HeliosCompilerOptions)
include(HeliosDependencies)

add_subdirectory(source)

if(HELIOS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
