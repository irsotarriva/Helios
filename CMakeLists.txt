cmake_minimum_required(VERSION 3.28)
project(helios VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Rationale: no C++20 modules yet. Scanning needs clang-scan-deps and injects @modmap
# flags that clang-tidy cannot read. Set before FetchContent so it applies to deps too.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

option(HELIOS_BUILD_TESTS "Build the unit tests" ON)
option(HELIOS_ENABLE_SANITIZERS "Enable ASan + UBSan (GCC/Clang)" OFF)
option(HELIOS_BUILD_TOOLS "Build developer tools (ephemeris probe)" ON)
option(HELIOS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(HELIOS_BUILD_CLIENT "Build the game client (bgfx renderer, SDL3 window, Dear ImGui)" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(HeliosCompilerOptions)
include(HeliosDependencies)
if(HELIOS_BUILD_CLIENT)
  include(HeliosClientDependencies)
endif()

add_subdirectory(source)

add_subdirectory(apps/helios)

if(HELIOS_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(HELIOS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
