# LumenLog (logging) is a git submodule at extern/LumenLog, pinned to a commit by the superproject.
# Clone with `git clone --recursive`, or run `git submodule update --init --recursive` after a
# plain clone. To bump it: `git -C extern/LumenLog fetch && git -C extern/LumenLog checkout <commit>`,
# then commit the new submodule pointer.
set(HELIOS_LUMEN_DIR "${PROJECT_SOURCE_DIR}/extern/LumenLog")
if(NOT EXISTS "${HELIOS_LUMEN_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "LumenLog is missing from extern/LumenLog: the git submodules were not checked out.\n"
    "Run:  git submodule update --init --recursive\n"
    "(or clone with:  git clone --recursive https://github.com/irsotarriva/Helios.git)")
endif()

# Lumen's tests, examples and sanitizers already default to OFF when it is a subproject.
set(LUMEN_ENABLE_DASHBOARD OFF CACHE BOOL "" FORCE)
set(LUMEN_ENABLE_PYTHON OFF CACHE BOOL "" FORCE)
add_subdirectory("${HELIOS_LUMEN_DIR}" "${PROJECT_BINARY_DIR}/_deps/lumen-build" SYSTEM)

# Jolt Physics (rigid bodies in the physics bubble, BRIEFING D23), from vcpkg. Used only by
# source/physics.
find_package(Jolt CONFIG REQUIRED)

if(HELIOS_BUILD_TESTS)
  find_package(GTest CONFIG REQUIRED)
endif()
