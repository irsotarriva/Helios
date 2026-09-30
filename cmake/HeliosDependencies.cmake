include(FetchContent)

# LumenLog is not in the vcpkg registry, so it is pinned to a commit here.
# Bump HELIOS_LUMEN_COMMIT deliberately; never track a moving branch.
set(HELIOS_LUMEN_COMMIT "d3ae5d5fcedb813a50481cb53c81eb379e353d9f" CACHE STRING "LumenLog commit to build against")

set(LUMEN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LUMEN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LUMEN_ENABLE_DASHBOARD OFF CACHE BOOL "" FORCE)
set(LUMEN_ENABLE_PYTHON OFF CACHE BOOL "" FORCE)
# Rationale: Lumen's reflection probe reads ${CMAKE_SOURCE_DIR}/cmake/try_reflection.cpp,
# which resolves to *our* source dir when Lumen is a subproject. Disable until fixed upstream.
set(LUMEN_ENABLE_REFLECTION "OFF" CACHE STRING "" FORCE)

FetchContent_Declare(lumen
  GIT_REPOSITORY https://github.com/irsotarriva/LumenLog.git
  GIT_TAG        ${HELIOS_LUMEN_COMMIT}
  SYSTEM
)
FetchContent_MakeAvailable(lumen)

if(HELIOS_BUILD_TESTS)
  find_package(Catch2 3 CONFIG REQUIRED)
endif()
