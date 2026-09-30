include(FetchContent)

# LumenLog is not in the vcpkg registry, so it is pinned to a commit here.
# Bump HELIOS_LUMEN_COMMIT deliberately; never track a moving branch.
set(HELIOS_LUMEN_COMMIT "7858fbbbe81215ed5b33de7c34f2978ea1f0db2d" CACHE STRING "LumenLog commit to build against")

# Lumen's tests, examples and sanitizers already default to OFF when it is a subproject.
set(LUMEN_ENABLE_DASHBOARD OFF CACHE BOOL "" FORCE)
set(LUMEN_ENABLE_PYTHON OFF CACHE BOOL "" FORCE)

FetchContent_Declare(lumen
  GIT_REPOSITORY https://github.com/irsotarriva/LumenLog.git
  GIT_TAG        ${HELIOS_LUMEN_COMMIT}
  SYSTEM
)
FetchContent_MakeAvailable(lumen)

if(HELIOS_BUILD_TESTS)
  find_package(GTest CONFIG REQUIRED)
endif()
