include(FetchContent)

# LumenLog is not in the vcpkg registry, so it is pinned to a commit here.
# Bump HELIOS_LUMEN_COMMIT deliberately; never track a moving branch.
# Rationale: a plain variable, not a CACHE entry, so bumping it takes effect in existing
# build directories. To try a local LumenLog checkout, pass -DFETCHCONTENT_SOURCE_DIR_LUMEN=<path>.
set(HELIOS_LUMEN_COMMIT "28e1d440cb6663d8724c43532081823a4cbbbd33")

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
