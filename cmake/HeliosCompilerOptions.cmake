# Project-wide compile settings are carried by an INTERFACE target rather than
# add_compile_options(), so they never leak into third-party code pulled in with
# FetchContent.
add_library(helios_compile_options INTERFACE)
add_library(helios::compile_options ALIAS helios_compile_options)

target_compile_features(helios_compile_options INTERFACE cxx_std_23)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(helios_compile_options INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wold-style-cast -Wnon-virtual-dtor -Woverloaded-virtual -Wnull-dereference
    # Rationale: aggregates with default member initializers are meant to be built with
    # designated initializers that name only the fields that differ from the defaults.
    -Wno-missing-field-initializers)
  if(HELIOS_WARNINGS_AS_ERRORS)
    target_compile_options(helios_compile_options INTERFACE -Werror)
  endif()
  if(HELIOS_ENABLE_SANITIZERS)
    target_compile_options(helios_compile_options INTERFACE
      -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    target_link_options(helios_compile_options INTERFACE -fsanitize=address,undefined)
  endif()
elseif(MSVC)
  target_compile_options(helios_compile_options INTERFACE /W4 /permissive- /utf-8)
  if(HELIOS_WARNINGS_AS_ERRORS)
    target_compile_options(helios_compile_options INTERFACE /WX)
  endif()
endif()

if(WIN32)
  target_compile_definitions(helios_compile_options INTERFACE PLATFORM_WINDOWS)
elseif(APPLE)
  target_compile_definitions(helios_compile_options INTERFACE PLATFORM_MACOS)
else()
  target_compile_definitions(helios_compile_options INTERFACE PLATFORM_LINUX)
endif()
