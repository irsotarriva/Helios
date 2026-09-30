# Header-only for now; an INTERFACE target keeps the dependency graph explicit.
add_library(helios_math INTERFACE)
add_library(helios::math ALIAS helios_math)
target_include_directories(helios_math INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(helios_math INTERFACE cxx_std_23)
