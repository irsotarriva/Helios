# Helios — notes for AI assistants and contributors

- Read `docs/BRIEFING.md` (design, decisions in §16) and `docs/CODING_STANDARDS.md` before writing code.
- C++23. Supported compilers: Clang ≥ 19 / Apple clang (macOS is the maintainer's main platform), GCC ≥ 14,
  MSVC 17.10+. Clang 18 cannot use libstdc++'s `std::expected`.
- Build: `cmake --preset debug && cmake --build --preset debug && ctest --preset debug` (needs `VCPKG_ROOT`).
  Without vcpkg, a system GoogleTest works: `cmake -S . -B build/local -G Ninja -DCMAKE_CXX_COMPILER=g++-14`.
- Lint: `run-clang-tidy-19 -p <build dir> "$PWD/(source|test)/"` and `clang-format --dry-run --Werror`.
- The simulation (`source/` minus render/xr/net) is headless and must stay GPU- and window-free.
- Never throw; return `helios::core::Result<T>`. Wrap third-party calls in `core::try_call`.
- Logging: LumenLog `LOG_*` (accepts `std::format` arguments) plus `.tag()`; metrics via `lumen::metric`.
- Conventions: header guards (`HELIOS_<PATH>_HPP`), GoogleTest, members `name` / `name_` (protected) / `name__` (private).
- No new third-party dependency without the maintainer's approval.
