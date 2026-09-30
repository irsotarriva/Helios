# Helios — notes for AI assistants and contributors

- Read `docs/BRIEFING.md` (design, decisions in §16) and `docs/CODING_STANDARDS.md` before writing code.
- C++23. Supported compilers: GCC ≥ 14, Clang ≥ 19 (Clang 18 cannot use libstdc++'s `std::expected`), MSVC 17.10+.
- Build: `cmake --preset debug && cmake --build --preset debug && ctest --preset debug` (needs `VCPKG_ROOT`).
  Without vcpkg, a system Catch2 v3 works: `cmake -S . -B build/local -G Ninja -DCMAKE_CXX_COMPILER=g++-14`.
- Lint: `run-clang-tidy-19 -p <build dir> "$PWD/(source|test)/"` and `clang-format --dry-run --Werror`.
- The simulation (`source/` minus render/xr/net) is headless and must stay GPU- and window-free.
- Never throw; return `helios::core::Result<T>`. Wrap third-party calls in `core::try_call`.
- Logging: LumenLog `LOG_*` with **string-literal messages only**; runtime values go in numeric
  `.tag()`s (see `include/helios/core/logging.hpp` for why).
- No new third-party dependency without the maintainer's approval.
