# Helios — notes for AI assistants and contributors

- On a machine that has not built Helios before, follow `docs/HANDOFF.md` §2 first: verify every tool
  (compiler included) and record the result in its machine log. Current state and next steps are there too.
- Read `docs/BRIEFING.md` (design, decisions in §16) and `docs/CODING_STANDARDS.md` before writing code.
- LumenLog is a git submodule (`extern/LumenLog`): clone with `--recursive` or run
  `git submodule update --init --recursive`. On Windows, build from a Developer PowerShell for VS 2022.
- C++23. Supported compilers: Clang ≥ 19 / Apple clang (macOS is the maintainer's main platform), GCC ≥ 14,
  MSVC 17.10+. Clang 18 cannot use libstdc++'s `std::expected`.
- Build: `cmake --preset debug && cmake --build --preset debug && ctest --preset debug` (needs `VCPKG_ROOT`).
  Without vcpkg, a system GoogleTest works: `cmake -S . -B build/local -G Ninja -DCMAKE_CXX_COMPILER=g++-14`.
- Lint: `run-clang-tidy-19 -p <build dir> "$PWD/(source|test|apps)/"` and `clang-format --dry-run --Werror`
  (CI uses clang-format 19; 18 formats some constructs differently).
- The simulation (`source/` minus render/xr/net) is headless and must stay GPU- and window-free.
  `source/render/*.cpp` (camera, meshes, snapshot → camera-space geometry) is GPU-free too and
  tested everywhere; `source/render/gpu` and `apps/helios` (bgfx, SDL3, Dear ImGui) build only with
  `HELIOS_BUILD_CLIENT=ON` (`cmake --preset client`). Screenshot runs for checking visuals:
  `helios --renderer opengl --frames N --screenshot out.png` (works under Xvfb with Mesa).
- Never throw; return `helios::core::Result<T>`. Wrap third-party calls in `core::try_call`.
- Logging: LumenLog `LOG_*` (accepts `std::format` arguments) plus `.tag()`; metrics via `lumen::metric`.
  Sinks are filtered by query strings; override at runtime with `HELIOS_LOG_TERMINAL`, `HELIOS_LOG_JSON`,
  `HELIOS_LOG_JSON_QUERY` (e.g. `HELIOS_LOG_TERMINAL='level >= DEBUG && subsystem == orbital'`).
- Conventions: header guards (`HELIOS_<PATH>_HPP`), GoogleTest, members `name` (public) / `name_` (protected, private).
- Physical quantities carry SI unit suffixes: `distance_m`, `velocity_m_s`, `inclination_rad` (CODING_STANDARDS §5.1).
- No new third-party dependency without the maintainer's approval.
- Validation tools: `tools/ephemeris/` (Python) and `tools/dynamics_scenarios` + `tools/dynamics/` (C++ scenarios,
  Python plots); reports in `docs/validation/`. Ephemeris data tools live in `tools/ephemeris/` (Python, dev-only; see `docs/validation/ephemeris/README.md`).
  The full `.hce` export (~15 MB) is not committed; `test/data/de421_2020_excerpt.hce` is.
