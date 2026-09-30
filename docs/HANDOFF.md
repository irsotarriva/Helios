# Handoff

For the next person or agent picking up Helios, especially on a **machine that has never built
it**. Read this first, then [`BRIEFING.md`](BRIEFING.md) (design, decisions in §16) and
[`CODING_STANDARDS.md`](CODING_STANDARDS.md). [`../CLAUDE.md`](../CLAUDE.md) has the short
version of the rules.

## 1. Where the project is

| Phase | State | Evidence |
|---|---|---|
| 0 Foundations | ✅ | CMake presets, vcpkg, clang-tidy / clang-format, CI, `core::Result`, LumenLog |
| 1 Headless universe | ✅ | [ephemeris](validation/ephemeris/README.md) and [dynamics](validation/dynamics/README.md) validation against JPL DE421 |
| 1.5 Headless groundwork | ✅ | conic events, warp-invariant impulses, exact trajectory prediction, time-warp controller, `sim::Simulation` |
| 2 Map view | ✅ | [map view](map_view/README.md): bgfx + SDL3 + Dear ImGui client, screenshots |
| 1b VR spike | not started | needs Linux or Windows (no OpenXR runtime on macOS) |
| 3 Flight | next | parts, rigid vessel, staging, on/off rails, the control bus |

Tests: 142 GoogleTest cases (`ctest`). CI (`.github/workflows/ci.yml`) builds and tests on
macOS (Apple clang, ASan/UBSan), Linux (Clang 19 and GCC 14, ASan/UBSan; GCC Release),
clang-tidy + clang-format, the client on macOS (Metal) and Linux (with a software-OpenGL smoke
run that uploads a screenshot), and Windows (MSVC headless and client).

**Tested vs untested.** Everything above passes in CI, and the client has been run under Mesa's
software OpenGL on Linux. The client has **never been run on real GPU hardware**: not on the
maintainer's M1 Mac, not on Windows with Direct3D. The Windows CI jobs were added last, so check
their status first (section 2.1) and treat Windows as the least-exercised platform.

## 2. First run on a new machine (checklist)

Nothing is guaranteed to be installed or to work. Check every item and report what was missing
or had to be fixed; if a step fails, fix the cause (or record it here) rather than work around it
silently.

### 2.1 Before touching the machine

- Look at the latest CI run on the branch (GitHub Actions). If a job is red, that is the first
  thing to fix, and the log says exactly what the compiler thinks.

### 2.2 Tools

Run each check and compare with the minimum.

| Tool | Check | Minimum | Install if missing |
|---|---|---|---|
| git | `git --version` | any recent | Windows: Git for Windows; macOS: Xcode CLT |
| Compiler | Windows: `cl` (in a Developer PowerShell) · macOS: `clang++ --version` | MSVC 19.40 (VS 2022 17.10) · Apple clang 16 (Xcode 16) · Clang 19 / GCC 14 | Windows: Visual Studio 2022 with the *Desktop development with C++* workload · macOS: `xcode-select --install` or Xcode |
| CMake | `cmake --version` | 3.28 | ships with Visual Studio's C++ workload; macOS: `brew install cmake` |
| Ninja | `ninja --version` | any | ships with Visual Studio; macOS: `brew install ninja` |
| vcpkg | `echo $env:VCPKG_ROOT` / `echo $VCPKG_ROOT`, then `& $env:VCPKG_ROOT/vcpkg version` | at the commit in `vcpkg.json` → `builtin-baseline` | `git clone https://github.com/microsoft/vcpkg`, `git -C vcpkg checkout <baseline>`, `vcpkg/bootstrap-vcpkg.bat` (or `.sh`), set `VCPKG_ROOT` permanently |
| Python (optional) | `python --version` | 3.11 | only for `tools/` (plots, ephemeris export): `pip install -r tools/ephemeris/requirements.txt` |
| clang-format (optional) | `clang-format --version` | **19** exactly (CI's version; 18 formats differently) | `pip install clang-format==19.1.7` |
| clang-tidy (optional) | `clang-tidy --version` | 19 | LLVM 19 release; on Windows it needs a `compile_commands.json` from a Ninja build |

**Windows specifics.**
- Every build command runs in a *Developer PowerShell for VS 2022* (or after
  `vcvars64.bat`); otherwise CMake finds no compiler or the wrong one. Confirm `where cl` and
  `where ninja` both resolve.
- Long paths: FetchContent unpacks deep trees under `build/`. Clone near the drive root (for
  example `C:\dev\Helios`) or enable long paths (`git config --global core.longpaths true`).
- Line endings are pinned by `.gitattributes` (LF in the working tree). If files show as
  modified right after cloning, check `git config core.autocrlf` and run
  `git add --renormalize .`.
- The GPU on the maintainer's Windows machine is old. bgfx picks Direct3D 12 or 11
  automatically; if the window stays black or bgfx fails to initialise, try
  `--renderer d3d11`, then `--renderer opengl`.

**macOS specifics.** Apple clang is the primary compiler. The client renders with Metal. There
is no OpenXR runtime on macOS, so no VR work there.

### 2.3 Get and build

```sh
git clone --recursive https://github.com/irsotarriva/Helios.git
cd Helios
git checkout claude/modable-space-game-6ie6qd   # or main, once merged
git submodule update --init --recursive         # harmless if already done

cmake --preset debug
cmake --build --preset debug
ctest --preset debug                             # expect 142/142
```

Then the client (the first build takes about ten minutes because of bgfx's shader compiler):

```sh
cmake --preset client
cmake --build --preset client
ctest --preset client
./build/client/apps/helios/helios --frames 120 --screenshot first.png   # Windows: build\client\apps\helios\helios.exe
```

Open `first.png` and compare it with [map_view/earth_dayside.png](map_view/earth_dayside.png):
panels on the left, Earth at the centre with the station's orbit, planet orbit lines, labels.
Then run it interactively (`helios` without flags) and check: 60 FPS in the *Time* panel,
dragging and zooming, warp buttons, and scheduling a burn in the *Vessel* panel.

### 2.4 What to record

Add to section 5 below: the machine (OS, CPU, GPU), compiler versions, anything installed, any
fix that was needed, measured frame rate, and which bgfx renderer was used (the log prints
`renderer: …`).

## 3. Where things are

| Path | What |
|---|---|
| `include/helios/`, `source/` | the engine, one library per directory: `core`, `time`, `math`, `orbital`, `ephemeris`, `frames`, `bodies`, `dynamics`, `sim` (all headless), `render` (GPU-free camera/geometry) and `render/gpu` (bgfx; client only) |
| `apps/helios/` | the game executable: window, main loop, UI, demo scenario, simulation thread |
| `test/unit/` | GoogleTest suites mirroring `source/`; shared test universes in `tools/support/universes.hpp` |
| `data/solar_system/` | `bodies.csv` (μ, radii, spheres of influence, rotation) and `mean_elements.csv` (fitted to DE421) |
| `tools/` | validation scenarios (C++), plotting and ephemeris export (Python, dev-only) |
| `docs/validation/` | ephemeris and dynamics validation reports with figures |
| `extern/LumenLog` | logging library (git submodule, pinned commit) |
| `cmake/` | compiler options, LumenLog wiring, the client's pinned dependencies |

Data flow at run time: `sim::Simulation` (bodies on rails, Encke-propagated vessels, clock,
warp) runs on its own thread in `SimulationHost`, publishes an immutable `sim::SceneSnapshot`
(positions as doubles relative to the camera focus), and the render thread turns it into
camera-relative floats (`render::build_frame_geometry`) for `render::gpu::MapRenderer`.

## 4. Next steps (proposed)

1. **Confirm the platforms.** Build and run on the maintainer's Windows and Mac machines
   (section 2) and fix what breaks. This comes before new features.
2. **Phase 3: Flight.** Data-defined parts, a rigid vessel, staging, on/off-rails transitions,
   finite burns with mass flow in the Encke force model, and the control bus (BRIEFING §8).
   This needs new dependencies (Jolt for the physics bubble), which require the maintainer's
   approval.
3. **Phase 1b: VR spike** on Windows or Linux: OpenXR (loader already approved) with a simulated
   headset (Monado on Linux; Meta XR Simulator on Windows), rendering through `MapRenderer`'s
   per-view API.
4. Smaller open items:
   - Encke's ~0.3 m accuracy floor over 20 days
     ([dynamics validation](validation/dynamics/README.md), "Open item").
   - A script to regenerate the full DE421 `.hce` (the export exists in
     `tools/ephemeris/export_de_chebyshev.py`; the ~15 MB file is not committed).
   - Thick screen-space orbit lines and textured planets.
   - An MSVC clang-tidy run.

## 5. Machine log

Record every new machine here (see section 2.4).

| Date | Machine | Result | Notes |
|---|---|---|---|
| 2026-09-30 | Linux container (Ubuntu 24.04, 4 cores, no GPU) | headless 142/142; client under Xvfb + Mesa llvmpipe (OpenGL 4.3), ~50 FPS at 1280×720 | Clang 19, GCC 14; the reference screenshots came from here |
