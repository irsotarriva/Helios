# Helios

An open-source, deeply moddable spaceflight and exploration sandbox. It features a real-scale
Solar System, pilot's-seat (IVA/VR) flight, ghost timelines for overlapping missions, and
physics-driven vessel design.

Status: early development. The headless universe (real-scale Solar System, Encke propagation,
events, warp-invariant time warp) and the first renderer, the [map view](docs/map_view/README.md),
are in. Start with [`docs/BRIEFING.md`](docs/BRIEFING.md); the current state and next steps are in
[`docs/HANDOFF.md`](docs/HANDOFF.md).

![Lunar flyby in the map view](docs/map_view/lunar_flyby.png)

## Getting the code

LumenLog (logging) is a git submodule, so clone recursively:

```sh
git clone --recursive https://github.com/irsotarriva/Helios.git
# already cloned without --recursive?
git submodule update --init --recursive
```

CMake stops with that command in its error message if `extern/LumenLog` is empty.

## Prerequisites

| | Windows | macOS | Linux |
|---|---|---|---|
| Compiler | Visual Studio 2022 17.10+ ("Desktop development with C++": MSVC, CMake and Ninja) | Xcode 16+ or its Command Line Tools (Apple clang) | Clang ≥ 19 or GCC ≥ 14 (Clang 18 cannot use libstdc++'s `std::expected`) |
| CMake ≥ 3.28, Ninja | included with Visual Studio | `brew install cmake ninja` | distribution packages |
| [vcpkg](https://github.com/microsoft/vcpkg) (GoogleTest) | `git clone https://github.com/microsoft/vcpkg` then `vcpkg\bootstrap-vcpkg.bat` | same, `./bootstrap-vcpkg.sh` | same |
| Client only | — | — | `libx11-dev libxext-dev libxrandr-dev libxcursor-dev libxi-dev libxfixes-dev libxkbcommon-dev libgl1-mesa-dev` |

Set `VCPKG_ROOT` to the vcpkg checkout. For reproducible builds, check out the commit in
`builtin-baseline` of [`vcpkg.json`](vcpkg.json) (`git -C $VCPKG_ROOT checkout <commit>`). The
client's other libraries (bgfx, SDL3, Dear ImGui) are downloaded by CMake at pinned commits,
so the first client configure needs network access.

**Windows:** run every command below from a *Developer PowerShell for VS 2022* (or *x64 Native
Tools Command Prompt*), so `cl` and `ninja` are on `PATH`. The presets use the Ninja generator.

## Building and testing

```sh
cmake --preset debug          # headless core and tests (Debug; ASan/UBSan with GCC/Clang)
cmake --build --preset debug
ctest --preset debug

cmake --preset client         # the game client: bgfx, SDL3, Dear ImGui
cmake --build --preset client
ctest --preset client
./build/client/apps/helios/helios          # Windows: build\client\apps\helios\helios.exe
```

The client's first build also compiles bgfx's shader compiler, which takes around ten minutes.
To check rendering without watching the window, render a fixed run to an image:
`helios --frames 120 --screenshot out.png` (add `--renderer d3d11|d3d12|vulkan|opengl|metal` to
force a back end). Controls and all flags: [docs/map_view](docs/map_view/README.md).

Other presets: `release` (headless, optimised, no tests), `client-release`, `ci`.

## Licence

[Mozilla Public License 2.0](LICENSE).
