# Helios

An open-source, deeply moddable spaceflight and exploration sandbox. It features a real-scale
Solar System, pilot's-seat (IVA/VR) flight, ghost timelines for overlapping missions, and
physics-driven vessel design.

Status: early development. The headless universe (real-scale Solar System, Encke propagation,
events, warp-invariant time warp) and the first renderer, the [map view](docs/map_view/README.md),
are in. Start with [`docs/BRIEFING.md`](docs/BRIEFING.md).

![Lunar flyby in the map view](docs/map_view/lunar_flyby.png)

## Building

Requires CMake ≥ 3.28, Ninja, [vcpkg](https://github.com/microsoft/vcpkg) (`VCPKG_ROOT` set)
and Clang ≥ 19 (or a current Apple clang), GCC ≥ 14 or MSVC 17.10+.

```sh
cmake --preset debug          # headless core and tests, Debug + ASan/UBSan
cmake --build --preset debug
ctest --preset debug

cmake --preset client         # the game client (bgfx, SDL3, Dear ImGui; fetched at pinned commits)
cmake --build --preset client
./build/client/apps/helios/helios
```

The client's first build also compiles bgfx's shader compiler, which takes several minutes. On
Linux it needs the X11 and OpenGL development packages (`libx11-dev libxext-dev libxrandr-dev
libxcursor-dev libxi-dev libxfixes-dev libxkbcommon-dev libgl1-mesa-dev`). Controls and flags:
[docs/map_view](docs/map_view/README.md).

## Licence

[Mozilla Public License 2.0](LICENSE).
