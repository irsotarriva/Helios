# Helios

An open-source, deeply moddable spaceflight and exploration sandbox. It features a real-scale
Solar System, pilot's-seat (IVA/VR) flight, ghost timelines for overlapping missions, and
physics-driven vessel design.

Status: design phase. Start with [`docs/BRIEFING.md`](docs/BRIEFING.md).

## Building

Requires CMake ≥ 3.28, Ninja, [vcpkg](https://github.com/microsoft/vcpkg) (`VCPKG_ROOT` set)
and Clang ≥ 19 (or a current Apple clang), GCC ≥ 14 or MSVC 17.10+.

```sh
cmake --preset debug          # Debug + ASan/UBSan
cmake --build --preset debug
ctest --preset debug
```

## Licence

[Mozilla Public License 2.0](LICENSE).
