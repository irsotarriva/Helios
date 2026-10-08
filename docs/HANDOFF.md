# Handoff

For the next person or agent picking up Helios, especially on a **machine that has never built
it**. Read this first, then [`BRIEFING.md`](BRIEFING.md) (design, decisions in ﾂｧ16) and
[`CODING_STANDARDS.md`](CODING_STANDARDS.md). [`../CLAUDE.md`](../CLAUDE.md) has the short
version of the rules.

## 1. Where the project is

| Phase | State | Evidence |
|---|---|---|
| 0 Foundations | 笨・| CMake presets, vcpkg, clang-tidy / clang-format, CI, `core::Result`, LumenLog |
| 1 Headless universe | 笨・| [ephemeris](validation/ephemeris/README.md) and [dynamics](validation/dynamics/README.md) validation against JPL DE440 |
| 1.5 Headless groundwork | 笨・| conic events, warp-invariant impulses, exact trajectory prediction, time-warp controller, `sim::Simulation` |
| 2 Map view | 笨・| [map view](map_view/README.md): bgfx + SDL3 + Dear ImGui client, screenshots |
| Warp to 10竅ｹﾃ・| 笨・| analytic (Kepler) regime in the propagator, warp limits that hold for 96-day ticks (BRIEFING D21) |
| 1b VR spike | ✅ on a stand-in headset | `helios --vr`: the cockpit in stereo through OpenXR on bgfx (BRIEFING D29), Windows and Direct3D 11; run against `tools/xr_null_runtime`, **not yet on a real runtime** |
| 3 Flight | 笨・| parts as datasheets, part tree, control bus, resources, staging, finite burns (BRIEFING D22); the physics bubble on Jolt: rotation, attitude hold, contact, landing (D23, D25); demo vessels flown from the map view |
| 4 Pilot's seat, first part | 笨・| [flight view](flight_view/README.md): cockpits as data, the view from the seat and from outside, instruments and keys on the bus (BRIEFING D26); two crewed demo vessels |
| 4 Pilot's seat, in the cabin | 笨・| the pilot leaves the seat: floats and holds on in free fall, walks under weight, and pushes the vessel back (BRIEFING D27); two demo vessels with a habitat |
| 4 Pilot's seat, loose items | 笨・| items to pick up, carry and throw (the recoil moves the pilot); one interact key with a prompt (BRIEFING D28) |
| 4 Pilot's seat, the rest | next | EVA, text on the panels, hands in VR |

Tests: 253 GoogleTest cases (`ctest`). CI (`.github/workflows/ci.yml`) builds and tests on
macOS (Apple clang, ASan/UBSan), Linux (Clang 19 and GCC 14, ASan/UBSan; GCC Release),
clang-tidy + clang-format, the client on macOS (Metal) and Linux (with a software-OpenGL smoke
run that uploads a screenshot), and Windows (MSVC headless and client).

**Tested vs untested.** Everything above passes in CI, and the client has been run under Mesa's
software OpenGL on Linux and on a Windows PC with real hardware (Direct3D 11, Direct3D 12 and
OpenGL on a GeForce GT 710; section 5). It has **not been run on the maintainer's M1 Mac**
(Metal on real hardware). MSVC support was brought up in CI on 2026-09-30 (runner: Visual
Studio 2026 18.10, MSVC 19.51, CMake 4.4). The fixes it needed were small (CRT deprecation
warnings, POSIX `setenv` in a test, one unreachable-code warning); the code otherwise compiled
cleanly at `/W4 /WX`. The Windows client job builds but does not run the executable (no GPU on
the runner), so the Windows client is only exercised on the PC of section 5.

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
| Compiler | Windows: `cl` (in a Developer PowerShell) ﾂｷ macOS: `clang++ --version` | MSVC 19.40 (VS 2022 17.10) ﾂｷ Apple clang 16 (Xcode 16) ﾂｷ Clang 19 / GCC 14 | Windows: Visual Studio 2022 with the *Desktop development with C++* workload ﾂｷ macOS: `xcode-select --install` or Xcode |
| CMake | `cmake --version` | 3.28 | ships with Visual Studio's C++ workload; macOS: `brew install cmake` |
| Ninja | `ninja --version` | any | ships with Visual Studio; macOS: `brew install ninja` |
| vcpkg | `echo $env:VCPKG_ROOT` / `echo $VCPKG_ROOT`, then `& $env:VCPKG_ROOT/vcpkg version` | at the commit in `vcpkg.json` 竊・`builtin-baseline` | `git clone https://github.com/microsoft/vcpkg`, `git -C vcpkg checkout <baseline>`, `vcpkg/bootstrap-vcpkg.bat` (or `.sh`), set `VCPKG_ROOT` permanently |
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
- **exFAT / FAT drives** have no file ownership, so Git's `safe.directory` check rejects every
  fresh clone on them, including FetchContent's bgfx/SDL/ImGui clones under `build/`. Either
  clone on an NTFS drive, or keep the checkout and move only the fetched sources:
  `cmake --preset client -DFETCHCONTENT_BASE_DIR=C:/dev/helios-deps/client`. Clones you make
  by hand (vcpkg) need `git config --global --add safe.directory <path>`.
- On a non-Latin system code page (e.g. 932, Japanese), MSVC reads UTF-8 sources in that code
  page unless `/utf-8` is given. Helios and the client's third-party code are built with
  `/utf-8`; if C4819 warnings reappear, a target is missing it.

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
ctest --preset debug                             # expect 152/152
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
`renderer: 窶ｦ`).

## 3. Where things are

| Path | What |
|---|---|
| `include/helios/`, `source/` | the engine, one library per directory: `core`, `time`, `math`, `orbital`, `ephemeris`, `frames`, `bodies`, `dynamics`, `vessel`, `physics` (the Jolt wrapper), `sim` (all headless), `render` (GPU-free camera/geometry) and `render/gpu` (bgfx; client only) |
| `apps/helios/` | the game executable: window, main loop, UI, demo scenario, simulation thread |
| `test/unit/` | GoogleTest suites mirroring `source/`; shared test universes in `tools/support/universes.hpp` |
| `data/solar_system/` | `bodies.csv` (ﾎｼ, radii, spheres of influence, rotation) and `mean_elements.csv` (fitted to DE440 over 2000窶・200) |
| `data/parts/`, `data/vessels/` | part datasheets and vessel blueprints (TOML; each directory has a README with the format) |
| `tools/` | validation scenarios (C++), plotting and ephemeris export (Python, dev-only) |
| `docs/validation/` | ephemeris and dynamics validation reports with figures |
| `extern/LumenLog` | logging library (git submodule, pinned commit) |
| `cmake/` | compiler options, LumenLog wiring, the client's pinned dependencies |

Data flow at run time: `sim::Simulation` (bodies on rails, Encke-propagated vessels, clock,
warp) runs on its own thread in `SimulationHost`, publishes an immutable `sim::SceneSnapshot`
(positions as doubles relative to the camera focus), and the render thread turns it into
camera-relative floats (`render::build_frame_geometry`) for `render::gpu::MapRenderer`.

A vessel made of parts carries a `vessel::VesselSystems` (part tree, stores, control bus).
Commands go in through `Simulation::command` / `schedule_command`; the systems work out, in
closed form, the thrust and mass over time, and the simulation hands that to the vessel's
propagator as a thrust plan. The snapshot lists every signal of the bus, and the map view
draws its controls and readings from that list alone.

The vessel the player flies (`Simulation::set_active_vessel`; the client uses the focused
vessel) is in the physics bubble while the warp is at most 4ﾃ・ `source/sim/simulation_bubble.cpp`
ticks it at 1/128 s as a rigid body in `physics::World`, turning it under torques and
resolving contact with the ground, while the propagator keeps its orbit (BRIEFING D25).

The flight view (BRIEFING D26, [flight_view/README.md](flight_view/README.md)) draws that
vessel from its seat or from outside. The snapshot carries every vessel's parts (pose and
datasheet); `render::build_flight_geometry` turns them into solids, the cockpit's instruments
and the ground patch, and `render::gpu::SolidRenderer` draws them into the map renderer's
view. `apps/helios/flight_controls.cpp` turns key actions and handled instruments into bus
commands; `application.cpp` only maps devices to those.

The pilot (BRIEFING D27) is `sim::Pilot`, aboard the focused vessel. Out of the seat it is a
body in `Simulation::Cabin` (`source/sim/simulation_pilot.cpp`): a second `physics::World`,
in the vessel's axes, whose walls are the cockpit's boxes. The client sends a `PilotInput`
every frame (where the pilot looks, from `apps/helios/pilot_controls.cpp`; the arm or the
legs; the hand) and draws from `SceneSnapshot::pilot` and `SceneSnapshot::items`. What the
interact key does is not decided in the client: the snapshot carries the simulation's offer
(`PilotOffer`, BRIEFING D28), the client shows it and calls `Simulation::interact()`.

## 4. Next steps (proposed)

1. **Confirm the platforms.** Build and run on the maintainer's Windows and Mac machines
   (section 2) and fix what breaks. This comes before new features.
2. **Phase 4: Pilot's seat, the rest** (BRIEFING ﾂｧ12). The seat (D26) and the cabin (D27)
   are in. Next, roughly in this order:
   - **Hand-test the cabin** (built and screenshot-tested on the maintainer's Windows PC, not
     yet moved about in by hand): mouse look with the pointer captured, grabbing and pushing
     off in Albatross, walking in Petrel, the feel of the arm (speeds and strengths are the
     constants at the top of `simulation_pilot.cpp`).
   - **EVA**: a suit as a small vessel, tethers, the airlock as the place where the inside
     and the outside representation are swapped. It needs more than one rigid body in the
     physics bubble (today: the active vessel only), which docking needs too.
   - **Text in the 3-D renderer**, so that instrument names and numbers are on the panel
     (rule 3 of ﾂｧ12.1) instead of drawn over the picture; then a screen (MFD) rendered to a
     texture.
   - **VR** (item 3 below): the picture is in (D29); hands would drive the same grab that the
     mouse button does, and the same instruments.
   - **Models.** Everything is boxes and cylinders. Loading glTF needs cgltf (named in
     BRIEFING ﾂｧ13, **not yet approved**); the maintainer plans to commission models later.
     Until then, shapes stay in the datasheets.
   - **Controls**: the maintainer wants them few and context-sensitive (D28). New things to
     do should become offers of the interact key before they become new keys, and bindings
     should move out of `application.cpp` into a file.
   - Smaller: loose items go back to their stowed places when the pilot boards again, and are
     not saved anywhere; the pilot's 80 kg are not part of the vessel's mass while seated; the pilot
     boards whichever vessel is focused, by teleport; one crewed part per vessel is drawn
     from inside; the reaction on the vessel lags the pilot by a frame; parts without a shape
     are invisible; nothing casts a shadow; the keys are bound in `application.cpp`, not in a
     file; no gamepad.
   What Phase 3 leaves open in the bubble, roughly in order of how much it will be missed:
   - **No terrain and no picture of the ground.** The ground is the body's mean sphere, as a
     plane under the vessel, and the map view draws planets as coarse spheres: a landing is
     flown on the altitude and the vertical speed. Terrain is Phase 7.
   - **Only the active vessel is a rigid body.** Two vessels cannot touch (no docking, no
     debris hitting anything); a stage dropped in the bubble goes straight onto rails.
   - **A hard landing destroys the whole vessel.** Parts do not break off one by one yet
     (BRIEFING ﾂｧ10.4), and there is no stress estimate.
   - **On rails the attitude is ideal.** Coming out of the bubble (warp above 4ﾃ・ the nose is
     taken to be on the commanded pointing whatever it was. A vessel with no torque authority
     cannot turn in the bubble but turns freely on rails.
   - **A landed vessel only lifts off as the active vessel at low warp.** Thrust on a landed
     vessel does nothing otherwise. A vessel on rails that reaches the surface is destroyed,
     however slowly.
   - The attitude hold is a proportional rate law per axis against the diagonal of the
     inertia tensor; it is not tuned for vessels whose principal axes are far from their
     own, and it knows nothing of thrust vectoring.
   - Thrust is the vacuum value; `Isp(p_amb)` and jet engines need the environment as a curve
     input, which comes with atmospheres (Phase 7).
   - Processes scale linearly with their level (thrust may have a curve). A datasheet cannot
     yet say that consumption is not proportional to throttle.
   - Thrust plans stop at 64 changes per plan; the plan is renewed at every change, so this
     only bounds what one frame can cross.
   - Jolt comes from vcpkg in single precision with its default instruction set (AVX2 on
     x86-64). The minimum CPU and cross-platform determinism have not been looked at.
3. **VR, what is left** (BRIEFING D29; how to run it: [flight view](flight_view/README.md), "In a
   headset"). `helios --vr` draws the pilot's view for a headset through OpenXR. It has only been
   run against the null headset (`tools/xr_null_runtime`). Next, in this order:
   - **A real runtime.** Install the Meta XR Simulator (Windows) and run `helios --vr --focus
     Osprey`: what to look for is the session coming up, the picture in both eyes the right way
     up and not washed out or too dark (D29 on sRGB), the head tracked, and the frame rate in
     the log. Then a headset.
   - **Hands**: OpenXR actions for the controllers, driving `PilotInput::grab` and the
     instruments (`press_instrument`, `move_lever`) that the mouse drives now.
   - **Linux**: a second graphics binding (D29 says why Vulkan is not straightforward), then
     Monado's simulated headset in CI.
   - Smaller: the headset shows nothing in the map and outside views; labels and prompts are
     2-D overlays and are not in the headset (text in the 3-D renderer comes first); the eyes'
     images are not anti-aliased; no re-centring key.
4. **High warp, what is left.** Warp reaches 10竅ｹﾃ・(BRIEFING D21): weakly perturbed orbits are
   analytic, the sim host caps each tick at two tick periods (a slow tick makes the simulation
   fall behind instead of taking ever larger steps) and the *Time* panel shows the achieved
   warp. In Release on a Ryzen 5800X the demo holds 10竅ｹﾃ・at 120 ticks/s with ~28 % of a core.
   Still open:
   - A vessel on a short-period orbit that is too perturbed for the analytic regime
     (geostationary, low lunar orbit) costs integration steps in proportion to the warp and
     caps the achieved warp for everything (~16 000 steps per wall second for a 90-minute
     orbit at 10竅ｶﾃ・ about one core). Auto-park (ﾂｧ7.1 rule 4, D15) is the designed answer.
   - A chemical-rocket trip to ﾎｱ Centauri (~77 000 years at 17 km/s) is ~40 min of play at
     10竅ｹﾃ・ but there is nowhere to arrive yet: other star systems are Phase 10.
   - The demo's lunar probe returns and hits Earth after a month; each of its events slows the
     warp, so the top levels are only reached after about 40 s of play.
5. Smaller open items:
   - Encke's ~0.3 m accuracy floor over 20 days
     ([dynamics validation](validation/dynamics/README.md), "Open item").
   - The stock data is JPL DE440 since 2026-10-01 (kernel and masses downloaded from JPL; steps
     and checksums in the [ephemeris validation](validation/ephemeris/README.md), "Reproducing").
     The full `.hce` export (~110 MB, 1550窶・650) is not committed and has to be regenerated
     from the kernel; the game runs on the mean elements unless given `--ephemeris`.
   - Beyond DE440's coverage only the mean elements exist, and they are fitted on 2000窶・200.
     With warp at 10竅ｹﾃ・players get there quickly; Jupiter and Saturn drift by degrees within a
     few centuries (the "great inequality"), which periodic terms would fix.
   - The mean-element fit of the Earth窶溺oon barycentre is 3窶・ﾃ・worse on windows of 400 years
     or more than on 300 years, while Venus and Mars are unaffected. That looks like the
     optimiser failing to converge rather than physics; it has not been investigated. Not a
     problem for the window in use; worth fixing before widening it.
   - Thick screen-space orbit lines and textured planets.
   - An MSVC clang-tidy run.

## 5. Machine log

Record every new machine here (see section 2.4).

| Date | Machine | Result | Notes |
|---|---|---|---|
| 2026-09-30 | Linux container (Ubuntu 24.04, 4 cores, no GPU) | headless 142/142; client under Xvfb + Mesa llvmpipe (OpenGL 4.3), ~50 FPS at 1280ﾃ・20 | Clang 19, GCC 14; the reference screenshots came from here |
| 2026-10-01 | Windows 11 Home (26200), Ryzen 7 5800X, 64 GB, NVIDIA GeForce GT 710 (driver 456.71), repo on an exFAT drive, code page 932 | headless 142/142 and client 142/142 (MSVC 19.44, Debug); screenshot runs on Direct3D 11 (auto-selected), Direct3D 12 and OpenGL 4.3 all match `earth_dayside.png` at `--start-unix 1790000000 --camera-yaw 180`; 60 FPS (vsync) on all three | VS 2022 Build Tools 17.14 (CMake 4.4.2 from PATH, Ninja from VS); vcpkg cloned at the baseline (`VCPKG_ROOT` set for the user); LLVM 19.1.5 from VS for clang-format/clang-tidy. Found the same MSVC errors as CI (fixed there in parallel). Fixes specific to this machine: `/utf-8` for bgfx/dawn (C4819 on cp932) and an OpenGL hang in `bgfx::init` (WGL never gives up looking for a 32-bit depth pixel format; Windows OpenGL now requests D24S8). FetchContent sources moved to C: (exFAT, see ﾂｧ2.2). The maintainer's interactive check (dragging, warp, burns) found the 10竅ｶﾃ・tick spiral, fixed with the tick cap (ﾂｧ4). After the warp work: 152/152; Release holds 10竅ｹﾃ・at 120 ticks/s |
