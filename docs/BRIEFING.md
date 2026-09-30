# Helios — Project Briefing

> Status: **draft for discussion**. Anything marked **[OPEN]** is a decision we have
> not made yet. See §16 for the list.

---

## 1. Vision in one paragraph

Helios is an open-source spaceflight and exploration sandbox built so that it can be
modded all the way down. It starts in a realistic, full-scale Solar System and scales up
to a galaxy. You can fly any vessel from the pilot's seat, including in VR. You can run
many missions that overlap in time without micromanaging them. You design vessels with a
simplified parametric CAD workflow, and the physics characterises each part for you.
Automation and planning tools are built in, not bolted on.

### What we do differently from existing games in the genre

| Pillar | One-line commitment |
|---|---|
| Pilot's seat first | IVA/FPV is a primary view, not an afterthought; VR-ready by architecture |
| Real scale | Default universe is the real Solar System at 1:1 scale, with real ephemerides |
| Ghost timelines | Every vessel has a **worldline**. Missions can overlap in game time. Multiplayer uses the same mechanism. |
| Hierarchical frames | Positions are always relative to the nearest *anchor body*, so there is no global float precision problem |
| On-rails large bodies | Stars, planets and large moons follow closed-form ephemerides, so their position at any time is an O(1) evaluation |
| Tree-truncated gravity | Vessels feel a physically chosen *subset* of bodies (enough for Lagrange points), not only their SOI parent |
| Extreme time warp | Stable from 0.01× to at least 10⁶× |
| Planning & automation | Launch windows, transfers, ascent/landing guidance, repeatable logistics routes, and scriptable autopilots |
| Rich environments | Procedural and data-driven planets with oceans, submarines, buoyant flight and dense atmospheres |
| Design by physics | Parts are designed (profile → extrude → characterise). Performance is *computed*, then reused one level up. |

### Clean-room / IP rules

- No names, characters, art, sounds, UI layouts or part names from any existing game. We
  use no "Kerbal", "Kerbin", "Mun", "Jeb", "MechJeb" and similar, even in code
  identifiers.
- No decompiled or ported code from any closed-source game. Game mechanics and physics
  are not protected, but specific expression is.
- Community mods for other games are good references, but their licences differ.
  Ferram Aerospace Research (aerodynamics), kOS (in-game scripting) and MechJeb
  (autopilot/planning) are GPL-3.0. We can read their docs and ideas, but we can only
  copy code if Helios is GPL-3.0 too. Principia (N-body gravity with careful
  integrators) is MIT, so its code could be reused under any licence. Verify each
  licence before reusing anything.
- Real-world data: JPL ephemerides and most NASA imagery and elevation data are public
  domain. Check other datasets (ESA, GEBCO, etc.) one by one.

---

## 2. Architecture overview

The main architectural rule: **the simulation is a headless, engine-agnostic C++23
library.** Rendering, VR, audio, UI and networking are *clients* of the simulation. This
keeps the hard part (physics and time) testable in CI without a GPU, and it keeps us free
to change the renderer.

```
                       ┌────────────────────────────────────────────┐
                       │                  app / game                │
                       └──────┬──────────┬──────────┬───────────────┘
                              │          │          │
        ┌─────────────────────▼─┐ ┌──────▼─────┐ ┌──▼──────────┐
        │ render (bgfx/Vulkan)  │ │ xr (OpenXR)│ │ net (server)│   ← clients
        └─────────────▲─────────┘ └──────▲─────┘ └──▲──────────┘
                      │ double-buffered snapshot     │ worldline sync
┌─────────────────────┴──────────────────────────────┴──────────────────┐
│                              sim (headless)                           │
│  time · frames · ephemeris · dynamics · worldline · vessel · environ  │
│  guidance · design (characterisation) · script (sandboxed VM)         │
└───────────────────────────────▲───────────────────────────────────────┘
                                │
┌───────────────────────────────┴───────────────────────────────────────┐
│ core: Result/ErrorCode · math (double3, quat) · ids · VFS · LumenLog   │
└────────────────────────────────────────────────────────────────────────┘
```

Proposed module layout (follows `CODING_STANDARDS.md` §6):

```
include/helios/<module>/<feature>.hpp
source/<module>/<feature>.cpp
test/unit/<module>/test_<feature>.cpp

modules: core, time, frames, ephemeris, dynamics, worldline, vessel, design,
         environment, terrain, guidance, script, mod, render, xr, net, app
```

### 2.1 Performance budget (a hard requirement)

**Decided:** realism scales with the hardware budget, but minimum settings must run at
**≥ 60 FPS on an old laptop**. The reference minimum-spec machine I propose is roughly a
2018 ultrabook: 4 cores, Intel UHD 620-class integrated GPU, 8 GB RAM. We never want
"a rocket in atmosphere at <1 FPS on a top-end GPU".

Frame budget at 60 FPS (16.6 ms) on min spec:

| Slice | Budget | How we stay inside it |
|---|---|---|
| Sim tick (all vessels, active + background) | ≤ 3 ms | Rails bodies are O(1). Background vessels are analytic or Encke with large steps. Only the active vessel is in the physics bubble. |
| Active vessel physics | ≤ 1 ms | **One rigid body per vessel** (§10.4). Aero/hydro from datasheets and per-part panels, never CFD. |
| Terrain / streaming | Off the main thread | Worker pool + atomic swap (standards §8) |
| Render | ≤ 10 ms | LOD everywhere; the fidelity tier picks the shading model |

Fidelity tiers are **config values, not code paths scattered around**. Each model
(gravity θ/ε, aero panel count, terrain depth, ocean model, atmosphere scattering) exposes
a small set of named quality levels. A perf CI job runs a fixed scene headless and
fails if the sim tick exceeds its budget.

This matters for the renderer choice. The min-spec GPU needs OpenGL 4.x or D3D11 class
back-ends. Mac needs Metal. bgfx already covers all of these, which is a strong argument
for keeping it despite the OpenXR integration work (§12).

---

## 3. Numbers: time and space precision

These numbers justify the rest of the design.

### 3.1 Space

The spacing of `double` values (ULP) at distance *x* is about `x · 2⁻⁵²`:

| Distance from origin | Example | ULP (`double`) | ULP (`float`) |
|---|---|---|---|
| 6.4 × 10⁶ m | Earth radius | 1 nm | **0.5 m** |
| 1.5 × 10¹¹ m | 1 AU | 30 µm | 16 km |
| 4.5 × 10¹² m | Neptune | 1 mm | 500 km |
| 9.5 × 10¹⁵ m | 1 light-year | 2 m | — |
| 2.5 × 10²⁰ m | Sun → Galactic centre | **33 km** | — |

What this means in practice:
- The **simulation** uses `double` everywhere, relative to a local anchor frame.
  Inside a planetary system, `double` is more than enough.
- **Rendering and rigid-body physics** use `float` in a *camera- or bubble-relative*
  frame (floating origin). That is a separate, very local frame.
- **Galactic scale** only works because of the hierarchy. We never subtract two
  galactic-frame positions to get something local.

### 3.2 Time

A `double` holding seconds since J2000 degrades as time grows:

| Elapsed | ULP |
|---|---|
| 100 yr | 0.5 µs |
| 10⁶ yr | 4 ms |
| 10⁹ yr | **4 s** |

At galactic travel times (interstellar trips take decades to millennia, and the galactic
year is about 230 Myr), `double` seconds are not enough. Proposal:

```cpp
namespace helios::time {

// Coordinate time of the universe (a TCB-like global time).
// Split representation: whole seconds exact to ±2.9e11 yr, fraction ~1e-16 s.
struct Epoch {
    std::int64_t seconds;   // since the universe's reference epoch
    double       fraction;  // in [0, 1)
};

// Time differences we integrate over are always small enough for a plain double.
[[nodiscard]] double seconds_between(Epoch from, Epoch to) noexcept;

} // namespace helios::time
```

Integrators and propagators always work in a **local time** `dt = t − t_segment_start`
as a `double`. Only bookkeeping uses `Epoch`.

---

## 4. Reference frames and anchor bodies

### 4.1 The frame tree

```
Galaxy (root, inertial)
 └─ Sol (barycentric, inertial)            ← anchor
     ├─ Earth (body-centred, inertial)      ← anchor
     │   ├─ Earth body-fixed (rotating)     ← for landed objects, atmosphere, terrain
     │   └─ Moon (body-centred, inertial)   ← anchor
     │       └─ Moon body-fixed (rotating)
     ├─ Jupiter …
     └─ …
```

- Only **anchor bodies** (stars, planets, large moons) create frames. Asteroids,
  comets and vessels never do.
- Every other object stores `(frame_id, position, velocity)` in `double`, relative to
  the anchor of its current **domain** (SOI-like region).
- Each body also has a rotating body-fixed frame, using the IAU rotation model (closed
  form, like the ephemeris).

### 4.2 Frames are for *representation*, not *dynamics*

This is the key difference from patched conics. The **frame** a vessel is stored in only
decides where its coordinate origin is. The **forces** it feels are decided separately
(§5.2).

Consequences:
- Switching frames is a coordinate transform plus a change of which indirect term we
  subtract. It does **not** change the trajectory. We can add hysteresis to the domain
  boundary (for example, Laplace SOI radius `r = a (m/M)^{2/5}` with ±5% bands) so we
  don't flip frames back and forth.
- Example: Sun–Earth L2 is ~1.5 × 10⁶ km from Earth, which is *outside* Earth's SOI
  (~0.93 × 10⁶ km). A vessel parked there is stored heliocentrically, but it still feels
  Earth, so the halo orbit exists. Patched conics cannot represent this at all.

### 4.3 Non-inertial frames: the indirect term

A body-centred "inertial" frame is actually accelerating (Earth falls around the Sun).
In Earth's frame, the equation of motion for a vessel at `r` (relative to Earth) is:

```
r̈ = −μ_E r/|r|³  +  Σ_k μ_k [ (d_k − r)/|d_k − r|³ ]  −  a_E^{rails}
```

Here `d_k` is body *k* relative to Earth. The last term is the anchor's own acceleration.
We take it **from the rails model** (the second derivative of the ephemeris), not from
summing forces. That keeps vessel dynamics consistent with how the planets actually move
in our universe, even though our planets are on rails. If we skip this, vessels drift
relative to their anchor for no reason.

For rotating frames, add Coriolis and centrifugal terms. Landed and atmospheric physics
uses these.

---

## 5. Gravity and dynamics

### 5.1 Large bodies are on rails (closed form)

Every anchor body has an `EphemerisModel`: a pure function `t → state relative to parent`.
None of them depend on each other at runtime. The absolute state is the sum along the
tree, which is also closed form. Possible models (all moddable):

| Model | Use |
|---|---|
| Keplerian + secular rates | Default for procedural systems; valid forever |
| Chebyshev segments (JPL DE440/441 style) | Real Solar System in a date window, with high accuracy |
| Analytic galactic orbit (e.g. epicyclic) | Stars moving through the galaxy |
| Scripted/custom | Modders: binary stars, rogue planets, anything expressible as `f(t)` |

```cpp
namespace helios::ephemeris {

template<typename T>
concept EphemerisModel = requires(const T& model, time::Epoch t) {
    // State of the body relative to its parent anchor, in the parent's inertial frame.
    { model.state_at(t) }        -> std::same_as<core::Result<StateVector>>;
    // Needed for the indirect term; must be the true second derivative of state_at.
    { model.acceleration_at(t) } -> std::same_as<core::Result<math::double3>>;
    { model.valid_range() }      -> std::same_as<time::Interval>;
};

} // namespace helios::ephemeris
```

Relative two-body motion of a moon about its planet uses `μ = G(M_p + M_m)`. Getting this
wrong visibly shifts Earth–Moon L-points.

Because rails are O(1) in time, arbitrary time warp for bodies is free, and ghost
timelines (§6) can evaluate the universe at *any* epoch.

### 5.2 Which bodies a vessel feels: a tree code, not a fixed depth

Your "one level up, one level down, siblings" rule is really a truncated **tree code**
over the body hierarchy. I suggest we make that explicit with a Barnes–Hut-style
**opening criterion**, instead of a hard-coded depth:

- Walk the body tree from the root.
- A subsystem (a body plus all its satellites) is treated as a **point mass at its
  barycentre** if `size / distance < θ`.
- Otherwise we *open* it and consider its children separately.
- Anything whose contribution to `|a|` is below `ε·|a_parent|` is dropped completely.

Why this is better than a fixed depth:
- It automatically gives you siblings when they matter (Jupiter for a Mars transfer)
  and ignores them when they don't.
- From Earth, the Jovian system is one point mass. Near Jupiter, it opens into Io,
  Europa, and so on.
- `θ` and `ε` are **fidelity knobs**, so a "casual" preset can go down to pure patched
  conics.
- Optional per-body **zonal harmonics** (J2, J3…) are the literal multipole part. J2
  matters for realism (sun-synchronous orbits, nodal precession). It costs almost
  nothing.

Cost: a typical vessel evaluates about 3–10 point masses per force call, which is cheap.

### 5.3 Propagating vessels: three regimes

| Regime | When | Method | Warp |
|---|---|---|---|
| **Analytic** | Perturbation ratio below ε, no thrust | Kepler (universal variables) | Unlimited, exact |
| **Perturbed coast** | Perturbations matter | Encke's method (integrate the deviation from an osculating conic) with an adaptive high-order integrator (DOP853 or IAS15-style Gauss–Radau) | Very high; the step is set by dynamics, not frame rate |
| **Physics bubble** | Near the player, thrusting, atmosphere, contact | Rigid-body engine (Jolt) in a local float frame, on a fixed substep | Low (≤ ~4×, "physics warp") |

Design rules:
- **The sim step is decoupled from the frame rate.** The propagator decides its own step
  size. The renderer interpolates between the latest two snapshots. That is how 0.01×
  and 10⁶× can both be stable.
- **Thrusting under warp** (ion engines, long burns) stays in the perturbed regime with
  thrust as a force term. Only contact and aero need the physics bubble.
- **Prediction = simulation.** The map-view trajectory predictor uses the *same*
  propagator and force model as the sim. Otherwise plans disagree with reality.
- Vessels don't interact with each other, so each vessel's propagation is independent.
  I'd relax the "sim is single-threaded" rule for **pure** per-vessel propagation jobs
  (a thread pool, no ECS mutation). All mutations stay on the sim thread.

References to read (not copy): Rein & Spiegel 2015 (IAS15), Rein & Tamayo 2015
(WHFast/Wisdom–Holman), Hairer–Nørsett–Wanner (DOP853), Battin (Encke and
universal variables).

### 5.4 Small bodies

Asteroids and comets are on rails by default, like vessels in the analytic regime. When a
mission targets one (proximity operations), it gets promoted: its gravity is added to that
vessel's force set, using a point mass or a polyhedral model for irregular shapes.

---

## 6. Ghost timelines and worldlines

### 6.1 Core idea

Every non-anchor object has a **worldline**: an ordered list of segments that together
cover `[t_start, t_end]` in universe coordinate time. When you finish flying a mission
and "go back" to fly another one, the first mission's worldline is **committed**. It
replays as a *ghost* while you fly the second.

```cpp
namespace helios::worldline {

struct KeplerSegment     { time::Interval span; frames::FrameId frame; orbital::Elements elements; };
struct ChebyshevSegment  { time::Interval span; frames::FrameId frame; ChebyshevFit3 fit; };  // compressed integrated arc
struct SurfaceSegment    { time::Interval span; frames::FrameId body_fixed; GeodeticPosition where; };
struct AttachedSegment   { time::Interval span; ObjectId host; math::Transform offset; };      // docked / carried

using Segment = std::variant<KeplerSegment, ChebyshevSegment, SurfaceSegment, AttachedSegment>;

// Evaluate a committed worldline at any time. O(log n) to find the segment, then O(1).
[[nodiscard]] core::Result<FrameState> evaluate(const Worldline& line, time::Epoch t) noexcept;

} // namespace helios::worldline
```

- We store **states, not inputs.** Replaying from recorded inputs would need bitwise
  determinism across compilers and CPUs, which is fragile. Chebyshev-fitted arcs (the
  JPL approach) compress integrated trajectories very well.
- Events (staging, docking, resource transfers, landing) sit on the worldline with
  timestamps.

### 6.2 Causality rules

**Decided:** a worldline is a frozen trajectory, and it never changes while it stands.
The only thing that can change history is an **interaction** with a ghost: a collision,
or its destruction. When that happens, we **taint** and roll back everything downstream
of the interaction.

#### The interaction graph

Worldlines are nodes. **Interaction events** are timestamped edges between them: dock,
undock, resource transfer, collision, crew transfer. Together they form a DAG in time.

```
 A ──────●dock(t1)──────●undock(t2)──────────►
         │               │
 S ──────●───────────────●──────●transfer(t3)─►
                                │
 B ─────────────────────────────●─────────────►
```

When an interaction hits object `X` at time `t` (for example, your live vessel rams
ghost `S`):

1. `X`'s worldline is **truncated at `t`**. After `t`, the damage model decides what
   `X` is: destroyed, broken into debris, or knocked off its path (then live).
2. Every interaction edge on `X` after `t` becomes **tainted**. That taints the other
   worldline from that edge onwards, and so on transitively. This is a forward BFS over
   the DAG.
3. A tainted worldline segment is **undefined** in the game: it is cut off at the
   tainted event. What remains of the mission from there is shown as "unresolved" and
   must be re-flown, re-run by its automation or route template, or abandoned.

Rules:

| Situation | Rule |
|---|---|
| Live vessel meets a ghost | The ghost is a solid kinematic body. Contact is a collision interaction and triggers the taint cascade above. |
| Docking with a ghost | Only at or after the ghost's worldline end ("frontier"), where it becomes live. |
| Resource transfer | Station inventories are **time-indexed ledgers**. A new event is accepted only if `inventory(t') ≥ 0` for all later `t'`. If it would go negative, the conflicting future events are tainted (the same mechanism as above). |
| Accidental damage | **Undo**: every cascade is a single transaction on the worldline store, so the game can revert it. Before a collision that would taint committed missions, show a warning such as "this will invalidate 3 missions". |

The worldline store is transactional and append-only (a cascade adds truncation and
taint records; it doesn't erase data). That makes undo, replays and multiplayer
conflict resolution the same operation.

Useful side effect: in single-player, you can fly the Mars lander and the Venus probe
"in parallel" without KSP-style warp juggling.

### 6.3 Multiplayer is the same system

Each player has their own "present" `t_p`. The server stores all worldlines and enforces
the ledger rules. Two players interact live only when their presents coincide, and
either can warp forward to meet the other. This generalises the "subspace" approach used by
community multiplayer mods for other games. It needs no lockstep and no determinism, just
a server-authoritative worldline store.

### 6.4 4-vectors and relativity: where they matter and where they don't

What I suggest:
- **Events are 4-vectors** `(t, x)` in a stated frame, with `t` = universe coordinate
  time. This is the natural key for worldlines anyway.
- **We keep one global coordinate time (a preferred frame).** **Decided:** a chosen
  reference object defines the universe clock and the distance scale. For the stock
  Solar System that is the Sun (barycentric, like TCB). For galactic-scale universes it
  is the galactic root frame. Stars move at about 10⁻³ c relative to each other, so
  Lorentz boosts *between anchor frames* are unnecessary. They would also make "the
  universe at time t" ill-defined, which breaks both the ghost system and multiplayer.
  Which object is the reference is a per-universe config value (moddable).
- **Fast vessels get relativistic kinematics**: relativistic momentum `p = γ m v`, thrust
  applied in the rest frame, and the relativistic rocket equation (rapidity
  `= (v_e/c) ln(m₀/m₁)`).
- **Every vessel and crew carries proper time**, `dτ/dt = √(1 − v²/c²) · (1 + Φ/c²)`
  (weak field). A crew that returns from a 0.9c trip is younger than the people who
  stayed home. Nothing breaks, because the universe clock is still `t`.
- **Optional rendering effects**: aberration and Doppler shift when you fly fast.
- Light-time delay for remote control could be a gameplay mechanic. It gives a physical
  reason to use automation (§8).

**Decided: no FTL in the stock game, but mods may add it.** Because there is a preferred
global time, an FTL drive is just a very fast worldline segment in coordinate time, and
it creates no time-travel paradoxes. The engine must not assume `|v| < c` anywhere
except the relativistic kinematics model, which an FTL mod replaces with its own
`PropulsionModel`. Long sub-light trips rely on time warp instead. For example, 0.1c to
Proxima is about 42 years of coasting. That is exact Kepler/rails propagation, so it can
run at arbitrarily high warp with very large steps.

---

## 7. Time warp

| Range | What runs |
|---|---|
| 0.01× – 1× | Everything, including the physics bubble, with smaller substeps at low warp |
| 1× – ~4× | "Physics warp" with bigger substeps (the player can opt in) |
| > 4× | No rigid-body physics. All vessels use regimes 1–2. Rails bodies are O(1). |
| ≥ 10⁶× | Same, plus a cap on how often we emit render snapshots |

- Warp is limited automatically near events: the next maneuver node, a domain change,
  atmosphere entry, or a scheduled automation action. The propagator provides
  "time-to-next-event" predictions.
- The maximum warp is a config value, not a constant. Slow interstellar craft may want
  10⁸×.

### 7.1 Everything warps, including chaotic background objects

Time warp advances **every tracked object** that is not a committed ghost, not just the
active vessel. Other live missions, debris and uncontrolled satellites all evolve under
the same laws of motion. Some of them sit in dynamically unstable places, such as
Lagrange-point orbits, and there numerical error behaves differently from physical drift.

**Physical instability is expected.** Around Sun–Earth L1/L2, perturbations grow roughly as
`e^{t/τ}` with `τ` ~ 3–4 weeks. The exact value depends on the orbit, so we should measure it
in the tests below. A seed error ε reaches order-one departure after about `τ·ln(1/ε)`:

| Seed | e-folds to O(1) | Time (τ ≈ 25 d) |
|---|---|---|
| Round-off, ε ≈ 1e-16 | ~37 | ~2.5 yr |
| Integrator tolerance, ε ≈ 1e-12 | ~28 | ~1.9 yr |
| Real-world perturbations (SRP, other bodies) | — | weeks to months |

So an uncontrolled halo orbit *must* eventually fall off. That matches reality, which is why
real L2 missions station-keep. Numerical error only picks the (random) seed. It cannot be
allowed to change the growth rate or make the result depend on how you watched it.

**Rules:**

1. **Warp-invariant trajectories.** An object's step sequence is chosen by its integrator's
   error control and the dynamics. It is never chosen by the frame `dt × warp`. The sim
   samples each object's dense output at the requested time. At 1× and 10⁶× the
   integrator takes *identical* steps, so on the same build the trajectory is the same
   bit for bit. Warp only changes how often we look at it.
2. **Tight, local error control.** Use the Encke formulation relative to the local anchor
   frame plus a high-order adaptive integrator (§5.3), with tolerances that keep the seed at
   the ~1e-12 level or better.
3. **Invariant monitors.** Track the energy (2-body regime) or the Jacobi constant
   (restricted 3-body regime) per object. These are LumenLog metrics, so a drift that
   isn't physical shows up in a plot, not as a player bug report.
4. **Auto-park: a pinned state, not a controller.** Stations in low orbit or at a
   Lagrange point can be *parked* by the player. A parked object is **not integrated** and
   runs **no corrective controller**:
   - Its worldline becomes a closed-form reference segment. That is a Keplerian orbit with
     drag removed for low orbits, or the computed periodic halo/Lyapunov orbit (or the
     point itself) for Lagrange stations.
   - Propellant is debited as a **closed-form function of elapsed time**:
     `m(t) = m₀ · exp(−Δv̇ · (t − t₀) / v_e)`. Here `Δv̇` is the station-keeping cost rate
     (drag make-up from the atmosphere model's orbit-averaged density, or a few m/s per year
     for L2). There is nothing that fires "every N seconds", so the cost of 10 s and of 10
     years is one evaluation, and it cannot go unstable at any warp.
   - The time the propellant runs out is solved analytically. That instant is scheduled as
     an event (§7), and warp stops there. After it the object switches to free
     propagation and drifts or decays *physically*.
   - Un-parking (by the player, or on an event) starts free propagation from the pinned
     state at that instant.
   - Rationale: in KSP-like games, a sampled autopilot at high warp applies corrections
     with a huge effective `dt`. Its loop gain times `dt` exceeds the stability limit and
     the vessel shakes itself apart. Auto-park removes the loop entirely.
5. **General rule for controllers under warp.** Any closed-loop controller (an autopilot,
   the RCS attitude hold, a player's Luau script) runs on a **fixed physical control
   period**, sub-stepped inside the propagator. It never runs on the frame `dt × warp`.
   If warp would need more sub-steps per frame than the budget allows, warp is capped
   while the controller is active. Controllers that must survive high warp need an
   analytic equivalent, like auto-park, or must hand off to one.
6. **Ghosts never drift.** Committed worldlines are frozen and evaluated, never
   re-integrated.

**Validation tests** (planned for Phase 1):
- Integrate a known halo orbit.
- Check that the measured divergence rate matches the linearised (monodromy) eigenvalue.
- Check that the trajectory is identical when the same span is stepped at different
  warp factors.

---

## 8. Planning, guidance and automation

### 8.1 Planning ("flight computer")

| Tool | Method |
|---|---|
| Launch windows / porkchop plots | Lambert solver (Izzo 2015) over a departure × arrival grid |
| Transfer design | Patched-conic initial guess, then differential correction in the full force model |
| Maneuver nodes | Impulsive nodes → finite-burn conversion |
| Low-thrust transfers | Q-law (Petropoulos) or Sims–Flanagan |
| Lagrange / halo orbits | Differential correction in the CR3BP, then refined in the full model |

### 8.2 Guidance (autopilots)

| Task | Algorithm family |
|---|---|
| Ascent to orbit | Gravity turn + closed-loop PEG/UPFG-style terminal guidance |
| Powered landing / booster return | Convex-optimisation guidance (G-FOLD family, Açıkmeşe et al.) |
| Rendezvous & docking | Clohessy–Wiltshire / Yamanaka–Ankersen + PD docking controller |
| Atmospheric entry | Predictor–corrector bank-angle guidance |

### 8.3 The control bus: one design for IVA, VR, autopilots and netcode

Every vessel input (throttle, attitude, action groups, individual switches) is a named
**signal on a control bus**. Anything can publish to it:

```
keyboard/gamepad ─┐
VR hand on switch ─┼─►  ControlBus (per vessel)  ─►  vessel systems (engines, RCS, …)
autopilot script ──┤
network (remote) ──┘
```

This unifies four features. A cockpit switch in IVA, a key binding, a guidance
algorithm and a remote player all do the same thing. Priorities and arbitration live in
one place (for example, the pilot can override the autopilot).

### 8.4 Automation and logistics

- **Scripts**: player-written autopilots in a sandboxed VM with an instruction budget
  per tick. See §11 for the VM choice.
- **Routes**: a flown (or simulated) mission can be saved as a **route template**: its
  worldline plus resource events, re-anchored to a new launch epoch. Repeated runs are
  resolved *abstractly* (Δv, resource and timing checks) instead of being re-simulated.
  They still produce ghost worldlines, so you can watch them.

---

## 9. Environments and exploration

### 9.1 One "medium" model for air, water and anything else

```cpp
// Every force from a fluid (lift, drag, buoyancy) is computed from the same query.
struct MediumSample {
    double density;          // kg/m³
    double pressure;         // Pa
    double temperature;      // K
    double speed_of_sound;   // m/s
    math::double3 velocity;  // wind / current, in body-fixed frame
    MediumKind kind;         // Gas, Liquid, (modded: Plasma, …)
};
```

- **Buoyancy** is `ρ · g · V_displaced`, computed from the part's mesh clipped against
  the medium boundary. The same code handles boats, submarines, blimps and Venusian
  cloud cities.
- **Aero and hydro forces** use per-panel models chosen by regime:
  - Modified Newtonian impact theory for hypersonic flow. It works on *any* mesh, which
    is essential for user-designed shapes.
  - Characterised polars for wings (§10).
  - Cross-sectional area distribution for transonic wave drag.
- **Submarines**: hull crush depth comes from the material and design (§10). The ocean
  exposes density, pressure and temperature as functions of depth.
- Gas giants: layered atmospheres with pressure going to the 10⁶–10⁷ Pa range, with
  crush depth as the limit on how deep you can go.

### 9.2 Planets

- **Cube-sphere quadtree LOD** (chunked or CDLOD). Patch generation runs on worker
  threads, and noise can be evaluated on the GPU (standards §7).
- Terrain is defined by a **node graph** (moddable): base layer (real DEM *or*
  procedural) → erosion/detail → biomes → **features**.
- **Features** are what you asked for with "specific items at specific positions":
  data-driven stamps anchored at `(body, lat, lon)`, such as craters, launch sites,
  ruins or custom meshes, with local terrain deformation.
- **Precision**: noise must be evaluated in `double` (or relative to each patch's local
  origin). At an Earth-radius distance, `float` noise has 0.5 m ULP, which causes
  visible terracing.
- The real Solar System uses public elevation data (MOLA, LOLA, SRTM-class) as the base
  layer, plus procedural detail at the metre scale.
- Oceans: FFT waves for rendering, with a cheaper analytic wave height function that the
  sim uses for buoyancy.

---

## 10. Vessel design: "characterise, then reuse one level up"

This is the most novel part of the project. Summary of the concept:

```
 Level 0  Profile / sketch        (2D: airfoil, nozzle contour, tank cross-section)
            │  extrude / revolve / loft / sweep
 Level 1  Component               (geometry + material + a physics *model*)
            │  characterise  ─────────►  Datasheet (numbers only)
 Level 2  Part                    (components + interfaces: mounts, fluid ports, power, data)
            │  characterise  ─────────►  Datasheet
 Level 3  Assembly / stage
            │  characterise  ─────────►  Datasheet (Δv, TWR, mass props …)
 Level 4  Vessel
```

The **datasheet** is a small table the flight sim consumes: mass properties, `Isp(p_amb)`,
`F(throttle, p_amb)`, `C_L(α, M)`, heat tolerance, and so on. The sim never runs the
design solvers during flight. That makes flight cheap and makes the design tool free to
be as sophisticated as we like.

### 10.1 Engines as thermodynamic graphs

To get the "nuclear turbofan? sure. fusion drive? why not" flexibility, an engine should
**not** be a fixed type. It is a small graph of standard stages connected by flow ports:

```
[inlet] → [compressor] → [HEAT SOURCE] → [turbine] → [nozzle]
                              ▲
        combustor | fission core | fusion plasma | solar-thermal | beamed …  (moddable)
```

- A turbofan, a ramjet, a nuclear-thermal rocket, a nuclear turbofan and a
  fusion torch are just different graphs or heat sources.
- **Physical constraints balance the game for us**: energy and mass conservation, jet
  power `P = ½ ṁ v_e² / η`, and **waste heat that must be radiated**. So a fusion drive
  with high Isp *and* high thrust needs enormous reactors and radiators, and the design
  shows this.
- Chemical rockets: `Isp = C_F · c*/g₀`. `c*` comes from precomputed equilibrium
  combustion tables per propellant pair, and `C_F` from the nozzle expansion ratio and
  ambient pressure.

### 10.2 Wings and bodies

- Airfoil profile → 2D panel method → section polars.
- Wing planform → vortex-lattice method → 3D wing datasheet.
- Fuselages: slender-body theory or area ruling. Hypersonic: Newtonian panels.
- Mass properties (volume, centre of mass, inertia tensor) come from exact mesh volume
  integrals × material density. Structural strength comes from material and wall
  thickness.

### 10.3 Geometry kernel

"Simplified CAD" means parametric sketches plus a few operations (extrude, revolve,
loft, sweep, boolean, shell/wall thickness), **not** a full B-rep kernel. I suggest
meshes with robust booleans (for example the *Manifold* library, Apache-2.0). Designs are
stored as **parametric history**, not baked meshes, so a modder can tweak a parameter and
everything upstream re-characterises.

### 10.4 Structure: rigid by default

**Decided.** A vessel in flight is **one rigid body** with composite mass properties,
the way flight simulators do it. The physics engine never holds parts together with
joints. Topology changes are explicit operations on the part tree:

| Event | What happens |
|---|---|
| Staging / decoupling | Split the part tree at the decoupler into two vessels. Recompute each vessel's mass properties. Apply the decoupler impulse to both. |
| Docking | Merge two part trees into one rigid body. |
| Collision | Run the damage model **only now**: impact energy/impulse against part strength → parts break off (split) or are destroyed. |
| Over-stress | Cheap load estimate, below → break at the interface (split). |

#### Cheap stress estimate (O(parts) per tick)

The part tree is exactly what we need. For each attachment interface `j`, let `S_j` be
the sub-tree on the far side. The rigid body has linear acceleration `a`, angular
velocity `ω` and angular acceleration `α`. The interface must transmit the force and
moment that give `S_j` that motion, minus the external forces acting directly on `S_j`:

```
F_j = Σ_{i∈S_j} [ m_i (a + α×r_i + ω×(ω×r_i)) − F_i^{ext} ]
M_j = Σ_{i∈S_j} [ r_ij × (m_i (a + α×r_i + ω×(ω×r_i)) − F_i^{ext}) ]
```

Here `F^ext` is thrust, aero, buoyancy and ground contact on each part. Gravity cancels
because it is uniform over the vessel. One post-order traversal gives every `F_j`, `M_j`.
Compare the axial force, shear and bending moment against the interface's
characterised limits (from §10.2 materials). Exceeding a limit for a sustained time
breaks the interface. This catches the classic failures, like a long stack breaking from
aerodynamic bending at max-Q or a heavy payload tearing off at high g. Tick cost is
negligible.

Flexible structures (long trusses, tethers) can be an opt-in model later.

### 10.5 UI

- Desktop: a clean CAD-like workspace (sketch plane, gizmos, parameter panel, live
  datasheet).
- VR: the same operations on a "holographic table". The design tool is a strong showcase
  for VR, and it shares code with the desktop UI.

---

## 11. Modability

Principle: **the base game is itself a mod** (`helios.core`). Anything the base game can
do, a mod can do.

| Layer | Mechanism | Examples |
|---|---|---|
| Data | TOML/JSON definitions in a virtual filesystem with overlay/patch semantics | Bodies, star systems, materials, propellants, parts, terrain graphs |
| Scripting | Sandboxed VM with an instruction budget. Proposed: **Luau** (see §11.1) | Autopilots, mission logic, custom part behaviour |
| Native | C ABI plugins implementing our concepts through a stable C interface | New integrators, ephemeris models, heat sources, render effects |
| Models | Concept-constrained plug-in points | `EphemerisModel`, `ForceModel`, `HeatSource`, `MediumModel`, `TerrainNode` |

- ECS choice: **flecs** is worth serious consideration over EnTT because of its runtime
  component and reflection support. Mods can define new components *at runtime*, and
  those components serialise and show up in tools automatically.
- Multiplayer: the server advertises a content hash of the mod set, and clients must
  match. Native plugins can't be sandboxed, so servers should support "data + script
  only" modes.

### 11.1 Scripting VM comparison

| | Lua 5.4 / LuaJIT | **Luau** | WASM (wasmtime / WAMR) |
|---|---|---|---|
| What it is | The classic game scripting language (WoW, Factorio, Garry's Mod) | Roblox's open-source (MIT) Lua derivative, in C++ | Portable bytecode; mods written in Rust, C, C++, Zig, AssemblyScript… |
| Who writes mods | Anyone; edited live | Anyone; edited live; optional type annotations | Programmers with a compiler toolchain |
| Sandboxing untrusted code | Manual and error-prone | **Built-in design goal** (runs untrusted code from millions of users) | Very strong (memory isolated) |
| Instruction budget | Debug hooks (disable the LuaJIT JIT) | Interrupt callback | "Fuel" metering |
| Speed | LuaJIT: very fast. Lua 5.4: moderate | Fast interpreter, optional native code generation | Near native |
| Risks | LuaJIT is stuck on Lua 5.1 semantics and has sporadic maintenance | Smaller ecosystem outside Roblox | Heavy to embed; no live in-game editing |

**Recommendation:** use Luau for in-game player scripts (kOS-style autopilots you write
in the cockpit) and gameplay mods. It is the only option designed around sandboxing,
live editing and budgets together. Heavy native extensions use the C ABI plugin layer.
WASM can come later as a sandboxed high-performance tier that is safe for multiplayer.

---

## 12. IVA, FPV and VR

- Crew modules are designed with **interiors**: seat, panels, windows. Panel controls
  publish to the control bus (§8.3).
- The character controller runs in the **vessel's local frame**, with fictitious forces
  (thrust acceleration, rotation) applied. You feel the g-load, and in zero-g you float.
- **VR comfort**: the cockpit is a stable visual reference frame, which is the most
  comfortable kind of VR locomotion. External view uses teleport/snap-turn.
- **OpenXR** is the target API. **[OPEN] renderer**: bgfx has no first-class OpenXR
  path, so integrating it means wrapping swapchain images through low-level backend
  hooks. VR is not the top priority, but it is hard to retrofit. So we build the
  architecture VR-ready from day 1 and ship VR later.

### 12.1 VR-ready rules (apply from the first renderer commit)

These are cheap now and very expensive to retrofit:

1. **The renderer draws a list of views, never "the camera".** A view is a pose plus a
   projection. Desktop has 1 view and VR has 2 (stereo, instanced if possible). All
   post-processing works per view.
2. **The head pose is relative to a seat/anchor frame**, not to the world. Desktop
   mouse-look just produces a head pose.
3. **All in-cockpit UI is world-space (diegetic)**: instruments, MFD screens and
   switches are 3D objects that render to textures. Only menus and dev tools are
   screen-space overlays.
4. **Input goes through the control bus (§8.3)** and an action-based input layer
   (like OpenXR actions). Keyboard bindings and hand interactions map to the same
   actions.
5. **Frame pacing**: the sim runs decoupled from rendering (§5.3). The render thread
   must be able to hit 72–90 Hz on its own.

### 12.2 Testing VR without a headset

- **Monado** (open-source OpenXR runtime, Linux/Windows) has a *simulated HMD* driver
  and a headless mode. It can run in CI to check that the OpenXR session, swapchain and
  frame loop work.
- **Meta XR Simulator** (Windows) emulates a headset and controllers with keyboard and
  mouse.
- Once the path works, a community tester with real hardware checks comfort and
  performance. Most VR bugs are in the pipeline plumbing, which the simulators catch.

---

## 13. Tech stack (proposed)

| Concern | Choice | Notes |
|---|---|---|
| Language / build | C++23, CMake ≥ 3.28, vcpkg manifest mode, presets | Same setup as LumenLog |
| Errors | `helios::core::Result<T>` = `std::expected<T, Error>` | Per standards §1 |
| Logging / telemetry | **LumenLog** | See §14 |
| Tests | GoogleTest, plus reference-data tests (JPL Horizons vectors, known L-point positions, energy-drift bounds) | Headless sim tests are the backbone of CI |
| Rigid body | Jolt Physics (`JPH_DOUBLE_PRECISION` evaluated) | Physics bubble only |
| ECS | flecs (or EnTT) | §11 |
| Rendering | bgfx **[OPEN, pending VR spike]** vs Vulkan-direct | |
| XR | OpenXR | |
| Geometry | Manifold (booleans), cgltf (assets) | |
| Scripting | Luau (proposed, §11.1) | |
| Networking | Server-authoritative worldline store; transport TBD (e.g. GameNetworkingSockets or ENet) | No lockstep needed |

---

## 14. LumenLog integration

LumenLog fits this project well:

- **Context layers map to our structure.**
  - Process tags: build, mod set hash.
  - Thread tags: sim, render, terrain worker.
  - `lumen::Tagged` instance tags on each system and vessel: `vessel_id`, `frame`,
    `timeline_id`.
  - `LUMEN_FRAME_SCOPE` for sim ticks.
  - So `LOG_DEBUG("domain change")` inside the propagator automatically says which
    vessel, frame and timeline it came from.
- **Metrics are ideal for physics diagnostics.** For example, energy drift, integrator
  step size, Encke rectification count and per-tick propagation cost. All of these can
  go to a JSONL sink we plot in Python.
- **Sinks as virtual classes** are "genuine runtime polymorphism", so they fit standards §2.

Status of the integration (Helios pins LumenLog `28e1d44`):

1. **Runtime queries: done upstream, used by Helios.** Every Helios sink is configured
   with a query string (`LoggingOptions::terminal_query` / `json_query`, default
   `level >= INFO` / `true`). The environment variables `HELIOS_LOG_TERMINAL` (or `off`),
   `HELIOS_LOG_JSON` and `HELIOS_LOG_JSON_QUERY` override them without recompiling. A bad query
   comes back as a `ParseFailure` with the column marked, ready for the future dev console
   (`log.sink add file run.log "<query>"`).
2. **Numeric predicates: done upstream.** `altitude_m < 70000` works in queries, and numeric
   tags are stored exactly.
3. **Exceptions: resolved upstream.** Built-in sinks never throw on I/O. An exception from a
   Helios-written sink is contained by Lumen and counted (`sink_exception_count`). Only sink
   construction (thread creation) can throw, and `initialise_logging` wraps that in `try_call`.
4. **Fixed upstream in 7858fbb:** records own their strings, `LOG_*` takes `std::format`
   arguments, `flush()` blocks, and the library is subproject-safe.
5. **Open, blocks macOS:** LumenLog's private member `__used` (`record.h`) collides with the
   `__used` macro from Apple's `<sys/cdefs.h>`, so LumenLog does not compile on macOS. The C++
   standard reserves every identifier containing `__`. The durable fix is to rename all ~100 of
   LumenLog's `__name` identifiers to `name_`, the convention Helios adopted for exactly this reason.
6. Helios uses LumenLog's `LOG_*` macros directly, with no project wrapper.

---

## 15. Roadmap (first pass)

Each phase ends with something demonstrable and a CI-tested headless core.

| Phase | Deliverable |
|---|---|
| **0. Foundations** ✅ started | Repo skeleton, CMake/vcpkg presets, clang-tidy, CI, `core::Result`, LumenLog wired in, math types |
| **1. Headless universe** 🚧 frames + body models done ([validation](validation/ephemeris/README.md)) | `Epoch`, frame tree, Kepler + Chebyshev ephemerides, tree-code gravity, Encke + adaptive integrator. Validated against JPL Horizons; an L2 halo orbit stays bounded. |
| **1b. VR spike** (parallel) | OpenXR on Monado's simulated HMD + bgfx, rendering a cockpit box in stereo at 90 Hz. Confirms or rejects bgfx. |
| **2. Map view** | Minimal renderer: spheres, orbit lines, floating origin, time warp 0.01× → 10⁶× |
| **3. Flight** | Data-defined parts, datasheets (hand-written at first), rigid vessel, Jolt bubble, staging, on/off-rails transitions, the control bus |
| **4. Pilot's seat** | IVA interior, cockpit controls on the bus, character controller in the vessel frame, VR |
| **5. Worldlines** | Recording, Chebyshev compression, ghosts, ledgers, causality rules |
| **6. Planning & guidance** | Lambert/porkchop, maneuver nodes, ascent and landing autopilots, scripting VM |
| **7. Worlds** | Cube-sphere terrain, node graphs, features, atmosphere, oceans, buoyancy |
| **8. Design tool v1** | Sketch → extrude/revolve, mesh mass properties, engine graph, wing VLM → datasheets |
| **9. Multiplayer** | Server worldline store, presents/subspaces |
| **10. Galaxy** | Procedural galaxy, star instancing, relativistic kinematics, proper time |

---

## 16. Decisions

### Decided

| # | Topic | Decision |
|---|---|---|
| D1 | Frames vs dynamics | Separate systems. The force set includes the children of the frame's system, so e.g. Sun–Earth L2 works (§4.2, §5.2). |
| D2 | Gravity truncation | Barnes–Hut-style opening criterion on the body tree (§5.2) |
| D3 | Time type | `Epoch {int64 seconds, double fraction}` (§3.2) |
| D4 | Universe clock | A per-universe reference object (the Sun for stock) defines coordinate time and distances. Relativistic kinematics and proper time for vessels (§6.4). |
| D5 | Ghosts | Frozen 4-vector worldlines. They are collidable, and interactions taint and roll back downstream history, with undo (§6.2). |
| D6 | Control bus | One bus for all inputs (§8.3) |
| D7 | Flight realism | Plausible, not exact. ≥ 60 FPS on min spec, with realism scaling by budget (§2.1). |
| D8 | Structure | Single rigid body per vessel. Explicit split/merge. Damage only on collision or over-stress (§10.4). |
| D9 | FTL | Not in stock, allowed for mods (§6.4) |
| D10 | VR | Not first to ship, but VR-ready from day 1 (§12.1). Tested on simulators (§12.2). |
| D11 | Naming | Namespace `helios::`. Standards are adapted in `docs/CODING_STANDARDS.md`. |
| D12 | Licence | MPL-2.0 for code (assets licensed separately, e.g. CC-BY-SA) |
| D13 | Scripting VM | Luau (§11.1) |
| D14 | Warp invariance | Trajectories must not depend on warp factor (§7.1) |
| D15 | Auto-park | Parked stations are pinned closed-form segments with closed-form propellant cost; no interval corrections. Controllers run on fixed physical periods (§7.1). |
| D17 | Stock ephemeris | Real Solar System = JPL DE Chebyshev series in a Helios `.hce` file (DE421 now; DE440 when the data pipeline can fetch it). Mean elements (Keplerian + rates) for procedural systems and as fallback; Uranus–Pluto fitted around the SSB. Validation: `docs/validation/ephemeris/`. |
| D16 | Conventions | Clang ≥ 19 (Apple clang on macOS is a primary platform), header guards, GoogleTest, members `name` / `name_` (non-public), SI unit suffixes in names |

### Open

1. **Renderer**: bgfx (favoured by the min-spec requirement, §2.1) unless the VR spike fails.
