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
- **The bubble does not take the orbit away from the propagator** (D25). The rigid-body
  engine works in a small frame that starts each tick at the vessel's centre of mass. Away
  from the ground that frame falls with the vessel, so there is no gravity in it: the engine
  turns the vessel and reports the change of velocity the thrust gave it, which the
  propagator takes as an impulse in the middle of the tick. Near the ground the frame moves
  with the ground, the ground is a plane, and the engine also integrates the fall and resolves
  the contact; the propagator is restarted from its result each tick. A vessel at rest on the
  ground leaves the bubble and is pinned to the body-fixed frame: closed-form, like
  everything else on rails.
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

#### Parts define the signals (D22)

The bus is not a fixed list of inputs. **Each part declares how it wants to be handled**, in
its datasheet (§10), and the vessel's bus is the sum of its parts:

- **Inputs** the part accepts (`throttle`, `ignition`, `deployed`, `separate`): a range, a
  default, and whether it is a switch or a lever.
- **Outputs** it reports (`thrust_n`, `chamber_temperature_k`, `lox_kg`). An output is just
  something that can be read from outside; how the part arrives at it is its own business. In a
  datasheet it is a curve of how hard the part is running, or the content of a store.
- **Resources** it holds and exchanges. An engine may burn fuel and oxidiser, or one
  propellant, or xenon and electricity; a solar panel produces electricity from nothing. The
  flight code knows about *resources flowing at rates*, never about "engines" or "fuel".

Every port becomes a signal named `<part>/<port>` (`upper_engine/throttle`).

A few **standard interfaces** sit on top, each a named set of ports: `engine` =
{`ignition`, `throttle`} in, {`thrust_n`, `mass_flow_kg_s`} out. A part that declares
`interfaces = ["engine"]` must have those ports, and may have any others. The rest of the game
binds to interfaces, not to parts:

- The vessel gets one signal per interface input, `engine/throttle`, which every member's
  own input follows unless it is commanded directly (shut one engine down, the others stay
  on the common throttle).
- A cockpit gauge, an autopilot or a staging list written against `engine` works with a
  kerolox engine, an ion thruster and a part a modder adds tomorrow.

Besides its parts' signals a vessel has `staging/stage` (stages activated so far; a stage is
a list of commands published by the sequencer), `guidance/frame|x|y|z` (where the nose points)
and the readings `vessel/mass_kg`, `vessel/thrust_n`, `vessel/mass_flow_kg_s`.

Sources, in increasing priority: sequencer, autopilot, pilot. A source's command stays until
it changes or releases it. The value of a signal is the command of the highest-priority source
on it *or on the signal it follows*; between equal sources its own command stands, and with no
command it has its default. So the pilot's hand on `engine/ignition` shuts down an engine the
sequencer lit by its own signal, and an engine the pilot shut down by its own signal stays off
the common throttle.

The simulation adds what the vessel's navigation knows: `nav/altitude_m`,
`nav/vertical_speed_m_s`, `nav/surface_speed_m_s`, `nav/speed_m_s`. Cockpit instruments,
autopilots and scripts read them like any other signal.

#### On rails everything is closed-form (D22)

Between two changes (a command, or a store running out or filling up) every process runs at a
constant level, so every store changes at a constant rate. Amounts are therefore exact
functions of time, the instant a tank runs dry is known in advance, and the thrust is a
sequence of constant segments on a mass that falls linearly. The propagator integrates that as
a force term (§5.3), which keeps D14: a burn is bit-identical at any warp and frame length, and
the predicted path includes burns that are only planned. This is the on-rails model; rotation,
contact and aerodynamics belong to the physics bubble (D25). On rails the vessel holds the
commanded pointing exactly and only the thrust along its nose acts.

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

Put the other way round: designing a turbofan means choosing stages, bypass ratio, blade
angles and profiles, perhaps an afterburner, and trying it on a **test bench** in the editor to
see what it does. None of that reaches flight. What is kept is the bundle the bench
measured: the interface (§8.3: which controls the engine takes, which resources it uses, what
it reports) and the curves that relate them (thrust, consumption, temperatures, drag). The
blade angles stay in the design file, as parametric history (§10.3), for the day it is
edited again.

As of Phase 3 the datasheets are written by hand (`data/parts/*.toml`, format in
`data/parts/README.md`): mass and inertia, inputs, outputs, stores, and processes that
exchange resources and produce thrust. The design tool of Phase 8 will write the same files.

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

**Status (D26, D27).** In: a cockpit is a table in the part's datasheet (seat, boxes,
instruments bound to bus signals by name), the flight view shows the vessel from the seat or
from outside, keys and handled instruments both become commands of the *Pilot* source, and the
pilot can leave the seat of a vessel with a cabin and move about in it
([flight view](flight_view/README.md)), with loose items to carry and throw. Still to come:
EVA, text on the panels (rule 3 below), and VR itself.

#### Moving about: nothing but contact (D27)

Inside a vessel the pilot moves the way a crew does aboard a station. **Nothing moves a body
except what it touches**: there are no thrusters indoors, on any platform.

- **Weightless**, the pilot hangs on to surfaces and launches from one to the next: take hold
  of what is in reach, pull in or push away with the arm, let go and keep the speed. A body in
  mid-air keeps its velocity until it meets something. Loose items can be picked up and
  thrown, and throwing one sends the pilot the other way with the same momentum: the way to
  get moving when nothing is in reach.
- **With weight** (on a planet, or under thrust) the pilot stands and walks on whatever is
  "down". There is one rule for both: the cabin's gravity is the opposite of the vessel's
  proper acceleration. So the pilot floats in free fall, stands on the Moon, and is pressed to
  the aft wall when the engine lights.
- **In VR** the hands do the holding. **On a flat screen** the hand is where the pilot looks:
  hold the button to grab, the movement keys work the arm, and an arm pushed out straight lets
  go (a push-off). The simulation is the same; only how the hand is driven differs.

**One key for "the obvious thing" (D28).** Controls stay few, and what a key does depends on
where the pilot is: the same key leaves a seat, sits down, picks a tool up, and later will open
an airlock or take the wheel of a rover. The simulation works out each frame what that key
would do now (an *offer*), the picture says it in words next to the key, and pressing the key
does it. Which key it is belongs to the bindings, not to the game's logic.

**Two representations of a vessel.** From outside a vessel is one rigid body: unless something
is said to move, everything in it is taken to be at rest relative to its centre of mass. That
is cheap, and it is all that is needed to fly it, to look at it, or to leave it on rails. The
inside is simulated only while someone is moving about in it. Then the cabin is a small
physics world in the vessel's frame, and the pilot is part of what moves: every push the cabin
gives the pilot is given back to the vessel, so kicking off moves it a little the other way
and running round a spinning cabin makes it wobble, by as much as the masses say.

**EVA** (not built) is planned on the same lines: a suit is a small vessel of its own (parts,
thrusters, oxygen and propellant, a control bus), a tether is a constraint between two bodies,
and the airlock cycle is where the detailed interior is swapped for the outside representation
and back, so its loading is hidden by putting the suit on and pumping the airlock down.

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

Status of the integration (Helios pins LumenLog `5b228ba` as the git submodule `extern/LumenLog`):

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
5. **Fixed upstream in 5b228ba:** LumenLog's `__name` members (which collided with Apple's
   `__used` macro) were renamed to `name_`; it builds on macOS.
6. Helios uses LumenLog's `LOG_*` macros directly, with no project wrapper.

---

## 15. Roadmap (first pass)

Each phase ends with something demonstrable and a CI-tested headless core.

| Phase | Deliverable |
|---|---|
| **0. Foundations** ✅ started | Repo skeleton, CMake/vcpkg presets, clang-tidy, CI, `core::Result`, LumenLog wired in, math types |
| **1. Headless universe** ✅ ([ephemerides](validation/ephemeris/README.md), [dynamics](validation/dynamics/README.md)) | `Epoch`, frame tree, Kepler + Chebyshev ephemerides, tree-code gravity, Encke + adaptive integrator, rotating frames, SOI domains. Validated against JPL DE440 (originally DE421), an independent Cowell/CR3BP integrator and L1/L4 linear theory. Halo-orbit construction moves to Phase 6 (planning). |
| **1.5 Headless groundwork** ✅ | Conic geometry and event timing for every eccentricity, impulsive manoeuvres that keep trajectories warp-invariant, predicted events (apsides, SOI exit/entry, impact) and exact trajectory predictions, the event-limited time-warp controller, `sim::Simulation` and focus-relative `SceneSnapshot`s |
| **1b. VR spike** (parallel) | OpenXR on Monado's simulated HMD + bgfx, rendering a cockpit box in stereo at 90 Hz. Confirms or rejects bgfx. Needs Linux or Windows: macOS has no OpenXR runtime. |
| **2. Map view** ✅ ([map view](map_view/README.md)) | bgfx + SDL3 + Dear ImGui client: lit spheres, orbit and trajectory lines, labels and apsis markers, floating origin, reversed-Z infinite projection, time warp 0.01× → 10⁶× with event limiting, burn planner, deterministic screenshot runs in CI |
| **3. Flight** ✅ | Data-defined parts and datasheets (hand-written), the part tree with composite mass properties, the control bus built from the parts' interfaces, resources and processes in closed form, staging and separation, finite burns in the propagator, vessel controls in the map view (D22). The physics bubble on Jolt: rigid-body rotation, reaction wheels and attitude hold, contact with the ground, landing and lift-off, and the transitions on and off rails (D23, D25). |
| **4. Pilot's seat** (three parts ✅, [flight view](flight_view/README.md)) | Done: cockpits as data (seat, solids, instruments bound to bus signals), the view from the seat and from outside, the ground under the vessel, keys and handled instruments as bus commands, navigation readings on the bus (D26); the pilot out of the seat, floating and holding on in free fall and walking under weight, with the reaction on the vessel, and loose items to carry and throw (D27); one interact key whose meaning the simulation offers (D28). To do: EVA, diegetic text, VR. |
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
| D17 | Stock ephemeris | Real Solar System = JPL DE Chebyshev series in a Helios `.hce` file (DE440, 1550–2650; DE421 until 2026-10-01). Mean elements (Keplerian + rates, fitted to DE440 over 2000–2200, the period the game is played in) for procedural systems and as fallback; Uranus–Pluto fitted around the SSB. Validation: `docs/validation/ephemeris/`. |
| D18 | Propagation | Encke + Dormand–Prince 5(4) with step-boundary-only events (bit-identical at any warp); tree-code gravity default θ = 0.25; SOI hysteresis 5 %. Validated in `docs/validation/dynamics/`. |
| D16 | Conventions | Clang ≥ 19 (Apple clang on macOS is a primary platform), header guards, GoogleTest, members `name` / `name_` (non-public), SI unit suffixes in names |
| D19 | Client stack | bgfx (renderer; avoids a hand-written Vulkan/Metal pipeline), SDL3 (window, input), Dear ImGui (debug and early UI), OpenXR loader (VR, later). Fetched at pinned commits with FetchContent (bgfx needs its host shader compiler at configure time); built only with `HELIOS_BUILD_CLIENT`. The simulation never links them. |
| D20 | Warp and manoeuvres | Warp is limited continuously so the next burn / SOI change / impact stays ≥ 0.5 s of wall time away, landing exactly on it; burns and impacts drop warp to real time. Impulses truncate the step that contains them with its own dense output, so a burn is bit-identical whether planned in advance or scheduled live. Map-view paths are sampled from a copy of the vessel's propagator, not drawn as conics. |
| D21 | Analytic regime and maximum warp | The analytic regime of §5.3 lives inside the Encke propagator: at a step boundary, a bound orbit that cannot leave its domain or reach a child's sphere, and whose perturbing acceleration stays below ε of the central gravity over the next 10 days (sampled at four points of the orbit and five instants, with a factor 2 of margin), follows its osculating conic for that segment; the test is repeated at every segment end. The decision uses the dynamics only (never the sampling instants or pending impulses), so D14 holds and an impulse truncates a segment like a step. ε = 10⁻⁶ in the simulation (low orbits; geostationary, lunar and interplanetary trajectories keep their perturbations), 0 = off in the propagator itself. Warp levels reach 10⁹× (`SimulationOptions::max_warp_factor`). For frames that long, warp is also limited by a domain change the propagator has not made yet, events are re-predicted as time passes, and a frame never advances beyond the horizon of the last event search. |
| D22 | Parts, datasheets and the control bus | A part is a datasheet (TOML): mass properties, inputs, outputs, stores, and processes that exchange resources at rates set by an input and may produce thrust; outputs are curves of a process level or store contents. The vessel's control bus is built from its parts' ports (`<part>/<port>`), plus one signal per input of each standard interface in use (`engine/throttle`), which members follow unless commanded directly (§8.3). Stages are lists of commands; a separator detaches its subtree as a new vessel. On rails all of it is closed-form between changes, so stores, burn-out instants and thrust do not depend on warp, and thrust enters the Encke propagator as constant segments on a linearly falling mass, truncating steps like impulses (D14, D20). Vessel axes: +x to the nose; on rails the nose holds the commanded direction (inertial, or prograde / normal / radial-out) and only thrust along it acts. |
| D23 | Rigid-body engine | Jolt Physics 5.6 (approved by the maintainer on 2026-10-01), from vcpkg with RTTI, single precision, its single-threaded job system. Only `source/physics` sees it, behind `physics::World`. |
| D24 | Data files | TOML for definitions (parts, vessels). Read by a small in-tree reader of the subset in use (`core/toml.hpp`), which rejects what it does not support, so every accepted file is valid TOML; replacing it with a full parser later needs no data change. |
| D25 | Physics bubble | Holds one vessel: the active one (the vessel the player flies), while the warp is ≤ 4×. Fixed tick of 1/128 s (an exact binary fraction). The Encke propagator keeps the vessel's place and velocity; Jolt supplies the rotation and what is not gravity (§5.3). In the bubble the attitude is state: torques from reaction wheels and off-axis thrust turn the vessel, and the attitude hold is an autopilot on the control bus that commands the `attitude` interface once per tick (§7.1 rule 5). On rails the nose is taken to be on the commanded pointing; entering the bubble starts from that, and leaving it hands the thrust in effect back to the plan. Contact: each part with a shape is a cylinder; touching the ground faster than the least `impact_tolerance_m_s` destroys the vessel; at rest it becomes `Landed`, pinned to the rotating body, and lifts off again when its engines push while it is the active vessel. Not warp-invariant by nature (D14 is for the rails). Pointing frames: inertial, orbital (prograde / normal / radial-out), local (forward / normal / up). |
| D26 | Cockpits and the flight view | A crewed part's datasheet has a `cockpit`: the seat (eyes, forward, up, in part axes) and, in the seat's own axes, boxes and instruments. An instrument (dial, lever, switch, button) names a signal of the *vessel's* bus, so a panel works in any vessel that has the signal and is dead in one that has not. The simulation reports `nav/…` readings on every bus. On the bus a higher-priority source on a leader beats a lower one on its follower (§8.3). Input is two layers: devices → actions (`Action`, bound to keys by position), actions and handled instruments → *Pilot* commands; steering is asked for in the seat's axes. The head pose is relative to the seat. The flight view's geometry is GPU-free like the map's; the ground near the camera is a cap of the body's sphere rebuilt each frame in camera-relative doubles, with the body's latitude/longitude grid drawn by the shader. Provisional until there are models: boxes and cylinders for everything, instrument text as an overlay, no hull from inside, no shadows. |
| D27 | The pilot in the cabin | A cockpit with `walkable` is a cabin; its boxes (some of them `glass`) are its walls. While the pilot is out of the seat the cabin is a `physics::World` of its own in the vessel's axes, with one moving body (a ball about the chest, 80 kg). Its gravity is minus the vessel's proper acceleration (`Vessel::proper_acceleration_m_s2`: thrust over mass, the push of the ground, or what the bubble measured), plus the centrifugal, Coriolis and Euler terms of the vessel's turning. The pilot is moved only by contact: legs (a damped spring to the surface below, when the weight is ≥ 0.5 m/s²), walking (no harder than the weight on the feet allows), and a hand that holds a surface within reach of the eyes while the arm moves the body relative to it and lets go when pushed out straight. Everything the cabin does to the pilot is returned to the vessel as a force at the pilot's place while the vessel is in the physics bubble; a vessel on rails or on the ground does not feel it. Outside the bubble's warp the pilot stays put relative to the vessel. Where the pilot looks belongs to the player, not to the simulation (it follows the mouse or the headset at once) and is sent in as part of `PilotInput`. Not yet counted: the pilot's mass in the vessel's while seated. |
| D28 | Offers: one interact key | The simulation computes what the pilot can do where they are (`PilotOffer`: leave the seat, sit down, pick up the item looked at, let go of the item in hand; more kinds as the game grows) and `Simulation::interact()` does it. The client shows the offer as a prompt beside the key and a line of hints for the other controls as things stand; it decides nothing itself. A cabin's loose items (`[[part.cockpit.item]]`: a box with a mass) are bodies of the cabin's world, which runs whenever the pilot is aboard, seated or not. An item in hand is part of the pilot's body (its mass added); picking up joins the two momenta, throwing parts them at a relative speed set by a fixed impulse (40 N s, between 1 and 5 m/s), so a heavy item moves the thrower more. The reaction on the vessel is the sum over the pilot and every loose item, as a force and a torque. Legs take weight only after it has lasted 0.25 s, so a jolt is not a floor. Items are put back where their datasheet has them when the pilot boards anew. |

### Open

1. **Renderer**: bgfx is in use (D19); the VR spike (1b, on Linux or Windows) still has to confirm OpenXR swapchain interop.
