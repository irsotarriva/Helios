# Helios — Coding Standards

These rules apply to every `.cpp` and `.hpp` file under `include/`, `source/`,
and `test/`.  They are enforced by clang-tidy (CI) and code review.

---

## 1. Error Handling

### 1.1 Internal code never throws

Every function in the project that can fail **must** return
`helios::core::Result<T>` or `helios::core::VoidResult`
(`std::expected<T, core::Error>`, where `Error` = `std::error_code` + context string;
see `include/helios/core/error.hpp`).

```cpp
// ✅ Correct
[[nodiscard]]
core::Result<double> solve_kepler(double M, double e) noexcept;

// ❌ Wrong — never use throw internally
double solve_kepler(double M, double e);  // throws on failure
```

### 1.2 Catch external library exceptions at the boundary

Any call into a third-party library that may throw must be wrapped with
`core::try_call()` so exceptions cannot propagate into project code.

```cpp
// ✅ Correct — Jolt / cgltf calls wrapped
auto result = core::try_call(
    core::ErrorCode::FileNotFound, "cgltf_parse_file failed",
    [&] { return cgltf_parse_file(&options, path.c_str(), &data); }
);
if (!result) return std::unexpected(result.error());

// ❌ Wrong — exception can escape
auto data = cgltf_load(path);  // if this throws, propagation is undefined
```

### 1.3 Propagate errors with `std::unexpected`, not by ignoring them

```cpp
// ✅ Correct — early-return on error
auto state = evaluate_at(elements, ut, mu);
if (!state) return std::unexpected(state.error());

// ✅ Also correct — std::expected monadic chaining
return evaluate_at(elements, ut, mu)
    .and_then([&](OrbitalState s) { return transform(s); });

// ❌ Wrong — silently discarding an error
auto state = evaluate_at(elements, ut, mu);
use(*state);  // UB if state is an error
```

---

## 2. Concepts over Inheritance

Use C++23 `concept` to express interface requirements on template parameters.
Reserve `virtual` for genuine runtime polymorphism (e.g., bgfx back-end
strategy, platform window abstraction).

```cpp
// ✅ Correct — constrain a template with a concept
template<helios::concepts::SimulatedEntity T>
void update_all(std::vector<T>& entities, double dt) { … }

// ❌ Wrong — base class just to get "is-a" relationship
class SimulatedEntityBase { virtual void update(double dt) = 0; };
class Vessel : public SimulatedEntityBase { … };
```

Defining a new concept:
```cpp
template<typename T>
concept HasParentBody = requires(const T& obj) {
    { obj.parent_body_id() } -> std::convertible_to<uint32_t>;
};
```

---

## 3. Pointer Policy

| Ownership situation              | Use              |
|----------------------------------|------------------|
| Sole owner, no sharing           | `std::unique_ptr` |
| Shared ownership (rare)          | `std::shared_ptr` |
| Non-owning reference             | Raw reference `T&` or `const T&` |
| Optional non-owning reference    | `std::optional<std::reference_wrapper<T>>` |
| C library requires `T*` (output) | Raw pointer **only at call site**, never stored |

```cpp
// ✅ Correct
auto island = std::make_unique<PhysicsIsland>(config);

// ✅ Correct — raw pointer only because Jolt API requires it
JPH::Body* body = body_interface.CreateBody(settings);

// ❌ Wrong — raw owning pointer
PhysicsIsland* island = new PhysicsIsland(config);
```

---

## 4. Casting Rules

Always use explicit C++ casts.  C-style casts (`(int)x`) are banned and
will fail clang-tidy (`-Wold-style-cast`).

| Intent                                     | Cast to use       |
|--------------------------------------------|-------------------|
| Numeric type conversion (known safe)       | `static_cast<T>`  |
| Downcasting through a class hierarchy      | `dynamic_cast<T>` |
| Reinterpreting bits (rare; e.g. float→int) | `std::bit_cast<T>` |
| Removing const (almost never)              | `const_cast<T>`   |
| `std::any` extraction                      | `std::any_cast<T>` |
| `std::variant` extraction                  | `std::get<T>` or `std::get_if<T>` |

```cpp
// ✅ Correct
const float f = static_cast<float>(some_double);
const auto* vessel = dynamic_cast<const Vessel*>(entity_ptr);

// ❌ Wrong
const float f = (float)some_double;
```

---

## 5. Naming Conventions

| Element              | Style             | Example                          |
|----------------------|-------------------|----------------------------------|
| Namespace            | `snake_case`      | `helios::sim::orbital`          |
| Class / Struct       | `PascalCase`      | `KeplerianElements`, `GhostTimeline` |
| Function / method    | `snake_case`      | `evaluate_at()`, `position_at()` |
| Variable             | `snake_case`      | `mean_anomaly`, `parent_body_id` |
| Constant (`constexpr`) | `k_snake_case`  | `k_mu`, `k_sma`                  |
| Enum value           | `PascalCase`      | `ErrorCode::FileNotFound`        |
| Template parameter   | `PascalCase`      | `template<typename Entity>`      |
| Concept              | `PascalCase`      | `concept SimulatedEntity`        |
| Private member       | `snake_case_`     | `impl_`, `segments_`             |
| Macro (avoid!)       | `HELIOS_UPPER`   | `HELIOS_ASSERT` (logging uses LumenLog's `LOG_*`) |
| Build/platform flag  | `UPPER`          | `PLATFORM_LINUX`, `HELIOS_VERSION` |

---

## 6. File Organisation

```
include/helios/<module>/<feature>.hpp   — Public API headers (no implementation)
source/<module>/<feature>.cpp            — Implementation
source/<module>/CMakeLists.txt           — Module build definition
test/unit/<module>/test_<feature>.cpp    — Catch2 v3 unit tests (one file per source file,
                                           always include an error-path test)
```

Each header must be self-contained (include all its own dependencies).  Use
`#pragma once` as the include guard.

---

## 7. GPU Parallelisation Guidelines

GPU compute is appropriate for:
- **Terrain patch generation** — domain-warped fBm, normal derivation, colour
  blending.  Each patch is independent → trivially parallel.
- **Atmospheric scattering LUT precomputation** — offline; run once per planet.
- **Particle system simulation** — re-entry plasma, exhaust.

GPU compute is **not** appropriate for:
- Orbital mechanics (double precision; GPU fp64 is slow on mobile).
- Ghost timeline evaluation (serial, branch-heavy).
- Jolt Physics (already SIMD-parallel on CPU; good enough).

Compute shaders are written in bgfx's shader language (GLSL-like), compiled
offline with `shaderc`, and loaded at runtime.  All compute dispatch goes
through the `helios::render` module; the simulation layer is GPU-free.

---

## 8. Thread Safety

- The simulation layer is **single-threaded by design**.  All ECS mutations
  happen on the simulation thread.
- Exception: *pure* per-vessel propagation jobs (no ECS access, inputs copied in,
  results returned by value) may run on a thread pool; results are applied on the
  simulation thread.
- Terrain patch generation runs on a thread pool (background workers); patches
  are transferred to the render thread as atomic pointer swaps.
- The render thread and simulation thread communicate through a
  **double-buffered state snapshot** — never share mutable state directly.
- Jolt's job system handles its own threading internally; do not call Jolt from
  multiple project threads simultaneously.

---

## 9. `[[nodiscard]]` Policy

Mark every function `[[nodiscard]]` if ignoring its return value is likely to
be a bug.  This includes:
- All `Result<T>` / `VoidResult` returning functions.
- Factory functions (`make_*`, `create_*`).
- Functions returning computed values (not purely side-effecting).

---

## 10. Documentation

Prefer **self-documenting code** over comments.  Write comments to explain
*why*, not *what*.  Algorithm-level notes (citing papers, explaining formulae)
are encouraged.

```cpp
// ✅ Good — explains non-obvious formula origin
// Bruneton & Neyret (2008), equation 7 — inscatter lookup.
const double cos_theta = dot(sun_dir, view_dir);

// ❌ Unnecessary — just noise
// Increment loop counter
++i;
```

Public API functions in headers should have a one-line purpose comment above
the declaration.  Full Doxygen is not required.
