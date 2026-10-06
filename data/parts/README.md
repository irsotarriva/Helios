# Part datasheets

Every `.toml` file in this directory is loaded, in name order. A file holds resources,
interfaces and parts; a part can use the resources and interfaces of files loaded before it.
All quantities are SI. Design background: [BRIEFING](../../docs/BRIEFING.md) §8.3, §10 and D22.

A datasheet says what a part *is to the flight simulation*: its mass, the controls it takes,
what it reports, what it holds, and what it does to resources and to the vessel while it runs.
It says nothing about how the part works inside.

The files are TOML, restricted to tables, arrays of tables, inline tables, strings, numbers,
booleans and arrays. A misspelt key is an error, not a silently ignored setting.

## Resources

```toml
[[resource]]
id = "lox"               # how parts refer to it
name = "Liquid oxygen"   # optional, for display
unit = "kg"              # of one unit of amount, for display
mass_per_unit_kg = 1.0   # 0 for massless resources such as electric charge (unit "J")
```

## Interfaces

A named set of ports. A part that lists the interface must have all of them. The vessel gets
one signal per input, `<interface>/<input>`, that every member follows unless it is commanded
directly.

```toml
[[interface]]
id = "engine"
inputs = ["ignition", "throttle"]
outputs = ["thrust_n", "mass_flow_kg_s"]
```

## Parts

```toml
[[part]]
id = "core.engine_kerolox_vacuum"
name = "Kerolox vacuum engine (30 kN)"   # optional
dry_mass_kg = 180.0
centre_of_mass_m = [0.0, 0.0, 0.0]       # optional; part axes, +x towards the nose
shape = { kind = "cylinder", radius_m = 0.5, length_m = 1.2 }  # a solid cylinder along x, centred on
                                         # the part's origin: its inertia, and what touches the ground
# or: inertia_kg_m2 = [ixx, iyy, izz]    # principal moments about the centre of mass, dry; such a
                                         # part has no shape and touches nothing
impact_tolerance_m_s = 6.0               # optional; touching the ground faster than the least
                                         # tolerance of its parts destroys the vessel
interfaces = ["engine"]                  # optional
crossfeed = true                         # optional; false: no resources flow to or from its parent
inputs = [
  { name = "ignition", toggle = true },                  # a switch: 0 or 1
  { name = "throttle", minimum = 0.0, maximum = 1.0, default = 0.0, unit = "" },
]
stores = [
  { resource = "lox", capacity = 2800.0, initial = 2800.0 },   # initial defaults to the capacity
]
separator = { input = "separate", impulse_n_s = 2500.0 }       # optional, see below
```

A part with a `[part.cockpit]` table is crewed (see *Cockpits* below).

The contents of a part's stores count as distributed like the part: its inertia scales with
its total mass.

### Processes

What the part does while it runs. Its **level** (0 = off, 1 = full) follows one input; the
resource rates scale with the level.

```toml
[[part.process]]
name = "burn"
enable = "ignition"      # optional: runs only while this input is on
level = "throttle"       # optional: without it the level is 1 whenever enabled
minimum_level = 0.4      # optional: a level above 0 is raised to this
consumes = [
  { resource = "rp1", rate_per_s = 2.7 },   # units of the resource per second at level 1
  { resource = "lox", rate_per_s = 6.3 },
]
produces = []            # same form; what does not fit in the stores is vented
thrust_n = 30000.0       # optional, at level 1 in vacuum
thrust_curve = [[0.0, 0.0], [1.0, 1.0]]   # optional: fraction of thrust_n against level
thrust_direction = [1.0, 0.0, 0.0]        # optional: of the force on the vessel, part axes
thrust_position_m = [-0.6, 0.0, 0.0]      # optional: where it acts
torque_n_m = [0.0, 1500.0, 0.0]           # optional: a pure torque on the vessel at level 1, part axes
```

An input whose range goes below zero gives a level of either sign: a reaction wheel is three
such processes, one per axis. Resources are exchanged at the magnitude of the level; thrust
and torque follow its sign. The stock `attitude` interface (inputs `roll`, `pitch`, `yaw`, each
from -1 to 1, positive torque about the vessel's +x, +y, +z) is what the attitude hold commands.

On rails only the part of the thrust along the vessel's nose acts. In the physics bubble every
force acts where its part puts it, so a thrust off the centre of mass turns the vessel.

A process draws from, and fills, every store of the resource it can reach through the part
tree (crossfeed), in proportion to what each holds or has room for. It stops when something
it consumes has run out and is not being produced at least as fast.

The exhaust velocity is not given: it is `thrust_n` divided by the mass consumed per second.

### Outputs

What the part reports, each a signal `<part>/<name>`.

```toml
[[part.output]]
name = "thrust_n"
unit = "N"               # optional, for display
process = "burn"
quantity = "thrust"      # level | thrust | mass_flow

[[part.output]]
name = "chamber_temperature_k"
unit = "K"
process = "burn"
curve = [[0.0, 290.0], [0.4, 3350.0], [1.0, 3600.0]]   # of the process level; linear between points

[[part.output]]
name = "lox_kg"
store = "lox"
quantity = "amount"      # amount | fraction (of the capacity)
```

### Separators

`separator = { input = ..., impulse_n_s = ... }` makes the part let go of its parent when the
input is switched on. It leaves with everything attached below it, as a vessel of its own, and
the impulse pushes the two apart.

### Cockpits

A crewed part says where its pilot sits and what is around the seat. Everything but the seat
itself is given in the **seat's axes**, measured from the seated pilot's eyes: +x is where the
pilot faces, +y is to the pilot's left, +z is up. The same panel can then be put into a part
whose seat faces the nose and into one whose seat faces sideways.

```toml
[part.cockpit]
eye_m = [0.1, -0.15, 0.0]      # the pilot's eyes, part axes
forward = [1.0, 0.0, 0.0]      # where the pilot faces, part axes
up = [0.0, -1.0, 0.0]          # part axes; made exactly perpendicular to `forward`
walkable = false               # optional; true: the pilot can leave the seat (see below)
boxes = [                      # the solids of the interior, edges along the seat's axes
  { centre_m = [0.65, 0.0, -0.38], size_m = [0.04, 1.2, 0.5], colour = [0.30, 0.31, 0.34] },
  { centre_m = [0.85, 0.0, 0.2], size_m = [0.1, 2.8, 1.8], glass = true },   # a window
]

[[part.cockpit.instrument]]
kind = "dial"                  # dial | lever | switch | button
label = "ALT"
signal = "nav/altitude_m"      # a signal of the vessel's control bus
position_m = [0.625, 0.45, -0.24]
facing = [-1.0, 0.0, 0.0]      # optional: out of the panel, towards the pilot (the default)
up = [0.0, 0.0, 1.0]           # optional: a dial's top, a lever's direction of increase
size_m = 0.11                  # a dial's diameter, a lever's travel, a switch's or button's width
minimum = 0.0                  # dial: the range of the needle
maximum = 500000.0
scale = 0.001                  # dial: the number shown is the value times this ...
unit = "km"                    # ... followed by this
```

- A **dial** shows any signal. A **lever** sets a command signal anywhere in its range, a
  **switch** to its minimum or maximum.
- A **button** publishes a list of commands when pressed, and may step a signal:

  ```toml
  kind = "button"
  label = "STAGE"
  signal = "staging/stage"
  step = 1.0                                           # each press adds this
  # and / or
  commands = [{ signal = "guidance/frame", value = 1 }, { signal = "guidance/x", value = 1 }]
  ```

  A button with commands is lit while every one of them holds.
- Instruments name signals of the *vessel*, not of a part: `engine/throttle`, not
  `upper_engine/throttle`. In a vessel without the signal the instrument is dead; a button one
  of whose signals is missing does nothing.
- Besides the parts' own signals a vessel has `nav/altitude_m`, `nav/vertical_speed_m_s`,
  `nav/surface_speed_m_s` and `nav/speed_m_s` (about the body whose domain it is in), and the
  `vessel/…`, `staging/…` and `guidance/…` signals of BRIEFING §8.3.
- On rails a vessel keeps its +z along the orbit's normal, so its −y is away from the body
  below. A seat facing the nose that should have the ground under its floor has `up = [0, -1, 0]`.
- A cockpit with `walkable = true` is a cabin: the pilot can leave the seat and move about in
  it (BRIEFING D27). Its boxes are then also what the pilot's body touches, so they have to
  close the cabin in: floor, ceiling, walls. A box with `glass = true` is a window: it stops
  a body like any other and is not drawn. A gap between boxes wider than about half a metre is
  a way out, and a pilot who gets out is put back in the seat.
- Design a cabin for the weight it will feel. The pilot stands on whatever is "down": towards
  the tail under thrust and when the vessel stands on its tail on the ground. The stock
  habitat therefore has its floor towards -x.
- A cabin may have loose items, for the pilot to carry and throw:

  ```toml
  [[part.cockpit.item]]
  name = "crate"                     # what the prompt calls it; unique in the cockpit
  position_m = [-1.4, 0.8, -1.3]     # of its centre, seat axes; where it starts, at rest
  size_m = [0.4, 0.4, 0.4]
  mass_kg = 10.0
  colour = [0.55, 0.42, 0.25]        # optional
  ```

  Put an item where it can rest when there is weight (on the floor), or it will fall there.
