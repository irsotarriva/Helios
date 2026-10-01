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
shape = { kind = "cylinder", radius_m = 0.5, length_m = 1.2 }  # inertia of a solid cylinder along x
# or: inertia_kg_m2 = [ixx, iyy, izz]    # principal moments about the centre of mass, dry
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
```

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
