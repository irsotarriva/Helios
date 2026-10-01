# Vessel blueprints

A blueprint is a tree of parts and a list of stages. Parts come from the datasheets in
[`../parts`](../parts/README.md). All quantities are SI.

```toml
[[vessel]]
name = "Kestrel"

[[vessel.part]]
name = "probe"                 # unique in the vessel; its signals are "probe/<port>"
part = "core.probe_core"       # the datasheet id

[[vessel.part]]
name = "upper_tank"
part = "core.tank_kerolox_small"
parent = "probe"               # a part defined above; only the first part has none
position_m = [-2.0, 0.0, 0.0]  # of this part's origin, in the parent's axes
rotation_axis = [0.0, 0.0, 1.0]  # optional: turn the part about this axis ...
rotation_deg = 0.0               # ... by this angle (right-handed)

[[vessel.stage]]
commands = [{ signal = "lower_engine/ignition", value = 1 }]

[[vessel.stage]]
commands = [
  { signal = "decoupler/separate", value = 1 },
  { signal = "upper_engine/ignition", value = 1 },
]
```

- The first part is the root: the vessel's axes and origin are its axes and origin, with +x
  towards the nose. Engines push along +x unless they are mounted turned.
- Part names must not contain `/`, and must differ from the interface ids in use (`engine`,
  `separator`) and from `vessel`, `staging` and `guidance`, which name the vessel's own signals.
- A vessel stands on the cylinders of its parts' shapes. One that is to land needs something
  wide at the bottom (the stock `core.landing_legs`).
- Stages are activated in order by raising the signal `staging/stage`. Activating a stage
  publishes its commands; a command for a signal that has left with a dropped stage is skipped.
