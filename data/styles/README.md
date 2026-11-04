# Style sheets

How a cockpit's controls and indicators look, kept apart from what they do (BRIEFING D43), as a
web page's style sheet is from its markup. What a control does is in the part's datasheet
(`data/parts/README.md`, *Cockpits*); a style sheet says which shape it has, how big, and in
which colours. The same switch can then be a bat-handle toggle in one vessel and a rocker in
another, and a replica can look exactly like the original without changing what it does.

Each `.toml` file here is one sheet, named after the file: `stock.toml` is `"stock"`. A cockpit
picks one with `style = "stock"`; with none it gets the built-in looks below.

```toml
[style]
extends = "stock"            # optional: start from another sheet, change only what differs

[switch]                     # every switch
shape = "toggle"
size_m = 0.04
travel = 0.44                # what it means depends on the shape (table below)
colour = [0.11, 0.12, 0.14]  # the body: red, green, blue from 0 to 1
accent = [0.82, 0.82, 0.80]  # what moves or lights: the handle, a needle, a lamp lit
paint = [0.86, 0.86, 0.82]   # the label lettered by it

[switch.red]                 # a class: the switches with `class = "red"`
accent = [0.85, 0.22, 0.18]
```

The elements are the control types `momentary`, `switch`, `axis`, `stick`, `encoder`; `guard`
(the cover over a guarded control, which takes the classes of its control); and the indicator
types `gauge`, `state`, `readout`.

## Which rule wins

As in CSS, from least to most:

1. the built-in look of the shape;
2. the rule for the element (`[switch]`), in the sheets a sheet extends and then in the sheet;
3. the rules for its classes (`[switch.red]`), in the order the control lists them
   (`class = "red large"`), each again from the most general sheet to the sheet itself;
4. the control's own `look = { ... }` in the datasheet.

A setting left out keeps what the rule before gave it, and a shape changed by a rule brings that
shape's own measures for whatever no rule gives.

## Shapes

| Element | Shape | `size_m` | `travel` |
|---|---|---|---|
| momentary | `pushbutton` | the side of its square cap; lit (`accent`) while everything it sets holds | how far it goes in when pressed, m |
| momentary | `round_button` | the diameter of its cap | as `pushbutton` |
| switch | `toggle` | the plate; a bat handle on it | the angle the handle leans each way at its ends, rad; positions are spread between |
| switch | `rocker` | its width | the angle it tilts each way, rad; lit (`accent`) in its upper positions |
| switch | `rotary` | the knob's diameter | the angle its positions are spread over, clockwise, rad |
| switch | `breaker` | the button's diameter | how far it stands out when pulled (its first position), m; in, it is flush |
| axis | `lever` | the handle's width | how far it slides along the control's `up`, m |
| axis | `knob` | the diameter | the angle from one end to the other, clockwise, rad |
| axis | `wheel` | the diameter; seen edge-on, it turns about the control's right | the angle from one end to the other, rad |
| stick | `stick` | the grip's width | its length, m; it leans up to 20° each way |
| encoder | `knob` | the diameter | (unused; a mark turns 30° each step) |
| guard | `cover` | 0: as big as its control | the angle it opens to, rad |
| gauge | `dial` | the diameter | the needle's sweep, rad |
| gauge | `tape` | its height; a mark moves up it | (unused) |
| state | `lamp` | the side of the lens; `colour` out, `accent` lit (with more states, between them) | (unused) |
| state | `talkback` | the window; grey (`colour`) in its first state, striped (`accent`) after | (unused) |
| readout | `digits` | the width of its window; the number lit in `accent` | (unused) |

The first shape of each element is its own when no rule names one.

## Not yet

A look will be able to name a model (`model = "…glb"`, read with cgltf, BRIEFING §13): a part
of a cockpit made from the original, down to the scratches, with named nodes that move as the
control does. Until then `model` is refused, and every look is one of the built-in shapes.
Check the licence of any model before it goes into `data/`.
