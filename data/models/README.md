# Models

Models a look can be drawn with instead of a built-in shape (BRIEFING D44): a control, an
indicator or a guard's cover, made as the original is, down to its nut and its bat handle. A
style sheet or a control's own look names one by its path here:

```toml
[switch]
shape = "toggle"              # still says how it moves, and where a hand takes it
model = "stock/toggle.glb"
```

## Licences

**Every model here is listed in `LICENCES.toml`, and one that is not is not read.** Check the
licence before a model goes in: it must let the model be shipped with Helios and changed (CC0,
CC-BY and the like, or the project's own MPL-2.0), with the credit it asks for in `author`.
"Personal use only", "editorial use", or assets taken from a game will not do. A scan of a
museum piece is under the museum's terms, which often allow no more than viewing.

```toml
[[model]]
file = "stock/toggle.glb"
title = "Bat-handle toggle switch"
author = "Helios contributors"
licence = "MPL-2.0"            # an SPDX identifier
source = "tools/models/make_stock_models.py"   # a URL, or what made it
```

The stock models are made by `tools/models/make_stock_models.py`; run it again after changing it.

## Making one

- **Format:** glTF 2.0, a `.glb` (one file) or a `.gltf` with its buffers beside it.
- **Axes and units:** glTF's own: x to the right of the panel, y up it (the place's `up`), z
  out of it towards whoever uses it (the place's `facing`); metres. The origin is the control's
  `position_m`. The model is drawn at its own size: `size_m` and `travel` still say how far it
  moves and how big a hand's target is, so keep them near the model's.
- **What moves:** the node called `moving`, with everything under it, moves as the control's
  shape does, about the node's own origin: a toggle's handle leans about the panel's right, a
  rotary's or a knob's turns about out, a lever's slides up, a breaker's comes out, a stick's
  leans and twists, a dial's or a tape's needle turns or rises, a cover opens about the panel's
  right. Put the node's origin where the original pivots. A model without a `moving` node
  stands still.
- **Colours:** a material's base colour is used as it is, unless its name is one of these:

  | Material | Colour |
  |---|---|
  | `colour` | the look's `colour` |
  | `accent` | the look's `accent`; grey while the vessel lacks a signal the control moves |
  | `paint` | the look's `paint` |
  | `lamp` | the look's `colour` when out, its `accent` lit; between them for a switch's middle positions or a state's middle states |

  So one model can be painted by many style sheets. Textures are not read yet.
- **Labels** are still lettered by Helios above the control, and a readout's number in its
  window.
- Only triangles are drawn; a mesh without normals gets smooth ones.
