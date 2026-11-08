# Key bindings

`bindings.toml` says which keys do what (BRIEFING D45). `helios --bindings <file>` reads another
file instead; copy this one and change what you like.

Keys are named by **where they are**, not by what they print: the name is the key's on a US
keyboard, so `"Z"` is the key left of `X` on an AZERTY keyboard too. Case does not matter.

| Keys | Names |
|---|---|
| Letters, digits | `"A"` … `"Z"`, `"0"` … `"9"` |
| Punctuation | `"-"` `"="` `"["` `"]"` `"\\"` `";"` `"'"` `` "`" `` `","` `"."` `"/"` |
| Others | `"Return"` `"Escape"` `"Backspace"` `"Tab"` `"Space"` `"CapsLock"` `"F1"` … `"F12"` |
| | `"Insert"` `"Home"` `"PageUp"` `"Delete"` `"End"` `"PageDown"` |
| Arrows | `"Up"` `"Down"` `"Left"` `"Right"` |
| Keypad | `"Keypad 0"` … `"Keypad 9"`, `"Keypad ."` `"Keypad /"` `"Keypad *"` `"Keypad -"` `"Keypad +"` `"Keypad Enter"` |
| Modifiers | `"Left Shift"` `"Right Shift"` `"Left Ctrl"` `"Right Ctrl"` `"Left Alt"` `"Right Alt"` `"Left GUI"` `"Right GUI"` |

## `[actions]`

Each action takes a key or an array of keys (`pause = ["P", "Space"]`). An action left out has
no key. A key may do more than one action; on foot, a key the pilot uses (turning, moving,
jumping) does only that.

| Action | Does |
|---|---|
| `throttle_up`, `throttle_down` | moves the throttle while held |
| `throttle_full`, `throttle_cut` | the throttle all the way |
| `pitch_up`, `pitch_down`, `yaw_left`, `yaw_right`, `roll_left`, `roll_right` | turn the vessel as seen from the seat; out of the seat, turn the view |
| `forward`, `back`, `left`, `right`, `up`, `down` | move: a vessel's translation, the pilot's walking, climbing and arm |
| `stage` | the next stage |
| `attitude_hold` | the attitude hold on or off |
| `interact` | what the picture offers (`[F] sit down`); held, lets go of a ladder or a hull |
| `jump` | on foot: jump, push off a hull |
| `pause`, `warp_faster`, `warp_slower` | time |
| `map_view`, `flight_view` | the map or the flight view; the seat or outside |
| `next_focus` | the next body or vessel |
| `help`, `panels`, `quit` | the help panel, the other panels, quit |

## `[[control]]`

A key on a control of the cockpit the pilot sits in, named by its `id` in the part's datasheet
(`data/parts/README.md`). It works any cockpit that has a control with that id, and does what a
hand on it does: while the key is down the control is held, and it is let go with the key.

| Key | Means |
|---|---|
| `key` | the key; not one an action has |
| `control` | the control's id |
| `part` | only in this part's cockpit (its datasheet id, `"core.command_pod"`); default any |
| `way` | `1` up (on, clockwise), `-1` the other way, `0` (default) as a click on its middle |
| `rate` | an axis: how fast it moves while the key is held, in fractions of its travel a second (default 0.5) |

What each type does with the key:

- **momentary**: pressed while the key is down;
- **switch**: one position `way` (with `0`, a two-way switch flips); a sprung one goes back with
  the key; a locked one is pulled out and over;
- **encoder**: one click `way` (`0` is up);
- **axis**: moves `way` (`0` is up) while the key is down, and settles in a detent or where its
  spring takes it after;
- **stick**: not bound to keys: the turning actions are its.

A control under a closed **guard** cannot be reached: the key does nothing until the cover is
lifted by hand.
