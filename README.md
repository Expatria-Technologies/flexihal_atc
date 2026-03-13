# flexihal_atc

Automatic tool change plugin for [grblHAL](https://www.grbl.org/what-is-grblhal)

To use, set `ATC_ENABLE=2` and `TOOLTABLE_ENABLE=2` in `platformio.ini` or your project setup. NGC expression support (`NGC_EXPRESSIONS_ENABLE`) must also be enabled for macro execution.

If `TOOLTABLE_ENABLE` is not set to `2`, only the spindle interlock and drawbar control features are compiled. The carousel, tooltable, and tool change machinery are excluded and grblHAL's built-in manual/semi-automatic tool change modes are used instead.

## Features

- Automatic carousel tool change via NGC macros (`tc.macro`)
- Integrated tool length measurement against a fixed toolsetter at G59.3
- Random-pocket carousel support via a persistent tooltable in LinuxCNC format
- Manual tool swap support with automatic return of outgoing carousel tools
- Operator pause (M0) when a hand-loaded tool is required
- Drawbar open/close control with spindle interlock
- Automatic spindle and coolant stop at the start of every M6, with restore on completion
- Drawbar button polling — hold to open, release to close — works in both `IDLE` and `TOOL_CHANGE` states
- Pocket assignment tracking — tooltable always reflects physical carousel state

## Architecture

The tool change sequence is driven by grblHAL's built-in `tc.macro` mechanism. When M6 is parsed, `tooltable.c` sets NGC parameters describing the outgoing and incoming tools, then grblHAL launches `tc.macro` which orchestrates the physical motion via subroutine macros. The C plugin handles only hardware I/O (drawbar, sensors, spindle interlock) and carousel administration commands.

### Tool Change Parameter Handoff

`tooltable.c` hooks `hal.tool.select` and sets the following NGC parameters on every `Txx` command, before `tc.macro` runs:

| Parameter | Description |
|-----------|-------------|
| `#4900` | Outgoing tool ID (0 if spindle was empty) |
| `#4901` | Outgoing tool carousel pocket (-1 if hand-loaded or unknown after power cycle) |
| `#4902` | Incoming tool ID (0 = T0) |
| `#4903` | Incoming tool carousel pocket (-1 if hand-loaded) |

`tc.macro` reads these parameters and calls the appropriate subroutine macros. At the end of `tc.macro`, `M61 Q#4902` updates the current tool in grblHAL, which triggers `onToolChanged` in `tooltable.c` to update pocket assignments in the tooltable file.

### Pocket Tracking

The tooltable stores the physical state of the carousel — `P0` means the tool is not in the carousel (hand-loaded or in the spindle), `Pn` means the tool is physically in pocket `n`. The tooltable is updated immediately when tools move:

- When a carousel tool is fetched (`P390.macro` completes) → pocket cleared to P0
- When a carousel tool is returned (`P391.macro` completes) → pocket restored via `tooltable_carousel_add()`

`last_fetched_pocket` is a volatile (RAM-only) variable that remembers which pocket the current spindle tool came from, so it can be returned there on the next tool change. This value is lost on power cycle — the operator can use `$TCRETURN` or `$TCADD` to re-establish the carousel state.

## Commands

All commands can be used from the terminal or embedded in gcode programs.

### Drawbar Control

| Command | Description |
|---------|-------------|
| `$DRBO` | Open the drawbar (release tool) |
| `$DRBC` | Close the drawbar (clamp tool) |

The drawbar cannot be opened while the spindle is running. The spindle cannot be started while the drawbar is open. Both commands are only permitted in `IDLE` or `TOOL_CHANGE` states.

### Carousel Management

These commands require `TOOLTABLE_ENABLE=2`.

| Command | Description |
|---------|-------------|
| `$TCADD [Tn] [;name]` | Deposit the current spindle tool (or specified tool) into the next free carousel pocket and register it in the tooltable. Runs `P391.macro` to physically move the tool. If the deposit motion fails to start, the pocket assignment is rolled back. Machine must be homed and IDLE. |
| `$TCRETURN` | Return the current spindle tool to its carousel pocket using `last_fetched_pocket`. Requires the tool to have been fetched from the carousel since the last power cycle. Machine must be homed and IDLE. Spindle must be off. |
| `$TCRM [Tn]` | Clear a carousel pocket assignment, moving the tool to P0. Administrative only — does not move the machine. The operator is responsible for physically removing the tool first. |

### Tool Measurement

These commands require `TOOLTABLE_ENABLE=2`.

| Command | Description |
|---------|-------------|
| `$TCMEASURE` | Probe the current tool against the G59.3 toolsetter, store the measured gauge length via `G10 L11`, and activate the offset via `G43`. Skips measurement if a valid offset is already stored. Runs `P394.macro`. Machine must be homed and IDLE. |
| `$TCREMEASURE` | Clear the stored offset for the current tool and re-probe unconditionally. Use after physically replacing a tool in the spindle. |

### Tooltable

These commands require `TOOLTABLE_ENABLE=2`.

| Command | Description |
|---------|-------------|
| `$TTLOAD` | Reload the tool table from disk. |
| `$TTLIST` | Print all entries in the tool table to the console. |
| `$TTINDEX` | Print the in-RAM pocket index to the console. Shows each tool's assigned pocket as currently held in memory. Useful for debugging carousel registration. |
| `$TTREG Tn [;name]` | Register a tool in the tooltable at P0, or update its name if already registered. |
| `$TTDEL Tn` | Delete a P0 tool entry from the tooltable entirely. Use `$TCRM` first if the tool is in a pocket. |

## NGC Macro Files

The following macro files must be present on the filesystem when `TOOLTABLE_ENABLE=2`. All macros use the grblHAL `P<n>.macro` naming convention and are located in the VFS root or littlefs.

| File | Purpose |
|------|---------|
| `tc.macro` | Top-level tool change orchestrator. Reads `#4900`–`#4903` and calls appropriate subroutines. Must end with `M61 Q#4902` to update the current tool. |
| `ts.macro` | Tool select notification. Runs on `Txx`. Prints the incoming tool number and whether it is in the carousel or hand-loaded. |
| `P390.macro` | Fetch a tool from the carousel. Parameters: `#4900`=tool_id, `#4901`=pocket. |
| `P391.macro` | Return a tool to the carousel. Parameters: `#4900`=tool_id, `#4901`=pocket. Also used by `$TCADD` and `$TCRETURN`. |
| `P392.macro` | Hand fetch — M0 pause prompting operator to install a hand-loaded tool. |
| `P393.macro` | Hand return — M0 pause prompting operator to remove a hand-loaded tool. |
| `P394.macro` | Measure tool length. Skips if tool already has a non-zero Z offset stored. Handles T0 gracefully. |

### tc.macro Flow

```
tc.macro
  ├─ outgoing tool exists (#4900 NE 0)
  │    ├─ outgoing in carousel (#4901 GE 1) → G65 P391 (return)
  │    └─ outgoing hand-loaded             → G65 P393 (M0 pause, operator removes)
  │
  ├─ incoming tool (#4902)
  │    ├─ T0                               → skip fetch, spindle empty
  │    ├─ incoming in carousel (#4903 GE 1) → G65 P390 (fetch)
  │    └─ incoming hand-loaded             → G65 P392 (M0 pause, operator installs)
  │                                           G65 P394 (measure)
  │
  └─ M61 Q#4902  → updates current tool, triggers tooltable pocket update
```

### ts.macro

Runs on every `Txx` command. Prints the selected tool and its carousel status so the operator knows what to expect before M6.

### Tool Length Measurement (P394.macro)

Tool length offsets are stored as absolute gauge lengths via `G10 L11` using G59.3 as the fixture reference:

- Offsets are persistent across power cycles — no re-probing unless the tool is physically replaced
- No reference tool required — all tools are on the same absolute scale
- `G43` is activated automatically after each probe

The toolsetter position must be configured in G59.3 (X, Y, Z). The probe sequence reads feed rates and probing distance from settings `$342`–`$345` via `PRM[]`.

Measurement is skipped if the tool already has a non-zero Z offset. Use `$TCREMEASURE` to force re-measurement.

### Machine Geometry Parameters

Tool change motion follows the **four-position model**. Each deposit or pickup moves through four explicit waypoints. Parameters are set once via a startup macro (e.g. `$N0=G65P200`).

| Parameter | Description |
|-----------|-------------|
| `#4920` | Position 1 (Safe) X |
| `#4921` | Position 1 (Safe) Y |
| `#4922` | Position 1 (Safe) Z — typically Z0 |
| `#4923` | Position 2 (Approach) X |
| `#4924` | Position 2 (Approach) Y |
| `#4925` | Position 2 (Approach) Z |
| `#4926` | Position 3 (Engage) X |
| `#4927` | Position 3 (Engage) Y |
| `#4928` | Position 3 (Engage) Z — tool fully seated in pocket |
| `#4929` | Position 4 (Exit) X |
| `#4930` | Position 4 (Exit) Y |
| `#4931` | Position 4 (Exit) Z — spindle clear of pocket |
| `#4932` | Feed rate for all carousel positioning moves (mm/min) |

Pocket positioning (rotating the carousel to the correct pocket) is handled externally — either by a dedicated axis or a separate MCU. The pocket number is available as `#4901` (outgoing) and `#4903` (incoming) in `tc.macro`, and as `#4901` in `P390.macro` and `P391.macro`.

## Tool Change Behaviour (M6)

At the start of every M6 the plugin automatically stops the spindle and coolant. Both are restored to their pre-M6 state when `M61` fires at the end of `tc.macro`.

M6 behaviour depends on the carousel status of both the outgoing and incoming tools:

### Outgoing in carousel → Incoming in carousel
`P391.macro` returns the outgoing tool, `P390.macro` fetches the incoming tool. No operator involvement required.

### Outgoing in carousel → Incoming hand-loaded
`P391.macro` returns the outgoing tool. `P392.macro` pauses (M0) for the operator to install the hand-loaded tool. `P394.macro` measures the new tool.

### Outgoing hand-loaded → Incoming in carousel
`P393.macro` pauses (M0) for the operator to remove the hand-loaded tool. `P390.macro` fetches the incoming carousel tool.

### Outgoing hand-loaded → Incoming hand-loaded
`P393.macro` pauses (M0) for the operator to remove the outgoing tool. `P392.macro` pauses (M0) for the operator to install the incoming tool. `P394.macro` measures the new tool.

### Tooltable Updates After M6

When `M61 Q#4902` fires at the end of `tc.macro`, `onToolChanged` in `tooltable.c` updates the tooltable:

- Incoming tool's pocket is cleared to P0 (it is now in the spindle)
- If the outgoing tool came from the carousel, its pocket assignment is restored via `tooltable_carousel_add()`
- `last_fetched_pocket` is updated for the next tool change

## Loading a New Tool into the Carousel

To load a new tool into the carousel for the first time:

1. Get the tool into the spindle and measured:
```gcode
T5 M6       ; operator installs T5 manually, machine measures it
```

2. Deposit into the carousel:
```gcode
$TCADD      ; deposits current spindle tool into next free pocket
$TCADD ;12mm EM   ; as above, with a name
```

After `$TCADD` the spindle is empty. `$TCADD` performs the physical deposit via `P391.macro` and writes the pocket assignment to the tooltable. If the deposit motion fails to start, the pocket assignment is rolled back automatically.

To register a hand-loaded tool without assigning a pocket (offsets preserved, not in carousel):
```gcode
$TTREG T7 ;6mm ballnose
```

## Tool Table

The tool table is stored at `/tooltable.tbl` in LinuxCNC format. The file is created automatically on first mount if it does not exist.

```
P<pocket> T<tool> [X<offset>] [Y<offset>] [Z<offset>] [D<diameter>] [; name]
```

- `P1` and above — tool is physically in that carousel pocket
- `P0` — tool is known (offsets preserved) but not currently in the carousel

## Safety

- Spindle and coolant are always stopped before any tool change motion
- The spindle cannot be started while the drawbar is open or no tool is detected
- `$TCADD` checks for a tool-present sensor (if configured) before registering
- `$TCADD` rolls back the pocket assignment if the deposit macro fails to start
- A soft reset during a tool change cleans up all macro state via grblHAL's reset chain

## Settings

| Setting | Description |
|---------|-------------|
| `$953` | Drawbar delay (ms) — time between operating the drawbar and reading sensors |
| `$954` | User input port |
| `$955` | Tool present port |
| `$956` | Drawbar status port |
| `$957` | Drawbar control port |
| `$958` | Air seal port |
| `$959` | Taper clear port |
| `$960` | TLO clear port |
| `$961` | ATC flags — enable/disable individual inputs and outputs |
| `$962` | Number of carousel pockets — `$TCADD` will refuse to assign a pocket beyond this limit (`TOOLTABLE_ENABLE=2` only) |

### ATC Flags (`$961`)

| Bit | Feature |
|-----|---------|
| 0 | User input button enabled |
| 1 | Tool present sensor enabled |
| 2 | Drawbar status sensor enabled |
| 3 | Drawbar control output enabled |
| 4 | Air seal output enabled |
| 5 | Taper clear output enabled |
| 6 | Toolsetter clear output enabled |
