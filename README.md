# flexihal_atc

Automatic tool change plugin for [grblHAL](https://www.grbl.org/what-is-grblhal)

To use, set `ATC_ENABLE=2` and `TOOLTABLE_ENABLE=2` in `platformio.ini` or your project setup. NGC expression support (`NGC_EXPRESSIONS_ENABLE`) must also be enabled for macro execution.

If `TOOLTABLE_ENABLE` is not set to `2`, only the spindle interlock and drawbar control features are compiled. The carousel, tooltable, and tool change machinery are excluded and grblHAL's built-in manual/semi-automatic tool change modes are used instead.

## Features

- Automatic carousel tool change via NGC macros
- Integrated tool length measurement against a fixed toolsetter at G59.3
- Random-pocket carousel support via a persistent tooltable in LinuxCNC format
- Manual tool swap support with automatic return of outgoing carousel tools
- Operator pause with tool name notification when a hand-loaded tool is required
- Drawbar open/close control with spindle interlock
- Automatic spindle and coolant stop at the start of every M6, with restore on completion
- Drawbar button polling — hold to open, release to close — works in both `IDLE` and `TOOL_CHANGE` states
- Optional `atc_pause.ngc` hook for operator notification (lights, buzzers, messages)
- Automatic creation of tooltable directory and file on first mount

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
| `$TCADD [Tn] [;name]` | Add tool to the carousel. Assigns the next free pocket. If no tool number is given, uses the tool currently in the spindle. An optional name can be appended after a semicolon. Existing tool offsets are preserved. |
| `$TCREG [Tn] [;name]` | Register a tool in the tooltable at P0 (known but not in the carousel). If the tool is already registered, updates the name if one is given. If the tool is already in the carousel, reports an error — use `$TCADD` instead. |
| `$TCRM [Tn]` | Remove tool from the carousel. Clears the pocket assignment while preserving offsets. If no tool number is given, removes the tool currently in the spindle; requires tool-present sensor if configured.

### Tool Measurement

These commands require `TOOLTABLE_ENABLE=2`.

| Command | Description |
|---------|-------------|
| `$TCMEASURE` | Probe the current tool against the G59.3 toolsetter, store the measured gauge length in the tool table via `G10 L11`, and activate the offset via `G43`. Called automatically at the end of every carousel tool change macro. |

### Tooltable

| Command | Description |
|---------|-------------|
| `$TTLOAD` | Reload the tool table from disk. |

## NGC Macro Files

The following macro files must be present on the filesystem at `/linuxcnc/` when `TOOLTABLE_ENABLE=2`:

| File | Purpose |
|------|---------|
| `atc_change.ngc` | Full carousel swap — returns outgoing tool if applicable, picks up incoming tool, then calls `$TCMEASURE` to measure |
| `atc_return.ngc` | Returns current tool to its carousel pocket (used when incoming tool is not in carousel) |
| `atc_measure.ngc` | Tool length measurement — cancels active TLO, probes against G59.3 toolsetter, stores gauge length via `G10 L11`, activates offset via `G43`. Reads feed rates and probing distance from `$342`–`$345` via `PRM[]` expressions |

The following file is **optional**:

| File | Purpose |
|------|---------|
| `atc_pause.ngc` | Operator notification hook — runs after the machine arrives at the change position, before the `STATE_TOOL_CHANGE` pause. Use for lights, buzzers, or display messages. If absent, it is silently skipped. |

If any required macro file is missing, M6 will report a warning and abort rather than leaving the machine in an undefined state.

> **Note:** Do not add `$TCMEASURE` to `atc_pause.ngc`. Measurement is handled automatically by the plugin after the operator presses cycle start.

### Tool Length Offsets

Tool length offsets are stored as absolute gauge lengths in the tool table via `G10 L11`, using G59.3 as the fixture reference. This means:

- Each tool's length is persistent across power cycles — no re-probing is needed unless the tool is physically replaced
- There is no reference tool requirement; all tools are measured on the same absolute scale
- `G43` (activated automatically after each probe) loads the stored offset from the table for the current tool

The toolsetter position (G59.3 X, Y, Z) must be configured accurately. G59.3 Z should be set to the toolsetter approach height — just above the trigger point. The probe sequence reads `$342`–`$345` for distances and feed rates.

### Parameters Set by Plugin

The plugin sets the following numbered NGC parameters before starting a macro:

| Parameter | Description |
|-----------|-------------|
| `#4900` | Incoming tool number |
| `#4901` | Incoming tool carousel pocket number |
| `#4902` | Outgoing tool carousel pocket (0 if outgoing tool was hand-loaded) |

### Machine Geometry Parameters

The following numbered parameters must be set to match your machine before any tool change runs. The recommended place to set these is a startup macro — see [Running atc_config.macro](#running-atc_configmacro).

| Parameter | Description |
|-----------|-------------|
| `#4910` | X position of carousel pocket 1 |
| `#4911` | Y position of all carousel pockets (fixed for linear carousel) |
| `#4912` | Pocket pitch — X spacing between pockets |
| `#4913` | Z start height — just above pocket (machine coordinates) |
| `#4914` | Z engage height — tool fully seated in pocket (machine coordinates) |
| `#4915` | Z engagement feed rate |

The manual tool change position is configured via **G30** (standard grblHAL mechanism) rather than NGC parameters. Set G30 with `G30.1` after jogging to your preferred change position. The plugin moves to G30 before pausing if `tool_change_at_g30` is enabled in grblHAL settings.

### Running atc_config.macro

grblHAL does not support `O<path> call` syntax for running files by path. Instead, the config file must be placed on the SD card named as `P<n>.macro` where `<n>` is an integer ≥ 100 of your choosing (e.g. `P200.macro`), then called from the MDI or a startup block:

```gcode
G65 P200
```

To run it automatically on every boot, assign it to a startup block:

```
$N0=G65P200
```

This ensures your geometry parameters are always set after a reset or power cycle.

## Tool Change Behaviour (M6)

At the start of every M6 the plugin automatically stops the spindle and coolant. Both are restored to their pre-M6 state when the tool change completes.

M6 behaviour depends on the carousel status of both the outgoing and incoming tools:

### Outgoing from carousel → Incoming from carousel
`atc_change.ngc` runs. The outgoing tool is returned to its pocket, the incoming tool is picked up, and `atc_measure.ngc` is called to probe the new tool, store its gauge length in the tool table, and activate the offset.

### Outgoing hand-loaded (P0) → Incoming from carousel
The machine moves to home Z, optionally moves to G30, and runs `atc_pause.ngc` (if present). The plugin enters `STATE_TOOL_CHANGE` and waits for the operator to **remove** the hand-loaded tool and press cycle start. Once resumed, `atc_change.ngc` runs to pick up the carousel tool and measure it.

### Outgoing from carousel → Incoming hand-loaded (P0)
`atc_return.ngc` runs first to return the outgoing tool to its carousel pocket. The machine then moves to home Z, optionally to G30, and runs `atc_pause.ngc` (if present). The plugin enters `STATE_TOOL_CHANGE` and waits for the operator to **load** the new tool and press cycle start. Once resumed, `atc_measure.ngc` probes the new tool, stores its gauge length, and activates the offset.

### Outgoing hand-loaded (P0) → Incoming hand-loaded (P0)
The machine moves to home Z, optionally to G30, and runs `atc_pause.ngc` (if present). The plugin enters `STATE_TOOL_CHANGE` and waits for the operator to swap the tool and press cycle start. Once resumed, `atc_measure.ngc` probes the new tool, stores its gauge length, and activates the offset.

### Tooltable updates after M6

After M6 completes the tooltable is updated to reflect the new physical state: the incoming tool's pocket is cleared to P0 (it is now in the spindle, not the carousel). If the outgoing tool originally came from the carousel, its pocket assignment is also restored to its original slot. If the outgoing tool was hand-loaded (P0), its tooltable entry is left unchanged.

### Tool Name Notification

If the incoming tool has a name or comment in the tool table, the plugin reports it to the sender before the operator pause — for example:

```
Load T5 (12mm EM) and press cycle start
```

This message is sent regardless of whether `atc_pause.ngc` is present.

## Loading a New Tool into the Carousel

To perform a manual tool load and register it in the carousel:

```gcode
T5 M6       ; swap to tool 5 — triggers manual change flow and measurement
$TCADD      ; register the current tool (T5) in the next free carousel pocket
```

To register a tool in the tooltable without assigning it a carousel pocket:

```gcode
$TCREG T7 ;6mm ballnose   ; add tool 7 to the tooltable as P0
```

## Tool Table

The tool table is stored at `/linuxcnc/tooltable.tbl` in LinuxCNC format. The directory and file are created automatically on first mount if they do not exist.

```
P<pocket> T<tool> [X<offset>] [Y<offset>] [Z<offset>] [D<diameter>] [; name]
```

- `P1` and above — tool is assigned to that carousel pocket
- `P0` — tool is known (offsets preserved) but not currently in the carousel

## Safety

- M6 aborts with an error if any required NGC macro file is missing
- Spindle and coolant are always stopped before any tool change motion and restored after
- The operator is always paused and prompted before the carousel picks a tool if the spindle is not empty
- `$TCADD` checks for a tool-present sensor (if configured) before registering
- The tooltable state is reset on any macro error so pocket assignments are not corrupted on failure
- A soft reset during a tool change cleans up all macro state

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
