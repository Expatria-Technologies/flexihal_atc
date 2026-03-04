# flexihal_atc

Automatic tool change plugin for [grblHAL](https://www.grbl.org/what-is-grblhal)

To use, set `ATC_ENABLE=2` and `TOOLTABLE_ENABLE=2` in `platformio.ini` or your project setup. NGC expression support (`NGC_EXPRESSIONS_ENABLE`) must also be enabled for macro execution.

## Features

- Automatic carousel tool change via NGC macros
- Integrated tool length measurement against a fixed toolsetter at G59.3
- Random-pocket carousel support via a persistent tooltable in LinuxCNC format
- Manual tool swap support with automatic return of outgoing carousel tools
- Drawbar open/close control with spindle interlock
- Automatic spindle and coolant stop at the start of every M6, with restore on completion
- Drawbar button polling — hold to open, release to close — works in both `IDLE` and `TOOL_CHANGE` states
- Optional `atc_pause.ngc` hook for operator notification (lights, buzzers, messages)
- Automatic creation of tooltable directory and file on first mount

## Todo

- [ ] Ensuring correct interaction with persistent TLO reference

## Commands

All commands can be used from the terminal or embedded in gcode programs.

### Drawbar Control

| Command | Description |
|---------|-------------|
| `$DRBO` | Open the drawbar (release tool) |
| `$DRBC` | Close the drawbar (clamp tool) |

The drawbar cannot be opened while the spindle is running. The spindle cannot be started while the drawbar is open. Both commands are only permitted in `IDLE` or `TOOL_CHANGE` states.

### Carousel Management

| Command | Description |
|---------|-------------|
| `$TCADD [Tn] [;name]` | Add tool to the carousel. Assigns the next free pocket. If no tool number is given, uses the tool currently in the spindle. An optional name can be appended after a semicolon. Existing tool offsets are preserved. |
| `$TCRM Tn` | Remove tool from the carousel. Clears the pocket assignment while preserving offsets. |

### Tool Measurement

| Command | Description |
|---------|-------------|
| `$TCMEASURE` | Probe the current tool against the G59.3 toolsetter and set the tool length offset. Called automatically at the end of every carousel tool change macro. |

### Tooltable

| Command | Description |
|---------|-------------|
| `$TTLOAD` | Reload the tool table from disk. |

## NGC Macro Files

The following macro files must be present on the filesystem at `/linuxcnc/`:

| File | Purpose |
|------|---------|
| `atc_change.ngc` | Full carousel swap — returns outgoing tool if applicable, picks up incoming tool, then measures |
| `atc_return.ngc` | Returns current tool to its carousel pocket (used when incoming tool is not in carousel) |
| `atc_config.ngc` | Sets machine geometry parameters — run once after homing |

The following file is **optional**:

| File | Purpose |
|------|---------|
| `atc_pause.ngc` | Operator notification hook — runs after the machine arrives at the manual change position, before the `STATE_TOOL_CHANGE` pause. Use for lights, buzzers, or display messages. If absent, it is silently skipped. |

If any **required** macro file is missing, M6 will report a warning and abort rather than leaving the machine in an undefined state.

### Parameters Set by Plugin

The plugin sets the following numbered NGC parameters before starting a macro:

| Parameter | Description |
|-----------|-------------|
| `#4900` | Incoming tool number |
| `#4901` | Incoming tool carousel pocket number |
| `#4902` | Outgoing tool carousel pocket (0 if outgoing tool was hand-loaded) |

### Machine Geometry Parameters

The following numbered parameters must be set to match your machine before any tool change runs. The provided `atc_config.ngc` is the recommended place to set these — run it once after homing:

| Parameter | Description |
|-----------|-------------|
| `#4910` | X position of carousel pocket 1 |
| `#4911` | Y position of all carousel pockets (fixed for linear carousel) |
| `#4912` | Pocket pitch — X spacing between pockets |
| `#4913` | Z start height — just above pocket (machine coordinates) |
| `#4914` | Z engage height — tool fully seated in pocket (machine coordinates) |
| `#4915` | Z engagement feed rate |

The manual tool change position is configured via **G30** (standard grblHAL mechanism) rather than NGC parameters. Set G30 with `G30.1` after jogging to your preferred change position. The plugin moves to G30 before pausing if `tool_change_at_g30` is enabled in grblHAL settings.

## Tool Change Behaviour (M6)

At the start of every M6 the plugin automatically stops the spindle and coolant. Both are restored to their pre-M6 state when the tool change completes.

M6 behaviour then depends on whether the requested tool is in the carousel:

**Tool is in the carousel:**
`atc_change.ngc` runs. The outgoing tool is returned to its pocket (if it came from the carousel), the incoming tool is picked up, and the new tool is measured.

**Tool is not in the carousel:**
If the outgoing tool came from the carousel, `atc_return.ngc` runs first to return it. The machine then moves to home Z, optionally moves to G30 for operator access, and runs `atc_pause.ngc` (if present). The plugin then enters `STATE_TOOL_CHANGE` and waits for the operator to load the tool and press cycle start. Once resumed, the new tool is probed and TLO is set automatically.

After any M6 the tooltable is updated automatically: the incoming tool's pocket is cleared (it is now in the spindle) and the outgoing tool's pocket is restored (it has been returned to the carousel).

### Tool Name Notification

If the incoming tool has a name or comment in the tool table, the plugin reports it to the sender before the operator pause — for example:

```
Load T5 (12mm EM) and press cycle start
```

This message is sent regardless of whether `atc_pause.ngc` is present.

## Loading a New Tool into the Carousel

To perform a manual tool load and register it in the carousel from gcode:

```gcode
T5 M6       ; swap to tool 5 — triggers manual change flow and measurement
$TCADD      ; register the current tool (T5) in the next free carousel pocket
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
| `$962` | Number of carousel pockets — `$TCADD` will refuse to assign a pocket beyond this limit |

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
