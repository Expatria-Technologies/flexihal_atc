# flexihal_atc

Automatic tool change plugin for [grblHal](https://www.grbl.org/what-is-grblhal) 

To use, set ATC_ENABLE=2 and TOOLTABLE_ENABLE=2 in platformio.ini or your project setup.

## Todo

- [X] Add commands to clamp and unclamp the drawbar.
- [ ] Tool setter via built-in probing functions, not new ones.
- [ ] Ensuring correct interaction with persistent TLC
- [ ] Tool Table support
- [ ] Chip cover
- [ ] Allow other orientations / axis of magazine than Z axis to load / unload

- Automatic carousel tool change via NGC macros
- Integrated tool length measurement against a fixed toolsetter at G59.3
- Random-pocket carousel support via a persistent tooltable in LinuxCNC format
- Manual tool swap support with automatic return of outgoing carousel tools
- Drawbar open/close control with spindle interlock
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

| Command | Description |
|---------|-------------|
| `$TCADD [Tn]` | Add tool to the carousel. Assigns the next free pocket. If no tool number is given, uses the tool currently in the spindle. Existing tool offsets are preserved. |
| `$TCRM Tn` | Remove tool from the carousel. Clears the pocket assignment while preserving offsets. |

### Tool Measurement

| Command | Description |
|---------|-------------|
| `$TCMEASURE` | Probe the current tool against the G59.3 toolsetter and set the tool length offset. Called automatically at the end of every tool change macro. |
| `$TCWAIT` | Enter tool change mode (`STATE_TOOL_CHANGE`) and wait for cycle start. Used in `atc_pause.ngc` to pause for a manual tool swap. |

### Tooltable

| Command | Description |
|---------|-------------|
| `$TTLOAD` | Reload the tool table from disk. |

## Files

The following NGC macro files must be present on the filesystem at `/linuxcnc/`:

| File | Purpose |
|------|---------|
| `atc_change.ngc` | Full carousel swap — picks up incoming tool, returns outgoing tool if applicable, then measures |
| `atc_return.ngc` | Returns current tool to its carousel pocket (used when incoming tool is not in carousel) |
| `atc_pause.ngc` | Moves to tool change position and waits for user to manually swap tool, then measures |

If any required macro file is missing, M6 will report a warning and abort rather than leaving the machine in an undefined state.

### Named Parameters

The plugin sets the following named NGC parameters before starting a macro:

| Parameter | Description |
|-----------|-------------|
| `#<_t>` | Incoming tool number |
| `#<_p>` | Incoming tool carousel pocket number |
| `#<_atc_outgoing_pocket>` | Outgoing tool carousel pocket (0 if tool was hand-loaded) |

You will also need to define the following named parameters in your startup gcode to match your machine geometry:

| Parameter | Description |
|-----------|-------------|
| `#<_home_z>` | Safe Z clearance height (machine coordinates) |
| `#<_atc_z_start>` | Z height just above carousel pocket |
| `#<_atc_z_engage>` | Z height to fully engage tool in pocket |
| `#<_atc_feed>` | Feed rate for Z engagement moves |
| `#<_atc_tc_x>` | X position for manual tool change |
| `#<_atc_tc_y>` | Y position for manual tool change |

## Tool Change Behaviour (M6)

M6 behaviour depends on whether the requested tool is in the carousel:

**Tool is in the carousel:**
`atc_change.ngc` is executed. The outgoing tool is returned to its pocket (if it came from the carousel), the incoming tool is picked up, and the new tool is measured.

**Tool is not in the carousel:**
If the outgoing tool came from the carousel, `atc_return.ngc` runs first to return it. Then `atc_pause.ngc` moves to the manual change position, opens the drawbar, and waits for the user to swap the tool. On cycle start the drawbar closes and the new tool is measured.

After any M6 the tooltable is updated automatically: the incoming tool's pocket is cleared (it is now in the spindle) and the outgoing tool's pocket is restored (it has been returned to the carousel).

## Loading a New Tool into the Carousel

To perform a manual tool load and register it in the carousel from gcode:

```gcode
T5 M6       ; swap to tool 5 (triggers atc_pause.ngc for manual swap and measurement)
$TCADD      ; register the current tool (T5) in the next free carousel pocket
```

## Safety

- M6 aborts with an error if any required NGC macro file is missing
- `$TCADD` checks for a tool-present sensor (if configured) before registering
- The tooltable dirty state is reset on any macro error so `onToolChanged` does not corrupt pocket assignments on failure
- A soft reset during a tool change cleans up all macro state

## Settings

| Setting | Description |
|---------|-------------|
| 953 | Drawbar delay (ms) — time between operating the drawbar and reading sensors |
| 954 | User input port |
| 955 | Tool present port |
| 956 | Drawbar status port |
| 957 | Drawbar control port |
| 958 | Air seal port |
| 959 | Taper clear port |
| 960 | TLO clear port |
| 961 | ATC flags (enable/disable individual inputs and outputs) |