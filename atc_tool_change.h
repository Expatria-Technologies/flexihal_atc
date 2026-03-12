#pragma once

#include "grbl/gcode.h"

// ATC macro IDs — resolved by macros.c to /P<n>.macro in the VFS root.
// Place the corresponding files at the root of the filesystem:
//   P390.macro  — atc_change.ngc  — full carousel pick/place sequence
//   P391.macro  — atc_return.ngc  — return spindle tool to carousel pocket
//   P392.macro  — atc_measure.ngc — probe tool length against G59.3 toolsetter
//   P393.macro  — atc_pause.ngc   — optional operator pause hook (chip cover etc.)
// Must not overlap with macros.c reserved IDs (97-99) or user G65 P<n> macros.
#define ATC_MACRO_ID_CHANGE   390
#define ATC_MACRO_ID_RETURN   391
#define ATC_MACRO_ID_MEASURE  392
#define ATC_MACRO_ID_PAUSE    393

// Optional hook called after the machine arrives at the change position
// (G30 or home Z) but before the STATE_TOOL_CHANGE pause.
// Register via tc_set_pause_hook().
// If the hook returns non-OK the tool change is aborted.
typedef status_code_t (*tc_pause_hook_ptr)(void);
void tc_set_pause_hook (tc_pause_hook_ptr hook);

// Probe the tool currently in the spindle against the G59.3 toolsetter,
// store the measured gauge length in the tool table, and activate the offset.
// Delegates to atc_measure.ngc, which cancels any active TLO (G49), runs a
// fast seek + slow locate probe cycle, stores the result via G10 L11, and
// activates it via G43.  Feed rates and probing distance are read from
// grblHAL settings via PRM[] expressions — no parameters are passed from C.
//
// Called by the $TCMEASURE system command after a carousel tool change has
// physically completed.  Does not move to G30 and does not pause for operator
// interaction — the tool must already be clamped.
//
// Requires:
//   - /linuxcnc/atc_measure.ngc present on the VFS
//   - NGC expression support enabled in config.h (required for PRM[], G10 L11)
//   - COMPATIBILITY_LEVEL <= 1
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_probe_tool (parser_state_t *parser_state);

// Force re-measurement of the current tool, regardless of whether a stored
// offset already exists.  Clears the tool's Z entry in the tool table first
// so the skip guard in atc_measure.ngc falls through, then probes normally.
//
// Use after physically replacing a tool in the spindle.
//
// Requires:
//   - /linuxcnc/atc_measure.ngc present on the VFS
//   - NGC expression support enabled in config.h (required for PRM[], G10 L11)
//   - COMPATIBILITY_LEVEL <= 1
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_reprobe_tool (parser_state_t *parser_state);

// Pause for the operator to remove a hand-loaded tool before the carousel
// picks the next one.  Moves to home Z, optionally to G30, runs the pause
// hook, then waits in STATE_TOOL_CHANGE for cycle start.  Does NOT probe.
//
// Call this when the outgoing tool is P0 (not in the carousel) and the
// incoming tool is in the carousel, before running atc_change.ngc.
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_operator_unload_pause (parser_state_t *parser_state);

// Full manual tool change for tools not in the carousel.
//
// Moves to home Z, optionally moves to G30 for operator access, pauses in
// STATE_TOOL_CHANGE waiting for cycle start, then runs atc_measure.ngc to
// probe the newly loaded tool, store its gauge length, and activate the offset.
//
// Mirrors ToolChange_SemiAutomatic from grblHAL's tool_change.c.
//
// Requires:
//   - /linuxcnc/atc_measure.ngc present on the VFS
//   - NGC expression support enabled in config.h (required for PRM[], G10 L11)
//   - COMPATIBILITY_LEVEL <= 1
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_manual_tool_change (parser_state_t *parser_state);
