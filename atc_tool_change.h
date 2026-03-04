#pragma once

#include "grbl/gcode.h"

// Optional hook called after machine arrives at change position (G30 or home Z)
// but before STATE_TOOL_CHANGE pause.  Register via tc_set_pause_hook().
// If the hook returns non-OK the tool change is aborted.
typedef status_code_t (*tc_pause_hook_ptr)(void);
void tc_set_pause_hook (tc_pause_hook_ptr hook);

// Probe the tool currently in the spindle against the G59.3 toolsetter and
// set the tool length offset.  Called by the $TCMEASURE system command after
// a carousel tool change has physically completed.
//
// Does not pause for operator interaction — the tool must already be clamped.
//
// parser_state is required to determine the active plane when
// TOOL_LENGTH_OFFSET_AXIS is -1 (the default).
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_probe_tool (parser_state_t *parser_state);

// Pause for the operator to remove a hand-loaded tool before the carousel
// picks the next one.  Moves to home Z, optionally to G30, runs the pause
// hook, then waits in STATE_TOOL_CHANGE for cycle start.  Does NOT probe.
// Call this when the outgoing tool is P0 and the incoming tool is in the
// carousel, before running atc_change.ngc.
status_code_t tc_operator_unload_pause (parser_state_t *parser_state);

// Full manual tool change for tools not in the carousel.
//
// Moves to home Z, optionally moves to G30 for operator access, pauses in
// STATE_TOOL_CHANGE waiting for cycle start, then probes the newly loaded
// tool against the G59.3 toolsetter and sets TLO.
//
// Mirrors ToolChange_SemiAutomatic from grblHAL's tool_change.c.
// The NGC macro is responsible for restore (spindle/coolant/position) after
// this function returns.
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_manual_tool_change (parser_state_t *parser_state);
