#pragma once

#include "grbl/gcode.h"

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
