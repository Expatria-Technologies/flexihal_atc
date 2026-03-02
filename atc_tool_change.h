#pragma once

#include "grbl/gcode.h"

// Probe the tool currently in the spindle against the G59.3 toolsetter
// and set the tool length offset.  Called by the $TCMEASURE system command
// after a carousel tool change has physically completed.
//
// parser_state is required to determine the active plane when
// TOOL_LENGTH_OFFSET_AXIS is -1 (the default, meaning all axes).
//
// Returns Status_OK on success, or an error code otherwise.
status_code_t tc_probe_tool (parser_state_t *parser_state);
