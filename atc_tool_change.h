#pragma once

#include "grbl/gcode.h"

// Probe the tool currently in the spindle against the G59.3 toolsetter
// and set the tool length offset.  Called by the $TCMEASURE system command
// after a carousel tool change has physically completed.
//
// Returns Status_OK on success, or an error code otherwise.
// See atc_tool_change.c for full preconditions and behaviour.
status_code_t tc_probe_tool (void);
