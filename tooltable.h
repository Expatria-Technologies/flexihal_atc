/*
  tooltable.h - file based tooltable, LinuxCNC format

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "grbl/gcode.h"

// Result codes for carousel operations
typedef enum {
    CarouselOp_OK = 0,
    CarouselOp_ToolNotFound,        // tool_id not in tooltable
    CarouselOp_ToolAlreadyInPocket, // tool already has a pocket assigned
    CarouselOp_NoPocketAvailable,   // carousel is full
    CarouselOp_WriteError,          // tooltable file write failed
    CarouselOp_TableNotLoaded       // tooltable not yet loaded
} carousel_op_result_t;

// Called by the ATC plugin's hal.tool.change handler to record the carousel
// pocket of the outgoing tool before M6 motion begins.
// Pass pocket = -1 if the outgoing tool was hand-loaded (not from the carousel).
// onToolChanged() uses this to restore the pocket assignment on completion.
void tooltable_set_m6_prev (pocket_id_t pocket);

// Return the name/comment string for a tool from the RAM index.
// Returns NULL if the tool is not indexed or has no name.
const char *tooltable_get_name (tool_id_t tool_id);

// Register a tool in the tooltable at P0 (not in the carousel).
// If the tool already exists its name is updated if name is non-NULL.
// If the tool already has a pocket assigned, returns CarouselOp_ToolAlreadyInPocket.
carousel_op_result_t tooltable_register_tool (tool_id_t tool_id, const char *name);

// Add a tool to the carousel.
// Finds the lowest-numbered free pocket, assigns the tool to it, and
// persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id, uint16_t max_pockets, const char *name);

// Remove a tool from the carousel (clears pocket_id only — offsets persist)
// and persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id);
