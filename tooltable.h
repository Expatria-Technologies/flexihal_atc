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

// Origin of the tool that was in the spindle before an M6 change.
// Set by the ATC plugin in hal.tool.change; consumed by tooltable in
// onToolChanged() to decide whether to return the outgoing tool to a pocket.
typedef enum {
    M6Origin_Unknown  = 0, // no M6 in progress / not determined
    M6Origin_Carousel,     // outgoing tool came from the carousel
    M6Origin_Manual        // outgoing tool was hand-loaded
} m6_tool_origin_t;

// Called by the ATC plugin's hal.tool.change handler to record the origin
// and original carousel pocket of the tool being replaced.
// Pass pocket = -1 when origin is M6Origin_Manual.
void tooltable_set_m6_prev (m6_tool_origin_t origin, pocket_id_t pocket);

// Add a tool to the carousel.
// Finds the lowest-numbered free pocket, assigns the tool to it, and
// persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id, uint16_t max_pockets);

// Remove a tool from the carousel (clears pocket_id only — offsets persist)
// and persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id);
