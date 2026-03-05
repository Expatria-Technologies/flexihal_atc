/*
  atc_tool_change.c — Tool length measurement for ATC plugin

  Part of grblHAL

  Implements two entry points for tool measurement against a fixed G59.3
  toolsetter, matching the ToolChange_SemiAutomatic behaviour from grblHAL's
  tool_change.c:

    tc_probe_tool()             — measurement only, for carousel tools already
                                  loaded by the NGC macro.  Called by $TCMEASURE.

    tc_manual_tool_change()     — full manual flow: move to home Z, optionally
                                  move to G30 for operator load/unload, enter
                                  STATE_TOOL_CHANGE (cycle-start pause), then
                                  measure.  Used for tools not in the carousel.

    tc_operator_unload_pause()  — pause-only flow for the case where the
                                  outgoing tool is hand-loaded and the incoming
                                  tool will be picked from the carousel.  No probe.

  Measurement is delegated entirely to atc_measure.ngc.  The macro reads all
  feed rates and distances from grblHAL settings via PRM[] inline expressions
  ($342–$345), stores each tool's absolute gauge length in the tool table via
  G10 L11, and activates it via G43.  No parameters are passed from C; the
  macro is self-contained and stateless across calls.

  atc_measure.ngc must be present at /linuxcnc/atc_measure.ngc.
  NGC expression support must be enabled in config.h (or via the Web Builder).

  Copyright (c) 2024 rvalotta
  Copyright (c) 2024 rcp1

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#if ATC_ENABLE == 2

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nuts_bolts.h"
#include "grbl/state_machine.h"
#include "grbl/stream_file.h"

#include "atc_tool_change.h"

static tc_pause_hook_ptr pause_hook = NULL;

void tc_set_pause_hook (tc_pause_hook_ptr hook)
{
    pause_hook = hook;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Resolve probe axis from compile-time TOOL_LENGTH_OFFSET_AXIS or from the
// active plane in the caller's parser_state modal.
static void get_probe_plane (plane_t *plane, gc_modal_t *modal)
{
#if TOOL_LENGTH_OFFSET_AXIS >= 0
    plane->axis_linear = TOOL_LENGTH_OFFSET_AXIS;
  #if TOOL_LENGTH_OFFSET_AXIS == X_AXIS
    plane->axis_0 = Y_AXIS;
    plane->axis_1 = Z_AXIS;
  #elif TOOL_LENGTH_OFFSET_AXIS == Y_AXIS
    plane->axis_0 = Z_AXIS;
    plane->axis_1 = X_AXIS;
  #else // Z_AXIS (default)
    plane->axis_0 = X_AXIS;
    plane->axis_1 = Y_AXIS;
  #endif
#else
    gc_get_plane_data(plane, modal->plane_select);
#endif
}

// Rapid the linear axis to home position.
static bool go_home_z (coord_data_t *target, plane_t *plane, plan_line_data_t *pl_data)
{
    system_convert_array_steps_to_mpos(target->values, sys.position);

    if(target->values[plane->axis_linear] != sys.home_position[plane->axis_linear]) {
        target->values[plane->axis_linear] = sys.home_position[plane->axis_linear];
        return mc_line(target->values, pl_data);
    }

    return true;
}

// ---------------------------------------------------------------------------
// run_measure_macro()
//
// Stream atc_measure.ngc into the grblHAL motion pipeline using the same
// stream_redirect_read() pattern as atc_macro_start() in flexihal_atc.c.
// Execution is synchronous from the caller's perspective: the function does
// not return until the macro has completed (M2/M99) or been aborted.
//
// The macro is responsible for:
//   - cancelling any active TLO (G49)
//   - moving to G59.3 (toolsetter XY and approach Z)
//   - fast seek + slow locate probe cycle
//   - storing the gauge length via G10 L11 P<tool> Z0
//   - activating the stored offset via G43
//   - retracting to home Z
//
// Returns Status_OK on success, Status_FileOpenFailed if the file is missing,
// or Status_Reset if the macro was aborted.
// ---------------------------------------------------------------------------
#define ATC_MEASURE_MACRO "/linuxcnc/atc_measure.ngc"

static status_code_t measure_macro_status;

static status_code_t measure_on_error (status_code_t status)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "ATC measure error: %d", (uint8_t)status);
    report_message(msg, Message_Warning);
    measure_macro_status = status;
    return status;
}

static status_code_t measure_on_eof (vfs_file_t *file, status_code_t status)
{
    measure_macro_status = status;
    return status;
}

static status_code_t run_measure_macro (void)
{
    // In check mode just verify the file exists.
    if(state_get() == STATE_CHECK_MODE) {
        vfs_stat_t st;
        return vfs_stat(ATC_MEASURE_MACRO, &st) == 0 ? Status_OK : Status_FileOpenFailed;
    }

    measure_macro_status = Status_OK;

    vfs_file_t *file = stream_redirect_read(ATC_MEASURE_MACRO,
                                            measure_on_error,
                                            measure_on_eof);
    if(file == NULL) {
        report_message("ATC: atc_measure.ngc not found at /linuxcnc/", Message_Warning);
        return Status_FileOpenFailed;
    }

    // Pump the grblHAL realtime loop until the macro finishes.
    // This mirrors the pattern used by tool_change() in flexihal_atc.c after
    // atc_macro_start() + EXEC_TOOL_CHANGE.
    protocol_execute_realtime();

    return ABORTED ? Status_Reset : measure_macro_status;
}

// ---------------------------------------------------------------------------
// tc_probe_tool()
//
// Measurement-only entry point.  Called by $TCMEASURE after a carousel tool
// change has physically completed; the tool is already clamped.
// Does not move to G30, does not pause for operator interaction.
//
// Preconditions:
//   - Machine homed on XYZ
//   - Tool clamped in spindle, spindle/coolant off
//   - atc_measure.ngc present at /linuxcnc/atc_measure.ngc
//   - NGC expression support enabled
//   - COMPATIBILITY_LEVEL <= 1
// ---------------------------------------------------------------------------
status_code_t tc_probe_tool (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    return Status_GcodeUnsupportedCommand;
#else
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    return run_measure_macro();
#endif
}

// ---------------------------------------------------------------------------
// tc_operator_unload_pause()
//
// Moves to home Z, optionally moves to G30, runs the pause hook, then waits
// in STATE_TOOL_CHANGE for the operator to remove the current hand-loaded
// tool and press cycle start.  Does NOT probe.
//
// Used when the outgoing tool is P0 (not in carousel) and the incoming tool
// is in the carousel.  The carousel pick sequence in atc_change.ngc runs
// after this returns.
//
// After cycle start the machine rapids back to home Z so the carousel pick
// sequence starts from a known safe position.
// ---------------------------------------------------------------------------
status_code_t tc_operator_unload_pause (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    return Status_GcodeUnsupportedCommand;
#else
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    plane_t plane;
    get_probe_plane(&plane, &parser_state->modal);

    plan_line_data_t plan_data;
    coord_data_t target = {};

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    // ── 1. Z to home ────────────────────────────────────────────────────────
    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    // ── 2. Optional move to G30 for operator access ──────────────────────────
    if(settings.flags.tool_change_at_g30) {
        coord_system_data_t g30_offset;
        settings_read_coord_data(CoordinateSystem_G30, &g30_offset);

        target.values[plane.axis_0]      = g30_offset.coord.values[plane.axis_0];
        target.values[plane.axis_1]      = g30_offset.coord.values[plane.axis_1];
        target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];

        if(!mc_line(target.values, &plan_data))
            return Status_Reset;

        if(g30_offset.coord.values[plane.axis_linear] != sys.home_position[plane.axis_linear]) {
            target.values[plane.axis_linear] = g30_offset.coord.values[plane.axis_linear];
            if(!mc_line(target.values, &plan_data))
                return Status_Reset;
        }
    }

    if(!protocol_buffer_synchronize())
        return Status_Reset;

    sync_position();

    // ── 3. Optional pause hook (e.g. runs atc_pause.ngc) ────────────────────
    if(pause_hook != NULL) {
        status_code_t hook_status = pause_hook();
        if(hook_status != Status_OK)
            return hook_status;
    }

    // ── 4. Enter tool change state — pause for operator to remove tool ────────
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    protocol_execute_realtime();

    if(ABORTED)
        return Status_Reset;

    // ── 5. Return to home Z — ready for carousel pick ────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    if(!protocol_buffer_synchronize())
        return Status_Reset;

    sync_position();

    return Status_OK;
#endif
}

// ---------------------------------------------------------------------------
// tc_manual_tool_change()
//
// Full manual tool change + measurement for tools not in the carousel.
// Mirrors ToolChange_SemiAutomatic from grblHAL's tool_change.c.
//
// Sequence:
//   1. Rapid Z to home
//   2. If settings.flags.tool_change_at_g30: rapid XY to G30 position
//   3. Optional pause hook (e.g. atc_pause.ngc for chip cover)
//   4. Enter STATE_TOOL_CHANGE — wait for operator to load tool and press
//      cycle start
//   5. Rapid Z to home (operator may have jogged during the pause)
//   6. Run atc_measure.ngc — move to G59.3, probe, set TLO, retract
//
// Preconditions:
//   - Machine homed on XYZ
//   - Spindle/coolant already stopped by caller
//   - atc_measure.ngc present at /linuxcnc/atc_measure.ngc
//   - NGC expression support enabled
//   - COMPATIBILITY_LEVEL <= 1
// ---------------------------------------------------------------------------
status_code_t tc_manual_tool_change (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    return Status_GcodeUnsupportedCommand;
#else
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    plane_t plane;
    get_probe_plane(&plane, &parser_state->modal);

    plan_line_data_t plan_data;
    coord_data_t target = {};

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    // ── 1. Z to home ────────────────────────────────────────────────────────
    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    // ── 2. Optional move to G30 for operator access ──────────────────────────
    if(settings.flags.tool_change_at_g30) {
        coord_system_data_t g30_offset;
        settings_read_coord_data(CoordinateSystem_G30, &g30_offset);

        // XY transit at home Z, then descend if G30 Z differs from home Z
        target.values[plane.axis_0]      = g30_offset.coord.values[plane.axis_0];
        target.values[plane.axis_1]      = g30_offset.coord.values[plane.axis_1];
        target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];

        if(!mc_line(target.values, &plan_data))
            return Status_Reset;

        if(g30_offset.coord.values[plane.axis_linear] != sys.home_position[plane.axis_linear]) {
            target.values[plane.axis_linear] = g30_offset.coord.values[plane.axis_linear];
            if(!mc_line(target.values, &plan_data))
                return Status_Reset;
        }
    }

    if(!protocol_buffer_synchronize())
        return Status_Reset;

    sync_position();

    // ── 3. Optional pause hook ───────────────────────────────────────────────
    if(pause_hook != NULL) {
        status_code_t hook_status = pause_hook();
        if(hook_status != Status_OK)
            return hook_status;
    }

    // ── 4. Enter tool change state — pause for operator ──────────────────────
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    protocol_execute_realtime();

    if(ABORTED)
        return Status_Reset;

    // ── 5. Z to home after operator interaction ──────────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    if(!protocol_buffer_synchronize())
        return Status_Reset;

    sync_position();

    // ── 6. Probe the new tool ────────────────────────────────────────────────
    return run_measure_macro();
#endif
}

#endif // ATC_ENABLE == 2
