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

#include "atc_tool_change.h"
#include "tooltable.h"

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
// Trigger atc_measure.ngc via the grbl.on_macro_execute hook.
// Ensures the current tool has a table entry first so G65 P2 in the macro
// does not error with "undefined tool".
// Returns Status_Handled on success (macro started), error code otherwise.
// ---------------------------------------------------------------------------

static status_code_t run_measure_macro (void)
{
    bool ok;
    plan_line_data_t plan_data;
    coord_data_t target = {};
    plane_t plane;
    get_probe_plane(&plane, &gc_state.modal);    
   
    // Clear tool_change state so the subsequent home move and probe macro
    // are not blocked by the NGC executor's tool-change guard.
    gc_state.tool_change = false;

    // ── 5. Z to home after operator interaction ──────────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    if(!protocol_buffer_synchronize())
        return Status_Reset;

    sync_position();
    
    // Ensure the current tool has a table entry so G65 P2 in atc_measure.ngc
    // does not error with "undefined tool".
    // During M6, gc_state.tool still holds the old tool while tool_pending
    // holds the incoming tool — matching what #5400 returns in the macro.
    tool_id_t measuring_tool = gc_state.tool_pending != 0
                               ? gc_state.tool_pending
                               : gc_state.tool->tool_id;
    if(measuring_tool != 0)
        tooltable_register_tool(measuring_tool, NULL);

    status_code_t status = grbl.on_macro_execute(ATC_MACRO_ID_MEASURE, (parameter_words_t){0}, 1);
    return (status == Status_Handled) ? Status_OK : status;
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
// tc_reprobe_tool()
//
// Force re-measurement regardless of whether a stored offset already exists.
// Clears the current tool's Z offset in the tool table (G10 L1 P<n> Z0) so
// that the skip-if-already-measured guard in atc_measure.ngc falls through,
// then delegates to run_measure_macro() as normal.
//
// Use this after physically replacing a tool in the spindle.
// ---------------------------------------------------------------------------
status_code_t tc_reprobe_tool (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    return Status_GcodeUnsupportedCommand;
#else
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    if(gc_state.tool->tool_id == 0)
        return Status_GCodeToolError;

    // Ensure the tool has a table entry, then clear its Z offset so that
    // G65 P2 in atc_measure.ngc returns 0 and the skip guard falls through.
    tooltable_register_tool(gc_state.tool->tool_id, NULL);

    tool_data_t tool_data = {};
    tool_data.tool_id = gc_state.tool->tool_id;
    // All offsets and radius remain zero — this clears the Z entry.
    grbl.tool_table.set_tool(&tool_data);

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

/*    // Clear tool_change state so the subsequent home move and probe macro
    // are not blocked by the NGC executor's tool-change guard.
    parser_state->tool_change = false;
    gc_state.tool_change = false;

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

    */
#endif
}

#endif // ATC_ENABLE == 2
