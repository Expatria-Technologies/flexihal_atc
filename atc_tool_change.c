/*
  atc_tool_change.c - Tool length measurement for ATC plugin

  Part of grblHAL

  Implements two entry points for tool measurement against a fixed G59.3
  toolsetter, matching the ToolChange_SemiAutomatic behaviour from grblHAL's
  tool_change.c:

    tc_probe_tool()          — measurement only, for carousel tools already
                               loaded by the NGC macro.

    tc_manual_tool_change()  — full manual flow: move to home Z, optionally
                               move to G30 for operator load/unload, enter
                               STATE_TOOL_CHANGE (cycle-start pause), then
                               measure.  Used for tools not in the carousel.

  Both share a common static probe sequence (do_probe_sequence).

  Copyright (c) 2024 rvalotta
  Copyright (c) 2024 rcp1

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

#if ATC_ENABLE == 2

#include <string.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nuts_bolts.h"
#include "grbl/state_machine.h"
#include "grbl/tool_change.h"   // for settings.tool_change, ToolChange_* enums

#include "atc_tool_change.h"

static tc_pause_hook_ptr pause_hook = NULL;

void tc_set_pause_hook (tc_pause_hook_ptr hook)
{
    pause_hook = hook;
}


#ifndef TOOL_CHANGE_PROBE_RETRACT_DISTANCE
#define TOOL_CHANGE_PROBE_RETRACT_DISTANCE 2.0f
#endif

// ---------------------------------------------------------------------------
// Helpers (mirror the static helpers in tool_change.c)
// ---------------------------------------------------------------------------

// Clamp probe target to machine envelope on the given axis.
static void set_probe_target (coord_data_t *target, uint8_t axis)
{
    target->values[axis] -= settings.tool_change.probing_distance;

    if(bit_istrue(sys.homed.mask, bit(axis)) && settings.axis[axis].max_travel < -0.0f)
        target->values[axis] = max(min(target->values[axis],
                                       sys.work_envelope.max.values[axis]),
                                       sys.work_envelope.min.values[axis]);
}

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
// do_probe_sequence() — shared core measurement sequence.
//
// Preconditions: machine homed on XYZ, tool clamped, spindle/coolant off.
// On entry the machine may be anywhere at or above home Z.
//
// Sequence:
//   1. Rapid Z to home (safe clearance before XY move)
//   2. Rapid XY to G59.3 position (notify toolsetter handler before move)
//   3. Rapid Z to G59.3 approach height
//   4. Fast probe down (probe_is_no_error so a miss doesn't hard-fault)
//   5. Retract TOOL_CHANGE_PROBE_RETRACT_DISTANCE
//      - fast pulloff:  slow retract until contact lost (probe_is_away)
//      - standard:      fixed retract then slow re-probe
//   6. Set TLO (or establish reference on first probe)
//   7. Rapid Z to home
// ---------------------------------------------------------------------------
static status_code_t do_probe_sequence (plane_t *plane, tool_data_t *tool)
{
    bool ok;
    plan_line_data_t plan_data;
    gc_parser_flags_t flags = {};
    coord_system_data_t g59_3_offset;
    coord_data_t target = {};

    settings_read_coord_data(CoordinateSystem_G59_3, &g59_3_offset);

    bool use_toolsetter = grbl.on_probe_toolsetter != NULL;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    // ── 1. Z to home ────────────────────────────────────────────────────────
    if(!(ok = go_home_z(&target, plane, &plan_data)))
        goto cleanup;

    // ── 2. XY to toolsetter position ────────────────────────────────────────
    target.values[plane->axis_0] = g59_3_offset.coord.values[plane->axis_0];
    target.values[plane->axis_1] = g59_3_offset.coord.values[plane->axis_1];

    // Notify toolsetter handler with target before the move (driver can
    // enable/pre-position the toolsetter while the machine is in transit).
    if(use_toolsetter)
        grbl.on_probe_toolsetter(tool, &target, false, true);

    if(!(ok = mc_line(target.values, &plan_data)))
        goto cleanup;

    // ── 3. Z to G59.3 approach height ───────────────────────────────────────
    target.values[plane->axis_linear] = g59_3_offset.coord.values[plane->axis_linear];
    if(!(ok = mc_line(target.values, &plan_data)))
        goto cleanup;

    // ── 4. Fast probe downward ───────────────────────────────────────────────
    plan_data_init(&plan_data);
    plan_data.feed_rate = settings.tool_change.seek_rate;

    // probe_is_no_error: a miss returns GCProbe_Failed rather than raising an
    // alarm, consistent with tc_probe_workpiece in tool_change.c.
    flags.probe_is_no_error = On;

    if(use_toolsetter)
        plan_data.condition.probing_toolsetter =
            grbl.on_probe_toolsetter(tool, NULL, true, true);

    set_probe_target(&target, plane->axis_linear);

    if(!(ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
        goto cleanup;

    // ── 5. Retract and slow probe ────────────────────────────────────────────
    system_convert_array_steps_to_mpos(target.values, sys.probe_position);
    target.values[plane->axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;

    flags.probe_is_no_error = Off;

    if((flags.probe_is_away = settings.flags.tool_change_fast_pulloff)) {
        // Fast pull-off: move away slowly until contact is lost
        plan_data.feed_rate = settings.tool_change.feed_rate;
    } else {
        // Standard: retract a fixed distance, then re-probe slowly
        plan_data.feed_rate = settings.tool_change.pulloff_rate;
        if((ok = mc_line(target.values, &plan_data))) {
            plan_data.feed_rate = settings.tool_change.feed_rate;
            target.values[plane->axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
        }
    }

    if(!(ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
        goto cleanup;

    // ── 6. Set TLO ───────────────────────────────────────────────────────────
    if(!(sys.tlo_reference_set.mask & bit(plane->axis_linear))) {
        // No reference yet — establish it from this probe
        sys.tlo_reference[plane->axis_linear] = sys.probe_position[plane->axis_linear];
        sys.tlo_reference_set.mask |= bit(plane->axis_linear);
        report_add_realtime(Report_TLOReference);
        grbl.report.feedback_message(Message_ReferenceTLOEstablished);
    } else {
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane->axis_linear,
                           sys.probe_position[plane->axis_linear] -
                           sys.tlo_reference[plane->axis_linear]);
    }

    // ── 7. Retract to home Z ─────────────────────────────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.values[plane->axis_linear] = sys.home_position[plane->axis_linear];
    ok = mc_line(target.values, &plan_data);

    if(ok)
        protocol_buffer_synchronize();

cleanup:
    if(use_toolsetter)
        grbl.on_probe_toolsetter(tool, NULL, true, false);

    sync_position();

    return ok ? Status_OK : Status_GCodeToolError;
}

// ---------------------------------------------------------------------------
// tc_probe_tool()
//
// Measurement-only entry point for carousel tools.  Called by $TCMEASURE
// after the NGC macro has physically loaded the tool into the spindle.
// Does not pause for operator interaction.
//
// Preconditions:
//   - Machine homed on XYZ
//   - Tool clamped in spindle, spindle/coolant off
//   - COMPATIBILITY_LEVEL <= 1
// ---------------------------------------------------------------------------
status_code_t tc_probe_tool (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    return Status_GcodeUnsupportedCommand;
#else
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    plane_t plane;
    get_probe_plane(&plane, &parser_state->modal);

    return do_probe_sequence(&plane, gc_state.tool);
#endif
}

// ---------------------------------------------------------------------------
// tc_operator_unload_pause()
//
// Moves to home Z, optionally moves to G30, runs the pause hook, then
// waits in STATE_TOOL_CHANGE for the operator to remove the current tool
// and press cycle start.  Does NOT probe — used when the outgoing tool is
// hand-loaded and the incoming tool will be picked from the carousel.
//
// After cycle start the machine rapids back to home Z before returning so
// the carousel pick sequence starts from a known safe position.
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
    if(settings.flags.tool_change_at_g30 &&
       (sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) == (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) {

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

    // ── 3. Optional pause hook ───────────────────────────────────────────────
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

    // ── 5. Z back to home — ready for carousel pick ──────────────────────────
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

// tc_manual_tool_change()
//
// Full manual tool change + measurement for tools not in the carousel.
// Mirrors ToolChange_SemiAutomatic from tool_change.c with the addition of
// the optional G30 transit.
//
// Sequence:
//   1. Rapid Z to home
//   2. If settings.flags.tool_change_at_g30: rapid XY to G30 position
//   3. Enter STATE_TOOL_CHANGE — wait for operator to load tool and press
//      cycle start
//   4. Rapid Z back to home (operator may have jogged the machine)
//   5. Probe sequence (do_probe_sequence)
//
// The NGC macro is responsible for restore (spindle/coolant/position) after
// this function returns, consistent with the carousel change flow.
//
// Preconditions:
//   - Machine homed on XYZ
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
    if(settings.flags.tool_change_at_g30 &&
       (sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) == (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) {

        coord_system_data_t g30_offset;
        settings_read_coord_data(CoordinateSystem_G30, &g30_offset);

        // XY move at home Z
        target.values[plane.axis_0]      = g30_offset.coord.values[plane.axis_0];
        target.values[plane.axis_1]      = g30_offset.coord.values[plane.axis_1];
        target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];

        if(!mc_line(target.values, &plan_data))
            return Status_Reset;

        // Descend to G30 Z if it differs from home Z
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
    // Registered by the ATC plugin (e.g. to run atc_pause.ngc).
    // Called after the machine has arrived at the change position so any
    // operator signal (light, buzzer, message) fires at the right location.
    if(pause_hook != NULL) {
        status_code_t hook_status = pause_hook();
        if(hook_status != Status_OK)
            return hook_status;
    }

    // ── 4. Enter tool change state — pause for operator ──────────────────────
    // Sets STATE_TOOL_CHANGE; execution resumes when the operator presses
    // cycle start (same mechanism as $TCWAIT in the NGC macros).
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    protocol_execute_realtime();

    if(ABORTED)
        return Status_Reset;

    // ── 5. Z back to home after operator interaction ─────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if(!go_home_z(&target, &plane, &plan_data))
        return Status_Reset;

    // ── 6. Measure ───────────────────────────────────────────────────────────
    return do_probe_sequence(&plane, gc_state.tool);
#endif
}

#endif // ATC_ENABLE == 2
