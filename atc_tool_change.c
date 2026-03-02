/*
  atc_tool_change.c - Tool length measurement for ATC plugin

  Part of grblHAL

  Implements tc_probe_tool(), a standalone G59.3 toolsetter probe sequence
  for use after an automatic carousel tool change.  This duplicates the
  probe logic from grblHAL's tool_change.c (execute_probe / SemiAutomatic
  mode) without requiring an active tool-change state or cycle-start trap.

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

#ifndef TOOL_CHANGE_PROBE_RETRACT_DISTANCE
#define TOOL_CHANGE_PROBE_RETRACT_DISTANCE 2.0f
#endif

// Set probe target, clamping to machine envelope if homed.
// Mirrors the static set_probe_target() in tool_change.c.
static void set_probe_target (coord_data_t *target, uint8_t axis)
{
    target->values[axis] -= settings.tool_change.probing_distance;

    if(bit_istrue(sys.homed.mask, bit(axis)) && settings.axis[axis].max_travel < -0.0f)
        target->values[axis] = max(min(target->values[axis],
                                       sys.work_envelope.max.values[axis]),
                                       sys.work_envelope.min.values[axis]);
}

// Establish plane assignment.
// If TOOL_LENGTH_OFFSET_AXIS is set to a specific axis at compile time
// (>= 0) we use it directly.  If it is -1 (all axes, the default) we
// fall back to the plane passed in from the caller's parser_state.
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
    // TOOL_LENGTH_OFFSET_AXIS == -1: axis determined by active plane modal
    gc_get_plane_data(plane, modal->plane_select);
#endif
}

// ---------------------------------------------------------------------------
// tc_probe_tool()
//
// Probe the tool currently in the spindle against the G59.3 toolsetter and
// set the tool length offset.  Mirrors the SemiAutomatic probe sequence in
// tool_change.c (execute_probe) but runs synchronously without a cycle-start
// trap.
//
// Preconditions:
//   - Machine must be homed on all three axes
//   - G59.3 must be set to the toolsetter position
//   - The tool to be measured must already be clamped in the spindle
//   - Spindle and coolant should be off (caller's responsibility)
//
// Returns:
//   Status_OK                    — TLO set successfully
//   Status_GCodeToolError        — probe did not trigger
//   Status_HomingRequired        — XYZ not fully homed
//   Status_GcodeUnsupportedCommand — COMPATIBILITY_LEVEL > 1
// ---------------------------------------------------------------------------
status_code_t tc_probe_tool (parser_state_t *parser_state)
{
#if COMPATIBILITY_LEVEL > 1
    // SemiAutomatic probing requires COMPATIBILITY_LEVEL <= 1
    return Status_GcodeUnsupportedCommand;
#else
    // ── Precondition checks ──────────────────────────────────────────────────
    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT))
        return Status_HomingRequired;

    plane_t plane;
    get_probe_plane(&plane, &parser_state->modal);

    bool ok;
    plan_line_data_t plan_data;
    gc_parser_flags_t flags = {};
    coord_system_data_t g59_3_offset;
    coord_data_t target = {};

    // G59.3 holds the toolsetter position
    settings_read_coord_data(CoordinateSystem_G59_3, &g59_3_offset);

    // The tool being measured is whatever is currently active
    tool_data_t *tool = gc_state.tool;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    // ── Notify probe_toolsetter handler (raises TLO clear output etc.) ───────
    bool use_toolsetter = grbl.on_probe_toolsetter != NULL;

    // Move at safe Z to toolsetter XY
    system_convert_array_steps_to_mpos(target.values, sys.position);
    target.values[plane.axis_0] = g59_3_offset.coord.values[plane.axis_0];
    target.values[plane.axis_1] = g59_3_offset.coord.values[plane.axis_1];

    if(use_toolsetter)
        grbl.on_probe_toolsetter(tool, &target, false, true);

    if(!(ok = mc_line(target.values, &plan_data)))
        goto cleanup;

    // Descend to toolsetter Z approach height
    target.values[plane.axis_linear] = g59_3_offset.coord.values[plane.axis_linear];
    if(!(ok = mc_line(target.values, &plan_data)))
        goto cleanup;

    // ── Fast probe downward ──────────────────────────────────────────────────
    plan_data.condition.rapid_motion = Off;
    plan_data.feed_rate = settings.tool_change.seek_rate;
    plan_data.condition.value = 0;
    plan_data.spindle.state.value = 0;

    if(use_toolsetter)
        plan_data.condition.probing_toolsetter =
            grbl.on_probe_toolsetter(tool, NULL, true, true);

    set_probe_target(&target, plane.axis_linear);

    if(!(ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
        goto cleanup;

    // ── Retract and slow probe ───────────────────────────────────────────────
    system_convert_array_steps_to_mpos(target.values, sys.probe_position);
    target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;

    if((flags.probe_is_away = settings.flags.tool_change_fast_pulloff)) {
        // Fast pull-off: retract away slowly until contact lost
        plan_data.feed_rate = settings.tool_change.feed_rate;
    } else {
        // Standard: retract a fixed distance then re-probe slowly
        plan_data.feed_rate = settings.tool_change.pulloff_rate;
        if((ok = mc_line(target.values, &plan_data))) {
            plan_data.feed_rate = settings.tool_change.feed_rate;
            target.values[plane.axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
        }
    }

    if(!(ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
        goto cleanup;

    // ── Set tool length offset ───────────────────────────────────────────────
    if(!(sys.tlo_reference_set.mask & bit(plane.axis_linear))) {
        // No reference established yet — use this probe as the reference
        sys.tlo_reference[plane.axis_linear] = sys.probe_position[plane.axis_linear];
        sys.tlo_reference_set.mask |= bit(plane.axis_linear);
        report_add_realtime(Report_TLOReference);
        grbl.report.feedback_message(Message_ReferenceTLOEstablished);
    } else {
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear,
                           sys.probe_position[plane.axis_linear] -
                           sys.tlo_reference[plane.axis_linear]);
    }

    // ── Retract to safe Z ────────────────────────────────────────────────────
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];
    ok = mc_line(target.values, &plan_data);

    if(ok)
        protocol_buffer_synchronize();

cleanup:
    // Always deactivate toolsetter handler regardless of outcome
    if(use_toolsetter)
        grbl.on_probe_toolsetter(tool, NULL, true, false);

    sync_position();

    return ok ? Status_OK : Status_GCodeToolError;
#endif
}

#endif // ATC_ENABLE == 2
