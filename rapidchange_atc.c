/*
  rapidchange_atc.c - Tool change routine to support Rapidchange magazine

  Part of grblHAL

  Copyright (c) 2024 rvalotta

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"

#include "rapidchange_atc.h"

typedef struct {
    char     alignment;
    char     direction;
    uint8_t  number_of_pockets;
    float    pocket_offset;
    float    pocket_1_x_pos;
    float    pocket_1_y_pos;
    float    tool_start_height;
    float    tool_z_retract;
    float    tool_engagement_feed_rate;
    float    tool_pickup_rpm;
    float    tool_dropoff_rpm;
    float    tool_z_engagement;
    float    tool_z_traverse;
    float    tool_z_safe_clearance;
    bool     tool_setter;
    bool     tool_recognition;
    bool     dust_cover;
    float    toolsetter_offset;
    float    toolsetter_seek_rate;
    float    toolsetter_retreat;
    float    toolsetter_feed_rate;
    float    toolsetter_max_travel;
    float    toolsetter_x_pos;
    float    toolsetter_y_pos;
    float    toolsetter_z_start_pos;
    float    toolsetter_safe_z;
    uint8_t  toolrecognition_input;
    float    toolrecognition_detect_zone_1;
    float    toolrecognition_detect_zone_2;
    uint8_t  dust_cover_axis;
    uint8_t  dust_cover_open_position;
    uint8_t  dust_cover_closed_position;
    uint8_t  dust_cover_output;
} atc_settings_t;

static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static nvs_address_t nvs_address;
static atc_settings_t atc;
static tool_data_t current_tool, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "RapidChange ATC"}
};

static const setting_detail_t atc_settings[] = {
    { 900, Group_UserSettings, "Alignment", "Axis", Format_RadioButtons, "X,Y", NULL, NULL, Setting_NonCore, &atc.alignment, NULL, NULL },
    { 901, Group_UserSettings, "Direction", NULL, Format_RadioButtons, "Positive,Negative", NULL, NULL, Setting_NonCore, &atc.direction, NULL, NULL },
    { 902, Group_UserSettings, "Number of tool pockets", NULL, Format_Int8, "#00", "0", "120", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
    { 903, Group_UserSettings, "Pocket Offset", "mm", Format_Int16, "###0", "0", "3000", Setting_NonCore, &atc.pocket_offset, NULL, NULL },
    { 904, Group_UserSettings, "Pocket 1 X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_1_x_pos, NULL, NULL },
    { 905, Group_UserSettings, "Pocket 1 Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_1_y_pos, NULL, NULL },
    { 906, Group_UserSettings, "Spindle Start Height", "mm", Format_Decimal, "-##0.000", "-999.999", "999.999", Setting_NonCore, &atc.tool_start_height, NULL, NULL },
    { 907, Group_UserSettings, "Z Retract", "mm", Format_Decimal, "-##0.000", "-127.000", "127.000", Setting_NonCore, &atc.tool_z_retract, NULL, NULL },
    { 908, Group_UserSettings, "Tool Engagement Feed Rate", "mm/min", Format_Int16, "###0", "0", "3000", Setting_NonCore, &atc.tool_engagement_feed_rate, NULL, NULL },
    { 909, Group_UserSettings, "Tool Pickup RPM", "rpm", Format_Int16, "###0", "0", "24000", Setting_NonCore, &atc.tool_pickup_rpm, NULL, NULL },
    { 910, Group_UserSettings, "Tool Dropoff RPM", "rpm", Format_Int16, "###0", "0", "24000", Setting_NonCore, &atc.tool_dropoff_rpm, NULL, NULL },
    { 911, Group_UserSettings, "Tool Z Engage", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_NonCore, &atc.tool_z_engagement, NULL, NULL },
    { 912, Group_UserSettings, "Tool Z Traverse", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_NonCore, &atc.tool_z_traverse, NULL, NULL },
    { 913, Group_UserSettings, "Tool Z Safe Clearance", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_NonCore, &atc.tool_z_safe_clearance, NULL, NULL },
    { 914, Group_UserSettings, "Tool Setter", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_setter, NULL, NULL },
    { 915, Group_UserSettings, "Tool Recognition", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_recognition, NULL, NULL },
    { 916, Group_UserSettings, "Dust Cover", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.dust_cover, NULL, NULL },
    { 917, Group_UserSettings, "Setter Tool Offset", NULL, Format_Int8, "##0", "0", "255", Setting_NonCore, &atc.toolsetter_offset, NULL, NULL },
    { 918, Group_UserSettings, "Setter Seek Rate", NULL, Format_Int8, "###0", "0", "5000", Setting_NonCore, &atc.toolsetter_seek_rate, NULL, NULL },
    { 919, Group_UserSettings, "Setter Retreat", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.toolsetter_retreat, NULL, NULL },
    { 920, Group_UserSettings, "Setter Feed Rate", NULL, Format_Int16, "###0", "0", "5000", Setting_NonCore, &atc.toolsetter_feed_rate, NULL, NULL },
    { 921, Group_UserSettings, "Setter Max Travel", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.toolsetter_max_travel, NULL, NULL },
    { 922, Group_UserSettings, "Setter X Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolsetter_x_pos, NULL, NULL },
    { 923, Group_UserSettings, "Setter Y Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolsetter_y_pos, NULL, NULL },
    { 924, Group_UserSettings, "Setter Z Start Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolsetter_z_start_pos, NULL, NULL },
    { 925, Group_UserSettings, "Setter Safe Z", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolsetter_safe_z, NULL, NULL },
    { 926, Group_UserSettings, "Tool Recognition Input", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.toolrecognition_input, NULL, NULL },
    { 927, Group_UserSettings, "Tool Recognition Detect Zone 1", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolrecognition_detect_zone_1, NULL, NULL },
    { 928, Group_UserSettings, "Tool Recognition Detect Zone 2", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_NonCore, &atc.toolrecognition_detect_zone_2, NULL, NULL },
    { 929, Group_UserSettings, "Dust Cover Axis", NULL, Format_RadioButtons, "Use Output Pin,A-Axis,B-Axis,C-Axis", NULL, NULL, Setting_NonCore, &atc.dust_cover_axis, NULL, NULL },
    { 930, Group_UserSettings, "Dust Cover Open Position", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.dust_cover_open_position, NULL, NULL },
    { 931, Group_UserSettings, "Dust Cover Closed Position", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.dust_cover_closed_position, NULL, NULL },
    { 932, Group_UserSettings, "Dust Cover Output", NULL, Format_Int8, "##0", "0", "250", Setting_NonCore, &atc.dust_cover_output, NULL, NULL },

};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t atc_descriptions[] = {
    { 900, "Value: X Axis or Y Axis\\n\\nThe axis along which the tool pockets of the magazine are aligned in the XY plane." },
    { 901, "Value: Positive or Negative\\n\\nThe direction of travel along the alignment axis from pocket 1 to pocket 2, either positive or negative." },
    { 902, "Value: Count\\n\\nThe total number of pockets in the magazine that may be occupied by a tool." },
    { 903, "Value: Distance (mm)\\n\\nThe distance from one pocket to the next when measuring from center to center." },
    { 904, "Value: X Machine Coordinate (mm)\\n\\nThe x axis position referencing the center of the first tool pocket." },
    { 905, "Value: Y Machine Coordinate (mm)\\n\\nThe y axis position referencing the center of the first tool pocket." },
    { 908, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the spindle plunges when engaging the clamping nut." },
    { 909, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle clockwise when engaging the clamping nut while picking up a tool." },
    { 910, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle counter-clockwise when engaging the clamping nut while dropping a tool." },
    { 911, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle plunges when engaging the clamping nut." },
    { 912, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle traverses the magazine between dropping off and picking up a tool." },
    { 913, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for safe clearances of all obstacles." },
    { 914, "Value: Enabled or Disabled\\n\\nAllows for enabling or disabling setting the tool offset during a tool change. This can be useful when configuring your magazine or performing diagnostics to shorten the tool change cycle." },
    { 915, "Value: Enabled or Disabled\\n\\nEnables or disables tool recognition as part of an automatic tool change. If tool recognition is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 916, "Value: Enabled or Disabled\\n\\nEnables or disables the dust cover. If a dust cover is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 917, "Value: Distance (mm)\\n\\nThe distance from the surface of the table bed to the top of the tool setter." },
    { 918, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the tool seeks the tool setter on the initial straight probe." },
    { 919, "Value: Distance (mm)\\n\\nThe distance to retreat after contact is made with the tool setter during seek mode." },
    { 920, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the tool plunges toward the tool setter on the final straight probe, performed after retreating from the initial straight probe." },
    { 921, "Value: Distance (mm)\\n\\nThe maximum distance of travel that should be attempted when probing from Z Seek Start." },
    { 922, "Value: X Machine Coordinate (mm)\\n\\nThe X position referencing the center of the tool setter." },
    { 923, "Value: Y Machine Coordinate (mm)\\n\\nThe Y position referencing the center of the tool setter." },
    { 924, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which to begin the initial straight probe." },
    { 925, "Value: Z Machine Coordinate (mm)\\n\\nThe minimum Z position at which it is safe to move above the tool setter with a tool." },
    { 926, "Value: Input Number\\n\\nThe input pin designation for reading the tool recognition sensor state." },
    { 927, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for recognizing the presence of a clamping nut attached to the spindle." },
    { 928, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for recognizing the complete threading of a clamping nut after picking up a tool." },
    { 929, "Value: A Axis, B Axis, or C Axis\\n\\nThe axis assigned for dust cover control. This is required to control the dust cover with an axis." },
    { 930, "Value: A, B, or C Machine Coordinate (mm)\\n\\nThe position along the assigned axis at which the dust cover is fully open." },
    { 931, "Value: A, B, or C Machine Coordinate (mm)\\n\\nThe position along the assigned axis at which the dust cover is fully closed." },
    { 932, "Value: Output Number\\n\\nThe output pin designation for dust cover control. This is required to control the dust cover with a third-party microcontroller." },
};

#endif

// Write settings to non volatile storage (NVS).
static void atc_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

// Load settings from volatile storage (NVS)
static void atc_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&atc, nvs_address, sizeof(atc_settings_t), true) != NVS_TransferResult_OK)
        atc_settings_restore();
}

// Restore default settings and write to non volatile storage (NVS).
static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));
    atc.pocket_offset = 0.0f;
    atc.pocket_1_x_pos = 0.0f;
    atc.pocket_1_y_pos = 0.0f;
    atc.tool_start_height = 0.0f;
    atc.tool_z_retract = 0.0f;
    atc.tool_engagement_feed_rate = 0.0f;
    atc.tool_pickup_rpm = 0.0f;
    atc.tool_dropoff_rpm = 0.0f;
    atc.tool_z_engagement = 0.0f;
    atc.tool_z_traverse = 0.0f;
    atc.tool_z_safe_clearance = 0.0f;
    atc.toolsetter_offset = 0.0f;
    atc.toolsetter_seek_rate = 0.0f;
    atc.toolsetter_retreat = 0.0f;
    atc.toolsetter_feed_rate = 0.0f;
    atc.toolsetter_max_travel = 0.0f;
    atc.toolsetter_x_pos = 0.0f;
    atc.toolsetter_y_pos = 0.0f;
    atc.toolsetter_z_start_pos = 0.0f;
    atc.toolsetter_safe_z = 0.0f;
    atc.toolrecognition_detect_zone_1 = 0.0f;
    atc.toolrecognition_detect_zone_2 = 0.0f;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = atc_groups,
    .n_groups = sizeof(atc_groups) / sizeof(setting_group_detail_t),
    .settings = atc_settings,
    .n_settings = sizeof(atc_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = atc_descriptions,
    .n_descriptions = sizeof(atc_descriptions) / sizeof(setting_descr_t),
#endif
    .save = atc_settings_save,
    .load = atc_settings_load,
    .restore = atc_settings_restore
};

// Return X,Y based on tool number
static coord_data_t get_tool_location(tool_data_t tool) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    target.x = atc.pocket_1_x_pos;
    target.y = atc.pocket_1_y_pos;

    float tool_offset = (tool.tool_id - 1) * atc.pocket_offset;
    if(atc.direction != 0)
        tool_offset *= -1.F;

    if(atc.alignment == 0) { // X Axis
        target.x = atc.pocket_1_x_pos + tool_offset;
    } else {
        target.y = atc.pocket_1_y_pos + tool_offset;
    }

    return target;
}

//     protocol_buffer_synchronize();


// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change

        if(current_tool.tool_id != next_tool->tool_id) {
            memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
        }

        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    driver_reset();
}

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
}

static status_code_t spindle(bool load) {

    debug_output(load ? "Loading" : "Unloading", NULL, NULL);
    coord_data_t target = {0}, current_pos;
    plan_line_data_t plan_data;

    if(current_tool.tool_id == 0 && !load) {
        debug_output("No tool to unload", NULL, NULL);
        return Status_OK;
    }

    if(next_tool->tool_id > atc.number_of_pockets) {
        debug_output("Tool number is larger than pocket. Manual Tool Change", NULL, NULL);
        if(load) {
            manualToolLoad();
        } else {
            manualToolUnLoad();
        }
        return Status_OK;
    }

    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    plan_data_init(&plan_data);

    // Lets stop the spindle and set the feed rate for all moves.
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){0}, 0.0f);
    plan_data.feed_rate = atc.tool_engagement_feed_rate;
    plan_data.condition.rapid_motion = Off;

    system_convert_array_steps_to_mpos(current_pos.values, sys.position);
    debug_output("Getting Current POS", &current_pos, &plan_data);

    // Raise Z to safe clearance
    target = current_pos;
    target.z = atc.tool_z_safe_clearance;
    debug_output("Raising Z to Clearance Height", NULL, &plan_data);
    mc_line(target.values, &plan_data);

    // Get X,Y for current tool and move to that position
    target = get_tool_location((load?*next_tool:current_tool));
    target.z = atc.tool_z_safe_clearance;
    debug_output("Determine tool position and go there", &target, &plan_data);
    mc_line(target.values, &plan_data);

    target.z = atc.tool_start_height;
    debug_output("Going to Spindle Start Height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Turn on the spindle CCW
    if(load) {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On }, atc.tool_pickup_rpm);
    } else {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On, .ccw = On }, atc.tool_dropoff_rpm);
    }

    // move to engagement height
    target.z = atc.tool_z_engagement;
    debug_output("Turning on spindle and moving to engagement height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Are we doing tool recognition
    if(atc.tool_recognition) {
        debug_output("Tool Recognition Enabled", NULL, NULL);
        // Move spindle to zone 2
        target.z = atc.toolrecognition_detect_zone_2;
        debug_output("Moving to zone 2", &target, &plan_data);
        mc_line(target.values, &plan_data);
        // Wait for spindle to be in the correct location
        protocol_buffer_synchronize();
        // IF the nut isn't all the way on lets try again
        if(laserBlocked()) {
            target.z = atc.tool_z_engagement;
            debug_output("Detection Failed Trying again", NULL, NULL);

            mc_line(target.values, &plan_data);
            target.z = atc.toolrecognition_detect_zone_1;
            mc_line(target.values, &plan_data);
            protocol_buffer_synchronize();
        }

        if(laserBlocked()) {
            // TODO: Need to error out.
            return Status_GcodeInvalidTarget;
        }

        // Bring Spindle up and Turn off spindle
        target.z = atc.tool_z_safe_clearance;
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t)(spindle_state_t){ .on = Off }, 0.0f);
        debug_output("Stopping spindle and raising to clearance height", &target, &plan_data);
        mc_line(target.values, &plan_data);

    }
    debug_output("Updating current tool", NULL, NULL);

    if(load) {
        memset(&current_tool, 0, sizeof(tool_data_t));
    } else {
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    protocol_buffer_synchronize();

    return Status_OK;

}

static void manualToolLoad() {

}

static void manualToolUnLoad() {

}

static void measureTool() {
    coord_data_t current_pos;

    system_convert_array_steps_to_mpos(current_pos.values, sys.position);

}

static bool laserBlocked() {

    return false;
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool_id == next_tool->tool_id)
        return Status_OK;

#ifndef DEBUG
    uint8_t homed_req =  (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);

    if((sys.homed.mask & homed_req) != homed_req)
        return Status_HomingRequired;
#endif

    // Save current position
    coord_data_t previous;
    system_convert_array_steps_to_mpos(previous.values, sys.position);

    debug_output("Turning off Coolant", NULL, NULL);

    // Stop spindle and coolant
    hal.coolant.set_state((coolant_state_t){0});

    debug_output("Check if we need to unload tool", NULL, NULL);
    spindle(false);
    debug_output("Check if we need to load a tool", NULL, NULL);
    spindle(true);
    debug_output("Check if we need to measure a tool", NULL, NULL);
    measureTool();


    return Status_OK;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN: RapidChange ATC v0.01]" ASCII_EOL);
    }
}

// Claim HAL tool change entry points and clear current tool offsets.
void my_plugin_init (void)
{
    protocol_enqueue_foreground_task(report_info, "RapidChange ATC plugin trying to initialize!");
    hal.driver_cap.atc = On;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    if(sys.tlo_reference_set.mask != 0) {
        sys.tlo_reference_set.mask = 0;
        system_add_rt_report(Report_TLOReference);
    }

    gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

    hal.tool.select = tool_select;
    hal.tool.change = tool_change;

    if((nvs_address = nvs_alloc(sizeof(atc_settings_t)))) {
         settings_register(&setting_details);
    } else {
        protocol_enqueue_foreground_task(report_warning, "RapidChange ATC plugin failed to initialize, no NVS storage for settings!");
    }

    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;
    }
}

void debug_output(char* message, coord_data_t *target, plan_line_data_t *pl_data) {

#ifdef DEBUG
    hal.stream.write("[R-ATC]: ");
    hal.stream.write(message);
    hal.stream.write(ASCII_EOL);

    if(target != NULL) {
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Target:" ASCII_EOL);
        hal.stream.write("X: ");
        hal.stream.write( ftoa(target->x, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("y: ");
        hal.stream.write( ftoa(target->y, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("z: ");
        hal.stream.write( ftoa(target->z, 3) );
        hal.stream.write(ASCII_EOL);
    }

    if(pl_data != NULL) {
        hal.stream.write(ASCII_EOL "Plan:" ASCII_EOL);
        hal.stream.write("Feed Rate:");
        hal.stream.write(ftoa(pl_data->feed_rate,3));
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Spindle RPM:");
        hal.stream.write(ftoa(pl_data->spindle.rpm,3));
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Spindle State:");
        char buffer[8U] = ""; /*the output buffer*/

        sprintf (buffer, "%d", pl_data->spindle.state.value);
        hal.stream.write(buffer);
        hal.stream.write(ASCII_EOL);

        hal.stream.write(ASCII_EOL);
    }
#endif
}
