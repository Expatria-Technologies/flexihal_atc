/*
  flexihal_atc.c - Tool change routine to support ATC spindles

  Part of grblHAL

  Copyright (c) 2024 rvalotta
  Copyright (c) 2024 rcp1

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

#if ATC_ENABLE == 2

#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/state_machine.h"
#include "grbl/ngc_flowctrl.h"
#include "grbl/stream_file.h"
#include "grbl/ngc_params.h"

#if TOOLTABLE_ENABLE == 2
#include "tooltable.h"
#endif

#include "atc_tool_change.h"

//#include "flexihal_atc.h"

// Used to print debug statements in the normal stream
#define FLEXIHAL_DEBUG 1

#define RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy

static uint8_t n_input_ports;
static uint8_t n_output_ports;

#if FLEXIHAL_DEBUG
#define FLEXIHAL_DEBUG_PRINT(message) \
    hal.stream.write("[ATC]: "); \
    hal.stream.write(message); \
    hal.stream.write(ASCII_EOL)
#else
#define FLEXIHAL_DEBUG_PRINT(...)
#endif

typedef struct {
    uint8_t userinput;
    uint8_t tool_present;
    uint8_t drawbar_status;
    uint8_t drawbar_control;
    uint8_t air_seal; //air seal is always on when spindle is running.
    uint8_t taper_clear;
    uint8_t tlo_clear;
} atc_ports_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        drawbar_control     :1, //1 is open, 0 is closed.
        airseal_control     :1, //1 is on, 0 is off
        taperclear_control  :1, 
        tloclear_control    :1,
        drawbar_status      :1, //1 is open, 0 is closed
        toolpresent_status  :1, //1 is present, 0 is absent
        userinput_status    :1, //1 is on, 0 is off
        reserved    :1;
    };
} atc_status_flags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        user_input_active   :1,
        tool_present_active   :1,
        drawbar_status_active   :1,
        drawbar_control_active   :1,
        air_seal_active   :1,
        taper_clear_active   :1,
        tlo_clear_active   :1,      
        reserved        :2;
    };
} atc_settings_flags_t;

typedef enum {
    DustCover_Disabled = 0,
    DustCover_UseAxis,
    DustCover_UsePort
} dust_cover_mode_t;

typedef struct {
    uint16_t drawbar_delay;
    atc_ports_t  ports;
    atc_settings_flags_t flags;
    int16_t number_of_pockets;
} atc_settings_t;

static nvs_address_t nvs_address;
static atc_settings_t atc;
static atc_status_flags_t atc_status;

static tool_data_t current_tool = {0}, *next_tool = NULL;
//static coord_data_t target = {0}, previous;

static on_spindle_select_ptr on_spindle_select;
static on_probe_toolsetter_ptr on_probe_fixture;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
static tool_change_ptr on_tool_change = NULL;
static parser_state_t *atc_parser_state = NULL; // saved from tool_change(), used by $TCMEASURE
//static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;

static uint8_t n_in_ports;
static uint8_t n_out_ports;
static char max_in_port[4] = "0";
static char max_out_port[4] = "0";

//static uint32_t debounce_ms = 0;
//static uint32_t polling_ms = 0;

static atc_ports_t active_ports;

//static void handle_userinput(uint_fast16_t state);
static void read_atc_ports(void);

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "FlexiHAL ATC"}
};

status_code_t drawbar_open (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state = {0};
    spindle_data_t spinddata;
    spindle_data_t *spindledata = &spinddata;
    
    report_message("ATC plugin: Drawbar Open", Message_Info);

    spindle = spindle_get(0);
    spindledata->rpm = 0.0f;

    if(spindle->get_data)
        spindledata = spindle->get_data(SpindleData_RPM);

    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);
   
    //check if the spindle is running or RPM > 0
    if(spindle_state.on || (spindledata->rpm > 0.0f)){
        report_message("Drawbar cannot open while spindle is running", Message_Warning);
        return 0;
    }

    //Make sure the spindle is off.
    spindle_state.value = 0;    
    spindle_set_state(spindle,spindle_state, 0);
    hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let the command propagate.

    //check that state is either IDLE or TOOL
    switch (state_get()){
    case STATE_IDLE:
    case STATE_TOOL_CHANGE:        
        break;    
    default:
        report_message("Drawbar can only open in IDLE or TOOL state", Message_Warning);
        return 0;    
    }

    //proceed to open the drawbar and turn on the taper clear.
    if (atc.flags.drawbar_control_active)
        hal.port.digital_out(active_ports.drawbar_control, 1);
    atc_status.drawbar_control = 1;
    //ensure taper clear is on
    if (atc.flags.taper_clear_active)
        hal.port.digital_out(active_ports.taper_clear, 1);
    atc_status.taperclear_control = 1;

    //debounce delay
    hal.delay_ms(atc.drawbar_delay, NULL); // Delay a bit to let the command propagate.

    //check tool and drawbar sensors and stop on issue
    read_atc_ports();

    if(    ((atc_status.drawbar_status == 1)     && (atc.flags.drawbar_status_active))//drawbar is sensed closed
        || ((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))//no tool is present
        ){

        if(atc.flags.air_seal_active)
            hal.port.digital_out(active_ports.air_seal, 0);//ensure air seal is off
        atc_status.airseal_control=0;
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("ATC Malfunction opening drawbar!!", Message_Warning);
        return 0; 
    }    

    return 0;
}

status_code_t drawbar_close (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state = {0};
    spindle_data_t spinddata;
    spindle_data_t *spindledata = &spinddata;

    spindle = spindle_get(0);
    spindledata->rpm = 0.0f;

    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);

    if(spindle->get_data)
        spindledata = spindle->get_data(SpindleData_RPM);
   
    //check if the spindle is running or RPM > 0
    if(spindle_state.on || (spindledata->rpm > 0.0f)){
        return 0;
    }        
    
    //check that state is either IDLE or TOOL
    switch (state_get()){
    case STATE_IDLE:
    case STATE_TOOL_CHANGE:        
        break;    
    default:
        return 0;    
    }    
    
    report_message("ATC plugin: Drawbar Close", Message_Info);
    //close the drawbar
    if (atc.flags.drawbar_control_active)
        hal.port.digital_out(active_ports.drawbar_control, 0);
    atc_status.drawbar_control = 0;
    //ensure taper clear is off
    if (atc.flags.taper_clear_active)
        hal.port.digital_out(active_ports.taper_clear, 0);
    atc_status.taperclear_control = 0;

    //debounce delay
    hal.delay_ms(atc.drawbar_delay, NULL); // Delay a bit to let the command propagate.

    //check tool and drawbar sensors and stop on issue
    read_atc_ports();

    if(    ((atc_status.drawbar_status == 0)     && (atc.flags.drawbar_status_active))//drawbar is sensed open
        || ((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))//no tool is present
        ){

        if(atc.flags.air_seal_active)
            hal.port.digital_out(active_ports.air_seal, 0);//ensure air seal is off
        atc_status.airseal_control=0;
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("ATC Malfunction closing drawbar!!", Message_Warning);
        return 0; 
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Carousel management commands
// ---------------------------------------------------------------------------

// $TCADD Tn  — Add the tool currently in the spindle to a free carousel pocket.
//
// Usage:
//   $TCADD T3    register tool 3 in the next free pocket
//
// The machine must be IDLE and a tool must be present in the spindle.
// The tooltable plugin assigns the pocket number automatically.

static status_code_t carousel_add (sys_state_t state, char *args)
{
#if TOOLTABLE_ENABLE != 2
    report_message("TCADD requires TOOLTABLE_ENABLE=2", Message_Warning);
    return Status_InvalidStatement;
#else
    // Must be idle
    if(state_get() != STATE_IDLE) {
        report_message("TCADD: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    // Parse tool number from args (expect "Tn").
    // If no argument is given, default to the tool currently in the spindle.
    uint32_t tool_id;

    if(!args || !*args) {
        tool_id = (uint32_t)gc_state.tool->tool_id;
        if(tool_id == 0) {
            report_message("TCADD: no tool selected and no argument given", Message_Warning);
            return Status_BadNumberFormat;
        }
    } else {
        if(*args != 'T' && *args != 't') {
            report_message("TCADD: usage is $TCADD Tn  (or $TCADD to use current tool)", Message_Warning);
            return Status_BadNumberFormat;
        }
        uint_fast8_t cc = 1;
        status_code_t parse_status = read_uint(args, &cc, &tool_id);
        if(parse_status != Status_OK) {
            report_message("TCADD: invalid tool number", Message_Warning);
            return parse_status;
        }
    }

    // Optional: check that a tool is physically present in the spindle
    if(atc.flags.tool_present_active) {
        read_atc_ports();
        if(!atc_status.toolpresent_status) {
            report_message("TCADD: no tool detected in spindle", Message_Warning);
            return Status_GcodeValueOutOfRange;
        }
    }

    carousel_op_result_t result = tooltable_carousel_add((tool_id_t)tool_id, atc.number_of_pockets);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[60];
                sprintf(msg, "Tool %lu added to carousel (offsets preserved)", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_ToolAlreadyInPocket:
            report_message("TCADD: tool already has a pocket assigned", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_NoPocketAvailable:
            {
                char msg[60];
                sprintf(msg, "TCADD: carousel is full (%u pockets configured)", (unsigned)atc.number_of_pockets);
                report_message(msg, Message_Warning);
            }
            return Status_GcodeValueOutOfRange;

        case CarouselOp_TableNotLoaded:
            report_message("TCADD: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TCADD: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TCADD: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
#endif
}

// $TCRM Tn  — Remove a tool from the carousel (mark its pocket as empty).
//
// Usage:
//   $TCRM T3    remove tool 3 from its carousel pocket
//
// The machine must be IDLE.

static status_code_t carousel_remove (sys_state_t state, char *args)
{
#if TOOLTABLE_ENABLE != 2
    report_message("TCRM requires TOOLTABLE_ENABLE=2", Message_Warning);
    return Status_GcodeUnsupportedCommand;
#else
    // Must be idle
    if(state_get() != STATE_IDLE) {
        report_message("TCRM: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    // Parse tool number from args (expect "Tn")
    if(!args || (*args != 'T' && *args != 't')) {
        report_message("TCRM: usage is $TCRM Tn", Message_Warning);
        return Status_BadNumberFormat;
    }

    uint8_t cc = 1;
    uint32_t tool_id;
    status_code_t parse_status = read_uint(args, &cc, &tool_id);
    if(parse_status != Status_OK) {
        report_message("TCRM: invalid tool number", Message_Warning);
        return parse_status;
    }

    carousel_op_result_t result = tooltable_carousel_remove((tool_id_t)tool_id);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[40];
                sprintf(msg, "Tool %lu removed from carousel", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_ToolNotFound:
            report_message("TCRM: tool not found in carousel", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_TableNotLoaded:
            report_message("TCRM: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TCRM: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TCRM: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
#endif
}

// ---------------------------------------------------------------------------
// hal.tool.change — called by grblHAL when M6 is parsed
//
// Decision tree:
//
//   Requested tool in carousel?
//   ├── YES → enqueue carousel ATC macro; returning tool re-pocketed in onToolChanged
//   └── NO  → outgoing tool in carousel?
//             ├── YES → enqueue "return current tool then pause" macro
//             └── NO  → enqueue "just pause for manual swap" macro
//
// In all cases the actual pocket-table updates happen in tooltable.c's
// onToolChanged() after the macro completes, using state set via
// tooltable_set_m6_prev().
// ---------------------------------------------------------------------------

#if TOOLTABLE_ENABLE == 2

// Query the tooltable for a tool's current carousel pocket (-1 = not in carousel)
static pocket_id_t get_carousel_pocket (tool_id_t tool_id)
{
    if(!grbl.tool_table.n_tools || !grbl.tool_table.get_tool)
        return -1;

    tool_table_entry_t *entry = grbl.tool_table.get_tool(tool_id);
    if(!entry || !entry->data)
        return -1;

    return (pocket_id_t)entry->pocket;
}

#endif // TOOLTABLE_ENABLE

// ---------------------------------------------------------------------------
// NGC macro execution helpers
//
// Mirrors the pattern in macros.c: stream_redirect_read() hooks the file
// into grblHAL's stream system so the NGC executes in the normal motion
// pipeline, with proper error handling and soft-reset cleanup.
// ---------------------------------------------------------------------------

static on_macro_return_ptr atc_on_macro_return = NULL;

static void atc_macro_end (void)
{
    // Restore the macro return handler we displaced
    grbl.on_macro_return = atc_on_macro_return;
    atc_on_macro_return = NULL;
}

static status_code_t atc_macro_on_error (status_code_t status)
{
    char msg[48];
    sprintf(msg, "ATC macro error: %d", (uint8_t)status);
    report_message(msg, Message_Warning);

    // Reset tooltable dirty state so onToolChanged is a no-op
    tooltable_set_m6_prev(M6Origin_Unknown, -1);

    atc_macro_end();
    grbl.report.status_message(status);
    return status;
}

static status_code_t atc_macro_on_eof (vfs_file_t *file, status_code_t status)
{
    if(status != Status_OK)
        tooltable_set_m6_prev(M6Origin_Unknown, -1);

    atc_macro_end();
    return status;
}

// Open an NGC file and redirect the grblHAL input stream to execute it.
// Returns Status_OK (file opened, execution started), or an error code.
static status_code_t atc_macro_start (const char *filename)
{
    vfs_file_t *file;

    if(state_get() == STATE_CHECK_MODE) {
        vfs_stat_t st;
        return vfs_stat(filename, &st) == 0 ? Status_OK : Status_FileOpenFailed;
    }

    if((file = stream_redirect_read(filename, atc_macro_on_error, atc_macro_on_eof)) == NULL) {
        report_message(filename, Message_Warning);
        report_message("ATC: macro file not found", Message_Warning);
        return Status_FileOpenFailed;
    }

    // Displace any existing macro return handler, install ours
    atc_on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = atc_macro_end;

    return Status_OK;
}

static status_code_t tool_change (parser_state_t *parser_state)
{
    atc_parser_state = parser_state; // save for use by $TCMEASURE
    tool_data_t *current = parser_state->tool;

#if TOOLTABLE_ENABLE == 2
    // tool_pending is the tool ID requested by the Tn word before M6
    tool_table_entry_t *incoming_entry = grbl.tool_table.get_tool(parser_state->tool_pending);
    if(!incoming_entry || !incoming_entry->data)
        return on_tool_change ? on_tool_change(parser_state) : Status_OK;

    tool_data_t *incoming = incoming_entry->data;

    if(incoming->tool_id == current->tool_id)
        return Status_OK; // already have the right tool

    next_tool = incoming;
    memcpy(&current_tool, current, sizeof(tool_data_t));

    // entry->pocket holds the carousel pocket, or -1 if not in carousel
    pocket_id_t incoming_pocket = (pocket_id_t)incoming_entry->pocket;
    pocket_id_t outgoing_pocket = get_carousel_pocket(current->tool_id);

    // Tell tooltable.c about the outgoing tool so onToolChanged() can
    // return it to the correct carousel pocket on completion.
    if(outgoing_pocket >= 1)
        tooltable_set_m6_prev(M6Origin_Carousel, outgoing_pocket);
    else
        tooltable_set_m6_prev(M6Origin_Manual, -1);

    status_code_t status;

    if(incoming_pocket >= 1) {
        // ── PATH A: requested tool is in the carousel ──────────────────────
        // Pass T and P as named parameters so the NGC can read them via
        // #<_t> and #<_p> without needing to parse the filename.
        FLEXIHAL_DEBUG_PRINT("M6: tool in carousel, running atc_change.ngc");
        // Pass parameters via numbered params in user range (31-5000):
        //   #4900 = incoming tool number
        //   #4901 = incoming carousel pocket
        //   #4902 = outgoing carousel pocket (0 if outgoing was hand-loaded)
        ngc_param_set(4900, (float)incoming->tool_id);
        ngc_param_set(4901, (float)incoming_pocket);
        ngc_param_set(4902, (float)(outgoing_pocket >= 1 ? outgoing_pocket : 0));
        status = atc_macro_start("/linuxcnc/atc_change.ngc");

    } else {
        // ── PATH B: requested tool is NOT in the carousel ──────────────────
        if(outgoing_pocket >= 1) {
            // Return the outgoing tool to its pocket first, then atc_pause.ngc
            // takes over for the manual swap.
            FLEXIHAL_DEBUG_PRINT("M6: tool not in carousel, running atc_return.ngc");
            // #4902 = outgoing carousel pocket
            ngc_param_set(4902, (float)outgoing_pocket);
            status = atc_macro_start("/linuxcnc/atc_return.ngc");
        } else {
            FLEXIHAL_DEBUG_PRINT("M6: tool not in carousel, running atc_pause.ngc");
            status = atc_macro_start("/linuxcnc/atc_pause.ngc");
        }
    }

    if(status != Status_OK) {
        tooltable_set_m6_prev(M6Origin_Unknown, -1);
        return status;
    }

    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    protocol_execute_realtime();

#else
    // No tooltable — always fall back to a simple pause for manual swap
    next_tool = NULL;
    memcpy(&current_tool, current, sizeof(tool_data_t));
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    protocol_execute_realtime();
#endif

    if(on_tool_change)
        return on_tool_change(parser_state);

    return Status_OK;
}

static status_code_t carousel_measure (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCMEASURE: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    report_message("ATC: measuring tool length", Message_Info);

    status_code_t result = tc_probe_tool(atc_parser_state);

    if(result != Status_OK)
        report_message("TCMEASURE: probe failed", Message_Warning);

    return result;
}

static status_code_t carousel_wait (sys_state_t state, char *args)
{
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    return Status_OK;
}

// ---------------------------------------------------------------------------
// Command table
// ---------------------------------------------------------------------------

const sys_command_t atc_command_list[] = {
    {"DRBO",      drawbar_open,     { .noargs = On  }, { .str = "Open the drawbar" }},
    {"DRBC",      drawbar_close,    { .noargs = On  }, { .str = "Close the drawbar" }},
    {"TCADD",     carousel_add,     { .noargs = Off }, { .str = "Add tool to carousel: $TCADD Tn" }},
    {"TCRM",      carousel_remove,  { .noargs = Off }, { .str = "Remove tool from carousel: $TCRM Tn" }},
    {"TCMEASURE", carousel_measure, { .noargs = On  }, { .str = "Measure current tool length against G59.3 toolsetter" }},
    {"TCWAIT",    carousel_wait,    { .noargs = On  }, { .str = "Pause for manual tool swap (enters tool change mode)" }},
};

static sys_commands_t atc_commands = {
    .n_commands = sizeof(atc_command_list) / sizeof(sys_command_t),
    .commands = atc_command_list
};

sys_commands_t *atc_get_commands()
{
    return &atc_commands;
}

static void atc_poll (void *data)
{
    #define DEBOUNCE_THRESHOLD 3
    #define ZERO_THRESHOLD 10
    
    static uint8_t val = 0;
    static uint8_t prev_val = 99;
    static uint8_t latch = 0;
    static int zero_count = 0;
    static int one_count = 0;    

    read_atc_ports();

    prev_val = val;
    val = atc_status.userinput_status;

    if (val == 0) {
        zero_count++;
        one_count = 0;
    } else {
        one_count++;
        zero_count = 0;
    }

    // Check for transition to active state
    if ((prev_val == 0) && (val == 0) && (latch == 0)) {
        if (zero_count >= DEBOUNCE_THRESHOLD) {
            latch = 1;
            grbl.enqueue_gcode("$DRBO");
            zero_count = 0;
        }
    }
    // Check for transition to inactive state
    else if (((prev_val == 1) && (val == 1) && (latch == 1)) || 
             (zero_count >= ZERO_THRESHOLD)) {
        if (one_count >= 1 || zero_count >= ZERO_THRESHOLD) {
            latch = 0;
            grbl.enqueue_gcode("$DRBC");
            one_count = 0;
            zero_count = 0;
        }
    }
    else {
        zero_count = 0;
        one_count = 0;
    }

    task_delete(atc_poll, NULL);
    task_add_delayed(atc_poll, NULL, 100); 
}

static void read_atc_ports(void)
{
    uint8_t val;

    if(atc.flags.drawbar_status_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.drawbar_status, WaitMode_Immediate, 0.0f);
        atc_status.drawbar_status = (val == 1);
    }

    if(atc.flags.tool_present_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.tool_present, WaitMode_Immediate, 0.0f);
        // BUG FIX: was incorrectly writing to drawbar_status instead of toolpresent_status
        atc_status.toolpresent_status = (val == 1);
    }

    if(atc.flags.user_input_active) {
        val = hal.port.wait_on_input(Port_Digital, active_ports.userinput, WaitMode_Immediate, 0.0f);
        atc_status.userinput_status = (val == 1);
    }
}

static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    read_atc_ports();
    
    //if turning the spindle on, turn on the air seal.
    if ((state.value !=0))
        atc_status.airseal_control=1;
    else
        atc_status.airseal_control=0;        

    if (atc.flags.air_seal_active)
        hal.port.digital_out(active_ports.air_seal, atc_status.airseal_control);
    
    //If the drawbar is open or the clamp sensor or the tool sensor are not ok, don't start the spindle.
    if (state.value != 0){
        if(    ((atc_status.drawbar_status == 0)     && (atc.flags.drawbar_status_active))//drawbar is sensed open
            || ((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))//no tool is present
            || (atc_status.drawbar_control == 1) && (atc.flags.drawbar_control_active)) //drawbar is commanded to be open
            {
            state.value = 0; //ensure spindle is off
            atc_status.airseal_control=0;
            hal.port.digital_out(active_ports.air_seal, atc_status.airseal_control);//ensure air seal is off
            
            grbl.enqueue_realtime_command(CMD_STOP);
            report_message("ATC Malfunction setting spindle state!!", Message_Warning);
        }
    }

    on_spindle_set_state(spindle, state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{   
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

//The grbl.on_probe_fixture event handler is called by the default tool change algorithm when probing at G59.3.
static bool probe_fixture (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    bool status = true;

    if(at_g59_3 && on){
        
        report_message("ATC tool probe", Message_Info);

        if (atc.flags.tlo_clear_active) {
            hal.port.digital_out(active_ports.tlo_clear, 1);
            hal.delay_ms(atc.drawbar_delay, NULL);
            hal.port.digital_out(active_ports.tlo_clear, 0);
        }
    }

    if(on_probe_fixture)
        status = on_probe_fixture(tool, position, at_g59_3, on);

    return status;
}


static const setting_detail_t atc_settings[] = {
    { 953, Group_Toolchange, "ATC Drawbar Delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_NonCore, &atc.drawbar_delay, NULL, NULL, },

    { 954, Group_Toolchange, "ATC User Input Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.userinput, NULL, NULL, { .reboot_required = On } },
    { 955, Group_Toolchange, "ATC Tool Present Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.tool_present, NULL, NULL, { .reboot_required = On } },
    { 956, Group_Toolchange, "ATC Drawbar Status Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.drawbar_status, NULL, NULL, { .reboot_required = On } },
    { 957, Group_Toolchange, "ATC Drawbar Control Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.drawbar_control, NULL, NULL, { .reboot_required = On } },
    { 958, Group_Toolchange, "ATC Air Seal Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.air_seal, NULL, NULL, { .reboot_required = On } },
    { 959, Group_Toolchange, "ATC Taper Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.taper_clear, NULL, NULL, { .reboot_required = On } },
    { 960, Group_Toolchange, "ATC TLO Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.tlo_clear, NULL, NULL, { .reboot_required = On } },
    { 962, Group_Toolchange, "ATC Number of Pockets", "pockets", Format_Int16, "##0", "1", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
    { 961, Group_Toolchange, "ATC Flags", NULL, Format_Bitfield, "User Input Enabled, Tool Detect Enabled, Drawbar Status Enabled, Drawbar Control Enabled, Air Seal Control Enabled, Taper Clear Enabled, Toolsetter Clear Enabled", NULL, NULL, Setting_NonCore, &atc.flags, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t atc_descriptions[] = {
    { 953, "Delay between operating the drawbar and reading the sensors." },
    { 954, "Aux input port for drawbar user input" },
    { 955, "Aux input port for tool detection" },
    { 956, "Aux input port for drawbar status" },
    { 957, "Aux output port for drawbar control" },
    { 958, "Aux output port for air seal control" },
    { 959, "Aux output port for taper clear control" },
    { 960, "Aux output port for toolsetter clearing" },
    { 962, "Number of physical pockets in the carousel (max 9999). $TCADD will refuse to assign a pocket number beyond this limit." },
    { 961, "Aux input for ATC button is enabled.\\n"
            "Aux input for tool clamp sensor is enabled.\\n"
            "Aux input for drawbar status is enabled.\\n\\n"
            "Aux output for drawbar control is enabled.\\n"
            "Aux output for air seal is enabled.\\n"    
            "Aux output for taper clear is enabled.\\n"
            "Aux output for toolsetter clear is enabled.\\n"                                      
    },      
};

#endif

static void warning_no_port (void *data)
{
    report_message("ATC plugin: configured port number is not available", Message_Warning);
}

static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));

    atc.number_of_pockets = 12; // default carousel size
    atc.ports.userinput = hal.port.num_digital_in ? hal.port.num_digital_in - 1 : 0;
    atc.ports.tool_present = hal.port.num_digital_in ? hal.port.num_digital_in - 1 : 0;
    atc.ports.drawbar_status = hal.port.num_digital_in ? hal.port.num_digital_in - 1 : 0;

    atc.ports.drawbar_control = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.air_seal = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.taper_clear = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.tlo_clear = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;

    atc.drawbar_delay = 352;
    atc.flags.value = 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static void atc_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static void atc_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&atc, nvs_address, sizeof(atc_settings_t), true) != NVS_TransferResult_OK)
        atc_settings_restore();

    active_ports.tool_present    = atc.ports.tool_present;
    active_ports.drawbar_status  = atc.ports.drawbar_status;
    active_ports.userinput       = atc.ports.userinput;
    active_ports.drawbar_control = atc.ports.drawbar_control;
    active_ports.taper_clear     = atc.ports.taper_clear;
    active_ports.air_seal        = atc.ports.air_seal;
    active_ports.tlo_clear       = atc.ports.tlo_clear;

    if(atc.flags.user_input_active) {
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.userinput, "ATC User Input"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.tool_present_active) {
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.tool_present, "Tool Present"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.drawbar_status_active) {
        if(!ioport_claim(Port_Digital, Port_Input, &active_ports.drawbar_status, "Drawbar Open/Closed"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.drawbar_control_active) {
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.drawbar_control, "Drawbar Control"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.taper_clear_active) {
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.taper_clear, "Taper Clear"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.air_seal_active) {
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.air_seal, "Air Seal"))
            task_add_immediate(warning_no_port, NULL);
    }

    if(atc.flags.tlo_clear_active) {
        if(!ioport_claim(Port_Digital, Port_Output, &active_ports.tlo_clear, "Toolsetter Clear"))
            task_add_immediate(warning_no_port, NULL);
    }


    //this is where we redirect the tool change
    if (settings.tool_change.mode != ToolChange_Automatic)
        return;  

    on_tool_change = hal.tool.change;
    hal.tool.change = tool_change;    
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

static void reset (void)
{
    FLEXIHAL_DEBUG_PRINT("Reset.");
    if(next_tool) {
        if(current_tool.tool_id != next_tool->tool_id) {
            if(grbl.tool_table.n_tools)
                memcpy(gc_state.tool, &current_tool, sizeof(tool_data_t));
            else
                memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            report_add_realtime(Report_Tool);
        }
        char tool_msg[20];
        sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
        FLEXIHAL_DEBUG_PRINT(tool_msg);
        sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
        FLEXIHAL_DEBUG_PRINT(tool_msg);

        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    driver_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN: FlexiHAL ATC v0.02]" ASCII_EOL);
}

static void atc_reset (void)
{
    driver_reset();
}

static atc_status_t atc_get_state (void)
{
    // If macros.c has claimed hal.tool.change via tc.macro, report Online
    // so that tc_init() does not overwrite it.  If tc.macro is not present,
    // our own tool_change() is in place — also report Online to block tc_init().
    return ATC_Online;
}

void atc_init (void)
{    
    protocol_enqueue_foreground_task(report_info, "FlexiHAL ATC plugin trying to initialize!");

    bool ok = (n_input_ports = ioports_available(Port_Digital, Port_Input));
    ok = (n_output_ports = ioports_available(Port_Digital, Port_Output));

    if(!ioport_can_claim_explicit()) {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, unable to claim required ioports!");
        return;
    } else {
        if((ok = (n_in_ports = ioports_available(Port_Digital, Port_Input)) >= 1))
            strcpy(max_in_port, uitoa(n_in_ports - 1));
        if((ok = ok && (n_out_ports = ioports_available(Port_Digital, Port_Output)) >= 1))
            strcpy(max_out_port, uitoa(n_out_ports - 1));
    }

    if(!ok) {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, unable to claim required ioports!");
        return;
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = onSpindleSelect;

    // Set atc_get_state so tc_init() sees ATC_Online and does not overwrite
    // hal.tool.change with the basic manual change implementation.
    // We do NOT set hal.driver_cap.atc here — leaving it Off allows
    // macros.c to claim hal.tool.change if a tc.macro file is found on the
    // filesystem, which is the intended override behaviour.
    hal.tool.atc_get_state = atc_get_state;

    driver_reset = hal.driver_reset;
    hal.driver_reset = atc_reset;    

    system_register_commands(&atc_commands);

    task_add_delayed(atc_poll, NULL, 1000);

    on_probe_fixture = grbl.on_probe_toolsetter;
    grbl.on_probe_toolsetter = probe_fixture;

    if((nvs_address = nvs_alloc(sizeof(atc_settings_t)))) {
        settings_register(&setting_details);
    } else {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to initialize, no NVS storage for settings!");
    }

    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;
    }
}

#endif //#if ATC_ENABLE == 2
