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

#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/state_machine.h"

#include "flexihal_atc.h"

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
    char     alignment;
    char     direction;
    uint8_t  number_of_pockets;
    float    pocket_offset;
    float    x_pocket_1;
    float    y_pocket_1;
    float    z_start;
    float    z_retract;
    float    z_engage;
    float    z_traverse;
    float    z_safe_clearance;
    float    engage_feed_rate;
    bool     tool_setter;
    float    tool_setter_x;
    float    tool_setter_y;
    float    tool_setter_z_seek_start;
    float    tool_setter_seek_feed_rate;
    float    tool_setter_set_feed_rate;
    float    tool_setter_max_travel;
    float    tool_setter_seek_retreat;
    bool     tool_recognition;
    dust_cover_mode_t dust_cover;
    uint8_t  dust_cover_axis;
    float    dust_cover_axis_open;
    float    dust_cover_axis_close;
    atc_ports_t  ports;
    atc_settings_flags_t flags;
} atc_settings_t;

static nvs_address_t nvs_address;
static atc_settings_t atc;
static atc_status_flags_t atc_status;

static tool_data_t current_tool = {0}, *next_tool = NULL;
static coord_data_t target = {0}, previous;

static on_spindle_select_ptr on_spindle_select;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;

static uint8_t n_in_ports;
static uint8_t n_out_ports;
static char max_in_port[4] = "0";
static char max_out_port[4] = "0";

static uint32_t debounce_ms = 0;
static uint32_t polling_ms = 0;
#define DEBOUNCE_DELAY 250

static uint8_t val = 0;
static uint8_t prev_val = 99;
static uint8_t latch = 0;

static atc_ports_t active_ports;

static void handle_userinput(uint_fast16_t state);
static void read_atc_ports(void);

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "FlexiHAL ATC"}
};

status_code_t drawbar_open (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state;
    
    report_message("ATC plugin: Drawbar Open", Message_Info);

    spindle = spindle_get(0);

    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);
   
    //check if the spindle is running or RPM > 0
    if(spindle_state.value){
        report_message("Drawbar cannot open while spindle is running", Message_Warning);
        return Status_UserException;
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
        return Status_UserException;    
        break;
    }
    //proceed to open the drawbar and turn on the taper clear.
    hal.port.digital_out(atc.ports.drawbar_control, 1);
    atc_status.drawbar_control = 1;
    //ensure taper clear is on
    hal.port.digital_out(atc.ports.taper_clear, 1);
    atc_status.taperclear_control = 1;

    return 0;
}

status_code_t drawbar_close (sys_state_t state, char *args)
{
    report_message("ATC plugin: Drawbar Close", Message_Info);
    //close the drawbar
    hal.port.digital_out(atc.ports.drawbar_control, 0);
    atc_status.drawbar_control = 0;
    //ensure taper clear is off
    hal.port.digital_out(atc.ports.taper_clear, 0);
    atc_status.taperclear_control = 0;

    //debounce delay
    hal.delay_ms(350, NULL); // Delay a bit to let the command propagate.

    //check tool and drawbar sensors and stop on issue
    read_atc_ports();

    if(    ((atc_status.drawbar_status == 0)     && (atc.flags.drawbar_status_active))//drawbar is sensed open
        || ((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))//no tool is present
        || (atc_status.drawbar_control == 1)){ //drawbar is commanded to be open

        hal.port.digital_out(atc.ports.air_seal, 0);//ensure air seal is off
        atc_status.airseal_control=0;
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("ATC Malfunction closing drawbar!!", Message_Warning);
        return Status_UserException; 
    }

    return 0;
}

const sys_command_t atc_command_list[2] = {
    {"DRBO", drawbar_open, { .noargs = On }, { .str = "Open the drawbar" }},
	{"DRBC", drawbar_close, { .noargs = On }, { .str = "Close the drawbar" }}
};

static sys_commands_t atc_commands = {
    .n_commands = sizeof(atc_command_list) / sizeof(sys_command_t),
    .commands = atc_command_list
};

sys_commands_t *atc_get_commands()
{
    return &atc_commands;
}

#if 0
ISR_CODE static void read_userinput (uint8_t irq_port, bool is_high)
{
    protocol_enqueue_rt_command(handle_userinput);    
}


static void handle_userinput(uint_fast16_t state){    
    
    report_message("User toggle drawbar", Message_Info);
    hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let the command propagate.
    read_atc_ports();

    if(atc_status.userinput_status)
        grbl.enqueue_gcode("$drawbar_open");
    else
        grbl.enqueue_gcode("$drawbar_close");

}
#endif


static void atc_poll (void)
{
    
    
    uint32_t ms = hal.get_elapsed_ticks();
    if(ms < polling_ms + 100)
        return;

    prev_val = val;
    val = hal.port.wait_on_input(Port_Digital, atc.ports.userinput, WaitMode_Immediate, 0.0f);

    if ((prev_val == 0) && (val == 0) && (latch == 0)) {
        latch = 1;
        grbl.enqueue_gcode("$DRBO");
    } else if ((prev_val == 1) && (val == 1) && (latch == 1)) {
        latch = 0;
        grbl.enqueue_gcode("$DRBC");  // Now only sends once when transitioning latch from 1 to 0
    }  

    polling_ms = ms;   
}

static void read_atc_ports(void){
        uint8_t val;

    if(atc.flags.drawbar_status_active){
        //hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
        val = hal.port.wait_on_input(Port_Digital, atc.ports.drawbar_status, WaitMode_Immediate, 0.0f);//read the IO pin        
        if(val == 1)
            atc_status.drawbar_status = true;
        else
            atc_status.drawbar_status = false;
    }

    if(atc.flags.tool_present_active){
        //hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
        val = hal.port.wait_on_input(Port_Digital, atc.ports.tool_present, WaitMode_Immediate, 0.0f);//read the IO pin        
        if(val == 1)
            atc_status.drawbar_status = true;
        else
            atc_status.drawbar_status = false;
    }

    if(atc.flags.user_input_active){
        //hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
        val = hal.port.wait_on_input(Port_Digital, atc.ports.userinput, WaitMode_Immediate, 0.0f);//read the IO pin        
        if(val == 1)
            atc_status.userinput_status = true;
        else
            atc_status.userinput_status = false;
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

    hal.port.digital_out(atc.ports.air_seal, atc_status.airseal_control);
    
    //If the drawbar is open or the clamp sensor or the tool sensor are not ok, don't start the spindle.
    if (state.value != 0){
        if(    ((atc_status.drawbar_status == 0)     && (atc.flags.drawbar_status_active))//drawbar is sensed open
            || ((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))//no tool is present
            || (atc_status.drawbar_control == 1)) //drawbar is commanded to be open
            {
            state.value = 0; //ensure spindle is off
            atc_status.airseal_control=0;
            hal.port.digital_out(atc.ports.air_seal, atc_status.airseal_control);//ensure air seal is off
            
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

static void atc_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    atc_poll();
}

static void atc_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    atc_poll();
}

static const setting_detail_t atc_settings[] = {
    { 954, Group_AuxPorts, "User Input Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.userinput, NULL, NULL, { .reboot_required = On } },
    { 955, Group_AuxPorts, "Tool Present Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.tool_present, NULL, NULL, { .reboot_required = On } },
    { 956, Group_AuxPorts, "Drawbar Status Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.drawbar_status, NULL, NULL, { .reboot_required = On } },
    { 957, Group_AuxPorts, "Drawbar Control Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.drawbar_control, NULL, NULL, { .reboot_required = On } },
    { 958, Group_AuxPorts, "Air Seal Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.air_seal, NULL, NULL, { .reboot_required = On } },
    { 959, Group_AuxPorts, "Taper Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.taper_clear, NULL, NULL, { .reboot_required = On } },
    { 960, Group_AuxPorts, "TLO Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.tlo_clear, NULL, NULL, { .reboot_required = On } },
    { 961, Group_AuxPorts, "ATC Flags", NULL, Format_Bitfield, "User Input Enabled, Tool Detect Enabled, Drawbar Status Enabled, Drawbar Control Enabled, Air Seal Control Enabled, Taper Clear Enabled, Toolsetter Clear Enabled", NULL, NULL, Setting_NonCore, &atc.flags, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t atc_descriptions[] = {

    { 954, "Aux input port for drawbar user input" },
    { 955, "Aux input port for tool detection" },
    { 956, "Aux input port for drawbar status" },
    { 957, "Aux output port for drawbar control" },
    { 958, "Aux output port for air seal control" },
    { 959, "Aux output port for taper clear control" },
    { 960, "Aux output port for toolsetter clearing" },
    { 961, "Aux input for ATC button is enabled.\\n"
            "Aux input for tool clamp sensor is enabled.\\n"
            "Aux input for drawbar status is enabled.\\n"
            "Aux output for drawbar control is enabled.\\n"
            "Aux output for air seal is enabled.\\n"    
            "Aux output for taper clear is enabled.\\n"
            "Aux output for toolsetter clear is enabled.\\n"                          
            "NOTE: A hard reset of the controller is required after changing this setting."
    },      
};

#endif

static void warning_no_port (uint_fast16_t state)
{
    report_message("ATC plugin: configured port number is not available", Message_Warning);
}

// Hal settings API
// Restore default settings and write to non volatile storage (NVS).
static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));

    atc.ports.userinput = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.tool_present = hal.port.num_digital_in ? hal.port.num_digital_in - 1 : 0;
    atc.ports.drawbar_status = hal.port.num_digital_in ? hal.port.num_digital_in - 1 : 0;
    atc.ports.drawbar_control = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.air_seal = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.taper_clear = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    atc.ports.tlo_clear = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;    
    atc.flags.value = 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

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

    // Sanity check
    if(atc.ports.tool_present >= n_input_ports)
        atc.ports.tool_present = n_input_ports - 1;

    active_ports.tool_present = atc.ports.tool_present;        

    if(atc.ports.drawbar_status >= n_input_ports)
        atc.ports.drawbar_status = n_input_ports - 2;

    active_ports.drawbar_status = atc.ports.drawbar_status;         

    if(atc.ports.userinput >= n_input_ports)
        atc.ports.userinput = n_input_ports - 3; 

    active_ports.userinput = atc.ports.userinput; 

    if(atc.ports.drawbar_control >= n_output_ports)
        atc.ports.drawbar_control = n_output_ports - 1;
    
    active_ports.drawbar_control = atc.ports.drawbar_control;

    if(atc.ports.taper_clear >= n_output_ports)
        atc.ports.taper_clear = n_output_ports - 2;
    
    active_ports.taper_clear = atc.ports.taper_clear;   

    if(atc.ports.air_seal >= n_output_ports)
        atc.ports.air_seal = n_output_ports - 3;
    
    active_ports.air_seal = atc.ports.air_seal;  

    if(atc.ports.tlo_clear >= n_output_ports)
        atc.ports.tlo_clear = n_output_ports - 4;
    
    active_ports.tlo_clear = atc.ports.tlo_clear;                        

    

    if(atc.flags.user_input_active){
        if(ioport_claim(Port_Digital, Port_Input, &active_ports.userinput, "ATC User Input")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    

        //Try to register the interrupt handler.
        //if(!(hal.port.register_interrupt_handler(active_ports.userinput, IRQ_Mode_Change, read_userinput)))
        //    protocol_enqueue_rt_command(warning_no_port);
    }

    if(atc.flags.tool_present_active){
        if(ioport_claim(Port_Digital, Port_Input, &active_ports.tool_present, "Tool Present")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }
    if(atc.flags.drawbar_status_active){
        if(ioport_claim(Port_Digital, Port_Input, &active_ports.drawbar_status, "Drawbar Open/Closed")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }

    if(atc.flags.drawbar_control_active){
        if(ioport_claim(Port_Digital, Port_Output, &active_ports.drawbar_control, "Drawbar Control")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }
    if(atc.flags.taper_clear_active){
        if(ioport_claim(Port_Digital, Port_Output, &active_ports.taper_clear, "Taper Clear")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }       
    if(atc.flags.air_seal_active){
        if(ioport_claim(Port_Digital, Port_Output, &active_ports.air_seal, "Air Seal")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }       
    if(atc.flags.tlo_clear_active){
        if(ioport_claim(Port_Digital, Port_Output, &active_ports.tlo_clear, "Toolsetter Clear")) {
        } else
            protocol_enqueue_rt_command(warning_no_port);    
        //Not an interrupt pin.
    }                                         
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

// HAL plugin API
// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    FLEXIHAL_DEBUG_PRINT("Reset.");
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change
        if(current_tool.tool_id != next_tool->tool_id) {
            if(grbl.tool_table.n_tools)
                memcpy(gc_state.tool, &current_tool, sizeof(tool_data_t));
            else
                memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
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

    if(!newopt) {
        hal.stream.write("[PLUGIN: FlexiHAL ATC v0.01]" ASCII_EOL);
    }
}

static void atc_reset (void)
{
   
    driver_reset();
}

// Claim HAL tool change entry points and clear current tool offsets.
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

    driver_reset = hal.driver_reset;
    hal.driver_reset = atc_reset;    

    atc_commands.on_get_commands = grbl.on_get_commands;
    grbl.on_get_commands = atc_get_commands;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = atc_poll_realtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = atc_poll_delay;

    //hal.tool.select = tool_select;
    //hal.tool.change = tool_change;

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
