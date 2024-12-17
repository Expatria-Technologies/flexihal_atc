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

#include "flexihal_atc.h"

// Used to print debug statements in the normal stream
#define FLEXIHAL_DEBUG 1

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

static const char *atc_port_names[] = {
    "User Input",
    "Tool Preset",
    "Drawbar Status",
    "Drawbar Control",
    "Air Seal",
    "Taper Clear",
    "TLO Clear",
};

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

static uint8_t n_in_ports;
static uint8_t n_out_ports;
static char max_in_port[4] = "0";
static char max_out_port[4] = "0";

static atc_ports_t active_ports;

static void handle_userinput(uint_fast16_t state);
static void read_atc_ports(void);

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "FlexiHAL ATC"}
};

ISR_CODE static void read_userinput (uint8_t irq_port, bool is_high)
{
    protocol_enqueue_rt_command(handle_userinput);    
}


static void handle_userinput(uint_fast16_t state){    

    report_message("User toggle drawbar", Message_Info);
    read_atc_ports();
 
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
        hal.port.digital_out(atc.ports.air_seal, 1);
    else
        hal.port.digital_out(atc.ports.air_seal, 0);
    
    //If the drawbar is open or the clamp sensor or the tool sensor are not ok, don't start the spindle.
    if((    (atc_status.drawbar_status == 0) //drawbar is sensed open
        || (atc_status.toolpresent_status == 0) //no tool is present
        || (atc_status.drawbar_control == 1)) //drawbar is commanded to be open
        && (state.value !=0) ){
        state.value = 0; //ensure spindle is off
        hal.port.digital_out(atc.ports.air_seal, 0);//ensure air seal is off
        grbl.enqueue_realtime_command(CMD_STOP);
        report_message("ATC Malfunction setting spindle state!!", Message_Warning);
    }

    on_spindle_set_state(spindle, state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{   
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static uint32_t atc_get_int (setting_id_t id)
{
    uint32_t value = 0;
    switch((uint32_t)id) {
        case 950:
            value = atc.dust_cover;
            break;
        case 951:
            value = bit(atc.dust_cover_axis);
            break;
        default:
            break;
    }

    return value;
}

static status_code_t set_dust_cover_mode (setting_id_t id, uint_fast16_t int_value)
{
    if(int_value <= DustCover_UsePort) {
        atc.dust_cover = (dust_cover_mode_t)int_value;
    } else
        return Status_InvalidStatement;

    return Status_OK;
}

static status_code_t set_dust_cover_axis_mask (setting_id_t id, uint_fast16_t int_value)
{
    // Allow only one bit / axis set
    if (!(int_value && !(int_value & (int_value-1))))
        return Status_InvalidStatement;

    atc.dust_cover_axis = log2(int_value);

    return Status_OK;
}

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch((uint32_t)setting->id) {
        case 931:
        case 932:
        case 933:
        case 934:
        case 935:
        case 936:
        case 937:
            available = atc.tool_setter;
            break;
        case 941:
            available = atc.tool_recognition && active_ports.tool_present != 0xFF;
            break;
        case 942:
        case 943:
            available = atc.tool_recognition;
            break;
        case 951:
        case 952:
        case 953:
            available = atc.dust_cover == DustCover_UseAxis;
            break;
        default:
            available = true;
            break;
    }

    return available;
}

static const setting_detail_t atc_settings[] = {
    /*
    { 900, Group_UserSettings, "Alignment", "Axis", Format_RadioButtons, "X,Y,Z,A,B", NULL, NULL, Setting_NonCore, &atc.alignment, NULL, NULL },
    { 901, Group_UserSettings, "Direction", NULL, Format_RadioButtons, "Positive,Negative", NULL, NULL, Setting_NonCore, &atc.direction, NULL, NULL },
    { 902, Group_UserSettings, "Number of tool pockets", NULL, Format_Int16, "#00", "0", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
    { 903, Group_UserSettings, "Pocket Offset", "mm", Format_Decimal, "###0", "0",  "9999.999", Setting_NonCore, &atc.pocket_offset, NULL, NULL },
    { 904, Group_UserSettings, "Pocket 1 X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.x_pocket_1, NULL, NULL },
    { 905, Group_UserSettings, "Pocket 1 Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.y_pocket_1, NULL, NULL },
    { 910, Group_UserSettings, "Pocket Z Start Offset", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_start, NULL, NULL },
    { 911, Group_UserSettings, "Pocket Z Retract Offset", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_retract, NULL, NULL },
    { 912, Group_UserSettings, "Pocket Z Engage", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_engage, NULL, NULL },
    { 913, Group_UserSettings, "Pocket Z Traverse", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_traverse, NULL, NULL },
    { 914, Group_UserSettings, "Pocket Z Safe Clearance", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_safe_clearance, NULL, NULL },

    { 930, Group_UserSettings, "Tool Setter", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_setter, NULL, NULL },
    { 931, Group_UserSettings, "Tool Setter X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_x, NULL, is_setting_available },
    { 932, Group_UserSettings, "Tool Setter Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_y, NULL, is_setting_available },
    { 933, Group_UserSettings, "Tool Setter Z Seek Start", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_z_seek_start, NULL, is_setting_available },
    { 934, Group_UserSettings, "Tool Setter Seek Feed Rate", "mm/min", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.tool_setter_seek_feed_rate, NULL, is_setting_available },
    { 935, Group_UserSettings, "Tool Setter Set Feed Rate", "mm/min", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.tool_setter_set_feed_rate, NULL, is_setting_available },
    { 936, Group_UserSettings, "Tool Setter Max Travel", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_max_travel, NULL, is_setting_available },
    { 937, Group_UserSettings, "Tool Setter Seek Retreat", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_seek_retreat, NULL, is_setting_available },
    { 940, Group_UserSettings, "Tool Recognition", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_recognition, NULL, NULL }, 
    { 941, Group_AuxPorts, "Tool Recognition Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.tool_recognition_port, NULL, is_setting_available, { .reboot_required = On } },
    { 942, Group_UserSettings, "Tool Recognition Z Zone 1", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_recognition_z_zone_1, NULL, is_setting_available },
    { 943, Group_UserSettings, "Tool Recognition Z Zone 2", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_recognition_z_zone_2, NULL, is_setting_available },
    { 950, Group_UserSettings, "Dust Cover", NULL, Format_RadioButtons, "Disabled, Axis, Port", NULL, NULL, Setting_NonCoreFn, set_dust_cover_mode, atc_get_int, NULL },
    { 951, Group_UserSettings, "Dust Cover Axis", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_NonCoreFn, set_dust_cover_axis_mask, atc_get_int, is_setting_available },
    { 952, Group_UserSettings, "Dust Cover Axis Open Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.dust_cover_axis_open, NULL, is_setting_available },
    { 953, Group_UserSettings, "Dust Cover Axis Close Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.dust_cover_axis_close, NULL, is_setting_available },
    { 955, Group_AuxPorts, "Dust Cover Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.dust_cover_port, NULL, is_setting_available, { .reboot_required = On } },*/
    { 954, Group_AuxPorts, "User Input Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.userinput, NULL, is_setting_available, { .reboot_required = On } },
    { 955, Group_AuxPorts, "Tool Present Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.tool_present, NULL, is_setting_available, { .reboot_required = On } },
    { 956, Group_AuxPorts, "Drawbar Status Port", NULL, Format_Int8, "#0", "0", max_in_port, Setting_NonCore, &atc.ports.drawbar_status, NULL, is_setting_available, { .reboot_required = On } },
    { 957, Group_AuxPorts, "Drawbar Control Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.drawbar_control, NULL, is_setting_available, { .reboot_required = On } },
    { 958, Group_AuxPorts, "Air Seal Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.air_seal, NULL, is_setting_available, { .reboot_required = On } },
    { 959, Group_AuxPorts, "Taper Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.taper_clear, NULL, is_setting_available, { .reboot_required = On } },
    { 960, Group_AuxPorts, "TLO Clear Port", NULL, Format_Int8, "#0", "0", max_out_port, Setting_NonCore, &atc.ports.tlo_clear, NULL, is_setting_available, { .reboot_required = On } },
    { 961, Group_Toolchange, "ATC Flags", NULL, Format_Bitfield, "User Input Enabled, Tool Detect Enabled, Drawbar Status Enabled, Drawbar Control Enabled, Air Seal Control Enabled, Taper Clear Enabled, Toolsetter Clear Enabled", NULL, NULL, Setting_NonCore, &atc.flags, NULL, NULL },

};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t atc_descriptions[] = {
    { 900, "Value: X Axis or Y Axis\\n\\nThe axis along which the tool pockets of the magazine are aligned in the XY plane." },
    { 901, "Value: Positive or Negative\\n\\nThe direction of travel along the alignment axis from pocket 1 to pocket 2, either positive or negative." },
    { 902, "Value: Count\\n\\nThe total number of pockets in the magazine that may be occupied by a tool." },
    { 903, "Value: Distance (mm)\\n\\nThe distance from one pocket to the next when measuring from center to center." },
    { 904, "Value: X Machine Coordinate (mm)\\n\\nThe X axis position referencing the center of the first tool pocket." },
    { 905, "Value: Y Machine Coordinate (mm)\\n\\nThe Y axis position referencing the center of the first tool pocket." },
    { 910, "Value: Z Machine Coordinate Offset (mm)\\n\\nThe Z offset added to Z Engage at which the spindle is started for (dis-)engagement." },
    { 911, "Value: Z Machine Coordinate Offset (mm)\\n\\nThe Z offset added to Z Engage at which the spindle is retracted between engagement." },
    { 912, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle plunges when engaging the clamping nut." },
    { 913, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle traverses the magazine between dropping off and picking up a tool." },
    { 914, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for safe clearances of all obstacles." },
    { 920, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the spindle moves when (dis-)engaging the clamping nut." },
    { 921, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle when loading a tool." },
    { 922, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle when unloading a tool." },
    { 923, "Value: Spindle Ramp-up Wait Time (ms)\\n\\nThe wait time till the spindle reaches the (un-)load speed." },
    { 930, "Value: Enabled or Disabled\\n\\nAllows for enabling or disabling setting the tool offset during a tool change. This can be useful when configuring your magazine or performing diagnostics to shorten the tool change cycle." },
    { 931, "Value: X Machine Coordinate (mm)\\n\\nThe X axis position referencing the center of the tool setter." },
    { 932, "Value: Y Machine Coordinate (mm)\\n\\nThe Y axis position referencing the center of the tool setter." },
    { 933, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle moves before starting the tool setting probe cycle." },
    { 934, "Value: Feed Rate (mm/min)\\n\\nThe feed rate to quickly find the tool change sensor before the slower locating phase." },
    { 935, "Value: Feed Rate (mm/min)\\n\\nThe feed rate to slowly engage tool change sensor to determine the tool offset accurately." },
    { 936, "Value: Distance (mm)\\n\\nThe maximum probing distance for tool setting." },
    { 937, "Value: Distance (mm)\\n\\nThe pull-off distance for the retract move before the slower locating phase." },
    { 940, "Value: Enabled or Disabled\\n\\nEnables or disables tool recognition as part of an automatic tool change. If tool recognition is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 941, "Aux input port number to use for tool recognition IR sensor." },
    { 942, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the clamping nut breaks the IR beam otherwise the nut is not loaded." },
    { 943, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the clamping nut should not break the IR beam otherwise it is not properly threaded." },
    { 950, "Disabled: Dust cover is disabled. \\n\\n"
           "Axis: Use axis to open and close dust cover.\\n\\n"
           "Port: Open and close dust cover via output port.\\n\\n" },
    { 951, "Value: Axis\\n\\nThe axis which controls the dust cover." },
    { 952, "Value: Dust Cover Axis Machine Coordinate (mm)\\n\\nThe dust cover axis position referencing an open dust cover." },
    { 953, "Value: Dust Cover Axis Machine Coordinate (mm)\\n\\nThe dust cover axis position referencing a closed dust cover." },

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
/*     atc.pocket_offset = 45.0f;
    atc.x_pocket_1 = 0.0f;
    atc.y_pocket_1 = 0.0f;
    atc.z_start = 23.0f;
    atc.z_retract = 13.0f;
    atc.z_engage = -10.0f;
    atc.z_traverse = -10.0f;
    atc.z_safe_clearance = -10.0f;

    atc.tool_setter_z_seek_start = -10.0f;
    atc.tool_setter_seek_feed_rate = DEFAULT_TOOLCHANGE_SEEK_RATE;
    atc.tool_setter_set_feed_rate = DEFAULT_TOOLCHANGE_FEED_RATE;
    atc.tool_setter_max_travel = DEFAULT_TOOLCHANGE_PROBING_DISTANCE;
    atc.tool_setter_seek_retreat = 2.0f;

    if(n_in_ports) {
        atc.tool_recognition_port = n_in_ports - 1;
    }
    atc.tool_recognition_z_zone_1 = -10.0f;
    atc.tool_recognition_z_zone_2 = -10.0f;

    atc.dust_cover = DustCover_Disabled;
    atc.dust_cover_axis = N_AXIS - 1;
    atc.dust_cover_axis_open = -10.0f;
    atc.dust_cover_axis_close = -10.0f;
    if(n_out_ports) {
        atc.dust_cover_port = n_out_ports - 1;
    } */

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

/*     bool ok = true;

    ports.tool_recognition = 0xFE;
    ports.dust_cover = 0xFE;
    if(n_in_ports)  {
        // Sanity check
        if(atc.tool_recognition_port >= n_in_ports)
            atc.tool_recognition_port = n_in_ports - 1;

        ports.tool_recognition = atc.tool_recognition_port;
        ok = ioport_claim(Port_Digital, Port_Input, &ports.tool_recognition, atc_port_names[0]);
    }
    if(ok && n_out_ports)  {
        // Sanity check
        if(atc.dust_cover_port >= n_out_ports)
            atc.dust_cover_port = n_out_ports - 1;

        ports.dust_cover = atc.dust_cover_port;
        ok = ioport_claim(Port_Digital, Port_Output, &ports.dust_cover, atc_port_names[1]);
    }

    if(!ok)
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Configured port number(s) not available"); */

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
        if(!(hal.port.register_interrupt_handler(active_ports.userinput, IRQ_Mode_Change, read_userinput)))
            protocol_enqueue_rt_command(warning_no_port);
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

// FluidNC port
static coord_data_t calculate_tool_pos (tool_id_t tool_id) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    target.x = atc.x_pocket_1;
    target.y = atc.y_pocket_1;

    int8_t multiplier = atc.direction ? -1 : 1;
    float tool_offset = (tool_id - 1) * atc.pocket_offset * multiplier;

    if(atc.alignment == X_AXIS)
        target.x = atc.x_pocket_1 + tool_offset;
    else if(atc.alignment == Y_AXIS)
        target.y = atc.y_pocket_1 + tool_offset;

    return target;
}

static coord_data_t get_manual_pos (void) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct

    target.x = atc.tool_setter_x;
    target.y = atc.tool_setter_y;

    return target;
}

static bool tool_has_pocket (tool_id_t tool_id) {
    return tool_id != 0 && tool_id <= atc.number_of_pockets;
}

static coord_data_t get_tool_pos (tool_id_t tool_id) {
    if(tool_has_pocket(tool_id)) {
        return calculate_tool_pos(tool_id);
    } else
        return get_manual_pos();
}

static bool rapid_to_tool_setter_xy() {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.x = atc.tool_setter_x;
    target.y = atc.tool_setter_y;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool rapid_to_pocket_xy(tool_id_t tool_id) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    coord_data_t tool = get_tool_pos(tool_id);
    target.x = tool.x;
    target.y = tool.y;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool rapid_to_z(float position) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.z = position;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool linear_to_z(float position, float feed_rate) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    target.z = position;
    plan_data.feed_rate = feed_rate;
    if(!mc_line(target.values, &plan_data))
        return false;

    // Do not execute (buffer sync) to not introduce delay
    return true;
}

bool spindle_has_tool() {
    return hal.port.wait_on_input(Port_Digital, active_ports.tool_present, WaitMode_Immediate, 0.0f) > 0;
}

static void message_start() {
    char tool_msg[20];
    sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
    FLEXIHAL_DEBUG_PRINT(tool_msg);
    if(next_tool) {
        sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
        FLEXIHAL_DEBUG_PRINT(tool_msg);
    }
}

static bool open_dust_cover_axis(bool open) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.values[atc.dust_cover_axis] = open ? atc.dust_cover_axis_open : atc.dust_cover_axis_open;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}


static void open_dust_cover_output(bool open) {
    //hal.port.digital_out(ports.dust_cover, open);
    // Wait till motion completed
    hal.delay_ms(1000, NULL);
}

static bool open_dust_cover(bool open) {
    if(atc.dust_cover == DustCover_Disabled) {
        return true;
    }

    if(open) {
        FLEXIHAL_DEBUG_PRINT("Open dust cover.");
    } else {
        FLEXIHAL_DEBUG_PRINT("Close dust cover.");
    }

    if(atc.dust_cover == DustCover_UsePort) {
        open_dust_cover_output(open);
        return true;
    } else {
        return open_dust_cover_axis(open);
    }
}

void record_program_state() {
    FLEXIHAL_DEBUG_PRINT("Record program state.");
    // Spindle off and coolant off
    FLEXIHAL_DEBUG_PRINT("Turning off spindle");
    spindle_all_off();
    FLEXIHAL_DEBUG_PRINT("Turning off coolant");
    hal.coolant.set_state((coolant_state_t){0});
    // Save current position.
    system_convert_array_steps_to_mpos(previous.values, sys.position);
    // Establish axis assignments.
    previous.z -= gc_get_offset(Z_AXIS, 1);
    // Store current position as start
    memcpy(&target, &previous, sizeof(coord_data_t));
}

// Restore coolant and spindle status, return controlled point to original position.
static bool restore (void)
{
    #if 0
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    target.z = atc.z_safe_clearance;
    mc_line(target.values, &plan_data);

    if(!settings.flags.no_restore_position_after_M6) {
        memcpy(&target, &previous, sizeof(coord_data_t));
        target.z = atc.z_safe_clearance;
        mc_line(target.values, &plan_data);
    }

    if(protocol_buffer_synchronize()) {

        sync_position();

        coolant_sync(gc_state.modal.coolant);
        spindle_restore(plan_data.spindle.hal, gc_state.modal.spindle.state, gc_state.spindle->rpm);

        if(!settings.flags.no_restore_position_after_M6) {
            previous.z += gc_get_offset(Z_AXIS);
            mc_line(previous.values, &plan_data);
        }
    }

    if(protocol_buffer_synchronize()) {
        sync_position();
        // Already done after load tool
        // memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    return !ABORTED;
    #endif
}

static bool restore_program_state (void) {
    if(current_tool.tool_id == 0) {
        return true;
    }
    FLEXIHAL_DEBUG_PRINT("Restore.");

    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys.position);

    bool ok = restore();

    return ok;
}

static void set_tool_change_state(void) {
    FLEXIHAL_DEBUG_PRINT("Set tool change state.");
    protocol_buffer_synchronize();
    sync_position();
}

static void pause (void) {
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime(); // Execute suspend
}

static bool unload_tool(void) {
    if(!rapid_to_z(atc.z_safe_clearance))
        return false;

    // If we don't have a tool we're done
    if(current_tool.tool_id == 0) {
        return true;
    }

    FLEXIHAL_DEBUG_PRINT("Unload tool.");

    // If the tool has a pocket, unload
    if(tool_has_pocket(current_tool.tool_id)) {

        // Perform first attempt
        if(!rapid_to_pocket_xy(current_tool.tool_id))
            return false;

        if(!rapid_to_z(atc.z_engage + atc.z_start))
            return false;

        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;

        // If we're using tool recognition, handle it
        if(atc.tool_recognition) {
            #if 0
            FLEXIHAL_DEBUG_PRINT("Move to recognition zone 1.");
            if(!rapid_to_z(atc.tool_recognition_z_zone_1))
                return false;

            // If we have a tool, try unloading one more time
            if (spindle_has_tool()) {
                FLEXIHAL_DEBUG_PRINT("Try to unload one more time.");
                if(!rapid_to_z(atc.z_engage + atc.z_start))
                    return false;
                if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
                    return false;
                if(!rapid_to_z(atc.tool_recognition_z_zone_1))
                    return false;
            
            }

            // Whether successful or not, we're done trying
            spin_stop();

            // If we have a tool at this point, rise and pause for manual unloading
            if (spindle_has_tool()) {
                if(!rapid_to_z(atc.z_safe_clearance))
                    return false;
                protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to unload the current tool. Please unload the tool manually and cycle start to continue.");
                pause();
            // Otherwise, get ready to unload
            } else {
                if(!rapid_to_z(atc.z_traverse))
                    return false;
            }

        // If we're not using tool recognition, go straight to traverse height for loading
        } else {
            if(!rapid_to_z(atc.z_traverse))
                return false;
            spin_stop();
        #endif
        }

    // If the tool doesn't have a pocket, let's pause for manual removal
    } else {
        protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Current tool does not have an assigned pocket. Please unload the tool manually and cycle start to continue.");
        pause();
    }

    // The tool has been removed, set current tool to 0, only set for completeness, not used anywhere
    current_tool.tool_id = 0;
    // Cancel tool length offset
    gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

    return true;
}

static bool load_tool(tool_id_t tool_id) {
    // If loading tool 0, we're done
    if(tool_id == 0) {
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
        return true;
    }

    FLEXIHAL_DEBUG_PRINT("Load tool.");

    // If selected tool has a pocket, perform automatic pick up
    if(tool_has_pocket(tool_id)) {
        if(!rapid_to_pocket_xy(tool_id))
            return false;
        if(!rapid_to_z(atc.z_engage + atc.z_start))
            return false;
        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;
        if(!rapid_to_z(atc.z_engage + atc.z_retract))
            return false;
        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;

        // If we're using tool recognition, let's handle it
        if(atc.tool_recognition) {
            #if 0
            FLEXIHAL_DEBUG_PRINT("Move to recognition zone 1.");
            if (!rapid_to_z(atc.tool_recognition_z_zone_1))
                return false;
            spin_stop();

            // If we don't have a tool rise and pause for a manual load
            if (!spindle_has_tool()) {
                if(!rapid_to_z(atc.z_safe_clearance))
                    return false;
                protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to load the selected tool. Please load the tool manually and cycle start to continue.");
                pause();

            // Otherwise we have a tool and can perform the next check
            } else {
                FLEXIHAL_DEBUG_PRINT("Move to recognition zone 2.");
                if (!rapid_to_z(atc.tool_recognition_z_zone_2))
                    return false;
                // If we show to have a tool here, we cross-threaded and need to manually load
                if (spindle_has_tool()) {
                    if(!rapid_to_z(atc.z_safe_clearance))
                        return false;

                    protocol_enqueue_foreground_task(report_warning, "FlexiHAL: Failed to properly thread the selected tool. Please reload the tool manually and cycle start to continue.");
                    pause();
                } // Otherwise all went well
                FLEXIHAL_DEBUG_PRINT("Tool recognized.");
            }
        #endif
        } else {
            if(!rapid_to_z(atc.z_traverse))
                return false;
            spin_stop();
        }


    // Otherwise, there is no pocket so let's rise and pause to load manually
    } else {
        if(!rapid_to_z(atc.z_safe_clearance))
            return false;
        FLEXIHAL_DEBUG_PRINT("Selected tool does not have an assigned pocket.");
        FLEXIHAL_DEBUG_PRINT("Please load the selected tool and press cycle start to continue.");
        pause();
    }

    // We've loaded our tool
    if(protocol_buffer_synchronize()) {
        sync_position();
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    return true;
}

static bool set_tool (void) {
    // If the tool setter is disabled or if we don't have a tool, rise up and be done
    if(!atc.tool_setter || current_tool.tool_id == 0) {
        if(!rapid_to_z(atc.z_safe_clearance))
            return false;
        return true;
    }
    FLEXIHAL_DEBUG_PRINT("Set tool length.");

    FLEXIHAL_DEBUG_PRINT("Move to probe.");
    if(!rapid_to_z(atc.z_safe_clearance))
        return false;
    if(!rapid_to_tool_setter_xy())
        return false;
    if(!rapid_to_z(atc.tool_setter_z_seek_start))
        return false;

    FLEXIHAL_DEBUG_PRINT("Probe cycle.");
    // Probe cycle using GCode interface since tool change interface is private
    plan_line_data_t plan_data;
    gc_parser_flags_t flags = {0};

    plan_data_init(&plan_data);
    plan_data.feed_rate = atc.tool_setter_seek_feed_rate;
    target.z -= atc.tool_setter_max_travel;
    bool ok = true;
    // Locate probe
    if((ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
    {
        system_convert_array_steps_to_mpos(target.values, sys.probe_position);

        // Retract a bit and perform slow probe
        target.z += atc.tool_setter_seek_retreat;
        if((ok = mc_line(target.values, &plan_data))) {
            plan_data.feed_rate = atc.tool_setter_set_feed_rate;
            target.z -= (atc.tool_setter_seek_retreat + 2.0f);
            ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found;
        }
    }

    if(ok) {
        if(!(sys.tlo_reference_set.mask & bit(Z_AXIS))) {
            FLEXIHAL_DEBUG_PRINT("Set TLO reference.");
            sys.tlo_reference[Z_AXIS] = sys.probe_position[Z_AXIS];
            sys.tlo_reference_set.mask |= bit(Z_AXIS);
            system_add_rt_report(Report_TLOReference);
            grbl.report.feedback_message(Message_ReferenceTLOEstablished);
        } else {
            FLEXIHAL_DEBUG_PRINT("Set TLO.");
            gc_set_tool_offset(ToolLengthOffset_EnableDynamic, Z_AXIS,
                               sys.probe_position[Z_AXIS] - sys.tlo_reference[Z_AXIS]);
        }
    }

    FLEXIHAL_DEBUG_PRINT("End of probing.");
    if(ok)
      if(!rapid_to_z(atc.z_safe_clearance))
        return false;

    return ok;
}

// HAL tool change API
// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    FLEXIHAL_DEBUG_PRINT("Tool select.");
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
    char tool_msg[20];
    sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
    FLEXIHAL_DEBUG_PRINT(tool_msg);
    sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
    FLEXIHAL_DEBUG_PRINT(tool_msg);
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    bool ok = true;
    FLEXIHAL_DEBUG_PRINT("Tool change start.");
    if(next_tool == NULL) {
        FLEXIHAL_DEBUG_PRINT("Next tool is not available!");
        return Status_GCodeToolError;
    }

    if(current_tool.tool_id == next_tool->tool_id) {
        FLEXIHAL_DEBUG_PRINT("Current tool selected, tool change bypassed.");
        return Status_OK;
    }

    // Require homing
    uint8_t homed_req = (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);
    if((sys.homed.mask & homed_req) != homed_req) {
        FLEXIHAL_DEBUG_PRINT("Homing is required before tool change.");
        return Status_HomingRequired;
    }

    message_start();
    protocol_buffer_synchronize();

    record_program_state();
    set_tool_change_state();

    ok = open_dust_cover(true);
    if(!ok)
        return Status_GCodeToolError;

    ok = unload_tool();
    if(!ok)
        return Status_GCodeToolError;

    ok = load_tool(next_tool->tool_id);
    if(!ok)
        return Status_GCodeToolError;

    ok = set_tool();
    if(!ok)
        return Status_GCodeToolError;

    ok = open_dust_cover(false);
    if(!ok)
        return Status_GCodeToolError;

    ok = restore_program_state();
    if(!ok)
        return Status_GCodeToolError;

    FLEXIHAL_DEBUG_PRINT("Tool change finished.");

    return Status_OK;
}

static void atc_reset (void)
{
   
    driver_reset();
}

// Claim HAL tool change entry points and clear current tool offsets.
void atc_init (void)
{
    protocol_enqueue_foreground_task(report_info, "FlexiHAL ATC plugin trying to initialize!");

    #if 0
    ports.tool_recognition = 0xFF;
    ports.dust_cover = 0xFF;
    bool ok;
    if(!ioport_can_claim_explicit()) {
        if((ok = hal.port.num_digital_in >= 1)) {
            hal.port.num_digital_in -= 1;
            ports.tool_recognition = hal.port.num_digital_in;
            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Input, ports.tool_recognition, atc_port_names[0]);
        }
        if((ok = ok && hal.port.num_digital_out >= 1)) {
            hal.port.num_digital_out -= 1;
            ports.dust_cover = hal.port.num_digital_out;
            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Output, ports.dust_cover, atc_port_names[1]);
        }
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

    hal.driver_cap.atc = On;

    // Clear TLO reference
    if(sys.tlo_reference_set.mask != 0) {
        sys.tlo_reference_set.mask = 0;
        system_add_rt_report(Report_TLOReference);
    }

    // If initialization runs a second time, clear TLO
    if (!sys.cold_start) {
        FLEXIHAL_DEBUG_PRINT("Clear TLO.");
        gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);
    }
    #endif

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
