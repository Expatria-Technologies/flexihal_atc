/*
  flexihal_atc.c - Tool change routine to support ATC spindles

  Part of grblHAL

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
#if TOOLTABLE_ENABLE == 2
#include "grbl/ngc_flowctrl.h"
#include "grbl/ngc_params.h"
#include "tooltable.h"

// Forward declarations — defined after the command handlers
static pocket_id_t get_carousel_pocket (tool_id_t tool_id);
static status_code_t atc_tc_advance (void);
#endif

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
#if TOOLTABLE_ENABLE == 2
    int16_t number_of_pockets;
#endif
} atc_settings_t;

static nvs_address_t nvs_address;
static atc_settings_t atc;
static atc_status_flags_t atc_status;

static tool_data_t current_tool = {}, *next_tool = NULL;

static on_macro_return_ptr on_macro_return = NULL;

static on_spindle_select_ptr on_spindle_select;
static on_probe_toolsetter_ptr on_probe_fixture;
static spindle_set_state_ptr on_spindle_set_state = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
#if TOOLTABLE_ENABLE == 2
static tool_change_ptr on_tool_change = NULL;
static tool_select_ptr tool_select = NULL;
static parser_state_t *atc_parser_state = NULL; // saved from tool_change(), used by $TCMEASURE

static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;

static control_signals_callback_ptr control_interrupt_callback = NULL;
static enqueue_realtime_command_ptr enqueue_realtime_command = NULL;

#define ATC_MACRO_ID_TCFETCH      390 //fetch from tool carousel
#define ATC_MACRO_ID_TCRETURN     391 //return to tool carousel
#define ATC_MACRO_ID_HANDFETCH    392 //manual tool fetch (raise Z, optional go to G30, then return from macro and trap cycle start.  Update tool number after cycle start)
#define ATC_MACRO_ID_HANDRETURN   393 //manual tool return (raise Z, optional go to G30, then return from macro and trap cycle start.  After cycle start Proceed with TCFETCH or HANDFETCH as appropriate)
#define ATC_MACRO_ID_MEASURE      394 //probe tool length against G59.3 toolsetter

// ---------------------------------------------------------------------------
// Tool change state machine
//
// tool_change() kicks off the sequence and returns Status_Unhandled so the
// stream machinery can drive macro execution asynchronously.
//
// State transitions:
//
//   TC_IDLE
//     │
//     ├─ outgoing in carousel, incoming in carousel
//     │    └─ → TC_RETURN_OUTGOING  (ATC_MACRO_ID_TCRETURN, fully autonomous)
//     │         └─ on macro complete → TC_FETCH_INCOMING  (ATC_MACRO_ID_TCFETCH, fully autonomous)
//     │                               └─ on macro complete → TC_IDLE - any tool in the carousel has been measured.
//     │
//     ├─ outgoing in carousel, incoming hand-loaded
//     │    └─ → TC_RETURN_OUTGOING  (ATC_MACRO_ID_TCRETURN, fully autonomous)
//     │         └─ on macro complete → TC_HAND_FETCH  (ATC_MACRO_ID_HANDFETCH + return and trap cycle-start)
//     │                               └─ on cycle-start → TC_MEASURE (ATC_MACRO_ID_MEASURE) only if the tool has not been previously measured
//     │                                                    └─ on macro complete → TC_IDLE
//     │
//     ├─ outgoing hand-loaded, incoming in carousel
//     │    └─ → TC_HAND_RETURN  (ATC_MACRO_ID_HANDRETURN + return and trap cycle-start)
//     │         └─ on cycle-start → TC_FETCH_INCOMING  (ATC_MACRO_ID_TCFETCH, fully autonomous)
//     │                             └─ on macro complete → TC_IDLE - any tool in the carousel has been measured.
//     │
//     └─ both hand-loaded
//          └─ → TC_HAND_RETURN  (ATC_MACRO_ID_HANDRETURN + return and trap cycle-start)
//               └─ on cycle-start → TC_MEASURE (ATC_MACRO_ID_MEASURE) only if the tool has not been previously measured
//                                    └─ on macro complete → TC_IDLE
// 
// The TC_MEASURE function needs to handle the T0 case before calling the measure macro.  T0 means no tool in the spindle, so measurement should not occur.
// ---------------------------------------------------------------------------
typedef enum {
    TC_IDLE = 0,
    TC_START,               // entry point — tool_change() calls atc_tc_advance() from here
    TC_RETURN_OUTGOING,     // TCRETURN macro running — returning outgoing carousel tool
    TC_HAND_RETURN,         // HANDRETURN macro running — operator must remove; cycle-start trapped
    TC_FETCH_INCOMING,      // TCFETCH macro running — fetching incoming carousel tool
    TC_HAND_FETCH,          // HANDFETCH macro running — operator must install; cycle-start trapped
    TC_MEASURE,             // MEASURE macro running — probing tool length
} tc_state_t;

static tc_state_t   tc_state           = TC_IDLE;
static pocket_id_t tc_incoming_pocket  = -1;  // carousel pocket of incoming tool
static pocket_id_t tc_outgoing_pocket  = -1;  // carousel pocket outgoing tool will return to
static pocket_id_t last_fetched_pocket = -1;  // pocket the current spindle tool came from (volatile, lost on power cycle)
static tool_id_t tc_outgoing_tool_id = 0;  // tool ID of outgoing tool, saved at TC_START

#endif

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
        return Status_OK;
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
        return Status_OK;
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
        if((atc_status.drawbar_status == 1) && (atc.flags.drawbar_status_active))
            report_message("ATC Malfunction opening drawbar: drawbar sensor still reads closed", Message_Warning);
        if((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))
            report_message("ATC Malfunction opening drawbar: tool present sensor reads no tool", Message_Warning);
        return Status_OK;
    }    

    return Status_OK;
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
        return Status_OK;
    }        
    
    //check that state is either IDLE or TOOL
    switch (state_get()){
    case STATE_IDLE:
    case STATE_TOOL_CHANGE:        
        break;    
    default:
        return Status_OK;
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
        if((atc_status.drawbar_status == 0) && (atc.flags.drawbar_status_active))
            report_message("ATC Malfunction closing drawbar: drawbar sensor still reads open", Message_Warning);
        if((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))
            report_message("ATC Malfunction closing drawbar: tool present sensor reads no tool", Message_Warning);
        return Status_OK;
    }

    return Status_OK;
}

#if TOOLTABLE_ENABLE == 2

// change_completed() restores the HAL pointers when the change finishes or
// is aborted.  It mirrors the core's change_completed() in tool_change.c.
// ---------------------------------------------------------------------------

static void change_completed (void)
{
    if(enqueue_realtime_command) {
        while(spin_lock);
        hal.irq_disable();
        hal.stream.set_enqueue_rt_handler(enqueue_realtime_command);
        enqueue_realtime_command = NULL;
        hal.irq_enable();
    }

    if(control_interrupt_callback) {
        while(spin_lock);
        hal.irq_disable();
        hal.control.interrupt_callback = control_interrupt_callback;
        control_interrupt_callback = NULL;
        hal.irq_enable();
    }

    gc_state.tool_change = false;
}

// Task callback — posted by the ISR trampolines, runs in protocol context
// where it is safe to call system_set_exec_state_flag().
static void execute_cycle_start (void *data)
{
    execute_posted = false;
    system_set_exec_state_flag(EXEC_CYCLE_START);
    atc_tc_advance();
}

ISR_CODE static void ISR_FUNC(trap_control_cycle_start)(control_signals_t signals)
{
    spin_lock++;

    if(signals.cycle_start) {
        if(!execute_posted)
            execute_posted = task_add_immediate(execute_cycle_start, NULL);
        signals.cycle_start = Off;
    } else
        control_interrupt_callback(signals);

    spin_lock--;
}

ISR_CODE static bool ISR_FUNC(trap_stream_cycle_start)(uint8_t c)
{
    bool drop = false;

    spin_lock++;

    if((drop = (c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY))) {
        if(!execute_posted)
            execute_posted = task_add_immediate(execute_cycle_start, NULL);
    } else
        drop = enqueue_realtime_command(c);

    spin_lock--;

    return drop;
}

#endif // TOOLTABLE_ENABLE == 2

// ---------------------------------------------------------------------------
// Carousel management commands (TOOLTABLE_ENABLE == 2 only)
// ---------------------------------------------------------------------------

#if TOOLTABLE_ENABLE == 2

typedef enum {
    STANDALONE_NONE = 0,
    STANDALONE_TCRETURN,    // $TCRETURN — return tool, clear spindle
    STANDALONE_TCADD,       // $TCADD — deposit tool, clear spindle
} standalone_op_t;

static standalone_op_t standalone_op     = STANDALONE_NONE;
static tool_id_t       standalone_tool_id = 0;
static pocket_id_t     standalone_pocket  = 0;

static void macro_exit (void)
{
    if(standalone_op != STANDALONE_NONE) {

        // Clear spindle state — motion is now complete
        memset(gc_state.tool, 0, sizeof(tool_data_t));
        gc_state.tool_pending = 0;
        current_tool.tool_id = 0;
        report_add_realtime(Report_Tool);

        char msg[64];
        if(standalone_op == STANDALONE_TCADD)
            sprintf(msg, "T%lu deposited in pocket %d — spindle empty",
                    (unsigned long)standalone_tool_id, (int)standalone_pocket);
        else
            sprintf(msg, "T%lu returned to pocket %d — spindle empty",
                    (unsigned long)standalone_tool_id, (int)standalone_pocket);

        report_message(msg, Message_Info);
        standalone_op = STANDALONE_NONE;
    }

    if(on_macro_return)
        on_macro_return();
}


// $TCADD [Tn] [;name]  — Deposit the current spindle tool into the next free
// carousel pocket and register it in the tooltable.
//
// Usage:
//   $TCADD              use current spindle tool
//   $TCADD T3           use tool 3 (must match tool in spindle)
//   $TCADD T3 ;12mm EM  as above, with a name
//
// The machine must be IDLE and homed.  The tool must be clamped in the spindle.
// The pocket assignment is written to the tooltable first; if the macro file
// cannot be opened the pocket assignment is rolled back so the table stays clean.
// Spindle state is cleared to T0 in macro_exit() once motion completes.

static status_code_t carousel_add (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCADD: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) {
        report_message("TCADD: machine must be homed", Message_Warning);
        return Status_HomingRequired;
    }

    // ── Parse arguments ──────────────────────────────────────────────────────
    uint32_t tool_id;
    const char *name = NULL;

    if(!args || !*args) {
        tool_id = (uint32_t)gc_state.tool->tool_id;
        if(tool_id == 0) {
            report_message("TCADD: no tool selected and no argument given", Message_Warning);
            return Status_BadNumberFormat;
        }
    } else {
        if(*args != 'T' && *args != 't') {
            report_message("TCADD: usage is $TCADD [Tn] [;name]", Message_Warning);
            return Status_BadNumberFormat;
        }
        uint_fast8_t cc = 1;
        status_code_t parse_status = read_uint(args, &cc, &tool_id);
        if(parse_status != Status_OK) {
            report_message("TCADD: invalid tool number", Message_Warning);
            return parse_status;
        }
        while(args[cc] == ' ' || args[cc] == '\t') cc++;
        if(args[cc] == ';')
            name = &args[cc + 1];
    }

    // ── Tool present check ───────────────────────────────────────────────────
    if(atc.flags.tool_present_active) {
        read_atc_ports();
        if(!atc_status.toolpresent_status) {
            report_message("TCADD: no tool detected in spindle", Message_Warning);
            return Status_GcodeValueOutOfRange;
        }
    }

    // ── Confirm spindle is off before moving ─────────────────────────────────
    spindle_ptrs_t *spindle = spindle_get(0);
    if(spindle && spindle->get_state && spindle->get_state(spindle).on) {
        report_message("TCADD: spindle must be off", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    // ── Assign pocket in tooltable ───────────────────────────────────────────
    pocket_id_t assigned_pocket = 0;
    carousel_op_result_t result = tooltable_carousel_add((tool_id_t)tool_id, atc.number_of_pockets, name, &assigned_pocket);

    switch(result) {
        case CarouselOp_OK:
            break;
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

    // ── Deposit tool into assigned pocket via TCRETURN macro ─────────────────
    ngc_param_set(4900, (float)tool_id);
    ngc_param_set(4901, (float)assigned_pocket);

    standalone_op      = STANDALONE_TCADD;
    standalone_tool_id = (tool_id_t)tool_id;
    standalone_pocket  = assigned_pocket;

    status_code_t motion_result = grbl.on_macro_execute(ATC_MACRO_ID_TCRETURN, (parameter_words_t){0}, 1);
    if(motion_result != Status_Handled) {
        // Macro file not found or other synchronous failure — roll back pocket assignment
        standalone_op = STANDALONE_NONE;
        tooltable_carousel_remove((tool_id_t)tool_id);
        report_message("TCADD: deposit motion failed — pocket assignment rolled back", Message_Warning);
        return motion_result == Status_OK ? Status_FileOpenFailed : motion_result;
    }

    return Status_Unhandled;
}

// $TCRETURN  — Return the current spindle tool to its carousel pocket.
//
// Checks that the current tool has a carousel pocket assigned, sets #4900 to
// the tool_id and 4901 to the pocket, runs ATC_MACRO_ID_TCRETURN, to physically deposit the tool, then clears
// the spindle state to T0.
//
// The machine must be homed and IDLE.  Spindle and coolant must be off.
// After this command the spindle is empty and gc_state.tool is T0.

static status_code_t carousel_return (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCRETURN: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if((sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) != (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) {
        report_message("TCRETURN: machine must be homed", Message_Warning);
        return Status_HomingRequired;
    }

    tool_id_t tool_id = gc_state.tool->tool_id;
    if(tool_id == 0) {
        report_message("TCRETURN: no tool in spindle", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    pocket_id_t pocket = get_carousel_pocket(tool_id);
    if(pocket < 1) {
        report_message("TCRETURN: current tool has no carousel pocket — use $TCADD first", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    // Confirm spindle is off before moving
    spindle_ptrs_t *spindle = spindle_get(0);
    if(spindle && spindle->get_state && spindle->get_state(spindle).on) {
        report_message("TCRETURN: spindle must be off", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    // Run ATC_MACRO_ID_TCRETURN to physically deposit the tool.
    // Completion handled in macro_exit() via standalone_op flag
    // State cleanup (T0, reporting) happens there, not here.
    ngc_param_set(4900, (float)tool_id);
    ngc_param_set(4901, (float)pocket);

    standalone_op      = STANDALONE_TCRETURN;
    status_code_t result = grbl.on_macro_execute(ATC_MACRO_ID_TCRETURN, (parameter_words_t){0}, 1);
    if(result != Status_Handled) {
        standalone_op      = STANDALONE_NONE;
        report_message("TCRETURN: return motion failed", Message_Warning);
        return result == Status_OK ? Status_FileOpenFailed : result;
    }

    return Status_Unhandled;
}


// $TCRM [Tn]  — Clear a carousel pocket assignment, moving the tool to P0.
// Usage:
//   $TCRM T3    clear tool 3's pocket assignment (moves it to P0)
//   $TCRM       use the tool currently in the spindle
//
// The tool must currently have a pocket assigned (P > 0).  P0 tools are
// rejected — this command only operates on tools that are in the carousel.
//
// This is a purely administrative operation — it updates the tooltable record
// only.  The operator is responsible for physically removing the tool from the
// carousel pocket before issuing this command.
//
// When called without arguments, uses gc_state.tool->tool_id.  If a tool-present
// sensor is configured, the tool must be detected in the spindle — confirming
// that gc_state.tool matches the physical tool.

static status_code_t carousel_remove (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCRM: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    uint32_t tool_id;

    if(!args || !*args) {
        // No argument — use current spindle tool
        tool_id = (uint32_t)gc_state.tool->tool_id;
        if(tool_id == 0) {
            report_message("TCRM: no tool selected and no argument given", Message_Warning);
            return Status_BadNumberFormat;
        }
        // If a tool-present sensor is configured, confirm the tool is actually
        // in the spindle — corroborating that gc_state.tool is accurate.
        if(atc.flags.tool_present_active) {
            read_atc_ports();
            if(!atc_status.toolpresent_status) {
                report_message("TCRM: no tool detected in spindle — specify tool number explicitly", Message_Warning);
                return Status_GcodeValueOutOfRange;
            }
        }
    } else {
        if(*args != 'T' && *args != 't') {
            report_message("TCRM: usage is $TCRM [Tn]", Message_Warning);
            return Status_BadNumberFormat;
        }
        uint_fast8_t cc = 1;
        status_code_t parse_status = read_uint(args, &cc, &tool_id);
        if(parse_status != Status_OK) {
            report_message("TCRM: invalid tool number", Message_Warning);
            return parse_status;
        }
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
}

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


#if TOOLTABLE_ENABLE == 2

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
FLASHMEM static void onToolSelect (tool_data_t *tool, bool next)
{
    next_tool = tool;

    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));

    if(tool_select)
        tool_select(tool, next);
}

#endif

static void atc_tc_complete (void)
{
    tc_state = TC_IDLE;
    change_completed();
    coolant_restore(gc_state.modal.coolant, settings.coolant.on_delay);
    spindle_t *spindle = gc_spindle_get(-1);
    spindle_restore(spindle->hal, spindle->state, spindle->rpm, settings.spindle.on_delay);
    if(on_tool_change)
        on_tool_change(atc_parser_state);
}

static status_code_t atc_tc_advance (void)
{
    status_code_t status = Status_Handled;

    switch(tc_state) {

        // ── Entry point ───────────────────────────────────────────────────
        case TC_START:
            if(tc_outgoing_pocket >= 1) {
                tc_state = TC_RETURN_OUTGOING;
                ngc_param_set(4900, (float)current_tool.tool_id);
                ngc_param_set(4901, (float)tc_outgoing_pocket);
                status = grbl.on_macro_execute(ATC_MACRO_ID_TCRETURN, (parameter_words_t){0}, 1);
            } else {
                tc_state = TC_HAND_RETURN;
                execute_posted = false;
                control_interrupt_callback = hal.control.interrupt_callback;
                hal.control.interrupt_callback = trap_control_cycle_start;
                enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(trap_stream_cycle_start);
                status = grbl.on_macro_execute(ATC_MACRO_ID_HANDRETURN, (parameter_words_t){0}, 1);
            }
            if(status != Status_Handled) {
                change_completed();
                tc_state = TC_IDLE;
                return status == Status_OK ? Status_FileOpenFailed : status;
            }
            break;

        // ── TCRETURN or HANDRETURN complete; fetch incoming ───────────────
        case TC_RETURN_OUTGOING:
        case TC_HAND_RETURN: {
            bool trap_installed = (tc_state == TC_HAND_RETURN);
            // Outgoing tool is now physically back in the carousel — restore its pocket
            if(tc_outgoing_pocket >= 1) {
                pocket_id_t restored_pocket;
                tooltable_carousel_add(tc_outgoing_tool_id, atc.number_of_pockets, NULL, &restored_pocket);
            }
            if(tc_incoming_pocket >= 1) {
                // Remove incoming tool from tooltable — pocket is now physically empty
                tooltable_carousel_remove(next_tool->tool_id);
                last_fetched_pocket = -1;  // outgoing tool returned, no longer tracked
                tc_state = TC_FETCH_INCOMING;
                ngc_param_set(4900, (float)next_tool->tool_id);
                ngc_param_set(4901, (float)tc_incoming_pocket);
                status = grbl.on_macro_execute(ATC_MACRO_ID_TCFETCH, (parameter_words_t){0}, 1);
                if(status != Status_Handled) {
                    // Roll back — tool didn't move, restore its pocket
                    tooltable_carousel_add(next_tool->tool_id, atc.number_of_pockets, NULL, &tc_incoming_pocket);
                    change_completed();
                    tc_state = TC_IDLE;
                    return status == Status_OK ? Status_FileOpenFailed : status;
                }
            } else {
                last_fetched_pocket = -1;  // outgoing tool returned, no longer tracked
                tc_state = TC_HAND_FETCH;
                if(!trap_installed) {
                    execute_posted = false;
                    control_interrupt_callback = hal.control.interrupt_callback;
                    hal.control.interrupt_callback = trap_control_cycle_start;
                    enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(trap_stream_cycle_start);
                }
                status = grbl.on_macro_execute(ATC_MACRO_ID_HANDFETCH, (parameter_words_t){0}, 1);
                if(status != Status_Handled) {
                    change_completed();
                    tc_state = TC_IDLE;
                    return status == Status_OK ? Status_FileOpenFailed : status;
                }
            }
            break;
        }

        // ── TCFETCH complete → record pocket and mark empty, sequence done ─
        case TC_FETCH_INCOMING:
            last_fetched_pocket = tc_incoming_pocket;  // remember where this tool came from
            atc_tc_complete();
            break;

        // ── HANDFETCH complete (cycle-start trap fired) ───────────────────
        case TC_HAND_FETCH:
            last_fetched_pocket = -1;   // hand-loaded — no carousel pocket
            if(next_tool->tool_id == 0 || next_tool->offset.z != 0.0f) {
                atc_tc_complete();
            } else {
                tc_state = TC_MEASURE;
                status = grbl.on_macro_execute(ATC_MACRO_ID_MEASURE, (parameter_words_t){0}, 1);
                if(status != Status_Handled) {
                    change_completed();
                    tc_state = TC_IDLE;
                    return status == Status_OK ? Status_FileOpenFailed : status;
                }
            }
            break;

        // ── MEASURE complete → sequence done ─────────────────────────────
        case TC_MEASURE:
            atc_tc_complete();
            break;

        default:
            tc_state = TC_IDLE;
            break;
    }

    return Status_Unhandled;
}

static status_code_t tool_change (parser_state_t *parser_state)
{
    atc_parser_state = parser_state;

    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool_id == next_tool->tool_id)
        return Status_OK;

#if TOOLTABLE_ENABLE == 2

    tool_table_entry_t *incoming_entry = grbl.tool_table.get_tool(next_tool->tool_id);

    if(!incoming_entry || !incoming_entry->data ||
       (incoming_entry->data->tool_id == 0 && parser_state->tool_pending != 0))
        return on_tool_change ? on_tool_change(parser_state) : Status_OK;

    tc_incoming_pocket = (pocket_id_t)incoming_entry->pocket;
    tc_outgoing_pocket = last_fetched_pocket;   // -1 if hand-loaded or after power cycle
    tc_outgoing_tool_id = current_tool.tool_id;  // save before anything changes

    if(next_tool->tool_id != 0)
        tooltable_register_tool(next_tool->tool_id, NULL);

    spindle_all_off(false);
    hal.coolant.set_state((coolant_state_t){0});

    tc_state = TC_START;
    return atc_tc_advance();

#else
    next_tool = NULL;
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);
    return Status_Unhandled;
#endif
}

static status_code_t carousel_measure (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCMEASURE: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    report_message("ATC: measuring tool length", Message_Info);

    status_code_t result = true;

    if(result != Status_OK)
        report_message("TCMEASURE: probe failed", Message_Warning);

    return result;
}

static status_code_t carousel_remeasure (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TCREMEASURE: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    report_message("ATC: clearing stored offset and re-measuring tool length", Message_Info);

    status_code_t result = true;

    if(result != Status_OK)
        report_message("TCREMEASURE: probe failed", Message_Warning);

    return result;
}

#endif // TOOLTABLE_ENABLE == 2

// ---------------------------------------------------------------------------
// Command table
// ---------------------------------------------------------------------------

const sys_command_t atc_command_list[] = {
    {"DRBO",      drawbar_open,       { .noargs = On  }, { .str = "Open the drawbar" }},
    {"DRBC",      drawbar_close,      { .noargs = On  }, { .str = "Close the drawbar" }},
#if TOOLTABLE_ENABLE == 2
    {"TCADD",     carousel_add,       { .noargs = Off }, { .str = "Deposit current spindle tool into next free carousel pocket and register it: $TCADD [Tn] [,name]" }},
    {"TCRETURN",  carousel_return,    { .noargs = On  }, { .str = "Return current spindle tool to its carousel pocket and clear spindle to T0" }},
    {"TCRM",      carousel_remove,    { .noargs = Off }, { .str = "Clear tool's carousel pocket in tooltable (does not move the tool): $TCRM [Tn]" }},
    {"TCMEASURE",   carousel_measure,   { .noargs = On  }, { .str = "Measure current tool length against G59.3 toolsetter (skips if already measured)" }},
    {"TCREMEASURE", carousel_remeasure, { .noargs = On  }, { .str = "Clear stored offset and re-measure current tool length against G59.3 toolsetter" }},
#endif
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
            zero_count = 0;  // Reset counter after activation
        }
    }
    // Check for transition to inactive state
    else if (((prev_val == 1) && (val == 1) && (latch == 1)) || 
             (zero_count >= ZERO_THRESHOLD)) {  // Added condition for 10 consecutive zeros
        if (one_count >= 1 || zero_count >= ZERO_THRESHOLD) {  // Modified condition
            latch = 0;
            grbl.enqueue_gcode("$DRBC");
            one_count = 0;  // Reset counter after activation
            zero_count = 0;  // Also reset zero counter
        }
    }
    // Reset counters if state is inconsistent
    else {
        zero_count = 0;
        one_count = 0;
    }

    //if the spindle is running and the drawbar or tool is sensed open/not present raise an error and stop.

    //polling_ms = ms;
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
            || ((atc_status.drawbar_control == 1)    && (atc.flags.drawbar_control_active))) //drawbar is commanded to be open
            {
            state.value = 0; //ensure spindle is off
            atc_status.airseal_control=0;
            hal.port.digital_out(active_ports.air_seal, atc_status.airseal_control);//ensure air seal is off
            
            grbl.enqueue_realtime_command(CMD_STOP);
            if((atc_status.drawbar_status == 0) && (atc.flags.drawbar_status_active))
                report_message("ATC Malfunction: spindle start blocked, drawbar sensor reads open", Message_Warning);
            if((atc_status.toolpresent_status == 0) && (atc.flags.tool_present_active))
                report_message("ATC Malfunction: spindle start blocked, tool present sensor reads no tool", Message_Warning);
            if((atc_status.drawbar_control == 1) && (atc.flags.drawbar_control_active))
                report_message("ATC Malfunction: spindle start blocked, drawbar is commanded open", Message_Warning);
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
    { 961, Group_Toolchange, "ATC Flags", NULL, Format_Bitfield, "User Input Enabled, Tool Detect Enabled, Drawbar Status Enabled, Drawbar Control Enabled, Air Seal Control Enabled, Taper Clear Enabled, Toolsetter Clear Enabled", NULL, NULL, Setting_NonCore, &atc.flags, NULL, NULL },
#if TOOLTABLE_ENABLE == 2
    { 962, Group_Toolchange, "ATC Number of Pockets", "pockets", Format_Int16, "##0", "1", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
#endif
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
#if TOOLTABLE_ENABLE == 2    
    { 962, "Number of physical pockets in the carousel (max 9999). $TCADD will refuse to assign a pocket number beyond this limit." },
#endif    
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

#if TOOLTABLE_ENABLE == 2
    atc.number_of_pockets = 12; // default carousel size
#endif
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
#if TOOLTABLE_ENABLE == 2
    if (settings.tool_change.mode != ToolChange_Automatic)
        return;  

    settings.macro_atc_flags.random_toolchanger = 1;

    grbl.tool_table.n_tools = atc.number_of_pockets;
    
    
    if(hal.tool.change != tool_change) {
        on_tool_change = hal.tool.change;
        hal.tool.change = tool_change;
        //grbl.on_toolchange_ack = on_toolchange_ack;
    }


    tool_select = hal.tool.select;
    hal.tool.select = onToolSelect;
#endif
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
    next_tool = NULL;
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
    next_tool = NULL;
#if TOOLTABLE_ENABLE == 2
    change_completed();
#endif
    driver_reset();
}

#if TOOLTABLE_ENABLE == 2
static atc_status_t atc_get_state (void)
{
    // If macros.c has claimed hal.tool.change via tc.macro, report Online
    // so that tc_init() does not overwrite it.  If tc.macro is not present,
    // our own tool_change() is in place — also report Online to block tc_init().
    return ATC_Online;
}
#endif

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
#if TOOLTABLE_ENABLE == 2
    hal.tool.atc_get_state = atc_get_state;
    on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = macro_exit;
#endif

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
