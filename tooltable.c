/*

  tooltable.c - file based tooltable, LinuxCNC format

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

#include "driver.h"

#if TOOLTABLE_ENABLE == 2

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#include "grbl/vfs.h"
#include "grbl/strutils.h"
#include "grbl/gcode.h"
#include "grbl/stream.h"
#include "grbl/core_handlers.h"
#include "grbl/state_machine.h"

#include "tooltable.h"

static bool loaded = false;
static uint32_t n_pockets = 1;
static tool_pocket_t pocket0, *pockets = &pocket0;
static tool_id_t current_tool = 0;
static char filename[] = "/linuxcnc/tooltable.tbl";

static tool_select_ptr tool_select;
static on_tool_changed_ptr on_tool_changed;
static on_vfs_mount_ptr on_vfs_mount;
static on_report_options_ptr on_report_options;

// M6 tool-change state — set by the ATC plugin via tooltable_set_m6_prev()
// before enqueueing its macro, consumed in onToolChanged() after completion.
static m6_tool_origin_t  m6_prev_tool_origin = M6Origin_Unknown;
static pocket_id_t       m6_prev_tool_pocket  = -1;
static tool_id_t         m6_prev_tool_id      = -1; // tool being replaced

void tooltable_set_m6_prev (m6_tool_origin_t origin, pocket_id_t pocket)
{
    m6_prev_tool_origin = origin;
    m6_prev_tool_pocket = pocket;
}

static tool_pocket_t *get_pocket (tool_id_t tool_id)
{
    uint_fast16_t idx;
    tool_pocket_t *pocket = NULL;

    if(tool_id >= 0) for(idx = 0; idx < n_pockets; idx++) {
        if(pockets[idx].tool.tool_id == tool_id) {
            pocket = &pockets[idx];
            break;
        }
    }

    return pocket;
}

static tool_table_entry_t *getTool (tool_id_t tool_id)
{
    static tool_table_entry_t tool = {0};

    tool_pocket_t *pocket;
    if((pocket = get_pocket(tool_id)) && (!settings.macro_atc_flags.random_toolchanger || pocket->pocket_id != -1)) {
        tool.data = &pocket->tool;
        tool.pocket = pocket->pocket_id;
        tool.name = pocket->name;
    } else
        tool.data = NULL;

    return &tool;
}

static tool_table_entry_t *getToolByIdx (uint32_t idx)
{
    static tool_table_entry_t tool = {0};

    tool_pocket_t *pocket = idx < n_pockets ? &pockets[idx] : NULL;

    if(pocket && pocket->tool.tool_id) {
        tool.data = &pocket->tool;
        tool.pocket = pocket->pocket_id;
        tool.name = pocket->name;
    } else
        tool.data = NULL;

    return &tool;
}

// Write a single pocket entry to the open file.
// Tools with a carousel pocket use their real pocket number.
// Tools with no pocket (removed from carousel / in spindle) are written
// as P0 so their offsets and name survive a reload.  P0 is the LinuxCNC
// convention for "tool known but not currently in a pocket".
static void write_pocket_line (vfs_file_t *file, const tool_pocket_t *p)
{
    char buf[400], tmp[20];
    uint_fast16_t axis;

    // P0 = no carousel pocket; real pocket number otherwise
    uint16_t file_pocket = (p->pocket_id >= 1) ? (uint16_t)p->pocket_id : 0;

    sprintf(buf, "P%u T%u ", file_pocket, (uint16_t)p->tool.tool_id);

    for(axis = 0; axis < N_AXIS; axis++) {
        if(p->tool.offset.values[axis] != 0.0f) {
            sprintf(tmp, "%s%-.3f ", axis_letter[axis], p->tool.offset.values[axis]);
            strcat(buf, tmp);
        }
    }
    if(p->tool.radius != 0.0f) {
        sprintf(tmp, "D%-.3f ", p->tool.radius * 2.0f);
        strcat(buf, tmp);
    }
    if(*p->name)
        sprintf(strchr(buf, '\0'), "; %s", p->name);

    strcat(buf, "\n");
    vfs_write(buf, strlen(buf), 1, file);
}

static bool writeTools (tool_data_t *tool_data)
{
    bool ok;
    tool_pocket_t *pocket;
    vfs_file_t *file;

    // Sync in-memory copy if caller passed a different buffer
    if((ok = !!(pocket = get_pocket(tool_data->tool_id)))) {
        if(&pocket->tool != tool_data)
            memcpy(&pocket->tool, tool_data, sizeof(tool_data_t));
    }

    if(ok && (ok = !!(file = vfs_open(filename, "w")))) {

        uint_fast16_t idx;

        for(idx = 1; idx < n_pockets; idx++) {
            // Write every known tool (tool_id >= 0), regardless of pocket status.
            // Tools in a carousel pocket get their real P number; tools with no
            // pocket (removed or in spindle) are written as P0 so offsets persist.
            if(pockets[idx].tool.tool_id >= 0)
                write_pocket_line(file, &pockets[idx]);
        }

        vfs_close(file);
    }

    return ok;
}

static bool clearTools (void)
{
    uint_fast8_t idx;

    for(idx = 0; idx < n_pockets; idx++) {
        pockets[idx].tool.radius = 0.0f;
        memset(&pockets[idx].tool.offset, 0, sizeof(coord_data_t));
        if(!loaded) {
            pockets[idx].pocket_id = -1;
            pockets[idx].tool.tool_id = idx == 0 ? 0 : -1;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Carousel management - find the lowest free pocket number (>= 1)
// ---------------------------------------------------------------------------

static pocket_id_t find_free_pocket (void)
{
    uint_fast16_t idx;
    pocket_id_t candidate;

    // Try every candidate pocket id starting at 1
    for(candidate = 1; candidate < (pocket_id_t)n_pockets + 1; candidate++) {
        bool in_use = false;
        for(idx = 0; idx < n_pockets; idx++) {
            if(pockets[idx].pocket_id == candidate) {
                in_use = true;
                break;
            }
        }
        if(!in_use)
            return candidate;
    }

    return -1; // no free pocket found
}

// Find a slot in the pockets array that is logically empty (tool_id == -1).
// Returns pointer to that slot, or NULL if the array is full.
static tool_pocket_t *find_empty_slot (void)
{
    uint_fast16_t idx;

    // Skip slot 0 — reserved for spindle / tool-in-hand
    for(idx = 1; idx < n_pockets; idx++) {
        if(pockets[idx].tool.tool_id < 0)
            return &pockets[idx];
    }

    return NULL; // no empty slot; would need realloc
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id)
{
    if(!loaded)
        return CarouselOp_TableNotLoaded;

    // Check if this tool is already known to the table
    tool_pocket_t *existing = get_pocket(tool_id);

    if(existing && existing->pocket_id >= 0)
        return CarouselOp_ToolAlreadyInPocket; // already in carousel

    // Find a free carousel pocket number
    pocket_id_t free_pocket = find_free_pocket();
    if(free_pocket < 0)
        return CarouselOp_NoPocketAvailable;

    tool_pocket_t *slot;

    if(existing) {
        // Tool is known (has offsets) but has no pocket — just assign one.
        // Offsets, name, radius are preserved as-is.
        slot = existing;
    } else {
        // Brand-new tool — find or grow a slot for it
        slot = find_empty_slot();

        if(slot == NULL) {
            // Array is full — grow by one
            uint32_t new_count = n_pockets + 1;
            tool_pocket_t *new_pockets;

            if(pockets == &pocket0) {
                new_pockets = malloc(new_count * sizeof(tool_pocket_t));
                if(new_pockets)
                    memcpy(new_pockets, &pocket0, sizeof(tool_pocket_t));
            } else {
                new_pockets = realloc(pockets, new_count * sizeof(tool_pocket_t));
            }

            if(!new_pockets)
                return CarouselOp_NoPocketAvailable;

            pockets    = new_pockets;
            n_pockets  = new_count;
            slot       = &pockets[n_pockets - 1];
        }

        // Initialise the new slot with zeroed offsets
        memset(slot, 0, sizeof(tool_pocket_t));
        slot->tool.tool_id = tool_id;
        slot->pocket_id    = -1; // will be set below
    }

    slot->pocket_id = free_pocket;

    grbl.tool_table.n_tools = n_pockets;

    if(!writeTools(&slot->tool))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id)
{
    if(!loaded)
        return CarouselOp_TableNotLoaded;

    tool_pocket_t *pocket = get_pocket(tool_id);

    if(!pocket || pocket->pocket_id < 0)
        return CarouselOp_ToolNotFound;

    // Only clear the carousel pocket assignment.
    // Offsets, radius, and name are intentionally preserved so they survive
    // the tool being out of the carousel and can be reused on $TCADD.
    pocket->pocket_id = -1;

    // writeTools will write this entry as P0, keeping offsets in the file.
    if(!writeTools(&pocket->tool))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// (rest of file unchanged from original)
// ---------------------------------------------------------------------------

static status_code_t load_tools (sys_state_t state, char *args)
{
    char c, buf[300] = "";
    uint_fast8_t n_tools = 0, idx = 0, entry = 0, cc;
    vfs_file_t *file;
    status_code_t status = Status_GcodeUnusedWords;

    args = filename;

    if((file = vfs_open(args, "r"))) {

        while(vfs_read(&c, 1, 1, file) == 1) {
            if(c == ASCII_CR || c == ASCII_LF) {
                if(*buf) {
                    *buf = '\0';
                    n_tools++;
                }
            } else
                *buf = c;
        }

        if(n_tools && (n_tools + 1 > n_pockets || pockets == &pocket0)) {
            if(pockets != &pocket0)
                free(pockets);
            if((pockets = malloc((n_tools + 1) * sizeof(tool_pocket_t))))
                n_pockets = n_tools + 1;
            else
                n_pockets = 1;
        }

        n_tools = 0;

        if(n_pockets > 1) {

            vfs_seek(file, 0);
            memset(pockets, 0, n_pockets * sizeof(tool_pocket_t));

            while(vfs_read(&c, 1, 1, file) == 1) {

                if(c == ASCII_CR || c == ASCII_LF) {

                    buf[idx] = '\0';

                    if(!(*buf == '\0' || *buf == ';')) {

                       char *param = strtok(buf, " ");
                       tool_pocket_t pocket = { .pocket_id = -1, .tool.tool_id = -1 };

                       status = Status_OK;

                       while(param && status == Status_OK) {

                           cc = 1;

                           switch(CAPS(*param)) {

                               case 'T':
                                   {
                                       uint32_t tool_id;
                                       if((status = read_uint(param, &cc, &tool_id)) == Status_OK)
                                           pocket.tool.tool_id = (tool_id_t)tool_id;
                                   }
                                   break;

                               case 'P':
                                   {
                                       uint32_t pocket_id;
                                       if((status = read_uint(param, &cc, &pocket_id)) == Status_OK)
                                           pocket.pocket_id = (pocket_id_t)pocket_id;
                                   }
                                   break;

                               case 'X':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[X_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;

                               case 'Y':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[Y_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;

                               case 'Z':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[Z_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#ifdef A_AXIS
                               case 'A':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[A_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#endif
#ifdef B_AXIS
                               case 'B':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[B_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#endif
#ifdef C_AXIS
                               case 'C':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[C_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#endif
#ifdef U_AXIS
                               case 'U':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[U_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#endif
#ifdef V_AXIS
                               case 'V':
                                   if(!read_float(param, &cc, &pocket.tool.offset.values[V_AXIS]))
                                       status = Status_GcodeValueOutOfRange;
                                   break;
#endif
                               case 'D':
                                   if(!read_float(param, &cc, &pocket.tool.radius))
                                       status = Status_GcodeValueOutOfRange;
                                   else
                                       pocket.tool.radius /= 2.0f;
                                   break;

                               case ';':
                                   strncpy(pocket.name, param + 1, sizeof(pocket.name) - 1);
                                   while((param = strtok(NULL, " "))) {
                                       if(strlen(pocket.name) + strlen(param) <= sizeof(pocket.name) - 2) {
                                           if(*pocket.name)
                                               strcat(pocket.name, " ");
                                           strcat(pocket.name, param);
                                       } else
                                           continue;

                                   }
                                   while((param = strchr(pocket.name, '|'))) // make safe for reporting
                                       *param = '%';
                                   break;
                           }

                           param = strtok(NULL, " ");
                       }

                       // Accept P0 as "tool known but not in a carousel pocket".
                       // pocket_id is stored as -1 internally; P0 is only a file convention.
                       if(pocket.pocket_id == 0)
                           pocket.pocket_id = -1;

                       if(status == Status_OK && pocket.tool.tool_id >= 0) {

                           if(pocket.pocket_id >= 1 && settings.macro_atc_flags.random_toolchanger) {
                               entry = pocket.pocket_id;
                           } else
                               entry++;

                           if(entry < n_pockets) {
                               n_tools++;
                               memcpy(&pockets[entry], &pocket, sizeof(tool_pocket_t));
                           }
                       }
                    }

                    idx = 0;

                } else if(idx < sizeof(buf))
                    buf[idx++] = c;
            }
        } else {
            // n_tools > 0: not enough memory for tool table - raise alarm?
            pockets = &pocket0;
        }

        loaded = n_tools > 0;

        vfs_close(file);
    }

    grbl.tool_table.n_tools = loaded ? n_pockets : 0;

    return status == Status_OK ? Status_OK : Status_FileReadError;
}

static void loadTools (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    load_tools(state_get(), filename);

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

static void onToolSelect (tool_data_t *tool, bool next)
{
    if(!next)
        current_tool = tool->tool_id;

    if(tool_select)
        tool_select(tool, next);
}

static void onToolChanged (tool_data_t *tool)
{
    if(settings.macro_atc_flags.random_toolchanger) {

        // ── Incoming tool: remove it from the carousel ─────────────────────
        // The tool is now in the spindle; its carousel pocket is free.
        // Offsets and name are preserved (pocket_id cleared to -1 only).
        tool_pocket_t *picked_up = get_pocket(tool->tool_id);
        if(picked_up && picked_up->pocket_id >= 0)
            picked_up->pocket_id = -1;

        // ── Outgoing tool: return it to the carousel if it came from one ───
        // The ATC plugin calls tooltable_set_m6_prev() before enqueueing its
        // macro; we consume those values here.
        // When $TCADD / $TCRM are used manually, origin stays M6Origin_Unknown
        // and we skip this block entirely.
        tool_pocket_t *returned = NULL;
        if(m6_prev_tool_origin == M6Origin_Carousel && m6_prev_tool_pocket >= 1) {
            returned = get_pocket(current_tool); // current_tool = outgoing tool id
            if(returned)
                returned->pocket_id = m6_prev_tool_pocket;
        }

        // Reset M6 state for next change
        m6_prev_tool_origin = M6Origin_Unknown;
        m6_prev_tool_pocket = -1;

        // Single write covers both updates
        tool_data_t *anchor = picked_up ? &picked_up->tool
                            : returned  ? &returned->tool
                            : NULL;
        if(anchor)
            writeTools(anchor);
    }

    current_tool = tool->tool_id;

    if(on_tool_changed)
        on_tool_changed(tool);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Tool table", "0.03");
}

void tooltable_init (void)
{
    static const sys_command_t tt_command_list[] = {
        { "TTLOAD", load_tools, {}, { .str = "(re)load tool table" } }
     };

    static sys_commands_t tt_commands = {
        .n_commands = sizeof(tt_command_list) / sizeof(sys_command_t),
        .commands = tt_command_list
    };

    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = loadTools;

    tool_select = hal.tool.select;
    hal.tool.select = onToolSelect;

    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = onToolChanged;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    grbl.tool_table.n_tools = 1;
    grbl.tool_table.get_tool = getTool;
    grbl.tool_table.set_tool = writeTools;
    grbl.tool_table.get_tool_by_idx = getToolByIdx;
    grbl.tool_table.clear = clearTools;

    system_register_commands(&tt_commands);

    clearTools();

#if SDCARD_ENABLE
    sdcard_early_mount();
#endif
}

#endif // TOOLTABLE_ENABLE
