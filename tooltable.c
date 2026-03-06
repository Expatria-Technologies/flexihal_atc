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

// ---------------------------------------------------------------------------
// Lightweight index entry - kept in RAM for fast enumeration.
// Only tool_id and pocket_id are stored here; all other data is read from
// the file on demand.  At 8 bytes per entry a 100-tool table costs 800 bytes.
// ---------------------------------------------------------------------------
typedef struct {
    tool_id_t   tool_id;    // -1 = empty slot
    pocket_id_t pocket_id;  // -1 = P0 (not in carousel), >= 1 = carousel pocket
    tool_data_t tool;       // cached from file — stable pointer for getTool()
    char        name[sizeof(((tool_pocket_t*)0)->name)];
} tool_index_entry_t;

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static bool              fs_available = false;  // VFS has been mounted
static uint16_t          n_tools      = 0;      // number of valid tools in index
static uint16_t          index_cap    = 0;      // allocated capacity of tt_index
static tool_index_entry_t *tt_index   = NULL;   // lightweight RAM index
static tool_id_t         current_tool = 0;      // tool currently in spindle
static char              filename[]   = "/linuxcnc/tooltable.tbl";

// M6 tool-change state - set by ATC plugin, consumed in onToolChanged().
// Holds the carousel pocket the outgoing tool came from, or -1 if it was hand-loaded.
static pocket_id_t       m6_prev_tool_pocket = -1;

static tool_select_ptr       tool_select;
static on_tool_changed_ptr   on_tool_changed;
static on_vfs_mount_ptr      on_vfs_mount;
static on_report_options_ptr on_report_options;

// ---------------------------------------------------------------------------
// Public: called by ATC plugin before enqueueing a macro
// ---------------------------------------------------------------------------
void tooltable_set_m6_prev (pocket_id_t pocket)
{
    m6_prev_tool_pocket = pocket;
}

// ---------------------------------------------------------------------------
// Index management
// ---------------------------------------------------------------------------

static bool index_grow (void)
{
    uint16_t new_cap = index_cap + 8;
    tool_index_entry_t *p = realloc(tt_index, new_cap * sizeof(tool_index_entry_t));
    if(!p)
        return false;
    tt_index     = p;
    index_cap = new_cap;
    return true;
}

static tool_index_entry_t *index_find (tool_id_t tool_id)
{
    for(uint16_t i = 0; i < n_tools; i++) {
        if(tt_index[i].tool_id == tool_id)
            return &tt_index[i];
    }
    return NULL;
}

static bool index_upsert_full (const tool_pocket_t *p)
{
    tool_index_entry_t *e = index_find(p->tool.tool_id);
    if(!e) {
        if(n_tools >= index_cap && !index_grow())
            return false;
        e = &tt_index[n_tools++];
    }
    e->tool_id   = p->tool.tool_id;
    e->pocket_id = p->pocket_id;
    memcpy(&e->tool, &p->tool, sizeof(tool_data_t));
    strncpy(e->name, p->name, sizeof(e->name) - 1);
    e->name[sizeof(e->name) - 1] = '\0';
    return true;
}

static void index_clear (void)
{
    n_tools = 0;
}

static uint16_t index_count_in_carousel (void)
{
    uint16_t count = 0;
    for(uint16_t i = 0; i < n_tools; i++) {
        if(tt_index[i].pocket_id >= 1)
            count++;
    }
    return count;
}

static pocket_id_t index_find_free_pocket (void)
{
    for(pocket_id_t candidate = 1; candidate <= (pocket_id_t)(n_tools + 1); candidate++) {
        bool in_use = false;
        for(uint16_t i = 0; i < n_tools; i++) {
            if(tt_index[i].pocket_id == candidate) {
                in_use = true;
                break;
            }
        }
        if(!in_use)
            return candidate;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Read one line from the file into buf. Returns true if a line was read.
// ---------------------------------------------------------------------------
static bool read_line (vfs_file_t *file, char *buf, size_t len)
{
    size_t idx = 0;
    char c;

    while(vfs_read(&c, 1, 1, file) == 1) {
        if(c == '\n') {
            buf[idx] = '\0';
            if(idx > 0 && buf[idx-1] == '\r')
                buf[idx-1] = '\0';
            return true;
        }
        if(idx < len - 1)
            buf[idx++] = c;
    }

    if(idx > 0) {
        buf[idx] = '\0';
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Parse one line into a tool_pocket_t.
// Returns true if a valid tool entry was parsed.
// ---------------------------------------------------------------------------
static bool parse_line (char *line, tool_pocket_t *out)
{
    if(!line || !*line || *line == ';' || *line == '\r' || *line == '\n')
        return false;

    memset(out, 0, sizeof(tool_pocket_t));
    out->pocket_id    = -1;
    out->tool.tool_id = -1;

    char *param = strtok(line, " \t");
    status_code_t status = Status_OK;

    while(param && status == Status_OK) {

        uint_fast8_t cc = 1;

        switch(CAPS(*param)) {

            case 'T':
            {
                uint32_t tool_id;
                if((status = read_uint(param, &cc, &tool_id)) == Status_OK)
                    out->tool.tool_id = (tool_id_t)tool_id;
            }
            break;

            case 'P':
            {
                uint32_t pocket_id;
                if((status = read_uint(param, &cc, &pocket_id)) == Status_OK)
                    out->pocket_id = (pocket_id == 0) ? -1 : (pocket_id_t)pocket_id;
            }
            break;

            case 'X':
                if(!read_float(param, &cc, &out->tool.offset.values[X_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;

            case 'Y':
                if(!read_float(param, &cc, &out->tool.offset.values[Y_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;

            case 'Z':
                if(!read_float(param, &cc, &out->tool.offset.values[Z_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#ifdef A_AXIS
            case 'A':
                if(!read_float(param, &cc, &out->tool.offset.values[A_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef B_AXIS
            case 'B':
                if(!read_float(param, &cc, &out->tool.offset.values[B_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef C_AXIS
            case 'C':
                if(!read_float(param, &cc, &out->tool.offset.values[C_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef U_AXIS
            case 'U':
                if(!read_float(param, &cc, &out->tool.offset.values[U_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef V_AXIS
            case 'V':
                if(!read_float(param, &cc, &out->tool.offset.values[V_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
            case 'D':
                if(!read_float(param, &cc, &out->tool.radius))
                    status = Status_GcodeValueOutOfRange;
                else
                    out->tool.radius /= 2.0f;
                break;

            case ';':
                strncpy(out->name, param + 1, sizeof(out->name) - 1);
                while((param = strtok(NULL, " \t"))) {
                    if(strlen(out->name) + strlen(param) + 1 <= sizeof(out->name) - 1) {
                        if(*out->name)
                            strcat(out->name, " ");
                        strcat(out->name, param);
                    }
                }
                while((param = strchr(out->name, '|')))
                    *param = '%';
                param = NULL;
                break;
        }

        if(param)
            param = strtok(NULL, " \t");
    }

    return status == Status_OK && out->tool.tool_id >= 0;
}

// ---------------------------------------------------------------------------
// Rebuild the RAM tt_index by scanning the file once.
// ---------------------------------------------------------------------------
static bool rebuild_index (void)
{
    vfs_file_t *file = vfs_open(filename, "r");
    if(!file)
        return false;

    index_clear();

    char line[300];
    tool_pocket_t entry;

    while(read_line(file, line, sizeof(line))) {
        if(parse_line(line, &entry) && entry.pocket_id >= 1)
            index_upsert_full(&entry);
    }

    vfs_close(file);
    grbl.tool_table.n_tools = n_tools;
    return true;
}

// ---------------------------------------------------------------------------
// Write one pocket entry as a line to an open file.
// ---------------------------------------------------------------------------
static void write_pocket_line (vfs_file_t *file, const tool_pocket_t *p)
{
    char buf[400], tmp[24];

    uint16_t file_pocket = (p->pocket_id >= 1) ? (uint16_t)p->pocket_id : 0;
    sprintf(buf, "P%u T%u", file_pocket, (uint16_t)p->tool.tool_id);

    for(uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
        if(fabsf(p->tool.offset.values[axis]) > 0.0001f) {
            sprintf(tmp, " %s%.3f", axis_letter[axis], p->tool.offset.values[axis]);
            strcat(buf, tmp);
        }
    }

    if(p->tool.radius != 0.0f) {
        sprintf(tmp, " D%.3f", p->tool.radius * 2.0f);
        strcat(buf, tmp);
    }

    if(*p->name) {
        strcat(buf, " ;");
        strcat(buf, p->name);
    }

    strcat(buf, "\n");
    vfs_write(buf, strlen(buf), 1, file);
}

// ---------------------------------------------------------------------------
// Rewrite the entire file, optionally overriding pocket_ids for specific
// tools.  Uses a temp file to avoid any heap allocation — one line at a
// time is read from the source, modified if it matches an override, and
// written to the temp file.  The temp file is then renamed over the original.
// Stack usage: one line buffer (300 bytes) + one tool_pocket_t (~200 bytes).
// ---------------------------------------------------------------------------
#define MAX_OVERRIDES 2

typedef struct {
    tool_id_t   tool_id;
    pocket_id_t new_pocket_id;
    char        name[sizeof(((tool_pocket_t*)0)->name)];  // optional: empty = no change
    bool        delete_entry;                              // if true, omit this tool from rewritten file
} pocket_override_t;

static char filename_tmp[] = "/linuxcnc/tooltable.tmp";

static bool rewrite_file (const pocket_override_t *overrides, uint8_t n_overrides)
{
    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {

        if(!parse_line(line, &entry)) {
            // Blank or comment line — skip (do not carry forward comments
            // since the first-pass counter bug is gone and we never write them)
            continue;
        }

        // Check if this tool has a pocket_id or name override
        bool skip = false;
        for(uint8_t oi = 0; oi < n_overrides; oi++) {
            if(entry.tool.tool_id == overrides[oi].tool_id) {
                if(overrides[oi].delete_entry) {
                    skip = true;
                } else {
                    entry.pocket_id = overrides[oi].new_pocket_id;
                    if(overrides[oi].name[0] != '\0')
                        strncpy(entry.name, overrides[oi].name, sizeof(entry.name) - 1);
                }
                break;
            }
        }

        if(!skip)
            write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    // Atomically replace original with temp file
    if(vfs_rename(filename_tmp, filename) != 0) {
        // rename failed — try to clean up temp file
        vfs_unlink(filename_tmp);
        return false;
    }

    rebuild_index();
    return true;
}

// ---------------------------------------------------------------------------
// Append a brand-new tool entry to the end of the file.
// Avoids a full read-modify-write for the common $TCADD new-tool case.
// ---------------------------------------------------------------------------
static bool append_tool (const tool_pocket_t *p)
{
    vfs_file_t *file = vfs_open(filename, "a");
    if(!file)
        return false;

    write_pocket_line(file, p);
    vfs_close(file);

    index_upsert_full(p);
    grbl.tool_table.n_tools = n_tools;
    return true;
}

// ---------------------------------------------------------------------------
// grbl.tool_table.get_tool - scan file for tool_id, return full entry.
// The RAM tt_index is checked first so we never open the file for unknown tools.
// ---------------------------------------------------------------------------
// getTool returns pointers directly into the index entry, which is stable
// persistent storage — no static result buffer needed, no file open required.
// The index is always kept up to date with full tool data by rebuild_index()
// and index_upsert_full(), so this is safe across successive calls.
// File-scan result buffer — only used for P0 tools not in the index.
// Safe to be a single static because P0 tools are never enumerated in a loop
// by report.c (getToolByIdx only returns carousel tools by pocket number).
typedef struct {
    tool_table_entry_t entry;
    tool_data_t        tool;
    char               name[sizeof(((tool_pocket_t*)0)->name)];
} tool_scan_result_t;

static tool_table_entry_t *getTool (tool_id_t tool_id)
{
    static tool_table_entry_t  empty  = { .data = NULL };
    static tool_scan_result_t  scanned = {0};

    tool_index_entry_t *ie = index_find(tool_id);

    if(ie) {
        // Carousel tool — return stable pointer into index entry
        static tool_table_entry_t result;
        result.data   = &ie->tool;
        result.pocket = ie->pocket_id;
        result.name   = ie->name;
        return &result;
    }

    // Not in index — P0 tool or unknown. Scan file.
    if(!fs_available)
        return &empty;

    vfs_file_t *file = vfs_open(filename, "r");
    if(!file)
        return &empty;

    char line[300];
    tool_pocket_t entry;
    bool found = false;

    while(read_line(file, line, sizeof(line))) {
        if(parse_line(line, &entry) && entry.tool.tool_id == tool_id) {
            found = true;
            break;
        }
    }
    vfs_close(file);

    if(!found)
        return &empty;

    memcpy(&scanned.tool, &entry.tool, sizeof(tool_data_t));
    strncpy(scanned.name, entry.name, sizeof(scanned.name) - 1);
    scanned.name[sizeof(scanned.name) - 1] = '\0';
    scanned.entry.data   = &scanned.tool;
    scanned.entry.pocket = entry.pocket_id;  // will be -1
    scanned.entry.name   = scanned.name;
    return &scanned.entry;
}

// ---------------------------------------------------------------------------
// grbl.tool_table.get_tool_by_idx - same implementation as getTool.
// ---------------------------------------------------------------------------
static tool_table_entry_t *getToolByIdx (uint32_t idx)
{
    // idx is a pocket number (1-based). Scan the index for the tool in that pocket.
    for(uint16_t i = 0; i < n_tools; i++) {
        if(tt_index[i].pocket_id == (pocket_id_t)idx)
            return getTool(tt_index[i].tool_id);
    }

    static tool_table_entry_t empty = { .data = NULL };
    return &empty;
}

// ---------------------------------------------------------------------------
// grbl.tool_table.set_tool - update offsets for an existing tool in the file.
// Streams through the file line by line with no heap allocation.
// ---------------------------------------------------------------------------
static bool setTool (tool_data_t *tool_data)
{
    if(!tool_data || tool_data->tool_id < 0)
        return false;

    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {
        if(!parse_line(line, &entry))
            continue;
        // Replace offset data for the matching tool; preserve pocket and name
        if(entry.tool.tool_id == tool_data->tool_id)
            memcpy(&entry.tool, tool_data, sizeof(tool_data_t));
        write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    rebuild_index();
    return true;
}

// ---------------------------------------------------------------------------
// grbl.tool_table.clear - zero offsets for all tools (tools remain in table).
// Streams through the file line by line with no heap allocation.
// ---------------------------------------------------------------------------
static bool clearTools (void)
{
    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {
        if(!parse_line(line, &entry))
            continue;
        memset(&entry.tool.offset, 0, sizeof(coord_data_t));
        entry.tool.radius = 0.0f;
        write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    rebuild_index();
    return true;
}

// Return the name/comment for a tool from the RAM index.
// Returns NULL if the tool is not indexed or has no name.
const char *tooltable_get_name (tool_id_t tool_id)
{
    tool_index_entry_t *e = index_find(tool_id);
    if(e == NULL || e->name[0] == '\0')
        return NULL;
    return e->name;
}

// Register a tool in the tooltable at P0 (not in the carousel).
// If the tool already exists its name is updated if name is non-NULL and non-empty.
// Returns CarouselOp_ToolAlreadyInPocket if the tool already has a pocket assigned —
// use $TCADD to move an existing P0 tool into the carousel instead.
carousel_op_result_t tooltable_register_tool (tool_id_t tool_id, const char *name)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_index_entry_t *ie = index_find(tool_id);

    if(ie && ie->pocket_id >= 1)
        return CarouselOp_ToolAlreadyInPocket;   // already in carousel, use $TCADD to reassign

    if(ie) {
        // Tool exists as P0 — update name only if one was provided
        if(name && *name) {
            pocket_override_t ov = {0};
            ov.tool_id       = tool_id;
            ov.new_pocket_id = 0;   // keep at P0
            strncpy(ov.name, name, sizeof(ov.name) - 1);
            ov.name[sizeof(ov.name) - 1] = '\0';
            if(!rewrite_file(&ov, 1))
                return CarouselOp_WriteError;
            return CarouselOp_OK;  // name updated
        }
        // Tool already registered at P0, no name provided — nothing to do
        return CarouselOp_AlreadyRegistered;
    }

    // Brand-new tool — append as P0
    tool_pocket_t newentry = {0};
    newentry.tool.tool_id = tool_id;
    newentry.pocket_id    = 0;
    if(name && *name) {
        strncpy(newentry.name, name, sizeof(newentry.name) - 1);
        newentry.name[sizeof(newentry.name) - 1] = '\0';
    }
    if(!append_tool(&newentry))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// Public carousel API
// ---------------------------------------------------------------------------

carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id, uint16_t max_pockets, const char *name, pocket_id_t *assigned_pocket)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_index_entry_t *ie = index_find(tool_id);

    if(ie && ie->pocket_id >= 1)
        return CarouselOp_ToolAlreadyInPocket;

    if(max_pockets > 0 && index_count_in_carousel() >= max_pockets)
        return CarouselOp_NoPocketAvailable;

    pocket_id_t free_pocket = index_find_free_pocket();
    if(free_pocket < 0)
        return CarouselOp_NoPocketAvailable;

    if(ie) {
        // Tool exists as P0 in file — update pocket and optionally name via rewrite
        pocket_override_t ov = {0};
        ov.tool_id       = tool_id;
        ov.new_pocket_id = free_pocket;
        if(name && *name) {
            strncpy(ov.name, name, sizeof(ov.name) - 1);
            ov.name[sizeof(ov.name) - 1] = '\0';
        }
        if(!rewrite_file(&ov, 1))
            return CarouselOp_WriteError;
    } else {
        // Brand-new tool — append entry with optional name
        tool_pocket_t newentry = {0};
        newentry.tool.tool_id = tool_id;
        newentry.pocket_id    = free_pocket;
        if(name && *name) {
            strncpy(newentry.name, name, sizeof(newentry.name) - 1);
            newentry.name[sizeof(newentry.name) - 1] = '\0';
        }
        if(!append_tool(&newentry))
            return CarouselOp_WriteError;
    }

    if(assigned_pocket)
        *assigned_pocket = free_pocket;

    return CarouselOp_OK;
}


carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_index_entry_t *ie = index_find(tool_id);
    if(!ie || ie->pocket_id < 1)
        return CarouselOp_ToolNotFound;

    pocket_override_t ov = { .tool_id = tool_id, .new_pocket_id = -1 };
    if(!rewrite_file(&ov, 1))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// tooltable_delete() — Remove a P0 tool entry from the tooltable entirely.
//
// Only tools with no carousel pocket assignment (P0) may be deleted.  If the
// tool is currently assigned to a pocket, returns CarouselOp_ToolAlreadyInPocket
// so the caller can report a clear error without touching the file.
// ---------------------------------------------------------------------------
carousel_op_result_t tooltable_delete (tool_id_t tool_id)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_index_entry_t *ie = index_find(tool_id);
    if(!ie)
        return CarouselOp_ToolNotFound;

    if(ie->pocket_id >= 1)
        return CarouselOp_ToolAlreadyInPocket;

    pocket_override_t ov = { .tool_id = tool_id, .delete_entry = true };
    if(!rewrite_file(&ov, 1))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// onToolChanged - atomically update two pocket assignments after M6.
// ---------------------------------------------------------------------------
static void onToolChanged (tool_data_t *tool)
{
    if(settings.macro_atc_flags.random_toolchanger) {

        pocket_override_t overrides[MAX_OVERRIDES];
        uint8_t n_overrides = 0;

        // If the incoming tool is not in the file at all, create it with zeroed
        // offsets now so setTool() (called after probing) has a valid entry to update.
        tool_table_entry_t *existing = getTool(tool->tool_id);
        if(!existing->data && fs_available) {
            tool_pocket_t blank = {0};
            blank.tool.tool_id = tool->tool_id;
            blank.pocket_id    = -1;   // P0 — no pocket yet
            append_tool(&blank);
        }

        // Incoming tool: clear its carousel pocket (now in spindle)
        tool_index_entry_t *picked_up = index_find(tool->tool_id);
        if(picked_up && picked_up->pocket_id >= 1)
            overrides[n_overrides++] = (pocket_override_t){ tool->tool_id, -1 };

        // Outgoing tool: return it to its carousel pocket
        if(m6_prev_tool_pocket >= 1)
            overrides[n_overrides++] = (pocket_override_t){ current_tool, m6_prev_tool_pocket };

        m6_prev_tool_pocket = -1;

        if(n_overrides > 0)
            rewrite_file(overrides, n_overrides);
    }

    current_tool = tool->tool_id;

    if(on_tool_changed)
        on_tool_changed(tool);
}

static void onToolSelect (tool_data_t *tool, bool next)
{
    if(!next)
        current_tool = tool->tool_id;

    if(tool_select)
        tool_select(tool, next);
}

// ---------------------------------------------------------------------------
// $TTINDEX - print the RAM index to console for debugging.
// Shows only what is in the lightweight in-memory index (tool_id + pocket_id),
// not the full tool data from the file.
static status_code_t list_index (sys_state_t state, char *args)
{
    char buf[60];

    sprintf(buf, "[TTINDEX: %u tool(s) in index, capacity %u]" ASCII_EOL, n_tools, index_cap);
    hal.stream.write(buf);

    if(n_tools == 0) {
        hal.stream.write("[TTINDEX: empty]" ASCII_EOL);
        return Status_OK;
    }

    for(uint16_t i = 0; i < n_tools; i++) {
        uint16_t file_pocket = (tt_index[i].pocket_id >= 1) ? (uint16_t)tt_index[i].pocket_id : 0;
        sprintf(buf, "[TTINDEX: [%u] T%d P%u]" ASCII_EOL,
                i,
                (int)tt_index[i].tool_id,
                file_pocket);
        hal.stream.write(buf);
    }

    return Status_OK;
}

// $TTLIST - print tool table to console directly from file.
// ---------------------------------------------------------------------------
static status_code_t list_tools (sys_state_t state, char *args)
{
    hal.stream.write("[TOOLTABLE: P=pocket T=tool offsets diameter name]" ASCII_EOL);

    if(!fs_available || n_tools == 0) {
        hal.stream.write("[TOOL: table is empty]" ASCII_EOL);
        return Status_OK;
    }

    vfs_file_t *file = vfs_open(filename, "r");
    if(!file) {
        hal.stream.write("[TOOL: file not accessible]" ASCII_EOL);
        return Status_FileReadError;
    }

    char line[300], buf[200], tmp[24];
    tool_pocket_t entry;
    bool any = false;

    while(read_line(file, line, sizeof(line))) {

        if(!parse_line(line, &entry))
            continue;

        any = true;

        uint16_t file_pocket = (entry.pocket_id >= 1) ? (uint16_t)entry.pocket_id : 0;
        sprintf(buf, "[TOOL: P%u T%u", file_pocket, (uint16_t)entry.tool.tool_id);

        for(uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
            if(fabsf(entry.tool.offset.values[axis]) > 0.0001f) {
                sprintf(tmp, " %s%.3f", axis_letter[axis], entry.tool.offset.values[axis]);
                strcat(buf, tmp);
            }
        }

        if(entry.tool.radius != 0.0f) {
            sprintf(tmp, " D%.3f", entry.tool.radius * 2.0f);
            strcat(buf, tmp);
        }

        if(*entry.name) {
            strcat(buf, " ;");
            strcat(buf, entry.name);
        }

        strcat(buf, "]" ASCII_EOL);
        hal.stream.write(buf);
    }

    vfs_close(file);

    if(!any)
        hal.stream.write("[TOOL: table is empty]" ASCII_EOL);

    return Status_OK;
}

// ---------------------------------------------------------------------------
// $TTLOAD - reload tt_index from file
// ---------------------------------------------------------------------------
static status_code_t load_tools (sys_state_t state, char *args)
{
    if(!fs_available)
        return Status_FileReadError;

    return rebuild_index() ? Status_OK : Status_FileReadError;
}

static status_code_t reload_tools (void)
{
    return load_tools(state_get(), NULL);
}

// ---------------------------------------------------------------------------
// File management
// ---------------------------------------------------------------------------
static void ensure_tooltable_exists (void)
{
    vfs_file_t *file = vfs_open(filename, "r");
    if(file) {
        vfs_close(file);
        return;
    }

    vfs_mkdir("/linuxcnc");

    file = vfs_open(filename, "w");
    if(file) {
        vfs_close(file);
        report_message("Tooltable: created /linuxcnc/tooltable.tbl", Message_Info);
    } else {
        report_message("Tooltable: failed to create /linuxcnc/tooltable.tbl", Message_Warning);
    }
}

static void loadTools (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    fs_available = true;
    ensure_tooltable_exists();
    rebuild_index();

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------
static void onReportOptions (bool newopt)
{
    on_report_options(newopt);
    if(!newopt)
        report_plugin("Tool table", "0.04");
}

// $TTREG Tn [;name] — Register a tool in the tooltable at P0.
// ---------------------------------------------------------------------------
static status_code_t register_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTREG: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTREG: usage is $TTREG Tn [;name]", Message_Warning);
        return Status_BadNumberFormat;
    }

    uint32_t tool_id;
    const char *name = NULL;

    uint_fast8_t cc = 1;
    status_code_t parse_status = read_uint(args, &cc, &tool_id);
    if(parse_status != Status_OK) {
        report_message("TTREG: invalid tool number", Message_Warning);
        return parse_status;
    }
    while(args[cc] == ' ' || args[cc] == '\t') cc++;
    if(args[cc] == ';')
        name = &args[cc + 1];

    carousel_op_result_t result = tooltable_register_tool((tool_id_t)tool_id, name);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[80];
                if(name && *name)
                    snprintf(msg, sizeof(msg), "Tool %lu registered in tooltable as P0 (%s)", (unsigned long)tool_id, name);
                else
                    snprintf(msg, sizeof(msg), "Tool %lu registered in tooltable as P0", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_AlreadyRegistered:
            {
                char msg[60];
                snprintf(msg, sizeof(msg), "Tool %lu is already in the tooltable at P0", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_ToolAlreadyInPocket:
            report_message("TTREG: tool is already in a carousel pocket — use $TCADD to reassign", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_TableNotLoaded:
            report_message("TTREG: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TTREG: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TTREG: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
}

// $TTDEL Tn — Delete a P0 tool entry from the tooltable entirely.
// ---------------------------------------------------------------------------
static status_code_t delete_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTDEL: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTDEL: usage is $TTDEL Tn", Message_Warning);
        return Status_BadNumberFormat;
    }

    uint32_t tool_id;
    uint_fast8_t cc = 1;
    status_code_t parse_status = read_uint(args, &cc, &tool_id);
    if(parse_status != Status_OK) {
        report_message("TTDEL: invalid tool number", Message_Warning);
        return parse_status;
    }

    carousel_op_result_t result = tooltable_delete((tool_id_t)tool_id);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[48];
                sprintf(msg, "Tool %lu deleted from tooltable", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_ToolNotFound:
            report_message("TTDEL: tool not found in tooltable", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_ToolAlreadyInPocket:
            report_message("TTDEL: tool is in a carousel pocket — use $TCRM first", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_TableNotLoaded:
            report_message("TTDEL: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TTDEL: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TTDEL: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void tooltable_init (void)
{
    static const sys_command_t tt_command_list[] = {
        { "TTLOAD",  load_tools,     {}, { .str = "(re)load tool table from SD card" } },
        { "TTLIST",  list_tools,     {}, { .str = "List all tools in the tool table" } },
        { "TTINDEX", list_index,     {}, { .str = "Print the RAM index (debug)" } },
        { "TTREG",   register_tool,  {}, { .str = "Register a tool at P0 in the tooltable: $TTREG Tn [;name]" } },
        { "TTDEL",   delete_tool,    {}, { .str = "Delete a P0 tool entry from the tooltable: $TTDEL Tn" } }
    };

    static sys_commands_t tt_commands = {
        .n_commands = sizeof(tt_command_list) / sizeof(sys_command_t),
        .commands   = tt_command_list
    };

    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = loadTools;

    tool_select = hal.tool.select;
    hal.tool.select = onToolSelect;

    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = onToolChanged;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    grbl.tool_table.n_tools         = 0;
    grbl.tool_table.get_tool        = getTool;
    grbl.tool_table.reload           = reload_tools;
    grbl.tool_table.set_tool        = setTool;
    grbl.tool_table.get_tool_by_idx = getToolByIdx;
    grbl.tool_table.clear           = clearTools;

    system_register_commands(&tt_commands);

#if SDCARD_ENABLE
    sdcard_early_mount();
#endif
}

#endif // TOOLTABLE_ENABLE
