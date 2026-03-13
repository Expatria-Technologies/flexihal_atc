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
static char              filename[]   = "/tooltable.tbl";

// ---------------------------------------------------------------------------
// Tool change pocket tracking — volatile, lost on power cycle.
// last_fetched_pocket remembers which carousel pocket the current spindle
// tool came from so it can be returned there on the next tool change.
// ---------------------------------------------------------------------------
static pocket_id_t last_fetched_pocket = -1;  // pocket current spindle tool came from
static uint16_t    max_pockets         = 0;   // set by ATC plugin via tooltable_set_max_pockets()

// Zeroed fallback pocket — always valid, used before FS mounts or on empty table.
// Mirrors the pocket0 pattern from the TOOLTABLE_ENABLE==1 implementation.
static tool_pocket_t     pocket0      = {0};

static tool_select_ptr       tool_select;
static on_tool_changed_ptr   on_tool_changed;
static on_vfs_mount_ptr      on_vfs_mount;
static on_report_options_ptr on_report_options;

pocket_id_t tooltable_get_last_fetched_pocket (void)
{
    return last_fetched_pocket;
}

void tooltable_set_max_pockets (uint16_t n)
{
    max_pockets = n;
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

static pocket_id_t index_find_free_pocket (uint16_t max_pockets)
{
    pocket_id_t limit = (max_pockets > 0) ? (pocket_id_t)max_pockets : (pocket_id_t)(n_tools + 1);
    for(pocket_id_t candidate = 1; candidate <= limit; candidate++) {
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

    // FAT filesystems cannot rename over an existing file — delete first.
    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
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
        write_pocket_line(dst, &entry);
    }

    write_pocket_line(dst, p);

    vfs_close(src);
    vfs_close(dst);

    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

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
    static tool_table_entry_t tool = {0};
    static tool_scan_result_t scanned = {0};

    tool = (tool_table_entry_t){0};  // reset on every call — no stale data

    tool_index_entry_t *ie = index_find(tool_id);

    if(ie) {
        tool.data   = &ie->tool;
        tool.pocket = ie->pocket_id;
        tool.name   = ie->name;
        return &tool;
    }

    // Not in index — P0 tool or unknown. Scan file.
    vfs_file_t *file = vfs_open(filename, "r");
    if(!file) {
        tool.data   = &pocket0.tool;
        tool.pocket = pocket0.pocket_id;
        tool.name   = pocket0.name;
        return &tool;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(file, line, sizeof(line))) {
        if(parse_line(line, &entry) && entry.tool.tool_id == tool_id) {
            memcpy(&scanned.tool, &entry.tool, sizeof(tool_data_t));
            strncpy(scanned.name, entry.name, sizeof(scanned.name) - 1);
            scanned.name[sizeof(scanned.name) - 1] = '\0';
            tool.data   = &scanned.tool;
            tool.pocket = entry.pocket_id;
            tool.name   = scanned.name;
            break;
        }
    }
    vfs_close(file);

    // Not found in file — return pocket0 so grblHAL never sees NULL data
    // on an empty table. Callers that need to distinguish "not in carousel"
    // should check pocket_id, not data.
    if(!tool.data) {
        tool.data   = &pocket0.tool;
        tool.pocket = pocket0.pocket_id;
        tool.name   = pocket0.name;
    }

    return &tool;
}

// ---------------------------------------------------------------------------
// grbl.tool_table.get_tool_by_idx - same implementation as getTool.
// ---------------------------------------------------------------------------
static tool_table_entry_t *getToolByIdx (uint32_t idx)
{
    static tool_table_entry_t tool = {0};

    tool = (tool_table_entry_t){0};  // reset on every call

    // idx is a pocket number (1-based). Scan the index for the tool in that pocket.
    for(uint16_t i = 0; i < n_tools; i++) {
        if(tt_index[i].pocket_id == (pocket_id_t)idx)
            return getTool(tt_index[i].tool_id);
    }

    // Not found — return pocket0 (zeroed, tool_id=0)
    tool.data   = &pocket0.tool;
    tool.pocket = pocket0.pocket_id;
    tool.name   = pocket0.name;
    return &tool;
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

    vfs_unlink(filename);
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
    // Zero offsets in the RAM index. If the table hasn't been loaded yet,
    // also reset pocket and tool IDs — matching the == 1 implementation.
    if(!fs_available || n_tools == 0) {
        // Nothing loaded yet — just reset pocket0
        pocket0.tool.radius = 0.0f;
        memset(&pocket0.tool.offset, 0, sizeof(coord_data_t));
        return true;
    }

    for(uint16_t i = 0; i < n_tools; i++) {
        tt_index[i].tool.radius = 0.0f;
        memset(&tt_index[i].tool.offset, 0, sizeof(coord_data_t));
    }

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

    if(ie) {
        // Tool already exists — update name if one was provided, preserve pocket
        if(name && *name) {
            pocket_override_t ov = {0};
            ov.tool_id       = tool_id;
            ov.new_pocket_id = ie->pocket_id;   // preserve existing pocket
            strncpy(ov.name, name, sizeof(ov.name) - 1);
            ov.name[sizeof(ov.name) - 1] = '\0';
            if(!rewrite_file(&ov, 1))
                return CarouselOp_WriteError;
            return CarouselOp_OK;
        }
        // Tool already registered, no name provided — nothing to do
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

    pocket_id_t free_pocket = index_find_free_pocket(max_pockets);
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
    // Ensure incoming tool has a table entry
    if(fs_available && index_find(tool->tool_id) == NULL) {
        tool_pocket_t blank = {0};
        blank.tool.tool_id = tool->tool_id;
        blank.pocket_id    = -1;
        append_tool(&blank);
    }

    if(max_pockets > 0) {
        // Incoming tool fetched from carousel — mark its pocket empty
        tool_index_entry_t *incoming = index_find(tool->tool_id);
        if(incoming && incoming->pocket_id >= 1)
            tooltable_carousel_remove(tool->tool_id);

        // Outgoing tool — return it to a free carousel pocket
        if(last_fetched_pocket >= 1)
            tooltable_carousel_add(current_tool, max_pockets, NULL, &last_fetched_pocket);

        // Update last_fetched_pocket for next tool change
        incoming = index_find(tool->tool_id);
        last_fetched_pocket = (incoming && incoming->pocket_id >= 1)
                              ? incoming->pocket_id : -1;
    }

    current_tool = tool->tool_id;

    if(on_tool_changed)
        on_tool_changed(tool);
}

static void onToolSelect (tool_data_t *tool, bool next)
{
    if(!next)
        current_tool = tool->tool_id;

    if(next && max_pockets > 0) {
        // Use grblHAL's current tool data directly — reliable on boot
        // since grblHAL restores it from persistent storage before we run
        tool_id_t outgoing_id = gc_state.tool ? gc_state.tool->tool_id : 0;

        tool_index_entry_t *incoming = index_find(tool->tool_id);
        pocket_id_t incoming_pocket = (incoming && incoming->pocket_id >= 1)
                                      ? incoming->pocket_id : -1;

        ngc_param_set(4900, (float)outgoing_id);
        ngc_param_set(4901, (float)last_fetched_pocket);
        ngc_param_set(4902, (float)tool->tool_id);
        ngc_param_set(4903, (float)incoming_pocket);
    }

    if(tool_select)
        tool_select(tool, next);
}

static on_macro_return_ptr on_macro_return = NULL;

static void onMacroReturn (void)
{
    // If tc.macro (id=99) just completed, fire on_tool_changed manually
    // since macro_tool_change() never calls gc_tool_changed()
    if(gc_state.tool && gc_state.tool->tool_id != current_tool) {
        if(grbl.on_tool_changed)
            grbl.on_tool_changed(gc_state.tool);
    }

    if(on_macro_return)
        on_macro_return();
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

// $TTREG=Tn [,name] — Register a tool in the tooltable at P0.
// ---------------------------------------------------------------------------
static status_code_t register_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTREG: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTREG: usage is $TTREG=Tn [,name]", Message_Warning);
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
    if(args[cc] == ',')
        name = &args[cc + 1];

    carousel_op_result_t result = tooltable_register_tool((tool_id_t)tool_id, name);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[80];
                if(name && *name)
                    snprintf(msg, sizeof(msg), "Tool %lu updated in tooltable (%s)", (unsigned long)tool_id, name);
                else
                    snprintf(msg, sizeof(msg), "Tool %lu registered in tooltable", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_AlreadyRegistered:
            {
                char msg[60];
                snprintf(msg, sizeof(msg), "Tool %lu already in tooltable — no changes made", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

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

// $TTDEL=Tn — Delete a P0 tool entry from the tooltable entirely.
// ---------------------------------------------------------------------------
static status_code_t delete_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTDEL: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTDEL: usage is $TTDEL=Tn", Message_Warning);
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
        { "TTREG",   register_tool,  {}, { .str = "Register a tool at P0 in the tooltable: $TTREG=Tn [,name]" } },
        { "TTDEL",   delete_tool,    {}, { .str = "Delete a P0 tool entry from the tooltable: $TTDEL=Tn" } }
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

    on_macro_return = grbl.on_macro_return;
    grbl.on_macro_return = onMacroReturn;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    // Initialise pocket0 as a safe zeroed fallback — tool_id=0, all offsets=0.
    // This mirrors the TOOLTABLE_ENABLE==1 pattern so getTool/getToolByIdx
    // always return valid data even before the VFS mounts or if the table is empty.
    memset(&pocket0, 0, sizeof(tool_pocket_t));
    pocket0.tool.tool_id = 0;
    pocket0.pocket_id    = -1;
    
    grbl.tool_table.n_tools         = 1;
    grbl.tool_table.get_tool        = getTool;
    grbl.tool_table.reload           = reload_tools;
    grbl.tool_table.set_tool        = setTool;
    grbl.tool_table.get_tool_by_idx = getToolByIdx;
    grbl.tool_table.clear           = clearTools;

    system_register_commands(&tt_commands);

    settings.macro_atc_flags.random_toolchanger = 1;

    // Seed current_tool from grblHAL's persisted spindle tool on boot
    current_tool = gc_state.tool ? gc_state.tool->tool_id : 0;

#if SDCARD_ENABLE
    sdcard_early_mount();
#endif
}

#endif // TOOLTABLE_ENABLE
