"""
tooltable_sync.py
-----------------
Syncs a local grblHAL/LinuxCNC tooltable (.tbl) with a CNC machine over FTP.

Sync rules:
  - Machine ALWAYS wins on Z offsets and pocket assignments.
  - New tools present in the local file but missing on the machine are pushed
    to the machine as P0 / Z0.000 entries (unassigned).
  - Tools present on the machine but missing locally are pulled into the local
    file so both stay complete.

Build to .exe (run once on dev machine):
  pip install pyinstaller
  pyinstaller --onefile --windowed tooltable_sync.py
"""

import ftplib
import io
import json
import os
import re
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

CONFIG_FILE = os.path.join(os.path.expanduser("~"), ".tooltable_sync.json")

# ---------------------------------------------------------------------------
# Tooltable parsing / serialisation
# ---------------------------------------------------------------------------

TOOL_RE = re.compile(
    r"^(?:(P\d+)\s+)?(T\d+)"
    r"(?:\s+X([-\d.]+))?(?:\s+Y([-\d.]+))?(?:\s+Z([-\d.]+))?"
    r"(?:\s+D([-\d.]+))?"
    r"(?:\s*;\s*(.*))?$",
    re.IGNORECASE,
)


def parse_table(text: str) -> tuple[list[dict], list[str]]:
    """Return (tools, header_comments).

    Handles both 'P0 T2 Z...' and bare 'T2 Z...' formats — lines without
    a pocket prefix are treated as P0 (unassigned).
    """
    tools: list[dict] = []
    headers: list[str] = []
    in_header = True

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        m = TOOL_RE.match(stripped)
        if m:
            in_header = False
            pocket_grp = m.group(1)   # e.g. "P0" or None
            tool_grp   = m.group(2)   # e.g. "T2"
            tools.append({
                "pocket":   int(pocket_grp[1:]) if pocket_grp else 0,
                "tool":     int(tool_grp[1:]),
                "x":        float(m.group(3)) if m.group(3) else None,
                "y":        float(m.group(4)) if m.group(4) else None,
                "z":        float(m.group(5)) if m.group(5) else 0.0,
                "diameter": float(m.group(6)) if m.group(6) else None,
                "name":     (m.group(7) or "").strip(),
            })
        elif stripped.startswith(";") and in_header:
            headers.append(line)

    return tools, headers


def serialise_table(tools: list[dict], headers: list[str]) -> str:
    lines = list(headers) + [""]
    for t in sorted(tools, key=lambda x: (x["pocket"] == 0, x["tool"])):
        parts = [f"P{t['pocket']}", f"T{t['tool']}"]
        if t.get("x") is not None:
            parts.append(f"X{t['x']:.3f}")
        if t.get("y") is not None:
            parts.append(f"Y{t['y']:.3f}")
        parts.append(f"Z{t['z']:.3f}")
        if t.get("diameter") is not None:
            parts.append(f"D{t['diameter']:.3f}")
        line = " ".join(parts)
        if t.get("name"):
            line += f" ; {t['name']}"
        lines.append(line)
    return "\n".join(lines) + "\n"


def merge(machine_tools: list[dict], local_tools: list[dict]) -> tuple[list[dict], list[str]]:
    """Merge local into machine. Machine wins on all conflicts."""
    log: list[str] = []
    machine_by_t = {t["tool"]: t for t in machine_tools}
    local_by_t   = {t["tool"]: t for t in local_tools}

    for tn, lt in local_by_t.items():
        if tn in machine_by_t:
            mt = machine_by_t[tn]
            diffs = []
            if abs(lt["z"] - mt["z"]) > 0.0005:
                diffs.append(f"Z {lt['z']:.3f}→{mt['z']:.3f}")
            if lt["pocket"] != mt["pocket"]:
                diffs.append(f"pocket {lt['pocket']}→{mt['pocket']}")
            if diffs:
                log.append(f"  T{tn} ({mt.get('name','')}) machine overrides: {', '.join(diffs)}")
        else:
            new_entry = dict(lt)
            new_entry["pocket"] = 0
            new_entry["z"] = 0.0
            machine_by_t[tn] = new_entry
            log.append(f"  T{tn} ({lt.get('name','')}) added to machine as P0 Z0.000")

    for tn in machine_by_t:
        if tn not in local_by_t:
            log.append(f"  T{tn} ({machine_by_t[tn].get('name','')}) pulled from machine (not in local file)")

    return list(machine_by_t.values()), log


# ---------------------------------------------------------------------------
# FTP helpers
# ---------------------------------------------------------------------------

def ftp_connect(host: str, port: int, user: str, password: str) -> ftplib.FTP:
    ftp = ftplib.FTP()
    ftp.connect(host, port, timeout=10)
    ftp.login(user, password)
    ftp.set_pasv(True)
    return ftp


def ftp_read(ftp: ftplib.FTP, remote_path: str) -> str:
    buf = io.BytesIO()
    ftp.retrbinary(f"RETR {remote_path}", buf.write)
    return buf.getvalue().decode("utf-8", errors="replace")


def ftp_write(ftp: ftplib.FTP, remote_path: str, content: str) -> None:
    buf = io.BytesIO(content.encode("utf-8"))
    ftp.storbinary(f"STOR {remote_path}", buf)


# ---------------------------------------------------------------------------
# Config persistence
# ---------------------------------------------------------------------------

def load_config() -> dict:
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def save_config(cfg: dict) -> None:
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

DARK_BG  = "#1a1a2e"
PANEL_BG = "#16213e"
ACCENT   = "#e94560"
FG       = "#eaeaea"
MUTED    = "#8892a4"
ENTRY_BG = "#0f3460"
TV_BG    = "#0a0a1a"
TV_ALT   = "#0d1525"
MONO     = ("Consolas", 9)
LABEL_F  = ("Segoe UI", 9)
HEADER_F = ("Segoe UI Semibold", 10)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Tool Table Sync")
        self.resizable(True, True)
        self.configure(bg=DARK_BG)
        self.minsize(780, 680)

        self._cfg = load_config()
        self._current_tools: list[dict] = []
        self._tv_sort_col = "tool"
        self._tv_sort_rev = False

        self._build_styles()
        self._build_ui()
        self._restore_config()

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure("TFrame",        background=DARK_BG)
        s.configure("Panel.TFrame",  background=PANEL_BG)
        s.configure("TLabel",        background=DARK_BG,  foreground=FG,    font=LABEL_F)
        s.configure("Panel.TLabel",  background=PANEL_BG, foreground=FG,    font=LABEL_F)
        s.configure("Muted.TLabel",  background=PANEL_BG, foreground=MUTED, font=("Segoe UI", 8))
        s.configure("DarkMuted.TLabel", background=DARK_BG, foreground=MUTED, font=("Segoe UI", 8))
        s.configure("Header.TLabel", background=PANEL_BG, foreground=ACCENT, font=HEADER_F)
        s.configure("TEntry",        fieldbackground=ENTRY_BG, foreground=FG,
                    insertcolor=FG, borderwidth=0)
        s.configure("Accent.TButton", background=ACCENT, foreground="#ffffff",
                    font=("Segoe UI Semibold", 9), borderwidth=0, relief="flat")
        s.map("Accent.TButton",
              background=[("active", "#c73652"), ("disabled", "#4a4a6a")],
              foreground=[("disabled", "#888")])
        s.configure("Ghost.TButton", background=PANEL_BG, foreground=MUTED,
                    font=("Segoe UI", 9), borderwidth=1, relief="flat")
        s.map("Ghost.TButton", foreground=[("active", FG)])

        s.configure("Treeview",
                    background=TV_BG, fieldbackground=TV_BG,
                    foreground="#c8d8e8", rowheight=22,
                    font=MONO, borderwidth=0)
        s.configure("Treeview.Heading",
                    background=PANEL_BG, foreground=ACCENT,
                    font=("Segoe UI Semibold", 9), relief="flat")
        s.map("Treeview",
              background=[("selected", ENTRY_BG)],
              foreground=[("selected", "#ffffff")])
        s.map("Treeview.Heading",
              background=[("active", ENTRY_BG)])

        s.configure("TSeparator", background="#2a2a4a")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        root_frame = ttk.Frame(self, padding=16)
        root_frame.pack(fill="both", expand=True)
        root_frame.rowconfigure(2, weight=1)
        root_frame.columnconfigure(0, weight=1)

        # ── Title ──────────────────────────────────────────────────────
        title_row = ttk.Frame(root_frame)
        title_row.grid(row=0, column=0, sticky="ew", pady=(0, 14))
        tk.Label(title_row, text="⟳", font=("Segoe UI", 22), fg=ACCENT,
                 bg=DARK_BG).pack(side="left", padx=(0, 8))
        tk.Label(title_row, text="Tool Table Sync",
                 font=("Segoe UI Semibold", 16), fg=FG, bg=DARK_BG).pack(side="left")
        tk.Label(title_row, text="grblHAL · LinuxCNC",
                 font=("Segoe UI", 9), fg=MUTED, bg=DARK_BG).pack(side="left", padx=12)

        # ── Config panels (two columns) ────────────────────────────────
        cfg_row = ttk.Frame(root_frame)
        cfg_row.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        cfg_row.columnconfigure(0, weight=1)
        cfg_row.columnconfigure(1, weight=1)

        # Left: local file
        lp = ttk.Frame(cfg_row, style="Panel.TFrame", padding=12)
        lp.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(lp, text="LOCAL FILE", style="Header.TLabel").pack(anchor="w")
        ttk.Label(lp, text="The .tbl file on this PC", style="Muted.TLabel").pack(anchor="w", pady=(0, 8))
        file_row = ttk.Frame(lp, style="Panel.TFrame")
        file_row.pack(fill="x")
        self._local_path = tk.StringVar()
        ttk.Entry(file_row, textvariable=self._local_path).pack(side="left", fill="x", expand=True)
        ttk.Button(file_row, text="Browse", style="Ghost.TButton",
                   command=self._browse).pack(side="left", padx=(6, 0))

        # Right: FTP
        rp = ttk.Frame(cfg_row, style="Panel.TFrame", padding=12)
        rp.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ttk.Label(rp, text="MACHINE FTP", style="Header.TLabel").pack(anchor="w")
        ttk.Label(rp, text="Connection to the CNC controller", style="Muted.TLabel").pack(anchor="w", pady=(0, 8))

        def labeled_entry(parent, label, var, show=None):
            row = ttk.Frame(parent, style="Panel.TFrame")
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=f"{label:<12}", style="Panel.TLabel", width=12).pack(side="left")
            kw = dict(textvariable=var)
            if show:
                kw["show"] = show
            ttk.Entry(row, **kw).pack(side="left", fill="x", expand=True)

        self._ftp_host     = tk.StringVar()
        self._ftp_port     = tk.StringVar(value="21")
        self._ftp_user     = tk.StringVar(value="anonymous")
        self._ftp_password = tk.StringVar()
        self._ftp_path     = tk.StringVar(value="/tooltable.tbl")

        labeled_entry(rp, "Host",        self._ftp_host)
        labeled_entry(rp, "Port",        self._ftp_port)
        labeled_entry(rp, "User",        self._ftp_user)
        labeled_entry(rp, "Password",    self._ftp_password, show="•")
        labeled_entry(rp, "Remote path", self._ftp_path)

        # ── Action buttons ─────────────────────────────────────────────
        btn_row = ttk.Frame(root_frame)
        btn_row.grid(row=2, column=0, sticky="ew", pady=(0, 10))

        ttk.Button(btn_row, text="⟳  Sync Now", style="Accent.TButton",
                   command=self._sync).pack(side="left", ipady=6, ipadx=12)
        ttk.Button(btn_row, text="↓  Pull from machine", style="Ghost.TButton",
                   command=self._pull_only).pack(side="left", padx=(10, 0), ipady=6, ipadx=8)
        ttk.Button(btn_row, text="↑  Push local only", style="Ghost.TButton",
                   command=self._push_only).pack(side="left", padx=(6, 0), ipady=6, ipadx=8)
        ttk.Button(btn_row, text="↺  Load local file", style="Ghost.TButton",
                   command=self._load_local_display).pack(side="left", padx=(6, 0), ipady=6, ipadx=8)
        ttk.Button(btn_row, text="Save settings", style="Ghost.TButton",
                   command=self._save_settings).pack(side="right", ipady=6, ipadx=8)

        # ── Main lower area: tool table + log ──────────────────────────
        lower = ttk.Frame(root_frame)
        lower.grid(row=3, column=0, sticky="nsew")
        root_frame.rowconfigure(3, weight=1)
        lower.rowconfigure(0, weight=3)
        lower.rowconfigure(1, weight=1)
        lower.columnconfigure(0, weight=1)

        # ── Tool table panel ───────────────────────────────────────────
        tv_panel = ttk.Frame(lower, style="Panel.TFrame", padding=10)
        tv_panel.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        tv_panel.rowconfigure(1, weight=1)
        tv_panel.columnconfigure(0, weight=1)

        tv_hdr = ttk.Frame(tv_panel, style="Panel.TFrame")
        tv_hdr.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        ttk.Label(tv_hdr, text="TOOL TABLE", style="Header.TLabel").pack(side="left")
        self._tool_count_var = tk.StringVar(value="no data — sync or load local file")
        ttk.Label(tv_hdr, textvariable=self._tool_count_var,
                  style="Muted.TLabel").pack(side="left", padx=12)

        # legend
        tk.Label(tv_hdr, text="■", fg="#facc15", bg=PANEL_BG,
                 font=("Segoe UI", 8)).pack(side="right")
        ttk.Label(tv_hdr, text="unset (P0 / Z=0)",
                  style="Muted.TLabel").pack(side="right", padx=(0, 4))

        tree_cols = ("pocket", "tool", "diameter", "z_offset", "name")
        self._tv = ttk.Treeview(tv_panel, columns=tree_cols,
                                show="headings", selectmode="browse")

        col_cfg = [
            ("pocket",   "Pocket",   65,  "center"),
            ("tool",     "Tool #",   65,  "center"),
            ("diameter", "Ø mm",     80,  "center"),
            ("z_offset", "Z Offset", 100, "center"),
            ("name",     "Description", 480, "w"),
        ]
        for cid, heading, width, anchor in col_cfg:
            self._tv.heading(cid, text=heading,
                             command=lambda c=cid: self._sort_tv(c))
            self._tv.column(cid, width=width, minwidth=40, anchor=anchor)

        # row tags
        self._tv.tag_configure("odd",     background=TV_BG,  foreground="#c8d8e8")
        self._tv.tag_configure("even",    background=TV_ALT, foreground="#c8d8e8")
        self._tv.tag_configure("unset_o", background=TV_BG,  foreground="#facc15")
        self._tv.tag_configure("unset_e", background=TV_ALT, foreground="#facc15")

        tv_scroll = ttk.Scrollbar(tv_panel, orient="vertical",
                                  command=self._tv.yview)
        self._tv.configure(yscrollcommand=tv_scroll.set)
        self._tv.grid(row=1, column=0, sticky="nsew")
        tv_scroll.grid(row=1, column=1, sticky="ns")

        # ── Log panel ─────────────────────────────────────────────────
        log_panel = ttk.Frame(lower, style="Panel.TFrame", padding=10)
        log_panel.grid(row=1, column=0, sticky="nsew")
        log_panel.rowconfigure(1, weight=1)
        log_panel.columnconfigure(0, weight=1)

        ttk.Label(log_panel, text="LOG", style="Header.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 4))

        self._log = scrolledtext.ScrolledText(
            log_panel, height=7, font=MONO,
            bg=TV_BG, fg="#c8d8e8", insertbackground=FG,
            relief="flat", borderwidth=0, state="disabled", wrap="word",
        )
        self._log.grid(row=1, column=0, sticky="nsew")

        self._log.tag_config("ok",    foreground="#4ade80")
        self._log.tag_config("warn",  foreground="#facc15")
        self._log.tag_config("error", foreground="#f87171")
        self._log.tag_config("info",  foreground="#93c5fd")
        self._log.tag_config("muted", foreground="#4a5568")

    # ------------------------------------------------------------------
    # Tool table display
    # ------------------------------------------------------------------

    def _populate_tv(self, tools: list[dict]):
        """Populate the treeview with a list of tool dicts."""
        self._current_tools = tools
        self._render_tv()

    def _render_tv(self):
        """Re-render treeview from self._current_tools respecting sort state."""
        col = self._tv_sort_col
        rev = self._tv_sort_rev

        def sort_key(t):
            if col == "pocket":
                return t["pocket"]
            elif col == "tool":
                return t["tool"]
            elif col == "diameter":
                return t["diameter"] if t["diameter"] is not None else -1
            elif col == "z_offset":
                return t["z"]
            else:
                return (t.get("name") or "").lower()

        sorted_tools = sorted(self._current_tools, key=sort_key, reverse=rev)

        # clear existing rows
        for row in self._tv.get_children():
            self._tv.delete(row)

        for i, t in enumerate(sorted_tools):
            pocket  = f"P{t['pocket']}"
            tool    = f"T{t['tool']}"
            diam    = f"{t['diameter']:.3f}" if t["diameter"] is not None else "—"
            z       = f"{t['z']:.3f}"
            name    = t.get("name") or ""
            unset   = (t["pocket"] == 0 or abs(t["z"]) < 0.0005)
            even    = (i % 2 == 0)
            if unset:
                tag = "unset_e" if even else "unset_o"
            else:
                tag = "even" if even else "odd"
            self._tv.insert("", "end", values=(pocket, tool, diam, z, name), tags=(tag,))

        n = len(sorted_tools)
        assigned = sum(1 for t in sorted_tools if t["pocket"] != 0)
        self._tool_count_var.set(
            f"{n} tool{'s' if n != 1 else ''}  ·  {assigned} assigned  ·  {n - assigned} unset"
        )

    def _sort_tv(self, col: str):
        if self._tv_sort_col == col:
            self._tv_sort_rev = not self._tv_sort_rev
        else:
            self._tv_sort_col = col
            self._tv_sort_rev = False
        self._render_tv()

    # ------------------------------------------------------------------
    # Config helpers
    # ------------------------------------------------------------------

    def _restore_config(self):
        c = self._cfg
        self._local_path.set(c.get("local_path", ""))
        self._ftp_host.set(c.get("ftp_host", ""))
        self._ftp_port.set(c.get("ftp_port", "21"))
        self._ftp_user.set(c.get("ftp_user", "anonymous"))
        self._ftp_password.set(c.get("ftp_password", ""))
        self._ftp_path.set(c.get("ftp_path", "/tooltable.tbl"))

    def _save_settings(self):
        self._cfg.update({
            "local_path":   self._local_path.get(),
            "ftp_host":     self._ftp_host.get(),
            "ftp_port":     self._ftp_port.get(),
            "ftp_user":     self._ftp_user.get(),
            "ftp_password": self._ftp_password.get(),
            "ftp_path":     self._ftp_path.get(),
        })
        save_config(self._cfg)
        self._log_line("Settings saved.", "ok")

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Select local tool table",
            filetypes=[("Tool table", "*.tbl"), ("All files", "*.*")],
        )
        if path:
            self._local_path.set(path)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_line(self, text: str, tag: str = "info"):
        self._log.configure(state="normal")
        self._log.insert("end", text + "\n", tag)
        self._log.see("end")
        self._log.configure(state="disabled")

    def _log_sep(self):
        self._log_line("─" * 60, "muted")

    def _log_clear(self):
        self._log.configure(state="normal")
        self._log.delete("1.0", "end")
        self._log.configure(state="disabled")

    # ------------------------------------------------------------------
    # FTP params validation
    # ------------------------------------------------------------------

    def _ftp_params(self):
        host     = self._ftp_host.get().strip()
        port     = int(self._ftp_port.get().strip() or "21")
        user     = self._ftp_user.get().strip()
        password = self._ftp_password.get()
        path     = self._ftp_path.get().strip()
        local    = self._local_path.get().strip()
        if not host:
            raise ValueError("FTP host is required.")
        if not local:
            raise ValueError("Local file path is required.")
        return host, port, user, password, path, local

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _load_local_display(self):
        """Load and display the local file without touching the machine."""
        local = self._local_path.get().strip()
        if not local or not os.path.exists(local):
            messagebox.showwarning("No file", "Select a valid local .tbl file first.")
            return
        with open(local, "r", encoding="utf-8") as f:
            text = f.read()
        tools, _ = parse_table(text)
        self._populate_tv(tools)
        self._log_line(f"Loaded {len(tools)} tools from {local}", "info")

    def _pull_only(self):
        self._log_clear()
        self._log_sep()
        self._log_line("↓  Pull from machine (overwrites local)", "info")
        self._log_sep()
        try:
            host, port, user, pw, rpath, local = self._ftp_params()
            self._log_line(f"Connecting to {host}:{port} …", "info")
            ftp = ftp_connect(host, port, user, pw)
            content = ftp_read(ftp, rpath)
            ftp.quit()
            with open(local, "w", encoding="utf-8") as f:
                f.write(content)
            tools, _ = parse_table(content)
            self._populate_tv(tools)
            self._log_line(f"Pulled {len(tools)} tools → {local}", "ok")
        except Exception as e:
            self._log_line(f"ERROR: {e}", "error")

    def _push_only(self):
        self._log_clear()
        self._log_sep()
        self._log_line("↑  Push local file to machine (no merge)", "warn")
        self._log_sep()
        if not messagebox.askyesno(
            "Push local only",
            "This will overwrite the machine tool table with your local file.\n"
            "Machine offsets and pocket numbers will be lost.\n\nContinue?",
        ):
            self._log_line("Cancelled.", "muted")
            return
        try:
            host, port, user, pw, rpath, local = self._ftp_params()
            with open(local, "r", encoding="utf-8") as f:
                content = f.read()
            self._log_line(f"Connecting to {host}:{port} …", "info")
            ftp = ftp_connect(host, port, user, pw)
            ftp_write(ftp, rpath, content)
            ftp.quit()
            tools, _ = parse_table(content)
            self._populate_tv(tools)
            self._log_line(f"Pushed {len(tools)} tools → {rpath}", "ok")
        except Exception as e:
            self._log_line(f"ERROR: {e}", "error")

    def _sync(self):
        self._log_clear()
        self._log_sep()
        self._log_line("⟳  Sync — machine wins on offsets & pockets", "info")
        self._log_sep()
        try:
            host, port, user, pw, rpath, local = self._ftp_params()

            # 1. Read local file
            if os.path.exists(local):
                with open(local, "r", encoding="utf-8") as f:
                    local_text = f.read()
                local_tools, local_headers = parse_table(local_text)
                self._log_line(f"Local:   {len(local_tools)} tools in {local}", "info")
            else:
                local_tools, local_headers = [], []
                self._log_line("Local file not found — will create from machine.", "warn")

            # 2. Fetch machine file
            self._log_line(f"Connecting to {host}:{port} …", "info")
            ftp = ftp_connect(host, port, user, pw)
            machine_text = ftp_read(ftp, rpath)
            machine_tools, machine_headers = parse_table(machine_text)
            self._log_line(f"Machine: {len(machine_tools)} tools at {rpath}", "info")
            self._log_sep()

            # 3. Merge
            merged, merge_log = merge(machine_tools, local_tools)
            if merge_log:
                for entry in merge_log:
                    tag = "warn" if "overrides" in entry else "ok"
                    self._log_line(entry, tag)
            else:
                self._log_line("No differences found.", "muted")
            self._log_sep()

            # 4. Serialise
            headers = machine_headers if machine_headers else local_headers
            merged_text = serialise_table(merged, headers)

            # 5. Write to machine
            ftp_write(ftp, rpath, merged_text)
            self._log_line(f"↑ Wrote {len(merged)} tools to machine ({rpath})", "ok")

            # 6. Write local file
            with open(local, "w", encoding="utf-8") as f:
                f.write(merged_text)
            self._log_line(f"↓ Updated local file ({local})", "ok")

            ftp.quit()
            self._log_sep()
            self._log_line("Sync complete ✓", "ok")

            # 7. Update display
            self._populate_tv(merged)

        except Exception as e:
            self._log_line(f"ERROR: {e}", "error")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = App()
    app.mainloop()
