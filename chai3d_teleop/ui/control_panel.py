#!/usr/bin/env python3
"""Local control panel for every Flexiv/wrist mission, tool, and setting."""

from __future__ import annotations

import argparse
from collections import deque
import grp
import json
import os
from pathlib import Path
import pwd
import re
import secrets
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
import tomllib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse
import webbrowser


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
FREEDRIVE_VENV = PROJECT_ROOT.parent / "freedrive_python" / ".venv"
TELEOP_SCRIPT = PROJECT_ROOT / "scripts" / "teleoperate.py"
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"


def ensure_project_python() -> None:
    expected = FREEDRIVE_VENV.resolve()
    if Path(sys.prefix).resolve() == expected:
        return
    executable = FREEDRIVE_VENV / "bin" / "python"
    if not executable.is_file():
        raise FileNotFoundError(f"Flexiv Python environment not found: {executable}")
    os.execv(str(executable), [str(executable), str(SCRIPT_PATH), *sys.argv[1:]])


ensure_project_python()
sys.path.insert(0, str(PROJECT_ROOT))

from controllers.config_io import (  # noqa: E402
    extract_toml_comments,
    normalize_config_document,
    rewrite_toml_document,
)


MODE_NAMES = {1: "7dof", 2: "9dof", 3: "pivot_orientation"}
STATUS_RE = re.compile(
    r"mode=(\S+) ready=(\d) clutch_pressed=(\d) enabled=(\d)"
)
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
TELEMETRY_PREFIX = "TELEMETRY "


def _primary_error(current: str, message: str) -> str:
    """Keep a specific bridge error instead of its later exit-code wrapper."""
    specific_prefixes = ("[wrist] WRIST_ERROR",)
    generic_wrappers = (
        "错误: 腕部 bridge 启动失败",
        "错误: 腕部 bridge 已退出",
    )
    if message.startswith(generic_wrappers) and current.startswith(specific_prefixes):
        return current
    return message


def _teleop_command_with_fdcanusb_access(
    command: list[str], config_path: Path
) -> tuple[list[str], str]:
    """Validate fdcanusb and use ``sg dialout`` for a stale login session."""
    with config_path.open("rb") as stream:
        document = tomllib.load(stream)
    configured = str(document.get("wrist", {}).get("fdcanusb", "")).strip()
    if not configured:
        raise RuntimeError("wrist.fdcanusb is empty")
    device = Path(configured)
    if not device.exists():
        raise RuntimeError(
            f"fdcanusb device not found: {device}. Reconnect the adapter and reload the UI."
        )
    if os.access(device, os.R_OK | os.W_OK):
        return command, f"fdcanusb access OK: {device}"

    try:
        group = grp.getgrnam("dialout")
        username = pwd.getpwuid(os.getuid()).pw_name
    except KeyError as error:
        raise RuntimeError(
            f"No read/write permission for {device}, and the dialout group is unavailable"
        ) from error
    eligible = os.getgid() == group.gr_gid or username in group.gr_mem
    sg_executable = shutil.which("sg")
    if eligible and sg_executable:
        wrapped = [sg_executable, "dialout", "-c", "exec " + shlex.join(command)]
        return wrapped, (
            f"fdcanusb is not accessible in this VS Code process; "
            f"launching teleoperation through sg dialout for {device}"
        )
    raise RuntimeError(
        f"No read/write permission for {device}. Add {username} to dialout, then fully restart "
        "VS Code or launch the UI with: sg dialout -c '<UI command>'"
    )


class TeleopProcessManager:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._logs: deque[tuple[int, str]] = deque(maxlen=5000)
        self._sequence = 0
        self._telemetry: deque[tuple[int, dict[str, Any]]] = deque(maxlen=6000)
        self._telemetry_sequence = 0
        self._state: dict[str, Any] = {
            "running": False,
            "pid": None,
            "exit_code": None,
            "phase": "stopped",
            "mode": "waiting",
            "requested_mode": None,
            "ready": False,
            "clutch_pressed": False,
            "enabled": False,
            "last_error": "",
            "task": None,
        }

    def _append(self, line: str) -> None:
        message = ANSI_ESCAPE_RE.sub("", line.rstrip("\r\n"))
        if not message:
            return
        if message.startswith(TELEMETRY_PREFIX):
            try:
                sample = json.loads(message[len(TELEMETRY_PREFIX):])
                if not isinstance(sample, dict):
                    raise ValueError("telemetry payload is not an object")
            except (json.JSONDecodeError, ValueError) as error:
                message = f"[UI] Ignored malformed telemetry: {error}"
            else:
                with self._lock:
                    self._telemetry_sequence += 1
                    self._telemetry.append((self._telemetry_sequence, sample))
                return
        with self._lock:
            self._sequence += 1
            self._logs.append((self._sequence, message))
            if "WRIST_READY" in message:
                self._state["phase"] = "wrist_ready"
            elif "Connected to the robot" in message:
                self._state["phase"] = "robot_connected"
            elif "等待脚踏板" in message:
                self._state["phase"] = "ready"
            if message.startswith(("错误:", "[wrist] WRIST_ERROR")):
                self._state["last_error"] = _primary_error(
                    str(self._state["last_error"]), message
                )
            status = STATUS_RE.search(message)
            if status:
                self._state.update(
                    mode=status.group(1),
                    ready=bool(int(status.group(2))),
                    clutch_pressed=bool(int(status.group(3))),
                    enabled=bool(int(status.group(4))),
                )

    def _read_process(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        try:
            for line in process.stdout:
                self._append(line)
        finally:
            return_code = process.wait()
            with self._lock:
                if self._process is process:
                    self._state.update(
                        running=False,
                        pid=None,
                        exit_code=return_code,
                        phase="stopped",
                        enabled=False,
                        clutch_pressed=False,
                    )
            self._append(f"[UI] Task process exited with code {return_code}")

    def start(self, config_path: Path) -> dict[str, Any]:
        return self.start_task("teleop", config_path)

    def start_task(self, task: str, config_path: Path) -> dict[str, Any]:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("Another task is already running")
            allowed = {
                "teleop", "demo7", "demo9", "set-zero", "go-zero", "home",
                "identify-inertia", "dynamic-inertia", "check-tool", "apply-pid",
                "build", "test",
            }
            if task not in allowed:
                raise ValueError(f"unknown task: {task}")
            if task == "teleop":
                command = [
                    str(FREEDRIVE_VENV / "bin" / "python"),
                    str(TELEOP_SCRIPT),
                    "--config", str(config_path), "--ui-control-stdin",
                ]
            else:
                command = [
                    str(FREEDRIVE_VENV / "bin" / "python"),
                    str(PROJECT_ROOT / "run.py"), task,
                ]
            access_message = "No fdcanusb access required for this task"
            if task not in {"check-tool", "build", "test"}:
                command, access_message = _teleop_command_with_fdcanusb_access(
                    command, config_path
                )
            environment = os.environ.copy()
            environment["PYTHONUNBUFFERED"] = "1"
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=environment,
                start_new_session=True,
            )
            self._process = process
            self._telemetry.clear()
            self._state.update(
                running=True,
                pid=process.pid,
                exit_code=None,
                phase="starting",
                mode="waiting",
                requested_mode=None,
                ready=False,
                clutch_pressed=False,
                enabled=False,
                last_error="",
                task=task,
            )
            self._append(f"[UI] {access_message}")
            self._append(f"[UI] Started task: {task}")
            self._reader = threading.Thread(
                target=self._read_process, args=(process,), daemon=True
            )
            self._reader.start()
            return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                raise RuntimeError("Teleoperation is not running")
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError as error:
                raise RuntimeError("Teleoperation process already exited") from error
            self._state["phase"] = "stopping"
            self._append("[UI] Ctrl-C sent; waiting for Flexiv Stop, wrist STOP, and zero haptic force")
            return self.status()

    def select_mode(self, mode: int) -> dict[str, Any]:
        if mode not in MODE_NAMES:
            raise ValueError("mode must be 1, 2, or 3")
        with self._lock:
            process = self._process
            if (
                process is None or process.poll() is not None
                or process.stdin is None or self._state.get("task") != "teleop"
            ):
                raise RuntimeError("Teleoperation is not running")
            process.stdin.write(f"MODE {mode}\n")
            process.stdin.flush()
            self._state["requested_mode"] = MODE_NAMES[mode]
            self._append(
                f"[UI] Requested Mode {mode} ({MODE_NAMES[mode]}); "
                "if the clutch is pressed, release and press it again"
            )
            return self.status()

    def status(
        self, since: int = 0, telemetry_since: int = 0
    ) -> dict[str, Any]:
        with self._lock:
            state = dict(self._state)
            state["last_sequence"] = self._sequence
            state["logs"] = [
                {"sequence": sequence, "line": line}
                for sequence, line in self._logs
                if sequence > since
            ]
            state["last_telemetry_sequence"] = self._telemetry_sequence
            state["telemetry"] = [
                {"sequence": sequence, "sample": sample}
                for sequence, sample in self._telemetry
                if sequence > telemetry_since
            ]
            return state

    def shutdown(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


class ConfigStore:
    def __init__(self, path: Path) -> None:
        self.path = path.resolve()
        self._lock = threading.Lock()

    def read(self) -> dict[str, Any]:
        with self._lock:
            text = self.path.read_text(encoding="utf-8")
            document = tomllib.loads(text)
            return {
                "path": str(self.path),
                "config": document,
                "comments": extract_toml_comments(text),
                "modified_ns": self.path.stat().st_mtime_ns,
            }

    def save(self, candidate: Any) -> dict[str, Any]:
        with self._lock:
            original = self.path.read_text(encoding="utf-8")
            original_mode = stat.S_IMODE(self.path.stat().st_mode)
            template = tomllib.loads(original)
            normalized = normalize_config_document(candidate, template)
            rewritten = rewrite_toml_document(original, normalized)
            if tomllib.loads(rewritten) != normalized:
                raise ValueError("TOML verification differed after serialization")

            descriptor, temporary_name = tempfile.mkstemp(
                prefix=".nine_dof_ui_",
                suffix=".toml",
                dir=self.path.parent,
                text=True,
            )
            temporary = Path(temporary_name)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    stream.write(rewritten)
                    stream.flush()
                    os.fsync(stream.fileno())
                check = subprocess.run(
                    [
                        str(FREEDRIVE_VENV / "bin" / "python"),
                        str(TELEOP_SCRIPT),
                        "--config",
                        str(temporary),
                        "--check-config",
                    ],
                    cwd=PROJECT_ROOT,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=20.0,
                    check=False,
                )
                if check.returncode != 0:
                    raise ValueError("Configuration validation failed:\n" + check.stdout.strip())
                backup = self.path.with_name(self.path.stem + ".ui-backup.toml")
                shutil.copy2(self.path, backup)
                os.chmod(temporary, original_mode)
                os.replace(temporary, self.path)
                return {
                    "ok": True,
                    "path": str(self.path),
                    "backup": str(backup),
                    "validation": check.stdout.strip(),
                }
            finally:
                if temporary.exists():
                    temporary.unlink()


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Flexiv 7/9-DoF Control Panel</title>
  <style>
    :root { color-scheme: dark; --bg:#0b1020; --panel:#131a2d; --panel2:#192238;
      --line:#2a3858; --text:#e9eefb; --muted:#95a3c2; --blue:#53a8ff;
      --green:#45d483; --orange:#ffb454; --red:#ff667a; }
    * { box-sizing:border-box; }
    body { margin:0; background:radial-gradient(circle at top,#16213c 0,var(--bg) 42%);
      color:var(--text); font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }
    header { position:sticky; top:0; z-index:10; display:flex; align-items:center;
      justify-content:space-between; gap:16px; padding:14px 24px; background:#0b1020e8;
      border-bottom:1px solid var(--line); backdrop-filter:blur(12px); }
    h1 { font-size:19px; margin:0; letter-spacing:.2px; }
    h2 { margin:0 0 12px; font-size:16px; }
    .layout { display:grid; grid-template-columns:minmax(420px,1.15fr) minmax(390px,.85fr);
      gap:16px; padding:16px; max-width:1700px; margin:auto; }
    .panel { background:var(--panel); border:1px solid var(--line); border-radius:14px;
      box-shadow:0 12px 32px #0005; overflow:hidden; }
    .panel-head { padding:14px 16px; border-bottom:1px solid var(--line);
      display:flex; align-items:center; justify-content:space-between; gap:12px; }
    .panel-body { padding:14px 16px; }
    .control-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }
    button { border:1px solid var(--line); background:var(--panel2); color:var(--text);
      border-radius:9px; padding:10px 13px; cursor:pointer; font-weight:650; }
    button:hover:not(:disabled) { border-color:var(--blue); transform:translateY(-1px); }
    button:disabled { opacity:.42; cursor:not-allowed; }
    button.primary { background:#1769aa; border-color:#2f91df; }
    button.stop { background:#7e2534; border-color:#b74354; }
    button.mode.active { outline:2px solid var(--green); border-color:var(--green); }
    .chips { display:flex; flex-wrap:wrap; gap:7px; }
    .chip { border:1px solid var(--line); background:#0d1426; border-radius:99px;
      padding:5px 9px; color:var(--muted); }
    .chip.on { color:#08140d; background:var(--green); border-color:var(--green); }
    .chip.warn { color:#211606; background:var(--orange); border-color:var(--orange); }
    .notice { margin-top:12px; padding:10px 12px; border-left:3px solid var(--orange);
      background:#2a2118; color:#ffd7a1; border-radius:6px; }
    #log { height:420px; margin:0; overflow:auto; white-space:pre-wrap; word-break:break-word;
      padding:12px; background:#070b14; color:#c8d4ec; font:12px/1.45 ui-monospace,monospace; }
    .config-sections { max-height:min(680px,calc(100vh - 180px)); overflow:auto; padding:12px; }
    details { background:#10172a; border:1px solid var(--line); border-radius:10px; margin-bottom:9px; }
    summary { padding:11px 13px; cursor:pointer; font-weight:700; color:#cfe4ff; }
    .fields { padding:0 12px 12px; display:grid; gap:10px; }
    .field { display:grid; grid-template-columns:minmax(190px,.7fr) minmax(220px,1.3fr);
      gap:10px; align-items:start; padding-top:9px; border-top:1px solid #26324c; }
    .field label { font-family:ui-monospace,monospace; color:#bdd4f5; }
    .desc { color:var(--muted); font:12px/1.35 system-ui,sans-serif; margin-top:3px; }
    input[type=text],input[type=number],textarea { width:100%; border:1px solid #344463;
      background:#090f1d; color:var(--text); border-radius:7px; padding:8px 9px;
      font:13px ui-monospace,monospace; }
    input:focus,textarea:focus { outline:2px solid #397fbd; border-color:transparent; }
    .switch { width:20px; height:20px; accent-color:var(--green); }
    .toolbar { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
    .small { font-size:12px; color:var(--muted); }
    .action { min-height:20px; margin-top:10px; color:#b9cdf1; font-weight:600; }
    .error { color:#ff9aaa; }
    .telemetry-panel { grid-column:1/-1; }
    .plot-toolbar { display:flex; gap:9px; align-items:center; flex-wrap:wrap; }
    .plot-toolbar select { border:1px solid #344463; background:#090f1d; color:var(--text);
      border-radius:7px; padding:7px 9px; }
    .telemetry-note { padding:11px 16px; color:var(--muted); border-bottom:1px solid var(--line); }
    .force-plots { display:grid; grid-template-columns:1fr; gap:12px;
      padding:14px 14px 7px; }
    .summary-plots { display:grid; grid-template-columns:repeat(2,minmax(360px,1fr)); gap:12px;
      padding:7px 14px; }
    .joint-plots { display:grid; grid-template-columns:repeat(3,minmax(280px,1fr)); gap:12px;
      padding:7px 14px 14px; }
    .plot-card { background:#090f1d; border:1px solid #2b3957; border-radius:10px; overflow:hidden; }
    .plot-card.important { border-color:#8a6334; box-shadow:inset 0 0 0 1px #5a4025; }
    .plot-title { display:flex; justify-content:space-between; gap:8px; align-items:baseline;
      padding:9px 11px 4px; font-weight:700; color:#dce9ff; }
    .plot-latest { color:var(--orange); font:12px ui-monospace,monospace; }
    .plot-legend { min-height:28px; display:flex; flex-wrap:wrap; gap:4px 10px; padding:0 11px 5px;
      color:var(--muted); font:11px ui-monospace,monospace; }
    .legend-item::before { content:'—'; color:var(--series-color); font-weight:900; margin-right:3px; }
    .plot-card canvas { display:block; width:100%; height:210px; background:#070b14; }
    .force-plots .plot-card { border-color:#427bb2; box-shadow:inset 0 0 0 1px #294f77; }
    .force-plots .plot-card canvas { height:320px; }
    @media (max-width:980px) { .layout{grid-template-columns:1fr}.config-sections{max-height:none}
      .field{grid-template-columns:1fr}.force-plots,.summary-plots,.joint-plots{grid-template-columns:1fr} }
  </style>
</head>
<body>
<header>
  <div><h1>Flexiv 7/9-DoF Control Panel</h1><div class="small">Localhost only · Missions, tools, settings, pedals, and live logs</div></div>
  <div class="chips"><span id="runChip" class="chip">Stopped</span><span id="phaseChip" class="chip">stopped</span><span id="modeChip" class="chip">waiting</span></div>
</header>
<main class="layout">
  <section class="panel">
    <div class="panel-head"><h2>Configuration</h2><div class="toolbar"><span id="saveState" class="small"></span><button id="reloadBtn">Reload</button><button id="saveBtn" class="primary">Save Configuration</button></div></div>
    <div id="configSections" class="config-sections">Loading…</div>
  </section>
  <div style="display:grid;gap:16px;align-content:start">
    <section class="panel">
      <div class="panel-head"><h2>Run Control</h2><span id="pidText" class="small"></span></div>
      <div class="panel-body">
        <div class="control-grid">
          <button id="startBtn" class="primary">Start Unified Teleoperation</button>
          <button id="stopBtn" class="stop" disabled>Stop (Ctrl-C)</button>
          <button id="clearBtn">Clear Log View</button>
        </div>
        <div style="height:12px"></div>
        <div class="small">Missions and tools (configuration is saved and validated before launch)</div>
        <div class="control-grid" id="taskGrid" style="margin-top:8px">
          <button class="task" data-task="demo7">7DoF Demo</button>
          <button class="task" data-task="demo9">9DoF Demo</button>
          <button class="task" data-task="go-zero">Wrist Go Zero</button>
          <button class="task" data-task="set-zero">Set Current Zero</button>
          <button class="task" data-task="home">Whole System Home</button>
          <button class="task" data-task="identify-inertia">Identify Inertia</button>
          <button class="task" data-task="dynamic-inertia">Dynamic Matrices</button>
          <button class="task" data-task="check-tool">Check Flexiv Tool</button>
          <button class="task" data-task="apply-pid">Apply Motor PID</button>
          <button class="task" data-task="build">Build RT Controllers</button>
          <button class="task" data-task="test">Run Unit Tests</button>
        </div>
        <div style="height:12px"></div>
        <div class="control-grid">
          <button class="mode" data-mode="1" disabled>Mode 1<br><span class="small">7DoF</span></button>
          <button class="mode" data-mode="2" disabled>Mode 2<br><span class="small">9DoF</span></button>
          <button class="mode" data-mode="3" disabled>Mode 3<br><span class="small">Orientation + Fz</span></button>
        </div>
        <div class="notice">
          <label><input id="startAcknowledgement" type="checkbox">
            I confirm that the robot workspace is clear, the active Tool is correct, and the emergency stop is within reach.
          </label>
          <div style="margin-top:6px">If the clutch is pressed while switching modes, release it and press it again to enable motion.</div>
          <div style="margin-top:6px">Mode 3: establish a light same-direction contact on a non-human test surface before pressing the clutch. The Tool-Z force ramps gently after contact is detected.</div>
        </div>
        <div id="runActionText" class="action"></div>
        <div id="errorText" class="error" style="margin-top:10px"></div>
      </div>
    </section>
    <section class="panel">
      <div class="panel-head"><h2>Live Log</h2><label class="small"><input id="autoScroll" type="checkbox" checked> Auto-scroll</label></div>
      <pre id="log"></pre>
    </section>
  </div>
  <section class="panel telemetry-panel" data-plot-count="14">
    <div class="panel-head">
      <div><h2 style="margin:0">Real-Time Tracking Plots</h2><div class="small">14 plots · large Mode 3 force chart · physical probe TCP · actual vs planned · joint tracking</div></div>
      <div class="plot-toolbar">
        <span id="telemetryState" class="small">Waiting for telemetry</span>
        <label class="small">Window
          <select id="plotWindow"><option value="15">15 s</option><option value="30" selected>30 s</option><option value="60">60 s</option></select>
        </label>
        <button id="clearPlotsBtn">Clear Plots</button>
      </div>
    </div>
    <div class="telemetry-note">
      Mode 3 force: measured = configured Flexiv wrench projected onto live probe Z; estimated = the low-pass value used by the controller (not an independent Bota ground truth). J1–J7 error = Flexiv null-space reference − measured joint angle. J8–J9 error = commanded wrist position − measured angle. Position and orientation errors are direct physical probe TCP task errors.
    </div>
    <div id="forcePlots" class="force-plots"></div>
    <div id="summaryPlots" class="summary-plots"></div>
    <div id="jointPlots" class="joint-plots"></div>
  </section>
</main>
<script>
const TOKEN = __TOKEN__;
let configDocument = null;
let comments = {};
let lastSequence = 0;
let dirty = false;
let renderingConfig = false;
let startBusy = false;
let lastTelemetrySequence = 0;
let telemetryHistory = [];
let plotWindowSeconds = 30;

const COLORS = {x:'#ff667a',y:'#45d483',z:'#53a8ff',norm:'#ffb454',actual:'#9bc7ff',target:'#f6c86b'};
const axisNames = ['X','Y','Z'];
const plotDefinitions = [
  {id:'mode3Force',group:'force',height:320,title:'Mode 3 Tool-Z Force — Real-Time Measured vs Estimated',unit:'N',important:true,
    emptyMessage:'Mode 3 force data appears after Mode 3 is selected',
    headline:s=>s.mode==='pivot_orientation'?`measured ${s.force_measured_tool_z_n.toFixed(2)} N · estimated ${s.force_estimated_tool_z_n.toFixed(2)} N · ${s.force_control_active?'force axis active':'position hold'}`:'Mode 3 inactive',series:[
      {label:'real-time measured',color:'#ffffff',value:s=>s.mode==='pivot_orientation'?s.force_measured_tool_z_n:NaN},
      {label:'controller estimate (LPF)',color:COLORS.z,value:s=>s.mode==='pivot_orientation'?s.force_estimated_tool_z_n:NaN},
      {label:'target sensed force',color:'#45d483',dash:true,value:s=>s.mode==='pivot_orientation'?s.force_target_tool_z_n:NaN},
      {label:'ramped command',color:COLORS.norm,dash:true,value:s=>s.mode==='pivot_orientation'?s.force_command_tool_z_n:NaN}]},
  {id:'tcpPosition',group:'summary',title:'TCP Position — Actual vs Planned',unit:'mm',series:[
    ...axisNames.map((name,i)=>({label:`${name} actual`,color:[COLORS.x,COLORS.y,COLORS.z][i],value:s=>s.position_actual_m[i]*1000})),
    ...axisNames.map((name,i)=>({label:`${name} planned`,color:[COLORS.x,COLORS.y,COLORS.z][i],dash:true,value:s=>s.position_target_m[i]*1000}))]},
  {id:'tcpOrientation',group:'summary',title:'TCP Orientation — Actual vs Planned',unit:'deg rotvec',series:[
    ...axisNames.map((name,i)=>({label:`R${name} actual`,color:[COLORS.x,COLORS.y,COLORS.z][i],value:s=>s.orientation_actual_rotvec_deg[i]})),
    ...axisNames.map((name,i)=>({label:`R${name} planned`,color:[COLORS.x,COLORS.y,COLORS.z][i],dash:true,value:s=>s.orientation_target_rotvec_deg[i]}))]},
  {id:'positionError',group:'summary',title:'TCP Position Error',unit:'mm',important:true,symmetric:true,headline:s=>`${s.position_error_norm_mm.toFixed(2)} mm norm`,series:[
    ...axisNames.map((name,i)=>({label:`e${name}`,color:[COLORS.x,COLORS.y,COLORS.z][i],value:s=>s.position_error_mm[i]})),
    {label:'norm',color:COLORS.norm,value:s=>s.position_error_norm_mm}]},
  {id:'orientationError',group:'summary',title:'TCP Orientation Error',unit:'deg',important:true,symmetric:true,headline:s=>`${s.orientation_error_norm_deg.toFixed(2)}° norm`,series:[
    ...axisNames.map((name,i)=>({label:`eR${name}`,color:[COLORS.x,COLORS.y,COLORS.z][i],value:s=>s.orientation_error_rotvec_deg[i]})),
    {label:'norm',color:COLORS.norm,value:s=>s.orientation_error_norm_deg}]},
  ...Array.from({length:9},(_,i)=>({id:`jointError${i+1}`,group:'joint',title:`Joint ${i+1} Error`,unit:'deg',symmetric:true,
    headline:s=>`${s.joint_error_deg[i].toFixed(2)}°`,series:[{label:`q${i+1} target − actual`,color:i<7?COLORS.actual:COLORS.target,value:s=>s.joint_error_deg[i]}]}))
];

function initializePlots() {
  const force=document.getElementById('forcePlots'); const summary=document.getElementById('summaryPlots'); const joints=document.getElementById('jointPlots');
  plotDefinitions.forEach((definition,index)=>{
    const card=document.createElement('div'); card.className='plot-card'+(definition.important?' important':''); card.dataset.plot=definition.id;
    const title=document.createElement('div'); title.className='plot-title';
    const label=document.createElement('span'); label.textContent=definition.title;
    const latest=document.createElement('span'); latest.className='plot-latest'; latest.textContent='—';
    title.append(label,latest);
    const legend=document.createElement('div'); legend.className='plot-legend';
    definition.series.forEach(series=>{const item=document.createElement('span');item.className='legend-item';item.style.setProperty('--series-color',series.color);item.textContent=series.label;legend.appendChild(item);});
    const canvas=document.createElement('canvas'); canvas.setAttribute('aria-label',definition.title);
    card.append(title,legend,canvas);
    (definition.group==='force'?force:(definition.group==='summary'?summary:joints)).appendChild(card);
  });
  renderPlots();
}

function clearPlotHistory() {
  telemetryHistory=[];
  document.getElementById('telemetryState').textContent='Waiting for telemetry';
  renderPlots();
}

function ingestTelemetry(items) {
  for(const item of items || []) {
    const sample=item.sample;
    if(!sample || !Number.isFinite(sample.timestamp_s)) continue;
    const previous=telemetryHistory[telemetryHistory.length-1];
    if(previous && sample.timestamp_s+0.5<previous.timestamp_s) telemetryHistory=[];
    telemetryHistory.push(sample);
  }
  if(telemetryHistory.length) {
    const newest=telemetryHistory[telemetryHistory.length-1];
    const cutoff=newest.timestamp_s-65;
    telemetryHistory=telemetryHistory.filter(sample=>sample.timestamp_s>=cutoff);
    document.getElementById('telemetryState').textContent=`${newest.mode} · ${newest.enabled?'enabled':'holding'} · ${newest.timestamp_s.toFixed(1)} s`;
  }
}

function drawPlot(definition,card,samples) {
  const canvas=card.querySelector('canvas'); const latest=card.querySelector('.plot-latest');
  const width=Math.max(300,canvas.clientWidth); const height=definition.height||210; const ratio=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);}
  const context=canvas.getContext('2d'); context.setTransform(ratio,0,0,ratio,0,0); context.clearRect(0,0,width,height); context.fillStyle='#070b14';context.fillRect(0,0,width,height);
  const left=54,right=12,top=12,bottom=27,plotWidth=width-left-right,plotHeight=height-top-bottom;
  if(!samples.length){context.fillStyle='#6f7e9c';context.font='12px system-ui';context.textAlign='center';context.fillText('Waiting for teleoperation telemetry',width/2,height/2);latest.textContent='—';return;}
  const newest=samples[samples.length-1]; const endTime=newest.timestamp_s; const startTime=endTime-plotWindowSeconds;
  const visible=samples.filter(sample=>sample.timestamp_s>=startTime);
  const values=[];
  definition.series.forEach(series=>visible.forEach(sample=>{const value=series.value(sample);if(Number.isFinite(value))values.push(value);}));
  if(!values.length){context.fillStyle='#6f7e9c';context.font='12px system-ui';context.textAlign='center';context.fillText(definition.emptyMessage||'No finite telemetry in this time window',width/2,height/2);latest.textContent='—';return;}
  let minimum=Math.min(...values),maximum=Math.max(...values);
  if(definition.symmetric){const bound=Math.max(Math.abs(minimum),Math.abs(maximum),definition.unit==='deg'?0.1:0.01);minimum=-bound;maximum=bound;}
  else {let padding=(maximum-minimum)*0.12;if(padding<1e-6)padding=Math.max(Math.abs(maximum)*0.02,0.01);minimum-=padding;maximum+=padding;}
  const ySpan=Math.max(maximum-minimum,1e-9); const xFor=t=>left+(t-startTime)/plotWindowSeconds*plotWidth; const yFor=value=>top+(maximum-value)/ySpan*plotHeight;
  context.lineWidth=1;context.font='10px ui-monospace,monospace';context.textAlign='right';context.textBaseline='middle';
  for(let index=0;index<=4;index++){const y=top+index*plotHeight/4;const value=maximum-index*ySpan/4;context.strokeStyle='#1c2740';context.beginPath();context.moveTo(left,y);context.lineTo(width-right,y);context.stroke();context.fillStyle='#7887a6';context.fillText(value.toFixed(Math.abs(value)<10?2:1),left-6,y);}
  context.textAlign='center';context.textBaseline='top';
  for(let index=0;index<=3;index++){const x=left+index*plotWidth/3;context.strokeStyle='#141e33';context.beginPath();context.moveTo(x,top);context.lineTo(x,height-bottom);context.stroke();context.fillStyle='#7887a6';context.fillText(`${(-plotWindowSeconds+index*plotWindowSeconds/3).toFixed(0)}s`,x,height-bottom+6);}
  context.save();context.translate(11,top+plotHeight/2);context.rotate(-Math.PI/2);context.fillStyle='#95a3c2';context.textAlign='center';context.textBaseline='top';context.fillText(definition.unit,0,0);context.restore();
  definition.series.forEach(series=>{context.strokeStyle=series.color;context.lineWidth=series.dash?1.3:1.8;context.setLineDash(series.dash?[5,4]:[]);context.beginPath();let started=false;visible.forEach(sample=>{const value=series.value(sample);if(!Number.isFinite(value))return;const x=xFor(sample.timestamp_s),y=yFor(value);if(!started){context.moveTo(x,y);started=true;}else context.lineTo(x,y);});if(started)context.stroke();});context.setLineDash([]);
  latest.textContent=definition.headline?definition.headline(newest):definition.series.slice(0,3).map(series=>`${series.label} ${series.value(newest).toFixed(2)}`).join(' · ');
}

function renderPlots() {
  const end=telemetryHistory.length?telemetryHistory[telemetryHistory.length-1].timestamp_s:0;
  const samples=telemetryHistory.filter(sample=>sample.timestamp_s>=end-plotWindowSeconds);
  plotDefinitions.forEach(definition=>{const card=document.querySelector(`[data-plot="${definition.id}"]`);if(card)drawPlot(definition,card,samples);});
}

async function api(path, options={}) {
  const headers = Object.assign({'X-Teleop-Token': TOKEN}, options.headers || {});
  if (options.body && typeof options.body !== 'string') {
    headers['Content-Type'] = 'application/json'; options.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, Object.assign({}, options, {headers}));
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function sectionTitle(name) {
  const titles={robot:'Flexiv Arm',haptic:'omega.7 Haptic Device',pedal:'Foot Pedals',wrist:'Wrist Motors',motor_pid:'Motor PID',demo:'Demo Trajectory',
    wrist_geometry:'Wrist Geometry',wrist_payload:'Wrist Payload',flexiv_tool:'Flexiv Tool',allocation:'9-DoF Allocation',
    osc_controller:'Torque OSC Controller',teleop_impedance:'Flexiv Impedance + Unified Mode 3',teleop_osc:'Experimental Torque OSC',pivot_orientation_osc:'Mode 3 Hybrid OSC',force_feedback:'Haptic Feedback',runtime:'Real-Motion Confirmation'};
  return `${titles[name] || name}  [${name}]`;
}

function fieldInput(section, key, value) {
  const input = document.createElement('input');
  input.className='config-input'; input.dataset.section=section; input.dataset.key=key;
  input.autocomplete='off';
  if (typeof value === 'boolean') { input.type='checkbox'; input.classList.add('switch'); input.checked=value; input.dataset.kind='bool'; }
  else if (typeof value === 'number') { input.type='number'; input.step='any'; input.value=String(value); input.dataset.kind=Number.isInteger(value)?'int':'float'; }
  else { input.type='text'; input.value=Array.isArray(value)?JSON.stringify(value):value; input.dataset.kind=Array.isArray(value)?'array':'string'; }
  input.addEventListener('input',()=>{if(!renderingConfig){dirty=true;document.getElementById('saveState').textContent='Unsaved changes';}});
  input.addEventListener('change',()=>{if(!renderingConfig){dirty=true;document.getElementById('saveState').textContent='Unsaved changes';}});
  return input;
}

function renderConfig() {
  renderingConfig=true;
  const root=document.getElementById('configSections'); root.textContent='';
  Object.entries(configDocument).forEach(([section,values], index)=>{
    const details=document.createElement('details'); details.open=index<3 || ['pivot_orientation_osc','force_feedback','runtime'].includes(section);
    const summary=document.createElement('summary'); summary.textContent=sectionTitle(section); details.appendChild(summary);
    const fields=document.createElement('div'); fields.className='fields';
    Object.entries(values).forEach(([key,value])=>{
      const row=document.createElement('div'); row.className='field';
      const info=document.createElement('div'); const label=document.createElement('label'); label.textContent=key; info.appendChild(label);
      const description=comments[`${section}.${key}`]; if(description){const d=document.createElement('div');d.className='desc';d.textContent=description;info.appendChild(d);}
      row.appendChild(info); row.appendChild(fieldInput(section,key,value)); fields.appendChild(row);
    });
    details.appendChild(fields); root.appendChild(details);
  });
  setTimeout(()=>{renderingConfig=false;},0);
}

async function loadConfig() {
  const data=await api('/api/config'); configDocument=data.config; comments=data.comments || {}; renderConfig();
  dirty=false; document.getElementById('saveState').textContent=`Loaded ${data.path}`;
}

function collectConfig() {
  const result=JSON.parse(JSON.stringify(configDocument));
  document.querySelectorAll('.config-input').forEach(input=>{
    let value;
    if(input.dataset.kind==='bool') value=input.checked;
    else if(input.dataset.kind==='int') { value=Number(input.value); if(!Number.isInteger(value)) throw new Error(`${input.dataset.section}.${input.dataset.key} must be an integer`); }
    else if(input.dataset.kind==='float') { value=Number(input.value); if(!Number.isFinite(value)) throw new Error(`${input.dataset.section}.${input.dataset.key} must be a finite number`); }
    else if(input.dataset.kind==='array') { try{value=JSON.parse(input.value);}catch(e){throw new Error(`${input.dataset.section}.${input.dataset.key} has invalid array syntax`);} if(!Array.isArray(value)) throw new Error(`${input.dataset.section}.${input.dataset.key} must be an array`); }
    else value=input.value;
    result[input.dataset.section][input.dataset.key]=value;
  });
  return result;
}

async function saveConfig() {
  const button=document.getElementById('saveBtn'); button.disabled=true;
  try {
    const candidate=collectConfig(); const data=await api('/api/config/save',{method:'POST',body:{config:candidate}});
    configDocument=candidate; dirty=false; document.getElementById('saveState').textContent=`Saved; backup: ${data.backup}`;
    document.getElementById('errorText').textContent=''; return data;
  } finally { button.disabled=false; }
}

async function startTask(task='teleop') {
  const acknowledgement=document.getElementById('startAcknowledgement');
  const action=document.getElementById('runActionText');
  const errorText=document.getElementById('errorText');
  if(!acknowledgement.checked){errorText.textContent='Check the safety confirmation before starting.';action.textContent='Start cancelled.';return;}
  startBusy=true; updateStartButton(false); errorText.textContent='';
  try {
    clearPlotHistory();
    action.textContent='Saving and validating configuration…';
    await saveConfig();
    action.textContent=`Starting ${task}…`;
    const state=await api('/api/task/start',{method:'POST',body:{task}});
    updateStatus(state); action.textContent=`Task ${task} started.`;
  } catch(error) {
    errorText.textContent=error.message; action.textContent='Start failed. See the error below.';
  } finally { startBusy=false; updateStartButton(false); }
}
async function stopTeleop(){try{await api('/api/teleop/stop',{method:'POST',body:{}});}catch(error){document.getElementById('errorText').textContent=error.message;}}
async function selectMode(mode){try{await api('/api/teleop/mode',{method:'POST',body:{mode:Number(mode)}});}catch(error){document.getElementById('errorText').textContent=error.message;}}

function updateStartButton(running) {
  document.getElementById('startBtn').disabled=running || startBusy;
  document.querySelectorAll('.task').forEach(button=>button.disabled=running || startBusy);
}

function updateStatus(state) {
  const running=state.running; const runChip=document.getElementById('runChip'); runChip.textContent=running?'Running':'Stopped'; runChip.className='chip '+(running?'on':'');
  document.getElementById('phaseChip').textContent=state.phase; document.getElementById('modeChip').textContent=state.mode;
  document.getElementById('pidText').textContent=state.pid?`PID ${state.pid}`:(state.exit_code===null?'':`Exit code ${state.exit_code}`);
  updateStartButton(running); document.getElementById('stopBtn').disabled=!running;
  document.querySelectorAll('.mode').forEach(button=>{button.disabled=!running || state.task!=='teleop';button.classList.toggle('active',state.mode===({1:'7dof',2:'9dof',3:'pivot_orientation'})[button.dataset.mode]);});
  if(state.last_error) document.getElementById('errorText').textContent=state.last_error;
  if(state.logs && state.logs.length){const log=document.getElementById('log'); log.textContent+=state.logs.map(item=>item.line).join('\n')+'\n'; if(log.textContent.length>400000)log.textContent=log.textContent.slice(-300000); if(document.getElementById('autoScroll').checked)log.scrollTop=log.scrollHeight; lastSequence=state.last_sequence;}
  if(state.telemetry && state.telemetry.length){ingestTelemetry(state.telemetry);renderPlots();}
  if(Number.isInteger(state.last_telemetry_sequence))lastTelemetrySequence=state.last_telemetry_sequence;
}

async function poll(){try{updateStatus(await api(`/api/status?since=${lastSequence}&telemetry_since=${lastTelemetrySequence}`));}catch(error){document.getElementById('errorText').textContent=error.message;}finally{setTimeout(poll,300);}}
document.getElementById('saveBtn').addEventListener('click',()=>saveConfig().catch(e=>document.getElementById('errorText').textContent=e.message));
document.getElementById('reloadBtn').addEventListener('click',()=>{if(!dirty||confirm('Discard unsaved changes?'))loadConfig().catch(e=>document.getElementById('errorText').textContent=e.message);});
document.getElementById('startBtn').addEventListener('click',()=>startTask('teleop'));
document.querySelectorAll('.task').forEach(button=>button.addEventListener('click',()=>startTask(button.dataset.task)));
document.getElementById('stopBtn').addEventListener('click',stopTeleop);
document.getElementById('clearBtn').addEventListener('click',()=>document.getElementById('log').textContent='');
document.getElementById('clearPlotsBtn').addEventListener('click',clearPlotHistory);
document.getElementById('plotWindow').addEventListener('change',event=>{plotWindowSeconds=Number(event.target.value);renderPlots();});
document.querySelectorAll('.mode').forEach(button=>button.addEventListener('click',()=>selectMode(button.dataset.mode)));
window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue='';}});
window.addEventListener('resize',renderPlots);
initializePlots();
loadConfig().then(poll).catch(error=>document.getElementById('errorText').textContent=error.message);
</script>
</body>
</html>"""


def make_handler(
    manager: TeleopProcessManager, store: ConfigStore, token: str
) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "FlexivTeleopUI/1.0"

        def log_message(self, _format: str, *_args: object) -> None:
            return

        def _json(self, payload: Any, status: int = HTTPStatus.OK) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                return

        def _authorized(self) -> bool:
            if self.headers.get("X-Teleop-Token") == token:
                return True
            self._json({"error": "invalid local UI token"}, HTTPStatus.FORBIDDEN)
            return False

        def _body(self) -> Any:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1_000_000:
                raise ValueError("invalid request size")
            return json.loads(self.rfile.read(length))

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                body = INDEX_HTML.replace("__TOKEN__", json.dumps(token)).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if not self._authorized():
                return
            try:
                if parsed.path == "/api/config":
                    self._json(store.read())
                elif parsed.path == "/api/status":
                    query = parse_qs(parsed.query)
                    since = int(query.get("since", ["0"])[0])
                    telemetry_since = int(
                        query.get("telemetry_since", ["0"])[0]
                    )
                    self._json(
                        manager.status(
                            max(0, since), max(0, telemetry_since)
                        )
                    )
                else:
                    self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except Exception as error:
                self._json({"error": str(error)}, HTTPStatus.BAD_REQUEST)

        def do_POST(self) -> None:  # noqa: N802
            if not self._authorized():
                return
            try:
                body = self._body()
                if self.path == "/api/config/save":
                    self._json(store.save(body.get("config")))
                elif self.path in {"/api/teleop/start", "/api/task/start"}:
                    task = "teleop" if self.path == "/api/teleop/start" else str(body.get("task"))
                    self._json(manager.start_task(task, store.path))
                elif self.path == "/api/teleop/stop":
                    self._json(manager.stop())
                elif self.path == "/api/teleop/mode":
                    self._json(manager.select_mode(int(body.get("mode"))))
                else:
                    self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (ValueError, RuntimeError, subprocess.SubprocessError) as error:
                self._json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            except Exception as error:
                self._json({"error": f"{type(error).__name__}: {error}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not args.config.is_file():
        raise FileNotFoundError(f"configuration file not found: {args.config}")
    if not 1024 <= args.port <= 65535:
        raise ValueError("--port must be in 1024..65535")

    manager = TeleopProcessManager()
    store = ConfigStore(args.config)
    token = secrets.token_urlsafe(24)
    server = ThreadingHTTPServer(
        ("127.0.0.1", args.port), make_handler(manager, store, token)
    )
    server.daemon_threads = True
    url = f"http://127.0.0.1:{args.port}/"
    print(f"Flexiv Control Panel: {url}")
    print("Press Ctrl-C in this terminal to close the UI; any active task will be stopped first.")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping the active task and UI server...")
    finally:
        server.shutdown()
        server.server_close()
        manager.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
