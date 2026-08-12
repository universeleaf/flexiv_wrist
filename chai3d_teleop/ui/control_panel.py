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

    def status(self, since: int = 0) -> dict[str, Any]:
        with self._lock:
            state = dict(self._state)
            state["last_sequence"] = self._sequence
            state["logs"] = [
                {"sequence": sequence, "line": line}
                for sequence, line in self._logs
                if sequence > since
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
    #log { height:480px; margin:0; overflow:auto; white-space:pre-wrap; word-break:break-word;
      padding:12px; background:#070b14; color:#c8d4ec; font:12px/1.45 ui-monospace,monospace; }
    .config-sections { max-height:calc(100vh - 180px); overflow:auto; padding:12px; }
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
    @media (max-width:980px) { .layout{grid-template-columns:1fr}.config-sections{max-height:none}
      .field{grid-template-columns:1fr} }
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
          <button id="startBtn" class="primary">Start Teleoperation</button>
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
</main>
<script>
const TOKEN = __TOKEN__;
let configDocument = null;
let comments = {};
let lastSequence = 0;
let dirty = false;
let renderingConfig = false;
let startBusy = false;

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
    osc_controller:'Torque OSC Controller',pivot_orientation_osc:'Mode 3 OSC',force_feedback:'Haptic Feedback',runtime:'Real-Motion Confirmation'};
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
}

async function poll(){try{updateStatus(await api(`/api/status?since=${lastSequence}`));}catch(error){document.getElementById('errorText').textContent=error.message;}finally{setTimeout(poll,300);}}
document.getElementById('saveBtn').addEventListener('click',()=>saveConfig().catch(e=>document.getElementById('errorText').textContent=e.message));
document.getElementById('reloadBtn').addEventListener('click',()=>{if(!dirty||confirm('Discard unsaved changes?'))loadConfig().catch(e=>document.getElementById('errorText').textContent=e.message);});
document.getElementById('startBtn').addEventListener('click',()=>startTask('teleop'));
document.querySelectorAll('.task').forEach(button=>button.addEventListener('click',()=>startTask(button.dataset.task)));
document.getElementById('stopBtn').addEventListener('click',stopTeleop);
document.getElementById('clearBtn').addEventListener('click',()=>document.getElementById('log').textContent='');
document.querySelectorAll('.mode').forEach(button=>button.addEventListener('click',()=>selectMode(button.dataset.mode)));
window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue='';}});
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
                    since = int(parse_qs(parsed.query).get("since", ["0"])[0])
                    self._json(manager.status(max(0, since)))
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
