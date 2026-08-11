"use strict";

const $ = (id) => document.getElementById(id);
const AXIS_COLORS = ["#49d9df", "#ff9e52", "#5d8dff"];
const history = [];
const offsets = {
  tcp: [0, 0, 0, 0, 0, 0],
  world: [0, 0, 0, 0, 0, 0],
  sensor: [0, 0, 0, 0, 0, 0],
};
let latestSample = null;
let latestStatus = null;
let recording = false;
let stopping = false;
let recordingStartedMs = 0;
let recordingSession = "";
let recordingOutputDirectory = "";
let recordedSamples = [];
let toastTimer = null;

function showToast(message, error = false) {
  const node = $("toast");
  node.textContent = message;
  node.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { node.className = "toast"; }, 5000);
}

function setPill(node, type, text) {
  node.classList.remove("ok", "error", "recording");
  if (type) node.classList.add(type);
  node.querySelector("span").textContent = text;
}

function updateClock() {
  $("clock").textContent = new Date().toLocaleTimeString("en-GB", { hour12: false });
  if (recording) {
    const elapsed = Math.max(0, Date.now() - recordingStartedMs);
    const minutes = Math.floor(elapsed / 60000);
    const seconds = (elapsed % 60000) / 1000;
    $("timerValue").textContent = `${String(minutes).padStart(2, "0")}:${seconds.toFixed(1).padStart(4, "0")}`;
  }
}
setInterval(updateClock, 100);
updateClock();

function updateStatus(status) {
  latestStatus = status;
  if (status.fault) setPill($("robotStatus"), "error", "Robot fault");
  else if (status.connected) setPill($("robotStatus"), "ok", status.phase === "demo" ? "Robot demo data" : "Robot data online");
  else if (status.phase === "error") setPill($("robotStatus"), "error", "Robot connection failed");
  else setPill($("robotStatus"), "", "Connecting to robot");

  $("serviceMessage").textContent = status.error
    ? `${status.message}: ${status.error}`
    : (status.message || "This UI monitors only and does not control EmJ.");
  $("robotIdentity").textContent = status.robot_sn || "Rizon4s-123456";
  $("robotMode").textContent = status.mode || "—";
  $("robotOperational").textContent = status.operational ? "READY" : "NOT READY";
  $("robotBusy").textContent = status.busy ? "RUNNING / BUSY" : "IDLE";
  $("assignedPlan").textContent = status.plan?.assigned_plan_name || "—";
  $("sampleRate").textContent = status.sample_rate_hz ? `${status.sample_rate_hz} Hz` : "—";
}

function adjustedWrench(sample) {
  const source = $("sourceSelect").value;
  const raw = sample?.wrench?.[source] || [0, 0, 0, 0, 0, 0];
  return raw.map((value, index) => Number(value) - offsets[source][index]);
}

function updateMetrics(sample) {
  const values = adjustedWrench(sample);
  $("forceNorm").textContent = Math.hypot(...values.slice(0, 3)).toFixed(2);
  $("momentNorm").textContent = Math.hypot(...values.slice(3)).toFixed(3);
  ["fx", "fy", "fz"].forEach((id, index) => { $(id).textContent = values[index].toFixed(2); });
  ["mx", "my", "mz"].forEach((id, index) => { $(id).textContent = values[index + 3].toFixed(3); });
  const time = new Date(sample.host_unix_ms);
  $("sampleTimestamp").textContent = `PC ${time.toLocaleTimeString("en-GB", { hour12: false })}.${String(time.getMilliseconds()).padStart(3, "0")} · sample #${sample.sequence}`;
}

const events = new EventSource("/api/stream");
events.addEventListener("status", (event) => {
  try { updateStatus(JSON.parse(event.data)); } catch (error) { console.error(error); }
});
events.addEventListener("sample", (event) => {
  try {
    const sample = JSON.parse(event.data);
    latestSample = sample;
    history.push(sample);
    if (history.length > 12000) history.splice(0, history.length - 12000);
    if (recording) {
      recordedSamples.push({
        ...sample,
        capture_status: {
          mode: latestStatus?.mode || "unknown",
          operational: Boolean(latestStatus?.operational),
          busy: Boolean(latestStatus?.busy),
          fault: Boolean(latestStatus?.fault),
          assigned_plan: latestStatus?.plan?.assigned_plan_name || "",
        },
      });
    }
    updateMetrics(sample);
  } catch (error) { console.error(error); }
});
events.onerror = () => {
  if (!latestStatus?.connected) setPill($("robotStatus"), "error", "UI data stream interrupted");
};

$("sourceSelect").addEventListener("change", () => { if (latestSample) updateMetrics(latestSample); });
$("tareButton").addEventListener("click", () => {
  if (!latestSample) { showToast("No force sample is available for display tare.", true); return; }
  const source = $("sourceSelect").value;
  offsets[source] = [...latestSample.wrench[source]];
  updateMetrics(latestSample);
  showToast("Display tare applied. Saved CSV values remain raw and unchanged.");
});
$("clearButton").addEventListener("click", () => { history.length = 0; showToast("Live plots cleared."); });

function resizeCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  return { width, height, ratio };
}

function drawChart(canvas, componentOffset, minimumRange) {
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  const left = 44 * ratio, right = 10 * ratio, top = 10 * ratio, bottom = 22 * ratio;
  const plotWidth = width - left - right, plotHeight = height - top - bottom;
  ctx.clearRect(0, 0, width, height);
  const seconds = Number($("windowSelect").value);
  const endMs = latestSample?.host_unix_ms || Date.now();
  const startMs = endMs - seconds * 1000;
  const visible = history.filter((sample) => sample.host_unix_ms >= startMs);
  const source = $("sourceSelect").value;
  let peak = minimumRange;
  visible.forEach((sample) => {
    for (let axis = 0; axis < 3; axis += 1) {
      peak = Math.max(peak, Math.abs(sample.wrench[source][componentOffset + axis] - offsets[source][componentOffset + axis]));
    }
  });
  peak *= 1.12;
  ctx.lineWidth = ratio;
  ctx.strokeStyle = "#202a35";
  ctx.fillStyle = "#63707c";
  ctx.font = `${9 * ratio}px ui-monospace, monospace`;
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let row = 0; row <= 4; row += 1) {
    const y = top + plotHeight * row / 4;
    ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + plotWidth, y); ctx.stroke();
    ctx.fillText((peak * (1 - row / 2)).toFixed(componentOffset ? 2 : 1), left - 6 * ratio, y);
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let column = 0; column <= 4; column += 1) {
    const x = left + plotWidth * column / 4;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + plotHeight); ctx.stroke();
    ctx.fillText(`${(-seconds + seconds * column / 4).toFixed(1)}s`, x, top + plotHeight + 6 * ratio);
  }
  if (visible.length < 2) return;
  for (let axis = 0; axis < 3; axis += 1) {
    ctx.strokeStyle = AXIS_COLORS[axis];
    ctx.lineWidth = 1.5 * ratio;
    ctx.beginPath();
    visible.forEach((sample, index) => {
      const x = left + (sample.host_unix_ms - startMs) / (seconds * 1000) * plotWidth;
      const value = sample.wrench[source][componentOffset + axis] - offsets[source][componentOffset + axis];
      const y = top + (1 - (value + peak) / (2 * peak)) * plotHeight;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

function renderCharts() {
  drawChart($("forceChart"), 0, 5);
  drawChart($("momentChart"), 3, 0.5);
  requestAnimationFrame(renderCharts);
}
requestAnimationFrame(renderCharts);

function csvNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toPrecision(12) : "";
}

function csvText(value) { return `"${String(value ?? "").replaceAll('"', '""')}"`; }

function samplesToCsv(samples) {
  const columns = [
    "recording_index", "sequence", "host_iso", "host_unix_ns", "elapsed_s",
    "robot_seconds", "robot_nanoseconds", "robot_mode", "robot_operational", "robot_busy", "robot_fault", "assigned_plan",
    "tcp_force_norm_N", "tcp_moment_norm_Nm",
    "tcp_fx_N", "tcp_fy_N", "tcp_fz_N", "tcp_mx_Nm", "tcp_my_Nm", "tcp_mz_Nm",
    "world_fx_N", "world_fy_N", "world_fz_N", "world_mx_Nm", "world_my_Nm", "world_mz_Nm",
    "sensor_fx_N", "sensor_fy_N", "sensor_fz_N", "sensor_mx_Nm", "sensor_my_Nm", "sensor_mz_Nm",
    "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_qw", "tcp_qx", "tcp_qy", "tcp_qz",
    "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad", "q6_rad", "q7_rad",
  ];
  const rows = [columns.join(",")];
  samples.forEach((sample, index) => {
    const tcp = sample.wrench.tcp;
    const values = [
      index, sample.sequence, new Date(sample.host_unix_ms).toISOString(), String(sample.host_unix_ns),
      (sample.host_unix_ms - recordingStartedMs) / 1000,
      sample.robot_time.seconds, sample.robot_time.nanoseconds,
      sample.capture_status.mode, sample.capture_status.operational, sample.capture_status.busy,
      sample.capture_status.fault, sample.capture_status.assigned_plan,
      Math.hypot(...tcp.slice(0, 3)), Math.hypot(...tcp.slice(3)),
      ...tcp, ...sample.wrench.world, ...sample.wrench.sensor, ...sample.tcp_pose, ...sample.q,
    ];
    rows.push(values.map((value, column) => (
      [2, 3, 7, 11].includes(column) ? csvText(value) : csvNumber(value)
    )).join(","));
  });
  return `${rows.join("\n")}\n`;
}

async function requestRecording(path, body, contentType = "") {
  const headers = {};
  if (contentType) headers["Content-Type"] = contentType;
  if (path !== "/api/record/start") headers["X-Capture-Session"] = recordingSession;
  const response = await fetch(path, { method: "POST", headers, body });
  let payload = {};
  try { payload = await response.json(); } catch (_) { /* handled by HTTP status */ }
  if (!response.ok) throw new Error(payload.error || `recording service returned HTTP ${response.status}`);
  return payload;
}

function setRecordingUi(active) {
  $("startButton").classList.toggle("hidden", active);
  $("stopButton").classList.toggle("hidden", !active);
  $("timer").classList.toggle("recording", active);
  $("timerLabel").textContent = active ? "REC" : "SAVED";
  $("timerValue").textContent = active ? $("timerValue").textContent : "00:00.0";
  setPill($("captureStatus"), active ? "recording" : "ok", active ? "Capturing force" : "Capture ready");
}

async function startRecording() {
  if (recording || stopping) return;
  if (!latestSample) { showToast("No robot force data has been received; capture cannot start.", true); return; }
  $("startButton").disabled = true;
  try {
    recordingStartedMs = Date.now();
    recordedSamples = [];
    recordingSession = "";
    recordingOutputDirectory = "";
    const result = await requestRecording(
      "/api/record/start",
      JSON.stringify({ project_label: "EmJ", robot_sn: latestStatus?.robot_sn || "Rizon4s-123456" }),
      "application/json",
    );
    recordingSession = result.session;
    recordingOutputDirectory = result.output_directory;
    recording = true;
    setRecordingUi(true);
    $("outputSummary").textContent = recordingSession;
    $("outputSummary").title = recordingOutputDirectory;
    showToast(`Force capture started: ${recordingOutputDirectory}`);
  } catch (error) {
    showToast(`Unable to start force capture: ${error?.message || "unknown error"}`, true);
  } finally { $("startButton").disabled = false; }
}

async function stopRecording() {
  if (!recording || stopping) return;
  stopping = true;
  recording = false;
  $("stopButton").disabled = true;
  $("timerLabel").textContent = "SAVING";
  try {
    const csv = samplesToCsv(recordedSamples);
    await requestRecording("/api/record/force", new Blob([csv], { type: "text/csv;charset=utf-8" }), "text/csv;charset=utf-8");
    const result = await requestRecording("/api/record/finish", new Blob([]));
    setRecordingUi(false);
    $("outputSummary").textContent = `SAVED · ${recordingSession}`;
    $("outputSummary").title = result.output_directory;
    showToast(`Saved ${recordedSamples.length} force samples to ${result.output_directory}`);
  } catch (error) {
    setRecordingUi(false);
    $("timerLabel").textContent = "FAILED";
    $("outputSummary").textContent = `PARTIAL · ${recordingSession}`;
    showToast(`Saving failed: ${error?.message || "unknown error"}. Partial data: ${recordingOutputDirectory}`, true);
  } finally {
    $("stopButton").disabled = false;
    stopping = false;
  }
}

$("startButton").addEventListener("click", startRecording);
$("stopButton").addEventListener("click", stopRecording);
window.addEventListener("beforeunload", (event) => {
  if (recording) { event.preventDefault(); event.returnValue = ""; }
});
