"use strict";

const $ = (id) => document.getElementById(id);
const AXIS_COLORS = ["#49d9df", "#ff9e52", "#5d8dff"];
const history = [];
const maxHistorySamples = 12000;
const offsets = {
  tcp: [0, 0, 0, 0, 0, 0],
  world: [0, 0, 0, 0, 0, 0],
  sensor: [0, 0, 0, 0, 0, 0],
};

let latestSample = null;
let latestStatus = null;
let cameraStream = null;
let cameraConnecting = false;
let cameraReconnectTimer = null;
let preferredCameraDeviceId = "";
let preferredCameraLabel = "";
let dashboardStream = null;
let dashboardCanvas = null;
let mediaRecorder = null;
let videoChunks = [];
let recordedSamples = [];
let recording = false;
let recordingStartedMs = 0;
let recordingCamera = null;
let recordingSession = "";
let recordingOutputDirectory = "";
let cameraFrameCanvas = null;
let cameraFrameContext = null;
let frameBatch = [];
let frameUploadChain = Promise.resolve();
let frameCapturePromises = new Set();
let recordingUploadError = null;
let encodedFrameCount = 0;
let uploadedFrameCount = 0;
let dashboardRecordingError = null;
let cameraFrameIndex = 0;
let latestCameraFrame = null;
let cameraFrameCaptureActive = false;
let cameraVideoFrameCallbackId = null;
let cameraFrameInterval = null;
let stoppingRecording = false;
let toastTimer = null;

function showToast(message, error = false) {
  const node = $("toast");
  node.textContent = message;
  node.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { node.className = "toast"; }, 4200);
}

function setPill(node, type, text) {
  node.classList.remove("ok", "error");
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
    $("outputSummary").textContent = `REC · ${recordedSamples.length} samples · ${uploadedFrameCount}/${encodedFrameCount} images`;
  }
}
setInterval(updateClock, 100);
updateClock();

function updateStatus(status) {
  latestStatus = status;
  const phase = status.phase || "unknown";
  if (status.fault) {
    setPill($("robotStatus"), "error", "Robot fault");
  } else if (status.connected) {
    setPill($("robotStatus"), "ok", phase === "demo" ? "Robot demo data" : "Robot data online");
  } else if (phase === "error") {
    setPill($("robotStatus"), "error", "Robot connection failed");
  } else {
    setPill($("robotStatus"), "", "Connecting to robot");
  }
  $("serviceMessage").textContent = status.fault
    ? "The robot reports a fault. Resolve it in Flexiv Elements before running Annie."
    : (status.error
      ? `${status.message}: ${status.error}`
      : (status.message || "This UI monitors only and does not control the project."));
  $("robotIdentity").textContent = status.robot_sn || "Rizon4s-123456";
  $("robotMode").textContent = status.mode || "—";
  $("robotOperational").textContent = status.operational ? "READY" : "NOT READY";
  $("robotBusy").textContent = status.busy ? "RUNNING / BUSY" : "IDLE";
  $("sampleRate").textContent = status.sample_rate_hz ? `${status.sample_rate_hz} Hz` : "—";
  $("taskState").textContent = status.fault
    ? "FAULT"
    : (status.busy ? "RUNNING" : (status.connected ? "READY" : "WAITING"));
  $("taskState").classList.toggle("running", Boolean(status.busy && !status.fault));
}

function adjustedWrench(sample) {
  const source = $("sourceSelect").value;
  const raw = sample?.wrench?.[source] || [0, 0, 0, 0, 0, 0];
  return raw.map((value, index) => Number(value) - offsets[source][index]);
}

function updateMetrics(sample) {
  const values = adjustedWrench(sample);
  const forceNorm = Math.hypot(...values.slice(0, 3));
  const momentNorm = Math.hypot(...values.slice(3, 6));
  $("forceNorm").textContent = forceNorm.toFixed(2);
  $("momentNorm").textContent = momentNorm.toFixed(3);
  $("forceVector").textContent = `Fx ${values[0].toFixed(2)} · Fy ${values[1].toFixed(2)} · Fz ${values[2].toFixed(2)}`;
  $("momentVector").textContent = `Mx ${values[3].toFixed(3)} · My ${values[4].toFixed(3)} · Mz ${values[5].toFixed(3)}`;
  const time = new Date(sample.host_unix_ms);
  $("sampleTimestamp").textContent =
    `PC ${time.toLocaleTimeString("en-GB", {hour12: false})}.${String(time.getMilliseconds()).padStart(3, "0")} · sample #${sample.sequence}`;
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
    if (history.length > maxHistorySamples) history.splice(0, history.length - maxHistorySamples);
    if (recording) recordSynchronizedSample(sample);
    updateMetrics(sample);
  } catch (error) {
    console.error(error);
  }
});
events.onerror = () => {
  if (!latestStatus?.connected) setPill($("robotStatus"), "error", "UI data connection interrupted");
};

async function listCameras() {
  if (!navigator.mediaDevices?.enumerateDevices) return [];
  const devices = await navigator.mediaDevices.enumerateDevices();
  const cameras = devices.filter((device) => device.kind === "videoinput");
  const selected = $("cameraSelect").value;
  $("cameraSelect").innerHTML = "";
  cameras.forEach((camera, index) => {
    const option = document.createElement("option");
    option.value = camera.deviceId;
    option.textContent = camera.label || `Camera ${index + 1}`;
    $("cameraSelect").appendChild(option);
  });
  if (!cameras.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No camera detected";
    $("cameraSelect").appendChild(option);
  } else if (cameras.some((camera) => camera.deviceId === selected)) {
    $("cameraSelect").value = selected;
  } else {
    const pixel = cameras.find((camera) => /pixel|android|webcam/i.test(camera.label));
    if (pixel) $("cameraSelect").value = pixel.deviceId;
  }
  return cameras;
}

function stopCameraTracks() {
  if (cameraStream) cameraStream.getTracks().forEach((track) => track.stop());
  cameraStream = null;
}

function showCameraWaiting(title, hint) {
  $("videoEmptyTitle").textContent = title;
  $("videoEmptyHint").textContent = hint;
  $("videoEmpty").classList.remove("hidden");
  $("videoResolution").textContent = "SIGNAL LOST";
}

function preferredCamera(cameras) {
  return cameras.find((camera) => camera.deviceId === preferredCameraDeviceId)
    || cameras.find((camera) => camera.label && camera.label === preferredCameraLabel)
    || (/pixel|android|webcam/i.test(preferredCameraLabel)
      ? cameras.find((camera) => /pixel|android|webcam/i.test(camera.label))
      : null);
}

function scheduleCameraReconnect(delayMs = 1500) {
  clearTimeout(cameraReconnectTimer);
  if (!preferredCameraLabel || cameraStream?.getVideoTracks?.()[0]?.readyState === "live") return;
  cameraReconnectTimer = setTimeout(async () => {
    try {
      const cameras = await listCameras();
      const camera = preferredCamera(cameras);
      if (!camera) {
        scheduleCameraReconnect(2500);
        return;
      }
      $("cameraSelect").value = camera.deviceId;
      await connectCamera({ automatic: true });
    } catch (error) {
      console.warn("Automatic camera recovery failed", error);
      scheduleCameraReconnect(2500);
    }
  }, delayMs);
}

function handleCameraDisconnected(track) {
  if (cameraStream?.getVideoTracks?.()[0] !== track) return;
  cameraStream = null;
  $("cameraVideo").srcObject = null;
  setPill($("cameraStatus"), "error", "Camera reconnecting");
  showCameraWaiting(
    "Pixel camera disconnected",
    "Reconnect the direct USB cable and select Use USB for → Webcam; recovery is automatic",
  );
  scheduleCameraReconnect();
}

function cameraErrorMessage(error) {
  switch (error?.name) {
    case "NotFoundError":
    case "DevicesNotFoundError":
      return "No UVC camera was found. On the Pixel, open USB Preferences and select Use USB for > Webcam (not File transfer/MTP).";
    case "NotAllowedError":
    case "PermissionDeniedError":
      return "Camera permission was denied. Allow camera access for http://127.0.0.1:8765 in the browser and try again.";
    case "NotReadableError":
    case "TrackStartError":
      return "The camera is busy or unavailable. Close other camera applications, reconnect the Pixel, and try again.";
    case "OverconstrainedError":
    case "ConstraintNotSatisfiedError":
      return "The selected camera cannot provide the requested format. Select another camera and reconnect.";
    default:
      return `Unable to open the camera: ${error?.message || "unknown error"}`;
  }
}

async function connectCamera({ automatic = false } = {}) {
  if (cameraConnecting) return;
  if (!navigator.mediaDevices?.getUserMedia) {
    showToast("This browser cannot access cameras. Use a current Firefox or Chrome browser.", true);
    return;
  }
  cameraConnecting = true;
  try {
    stopCameraTracks();
    const deviceId = $("cameraSelect").value;
    const constraints = {
      audio: false,
      video: {
        ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
        width: { ideal: 1280 },
        height: { ideal: 720 },
        frameRate: { ideal: 30, max: 30 },
      },
    };
    cameraStream = await navigator.mediaDevices.getUserMedia(constraints);
    $("cameraVideo").srcObject = cameraStream;
    await $("cameraVideo").play();
    await listCameras();
    const track = cameraStream.getVideoTracks()[0];
    const settings = track.getSettings();
    preferredCameraDeviceId = settings.deviceId || $("cameraSelect").value;
    preferredCameraLabel = track.label || preferredCameraLabel || "USB camera";
    clearTimeout(cameraReconnectTimer);
    track.addEventListener("ended", () => handleCameraDisconnected(track));
    track.addEventListener("mute", () => {
      if (cameraStream?.getVideoTracks?.()[0] === track) {
        setPill($("cameraStatus"), "error", "Camera signal paused");
      }
    });
    track.addEventListener("unmute", () => {
      if (cameraStream?.getVideoTracks?.()[0] === track) {
        setPill($("cameraStatus"), "ok", "Camera online");
      }
    });
    $("videoEmpty").classList.add("hidden");
    $("videoResolution").textContent = `${settings.width || "—"} × ${settings.height || "—"} · ${Math.round(settings.frameRate || 0)} FPS`;
    $("cameraName").textContent = track.label || "USB camera";
    setPill($("cameraStatus"), "ok", "Camera online");
    $("cameraButton").textContent = "Reconnect";
    showToast(`${automatic ? "Camera reconnected automatically" : "Camera connected"}: ${track.label || "USB camera"}`);
  } catch (error) {
    cameraStream = null;
    setPill($("cameraStatus"), "error", automatic ? "Camera reconnecting" : "Camera connection failed");
    if (automatic) {
      showCameraWaiting(
        "Waiting for Pixel webcam",
        "If the phone shows MTP/File transfer, select Use USB for → Webcam",
      );
      if (!["NotAllowedError", "PermissionDeniedError"].includes(error?.name)) {
        scheduleCameraReconnect(2500);
      }
    } else {
      showToast(cameraErrorMessage(error), true);
    }
  } finally {
    cameraConnecting = false;
  }
}

$("cameraButton").addEventListener("click", () => connectCamera());
$("cameraSelect").addEventListener("change", () => connectCamera());
if (navigator.mediaDevices) {
  navigator.mediaDevices.addEventListener("devicechange", async () => {
    const cameras = await listCameras();
    const currentTrack = cameraStream?.getVideoTracks?.()[0];
    if ((!currentTrack || currentTrack.readyState !== "live") && preferredCamera(cameras)) {
      scheduleCameraReconnect(250);
    }
  });
  listCameras().catch(console.error);
}

$("sourceSelect").addEventListener("change", () => {
  if (latestSample) updateMetrics(latestSample);
});
$("tareButton").addEventListener("click", () => {
  if (!latestSample) {
    showToast("No force data has been received; display tare is unavailable.", true);
    return;
  }
  const source = $("sourceSelect").value;
  offsets[source] = [...latestSample.wrench[source]];
  showToast("Display tare applied. The robot and saved raw values were not modified.");
  updateMetrics(latestSample);
});
$("clearButton").addEventListener("click", () => {
  history.length = 0;
  showToast("Live charts cleared.");
});

function resizeCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function drawChart(canvas, componentOffset, minimumRange) {
  const { width, height, ratio } = resizeCanvas(canvas);
  const ctx = canvas.getContext("2d");
  const left = 42 * ratio, right = 10 * ratio, top = 9 * ratio, bottom = 21 * ratio;
  const plotW = width - left - right, plotH = height - top - bottom;
  ctx.clearRect(0, 0, width, height);
  const seconds = Number($("windowSelect").value);
  const endMs = latestSample?.host_unix_ms || Date.now();
  const startMs = endMs - seconds * 1000;
  const visible = history.filter((sample) => sample.host_unix_ms >= startMs);
  const source = $("sourceSelect").value;
  let peak = minimumRange;
  visible.forEach((sample) => {
    const raw = sample.wrench[source];
    for (let axis = 0; axis < 3; axis += 1) {
      peak = Math.max(peak, Math.abs(raw[componentOffset + axis] - offsets[source][componentOffset + axis]));
    }
  });
  peak *= 1.12;

  ctx.lineWidth = ratio;
  ctx.strokeStyle = "#202a35";
  ctx.fillStyle = "#596571";
  ctx.font = `${9 * ratio}px ui-monospace, monospace`;
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let row = 0; row <= 4; row += 1) {
    const y = top + plotH * row / 4;
    ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + plotW, y); ctx.stroke();
    const value = peak * (1 - row / 2);
    ctx.fillText(value.toFixed(componentOffset ? 2 : 1), left - 6 * ratio, y);
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let column = 0; column <= 4; column += 1) {
    const x = left + plotW * column / 4;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + plotH); ctx.stroke();
    ctx.fillText(`${(-seconds + seconds * column / 4).toFixed(1)}s`, x, top + plotH + 6 * ratio);
  }
  if (visible.length < 2) return;

  for (let axis = 0; axis < 3; axis += 1) {
    ctx.strokeStyle = AXIS_COLORS[axis];
    ctx.lineWidth = 1.45 * ratio;
    ctx.beginPath();
    visible.forEach((sample, index) => {
      const x = left + ((sample.host_unix_ms - startMs) / (seconds * 1000)) * plotW;
      const value = sample.wrench[source][componentOffset + axis] - offsets[source][componentOffset + axis];
      const y = top + (1 - (value + peak) / (2 * peak)) * plotH;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

function renderCharts() {
  drawChart($("forceChart"), 0, 5);
  drawChart($("momentChart"), 3, 0.5);
  if (recording) drawDashboardFrame();
  requestAnimationFrame(renderCharts);
}
requestAnimationFrame(renderCharts);

const FRAME_BATCH_SIZE = 10;
const CAMERA_FRAME_WIDTH = 1280;
const CAMERA_FRAME_HEIGHT = 720;

function mediaRecorderCandidates() {
  const candidates = [
    "video/mp4;codecs=avc1.42E01E",
    "video/mp4;codecs=avc1",
    "video/mp4",
    "video/webm;codecs=vp9",
    "video/webm;codecs=vp8",
    "video/webm",
  ];
  return candidates.filter((type) => MediaRecorder.isTypeSupported(type));
}

function createDashboardRecorder(stream) {
  const errors = [];
  for (const mimeType of mediaRecorderCandidates()) {
    try {
      return new MediaRecorder(stream, { mimeType, videoBitsPerSecond: 8000000 });
    } catch (error) {
      errors.push(`${mimeType}: ${error?.message || error}`);
    }
  }
  try {
    return new MediaRecorder(stream, { videoBitsPerSecond: 8000000 });
  } catch (error) {
    errors.push(`browser default: ${error?.message || error}`);
  }
  throw new Error(`no MediaRecorder format could start (${errors.join("; ")})`);
}

function setRecordingUi(active) {
  $("recordButton").classList.toggle("hidden", active);
  $("stopButton").classList.toggle("hidden", !active);
  $("stopButton").disabled = false;
  $("stopButton").innerHTML = '<span class="button-icon">■</span>Stop & save';
  $("recordTimer").classList.toggle("recording", active);
  $("recordOverlay").classList.toggle("recording", active);
  $("timerLabel").textContent = active ? "REC" : "SAVED";
  if (!active) $("timerValue").textContent = "00:00.0";
}

function setFinalizingUi() {
  $("stopButton").disabled = true;
  $("stopButton").innerHTML = '<span class="button-icon">■</span>Saving MP4…';
  $("timerLabel").textContent = "FINALIZING";
}

async function recordingRequest(path, body, contentType = "") {
  const headers = {};
  if (contentType) headers["Content-Type"] = contentType;
  if (path !== "/api/record/start") headers["X-Annie-Session"] = recordingSession;
  const response = await fetch(path, { method: "POST", headers, body });
  let payload = {};
  try {
    payload = await response.json();
  } catch (_) {
    // The status code below still provides a useful failure if JSON is unavailable.
  }
  if (!response.ok) {
    throw new Error(payload.error || `recording service returned HTTP ${response.status}`);
  }
  return payload;
}

function csvNumber(value) {
  if (value === null || value === undefined || value === "") return "";
  const number = Number(value);
  return Number.isFinite(number) ? number.toPrecision(12) : "";
}

function csvText(value) {
  return `"${String(value ?? "").replaceAll('"', '""')}"`;
}

function samplesToCsv(samples) {
  const columns = [
    "recording_index", "camera_frame", "sequence",
    "force_host_iso", "force_host_unix_ns",
    "camera_capture_iso", "camera_capture_unix_ms", "camera_delay_from_force_ms",
    "elapsed_s", "robot_seconds", "robot_nanoseconds",
    "tcp_fx_N", "tcp_fy_N", "tcp_fz_N", "tcp_mx_Nm", "tcp_my_Nm", "tcp_mz_Nm",
    "world_fx_N", "world_fy_N", "world_fz_N", "world_mx_Nm", "world_my_Nm", "world_mz_Nm",
    "sensor_fx_N", "sensor_fy_N", "sensor_fz_N", "sensor_mx_Nm", "sensor_my_Nm", "sensor_mz_Nm",
    "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_qw", "tcp_qx", "tcp_qy", "tcp_qz",
    "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad", "q6_rad", "q7_rad",
  ];
  const rows = [columns.join(",")];
  samples.forEach((sample) => {
    const values = [
      sample.recording_index,
      sample.camera_frame,
      sample.sequence,
      new Date(sample.host_unix_ms).toISOString(),
      String(sample.host_unix_ns),
      sample.camera_capture_unix_ms
        ? new Date(sample.camera_capture_unix_ms).toISOString()
        : "",
      sample.camera_capture_unix_ms,
      sample.camera_delay_from_force_ms,
      csvNumber((sample.host_unix_ms - recordingStartedMs) / 1000),
      sample.robot_time.seconds,
      sample.robot_time.nanoseconds,
      ...sample.wrench.tcp,
      ...sample.wrench.world,
      ...sample.wrench.sensor,
      ...sample.tcp_pose,
      ...sample.q,
    ];
    rows.push(values.map((value, index) => (
      [1, 3, 4, 5].includes(index) ? csvText(value) : csvNumber(value)
    )).join(","));
  });
  return `${rows.join("\n")}\n`;
}

function canvasBlob(canvas, type, quality) {
  return new Promise((resolve, reject) => {
    let settled = false;
    const timeout = setTimeout(() => {
      if (!settled) {
        settled = true;
        reject(new Error(`encoding ${type} timed out`));
      }
    }, 5000);
    canvas.toBlob((blob) => {
      if (settled) return;
      settled = true;
      clearTimeout(timeout);
      if (blob) resolve(blob); else reject(new Error(`unable to encode ${type}`));
    }, type, quality);
  });
}

function drawVideoContained(ctx, video, x, y, width, height) {
  ctx.fillStyle = "#030507";
  ctx.fillRect(x, y, width, height);
  if (!video || video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA || !video.videoWidth) {
    return false;
  }
  const scale = Math.min(width / video.videoWidth, height / video.videoHeight);
  const drawWidth = video.videoWidth * scale;
  const drawHeight = video.videoHeight * scale;
  ctx.drawImage(
    video,
    x + (width - drawWidth) / 2,
    y + (height - drawHeight) / 2,
    drawWidth,
    drawHeight,
  );
  return true;
}

async function encodeFrameBatch(entries) {
  const encoder = new TextEncoder();
  const parts = [];
  const batchHeader = new ArrayBuffer(8);
  const batchView = new DataView(batchHeader);
  new Uint8Array(batchHeader, 0, 4).set([65, 70, 66, 49]); // AFB1
  batchView.setUint32(4, entries.length, false);
  parts.push(batchHeader);
  for (const entry of entries) {
    const name = encoder.encode(entry.name);
    const itemHeader = new ArrayBuffer(6);
    const itemView = new DataView(itemHeader);
    itemView.setUint16(0, name.length, false);
    itemView.setUint32(2, entry.blob.size, false);
    parts.push(itemHeader, name, entry.blob);
  }
  return new Blob(parts, { type: "application/octet-stream" });
}

function queueFrameBatch(force = false) {
  if (!frameBatch.length || (!force && frameBatch.length < FRAME_BATCH_SIZE)) return;
  const entries = frameBatch.splice(0, force ? frameBatch.length : FRAME_BATCH_SIZE);
  frameUploadChain = frameUploadChain.then(async () => {
    try {
      const payload = await encodeFrameBatch(entries);
      const result = await recordingRequest("/api/record/frames", payload, "application/octet-stream");
      const saved = Number(result.saved || 0);
      if (saved !== entries.length) {
        throw new Error(`batch saved ${saved} of ${entries.length} images`);
      }
      uploadedFrameCount += saved;
    } catch (batchError) {
      console.warn("Camera frame batch upload failed; retrying images one at a time", batchError);
      for (const entry of entries) {
        await recordingRequest(
          `/api/record/frame/${encodeURIComponent(entry.name)}`,
          entry.blob,
          "image/jpeg",
        );
        uploadedFrameCount += 1;
      }
    }
  }).catch((error) => {
    recordingUploadError ||= error;
    console.error("Camera frame upload failed", error);
  });
}

async function captureCameraFrame() {
  const ctx = cameraFrameContext;
  if (!ctx) throw new Error("camera capture canvas is unavailable");
  const frameIndex = cameraFrameIndex;
  cameraFrameIndex += 1;
  const captureUnixMs = Date.now();
  const frameName = `frame_${String(frameIndex).padStart(9, "0")}_${captureUnixMs}000000.jpg`;
  drawVideoContained(
    ctx,
    $("cameraVideo"),
    0,
    0,
    CAMERA_FRAME_WIDTH,
    CAMERA_FRAME_HEIGHT,
  );
  const blob = await canvasBlob(cameraFrameCanvas, "image/jpeg", 0.80);
  frameBatch.push({ name: frameName, blob });
  encodedFrameCount += 1;
  if (!latestCameraFrame || frameIndex > latestCameraFrame.index) {
    latestCameraFrame = {
      index: frameIndex,
      name: frameName,
      capture_unix_ms: captureUnixMs,
    };
  }
  queueFrameBatch();
}

function trackCameraFrameCapture() {
  const capture = captureCameraFrame().catch((error) => {
    recordingUploadError ||= error;
    console.error("Camera frame capture failed", error);
  });
  frameCapturePromises.add(capture);
  capture.finally(() => frameCapturePromises.delete(capture));
  return capture;
}

async function startCameraFrameCapture() {
  cameraFrameCaptureActive = true;
  await trackCameraFrameCapture();
  const video = $("cameraVideo");
  if (typeof video.requestVideoFrameCallback === "function") {
    const onVideoFrame = () => {
      if (!cameraFrameCaptureActive) return;
      trackCameraFrameCapture();
      cameraVideoFrameCallbackId = video.requestVideoFrameCallback(onVideoFrame);
    };
    cameraVideoFrameCallbackId = video.requestVideoFrameCallback(onVideoFrame);
  } else {
    cameraFrameInterval = setInterval(() => {
      if (cameraFrameCaptureActive) trackCameraFrameCapture();
    }, 1000 / 30);
  }
}

function stopCameraFrameCapture() {
  cameraFrameCaptureActive = false;
  const video = $("cameraVideo");
  if (
    cameraVideoFrameCallbackId !== null
    && typeof video.cancelVideoFrameCallback === "function"
  ) {
    video.cancelVideoFrameCallback(cameraVideoFrameCallbackId);
  }
  cameraVideoFrameCallbackId = null;
  clearInterval(cameraFrameInterval);
  cameraFrameInterval = null;
}

function recordSynchronizedSample(sample) {
  const recordingIndex = recordedSamples.length;
  const cameraCaptureUnixMs = latestCameraFrame?.capture_unix_ms ?? null;
  const recordedSample = {
    ...sample,
    recording_index: recordingIndex,
    camera_frame: latestCameraFrame?.name || "",
    camera_capture_unix_ms: cameraCaptureUnixMs,
    camera_delay_from_force_ms: cameraCaptureUnixMs === null
      ? null
      : cameraCaptureUnixMs - Number(sample.host_unix_ms),
  };
  recordedSamples.push(recordedSample);
}

function dashboardPanel(ctx, x, y, width, height) {
  ctx.fillStyle = "#101720";
  ctx.fillRect(x, y, width, height);
  ctx.strokeStyle = "#293542";
  ctx.lineWidth = 2;
  ctx.strokeRect(x, y, width, height);
}

function drawDashboardFrame() {
  if (!dashboardCanvas) return;
  const ctx = dashboardCanvas.getContext("2d");
  const width = dashboardCanvas.width;
  ctx.fillStyle = "#080c11";
  ctx.fillRect(0, 0, width, dashboardCanvas.height);

  ctx.fillStyle = "#49d9df";
  ctx.font = "700 20px sans-serif";
  ctx.fillText("RIZON 4S · READ-ONLY ACQUISITION", 40, 42);
  ctx.fillStyle = "#f3f6f8";
  ctx.font = "700 34px sans-serif";
  ctx.fillText("Annie · Vision & Force Monitor", 40, 82);
  ctx.fillStyle = "#a8b4bf";
  ctx.font = "500 21px monospace";
  ctx.textAlign = "right";
  ctx.fillText(new Date().toISOString(), width - 40, 48);
  const elapsed = Math.max(0, (Date.now() - recordingStartedMs) / 1000);
  ctx.fillText(`REC ${elapsed.toFixed(3)} s · ${recordedSamples.length} samples`, width - 40, 79);
  ctx.textAlign = "left";

  dashboardPanel(ctx, 40, 110, 1180, 650);
  drawVideoContained(ctx, $("cameraVideo"), 50, 120, 1160, 630);

  dashboardPanel(ctx, 1240, 110, 640, 650);
  const wrench = latestSample ? adjustedWrench(latestSample) : [0, 0, 0, 0, 0, 0];
  const forceNorm = Math.hypot(...wrench.slice(0, 3));
  const momentNorm = Math.hypot(...wrench.slice(3, 6));
  ctx.fillStyle = "#49d9df";
  ctx.font = "700 17px sans-serif";
  ctx.fillText("CURRENT END-EFFECTOR WRENCH", 1270, 153);
  ctx.fillStyle = "#f3f6f8";
  ctx.font = "700 54px monospace";
  ctx.fillText(`${forceNorm.toFixed(2)} N`, 1270, 222);
  ctx.fillStyle = "#8d9aa7";
  ctx.font = "500 22px monospace";
  ctx.fillText(`Fx ${wrench[0].toFixed(3)} N`, 1270, 271);
  ctx.fillText(`Fy ${wrench[1].toFixed(3)} N`, 1270, 309);
  ctx.fillText(`Fz ${wrench[2].toFixed(3)} N`, 1270, 347);
  ctx.fillStyle = "#ff9e52";
  ctx.font = "700 42px monospace";
  ctx.fillText(`${momentNorm.toFixed(3)} Nm`, 1270, 415);
  ctx.fillStyle = "#8d9aa7";
  ctx.font = "500 22px monospace";
  ctx.fillText(`Mx ${wrench[3].toFixed(4)} Nm`, 1270, 459);
  ctx.fillText(`My ${wrench[4].toFixed(4)} Nm`, 1270, 497);
  ctx.fillText(`Mz ${wrench[5].toFixed(4)} Nm`, 1270, 535);
  ctx.strokeStyle = "#293542";
  ctx.beginPath(); ctx.moveTo(1270, 565); ctx.lineTo(1850, 565); ctx.stroke();
  ctx.fillStyle = "#c2ccd5";
  ctx.font = "600 19px sans-serif";
  const sourceLabels = {
    tcp: "Compensated TCP wrench · TCP frame",
    world: "Compensated TCP wrench · world frame",
    sensor: "Raw flange F/T sensor",
  };
  ctx.fillText(sourceLabels[$("sourceSelect").value], 1270, 606);
  ctx.fillStyle = "#84909d";
  ctx.font = "500 18px monospace";
  ctx.fillText(`Robot: ${latestStatus?.robot_sn || "Rizon4s-123456"}`, 1270, 646);
  ctx.fillText(`Mode: ${latestStatus?.mode || "unknown"}`, 1270, 674);
  ctx.fillText(`State: ${latestStatus?.fault ? "FAULT" : (latestStatus?.busy ? "RUNNING / BUSY" : "IDLE")}`, 1270, 702);
  ctx.fillText(`RDK: ${latestStatus?.sample_rate_hz || "—"} Hz`, 1270, 730);

  dashboardPanel(ctx, 40, 780, 910, 260);
  dashboardPanel(ctx, 970, 780, 910, 260);
  ctx.fillStyle = "#49d9df";
  ctx.font = "700 18px sans-serif";
  ctx.fillText("FORCE · Fx / Fy / Fz · N", 65, 812);
  ctx.fillStyle = "#ff9e52";
  ctx.fillText("MOMENT · Mx / My / Mz · Nm", 995, 812);
  ctx.drawImage($("forceChart"), 55, 825, 880, 200);
  ctx.drawImage($("momentChart"), 985, 825, 880, 200);
  // Deliberately do not draw recordingSession or the MP4 filename anywhere.
}

async function startRecording() {
  if (recording || stoppingRecording) return;
  if (!latestSample) {
    showToast("No robot force data has been received; synchronized capture cannot start.", true);
    return;
  }
  const cameraTrack = cameraStream?.getVideoTracks?.()[0];
  if (!cameraTrack || cameraTrack.readyState !== "live") {
    showToast("Connect the Pixel camera before starting synchronized capture.", true);
    return;
  }
  $("recordButton").disabled = true;
  try {
    recordingCamera = { label: cameraTrack.label, settings: cameraTrack.getSettings() };
    recordedSamples = [];
    videoChunks = [];
    frameBatch = [];
    frameUploadChain = Promise.resolve();
    frameCapturePromises = new Set();
    recordingUploadError = null;
    encodedFrameCount = 0;
    uploadedFrameCount = 0;
    dashboardRecordingError = null;
    cameraFrameIndex = 0;
    latestCameraFrame = null;
    recordingSession = "";
    recordingOutputDirectory = "";
    recordingStartedMs = Date.now();
    cameraFrameCanvas = document.createElement("canvas");
    cameraFrameCanvas.width = CAMERA_FRAME_WIDTH;
    cameraFrameCanvas.height = CAMERA_FRAME_HEIGHT;
    cameraFrameContext = cameraFrameCanvas.getContext("2d", { alpha: false });
    if (window.MediaRecorder) {
      try {
        dashboardCanvas = document.createElement("canvas");
        dashboardCanvas.width = 1920;
        dashboardCanvas.height = 1080;
        dashboardStream = dashboardCanvas.captureStream(30);
        mediaRecorder = createDashboardRecorder(dashboardStream);
        mediaRecorder.addEventListener("dataavailable", (event) => {
          if (event.data.size) videoChunks.push(event.data);
        });
        mediaRecorder.addEventListener("error", (event) => {
          dashboardRecordingError = event.error || new Error("dashboard MediaRecorder failed");
          console.error("Dashboard recording failed", dashboardRecordingError);
        });
        drawDashboardFrame();
        mediaRecorder.start(1000);
      } catch (error) {
        dashboardRecordingError = error;
        dashboardStream?.getTracks().forEach((track) => track.stop());
        dashboardStream = null;
        dashboardCanvas = null;
        mediaRecorder = null;
        console.warn("Browser MP4 recording unavailable; the server will build it", error);
      }
    } else {
      dashboardRecordingError = new Error("browser MediaRecorder is unavailable");
    }

    const started = await recordingRequest(
      "/api/record/start",
      JSON.stringify({
        project_label: "Annie",
        robot_sn: latestStatus?.robot_sn || "Rizon4s-123456",
        recording_started_pc_iso: new Date(recordingStartedMs).toISOString(),
        requested_force_rate_hz: latestStatus?.sample_rate_hz || null,
        camera: recordingCamera,
        video_mime_type: mediaRecorder?.mimeType || "server-generated",
      }),
      "application/json",
    );
    recordingSession = started.session;
    recordingOutputDirectory = started.output_directory;
    $("outputSummary").textContent = recordingSession;
    $("outputSummary").title = recordingOutputDirectory;
    await startCameraFrameCapture();
    recording = true;
    setRecordingUi(true);
    showToast(`Synchronized capture started. Output: ${recordingOutputDirectory}`);
  } catch (error) {
    if (mediaRecorder && mediaRecorder.state !== "inactive") {
      try { mediaRecorder.stop(); } catch (_) { /* recorder cleanup only */ }
    }
    dashboardStream?.getTracks().forEach((track) => track.stop());
    dashboardStream = null;
    stopCameraFrameCapture();
    dashboardCanvas = null;
    mediaRecorder = null;
    recording = false;
    setRecordingUi(false);
    $("timerLabel").textContent = "READY";
    if (recordingSession) {
      $("outputSummary").textContent = `PARTIAL · ${recordingSession}`;
      $("outputSummary").title = recordingOutputDirectory;
    }
    showToast(`Unable to start synchronized capture: ${error?.message || "unknown error"}`, true);
  } finally {
    $("recordButton").disabled = false;
  }
}

async function stopRecording() {
  if (!recording || stoppingRecording) return;
  stoppingRecording = true;
  recording = false;
  stopCameraFrameCapture();
  setFinalizingUi();
  let saved = false;
  const errors = [];
  try {
    if (mediaRecorder && mediaRecorder.state !== "inactive") {
      try {
        await new Promise((resolve) => {
          mediaRecorder.addEventListener("stop", resolve, { once: true });
          mediaRecorder.stop();
        });
      } catch (error) {
        errors.push(`video stop: ${error?.message || error}`);
      }
    }
    dashboardStream?.getTracks().forEach((track) => track.stop());
    dashboardStream = null;

    const csv = samplesToCsv(recordedSamples);
    try {
      await recordingRequest("/api/record/force", new Blob([csv], { type: "text/csv;charset=utf-8" }), "text/csv;charset=utf-8");
    } catch (error) {
      errors.push(`force CSV: ${error?.message || error}`);
    }

    await Promise.all([...frameCapturePromises]);
    queueFrameBatch(true);
    await frameUploadChain;
    if (recordingUploadError) errors.push(`camera frames: ${recordingUploadError.message || recordingUploadError}`);

    if (videoChunks.length) {
      const videoType = mediaRecorder?.mimeType || videoChunks[0]?.type || "video/webm";
      const video = new Blob(videoChunks, { type: videoType });
      try {
        await recordingRequest("/api/record/dashboard", video, videoType);
      } catch (error) {
        console.warn("Browser video upload failed; server fallback will be used", error);
      }
    }

    let result = null;
    try {
      result = await recordingRequest("/api/record/finish", new Blob([]));
      if (result.camera_frame_count !== encodedFrameCount) {
        errors.push(`camera frames: saved ${result.camera_frame_count} of ${encodedFrameCount} encoded images`);
      }
    } catch (error) {
      errors.push(`final package: ${error?.message || error}`);
    }
    if (errors.length) {
      throw new Error(errors.join(" | "));
    }
    saved = true;
    $("outputSummary").textContent = `SAVED · ${recordingSession}`;
    $("outputSummary").title = result.output_directory;
    showToast(`Saved ${recordedSamples.length} synchronized samples to ${result.output_directory}`);
  } catch (error) {
    console.error(error);
    $("outputSummary").textContent = `PARTIAL · ${recordingSession || "capture failed"}`;
    $("outputSummary").title = recordingOutputDirectory;
    showToast(`Saving failed: ${error?.message || "unknown error"}. Partial data remains in ${recordingOutputDirectory}.`, true);
  } finally {
    dashboardStream?.getTracks().forEach((track) => track.stop());
    dashboardStream = null;
    dashboardCanvas = null;
    cameraFrameCanvas = null;
    cameraFrameContext = null;
    mediaRecorder = null;
    videoChunks = [];
    setRecordingUi(false);
    if (!saved) $("timerLabel").textContent = "FAILED";
    stoppingRecording = false;
  }
}

$("recordButton").addEventListener("click", startRecording);
$("stopButton").addEventListener("click", stopRecording);
window.addEventListener("beforeunload", (event) => {
  if (recording) {
    event.preventDefault();
    event.returnValue = "";
  }
});
