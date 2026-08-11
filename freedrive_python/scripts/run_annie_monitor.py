#!/usr/bin/env python3
"""Serve the local Annie camera + Flexiv force monitoring UI."""

from __future__ import annotations

import argparse
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import sys
import threading
import webbrowser
from urllib.error import URLError
from urllib.parse import urlparse
from urllib.request import urlopen

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.annie_monitor import (  # noqa: E402
    DemoMonitorWorker,
    MonitorHub,
    RobotMonitorConfig,
    RobotMonitorWorker,
)
from src.annie_recording import (  # noqa: E402
    AnnieRecordingStore,
    RecordingError,
)
from src.force_recording import ForceRecordingStore  # noqa: E402


STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a localhost UI that reads Flexiv force/torque states without "
            "sending commands and captures a browser-selected USB camera."
        )
    )
    parser.add_argument("--robot-sn", default="Rizon4s-123456")
    parser.add_argument("--network-interface-ip", default="127.0.0.1")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--project-label", default="Annie")
    parser.add_argument(
        "--monitor-profile",
        choices=("vision-force", "force-only"),
        default="vision-force",
    )
    parser.add_argument(
        "--recording-output",
        default=str(Path.home() / "Annie"),
        help="directory for timestamped synchronized recording sessions",
    )
    parser.add_argument(
        "--demo", action="store_true", help="use synthetic force data; do not connect"
    )
    parser.add_argument(
        "--open-browser", action="store_true", help="open the UI in the default browser"
    )
    parser.add_argument(
        "--quiet-rdk", action="store_true", help="disable Flexiv RDK info prints"
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not args.demo:
        address = ipaddress.ip_address(args.network_interface_ip)
        if address.version != 4 or address.is_unspecified:
            raise ValueError("network interface must be a usable IPv4 address")
    if not (1 <= args.port <= 65535):
        raise ValueError("port must be between 1 and 65535")
    if not (1.0 <= args.sample_rate <= 200.0):
        raise ValueError("sample rate must be between 1 and 200 Hz")
    if not args.project_label or not args.project_label.isascii():
        raise ValueError("project label must be non-empty ASCII text")


def monitor_url(host: str, port: int) -> str:
    public_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    return f"http://{public_host}:{port}"


def probe_annie_monitor(
    url: str, project_label: str = "Annie"
) -> dict[str, object] | None:
    """Return status when *url* belongs to the expected project monitor."""
    try:
        with urlopen(f"{url}/api/status", timeout=0.75) as response:
            payload = json.load(response)
    except (OSError, URLError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    status = payload.get("status")
    if not isinstance(status, dict):
        return None
    if (
        status.get("read_only") is not True
        or status.get("project_label") != project_label
    ):
        return None
    return status


def compatible_existing_monitor(
    status: dict[str, object] | None,
    *,
    demo: bool,
    robot_sn: str,
    project_label: str = "Annie",
    recording_profile: str = "vision-force",
) -> bool:
    if status is None:
        return False
    if status.get("recording_api_version") != 5:
        return False
    if status.get("project_label") != project_label:
        return False
    if status.get("recording_profile", "vision-force") != recording_profile:
        return False
    existing_demo = status.get("phase") == "demo"
    if existing_demo != demo:
        return False
    if not demo:
        existing_sn = status.get("robot_sn")
        if existing_sn not in {None, robot_sn}:
            return False
    return True


def open_browser_later(url: str) -> None:
    timer = threading.Timer(0.7, lambda: webbrowser.open(url))
    timer.daemon = True
    timer.start()


def handler_class(
    hub: MonitorHub,
    static_root: Path,
    recording_store: AnnieRecordingStore,
) -> type[BaseHTTPRequestHandler]:
    class MonitorRequestHandler(BaseHTTPRequestHandler):
        server_version = "FlexivProjectMonitor/1.0"

        def _headers(
            self, status: int, content_type: str, content_length: int | None = None
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "img-src 'self' data:; media-src 'self' blob:; connect-src 'self'",
            )
            if content_length is not None:
                self.send_header("Content-Length", str(content_length))
            self.end_headers()

        def _json(self, value: object, status: int = 200) -> None:
            payload = json.dumps(
                value, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            self._headers(status, "application/json; charset=utf-8", len(payload))
            self.wfile.write(payload)

        def do_GET(self) -> None:  # noqa: N802
            route = urlparse(self.path).path
            if route == "/api/status":
                sample, status = hub.snapshot()
                self._json({"status": status, "latest_sample": sample})
                return
            if route == "/api/stream":
                self._stream_events()
                return
            if route.startswith("/api/record/download/"):
                self._download_recording(route)
                return
            self._static(route)

        def _content_length(self, maximum: int) -> int:
            try:
                length = int(self.headers.get("Content-Length", "-1"))
            except ValueError as exc:
                raise RecordingError("invalid Content-Length") from exc
            if length < 0 or length > maximum:
                raise RecordingError("request body has an invalid size")
            return length

        def _read_body(self, maximum: int) -> bytes:
            length = self._content_length(maximum)
            payload = self.rfile.read(length)
            if len(payload) != length:
                raise RecordingError("request body ended early")
            return payload

        def _read_json(self, maximum: int = 2_000_000) -> dict[str, object]:
            try:
                value = json.loads(self._read_body(maximum).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise RecordingError("request body is not valid JSON") from exc
            if not isinstance(value, dict):
                raise RecordingError("JSON request body must be an object")
            return value

        def _session_header(self) -> str:
            session = self.headers.get(
                "X-Capture-Session", self.headers.get("X-Annie-Session", "")
            )
            if not session:
                raise RecordingError("missing capture session header")
            return session

        def do_POST(self) -> None:  # noqa: N802
            route = urlparse(self.path).path
            try:
                if route == "/api/record/start":
                    self._json(recording_store.start(self._read_json()))
                    return
                if route == "/api/record/frames":
                    count = recording_store.save_frame_batch(
                        self._session_header(), self._read_body(100_000_000)
                    )
                    self._json({"saved": count})
                    return
                if route.startswith("/api/record/frame/"):
                    name = route.removeprefix("/api/record/frame/")
                    path = recording_store.save_frame(
                        self._session_header(), name, self._read_body(10_000_000)
                    )
                    self._json({"saved": path.name})
                    return
                if route == "/api/record/force":
                    path = recording_store.save_force_csv(
                        self._session_header(), self._read_body(500_000_000)
                    )
                    self._json({"saved": path.name})
                    return
                if route == "/api/record/dashboard":
                    path = recording_store.save_dashboard_video(
                        self._session_header(),
                        self.rfile,
                        self._content_length(4_000_000_000),
                        self.headers.get("Content-Type", "video/webm"),
                    )
                    self._json({"saved": path.name})
                    return
                if route == "/api/record/finish":
                    self._json(recording_store.finish(self._session_header()))
                    return
                self._json({"error": "not found"}, status=404)
            except RecordingError as exc:
                self._json({"error": str(exc)}, status=400)
            except Exception as exc:
                self._json({"error": f"recording operation failed: {exc}"}, status=500)

        def _download_recording(self, route: str) -> None:
            parts = route.split("/")
            if len(parts) != 6:
                self._json({"error": "invalid recording download URL"}, status=404)
                return
            session, artifact = parts[4], parts[5]
            try:
                path = recording_store.download_path(session, artifact)
            except RecordingError as exc:
                self._json({"error": str(exc)}, status=404)
                return
            content_types = {
                ".csv": "text/csv; charset=utf-8",
                ".zip": "application/zip",
                ".mp4": "video/mp4",
            }
            self.send_response(200)
            self.send_header(
                "Content-Type", content_types.get(path.suffix, "application/octet-stream")
            )
            self.send_header("Content-Length", str(path.stat().st_size))
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with path.open("rb") as source:
                while chunk := source.read(1_048_576):
                    self.wfile.write(chunk)

        def _stream_events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            last_sequence = -1
            last_revision = -1
            try:
                while True:
                    sample, status = hub.wait_for_change(
                        last_sequence, last_revision, timeout=1.0
                    )
                    revision = int(status.get("revision", -1))
                    if revision != last_revision:
                        self._send_event("status", {"type": "status", **status})
                        last_revision = revision
                    if sample is not None:
                        sequence = int(sample.get("sequence", -1))
                        if sequence != last_sequence:
                            self._send_event("sample", sample)
                            last_sequence = sequence
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return

        def _send_event(self, event: str, value: object) -> None:
            payload = json.dumps(
                value, ensure_ascii=False, separators=(",", ":"), allow_nan=False
            )
            self.wfile.write(f"event: {event}\ndata: {payload}\n\n".encode("utf-8"))

        def _static(self, route: str) -> None:
            relative = "index.html" if route in {"", "/"} else route.lstrip("/")
            allowed = {"index.html", "styles.css", "app.js"}
            if relative not in allowed:
                self._json({"error": "not found"}, status=404)
                return
            path = static_root / relative
            try:
                payload = path.read_bytes()
            except OSError:
                self._json({"error": "UI asset missing"}, status=500)
                return
            self._headers(
                200,
                STATIC_TYPES.get(path.suffix, "application/octet-stream"),
                len(payload),
            )
            self.wfile.write(payload)

        def log_message(self, message: str, *args: object) -> None:
            if not any(route in str(args) for route in ("/api/stream",)):
                print(f"[UI] {self.address_string()} {message % args}")

    return MonitorRequestHandler


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"Invalid argument: {exc}", file=sys.stderr)
        return 2

    hub = MonitorHub(args.project_label, args.monitor_profile)
    static_root = PROJECT_ROOT / (
        "ui_emj" if args.monitor_profile == "force-only" else "ui"
    )
    try:
        if args.monitor_profile == "force-only":
            recording_store = ForceRecordingStore(
                Path(args.recording_output), args.project_label
            )
        else:
            recording_store = AnnieRecordingStore(Path(args.recording_output))
    except OSError as exc:
        print(f"Unable to create recording output directory: {exc}", file=sys.stderr)
        return 1
    request_handler = handler_class(hub, static_root, recording_store)
    selected_port = args.port
    requested_url = monitor_url(args.host, args.port)
    try:
        server = ThreadingHTTPServer((args.host, selected_port), request_handler)
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            print(f"Unable to start the UI server: {exc}", file=sys.stderr)
            return 1

        existing_status = probe_annie_monitor(requested_url, args.project_label)
        if compatible_existing_monitor(
            existing_status,
            demo=args.demo,
            robot_sn=args.robot_sn,
            project_label=args.project_label,
            recording_profile=args.monitor_profile,
        ):
            print()
            print(
                f"A compatible {args.project_label} monitor is already running "
                f"at {requested_url}."
            )
            print("Reusing the existing monitor; no second RDK connection was created.")
            if existing_status and existing_status.get("fault") is True:
                print(
                    "WARNING: the robot currently reports fault=true. Resolve the "
                    f"fault in Flexiv Elements before running {args.project_label}."
                )
            if args.open_browser:
                webbrowser.open(requested_url)
            return 0

        server = None
        for candidate_port in range(args.port + 1, min(args.port + 21, 65536)):
            try:
                server = ThreadingHTTPServer(
                    (args.host, candidate_port), request_handler
                )
                selected_port = candidate_port
                break
            except OSError as candidate_exc:
                if candidate_exc.errno != errno.EADDRINUSE:
                    print(
                        f"Unable to start the UI server: {candidate_exc}",
                        file=sys.stderr,
                    )
                    return 1
        if server is None:
            print(
                f"Ports {args.port}-{min(args.port + 20, 65535)} are unavailable.",
                file=sys.stderr,
            )
            return 1
        print(
            f"Port {args.port} is occupied by another service or an incompatible "
            f"monitor; using {selected_port} instead."
        )

    server.daemon_threads = True
    url = monitor_url(args.host, selected_port)

    if args.demo:
        worker: DemoMonitorWorker | RobotMonitorWorker = DemoMonitorWorker(
            hub, args.sample_rate
        )
    else:
        worker = RobotMonitorWorker(
            hub,
            RobotMonitorConfig(
                robot_sn=args.robot_sn,
                network_interface_ip=args.network_interface_ip,
                sample_rate_hz=args.sample_rate,
                verbose_rdk=not args.quiet_rdk,
            ),
        )
    worker.start()
    print()
    print(f"{args.project_label} 6-axis end-effector force monitor started")
    print(f"UI: {url}")
    print(f"Recording output: {recording_store.output_root}")
    print(
        "Robot access: strictly read-only. This program does not start/stop "
        f"{args.project_label} or send robot commands."
    )
    print(
        "Workflow: start force recording in the UI, then run "
        f"{args.project_label} in Flexiv Elements."
    )
    print("Finish: stop and save in the UI; press Ctrl+C here to close the monitor.")
    if args.demo:
        print("--demo is active: using synthetic force signals without a robot connection.")

    if args.open_browser:
        open_browser_later(url)

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print(f"\nShutting down the {args.project_label} monitor...")
    finally:
        server.shutdown()
        server.server_close()
        worker.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
