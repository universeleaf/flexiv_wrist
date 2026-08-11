"""Timestamp-aligned Annie recording artifacts stored on the local PC."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re
import shutil
import struct
import subprocess
import threading
import uuid
import zipfile
from typing import BinaryIO, Any


SESSION_PATTERN = re.compile(r"Annie_\d{8}_\d{6}_\d{3}_[0-9a-f]{4}")
FRAME_PATTERN = re.compile(r"frame_\d{9}_\d+\.jpg")
DOWNLOAD_ARTIFACTS = {
    "force_torque.csv": "{session}_force_torque.csv",
    "camera_frames.zip": "{session}_camera_frames.zip",
    "demo.mp4": "{session}_demo.mp4",
}


class RecordingError(RuntimeError):
    """An invalid session, upload, or recording artifact."""


class AnnieRecordingStore:
    """Manage timestamped recordings below one explicitly configured root."""

    def __init__(self, output_root: Path) -> None:
        self.output_root = output_root.expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def start(self, _metadata: dict[str, Any] | None = None) -> dict[str, str]:
        now = datetime.now()
        session = (
            f"Annie_{now:%Y%m%d_%H%M%S}_{now.microsecond // 1000:03d}_"
            f"{uuid.uuid4().hex[:4]}"
        )
        session_dir = self.output_root / session
        frames_dir = session_dir / "camera_frames"
        with self._lock:
            frames_dir.mkdir(parents=True, exist_ok=False)
        return {"session": session, "output_directory": str(session_dir)}

    def _session_dir(self, session: str) -> Path:
        if not SESSION_PATTERN.fullmatch(session):
            raise RecordingError("invalid recording session identifier")
        path = self.output_root / session
        if not path.is_dir():
            raise RecordingError(f"recording session does not exist: {session}")
        return path

    def save_frame_batch(self, session: str, payload: bytes) -> int:
        """Save AFB1 binary batch: count, then name length/name/data length/data."""
        session_dir = self._session_dir(session)
        if len(payload) < 8 or payload[:4] != b"AFB1":
            raise RecordingError("invalid camera frame batch header")
        offset = 4
        (count,) = struct.unpack_from(">I", payload, offset)
        offset += 4
        if count > 500:
            raise RecordingError("camera frame batch contains too many entries")

        entries: list[tuple[str, bytes]] = []
        for _ in range(count):
            if offset + 2 > len(payload):
                raise RecordingError("truncated camera frame name length")
            (name_length,) = struct.unpack_from(">H", payload, offset)
            offset += 2
            if offset + name_length + 4 > len(payload):
                raise RecordingError("truncated camera frame entry")
            try:
                name = payload[offset : offset + name_length].decode("ascii")
            except UnicodeDecodeError as exc:
                raise RecordingError("camera frame name must be ASCII") from exc
            offset += name_length
            (data_length,) = struct.unpack_from(">I", payload, offset)
            offset += 4
            if data_length > 10_000_000 or offset + data_length > len(payload):
                raise RecordingError("invalid camera frame data length")
            if not FRAME_PATTERN.fullmatch(name):
                raise RecordingError(f"invalid camera frame filename: {name!r}")
            entries.append((name, payload[offset : offset + data_length]))
            offset += data_length
        if offset != len(payload):
            raise RecordingError("unexpected bytes at end of camera frame batch")

        frames_dir = session_dir / "camera_frames"
        for name, image_data in entries:
            (frames_dir / name).write_bytes(image_data)
        return len(entries)

    def save_frame(self, session: str, name: str, image_data: bytes) -> Path:
        """Save one browser-encoded JPEG as a reliable fallback to batch upload."""
        session_dir = self._session_dir(session)
        if not FRAME_PATTERN.fullmatch(name):
            raise RecordingError(f"invalid camera frame filename: {name!r}")
        if not image_data or len(image_data) > 10_000_000:
            raise RecordingError("invalid camera frame size")
        if not image_data.startswith(b"\xff\xd8"):
            raise RecordingError("camera frame is not a JPEG image")
        path = session_dir / "camera_frames" / name
        path.write_bytes(image_data)
        return path

    def save_force_csv(self, session: str, payload: bytes) -> Path:
        if len(payload) > 500_000_000:
            raise RecordingError("force CSV is too large")
        path = self._session_dir(session) / f"{session}_force_torque.csv"
        path.write_bytes(payload)
        return path

    def save_dashboard_video(
        self,
        session: str,
        stream: BinaryIO,
        content_length: int,
        content_type: str = "video/webm",
    ) -> Path:
        if content_length <= 0 or content_length > 4_000_000_000:
            raise RecordingError("invalid dashboard video size")
        session_dir = self._session_dir(session)
        is_mp4 = content_type.lower().split(";", 1)[0].strip() == "video/mp4"
        source_suffix = "mp4" if is_mp4 else "webm"
        source_path = session_dir / f"{session}_demo_source.{source_suffix}"
        remaining = content_length
        with source_path.open("wb") as destination:
            while remaining:
                chunk = stream.read(min(1_048_576, remaining))
                if not chunk:
                    raise RecordingError("dashboard video upload ended early")
                destination.write(chunk)
                remaining -= len(chunk)

        output_path = session_dir / f"{session}_demo.mp4"
        if is_mp4:
            if b"ftyp" not in source_path.read_bytes()[:64]:
                raise RecordingError("browser MP4 upload has an invalid file header")
            source_path.replace(output_path)
            return output_path

        if shutil.which("ffmpeg"):
            command = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-i",
                str(source_path),
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output_path),
            ]
            converter = "ffmpeg"
        elif shutil.which("gst-launch-1.0"):
            command = [
                "gst-launch-1.0",
                "-q",
                "-e",
                "uridecodebin",
                f"uri={source_path.as_uri()}",
                "!",
                "videoconvert",
                "!",
                "x264enc",
                "speed-preset=veryfast",
                "bitrate=8000",
                "!",
                "h264parse",
                "!",
                "mp4mux",
                "faststart=true",
                "!",
                "filesink",
                f"location={output_path}",
            ]
            converter = "GStreamer"
        else:
            raise RecordingError(
                "the browser produced WebM, but neither ffmpeg nor GStreamer "
                "is available for MP4 conversion"
            )
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=900, check=False
        )
        if (
            completed.returncode != 0
            or not output_path.is_file()
            or output_path.stat().st_size == 0
        ):
            detail = completed.stderr.strip()[-2000:]
            output_path.unlink(missing_ok=True)
            raise RecordingError(f"{converter} MP4 conversion failed: {detail}")
        source_path.unlink(missing_ok=True)
        return output_path

    def finish(self, session: str) -> dict[str, Any]:
        session_dir = self._session_dir(session)
        frames_dir = session_dir / "camera_frames"
        frame_paths = sorted(frames_dir.glob("frame_*.jpg"))
        archive_path = session_dir / f"{session}_camera_frames.zip"
        if frame_paths:
            with zipfile.ZipFile(
                archive_path, "w", compression=zipfile.ZIP_STORED, allowZip64=True
            ) as archive:
                for frame_path in frame_paths:
                    archive.write(frame_path, arcname=frame_path.name)
            frame_count = len(frame_paths)
        elif archive_path.is_file():
            try:
                with zipfile.ZipFile(archive_path, "r") as archive:
                    frame_count = len(
                        [name for name in archive.namelist() if FRAME_PATTERN.fullmatch(name)]
                    )
            except zipfile.BadZipFile as exc:
                raise RecordingError("camera frame archive is damaged") from exc
        else:
            frame_count = 0

        if frame_count == 0:
            archive_path.unlink(missing_ok=True)
            raise RecordingError(
                "no camera pictures reached the recording service; "
                "the empty camera ZIP was not saved"
            )

        demo_path = session_dir / f"{session}_demo.mp4"
        demo_video_source = "browser"
        if not demo_path.is_file() or demo_path.stat().st_size == 0:
            demo_path.unlink(missing_ok=True)
            self._generate_fallback_demo(session, session_dir, frames_dir, demo_path)
            demo_video_source = "server-fallback"

        required = {
            key: session_dir / template.format(session=session)
            for key, template in DOWNLOAD_ARTIFACTS.items()
        }
        missing = [name for name, path in required.items() if not path.is_file()]
        if missing:
            raise RecordingError(
                "recording is missing required artifacts: " + ", ".join(missing)
            )
        shutil.rmtree(frames_dir, ignore_errors=True)
        return {
            "session": session,
            "output_directory": str(session_dir),
            "camera_frame_count": frame_count,
            "demo_video_source": demo_video_source,
            "downloads": {
                key: f"/api/record/download/{session}/{key}"
                for key in required
            },
        }

    def _generate_fallback_demo(
        self,
        session: str,
        session_dir: Path,
        frames_dir: Path,
        output_path: Path,
    ) -> None:
        csv_path = session_dir / f"{session}_force_torque.csv"
        renderer = Path(__file__).resolve().parents[1] / "scripts" / "render_annie_demo.py"
        system_python = Path("/usr/bin/python3")
        gst_launch = shutil.which("gst-launch-1.0")
        if not csv_path.is_file():
            raise RecordingError("cannot build full-window MP4 without force CSV")
        if not any(frames_dir.glob("frame_*.jpg")):
            raise RecordingError("cannot build demo MP4 without camera frames")
        if not system_python.is_file() or not renderer.is_file():
            raise RecordingError("server-side full-window renderer is unavailable")
        if gst_launch is None:
            raise RecordingError("GStreamer is required for server-side MP4 fallback")

        rendered_dir = session_dir / ".dashboard_frames"
        shutil.rmtree(rendered_dir, ignore_errors=True)
        try:
            render_result = subprocess.run(
                [
                    str(system_python),
                    str(renderer),
                    "--csv",
                    str(csv_path),
                    "--camera-dir",
                    str(frames_dir),
                    "--output-dir",
                    str(rendered_dir),
                    "--fps",
                    "25",
                ],
                capture_output=True,
                text=True,
                timeout=900,
                check=False,
            )
            rendered_frames = sorted(rendered_dir.glob("dashboard_*.jpg"))
            if render_result.returncode != 0 or not rendered_frames:
                detail = render_result.stderr.strip()[-2000:]
                raise RecordingError(f"full-window rendering failed: {detail}")

            command = [
                gst_launch,
                "-q",
                "-e",
                "multifilesrc",
                f"location={rendered_dir / 'dashboard_%09d.jpg'}",
                "index=0",
                f"stop-index={len(rendered_frames) - 1}",
                "caps=image/jpeg,framerate=25/1",
                "!",
                "jpegdec",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=I420",
                "!",
                "x264enc",
                "speed-preset=veryfast",
                "bitrate=6000",
                "!",
                "h264parse",
                "!",
                "mp4mux",
                "faststart=true",
                "!",
                "filesink",
                f"location={output_path}",
            ]
            encode_result = subprocess.run(
                command, capture_output=True, text=True, timeout=900, check=False
            )
            if (
                encode_result.returncode != 0
                or not output_path.is_file()
                or output_path.stat().st_size == 0
            ):
                detail = encode_result.stderr.strip()[-2000:]
                output_path.unlink(missing_ok=True)
                raise RecordingError(f"server-side MP4 encoding failed: {detail}")
        finally:
            shutil.rmtree(rendered_dir, ignore_errors=True)

    def download_path(self, session: str, artifact: str) -> Path:
        template = DOWNLOAD_ARTIFACTS.get(artifact)
        if template is None:
            raise RecordingError("unknown recording artifact")
        path = self._session_dir(session) / template.format(session=session)
        if not path.is_file():
            raise RecordingError("recording artifact is not available")
        return path
