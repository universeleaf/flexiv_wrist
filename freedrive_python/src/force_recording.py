"""Timestamped force-only CSV recordings for a Flexiv Elements project."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re
import threading
import uuid
from typing import Any

from src.annie_recording import RecordingError


LABEL_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_-]{0,31}")


class ForceRecordingStore:
    """Store one clearly named force/torque CSV per timestamped session."""

    def __init__(self, output_root: Path, project_label: str) -> None:
        if not LABEL_PATTERN.fullmatch(project_label):
            raise ValueError("project label must contain only letters, digits, _ or -")
        self.output_root = output_root.expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.project_label = project_label
        self._session_pattern = re.compile(
            rf"{re.escape(project_label)}_\d{{8}}_\d{{6}}_\d{{3}}_[0-9a-f]{{4}}"
        )
        self._lock = threading.Lock()

    def start(self, _metadata: dict[str, Any] | None = None) -> dict[str, str]:
        now = datetime.now()
        session = (
            f"{self.project_label}_{now:%Y%m%d_%H%M%S}_"
            f"{now.microsecond // 1000:03d}_{uuid.uuid4().hex[:4]}"
        )
        session_dir = self.output_root / session
        with self._lock:
            session_dir.mkdir(parents=True, exist_ok=False)
        return {"session": session, "output_directory": str(session_dir)}

    def _session_dir(self, session: str) -> Path:
        if not self._session_pattern.fullmatch(session):
            raise RecordingError("invalid force recording session identifier")
        path = self.output_root / session
        if not path.is_dir():
            raise RecordingError(f"force recording session does not exist: {session}")
        return path

    def save_force_csv(self, session: str, payload: bytes) -> Path:
        if not payload or len(payload) > 500_000_000:
            raise RecordingError("force CSV has an invalid size")
        path = self._session_dir(session) / f"{session}_force_torque.csv"
        path.write_bytes(payload)
        return path

    def finish(self, session: str) -> dict[str, Any]:
        session_dir = self._session_dir(session)
        csv_path = session_dir / f"{session}_force_torque.csv"
        if not csv_path.is_file():
            raise RecordingError("recording is missing force_torque.csv")
        return {
            "session": session,
            "output_directory": str(session_dir),
            "downloads": {
                "force_torque.csv": (
                    f"/api/record/download/{session}/force_torque.csv"
                )
            },
        }

    def download_path(self, session: str, artifact: str) -> Path:
        if artifact != "force_torque.csv":
            raise RecordingError("unknown force recording artifact")
        path = self._session_dir(session) / f"{session}_force_torque.csv"
        if not path.is_file():
            raise RecordingError("force recording artifact is not available")
        return path
