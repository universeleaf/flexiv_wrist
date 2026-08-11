from __future__ import annotations

from pathlib import Path

import pytest

from src.annie_recording import RecordingError
from src.force_recording import ForceRecordingStore


def test_force_recording_creates_one_csv_artifact(tmp_path: Path) -> None:
    store = ForceRecordingStore(tmp_path, "EmJ")
    started = store.start()
    session = started["session"]
    csv_path = store.save_force_csv(session, b"host_iso,tcp_fx_N\nnow,1.0\n")
    result = store.finish(session)

    session_dir = Path(started["output_directory"])
    assert csv_path.name == f"{session}_force_torque.csv"
    assert [path.name for path in session_dir.iterdir()] == [csv_path.name]
    assert result["downloads"] == {
        "force_torque.csv": (
            f"/api/record/download/{session}/force_torque.csv"
        )
    }


def test_force_recording_rejects_wrong_project_session(tmp_path: Path) -> None:
    store = ForceRecordingStore(tmp_path, "EmJ")
    with pytest.raises(RecordingError, match="invalid force recording session"):
        store.save_force_csv("Annie_20260806_120000_000_abcd", b"csv")


def test_force_recording_rejects_unsafe_project_label(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="project label"):
        ForceRecordingStore(tmp_path, "../EmJ")
