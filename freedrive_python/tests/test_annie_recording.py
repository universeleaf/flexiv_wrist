from __future__ import annotations

import io
from pathlib import Path
import shutil
import struct
import subprocess
import zipfile

import pytest

from src.annie_recording import AnnieRecordingStore, RecordingError


def frame_batch(entries: list[tuple[str, bytes]]) -> bytes:
    payload = bytearray(b"AFB1")
    payload.extend(struct.pack(">I", len(entries)))
    for name, image in entries:
        encoded_name = name.encode("ascii")
        payload.extend(struct.pack(">H", len(encoded_name)))
        payload.extend(encoded_name)
        payload.extend(struct.pack(">I", len(image)))
        payload.extend(image)
    return bytes(payload)


def test_finish_creates_three_timestamped_artifacts(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    started = store.start({"robot_sn": "Rizon4s-123456"})
    session = started["session"]
    names = [
        "frame_000000000_1000000000.jpg",
        "frame_000000001_1020000000.jpg",
    ]
    assert store.save_frame_batch(
        session,
        frame_batch([(names[0], b"jpeg-zero"), (names[1], b"jpeg-one")]),
    ) == 2
    store.save_force_csv(session, b"recording_index,camera_frame\n0,first.jpg\n")
    session_dir = Path(started["output_directory"])
    (session_dir / f"{session}_demo.mp4").write_bytes(b"test-mp4")

    result = store.finish(session)

    assert result["camera_frame_count"] == 2
    assert set(result["downloads"]) == {
        "force_torque.csv",
        "camera_frames.zip",
        "demo.mp4",
    }
    assert sorted(path.name for path in session_dir.iterdir()) == [
        f"{session}_camera_frames.zip",
        f"{session}_demo.mp4",
        f"{session}_force_torque.csv",
    ]
    with zipfile.ZipFile(session_dir / f"{session}_camera_frames.zip") as archive:
        assert archive.namelist() == names
        assert archive.read(names[1]) == b"jpeg-one"


def test_frame_batch_rejects_unsafe_filename(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    session = store.start()["session"]
    with pytest.raises(RecordingError, match="invalid camera frame filename"):
        store.save_frame_batch(session, frame_batch([("../frame.jpg", b"jpeg")]))


def test_single_frame_fallback_saves_jpeg(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    session = store.start()["session"]
    name = "frame_000000000_1000000000.jpg"

    path = store.save_frame(session, name, b"\xff\xd8jpeg-data\xff\xd9")

    assert path.name == name
    assert path.read_bytes() == b"\xff\xd8jpeg-data\xff\xd9"


def test_single_frame_fallback_rejects_non_jpeg(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    session = store.start()["session"]
    with pytest.raises(RecordingError, match="not a JPEG"):
        store.save_frame(session, "frame_000000000_1000000000.jpg", b"not-jpeg")


def test_finish_rejects_empty_camera_archive(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    started = store.start()
    session = started["session"]
    session_dir = Path(started["output_directory"])
    store.save_force_csv(session, b"recording_index,camera_frame\n")
    (session_dir / f"{session}_demo.mp4").write_bytes(b"test-mp4")

    with pytest.raises(RecordingError, match="no camera pictures"):
        store.finish(session)

    assert not (session_dir / f"{session}_camera_frames.zip").exists()


def test_dashboard_video_rejects_empty_upload(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    session = store.start()["session"]
    with pytest.raises(RecordingError, match="invalid dashboard video size"):
        store.save_dashboard_video(session, io.BytesIO(), 0)


def test_dashboard_video_accepts_browser_mp4_without_converter(tmp_path: Path) -> None:
    store = AnnieRecordingStore(tmp_path)
    session = store.start()["session"]
    payload = b"\x00\x00\x00\x18ftypisom" + b"test-browser-mp4"
    path = store.save_dashboard_video(
        session, io.BytesIO(payload), len(payload), "video/mp4;codecs=avc1"
    )
    assert path.name == f"{session}_demo.mp4"
    assert path.read_bytes() == payload


@pytest.mark.skipif(
    shutil.which("gst-launch-1.0") is None or not Path("/usr/bin/python3").is_file(),
    reason="server-side MP4 dependencies are unavailable",
)
def test_finish_can_build_full_window_mp4_from_force_and_camera_frames(
    tmp_path: Path,
) -> None:
    store = AnnieRecordingStore(tmp_path)
    started = store.start()
    session = started["session"]
    reference_jpeg = tmp_path / "reference.jpg"
    subprocess.run(
        [
            "/usr/bin/python3",
            "-c",
            (
                "from PIL import Image; import sys; "
                "Image.new('RGB',(160,90),(20,80,130)).save(sys.argv[1],'JPEG')"
            ),
            str(reference_jpeg),
        ],
        check=True,
    )
    image = reference_jpeg.read_bytes()
    entries = [
        (f"frame_{index:09d}_{1_000_000_000 + index * 20_000_000}.jpg", image)
        for index in range(20)
    ]
    store.save_frame_batch(session, frame_batch(entries))
    columns = [
        "recording_index,camera_frame,sequence,force_host_iso,force_host_unix_ns,"
        "camera_capture_iso,camera_capture_unix_ms,camera_delay_from_force_ms,"
        "elapsed_s,robot_seconds,robot_nanoseconds,tcp_fx_N,tcp_fy_N,tcp_fz_N,"
        "tcp_mx_Nm,tcp_my_Nm,tcp_mz_Nm"
    ]
    rows = [
        (
            f"{index},{name},{index},2026-08-07T12:00:00Z,"
            f"{1_000_000_000 + index * 20_000_000},2026-08-07T12:00:00Z,0,0,"
            f"{index * 0.02},1,0,{index * 0.1},1,2,0.1,0.2,0.3"
        )
        for index, (name, _) in enumerate(entries)
    ]
    store.save_force_csv(session, ("\n".join(columns + rows) + "\n").encode())

    result = store.finish(session)

    output = Path(started["output_directory"]) / f"{session}_demo.mp4"
    assert result["demo_video_source"] == "server-fallback"
    assert output.stat().st_size > 1000
    assert b"ftyp" in output.read_bytes()[:64]
