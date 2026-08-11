#!/usr/bin/env python3
"""Recover a camera-only JPEG ZIP from an Annie dashboard MP4.

This is intended only for recordings made before reliable JPEG upload was added.
The recovered frame names are taken from the force CSV so its alignment references
remain usable. Video frames are matched proportionally across the recording.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile


FRAME_PATTERN = re.compile(r"frame_\d{9}_\d+\.jpg")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("session_dir", type=Path)
    result.add_argument(
        "--output",
        type=Path,
        help="Output ZIP (default: <session>_camera_frames_recovered.zip)",
    )
    return result


def referenced_frames(csv_path: Path) -> list[str]:
    names: list[str] = []
    seen: set[str] = set()
    with csv_path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            name = row.get("camera_frame", "")
            if name and name not in seen:
                if not FRAME_PATTERN.fullmatch(name):
                    raise RuntimeError(f"unsafe camera frame name in CSV: {name!r}")
                seen.add(name)
                names.append(name)
    if not names:
        raise RuntimeError("force CSV does not reference any camera frames")
    return names


def main() -> int:
    args = parser().parse_args()
    session_dir = args.session_dir.expanduser().resolve()
    session = session_dir.name
    csv_path = session_dir / f"{session}_force_torque.csv"
    video_path = session_dir / f"{session}_demo.mp4"
    output_path = args.output or session_dir / f"{session}_camera_frames_recovered.zip"
    if not csv_path.is_file() or not video_path.is_file():
        raise RuntimeError("the session must contain its force CSV and demo MP4")
    gst_launch = shutil.which("gst-launch-1.0")
    if gst_launch is None:
        raise RuntimeError("gst-launch-1.0 is required")

    names = referenced_frames(csv_path)
    with tempfile.TemporaryDirectory(prefix="annie-camera-recovery-") as temporary:
        temp_dir = Path(temporary)
        command = [
            gst_launch,
            "-q",
            "filesrc",
            f"location={video_path}",
            "!",
            "qtdemux",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
            "!",
            "videocrop",
            "left=50",
            "right=710",
            "top=120",
            "bottom=330",
            "!",
            "videoconvert",
            "!",
            "jpegenc",
            "quality=90",
            "!",
            "multifilesink",
            f"location={temp_dir / 'source_%09d.jpg'}",
        ]
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        sources = sorted(temp_dir.glob("source_*.jpg"))
        if completed.returncode != 0 or not sources:
            raise RuntimeError(f"camera extraction failed: {completed.stderr.strip()}")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(
            output_path, "w", compression=zipfile.ZIP_STORED, allowZip64=True
        ) as archive:
            for index, name in enumerate(names):
                if len(names) == 1:
                    source_index = 0
                else:
                    source_index = round(index * (len(sources) - 1) / (len(names) - 1))
                archive.write(sources[source_index], arcname=name)

    with zipfile.ZipFile(output_path) as archive:
        if len(archive.namelist()) != len(names) or archive.testzip() is not None:
            raise RuntimeError("recovered ZIP verification failed")
    print(f"Recovered {len(names)} camera-only pictures: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
