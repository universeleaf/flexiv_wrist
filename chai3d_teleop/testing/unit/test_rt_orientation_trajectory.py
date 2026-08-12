from __future__ import annotations

import csv
from pathlib import Path

import pytest

from tools.osc_launcher import write_python_orientation_dry_run


def test_orientation_trajectory_fixes_position_and_closes(tmp_path: Path) -> None:
    output = tmp_path / "orientation.csv"
    write_python_orientation_dry_run(output, 20.0, 15.0)

    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    assert rows
    for axis in ("x_m", "y_m", "z_m"):
        assert {float(row[axis]) for row in rows} == {0.0}
    for axis in ("tilt_x_deg", "tilt_y_deg", "tilt_z_deg"):
        assert float(rows[0][axis]) == pytest.approx(float(rows[-1][axis]), abs=1e-10)
    assert max(abs(float(row["tilt_y_deg"])) for row in rows) == pytest.approx(15.0)
