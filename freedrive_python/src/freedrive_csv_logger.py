"""CSV logger for freedrive and drift-diagnosis sessions."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Mapping, Sequence, TextIO


class FreedriveCSVLogger:
    """Write timestamped robot and state-machine diagnostics."""

    HEADER = [
        "elapsed_s",
        "state_machine",
        "enabling_button",
        "primitive_state",
        "tcp_x_m",
        "tcp_y_m",
        "tcp_z_m",
        "tcp_qw",
        "tcp_qx",
        "tcp_qy",
        "tcp_qz",
        *[f"q_{i}_rad_or_m" for i in range(1, 8)],
        *[f"dq_{i}_rad_s_or_m_s" for i in range(1, 8)],
        *[f"ext_wrench_tcp_{axis}" for axis in ("fx", "fy", "fz", "mx", "my", "mz")],
        *[f"ext_wrench_tcp_raw_{axis}" for axis in ("fx", "fy", "fz", "mx", "my", "mz")],
        *[f"tau_ext_{i}" for i in range(1, 8)],
        "fault",
        "operational",
        "connected",
        "mode",
        "active_tool",
        "tool_mass_kg",
    ]

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()

    def write_row(self, row: Mapping[str, Any]) -> None:
        values = [row.get(column, "") for column in self.HEADER]
        self._writer.writerow(values)
        self._file.flush()

    def write_states(
        self,
        *,
        elapsed_s: float,
        state_machine: str,
        enabling_button: bool,
        primitive_state: Mapping[str, Any] | str,
        tcp_pose: Sequence[float],
        q: Sequence[float],
        dq: Sequence[float],
        ext_wrench_tcp: Sequence[float],
        ext_wrench_tcp_raw: Sequence[float],
        tau_ext: Sequence[float],
        fault: bool,
        operational: bool,
        connected: bool,
        mode: str,
        active_tool: str,
        tool_mass_kg: float | str,
    ) -> None:
        if len(tcp_pose) != 7:
            raise ValueError(f"tcp_pose must have 7 values, got {len(tcp_pose)}")
        q7 = _pad(q, 7)
        dq7 = _pad(dq, 7)
        wrench = _pad(ext_wrench_tcp, 6)
        wrench_raw = _pad(ext_wrench_tcp_raw, 6)
        tau = _pad(tau_ext, 7)
        self.write_row(
            {
                "elapsed_s": elapsed_s,
                "state_machine": state_machine,
                "enabling_button": int(bool(enabling_button)),
                "primitive_state": (
                    dict(primitive_state)
                    if isinstance(primitive_state, Mapping)
                    else primitive_state
                ),
                "tcp_x_m": tcp_pose[0],
                "tcp_y_m": tcp_pose[1],
                "tcp_z_m": tcp_pose[2],
                "tcp_qw": tcp_pose[3],
                "tcp_qx": tcp_pose[4],
                "tcp_qy": tcp_pose[5],
                "tcp_qz": tcp_pose[6],
                **{f"q_{i}_rad_or_m": q7[i - 1] for i in range(1, 8)},
                **{f"dq_{i}_rad_s_or_m_s": dq7[i - 1] for i in range(1, 8)},
                **{
                    f"ext_wrench_tcp_{axis}": wrench[idx]
                    for idx, axis in enumerate(("fx", "fy", "fz", "mx", "my", "mz"))
                },
                **{
                    f"ext_wrench_tcp_raw_{axis}": wrench_raw[idx]
                    for idx, axis in enumerate(("fx", "fy", "fz", "mx", "my", "mz"))
                },
                **{f"tau_ext_{i}": tau[i - 1] for i in range(1, 8)},
                "fault": int(bool(fault)),
                "operational": int(bool(operational)),
                "connected": int(bool(connected)),
                "mode": mode,
                "active_tool": active_tool,
                "tool_mass_kg": tool_mass_kg,
            }
        )

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "FreedriveCSVLogger":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _pad(values: Sequence[float], length: int) -> list[float]:
    out = [float(v) for v in values[:length]]
    while len(out) < length:
        out.append(float("nan"))
    return out
