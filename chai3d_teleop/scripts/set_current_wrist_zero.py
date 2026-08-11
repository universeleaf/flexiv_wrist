#!/usr/bin/env python3
"""Save the two stopped moteus positions as the application's wrist zero.

This utility never enables motor motion and never writes persistent moteus
firmware configuration.  It sends STOP, queries both axes, and atomically
updates ``wrist.zero_position_rev`` in the 9-DoF TOML profile.  A timestamped
backup is always created before applying the change.
"""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
import tomllib
from typing import Any, Sequence

try:
    import moteus
except ImportError as exc:
    raise SystemExit(
        "请使用项目的 .venv_moteus/bin/python 运行本脚本"
    ) from exc


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"
CONFIRMATION = "SET_CURRENT_WRIST_ZERO"
ZERO_PATTERN = re.compile(
    r"(?m)^(?P<prefix>\s*zero_position_rev\s*=\s*)\[[^\]\n]*\](?P<suffix>\s*)$"
)


def _load_wrist_config(path: Path) -> tuple[dict[str, Any], list[int]]:
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    wrist = document.get("wrist")
    if not isinstance(wrist, dict):
        raise ValueError(f"配置缺少 [wrist]: {path}")
    ids_raw = wrist.get("ids")
    if (
        not isinstance(ids_raw, list)
        or len(ids_raw) != 2
        or not all(isinstance(value, int) and value > 0 for value in ids_raw)
        or ids_raw[0] == ids_raw[1]
    ):
        raise ValueError("wrist.ids 必须是两个不同的正整数")
    return wrist, [int(value) for value in ids_raw]


def _apply_transport_defaults(args: argparse.Namespace, wrist: dict[str, Any]) -> None:
    if getattr(args, "fdcanusb", None) is None:
        # moteus declares --fdcanusb with action="append", so the transport
        # factory requires a list of paths. Assigning a plain string makes it
        # iterate characters and try to open "/" first.
        device = str(wrist.get("fdcanusb", "")).strip()
        if not device:
            raise ValueError("wrist.fdcanusb 不能为空")
        args.fdcanusb = [device]


def _replace_zero_positions(text: str, positions_rev: Sequence[float]) -> str:
    if len(positions_rev) != 2 or not all(
        math.isfinite(float(value)) for value in positions_rev
    ):
        raise ValueError("必须提供两个有限的原始位置")
    replacement_values = ", ".join(f"{float(value):.9f}" for value in positions_rev)
    updated, count = ZERO_PATTERN.subn(
        lambda match: (
            f"{match.group('prefix')}[{replacement_values}]"
            f"{match.group('suffix')}"
        ),
        text,
    )
    if count != 1:
        raise ValueError(
            f"配置中应恰好有一个 wrist.zero_position_rev，实际匹配 {count} 个"
        )
    # Validate the complete result before it can replace the live profile.
    tomllib.loads(updated)
    return updated


def _atomic_update(path: Path, updated: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = path.with_name(f"{path.name}.bak-{timestamp}-{os.getpid()}")
    original = path.read_bytes()
    backup.write_bytes(original)
    original_mode = stat.S_IMODE(path.stat().st_mode)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_name, original_mode)
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
    return backup


def _value(state: Any, register: Any, default: Any = None) -> Any:
    return state.values.get(register, default)


async def _stop_all(controllers: Sequence[Any]) -> None:
    for controller in controllers:
        try:
            await asyncio.wait_for(controller.set_stop(), timeout=2.0)
        except BaseException as error:
            print(
                f"警告：STOP 未确认：{type(error).__name__}: {error}",
                file=sys.stderr,
            )


async def _read_stopped_positions(args: argparse.Namespace, ids: Sequence[int]) -> list[float]:
    transport = moteus.get_singleton_transport(args)
    controllers = [
        moteus.Controller(id=motor_id, transport=transport) for motor_id in ids
    ]
    try:
        await _stop_all(controllers)
        positions: list[float] = []
        for motor_id, controller in zip(ids, controllers):
            state = await asyncio.wait_for(controller.query(), timeout=2.0)
            if state is None:
                raise RuntimeError(f"电机 ID {motor_id} 没有响应")
            position = float(_value(state, moteus.Register.POSITION, math.nan))
            velocity = float(_value(state, moteus.Register.VELOCITY, math.nan))
            mode_raw = _value(state, moteus.Register.MODE)
            fault_raw = _value(state, moteus.Register.FAULT)
            mode = None if mode_raw is None else int(mode_raw)
            fault = None if fault_raw is None else int(fault_raw)
            if not math.isfinite(position) or not math.isfinite(velocity):
                raise RuntimeError(f"电机 ID {motor_id} 返回无效位置/速度")
            if fault not in (None, 0) or mode == 1:
                raise RuntimeError(
                    f"电机 ID {motor_id} 状态异常：mode={mode}, fault={fault}"
                )
            positions.append(position)
            print(
                f"ID {motor_id}: raw={position:+.9f} rev, "
                f"velocity={velocity * 360.0:+.3f} deg/s, "
                f"mode={mode}, fault={fault}"
            )
        return positions
    finally:
        print("再次向两台电机发送 STOP；本脚本从未使能运动。")
        try:
            await _stop_all(controllers)
        finally:
            transport.close()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--confirm-set-zero",
        default="",
        help=f"实际保存必须填写 {CONFIRMATION}；不填写时只预览",
    )
    moteus.make_transport_args(parser)
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    try:
        wrist, ids = _load_wrist_config(config_path)
        _apply_transport_defaults(args, wrist)
        print("当前零位采集：只发送 STOP 和 QUERY，不会命令任何电机运动。")
        print(
            "执行前必须把 joint 8 和 joint 9 手动放在你希望定义为机械/几何零位的姿态。"
        )
        positions = asyncio.run(_read_stopped_positions(args, ids))
        original = config_path.read_text(encoding="utf-8")
        updated = _replace_zero_positions(original, positions)
        print(
            "拟保存 wrist.zero_position_rev = [{}]（顺序 ID {}）".format(
                ", ".join(f"{value:+.9f}" for value in positions),
                ", ".join(str(value) for value in ids),
            )
        )
        if args.confirm_set_zero != CONFIRMATION:
            print("预览完成：配置未修改。")
            print(
                "确认当前姿态确实是两关节机械零位后，加参数：\n"
                f"  --confirm-set-zero {CONFIRMATION}"
            )
            return 0
        backup = _atomic_update(config_path, updated)
        print(f"已更新配置：{config_path}")
        print(f"旧配置备份：{backup}")
        print("新的应用层关节状态为 q8=0.0deg、q9=0.0deg。")
        print("没有修改 moteus 的永久编码器配置，也没有移动电机。")
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C；已请求 STOP，配置未修改。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误：{type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
