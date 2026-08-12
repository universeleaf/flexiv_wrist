#!/usr/bin/env python3
"""Read-only wrist encoder direction check while an operator moves one joint.

The selected moteus is kept in STOP.  No position, velocity, current, or
torque command is ever sent.  Move only the selected physical wrist joint
slowly by hand during the sampling interval.
"""

from __future__ import annotations

import argparse
import asyncio
import math
from pathlib import Path
import sys
import time
import tomllib
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"


def _shortest_periodic_delta(current: float, previous: float) -> float:
    """Return a one-turn encoder increment in [-0.5, 0.5)."""
    delta = current - previous
    return delta - math.floor(delta + 0.5)


def _sign(value: float, *, minimum: float) -> int:
    if abs(value) < minimum:
        return 0
    return 1 if value > 0.0 else -1


def _load(path: Path, target: int) -> tuple[float, str]:
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    wrist = document["wrist"]
    ids = [int(value) for value in wrist["ids"]]
    ratios = [float(value) for value in wrist["reduction_ratio"]]
    if target not in ids:
        raise ValueError(f"ID {target} 不在 wrist.ids={ids} 中")
    return ratios[ids.index(target)], str(wrist["fdcanusb"])


def _required(state: object, register: Any) -> float:
    value = state.values.get(register)  # type: ignore[attr-defined]
    if value is None or not math.isfinite(float(value)):
        raise RuntimeError(f"moteus 没有返回寄存器 {register.name}")
    return float(value)


async def _run(args: argparse.Namespace) -> int:
    import moteus

    ratio, configured_device = _load(args.config.expanduser().resolve(), args.target)
    if args.fdcanusb is None:
        args.fdcanusb = configured_device

    query_resolution = moteus.QueryResolution()
    query_resolution._extra = {
        moteus.Register.ENCODER_0_POSITION: moteus.F32,
        moteus.Register.ENCODER_1_POSITION: moteus.F32,
        moteus.Register.ENCODER_0_VELOCITY: moteus.F32,
        moteus.Register.ENCODER_1_VELOCITY: moteus.F32,
    }
    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(
        id=args.target,
        transport=transport,
        query_resolution=query_resolution,
        can_prefix=args.can_prefix,
    )
    try:
        await asyncio.wait_for(controller.set_stop(), timeout=2.0)
        print(
            f"ID {args.target} 保持 STOP；不会发送任何运动/电流/力矩命令。",
            flush=True,
        )
        print(
            f"准备 {args.prepare_s:g} 秒后开始采样。采样期间只用手缓慢转动该物理关节，"
            "总移动量建议 10–30°；方向任选。",
            flush=True,
        )
        await asyncio.sleep(args.prepare_s)

        initial = await asyncio.wait_for(controller.query(), timeout=2.0)
        if initial is None:
            raise RuntimeError("初始 QUERY 无响应")
        output_start = _required(initial, moteus.Register.POSITION)
        previous_encoder0 = _required(initial, moteus.Register.ENCODER_0_POSITION)
        previous_encoder1 = _required(initial, moteus.Register.ENCODER_1_POSITION)
        accumulated_encoder0 = 0.0
        accumulated_encoder1 = 0.0
        latest_output = output_start
        started = time.monotonic()
        next_status = started
        period = 1.0 / args.rate_hz

        while time.monotonic() - started < args.seconds:
            cycle = time.monotonic()
            state = await asyncio.wait_for(controller.query(), timeout=2.0)
            if state is None:
                raise RuntimeError("采样 QUERY 无响应")
            latest_output = _required(state, moteus.Register.POSITION)
            encoder0 = _required(state, moteus.Register.ENCODER_0_POSITION)
            encoder1 = _required(state, moteus.Register.ENCODER_1_POSITION)
            accumulated_encoder0 += _shortest_periodic_delta(
                encoder0, previous_encoder0
            )
            accumulated_encoder1 += _shortest_periodic_delta(
                encoder1, previous_encoder1
            )
            previous_encoder0 = encoder0
            previous_encoder1 = encoder1
            now = time.monotonic()
            if now >= next_status:
                print(
                    "elapsed_s={:.1f} output_delta_deg={:+.2f} "
                    "encoder0_mapped_deg={:+.2f} encoder1_deg={:+.2f}".format(
                        now - started,
                        (latest_output - output_start) * 360.0,
                        accumulated_encoder0 * ratio * 360.0,
                        accumulated_encoder1 * 360.0,
                    ),
                    flush=True,
                )
                next_status = now + 0.5
            await asyncio.sleep(max(0.0, period - (time.monotonic() - cycle)))

        output_delta_deg = (latest_output - output_start) * 360.0
        encoder0_mapped_deg = accumulated_encoder0 * ratio * 360.0
        encoder1_deg = accumulated_encoder1 * 360.0
        print(
            "RESULT ID{} output_delta_deg={:+.3f} encoder0_mapped_deg={:+.3f} "
            "encoder1_deg={:+.3f}".format(
                args.target,
                output_delta_deg,
                encoder0_mapped_deg,
                encoder1_deg,
            )
        )
        output_sign = _sign(output_delta_deg, minimum=1.0)
        encoder0_sign = _sign(encoder0_mapped_deg, minimum=1.0)
        encoder1_sign = _sign(encoder1_deg, minimum=1.0)
        if 0 in (output_sign, encoder0_sign, encoder1_sign):
            print("INCONCLUSIVE：至少一个读数变化不足 1°；请增加手动移动量后重试。")
            return 2
        if output_sign == encoder0_sign == encoder1_sign:
            print("PASS：输出、source 0 和 source 1 的逻辑方向一致。")
            return 0
        print(
            "FAIL：双编码器逻辑方向不一致；不要继续位置模式或惯量辨识，"
            "也不要直接删除跟随误差保护。"
        )
        return 3
    finally:
        try:
            await asyncio.wait_for(controller.set_stop(), timeout=2.0)
        finally:
            if hasattr(transport, "close"):
                transport.close()


def main() -> int:
    try:
        import moteus
    except ImportError as error:
        print(
            "错误: 请使用 .venv_moteus/bin/python 运行本脚本",
            file=sys.stderr,
        )
        return 1

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--target", type=int, choices=(1, 2), required=True)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--prepare-s", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--can-prefix", type=lambda value: int(value, 0), default=0)
    moteus.make_transport_args(parser)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 3.0:
        parser.error("--seconds 必须至少为 3 秒")
    if not math.isfinite(args.prepare_s) or args.prepare_s < 0.0:
        parser.error("--prepare-s 不能为负数")
    if not math.isfinite(args.rate_hz) or not 10.0 <= args.rate_hz <= 100.0:
        parser.error("--rate-hz 必须在 10..100 Hz")
    try:
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C；已发送 STOP。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
