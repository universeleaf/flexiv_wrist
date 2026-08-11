#!/usr/bin/env python3
"""Render a full Annie camera + force view without browser/player chrome."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


WIDTH, HEIGHT = 1280, 720
COLORS = ("#49d9df", "#ff9e52", "#5d8dff")
FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")


def get_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(str(FONT_DIR / name), size)
    except OSError:
        return ImageFont.load_default()


def value(row: dict[str, str], field: str) -> float:
    try:
        return float(row.get(field, "0"))
    except (TypeError, ValueError):
        return 0.0


def frame_indices(rows: list[dict[str, str]], fps: int) -> list[int]:
    if not rows:
        return []
    if len(rows) == 1:
        return [0]
    times = [value(row, "elapsed_s") for row in rows]
    if times[-1] <= times[0]:
        return list(range(len(rows)))
    result: list[int] = []
    cursor = 0
    target = times[0]
    step = 1.0 / fps
    while target <= times[-1] + step / 2:
        while (
            cursor + 1 < len(times)
            and abs(times[cursor + 1] - target) <= abs(times[cursor] - target)
        ):
            cursor += 1
        if not result or result[-1] != cursor:
            result.append(cursor)
        target += step
    return result


def panel(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int]) -> None:
    draw.rectangle(box, fill="#101720", outline="#293542", width=2)


def camera_frame(path: Path, size: tuple[int, int]) -> Image.Image:
    try:
        with Image.open(path) as source:
            return ImageOps.contain(
                ImageOps.exif_transpose(source).convert("RGB"),
                size,
                Image.Resampling.LANCZOS,
            )
    except (OSError, ValueError):
        return Image.new("RGB", size, "#030507")


def chart(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    rows: list[dict[str, str]],
    end: int,
    fields: tuple[str, str, str],
    title: str,
    unit: str,
    minimum: float,
) -> None:
    left, top, right, bottom = box
    panel(draw, box)
    draw.text((left + 14, top + 8), title, fill="#dce5ec", font=get_font(13, True))
    draw.text((right - 12, top + 8), unit, fill="#7f8b97", font=get_font(11), anchor="ra")
    plot_left, plot_top, plot_right, plot_bottom = left + 42, top + 32, right - 12, bottom - 15
    end_time = value(rows[end], "elapsed_s")
    start = end
    while start > 0 and value(rows[start - 1], "elapsed_s") >= end_time - 10:
        start -= 1
    visible = rows[start : end + 1]
    peak = minimum
    for row in visible:
        peak = max(peak, *(abs(value(row, field)) for field in fields))
    peak *= 1.12
    for line in range(5):
        y = plot_top + (plot_bottom - plot_top) * line / 4
        draw.line((plot_left, y, plot_right, y), fill="#26313c")
    start_time = value(visible[0], "elapsed_s")
    span = max(0.001, end_time - start_time)
    for axis, field in enumerate(fields):
        points = []
        for row in visible:
            x = plot_left + (value(row, "elapsed_s") - start_time) / span * (plot_right - plot_left)
            y = plot_top + (1 - (value(row, field) + peak) / (2 * peak)) * (plot_bottom - plot_top)
            points.append((x, y))
        if len(points) > 1:
            draw.line(points, fill=COLORS[axis], width=2)


def render(
    row: dict[str, str],
    rows: list[dict[str, str]],
    index: int,
    camera_dir: Path,
) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), "#080c11")
    draw = ImageDraw.Draw(image)
    draw.text((22, 12), "Annie · Vision & Force Monitor", fill="#f3f6f8", font=get_font(20, True))
    draw.text((WIDTH - 22, 15), row.get("force_host_iso", ""), fill="#a8b4bf", font=get_font(11), anchor="ra")

    panel(draw, (18, 46, 835, 505))
    camera = camera_frame(camera_dir / row.get("camera_frame", ""), (797, 439))
    image.paste(camera, (28 + (797 - camera.width) // 2, 56 + (439 - camera.height) // 2))

    panel(draw, (853, 46, 1262, 505))
    force = [value(row, key) for key in ("tcp_fx_N", "tcp_fy_N", "tcp_fz_N")]
    moment = [value(row, key) for key in ("tcp_mx_Nm", "tcp_my_Nm", "tcp_mz_Nm")]
    draw.text((878, 73), "TCP WRENCH", fill="#49d9df", font=get_font(15, True))
    draw.text((878, 108), f"{sum(v*v for v in force)**0.5:.2f} N", fill="#f3f6f8", font=get_font(35, True))
    for axis, label in enumerate(("Fx", "Fy", "Fz")):
        draw.text((878, 168 + axis * 33), f"{label}  {force[axis]: .3f} N", fill="#a9b5bf", font=get_font(17))
    draw.text((878, 288), f"{sum(v*v for v in moment)**0.5:.3f} Nm", fill="#ff9e52", font=get_font(28, True))
    for axis, label in enumerate(("Mx", "My", "Mz")):
        draw.text((878, 337 + axis * 31), f"{label}  {moment[axis]: .4f} Nm", fill="#a9b5bf", font=get_font(16))
    draw.text((878, 466), f"Sample {row.get('sequence', '—')} · {value(row, 'elapsed_s'):.3f} s", fill="#71808d", font=get_font(11))

    chart(draw, (18, 520, 630, 705), rows, index, ("tcp_fx_N", "tcp_fy_N", "tcp_fz_N"), "FORCE · Fx / Fy / Fz", "N", 5.0)
    chart(draw, (650, 520, 1262, 705), rows, index, ("tcp_mx_Nm", "tcp_my_Nm", "tcp_mz_Nm"), "MOMENT · Mx / My / Mz", "Nm", 0.5)
    # Never draw the recording session name or output MP4 filename.
    return image


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--camera-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--fps", type=int, default=25)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with args.csv.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    indices = frame_indices(rows, args.fps)
    if not indices:
        raise SystemExit("force CSV contains no samples")
    for output_index, row_index in enumerate(indices):
        render(rows[row_index], rows, row_index, args.camera_dir).save(
            args.output_dir / f"dashboard_{output_index:09d}.jpg",
            quality=88,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
