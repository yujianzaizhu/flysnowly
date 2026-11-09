"""割草任务报告：任务结束后在 data/data 下写一份文档，记录本次割草的各项数据。

产物（同一个时间戳）：
- ``<时间戳>/report.html`` —— 文档本体：摘要、逐区域数据、覆盖图（内嵌 PNG）、规划路径说明
- ``<时间戳>/coverage.png`` —— 覆盖图：已割/未割/障碍/纯通道分区着色，叠加沿边规划线、
  弓字规划线、实际轨迹与充电桩
- ``<时间戳>/data.json``   —— 全部原始数据（逐区域统计、覆盖栅格摘要、规划线与实际轨迹坐标）

只依赖标准库：PNG 由 zlib + struct 直接编码（aapt 没有图像库）。
"""

import base64
import json
import math
import os
import struct
import zlib
from datetime import datetime

import world

# 输出目录：默认仓库根的 data/data，可用 ROBOT_REPORT_DIR 覆盖。
# 每次任务写进该目录下以时间命名的子文件夹（默认窗口 2026-11-09_16-31-12/）。
OUTPUT_DIR = None
# 报告文件夹里的三个固定文件名
REPORT_NAME = "report"

_BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))

# 覆盖图配色（RGB）
COLOR_UNKNOWN = (245, 247, 250)
COLOR_UNMOWED = (232, 138, 122)
COLOR_MOWED = (86, 196, 138)
COLOR_OCCUPIED = (52, 61, 80)
COLOR_PASSAGE = (233, 196, 106)
COLOR_ZONE_LINE = (72, 106, 176)
COLOR_EDGE_PLAN = (114, 176, 232)
COLOR_STRIPE_PLAN = (52, 152, 219)
COLOR_TRACK = (198, 78, 46)
COLOR_DOCK = (46, 160, 96)
COLOR_BACKGROUND = (255, 255, 255)


def default_output_dir():
    return os.path.normpath(
        os.path.join(_BACKEND_DIR, os.pardir, os.pardir, "data", "data")
    )


def output_dir():
    if OUTPUT_DIR:
        return OUTPUT_DIR
    env = os.environ.get("ROBOT_REPORT_DIR")
    return os.path.normpath(env) if env else default_output_dir()


# ----------------------------------------------------------------- PNG 编码
def _chunk(tag, payload):
    body = tag + payload
    return (
        struct.pack(">I", len(payload))
        + body
        + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)
    )


def encode_png(width, height, rows):
    """rows: 每行一个 bytes（RGB 连续），返回 PNG 字节串。"""
    raw = bytearray()
    for row in rows:
        raw.append(0)  # filter type 0
        raw.extend(row)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", header)
        + _chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + _chunk(b"IEND", b"")
    )


class _Canvas(object):
    """RGB 画布：铺色块 + 画粗线。坐标单位是像素。"""

    def __init__(self, width, height, background=COLOR_BACKGROUND):
        self.width = width
        self.height = height
        self.pixels = bytearray(background * (width * height))

    def _set(self, x, y, color):
        if 0 <= x < self.width and 0 <= y < self.height:
            offset = (y * self.width + x) * 3
            self.pixels[offset : offset + 3] = bytes(color)

    def rect(self, x0, y0, x1, y1, color):
        x0 = max(0, int(x0))
        y0 = max(0, int(y0))
        x1 = min(self.width, int(x1))
        y1 = min(self.height, int(y1))
        if x1 <= x0 or y1 <= y0:
            return
        fill = bytes(color) * (x1 - x0)
        for y in range(y0, y1):
            offset = (y * self.width + x0) * 3
            self.pixels[offset : offset + len(fill)] = fill

    def line(self, x0, y0, x1, y1, color, width=1):
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        half = max(0, width // 2)
        while True:
            if width <= 1:
                self._set(x0, y0, color)
            else:
                self.rect(x0 - half, y0 - half, x0 - half + width, y0 - half + width, color)
            if x0 == x1 and y0 == y1:
                break
            err2 = 2 * err
            if err2 > -dy:
                err -= dy
                x0 += sx
            if err2 < dx:
                err += dx
                y0 += sy

    def polyline(self, points, color, width=2):
        for index in range(len(points) - 1):
            self.line(
                points[index][0],
                points[index][1],
                points[index + 1][0],
                points[index + 1][1],
                color,
                width,
            )

    def disc(self, cx, cy, radius, color):
        radius = max(1, int(radius))
        for dy in range(-radius, radius + 1):
            span = int(math.sqrt(max(0.0, radius * radius - dy * dy)))
            self.rect(cx - span, cy + dy, cx + span + 1, cy + dy + 1, color)

    def rows(self):
        stride = self.width * 3
        for y in range(self.height):
            yield bytes(self.pixels[y * stride : (y + 1) * stride])


# ----------------------------------------------------------------- 数据整理
def _zone_lines(grid):
    """区域边界：把每个区域的栅格集合按行合并成水平线段，画图时不用逐格描边。"""
    spans = {}
    cols = grid.cols
    zone_map = grid.zone_id_map
    for index, zone_id in enumerate(zone_map):
        if not zone_id:
            continue
        col = index % cols
        row = index // cols
        edge = (
            col == 0
            or col == cols - 1
            or zone_map[index - 1] != zone_id
            or zone_map[index + 1] != zone_id
            or row == 0
            or row == grid.rows - 1
            or zone_map[index - cols] != zone_id
            or zone_map[index + cols] != zone_id
        )
        if edge:
            spans.setdefault(zone_id, []).append((col, row))
    return spans


RASTER_LEGEND = {
    "M": "已割草",
    ".": "未割草（可割）",
    "O": "障碍物",
    "P": "纯通道（只通行不割草）",
    " ": "未知 / 场外",
}


def _coverage_raster(grid, limit=260000):
    """覆盖栅格的 ASCII 版（行从 y=0 起，每行 cols 个字符），方便直接比对/二次分析。"""
    if grid.cols * grid.rows > limit:
        return None
    rows = []
    for row in range(grid.rows):
        line = []
        base = row * grid.cols
        for col in range(grid.cols):
            index = base + col
            if grid.data[index] == 2:
                line.append("O")
            elif grid.no_mow[index]:
                line.append("P")
            elif grid.mowed[index]:
                line.append("M")
            elif grid.free_mask[index]:
                line.append(".")
            else:
                line.append(" ")
        rows.append("".join(line))
    return rows


def build_payload(manager):
    """把 TaskManager 里这场割草的数据整理成可直接序列化的结构。"""
    grid = manager.grid
    robot = manager.robot
    cell = grid.cell
    zones = []
    for zone in grid.zones:
        zone_id = zone["id"]
        free = grid.zone_totals.get(zone_id, 0)
        mowed = grid.zone_mowed.get(zone_id, 0)
        zones.append(
            {
                "id": zone_id,
                "name": zone.get("name") or "区域 {}".format(zone_id),
                "bbox": [
                    round(zone["x0"], 2),
                    round(zone["y0"], 2),
                    round(zone["x1"], 2),
                    round(zone["y1"], 2),
                ],
                "area": round(free * cell * cell, 2),
                "mowed_area": round(mowed * cell * cell, 2),
                "coverage": round(min(1.0, mowed / free) if free else 0.0, 4),
                "outline": [
                    [round(x, 2), round(y, 2)] for x, y in zone.get("outline") or []
                ],
            }
        )
    counts = {"mowed": 0, "unmowed": 0, "occupied": 0, "passage": 0, "unknown": 0}
    for index, value in enumerate(grid.data):
        if value == 2:
            counts["occupied"] += 1
        elif grid.no_mow[index]:
            counts["passage"] += 1
        elif grid.mowed[index]:
            counts["mowed"] += 1
        elif grid.free_mask[index]:
            counts["unmowed"] += 1
        else:
            counts["unknown"] += 1
    total_free = grid.zone_totals and sum(grid.zone_totals.values()) or 0
    mowed_total = sum(grid.zone_mowed.values()) if grid.zone_mowed else 0
    stats = dict(manager.stats or {})
    records = list(getattr(manager, "mow_zone_records", []) or [])
    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "map": {
            "id": getattr(world, "MAP_ID", None),
            "name": getattr(world, "MAP_NAME", None),
            "width": round(grid.cols * cell, 2),
            "height": round(grid.rows * cell, 2),
            "cell": cell,
        },
        "dock": {
            "x": round(world.DOCK["x"], 2),
            "y": round(world.DOCK["y"], 2),
            "r": round(world.DOCK.get("r", 0.7), 2),
        } if getattr(world, "DOCK", None) else None,
        "task": {
            "mode": getattr(manager, "mowing_mode", "A"),
            "zone_order": list(getattr(manager, "zone_order", []) or []),
            "start_sim": round(getattr(manager, "mow_start_sim", 0.0) or 0.0, 1),
            "end_sim": round(robot.sim_time, 1),
            "duration": round(
                max(0.0, robot.sim_time - (getattr(manager, "mow_start_sim", 0.0) or 0.0)), 1
            ),
        },
        "robot": {
            "distance": round(robot.distance_traveled, 2),
            "task_distance": round(
                max(
                    0.0,
                    robot.distance_traveled
                    - (getattr(manager, "mow_start_distance", 0.0) or 0.0),
                ),
                2,
            ),
            "collisions": robot.collisions,
            "task_collisions": max(
                0,
                robot.collisions - (getattr(manager, "mow_start_collisions", 0) or 0),
            ),
            "battery_start": round(
                getattr(manager, "mow_start_battery", robot.battery) or 0.0, 1
            ),
            "battery_end": round(robot.battery, 1),
        },
        "coverage": {
            "cells": counts,
            "raster": _coverage_raster(grid),
            "zone_total": int(total_free),
            "zone_mowed": int(mowed_total),
            "ratio": round(min(1.0, mowed_total / total_free) if total_free else 0.0, 4),
            "area": round(total_free * cell * cell, 2),
            "mowed_area": round(mowed_total * cell * cell, 2),
        },
        "stats": stats,
        "zones": zones,
        "zone_records": records,
        "plan": {
            "edges": [
                {
                    "zone": zone_id,
                    "points": [[round(x, 3), round(y, 3)] for x, y in points],
                }
                for zone_id, points in sorted(
                    (getattr(manager, "mow_plan_edges", {}) or {}).items()
                )
            ],
            "stripes": [
                [
                    [round(a[0], 3), round(a[1], 3)],
                    [round(b[0], 3), round(b[1], 3)],
                ]
                for a, b in (getattr(manager, "mow_plan_segments", {}) or {}).values()
            ],
        },
        "track": [
            [round(x, 3), round(y, 3)]
            for x, y in (getattr(manager, "mow_track", []) or [])
        ],
    }


def render_map_png(grid, payload, scale=None):
    """把覆盖情况 + 规划路径 + 实际轨迹画成 PNG。"""
    cell = grid.cell
    if scale is None:
        pixels = grid.cols * grid.rows
        scale = 2 if pixels <= 400000 else 1
    width = max(1, grid.cols * scale)
    height = max(1, grid.rows * scale)
    canvas = _Canvas(width, height, COLOR_UNKNOWN)

    # 栅格底色：按行合并同类格，减少绘制次数
    cols = grid.cols
    rows = grid.rows
    no_mow = grid.no_mow
    mowed = grid.mowed
    free = grid.free_mask
    data = grid.data
    for row in range(rows):
        col = 0
        base = row * cols
        while col < cols:
            index = base + col
            if data[index] == 2:
                color = COLOR_OCCUPIED
            elif no_mow[index]:
                color = COLOR_PASSAGE
            elif mowed[index]:
                color = COLOR_MOWED
            elif free[index]:
                color = COLOR_UNMOWED
            else:
                color = COLOR_UNKNOWN
            span = 1
            while col + span < cols:
                nindex = index + span
                if data[nindex] == 2:
                    other = COLOR_OCCUPIED
                elif no_mow[nindex]:
                    other = COLOR_PASSAGE
                elif mowed[nindex]:
                    other = COLOR_MOWED
                elif free[nindex]:
                    other = COLOR_UNMOWED
                else:
                    other = COLOR_UNKNOWN
                if other != color:
                    break
                span += 1
            # 栅格第 0 行在场地 y=0（下方），画到图像上要翻过来
            top = height - (row + 1) * scale
            canvas.rect(col * scale, top, (col + span) * scale, top + scale, color)
            col += span

    # 区域边界
    for zone_id, cells in _zone_lines(grid).items():
        for col, row in cells:
            top = height - (row + 1) * scale
            canvas.rect(col * scale, top, (col + 1) * scale, top + scale, COLOR_ZONE_LINE)

    def to_pixel(point):
        return (point[0] / cell * scale, height - point[1] / cell * scale)

    # 沿边 + 弓字规划线
    for item in payload["plan"]["edges"]:
        canvas.polyline([to_pixel(point) for point in item["points"]], COLOR_EDGE_PLAN, max(1, scale // 2))
    for segment in payload["plan"]["stripes"]:
        canvas.polyline([to_pixel(point) for point in segment], COLOR_STRIPE_PLAN, 1)

    # 实际轨迹
    if len(payload["track"]) > 1:
        canvas.polyline([to_pixel(point) for point in payload["track"]], COLOR_TRACK, max(1, scale // 2))

    # 充电桩
    dock = payload.get("dock")
    if dock:
        cx, cy = to_pixel((dock["x"], dock["y"]))
        canvas.disc(cx, cy, max(3, int(0.5 / cell * scale / 2)), COLOR_DOCK)

    return encode_png(width, height, canvas.rows()), width, height


def _table(headers, rows):
    head = "".join("<th>{}</th>".format(item) for item in headers)
    body = "".join(
        "<tr>{}</tr>".format("".join("<td>{}</td>".format(cell) for cell in row))
        for row in rows
    )
    return "<table><thead><tr>{}</tr></thead><tbody>{}</tbody></table>".format(head, body)


def render_html(payload, png_bytes, file_stem):
    png_b64 = base64.b64encode(png_bytes).decode("ascii")
    task = payload["task"]
    coverage = payload["coverage"]
    robot = payload["robot"]
    records = payload["zone_records"] or payload["zones"]
    summary_rows = [
        ["地图", "{}（{}）".format(payload["map"]["name"] or "未知", payload["map"]["id"])],
        ["割草模式", "{} 模式".format(task["mode"])],
        ["区域顺序", "、".join(str(item) for item in task["zone_order"]) or "-"],
        ["任务时长", "{:.0f} s".format(task["duration"])],
        ["行驶里程", "{:.1f} m".format(robot["task_distance"])],
        ["碰撞次数", str(robot["task_collisions"])],
        ["电量", "{:.1f}% → {:.1f}%".format(robot["battery_start"], robot["battery_end"])],
        ["区域总面积", "{:.1f} m²".format(coverage["area"])],
        ["已割面积", "{:.1f} m²".format(coverage["mowed_area"])],
        [
            "总覆盖率",
            "{:.1f}%（{}/{} 格）".format(
                coverage["ratio"] * 100.0, coverage["zone_mowed"], coverage["zone_total"]
            ),
        ],
        [
            "绕障 / 跳过边缘",
            "{} / {}".format(payload["stats"].get("rings", 0), payload["stats"].get("edge_skips", 0)),
        ],
        [
            "规划弓字段",
            "{} 条（沿边 {} 段）".format(
                len(payload["plan"]["stripes"]), len(payload["plan"]["edges"])
            ),
        ],
        ["轨迹点", "{} 个".format(len(payload["track"]))],
    ]
    zone_rows = [
        [
            record.get("name") or record["id"],
            "{:.1f}".format(record.get("area", 0.0)),
            "{:.1f}".format(record.get("mowed_area", 0.0)),
            "{:.1f}%".format(record.get("coverage", 0.0) * 100.0),
            "{:.0f} s".format(record.get("seconds", 0.0)) if "seconds" in record else "-",
            record.get("lane_axis", "-"),
            record.get("rings", "-"),
            record.get("segments_planned", "-"),
        ]
        for record in records
    ]
    cells = coverage["cells"]
    raster = coverage.get("raster")
    legend = [
        ("已割草", COLOR_MOWED),
        ("未割草", COLOR_UNMOWED),
        ("障碍物", COLOR_OCCUPIED),
        ("纯通道（不割）", COLOR_PASSAGE),
        ("区域边界", COLOR_ZONE_LINE),
        ("沿边规划", COLOR_EDGE_PLAN),
        ("弓字规划", COLOR_STRIPE_PLAN),
        ("实际轨迹", COLOR_TRACK),
        ("充电桩", COLOR_DOCK),
    ]
    legend_html = "".join(
        '<span class="swatch"><i style="background:#{:02x}{:02x}{:02x}"></i>{}</span>'.format(
            color[0], color[1], color[2], name
        )
        for name, color in legend
    )
    return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8" />
<title>割草任务报告 · {title}</title>
<style>
  :root {{ color-scheme: light; }}
  body {{ margin: 0; padding: 26px 32px 40px; font: 14px/1.6 "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
         background: #f4f6fa; color: #1f2733; }}
  h1 {{ font-size: 21px; margin: 0 0 4px; }}
  h2 {{ font-size: 15px; margin: 26px 0 8px; color: #2c3e50; }}
  .meta {{ color: #63708a; font-size: 12px; margin-bottom: 18px; }}
  .card {{ background: #fff; border: 1px solid #e1e6ef; border-radius: 10px; padding: 14px 18px; margin-bottom: 18px; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  th, td {{ text-align: left; padding: 6px 10px; border-bottom: 1px solid #edf0f6; }}
  th {{ color: #5b6880; font-weight: 600; background: #f8fafd; }}
  tbody tr:last-child td {{ border-bottom: none; }}
  .swatch {{ display: inline-flex; align-items: center; gap: 6px; margin-right: 16px; font-size: 12px; color: #45536b; }}
  .swatch i {{ width: 12px; height: 12px; border-radius: 3px; display: inline-block; border: 1px solid rgba(0,0,0,.12); }}
  figure {{ margin: 0; }}
  figure img {{ width: 100%; height: auto; border-radius: 8px; border: 1px solid #e1e6ef; image-rendering: pixelated; }}
  figcaption {{ margin-top: 8px; color: #63708a; font-size: 12px; }}
  code {{ background: #eef1f7; padding: 1px 5px; border-radius: 4px; font-size: 12px; }}
</style>
</head>
<body>
<h1>割草任务报告</h1>
<div class="meta">生成时间 {generated} · 数据文件 <code>data.json</code> · 覆盖图 <code>coverage.png</code>（与本页同一文件夹）</div>

<div class="card">
  <h2 style="margin-top:0">任务摘要</h2>
  {summary}
</div>

<div class="card">
  <h2 style="margin-top:0">逐区域数据</h2>
  {zones}
</div>

<div class="card">
  <h2 style="margin-top:0">覆盖图与规划路径</h2>
  <p style="margin:0 0 10px">{legend}</p>
  <figure>
    <img alt="覆盖图" src="data:image/png;base64,{png}" />
    <figcaption>绿色=已割，红色=漏割，深色=障碍物，黄色=纯通道（只通行不割草）；
      浅蓝线=沿边规划，蓝线=弓字规划，砖红线=实际行驶轨迹，绿点=充电桩。</figcaption>
  </figure>
</div>

<div class="card">
  <h2 style="margin-top:0">统计</h2>
  {stats}
</div>
</body>
</html>
""".format(
        title=payload["map"]["name"] or "割草",
        generated=payload["generated_at"],
        stem=file_stem,
        summary=_table(["项目", "数值"], summary_rows),
        zones=_table(
            ["区域", "面积 (m²)", "已割 (m²)", "覆盖率", "用时", "主方向", "绕障", "弓字段"],
            zone_rows,
        ),
        legend=legend_html,
        png=png_b64,
        stats=_table(
            ["项目", "数值"],
            [
                ["未知/障碍/通道格", "{} / {} / {}".format(cells["unknown"], cells["occupied"], cells["passage"])],
                ["覆盖栅格", "已写入 JSON（{} 行 × {} 列，字符含义见下一行）".format(
                    len(raster), len(raster[0]) if raster else 0
                ) if raster else "地图过大，未写入 JSON（看 PNG 覆盖图）"],
                ["栅格字符", " ".join(
                    "{}={}".format(key, value) for key, value in sorted(RASTER_LEGEND.items())
                )],
                ["充电次数", payload["stats"].get("charge_count", 0)],
                ["脱困次数", payload["stats"].get("stuck_recoveries", 0)],
                ["大小障碍重规划", payload["stats"].get("large_replans", 0)],
            ],
        ),
    )


def generate(manager, output=None):
    """写一份割草报告，返回 {"html":..., "png":..., "json":..., "payload":...}。

    manager 需要提供 grid / robot / stats / mow_* 记录字段（TaskManager 满足）。
    """
    grid = manager.grid
    target_dir = output_dir() if output is None else output
    os.makedirs(target_dir, exist_ok=True)
    payload = build_payload(manager)
    png_bytes, width, height = render_map_png(grid, payload)
    payload["coverage_map"] = {"width": width, "height": height, "image": "png"}

    now = datetime.now()
    # 每次任务一个以时间命名的文件夹，HTML / PNG / JSON 三件套放进去，
    # 一眼就能对应到某一次割草（2026-11-09_16-31-12/report.html …）。
    folder = now.strftime("%Y-%m-%d_%H-%M-%S")
    report_dir = os.path.join(target_dir, folder)
    suffix = 2
    while os.path.isdir(report_dir):
        report_dir = os.path.join(target_dir, "{}_{}".format(folder, suffix))
        suffix += 1
    os.makedirs(report_dir, exist_ok=True)
    stem = REPORT_NAME
    html_path = os.path.join(report_dir, "report.html")
    png_path = os.path.join(report_dir, "coverage.png")
    json_path = os.path.join(report_dir, "data.json")
    with open(png_path, "wb") as handle:
        handle.write(png_bytes)
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(render_html(payload, png_bytes, stem))
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    return {
        "dir": report_dir,
        "folder": os.path.basename(report_dir),
        "html": html_path,
        "png": png_path,
        "json": json_path,
        "stem": stem,
        "payload": payload,
    }
