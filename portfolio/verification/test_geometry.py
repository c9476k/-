#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景几何校验（纯 Python，不启动 ROS/Gazebo）。

校验内容：
    1. 10 个方块的任意两块间距 >= 2.0 m（项目要求）
    2. 所有方块与放置区都落在静态地图覆盖范围内
    3. 放置区底板尺寸与"无碰撞"结论（解释方块落地会陷 2cm 的现象）

数据来源（优先读 portfolio/src，上传到 GitHub 后只有这份）：
    src/yzbot/scripts/positions.py                     由 scan_positions.py 生成
    src/yzbot/bot_navigation/maps/mapn3.yaml / .pgm     静态栅格地图
    src/yzbot/mybot_description/worlds/gazebo_world3.new Gazebo 场景

运行：python3 test_geometry.py
退出码：0 全部通过，1 有失败。
"""

import importlib.util
import itertools
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
# 优先用 portfolio/src 里的源码副本，本地开发时回退到原工程根目录
_CANDIDATES = [HERE.parent / "src", HERE.parents[2]]
ROOT = next(
    (p for p in _CANDIDATES
     if (p / "yzbot" / "scripts" / "positions.py").is_file()),
    _CANDIDATES[0])

POSITIONS = ROOT / "yzbot" / "scripts" / "positions.py"
MAP_YAML = ROOT / "yzbot" / "bot_navigation" / "maps" / "mapn3.yaml"
MAP_PGM = ROOT / "yzbot" / "bot_navigation" / "maps" / "mapn3.pgm"
WORLD = ROOT / "yzbot" / "mybot_description" / "worlds" / "gazebo_world3.new"

MIN_SPACING = 2.0


def load_positions():
    spec = importlib.util.spec_from_file_location("positions", POSITIONS)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.block_positions, module.zone_positions


def read_map_meta():
    """读 map 的 resolution / origin（不依赖 PyYAML）。"""
    meta = {}
    for line in MAP_YAML.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key, value = key.strip(), value.strip()
        if key == "resolution":
            meta["resolution"] = float(value)
        elif key == "origin":
            meta["origin"] = [float(v) for v in
                              value.strip("[]").split(",")[:2]]
    return meta


def read_pgm_size():
    """从 PGM 头读宽高（P5 二进制 / P2 文本均支持）。"""
    raw = MAP_PGM.read_bytes()
    idx, tokens = 0, []
    while len(tokens) < 4:
        while idx < len(raw) and raw[idx:idx + 1].isspace():
            idx += 1
        if raw[idx:idx + 1] == b"#":
            while idx < len(raw) and raw[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(raw) and not raw[idx:idx + 1].isspace():
            idx += 1
        tokens.append(raw[start:idx].decode("ascii"))
    return int(tokens[1]), int(tokens[2])


def zone_geometry():
    """返回 zone_a 底板尺寸与模型内 collision 数量。"""
    tree = ET.parse(str(WORLD))
    for model in tree.getroot().iter("model"):
        if model.get("name") != "zone_a":
            continue
        collisions = len(list(model.iter("collision")))
        for link in model.findall("link"):
            if link.get("name") != "base":
                continue
            for box in link.iter("box"):
                size = box.findtext("size")
                if size:
                    return [float(v) for v in size.split()], collisions
    return None, None


def main():
    failures = []

    def check(cond, msg):
        print(("  PASS  " if cond else "  FAIL  ") + msg)
        if not cond:
            failures.append(msg)

    print(f"数据路径: {POSITIONS}")
    cubes, zones = load_positions()
    print(f"== 数据规模 ==\n  方块 {len(cubes)} 个，放置区 {len(zones)} 个\n")

    print("== 1. 方块两两间距（要求 >= %.1f m） ==" % MIN_SPACING)
    pairs = sorted((math.dist(cubes[a][:2], cubes[b][:2]), a, b)
                   for a, b in itertools.combinations(sorted(cubes), 2))
    print(f"  最小 {pairs[0][0]:.3f} m ({pairs[0][1]} <-> {pairs[0][2]})")
    print(f"  最大 {pairs[-1][0]:.3f} m")
    print("  最近 5 对：")
    for dist, a, b in pairs[:5]:
        print(f"    {dist:6.3f} m  {a:14s} <-> {b}")
    too_close = [p for p in pairs if p[0] < MIN_SPACING]
    print(f"  低于阈值的配对: {len(too_close)} / {len(pairs)}")
    check(not too_close, f"全部 {len(pairs)} 对间距 >= {MIN_SPACING} m")

    print("\n== 2. 坐标是否在地图范围内 ==")
    if not (MAP_YAML.is_file() and MAP_PGM.is_file()):
        print("  SKIP  未找到 mapn3.yaml/pgm（仅上传源码时属正常）")
    else:
        meta = read_map_meta()
        width, height = read_pgm_size()
        res = meta["resolution"]
        ox, oy = meta["origin"]
        xmin, xmax = ox, ox + width * res
        ymin, ymax = oy, oy + height * res
        print(f"  地图范围 x[{xmin:.2f}, {xmax:.2f}] y[{ymin:.2f}, {ymax:.2f}]")
        outside = []
        for name, pose in list(cubes.items()) + list(zones.items()):
            x, y = pose[0], pose[1]
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                outside.append(name)
        check(not outside, f"全部坐标在地图内（越界: {outside or '无'}）")

    print("\n== 3. 放置区底板几何 ==")
    if not WORLD.is_file():
        print("  SKIP  未找到 gazebo_world3.new")
    else:
        size, collisions = zone_geometry()
        if size is None:
            print("  SKIP  未解析到 zone_a 模型")
        else:
            print(f"  zone_a 底板尺寸: {size[0]:g} x {size[1]:g} x {size[2]:g} m")
            print(f"  zone_a 模型内 collision 数量: {collisions}")
            check(abs(size[0] - 1.0) < 1e-6 and abs(size[1] - 0.5) < 1e-6,
                  "底板为 1.0 x 0.5 m（投放点落在中心即在板内）")
            check(collisions == 0,
                  "底板与字母均为纯视觉体（无碰撞 → 方块会陷进 2cm，符合已记录现象）")

    print("\n" + "=" * 52)
    if failures:
        print(f"结果: {len(failures)} 项失败")
        return 1
    print("结果: 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
