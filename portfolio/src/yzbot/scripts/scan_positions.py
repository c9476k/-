#!/usr/bin/env python3
"""从 /gazebo/model_states 一次性读取所有模型位姿，生成 positions.py。

依赖:
    1. 经典 Gazebo 已启动，且 launch 里加载了 libgazebo_ros_state.so
       (gazebo_world2.launch.py 已加入该插件，发布会发布 /gazebo/model_states)
    2. 运行: python3 scan_positions.py
    3. 自动生成 positions.py，robot_mission_controller.py 导入即可使用
"""
import re
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

# 坐标固定输出到 scripts 目录下（绝对路径，避免运行目录不同导致写错位置）
OUTPUT_PATH = Path("/home/ros/demo1_ws/src/yzbot/scripts/positions.py")
MODEL_STATES_TOPIC = "/gazebo/model_states"
TIMEOUT = 10.0   # 最多等 10 秒


class StatesListener(Node):
    def __init__(self):
        super().__init__("scan_positions")
        self.data = None
        self.create_subscription(ModelStates, MODEL_STATES_TOPIC, self.cb, 10)

    def cb(self, msg):
        self.data = msg


def get_models() -> dict:
    """订阅一次 /gazebo/model_states，返回 {模型名: (x, y, z)}。"""
    rclpy.init()
    node = StatesListener()

    print(f"正在从 {MODEL_STATES_TOPIC} 读取模型位姿...")
    iters = 0
    max_iters = int(TIMEOUT / 0.1)
    while node.data is None and iters < max_iters:
        rclpy.spin_once(node, timeout_sec=0.1)
        iters += 1

    if node.data is None:
        node.get_logger().error(
            f"{TIMEOUT} 秒内未收到 {MODEL_STATES_TOPIC}，"
            "请确认 Gazebo 已启动，且 launch 里加载了 libgazebo_ros_state.so")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    models = {}
    for name, pose in zip(node.data.name, node.data.pose):
        models[name] = (pose.position.x, pose.position.y, pose.position.z)

    node.destroy_node()
    rclpy.shutdown()
    return models


def generate_positions(models: dict) -> tuple[dict, dict]:
    """筛选出方块和放置区坐标。"""
    block_positions = {}
    zone_positions = {}
    for name, pos in sorted(models.items()):
        if re.match(r"(red|blue)_cube_\d+", name):
            block_positions[name] = pos
        elif re.match(r"zone_[a-c]", name):
            zone_positions[name] = pos
    return block_positions, zone_positions


def write_positions_file(block_positions: dict, zone_positions: dict):
    """生成 positions.py 文件。"""
    lines = [
        '"""自动生成的方块和放置区坐标 (由 scan_positions.py 扫描 Gazebo 生成)。',
        "",
        "请勿手动编辑 — 运行 `python3 scan_positions.py` 重新扫描生成。",
        '"""',
    ]
    lines.append("block_positions = {")
    for name in sorted(block_positions):
        x, y, z = block_positions[name]
        lines.append(f'    "{name}": ({x}, {y}, {z}),')
    lines.append("}")
    lines.append("zone_positions = {")
    for name in sorted(zone_positions):
        x, y, z = zone_positions[name]
        lines.append(f'    "{name}": ({x}, {y}, {z}),')
    lines.append("}")
    OUTPUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"已生成: {OUTPUT_PATH}")
    print(f"  方块: {len(block_positions)} 个")
    print(f"  放置区: {len(zone_positions)} 个")


def main():
    models = get_models()
    if not models:
        print("未扫描到任何模型, 请确认 Gazebo 已启动并加载了场景")
        sys.exit(1)
    print(f"共扫描到 {len(models)} 个模型")
    block_positions, zone_positions = generate_positions(models)
    write_positions_file(block_positions, zone_positions)


if __name__ == "__main__":
    main()