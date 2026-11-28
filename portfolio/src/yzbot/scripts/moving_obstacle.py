#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让 Gazebo 里的方块沿指定轴来回移动（正弦往复），做动态障碍物。

原理：world 里的 gazebo_ros_state 插件提供两个服务
    /gazebo/get_entity_state   查询位姿
    /gazebo/set_entity_state   设置位姿
本节点按固定频率把目标位姿设为
    x(t) = center + half * sin(2π t / period + phase)
于是方块在固定区间 [xmin, xmax] 内左右往复（默认 x ∈ [-2, 3]）。
y、z、朝向保持方块初始值不变。

默认目标是你新加的两个 0.5m 方块（unit_box 模型的两个 link）：
    unit_box::link
    unit_box::link_clone
两者默认相差 180° 相位（一左一右交错移动）；加 --phase 0 让它们同步。

用法：
    # 默认：两个方块在 x ∈ [-2, 3] 之间往复，周期 20s
    python3 moving_obstacle.py

    # 先自检：查服务是否在、打印初始位姿（不做任何移动）
    python3 moving_obstacle.py --check

    # 改区间 / 改速度
    python3 moving_obstacle.py --range -3 4 --period 26
    python3 moving_obstacle.py --range -2 3 --period 14   # 更快

    # 相对模式：从原位沿 +x 走 1m 再回来（覆盖 --range）
    python3 moving_obstacle.py --span 1.0 --period 6

    # 只移动整个模型（两个方块当成一个整体一起动）
    python3 moving_obstacle.py --targets unit_box --range -2 3

注意：
* 峰值速度 = π · 行程 / 周期。默认 5m / 20s ≈ 0.79 m/s，脚本会打印实际值。
* 方块是动态物体，被机器人顶到也没关系——每个周期都会按轨迹复位。
* 导航侧靠激光实时识别（局部代价地图 15Hz），全局代价地图 2Hz 会略有滞后。
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

DEFAULT_TARGETS = "unit_box::link,unit_box::link_clone"
DEFAULT_RANGE = (-2.0, 3.0)


class MovingObstacle(Node):
    def __init__(self, targets, span, period, rate, phase_deg, axis,
                 service_prefix="/gazebo", x_range=None):
        super().__init__("moving_obstacle")
        self.targets = targets
        self.span = span
        self.period = period
        self.rate = rate
        self.phase_deg = phase_deg
        self.axis = axis
        self.x_range = x_range

        prefix = service_prefix.rstrip("/")
        self.service_set = f"{prefix}/set_entity_state"
        self.service_get = f"{prefix}/get_entity_state"
        self.set_cli = self.create_client(SetEntityState, self.service_set)
        self.get_cli = self.create_client(GetEntityState, self.service_get)

    # ---------------- 基础调用 ----------------
    def wait_services(self, timeout=10.0):
        for cli, name in ((self.get_cli, self.service_get),
                          (self.set_cli, self.service_set)):
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    f"{name} 不可用。确认 world 里加载了 libgazebo_ros_state.so，"
                    "并用 --service-prefix 指定正确的命名空间（默认 /gazebo）")
            self.get_logger().info(f"服务就绪: {name}")

    def query_pose(self, name):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = "world"
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            raise RuntimeError(f"查询 {name} 超时")
        if not result.success:
            raise RuntimeError(f"查询 {name} 失败: {result.status_message}")
        return result.state.pose

    def set_pose(self, name, pose):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose = pose
        req.state.twist = Twist()          # 清零，避免残留速度让它漂移
        req.state.reference_frame = "world"
        return self.set_cli.call_async(req)

    # ---------------- 轨迹 ----------------
    def describe_motion(self):
        if self.x_range is not None:
            xmin, xmax = self.x_range
            travel = abs(xmax - xmin)
            center = 0.5 * (xmin + xmax)
            text = f"固定区间 x ∈ [{xmin:g}, {xmax:g}]（中心 {center:g}）"
        else:
            travel = self.span
            text = f"相对初始位置沿 {self.axis} 走 {travel:g}m 再返回"
        peak = math.pi * travel / self.period if self.period > 0 else 0.0
        self.get_logger().info(
            f"开始往复：{text}，行程 {travel:.2f}m，周期 {self.period:.1f}s，"
            f"频率 {self.rate:.0f}Hz，相位差 {self.phase_deg:.0f}°，"
            f"峰值速度约 {peak:.2f} m/s")
        if peak > 1.2:
            self.get_logger().warn(
                f"峰值速度 {peak:.2f} m/s 偏快，方块会明显'扫'过来；"
                f"建议把周期加大到 {math.pi * travel / 0.8:.0f}s 以上")

    def target_pose(self, name, base, t):
        omega = 2.0 * math.pi / self.period
        index = self.targets.index(name)
        phase = math.radians(self.phase_deg) * index
        pose = Pose()
        pose.position = Point(x=base.position.x, y=base.position.y,
                              z=base.position.z)
        pose.orientation = Quaternion(
            x=base.orientation.x, y=base.orientation.y,
            z=base.orientation.z, w=base.orientation.w)

        if self.x_range is not None:
            xmin, xmax = self.x_range
            center = 0.5 * (xmin + xmax)
            half = 0.5 * abs(xmax - xmin)
            value = center + half * math.sin(omega * t + phase)
            if self.axis == "x":
                pose.position.x = value
            else:
                # y 轴模式：区间仍作用于 y，x 保持初始
                pose.position.y = center + half * math.sin(omega * t + phase)
            return pose

        # 相对模式：从原位静止出发，沿 +axis 走 span 再回来
        s = 0.5 * (1.0 - math.cos(omega * t + phase))
        if self.axis == "x":
            pose.position.x = base.position.x + self.span * s
        else:
            pose.position.y = base.position.y + self.span * s
        return pose

    # ---------------- 主循环 ----------------
    def run(self):
        self.wait_services()
        bases = {}
        for name in self.targets:
            bases[name] = self.query_pose(name)
            p = bases[name].position
            self.get_logger().info(
                f"{name}: 初始位姿 ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

        self.describe_motion()

        started = time.monotonic()
        dt = 1.0 / max(1.0, self.rate)
        pending = {name: None for name in self.targets}
        last_report = started

        while rclpy.ok():
            now = time.monotonic()
            t = now - started

            for name in self.targets:
                # 上一个请求还没回来就跳过这一拍，避免请求堆积
                if pending[name] is not None and not pending[name].done():
                    continue
                pose = self.target_pose(name, bases[name], t)
                pending[name] = self.set_pose(name, pose)

            rclpy.spin_once(self, timeout_sec=0.0)

            if now - last_report >= 2.0:
                last_report = now
                for name in self.targets:
                    p = self.query_pose(name).position
                    self.get_logger().info(
                        f"{name}: ({p.x:+.2f}, {p.y:+.2f})")

            time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(
        description="让 Gazebo 方块沿指定轴正弦往复移动（动态障碍物）")
    parser.add_argument("--targets", default=DEFAULT_TARGETS,
                        help=f"逗号分隔的模型/link 名，默认 {DEFAULT_TARGETS}")
    parser.add_argument("--range", nargs=2, type=float,
                        default=list(DEFAULT_RANGE), metavar=("MIN", "MAX"),
                        help=f"固定移动区间，默认 {DEFAULT_RANGE[0]:g} {DEFAULT_RANGE[1]:g}")
    parser.add_argument("--span", type=float, default=None,
                        help="相对模式：从原位沿 +轴 走这么多米（会覆盖 --range）")
    parser.add_argument("--period", type=float, default=20.0,
                        help="往复周期（秒），默认 20")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="设置位姿的频率（Hz），默认 20")
    parser.add_argument("--phase", type=float, default=180.0,
                        help="相邻目标之间的相位差（度），默认 180（交错移动）")
    parser.add_argument("--axis", choices=("x", "y"), default="x",
                        help="移动轴，默认 x")
    parser.add_argument("--service-prefix", default="/gazebo",
                        help="gazebo_ros_state 服务的命名空间，默认 /gazebo；"
                             "若服务在根下可传空串")
    parser.add_argument("--check", action="store_true",
                        help="只检查服务和初始位姿，不移动")
    args = parser.parse_args()

    targets = [t.strip() for t in args.targets.split(",") if t.strip()]
    if not targets:
        raise SystemExit("--targets 不能为空")
    if args.period <= 0:
        raise SystemExit("--period 必须大于 0")

    x_range = None
    if args.span is not None:
        if args.span <= 0:
            raise SystemExit("--span 必须大于 0")
    else:
        low, high = args.range
        if abs(high - low) < 1e-6:
            raise SystemExit("--range 的两个值不能相同")
        x_range = (min(low, high), max(low, high))
        args.span = abs(high - low)

    rclpy.init()
    node = MovingObstacle(targets, args.span, args.period, args.rate,
                          args.phase, args.axis, args.service_prefix, x_range)
    try:
        if args.check:
            node.wait_services()
            for name in targets:
                p = node.query_pose(name).position
                print(f"{name}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
        else:
            node.run()
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，停止移动")
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
