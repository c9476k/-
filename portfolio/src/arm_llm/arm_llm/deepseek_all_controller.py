#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""大模型驱动的「导航 + 六轴机械臂」全流程搬运控制器。

一句话指令（例如"将红方块1送到A区域"）即可完成：
    收臂 → 导航到方块正前方 → 精停靠 → 抬臂对准 → 下放到抓取点
    → 夹爪闭合 + 吸附 → 抬臂 → 导航到放置区 → 下放 → 解除吸附
    → 抬臂 → 验证

与旧版的区别（重要）：
1. 停靠点计算修正：旧版固定沿 -x 退 0.5m，正好退进方块列里，而且 0.5m 已经
   超过贴地抓取的臂展；现改为按接近方向停在目标正前方 NAV_STANDOFF 米、
   车头朝向目标（红方块从南侧、蓝方块从北侧、放置区从西侧接近）。
2. 机械臂姿态用逆运动学按真实连杆长度现算，不再猜关节角；导航停歪时
   还能用 joint1 补偿横向误差。
3. 抓取前检查是否在臂展内，吸附后检查方块是否真的跟着抬起，避免
   LinkAttacher 把远处的方块"吸"过来（它本身不做距离判断）。
4. 导航会检查 action 结果状态，失败/超时会重试或安全退出，不再假报"完成"。
5. 大模型不可用（没网、没有 API Key）时自动降级为本地中文指令解析。
6. 支持出题程序：输入"题目"后粘贴题目+映射规则，大模型解出 x/y 并按
   [red,4,A;blue,1,C] 格式输出，代码用正则解析后自动搬运；执行时固定
   "先蓝后红、同色按编号 1~5" 的顺序。解析结果同时发到 /mission/task
   和 /mission/count|zone 等看板话题。

   解题只走大模型 API，分两阶段：
     阶段一  让模型列算式解出 x、y（不满足 x+y=5 会带约束追问一次）
     阶段二  把已确定的 x、y 连同本次映射规则交给模型，翻译成 [color,count,zone]
   API 不可用（未装 requests / 未设 Key / 网络不通 / 超时）时直接报"无法连接"
   并放弃执行，不再用固定题目模板兜底。规则解析支持编号/引号/无前缀等写法。

运行（先按你的流程启动 Gazebo、Nav2、Foxglove）：
    export DEEPSEEK_API_KEY=sk-xxxx          # 必需；未设置直接报"无法连接"退出
    python3 deepseek_all_controller.py
    # 或： ros2 run arm_llm deepseek_all_controller
常用参数：
    --no-dock        关闭导航后的厘米级精停靠（只用 Nav2 的结果）
    --init-pose      用 Gazebo 真值自动发布 /initialpose（省去 RViz 手动 2D Pose Estimate）

响应速度相关（环境变量，可按需覆盖）：
    LLM_TIMEOUT=10         简单指令请求大模型的超时（秒）
    LLM_TIMEOUT_TASK=60    解题请求超时
    LLM_MAX_TOKENS=256     简单指令的输出上限
    LLM_MAX_TOKENS_TASK=128 阶段二输出上限
    LLM_MAX_TOKENS_XY=1500 阶段一输出上限
    DEEPSEEK_REASON_MODEL=deepseek-reasoner  阶段一用推理模型深度思考
    DEEPSEEK_MODEL=deepseek-chat   换模型；deepseek-reasoner 会明显更慢
    导航过程中每 2 秒刷新一次 /mission/progress（显示剩余米数），
    状态/当前任务在每个动作开始时更新。

最常需要微调的参数（都在文件开头"配置区"）：
    NAV_STANDOFF / DOCK_RANGE   车停在目标前方多远（默认 0.34m，等于臂展舒适区）
    APPROACH_DIRS               从目标的哪一侧接近（红方块南侧、蓝方块北侧、放置区西侧）
    DROP_OFFSET                 放置点相对区域中心的偏移
    PICK_GRASP_Z / ABOVE_GRASP_Z 抓取高度 / 上方过渡高度
    MIN_REACH / MAX_REACH       允许 IK 求解的臂展范围
    GRIPPER_CLOSE               夹爪闭合量（0.02：留 7.5mm 安全间隙，避免和方块碰撞）
注意：抓取/放置的六个关节角不再写死，而是用真值位姿 + 逆运动学生成，
      POSE_DICT 里的 pick/place 只在拿不到 Gazebo 真值时兜底。

依赖话题/服务（你的启动流程里都已经具备）：
    /navigate_to_pose (nav2_msgs/NavigateToPose)
    /arm_controller/follow_joint_trajectory
    /gripper_controller/follow_joint_trajectory
    /ATTACHLINK、/DETACHLINK (linkattacher_msgs)
    /gazebo/model_states（世界文件里已加载 libgazebo_ros_state.so）
    /joint_states
给 Foxglove 看板的话题（std_msgs/String，字段都是 data）：
    /mission/task        种子 + 题目 + 映射规则 + 方案
    /mission/state       状态：待命 / 解析任务 / 导航中 / 机械臂移动 / 抓取中 / 放置中 / 已送达 / 任务完成
    /mission/current     当前任务：红方块1 → A区
    /mission/progress    任务进度：第 2/5 个
    /mission/count/red   红色数量（std_msgs/Int32）
    /mission/count/blue  蓝色数量（std_msgs/Int32）
    /mission/zone/red    红色到A区
    /mission/zone/blue   蓝色到C区
    /mission/status      以上信息的 JSON 合集（备用）
"""

import argparse
import importlib.util
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from linkattacher_msgs.srv import AttachLink, DetachLink
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    import requests
    HAVE_REQUESTS = True
except Exception:                       # 没装 requests 也能离线跑
    HAVE_REQUESTS = False

try:
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    HAVE_TF2 = True
except Exception:
    HAVE_TF2 = False


# ============================================================================
# 1. 大模型配置
# ============================================================================
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")
DEEPSEEK_URL = os.environ.get("DEEPSEEK_URL", "https://api.deepseek.com/v1/chat/completions")
MODEL_NAME = os.environ.get("DEEPSEEK_MODEL", "deepseek-chat")
# 阶段一解应用题想要更强推理时指定，例如 deepseek-reasoner（更慢但更会算）
REASON_MODEL_NAME = os.environ.get("DEEPSEEK_REASON_MODEL", "")
# 限时/限长是提速的关键：方案只有一行，不需要让模型长篇输出
LLM_TIMEOUT_COMMAND = float(os.environ.get("LLM_TIMEOUT", "10"))
LLM_TIMEOUT_TASK = float(os.environ.get("LLM_TIMEOUT_TASK", "30"))
# 输出上限按"够用就好"设，模型少写就少花钱；实际计费按真实输出，不会浪费
LLM_MAX_TOKENS = int(os.environ.get("LLM_MAX_TOKENS", "96"))
LLM_MAX_TOKENS_TASK = int(os.environ.get("LLM_MAX_TOKENS_TASK", "48"))
# 阶段一只要求输出一行 x=..,y=..，不需要长篇演算
LLM_MAX_TOKENS_XY = int(os.environ.get("LLM_MAX_TOKENS_XY", "160"))
# 所有自然语言（简单指令 + 出题题目）都只走大模型 API，本地不做兜底解析


# ============================================================================
# 2. 机械臂几何参数（严格按 mybot_description/urdf/arm.xacro 量取）
#    改动 URDF 后需要同步这里，抓取点才会准
# ============================================================================
ARM_BASE_Z = 0.16        # arm_base_link 离地高度 = base_link 0.10 + 安装 0.06
JOINT2_Z = 0.12          # joint2 相对 arm_base_link 的高度
LINK_UPPER = 0.14        # link2：joint2 -> joint3
LINK_FOREARM = 0.28      # joint3 -> joint5（link3 0.22 + link4 0.06，joint4 是 roll 不改变方向）
LINK_WRIST = 0.102       # joint5 -> 夹爪中心（link5 0.06 + 指尖偏置 0.042）

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_LIMITS = [
    (-3.14, 3.14),       # joint1 绕 z 回转
    (-2.35, 2.35),       # joint2 大臂俯仰
    (-2.62, 2.62),       # joint3 小臂俯仰
    (-3.14, 3.14),       # joint4 腕部 roll
    (-2.04, 2.04),       # joint5 腕部俯仰（轴为 -y，方向与 joint2 相反）
    (-3.14, 3.14),       # joint6 末端 roll
]
GRIPPER_JOINT = "finger_joint1"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.020    # 只做"夹"的动作：3cm 方块两侧各留 7.5mm 安全间隙，绝不接触

# 固定姿态（安全/调试用）。真正的抓取姿态由 IK 计算，这里只做兜底
POSE_CARRY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]        # 竖直收臂，行驶姿态

# 抓取高度：夹爪中心离地高度。方块中心 0.015，手指长 0.08，取 0.05 时
# 指尖约在地面上方 1cm，方块的上一半正好在两根手指之间
PICK_GRASP_Z = 0.050
ABOVE_GRASP_Z = 0.200     # 抓取/放置前的"上方"过渡点
PLACE_GRASP_Z = 0.050

MIN_REACH = 0.270         # 小于这个距离：车头/手指会碰到方块或底盘
MAX_REACH = 0.450         # 大于这个距离：腕部要倾斜太多，再远就抓不到
WRIST_TILTS = (0.0, 10.0, 20.0, 30.0, 40.0)   # 够不到时依次把腕部往前倾

ARM_MOVE_MAX_SPEED = 0.9   # 规划轨迹时假设的关节角速度 (rad/s)，用于估算运动时间
GRIPPER_MOVE_SECONDS = 0.7


# ============================================================================
# 3. 导航 / 停靠参数
# ============================================================================
NAV_STANDOFF = 0.34       # 导航目标点离目标物体的距离（米，从车体中心算）
DOCK_RANGE = 0.34         # 精停靠想达到的距离
DOCK_TOL = 0.02           # 精停靠距离容差
DOCK_YAW_TOL = 0.08       # 精停靠朝向容差（弧度）
NAV_TIMEOUT = 90.0        # 单次导航超时（秒）
# Nav2 单次失败（如 MPPI "Optimizer fail to compute path"）后清理代价地图重试的次数
NAV_RETRIES = int(os.environ.get("NAV_RETRIES", "2"))
# 只要进入这个半径并保持 NAV_ARRIVE_HOLD 秒，就认为"够近"，立刻交给
# dock_to_object 做厘米级精停靠——不必等 Nav2 的 xy_goal_tolerance 严格判定。
# 实测：车停在离目标 0.19m 处（容差 0.15m），Nav2 会一直不判到达直到 90s 超时。
NAV_ARRIVE_EPS = float(os.environ.get("NAV_ARRIVE_EPS", "0.30"))
NAV_ARRIVE_HOLD = float(os.environ.get("NAV_ARRIVE_HOLD", "1.0"))
NAV_ACCEPT_TIMEOUT = 10.0
SETTLE_TIME = 0.8         # 导航结束后的稳定时间（等速度平滑器/底盘完全停下）
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
# 精停靠同时发到 Nav2 速度链路的入口，兼容 collision_monitor / velocity_smoother 接管的情况
CMD_VEL_SMOOTHED_TOPIC = os.environ.get("CMD_VEL_SMOOTHED_TOPIC", "/cmd_vel_smoothed")

# 接近方向：从物体指向"车应该停在哪一侧"的单位向量。
# red 方块在 y=+1（过道在南侧），blue 在 y=-1（过道在北侧），三个放置区从西侧接近。
APPROACH_DIRS = {
    "red_cube": (0.0, -1.0),
    "blue_cube": (0.0, 1.0),
    "zone": (-1.0, 0.0),
}
# 放置点相对放置区中心的偏移。底板是 1.0(x) × 0.5(y) × 0.02m 的纯视觉体，
# 字母牌立在正中（两腿在 x=±0.1，中间有 16cm 空隙，放得下 3cm 方块）。
# 默认放到区域正中 (0,0)：方块落在彩色底板范围内，不再偏到板外。
# 需要微调时用环境变量 DROP_OFFSET_X / DROP_OFFSET_Y（米）。
DROP_OFFSET = (float(os.environ.get("DROP_OFFSET_X", "0.0")),
               float(os.environ.get("DROP_OFFSET_Y", "0.0")))

ROBOT_MODEL = "six_arm"
ARM_MODEL = "six_arm"
ARM_LINK = "gripper_finger_link1"   # 你的实测值；若改用 link6 也支持
CUBE_LINK = "link"

MODEL_STATES_TOPIC = "/gazebo/model_states"
STATUS_TOPIC = "/mission/status"
TASK_TOPIC = "/mission/task"          # 种子/题目/映射规则/方案，给 Foxglove Raw Message 面板
JOINT_STATE_TOPIC = "/joint_states"

# ---- 给 Foxglove 看板用的简单话题（面板直接绑 data 字段）----
STATE_TOPIC = "/mission/state"                 # 状态：待命 / 导航中 / 机械臂移动 ...
CURRENT_TOPIC = "/mission/current"             # 当前任务：红方块1 → A区
PROGRESS_TOPIC = "/mission/progress"           # 任务进度：第 2/5 个
COUNT_TOPIC_FMT = "/mission/count/{color}"     # 红色/蓝色数量：Int32
ZONE_TOPIC_FMT = "/mission/zone/{color}"       # 红色/蓝色去向：String
STATE_TEXT = {
    "idle": "待命",
    "task": "解析任务",
    "deliver": "执行任务",
    "navigate": "导航中",
    "dock": "精停靠",
    "arm": "机械臂移动",
    "pick": "抓取中",
    "grasp": "抓取中",
    "place": "放置中",
    "delivered": "已送达",
    "done": "任务完成",
    "error": "出错",
}

# 出题解析完成后是否自动开始搬运；设为 0 则只更新看板，等待逐条指令
TASK_AUTO_RUN = os.environ.get("TASK_AUTO_RUN", "1") != "0"

# 解题只走大模型 API（两阶段：先算 x/y，再套映射规则）。
# API 不可用时直接报"无法连接"，不再用本地模板兜底，避免掩盖接口故障。
CONN_ERROR = "无法连接"
# 出题程序保证 x>=1、y>=1、x+y=5，用于校验大模型有没有算错
EXPECTED_TOTAL = 5


# ============================================================================
# 4. 运动学：正解用于自检，逆解用于生成抓取姿态
# ============================================================================
def arm_fk(q: Sequence[float]) -> Tuple[float, float, float]:
    """正运动学。返回 (r, z_rel, 腕部俯仰角/度)，r/z_rel 相对 arm_base_link。"""
    a1 = q[1]
    a2 = q[1] + q[2]
    a5 = a2 - q[4]
    p3 = (LINK_UPPER * math.sin(a1), JOINT2_Z + LINK_UPPER * math.cos(a1))
    p5 = (p3[0] + LINK_FOREARM * math.sin(a2), p3[1] + LINK_FOREARM * math.cos(a2))
    grasp_r = p5[0] + LINK_WRIST * math.sin(a5)
    grasp_z = p5[1] + LINK_WRIST * math.cos(a5)
    return grasp_r, grasp_z, math.degrees(a5)


def solve_arm_ik(radius: float, z_rel: float, tilt_deg: float = 0.0) -> Optional[List[float]]:
    """平面三连杆逆解（joint2 / joint3 / joint5），其余关节给 0。

    radius : 目标点相对 arm_base_link 轴线的水平距离
    z_rel  : 目标点相对 arm_base_link 的高度
    tilt_deg: 腕部相对竖直向前的倾角，0 表示指尖垂直向下夹取
    返回 [q1..q6]；不可达或超关节限位返回 None。
    """
    # 目标点是夹爪中心；腕部倾角 tilt 时，先反推 joint5 的位置
    gamma = math.pi - math.radians(tilt_deg)
    p5_r = radius - LINK_WRIST * math.sin(gamma)
    p5_z = z_rel - LINK_WRIST * math.cos(gamma)

    dr = p5_r
    dz = p5_z - JOINT2_Z
    dist = math.hypot(dr, dz)
    if dist > LINK_UPPER + LINK_FOREARM - 0.002:
        return None
    if dist < abs(LINK_FOREARM - LINK_UPPER) + 0.005:
        return None

    psi = math.atan2(dr, dz)                # 从 +z 转向 +x 的角度
    cos_a = (LINK_UPPER ** 2 + dist ** 2 - LINK_FOREARM ** 2) / (2 * LINK_UPPER * dist)
    cos_b = (LINK_UPPER ** 2 + LINK_FOREARM ** 2 - dist ** 2) / (2 * LINK_UPPER * LINK_FOREARM)
    cos_a = max(-1.0, min(1.0, cos_a))
    cos_b = max(-1.0, min(1.0, cos_b))

    angle_a = math.acos(cos_a)              # 大臂与"指向目标连线"的夹角
    beta = math.pi - math.acos(cos_b)       # 肘部弯曲量
    a1 = psi - angle_a                      # 取"肘在前上方"的解
    a2 = a1 + beta

    q = [0.0, a1, beta, 0.0, a2 - gamma, 0.0]
    for value, (low, high) in zip(q, JOINT_LIMITS):
        if not (low - 1e-6 <= value <= high + 1e-6):
            return None
    return q


# 兜底姿态：假设车停在 0.34m 处（没法拿到真值时才用）
POSE_PICK_DEFAULT = solve_arm_ik(0.34, PICK_GRASP_Z - ARM_BASE_Z, 0.0) or [0.0, 1.19, 1.10, 0.0, -0.85, 0.0]
POSE_ABOVE_DEFAULT = solve_arm_ik(0.34, ABOVE_GRASP_Z - ARM_BASE_Z, 0.0) or [0.0, 0.58, 1.34, 0.0, -1.22, 0.0]
POSE_PLACE_DEFAULT = solve_arm_ik(0.34, PLACE_GRASP_Z - ARM_BASE_Z, 0.0) or [0.0, 1.19, 1.10, 0.0, -0.85, 0.0]

POSE_DICT = {
    "home": POSE_CARRY,
    "lift": POSE_CARRY,
    "carry": POSE_CARRY,
    "pick": POSE_PICK_DEFAULT,
    "place": POSE_PLACE_DEFAULT,
}


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class ObjectPose:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass
class BasePose:
    x: float
    y: float
    yaw: float


# ============================================================================
# 5. 坐标库：优先读 yzbot/scripts/positions.py（scan_positions.py 生成）
# ============================================================================
FALLBACK_BLOCKS = {
    "blue_cube_1": (-1.5, -1.0, 0.015), "blue_cube_2": (-1.6, -1.0, 0.015),
    "blue_cube_3": (-1.7, -1.0, 0.015), "blue_cube_4": (-1.8, -1.0, 0.015),
    "blue_cube_5": (-1.9, -1.0, 0.015),
    "red_cube_1": (-1.5, 1.0, 0.015), "red_cube_2": (-1.6, 1.0, 0.015),
    "red_cube_3": (-1.7, 1.0, 0.015), "red_cube_4": (-1.8, 1.0, 0.015),
    "red_cube_5": (-1.9, 1.0, 0.015),
}
FALLBACK_ZONES = {
    "zone_a": (4.18567, -1.9437, 0.01),
    "zone_b": (5.0, -12.0, 0.01),
    "zone_c": (-4.61913, 2.34331, 0.01),
}


def load_positions() -> Tuple[Dict[str, tuple], Dict[str, tuple], Optional[str]]:
    """在几个常见位置找 positions.py；找不到就用内置值并告警。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.environ.get("POSITIONS_PY", ""),
        os.path.join(here, "..", "..", "yzbot", "scripts", "positions.py"),
        os.path.expanduser("~/demo1_ws/src/yzbot/scripts/positions.py"),
        os.path.expanduser("~/dev_ws/src/yzbot/scripts/positions.py"),
        os.path.expanduser("~/yzbot/scripts/positions.py"),
        "/home/ros/demo1_ws/src/yzbot/scripts/positions.py",
        os.path.join(os.getcwd(), "positions.py"),
    ]
    for path in candidates:
        if not path:
            continue
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            continue
        try:
            spec = importlib.util.spec_from_file_location("positions_auto", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            blocks = dict(getattr(module, "block_positions", {}) or {})
            zones = dict(getattr(module, "zone_positions", {}) or {})
            if blocks or zones:
                return blocks, zones, path
        except Exception as exc:
            print(f"[坐标] 读取 {path} 失败: {exc}")
    return dict(FALLBACK_BLOCKS), dict(FALLBACK_ZONES), None


BLOCK_POSITIONS, ZONE_POSITIONS, POSITIONS_SOURCE = load_positions()


# ============================================================================
# 6. 大模型 Function Calling 工具定义
# ============================================================================
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "deliver_cube",
            "description": "搬运指定编号的方块到放置区（含抓取与放置）",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {"type": "string", "enum": ["red", "blue"]},
                    "id": {"type": "integer"},
                    "zone": {"type": "string", "enum": ["A", "B", "C"]},
                },
                "required": ["color", "id", "zone"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "deliver_cubes",
            "description": "搬运 N 个同色方块到放置区（用户没说编号时用）",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {"type": "string", "enum": ["red", "blue"]},
                    "count": {"type": "integer"},
                    "zone": {"type": "string", "enum": ["A", "B", "C"]},
                },
                "required": ["color", "count", "zone"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to",
            "description": "只移动底盘到目标附近，不抓取",
            "parameters": {
                "type": "object",
                "properties": {"target": {"type": "string"}},
                "required": ["target"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "move_to_pose",
            "description": "机械臂到预设姿态（调试）",
            "parameters": {
                "type": "object",
                "properties": {
                    "pose": {"type": "string", "enum": ["home", "lift", "carry", "pick", "place"]}
                },
                "required": ["pose"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "control_gripper",
            "description": "张开/闭合夹爪。放下或松开物块用 open",
            "parameters": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["close", "open"]}},
                "required": ["action"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "attach_cube",
            "description": "吸附/释放方块（调试）",
            "parameters": {
                "type": "object",
                "properties": {
                    "op": {"type": "string", "enum": ["attach", "detach"]},
                    "color": {"type": "string", "enum": ["red", "blue"]},
                    "id": {"type": "integer"},
                },
                "required": ["op", "color", "id"],
            },
        },
    },
]

SYSTEM_PROMPT = (
    "把中文指令翻成工具调用，只输出工具调用或一句追问。\n"
    "场景：红/蓝方块 red_cube_1~5 / blue_cube_1~5；放置区 A/B/C。\n"
    "指定编号→deliver_cube；只说数量→deliver_cubes；缺信息就追问，不要编造。\n"
    "只说「放下/松开/释放物块」→ control_gripper(open)；「抓取/夹紧」→ control_gripper(close)。\n"
    "只用 deliver_* 完成整段搬运，不要用 control_gripper/move_to_pose/attach_cube 拆分搬运流程。"
)

# 出题程序提示词（移植自 ai.py）：刻意不给"某规则→某答案"的示例，
# 否则模型会照抄示例而不真正计算（实测踩过这个坑）。
XY_SYSTEM_PROMPT = (
    "解应用题求 x、y，禁止猜测。\n"
    "1) 找准两种物品名与每容器容量 N；\n"
    "2) 逐句累加需求（『甲、乙都需要n个』要乘人数；『设…为x』『有A、B两种…』不计入）；\n"
    "3) x=ceil(totalX/N)，y=ceil(totalY/N)，且 x,y>=1。\n"
    f"不要过程，只输出一行：x=<整数>,y=<整数>（x+y应为{EXPECTED_TOTAL}）"
)

MAP_SYSTEM_PROMPT = (
    "按本次映射规则把已知 x、y 翻译成方案，不要重新解题、不要改数值。\n"
    "颜色可能按 x/y 指定、按数值阈值或两数大小决定；区域可能按颜色、x/y 或阈值决定。\n"
    "只输出一行：[color,count,zone;...]，color 取 red/blue，zone 取 A/B/C，count 即 x 或 y。\n"
    "规则缺颜色或区域时输出：[ERROR] 缺少映射"
)


# ============================================================================
# 7. 通用解析工具（供"大模型回复解析"和"执行链路"使用，不含本地自然语言解析）
# ============================================================================
CN_DIGITS = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def parse_int(value) -> Optional[int]:
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.isdigit():
        return int(text)
    try:
        number = float(text)
        if number.is_integer():
            return int(number)
    except ValueError:
        pass
    if text in CN_DIGITS:
        return CN_DIGITS[text]
    if len(text) == 2 and text[0] == "十" and text[1] in CN_DIGITS:
        return 10 + CN_DIGITS[text[1]]
    if len(text) == 2 and text[1] == "十" and text[0] in CN_DIGITS:
        return CN_DIGITS[text[0]] * 10
    return None


def normalize_color(value) -> Optional[str]:
    text = str(value).lower()
    if "红" in text or "red" in text:
        return "red"
    if "蓝" in text or "blue" in text:
        return "blue"
    return None


def normalize_zone(value) -> Optional[str]:
    text = str(value).strip().lower()
    if text in ("a", "b", "c"):          # 大模型按 enum 直接回 "A"/"B"/"C"
        return "zone_" + text
    match = re.search(r"zone[_ ]?([abc])|([abc])\s*区|(?:区域|区)\s*([abc])", text)
    if match:
        letter = match.group(1) or match.group(2) or match.group(3)
        return "zone_" + letter
    return None


# ============================================================================
# 7.1 出题程序：两阶段解题（先算 x/y，再套映射规则）
#     本节逻辑移植自 ai.py：只走大模型 API，不再使用固定题目模板兜底
# ============================================================================
PLAN_ITEM_RE = re.compile(
    r"(red|blue|红|蓝)\s*[,:;\s]+\s*(\d+)\s*[,:;\s]+\s*([abcABC])(?![a-zA-Z])",
    re.IGNORECASE)

# 从大模型回复里取 x=<数>,y=<数>（取最后一处，避免被中间演算干扰）
XY_ANSWER_RE = re.compile(
    r"x\s*[=＝:：]\s*(\d+)[^\d\n]{0,20}?y\s*[=＝:：]\s*(\d+)", re.IGNORECASE)


def is_task_problem(text: str) -> bool:
    """判断输入是出题程序的"题目+映射规则"，而不是"把红方块1送到A区"这种简单指令。

    不能用"有颜色词 + 有区域词"来判断——简单指令本身就同时含这两者。
    """
    t = text.strip()
    if not t:
        return False
    if "映射规则" in t:
        return True
    if SEED_RE.search(t):
        return True
    if re.match(r"^\s*(题目|题干|task|problem|出题)", t, re.IGNORECASE):
        return True
    if re.search(r"数量\s*为\s*[xy]", t, re.IGNORECASE):
        return True
    if "每" in t and re.search(r"计算\s*[xXyY]", t):
        return True
    if "两种" in t and "每个" in t:
        return True
    return False


# 出题输入常带"使用种子: 123456"和一行分隔线，解析前先剥掉：
# 否则种子和分隔线会被当成题目内容喂给大模型，也会混进 /mission/task。
SEED_RE = re.compile(r"使用种子\s*[:：]?\s*(\d+)")
SEPARATOR_RE = re.compile(r"^[\s\-—_=─·•*]+$")


def extract_seed(problem: str) -> str:
    """取出"使用种子: 997072"里的编号，没有则返回空串。"""
    match = SEED_RE.search(str(problem))
    return match.group(1) if match else ""


def strip_seed(problem: str) -> str:
    """去掉种子片段和分隔线（种子可能在行首，也可能与题目同一行）。"""
    text = SEED_RE.sub(" ", str(problem))
    lines = [ln for ln in text.splitlines() if not SEPARATOR_RE.match(ln)]
    return "\n".join(lines).strip()


def clean_problem_text(problem: str) -> str:
    """剥掉种子/分隔线，再去掉开头的"题目:"/"题干:"，避免看板出现两个前缀。"""
    text = strip_seed(problem)
    return re.sub(r"^\s*(题目|题干|题|task|problem|出题)\s*[:：]\s*", "",
                  text, flags=re.IGNORECASE).strip()


# ---- 从输入里分离映射规则：兼容编号/引号/换行/无"映射规则"前缀等写法 ----
LIST_PREFIX_RE = re.compile(r"^\s*(?:\d+\s*[\.、\)）]|[（(]\s*\d+\s*[)）]|[-*•])\s*")
QUOTE_CHARS = "\"'“”‘’「」『』《》 \t"
RULE_COLOR_RE = re.compile(r"(红色|蓝色|红|蓝|red|blue)", re.IGNORECASE)
RULE_ZONE_RE = re.compile(r"[abcABC]\s*区|zone[_ ]?[abc]", re.IGNORECASE)
RULE_MAP_RE = re.compile(r"代表|对应|表示|去|放到|送往|放在|送到|分配到|分到")


def _strip_decorations(text: str) -> str:
    """剥掉编号前缀和引号，可能嵌套（如 1. "..."）。"""
    cleaned = str(text).strip()
    for _ in range(3):
        stripped = LIST_PREFIX_RE.sub("", cleaned).strip(QUOTE_CHARS).strip()
        if stripped == cleaned:
            break
        cleaned = stripped
    return cleaned


def looks_like_rule(segment: str) -> bool:
    """片段是否像映射规则：必须同时提到颜色和（区域或映射动词）。"""
    if not RULE_COLOR_RE.search(segment):
        return False
    return bool(RULE_ZONE_RE.search(segment) or RULE_MAP_RE.search(segment))


def extract_mapping_rule(problem: str) -> Tuple[str, str]:
    """从输入中分离出映射规则，返回 (题干, 规则)。找不到规则时规则为空串。

    兼容：带"映射规则:"前缀 / 单独一行 / 编号列表(1. 2、) / 引号包裹 /
    紧跟题干句号后；全角半角标点均可。
    """
    text = clean_problem_text(problem)
    if not text:
        return "", ""

    match = re.search(r"映射规则\s*[:：]?\s*", text)
    if match:
        return text[:match.start()].strip(), _strip_decorations(text[match.end():])

    segments = [s for s in re.split(r"[\n\r]+", text) if s.strip()]
    if len(segments) == 1:
        segments = [s for s in re.split(r"(?<=。)", text) if s.strip()]

    rule_parts: List[str] = []
    body_parts: List[str] = []
    for segment in segments:
        cleaned = _strip_decorations(segment)
        if cleaned and looks_like_rule(cleaned):
            rule_parts.append(cleaned)
        else:
            body_parts.append(segment.strip())

    if rule_parts:
        return " ".join(body_parts).strip(), " ".join(rule_parts).strip()
    return text, ""


def split_task_problem(problem: str) -> Tuple[str, str]:
    """兼容旧名：拆成 (题干, 映射规则)。"""
    return extract_mapping_rule(problem)


def _entries_from_segment(segment: str) -> List[dict]:
    entries: List[dict] = []
    seen = set()
    for match in PLAN_ITEM_RE.finditer(segment):
        color = normalize_color(match.group(1))
        zone = normalize_zone(match.group(3))
        count = parse_int(match.group(2))
        if color is None or zone is None or count is None or color in seen:
            continue
        if not 1 <= count <= EXPECTED_TOTAL:
            continue
        seen.add(color)
        entries.append({"color": color, "count": count, "zone": zone})
    return entries


def order_entries(entries: Sequence[dict]) -> List[dict]:
    """统一输出顺序：红在前、蓝在后，便于和标准答案逐字对比。"""
    return sorted(entries, key=lambda e: 0 if str(e.get("color")) == "red" else 1)


def parse_plan_text(text: str) -> List[dict]:
    """从大模型回复里抠出 [red,4,A;blue,1,C]。

    模型带思考过程时推理里可能出现别的方括号，所以优先取"答案:"标记之后、
    或最后一个能解析出条目的 [...] 片段（最终答案通常在末尾）。
    """
    if not text:
        return []
    normalized = (text.replace("；", ";").replace("，", ",")
                      .replace("：", ":").replace("［", "[").replace("］", "]"))
    marked = re.split(r"(?:答案|最终结果|最终方案|结果)\s*[:：]", normalized)
    if len(marked) > 1:
        entries = _entries_from_segment(marked[-1])
        if entries:
            return order_entries(entries)
    for inner in reversed(re.findall(r"\[([^\[\]]*)\]", normalized)):
        entries = _entries_from_segment(inner)
        if entries:
            return order_entries(entries)
    return order_entries(_entries_from_segment(normalized))


def format_plan(entries: Sequence[dict]) -> str:
    """[{"color":"red","count":4,"zone":"zone_a"}, ...] -> "[red,4,A;blue,1,C]" """
    parts = []
    for entry in entries:
        zone = str(entry.get("zone") or "")
        letter = zone[-1].upper() if zone else "?"
        parts.append(f"{entry.get('color')},{int(entry.get('count', 0))},{letter}")
    return "[" + ";".join(parts) + "]"


def cube_display_name(cube: str) -> str:
    """red_cube_2 -> 红方块2，给看板显示用。"""
    match = re.match(r"(red|blue)_cube_(\d+)", cube or "")
    if not match:
        return str(cube)
    color = "红" if match.group(1) == "red" else "蓝"
    return f"{color}方块{match.group(2)}"


def zone_display_name(zone: str) -> str:
    """zone_a -> A区。"""
    text = str(zone or "")
    return f"{text[-1].upper()}区" if text else "-"


def format_execution_order(entries: Sequence[dict]) -> str:
    """按实际执行顺序（先蓝后红）生成 "蓝色×3到A区 → 红色×2到C区"。"""
    ordered = sorted(entries, key=lambda e: 0 if str(e.get("color")) == "blue" else 1)
    return " → ".join(
        f"{'蓝色' if e.get('color') == 'blue' else '红色'}×{e.get('count')}"
        f"到{zone_display_name(e.get('zone'))}"
        for e in ordered)


def validate_plan(entries: Sequence[dict], x: int, y: int) -> List[str]:
    """一致性校验（不重新解题）：方案里的数量应当正好是 x 和 y。"""
    warnings = []
    counts = sorted(int(e["count"]) for e in entries)
    if counts != sorted([x, y]):
        warnings.append(f"方案数量 {counts} 与解得的 x={x},y={y} 不一致，请人工核对")
    if len(entries) != 2:
        warnings.append(f"方案含 {len(entries)} 项，预期 2 项（红、蓝各一）")
    if len({e["zone"] for e in entries}) != len(entries):
        warnings.append("两种颜色被分到了同一个区域，请核对映射规则")
    return warnings


# ============================================================================
# 8. 主节点
# ============================================================================
class MissionNode(Node):
    def __init__(self, use_dock: bool = True):
        super().__init__("deepseek_mission_controller")
        self.use_dock = use_dock
        # 复用 HTTP 连接（keep-alive），比每次重新握手快 100~300ms。
        # 所有自然语言理解都依赖大模型；连接不可用时 _http 为 None，调用会报"无法连接"
        self._http = (requests.Session()
                      if (HAVE_REQUESTS and DEEPSEEK_API_KEY) else None)

        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       "/arm_controller/follow_joint_trajectory")
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           "/gripper_controller/follow_joint_trajectory")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")
        # MPPI 偶发 "Optimizer fail to compute path" 时，清代价地图后再试一次
        self.local_clear_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.global_clear_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.task_pub = self.create_publisher(String, TASK_TOPIC, 10)
        self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self.current_pub = self.create_publisher(String, CURRENT_TOPIC, 10)
        self.progress_pub = self.create_publisher(String, PROGRESS_TOPIC, 10)
        self.count_pubs = {
            color: self.create_publisher(Int32, COUNT_TOPIC_FMT.format(color=color), 10)
            for color in ("red", "blue")
        }
        self.zone_pubs = {
            color: self.create_publisher(String, ZONE_TOPIC_FMT.format(color=color), 10)
            for color in ("red", "blue")
        }
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cmd_pub2 = self.create_publisher(Twist, CMD_VEL_SMOOTHED_TOPIC, 10)
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.create_subscription(ModelStates, MODEL_STATES_TOPIC, self._on_model_states, 10)
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)

        self._objects: Dict[str, ObjectPose] = {}
        self._objects_wall = 0.0
        self._joint_positions: Dict[str, float] = {}

        self._tf_buffer = None
        if HAVE_TF2:
            try:
                self._tf_buffer = Buffer()
                TransformListener(self._tf_buffer, self, spin_thread=False)
            except Exception as exc:
                self.get_logger().warn(f"TF 初始化失败，只用 Gazebo 真值: {exc}")
                self._tf_buffer = None

        self._delivered: set = set()
        self._attached_cube: Optional[str] = None
        self._total = 1
        self._index = 0
        self._task_text = ""

        self._wait_for_servers()

    # ---------------- 基础工具 ----------------
    @staticmethod
    def _wait_server(client, timeout: float) -> bool:
        """兼容不同 rclpy 版本：有的 wait_for_server 不返回 bool。"""
        result = client.wait_for_server(timeout_sec=timeout)
        if result is not None:
            return bool(result)
        ready = getattr(client, "server_is_ready", None)
        if ready is None:
            return True
        return bool(ready() if callable(ready) else ready)

    def _wait_for_servers(self):
        self.get_logger().info("等待 Nav2 / 机械臂 / 夹爪 / 吸附服务 ...")
        if not self._wait_server(self.nav_client, 20.0):
            raise RuntimeError("/navigate_to_pose 不存在，请先启动 nav_bringup_gazebo2.launch.py")
        if not self._wait_server(self.arm_client, 10.0):
            raise RuntimeError("/arm_controller 不存在，请先启动 gazebo_world2.launch.py")
        if not self._wait_server(self.gripper_client, 10.0):
            raise RuntimeError("/gripper_controller 不存在")
        if not self.attach_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/ATTACHLINK 不存在，请确认世界文件加载了 libgazebo_link_attacher.so")
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/DETACHLINK 不存在")
        self.get_logger().info("全部服务就绪。")

    def spin_for(self, seconds: float):
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            remain = end - time.monotonic()
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, remain)))

    def _on_model_states(self, msg: ModelStates):
        # 只缓存关心的模型，世界文件里上百个墙体/家具没必要每帧都转成 Python 对象
        objects = {}
        for name, pose in zip(msg.name, msg.pose):
            if not (name == ROBOT_MODEL or "cube" in name or "zone" in name):
                continue
            objects[name] = ObjectPose(
                pose.position.x, pose.position.y, pose.position.z,
                quat_to_yaw(pose.orientation))
        self._objects = objects
        self._objects_wall = time.monotonic()

    def _on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position

    def get_object_pose(self, name: str) -> Optional[ObjectPose]:
        """优先取 Gazebo 实时真值，其次取 positions.py 的静态坐标。"""
        if time.monotonic() - self._objects_wall < 5.0:
            pose = self._objects.get(name)
            if pose is not None:
                return pose
        if name in BLOCK_POSITIONS:
            x, y, z = BLOCK_POSITIONS[name]
            return ObjectPose(x, y, z, 0.0)
        if name in ZONE_POSITIONS:
            x, y, z = ZONE_POSITIONS[name]
            return ObjectPose(x, y, z, 0.0)
        return None

    def get_base_pose(self) -> Optional[BasePose]:
        if time.monotonic() - self._objects_wall < 5.0:
            pose = self._objects.get(ROBOT_MODEL)
            if pose is not None:
                return BasePose(pose.x, pose.y, pose.yaw)
        if self._tf_buffer is not None:
            try:
                tr = self._tf_buffer.lookup_transform("map", "base_footprint", RclpyTime())
                trans = tr.transform.translation
                return BasePose(trans.x, trans.y, quat_to_yaw(tr.transform.rotation))
            except Exception:
                return None
        return None

    def publish_status(self, stage: str, message: str, announce: bool = True, **kwargs):
        payload = {
            "stamp": round(self.get_clock().now().nanoseconds / 1e9, 3),
            "stage": stage,
            "message": message,
            "task": self._task_text,
            "index": self._index,
            "total": self._total,
            "delivered": len(self._delivered),
        }
        payload.update({k: v for k, v in kwargs.items() if v is not None})
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.state_pub.publish(String(data=STATE_TEXT.get(stage, "运行中")))
        if announce:
            self.get_logger().info(f"[{stage}] {message}")

    def publish_startup_panels(self):
        """启动时给看板一个确定状态，避免面板一直显示"等待消息"。"""
        self.state_pub.publish(String(data="待命"))
        self.current_pub.publish(String(data="待命"))
        self.progress_pub.publish(String(data="待命"))
        for color in ("red", "blue"):
            label = "红色" if color == "red" else "蓝色"
            self.count_pubs[color].publish(Int32(data=0))
            self.zone_pubs[color].publish(String(data=f"{label}到…区"))

    def publish_idle(self):
        self.publish_status("idle", "待命")
        self.current_pub.publish(String(data="待命"))
        self.progress_pub.publish(String(data="待命"))

    def _progress_label(self) -> str:
        if self._index > 0:
            return f"第 {self._index}/{self._total} 个"
        return "导航"

    # ---------------- 机械臂 ----------------
    def _current_arm_angles(self) -> Optional[List[float]]:
        try:
            return [float(self._joint_positions[name]) for name in JOINT_NAMES]
        except (KeyError, TypeError):
            return None

    def _send_traj(self, client, joint_names: Sequence[str], positions: Sequence[float],
                   seconds: float, label: str) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = max(0.5, float(seconds))
        point.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1.0) * 1e9))
        goal.trajectory.points = [point]

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f"{label}: 发送 action goal 超时")
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label}: goal 被拒绝（关节角超限？）")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=sec + 10.0)
        if not result_future.done():
            self.get_logger().error(f"{label}: 等待执行结果超时")
            return False
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"{label}: 执行结束但状态码为 {status}")
            return False
        return True

    def move_arm(self, positions: Sequence[float], label: str = "机械臂运动",
                 seconds: Optional[float] = None) -> bool:
        if len(positions) != len(JOINT_NAMES):
            self.get_logger().error(f"{label}: 关节数量错误")
            return False
        if seconds is None:
            current = self._current_arm_angles()
            if current:
                span = max(abs(a - b) for a, b in zip(positions, current))
                seconds = clamp(span / ARM_MOVE_MAX_SPEED, 0.8, 2.6)
            else:
                seconds = 2.8
        angles = "[" + ", ".join(f"{v:+.2f}" for v in positions) + "]"
        self.get_logger().info(f"[机械臂] {label} {angles} ({seconds:.1f}s)")
        return self._send_traj(self.arm_client, JOINT_NAMES, positions, seconds, label)

    def move_arm_named(self, name: str) -> bool:
        pose = POSE_DICT.get(name)
        if pose is None:
            self.get_logger().warn(f"未知姿态: {name}")
            return False
        return self.move_arm(pose, f"姿态 {name}")

    def move_gripper(self, position: float, label: str) -> bool:
        self.get_logger().info(f"[夹爪] {label} ({position:.3f})")
        return self._send_traj(self.gripper_client, [GRIPPER_JOINT], [position],
                               GRIPPER_MOVE_SECONDS, label)

    # ---------------- IK：根据真值生成抓取姿态 ----------------
    def plan_ik(self, target_x: float, target_y: float, grasp_z: float) -> Optional[List[float]]:
        base = self.get_base_pose()
        if base is None:
            self.get_logger().warn("拿不到机器人位姿，无法用 IK")
            return None
        dx, dy = target_x - base.x, target_y - base.y
        radius = math.hypot(dx, dy)
        if radius < MIN_REACH:
            self.get_logger().error(
                f"目标太近: {radius:.3f}m < {MIN_REACH}m，车头会撞到目标，需要先后退")
            return None
        if radius > MAX_REACH:
            self.get_logger().error(
                f"目标太远: {radius:.3f}m > {MAX_REACH}m，超出臂展，需要先前进")
            return None
        q1 = wrap_angle(math.atan2(dy, dx) - base.yaw)
        z_rel = grasp_z - ARM_BASE_Z
        for tilt in WRIST_TILTS:
            q = solve_arm_ik(radius, z_rel, tilt)
            if q is not None:
                q = list(q)
                q[0] = q1
                r_fk, z_fk, wrist = arm_fk(q)
                self.get_logger().info(
                    f"[IK] 目标 r={radius:.3f} z={grasp_z:.3f}，腕倾 {tilt:.0f}°，"
                    f"q1={q1:+.2f}，正解 r={r_fk:.3f} z={z_fk + ARM_BASE_Z:.3f} 腕={wrist:.0f}°")
                return q
        self.get_logger().error(f"IK 无解（r={radius:.3f}, z={grasp_z:.3f}, 倾斜到 40° 仍不可达）")
        return None

    def log_grasp_error(self, q: Sequence[float], target: ObjectPose):
        base = self.get_base_pose()
        if base is None:
            return
        r, z_rel, _ = arm_fk(q)
        angle = base.yaw + q[0]
        gx = base.x + r * math.cos(angle)
        gy = base.y + r * math.sin(angle)
        gz = z_rel + ARM_BASE_Z
        error = math.sqrt((gx - target.x) ** 2 + (gy - target.y) ** 2 + (gz - target.z) ** 2)
        self.get_logger().info(
            f"[校验] 夹爪中心 ({gx:.3f},{gy:.3f},{gz:.3f}) / 方块 "
            f"({target.x:.3f},{target.y:.3f},{target.z:.3f}) 偏差 {error * 100:.1f}cm")

    # ---------------- 导航与精停靠 ----------------
    def _approach_vector(self, name: str, target: ObjectPose) -> Tuple[float, float]:
        for prefix, vector in APPROACH_DIRS.items():
            if name.startswith(prefix):
                return vector
        base = self.get_base_pose()
        if base is not None:
            dx, dy = base.x - target.x, base.y - target.y
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                return dx / norm, dy / norm
        return -1.0, 0.0

    def _clear_costmaps(self):
        """请求 Nav2 清空局部/全局代价地图（MPPI 判为全碰撞时的标准恢复动作）。"""
        for name, client in (("local", self.local_clear_client),
                             ("global", self.global_clear_client)):
            if not client.service_is_ready():
                continue
            try:
                future = client.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)
                self.get_logger().info(f"[导航] 已请求清空 {name}_costmap")
            except Exception as exc:
                self.get_logger().warn(f"[导航] 清空 {name}_costmap 失败: {exc}")

    def navigate(self, x: float, y: float, yaw: float, label: str = "") -> bool:
        """只走 Nav2：失败就清代价地图重试，仍失败则放弃并给出诊断。

        MPPI 报 "Optimizer fail to compute path" 时（所有采样轨迹都被判碰撞），
        清一次代价地图往往就能恢复；但不再用 /cmd_vel 直驱兜底——直驱不避障，
        实测会直接撞上障碍物。
        """
        for attempt in range(1, max(1, NAV_RETRIES) + 1):
            if self._navigate_once(x, y, yaw, label):
                return True
            self.stop_base()
            if attempt < NAV_RETRIES:
                self.get_logger().warn(
                    f"[导航] {label} 第 {attempt} 次失败，清理代价地图后重试")
                self._clear_costmaps()
                self.spin_for(0.5)

        base = self.get_base_pose()
        if base is not None:
            where = (f"当前位姿 ({base.x:.2f},{base.y:.2f},"
                     f"{math.degrees(base.yaw):.0f}°)，"
                     f"距目标 {math.hypot(x - base.x, y - base.y):.2f}m")
        else:
            where = "位姿未知"
        self.get_logger().error(
            f"[导航] {label} Nav2 连续 {NAV_RETRIES} 次失败，放弃（{where}）。"
            "常见原因：目标点/走廊被障碍堵住，或机器人贴障碍太近导致 MPPI 无可行轨迹")
        return False

    def _navigate_once(self, x: float, y: float, yaw: float, label: str = "") -> bool:
        self.publish_status("navigate", f"导航前往 {label} ({x:.2f}, {y:.2f})")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=NAV_ACCEPT_TIMEOUT)
        if not future.done() or future.result() is None:
            self.get_logger().error("[导航] 发送目标超时（Nav2 未响应）")
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[导航] 目标被拒绝（可能在地图外/代价地图上）")
            return False

        result_future = goal_handle.get_result_async()
        # 导航期间每 2 秒刷新一次看板：进度显示"第 i/N 个 · 剩余 x.x m"
        label_text = self._progress_label()
        deadline = time.monotonic() + NAV_TIMEOUT
        last_tick = 0.0
        close_since = None
        while not result_future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            base = self.get_base_pose()
            remaining = None
            if base is not None:
                remaining = math.hypot(x - base.x, y - base.y)
                # 已经进入"够近"半径并稳住一小会儿：不再等 Nav2 的严格判定，
                # 取消目标直接返回，后面 dock_to_object 会把厘米级误差补掉。
                if remaining <= NAV_ARRIVE_EPS:
                    if close_since is None:
                        close_since = now
                    elif now - close_since >= NAV_ARRIVE_HOLD:
                        self.get_logger().info(
                            f"[导航] {label} 已进入 {remaining:.2f}m，"
                            "提前结束导航，交给精停靠修正")
                        goal_handle.cancel_goal_async()
                        self.spin_for(0.2)
                        self.stop_base()
                        self.progress_pub.publish(String(data=label_text))
                        return True
                else:
                    close_since = None
            if now - last_tick >= 2.0:
                last_tick = now
                if remaining is not None:
                    self.progress_pub.publish(String(
                        data=f"{label_text} · 剩余 {remaining:.1f}m"))
                self.state_pub.publish(String(data="导航中"))
        if not result_future.done():
            base = self.get_base_pose()
            remain_text = ""
            if base is not None:
                remain_text = f"，距目标 {math.hypot(x - base.x, y - base.y):.2f}m"
            self.get_logger().error(
                f"[导航] 超过 {NAV_TIMEOUT:.0f}s 未到达{remain_text}，取消目标")
            goal_handle.cancel_goal_async()
            self.spin_for(0.5)
            self.stop_base()
            return False
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            # 失败但人已经到附近了（常见于目标点在膨胀区内被 MPPI 拒绝），
            # 同样交给精停靠，不要让导航判死。
            base = self.get_base_pose()
            if base is not None:
                remaining = math.hypot(x - base.x, y - base.y)
                if remaining <= NAV_ARRIVE_EPS:
                    self.get_logger().warn(
                        f"[导航] {label} 报失败但已在 {remaining:.2f}m 内，"
                        "改由精停靠接管")
                    self.stop_base()
                    self.progress_pub.publish(String(data=label_text))
                    return True
            self.get_logger().error(f"[导航] 失败，状态码 {status}")
            return False
        self.progress_pub.publish(String(data=label_text))
        self.get_logger().info(f"[导航] 已到达 {label}")
        return True

    def navigate_near(self, name: str, target: ObjectPose, standoff: float = NAV_STANDOFF) -> bool:
        ux, uy = self._approach_vector(name, target)
        gx = target.x + ux * standoff
        gy = target.y + uy * standoff
        yaw = math.atan2(-uy, -ux)       # 车头朝向目标
        self.get_logger().info(
            f"[导航] {name}: 目标 ({target.x:.2f},{target.y:.2f})，"
            f"停靠 ({gx:.2f},{gy:.2f})，朝向 {math.degrees(yaw):.0f}°")
        return self.navigate(gx, gy, yaw, label=name)

    def _settle(self):
        self.spin_for(SETTLE_TIME)

    def _publish_twist(self, twist: Twist):
        """同时发 /cmd_vel 和 /cmd_vel_smoothed，兼容 Nav2 的 collision_monitor 链路。"""
        self.cmd_pub.publish(twist)
        self.cmd_pub2.publish(twist)

    def stop_base(self):
        zero = Twist()
        for _ in range(3):
            self._publish_twist(zero)
            self.spin_for(0.05)

    def dock_to_object(self, name: str, target: ObjectPose,
                       target_range: float = DOCK_RANGE) -> bool:
        """用 Gazebo 真值 + /cmd_vel 做厘米级停靠修正（Nav2 的 xy 容差是 0.15m，太松）。

        命令无效（话题不对/被碰撞监视器覆盖）时会自动放弃，不影响后续 IK 抓取。
        """
        if not self.use_dock:
            return False
        self.publish_status("dock", f"精停靠 {name}")
        start = time.monotonic()
        last_gap = None
        stable_since = start
        best = None
        while time.monotonic() - start < 6.0 and rclpy.ok():
            base = self.get_base_pose()
            if base is None:
                return False
            dx, dy = target.x - base.x, target.y - base.y
            distance = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            yaw_error = wrap_angle(bearing - base.yaw)
            gap = distance - target_range
            if best is None or abs(gap) < abs(best):
                best = gap
            if abs(gap) < DOCK_TOL and abs(yaw_error) < DOCK_YAW_TOL:
                self.stop_base()
                self.get_logger().info(
                    f"[精停靠] {name} 完成：距离 {distance:.3f}m，朝向误差 {math.degrees(yaw_error):+.1f}°")
                return True

            twist = Twist()
            if abs(yaw_error) < 0.35:
                twist.linear.x = clamp(2.4 * gap, -0.15, 0.15)
                if abs(gap) < 0.008:
                    twist.linear.x = 0.0
            twist.angular.z = clamp(1.95 * yaw_error, -0.9, 0.9)
            self._publish_twist(twist)

            # 1.5 秒内距离没有明显改善 → 认为速度指令没生效，别死等
            # （朝向偏差大时先原地转，这段时间距离本来就不变，不做判定）
            if last_gap is None or abs(gap) < abs(last_gap) - 0.01:
                last_gap = gap
                stable_since = time.monotonic()
            elif abs(yaw_error) < 0.2 and time.monotonic() - stable_since > 1.5:
                self.get_logger().warn(
                    "[精停靠] 底盘没有响应速度指令（检查 CMD_VEL_TOPIC / CMD_VEL_SMOOTHED_TOPIC），"
                    "直接使用当前位姿做 IK")
                self.stop_base()
                return False
            self.spin_for(0.1)

        self.stop_base()
        self.get_logger().warn(f"[精停靠] {name} 未收敛（最好误差 {best}m），继续用 IK 补偿")
        return False

    # ---------------- 吸附 / 释放 ----------------
    def attach_link(self, op: str, cube: str) -> bool:
        client = self.attach_client if op == "attach" else self.detach_client
        srv_type = AttachLink if op == "attach" else DetachLink
        if not client.service_is_ready():
            self.get_logger().error(f"{op}: 服务未就绪")
            return False
        request = srv_type.Request()
        request.model1_name = ARM_MODEL
        request.link1_name = ARM_LINK
        request.model2_name = cube
        request.link2_name = CUBE_LINK
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f"{op} {cube}: 调用超时")
            return False
        response = future.result()
        log = self.get_logger().info if response.success else self.get_logger().error
        log(f"{'吸附' if op == 'attach' else '释放'} {cube}: success={response.success} | {response.message}")
        if response.success:
            self._attached_cube = cube if op == "attach" else None
        return bool(response.success)

    # ---------------- 核心技能 ----------------
    def _align_for_reach(self, name: str, target: ObjectPose,
                         label: str = "抓取", grasp_z: float = PICK_GRASP_Z,
                         refresh_target: bool = True,
                         fallback_pose: Optional[List[float]] = None):
        """导航到目标前、精停靠，并确认停靠距离落在臂展内（最多修正一次）。

        refresh_target=False 用于放置：放置点是由区域中心算出来的，不能再被
        区域的实时位姿覆盖。返回 (target, q_above, q_grasp)，失败返回 None。
        """
        standoff = NAV_STANDOFF
        for attempt in (0, 1):
            if not self.navigate_near(name, target, standoff=standoff):
                return None
            self._settle()
            self.dock_to_object(name, target)
            self.spin_for(0.2)
            if refresh_target:
                target = self.get_object_pose(name) or target
            base = self.get_base_pose()
            if base is None:
                self.get_logger().warn("没有机器人真值，使用固定姿态（可能不准）")
                return target, POSE_ABOVE_DEFAULT, (fallback_pose or POSE_PICK_DEFAULT)

            distance = math.hypot(target.x - base.x, target.y - base.y)
            self.get_logger().info(f"[{label}] 第 {attempt + 1} 次停靠，车体到目标 {distance:.3f}m")
            if MIN_REACH <= distance <= MAX_REACH:
                q_above = self.plan_ik(target.x, target.y, ABOVE_GRASP_Z)
                q_grasp = self.plan_ik(target.x, target.y, grasp_z)
                if q_grasp is not None:
                    return target, q_above, q_grasp
                self.get_logger().error(f"[{label}] 距离合适但逆解失败（关节限位），放弃")
                return None

            if attempt == 0:
                standoff = 0.30 if distance > MAX_REACH else 0.42
                self.get_logger().warn(
                    f"[{label}] 停靠距离 {distance:.3f}m 不在 [{MIN_REACH:.2f}, {MAX_REACH:.2f}] 内，"
                    f"按 {standoff:.2f}m 重新导航修正")
                continue
            self.get_logger().error(f"[{label}] 修正后距离仍为 {distance:.3f}m，放弃")
            return None
        return None

    def pick(self, cube: str) -> bool:
        self.publish_status("pick", f"开始抓取 {cube}", cube=cube)
        target = self.get_object_pose(cube)
        if target is None:
            self.get_logger().error(f"找不到 {cube} 的坐标（扫描 positions.py 或检查 Gazebo）")
            return False
        self.get_logger().info(
            f"[抓取] {cube} 坐标 ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})")

        if self._attached_cube and self._attached_cube != cube:
            self.get_logger().warn(f"机械臂上还吸着 {self._attached_cube}，先释放")
            self.attach_link("detach", self._attached_cube)
            self.move_gripper(GRIPPER_OPEN, "张开夹爪")

        if not self.move_arm(POSE_CARRY, "收臂（行驶姿态）"):
            self.get_logger().warn("收臂失败，继续尝试")
        self.move_gripper(GRIPPER_OPEN, "张开夹爪")

        aligned = self._align_for_reach(cube, target, label="抓取", grasp_z=PICK_GRASP_Z)
        if aligned is None:
            self.move_arm(POSE_CARRY, "收臂")
            return False
        target, q_above, q_grasp = aligned

        self.publish_status("arm", f"下放机械臂抓取 {cube}")
        if q_above is not None:
            self.move_arm(q_above, "对准方块上方")
        if not self.move_arm(q_grasp, "下放到抓取点"):
            self.move_arm(POSE_CARRY, "收臂")
            return False
        self.log_grasp_error(q_grasp, target)

        self.move_gripper(GRIPPER_CLOSE, "闭合夹爪")
        self.spin_for(0.4)
        if not self.attach_link("attach", cube):
            self.get_logger().error("[抓取] 吸附失败")
            self.move_arm(q_above or POSE_ABOVE_DEFAULT, "抬臂")
            self.move_arm(POSE_CARRY, "收臂")
            return False

        self.publish_status("grasp", f"已吸附 {cube}", cube=cube)
        if q_above is not None:
            self.move_arm(q_above, "抬起机械臂")
        self.move_arm(POSE_CARRY, "收臂")

        self.spin_for(0.25)
        after = self.get_object_pose(cube)
        if after is not None and after.z < target.z + 0.08:
            self.get_logger().error(
                f"[抓取] 吸附后 {cube} 高度只有 {after.z:.3f}m，没有跟随抬起，判定失败")
            return False
        self.get_logger().info(f"[抓取] {cube} 抓取成功")
        return True

    def drop_point(self, zone: str) -> Optional[ObjectPose]:
        zone_pose = self.get_object_pose(zone)
        if zone_pose is None:
            return None
        return ObjectPose(zone_pose.x + DROP_OFFSET[0], zone_pose.y + DROP_OFFSET[1],
                          zone_pose.z, zone_pose.yaw)

    def place(self, zone: str, cube: str) -> bool:
        self.publish_status("place", f"前往 {zone} 放置 {cube}", zone=zone, cube=cube)
        drop = self.drop_point(zone)
        if drop is None:
            self.get_logger().error(f"找不到放置区 {zone} 的坐标")
            return False
        self.get_logger().info(f"[放置] {zone} 放置点 ({drop.x:.3f}, {drop.y:.3f})")

        aligned = self._align_for_reach(zone, drop, label="放置", grasp_z=PLACE_GRASP_Z,
                                        refresh_target=False, fallback_pose=POSE_PLACE_DEFAULT)
        if aligned is None:
            self.move_arm(POSE_CARRY, "收臂")
            return False
        drop, q_above, q_place = aligned

        self.publish_status("arm", f"下放机械臂放置 {cube}")
        if q_above is not None:
            self.move_arm(q_above, "对准放置点上方")
        if not self.move_arm(q_place, "下放到放置点"):
            self.move_arm(POSE_CARRY, "收臂")
            return False

        if not self.attach_link("detach", cube):
            self.get_logger().error("[放置] 解除吸附失败，重试一次")
            if not self.attach_link("detach", cube):
                self.get_logger().error("[放置] 仍然解除不了吸附，方块还在夹爪上，任务失败")
                return False
        self.move_gripper(GRIPPER_OPEN, "张开夹爪")
        self.spin_for(0.2)
        if q_above is not None:
            self.move_arm(q_above, "抬起机械臂")
        self.move_arm(POSE_CARRY, "收臂")

        self.spin_for(0.3)
        final = self.get_object_pose(cube)
        if final is not None:
            error = math.hypot(final.x - drop.x, final.y - drop.y)
            self.get_logger().info(
                f"[放置] {cube} 落点 ({final.x:.2f},{final.y:.2f},{final.z:.3f})，"
                f"距目标 {error * 100:.0f}cm")
            if final.z > 0.10:
                self.get_logger().error("[放置] 方块还在空中，说明没有真正释放")
                return False
            if error > 0.35:
                self.get_logger().warn("[放置] 落点离目标偏远，检查导航/臂展")
        return True

    def deliver(self, cube: str, zone: str) -> bool:
        started = time.monotonic()
        self._index += 1
        self.current_pub.publish(String(
            data=f"{cube_display_name(cube)} → {zone_display_name(zone)}"))
        self.progress_pub.publish(String(data=f"第 {self._index}/{self._total} 个"))
        self.publish_status("deliver", f"任务 {self._index}/{self._total}：{cube} → {zone}",
                            cube=cube, zone=zone)
        if not self.pick(cube):
            self.publish_status("error", f"{cube} 抓取失败", cube=cube, zone=zone, success=False)
            self.safe_state()
            return False
        if not self.place(zone, cube):
            self.publish_status("error", f"{cube} 放置失败", cube=cube, zone=zone, success=False)
            self.safe_state()
            return False
        self._delivered.add(cube)
        elapsed = time.monotonic() - started
        self.current_pub.publish(String(
            data=f"{cube_display_name(cube)} 已送达 {zone_display_name(zone)}"))
        self.publish_status("delivered", f"{cube} 已送到 {zone}（用时 {elapsed:.0f}s）",
                            cube=cube, zone=zone, success=True)
        return True

    def deliver_many(self, color: str, count: int, zone: str, task: str = "",
                     update_total: bool = True) -> bool:
        count = int(clamp(int(count), 1, 5))
        candidates = self.available_cubes(color)
        if len(candidates) < count:
            self.get_logger().warn(
                f"只有 {len(candidates)} 个可用的 {color} 方块，少于要求的 {count} 个")
            count = len(candidates)
        if count <= 0:
            return False
        if update_total:
            self._total = count
            self._index = 0
        if task:
            self._task_text = task
        for cube in candidates[:count]:
            if not self.deliver(cube, zone):
                return False
        self.publish_status("delivered", f"全部完成：{count} 个 {color} 方块 → {zone}", success=True)
        return True

    # ---------------- 出题程序：两阶段解题（先算 x/y，再套映射规则）----------------
    def _llm_call(self, messages: List[dict], max_tokens: int,
                  model: str = "") -> Tuple[str, str, str]:
        """单次对话调用，返回 (content, reasoning_content, error)。"""
        if self._http is None:
            reason = "未安装 requests" if not HAVE_REQUESTS else "未设置 DEEPSEEK_API_KEY"
            return "", "", f"{CONN_ERROR}：{reason}"
        payload = {
            "model": model or MODEL_NAME,
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": max_tokens,
        }
        headers = {"Authorization": f"Bearer {DEEPSEEK_API_KEY}",
                   "Content-Type": "application/json"}
        try:
            response = self._http.post(DEEPSEEK_URL, headers=headers, json=payload,
                                       timeout=LLM_TIMEOUT_TASK)
            response.raise_for_status()
            message = response.json()["choices"][0]["message"]
            return ((message.get("content") or "").strip(),
                    (message.get("reasoning_content") or "").strip(), "")
        except requests.exceptions.Timeout:
            return "", "", f"{CONN_ERROR}：请求超时（{LLM_TIMEOUT_TASK:.0f}s）"
        except requests.exceptions.ConnectionError as exc:
            return "", "", f"{CONN_ERROR}：网络不可达（{exc.__class__.__name__}）"
        except requests.exceptions.HTTPError as exc:
            return "", "", f"{CONN_ERROR}：接口返回错误（{exc}）"
        except Exception as exc:
            return "", "", f"{CONN_ERROR}：{exc.__class__.__name__}: {exc}"

    def solve_xy_with_llm(self, problem: str):
        """阶段一：让大模型列算式解出 x、y。不满足 x+y=5 时带约束追问一次。

        返回 (x, y, 演算文本, 错误)。
        """
        model = REASON_MODEL_NAME or MODEL_NAME
        messages = [
            {"role": "system", "content": XY_SYSTEM_PROMPT},
            {"role": "user", "content": clean_problem_text(problem)},
        ]
        content, reasoning, error = self._llm_call(messages, LLM_MAX_TOKENS_XY, model)
        if error:
            return None, None, "", error
        text = content or reasoning
        matches = XY_ANSWER_RE.findall(text)
        if not matches:
            return None, None, text, f"{CONN_ERROR}：回复中找不到 x=<数>,y=<数>"
        x, y = int(matches[-1][0]), int(matches[-1][1])

        if x + y != EXPECTED_TOTAL:
            # 结果不满足题目约束，带约束追问一次（不打印过程）
            messages.append({"role": "assistant", "content": content})
            messages.append({"role": "user", "content": (
                f"你给出 x={x}, y={y}，但 x+y={x + y}，而题目保证 x+y={EXPECTED_TOTAL} "
                "且 x>=1、y>=1。请重新逐句累加每种物品的总需求量"
                "（注意『甲、乙都需要 n 个』要乘以人数），重新按 ceil(总量/N) 计算，"
                "最后只输出一行：x=<整数>,y=<整数>")})
            content2, reasoning2, error2 = self._llm_call(messages, LLM_MAX_TOKENS_XY, model)
            if error2:
                text = f"{text}\n--- 追问失败: {error2} ---"
            else:
                text2 = content2 or reasoning2
                matches2 = XY_ANSWER_RE.findall(text2)
                if matches2:
                    x, y = int(matches2[-1][0]), int(matches2[-1][1])
                text = f"{text}\n--- 追问修正 ---\n{text2}"

        return x, y, text, ""

    def apply_mapping_with_llm(self, x: int, y: int, rule: str,
                               problem: str) -> Tuple[List[dict], str, str]:
        """阶段二：把已确定的 x、y 交给大模型套用本次映射规则。

        返回 (方案, 原始回复, 错误)。规则没分离出来时退化为把整题给它自行判断。
        """
        context = rule if rule else clean_problem_text(problem)
        user = (f"已知：x={x}, y={y}\n"
                f"本次映射规则：{context}\n"
                "请严格按本次规则输出方案，只输出一行。")
        content, _reasoning, error = self._llm_call(
            [{"role": "system", "content": MAP_SYSTEM_PROMPT},
             {"role": "user", "content": user}], LLM_MAX_TOKENS_TASK)
        if error:
            return [], "", error
        entries = parse_plan_text(content)
        if not entries:
            return [], content, "大模型未给出可解析的方案（回复里没有 [color,count,zone]）"
        return entries, content, ""

    def solve_task_with_llm(self, problem: str) -> Tuple[List[dict], dict]:
        """两阶段解题入口，返回 (方案, 诊断信息)。info["error"] 非空即失败。"""
        body, rule = extract_mapping_rule(problem)
        info: dict = {"body": body, "rule": rule, "x": None, "y": None,
                      "reasoning": "", "llm_raw": "", "warnings": [], "error": ""}

        if self._http is None:
            reason = "未安装 requests" if not HAVE_REQUESTS else "未设置 DEEPSEEK_API_KEY"
            info["error"] = f"{CONN_ERROR}：{reason}"
            return [], info

        x, y, reasoning, error = self.solve_xy_with_llm(problem)
        info["reasoning"] = reasoning
        if error:
            info["error"] = error
            return [], info
        info["x"], info["y"] = x, y
        if x + y != EXPECTED_TOTAL:
            info["warnings"].append(
                f"x+y={x + y}，不等于题目保证的 {EXPECTED_TOTAL}，大模型可能算错")

        entries, raw, error = self.apply_mapping_with_llm(x, y, rule, problem)
        info["llm_raw"] = raw
        if error:
            info["error"] = error
            return [], info

        info["warnings"].extend(validate_plan(entries, x, y))
        for warning in info["warnings"]:
            self.get_logger().warn(f"[题目] {warning}")
        return entries, info

    def publish_task_text(self, problem: str, entries: Optional[List[dict]],
                          source: str, done: bool = False,
                          info: Optional[dict] = None):
        """把种子、题目、映射规则和最终方案发到 /mission/task。

        Foxglove 用 Raw Message 面板显示，内容形如：
            种子: 997072
            题目: 每个礼品盒有5个糖果;...
            映射规则: x 代表红色,y 代表蓝色;...
            方案: [red,4,A;blue,1,C]
        """
        seed = extract_seed(problem)
        body, rule = extract_mapping_rule(problem)
        lines = [f"种子: {seed or '-'}"]
        if body:
            lines.append(f"题目: {body}")
        if rule:
            lines.append(f"映射规则: {rule}")
        if entries:
            lines.append(f"方案: {format_plan(entries)}")
        else:
            error = (info or {}).get("error") or "解析失败"
            lines.append(f"方案: [ERROR] {error}")
        self.task_pub.publish(String(data="\n".join(lines)))

    def publish_task_info(self, problem: str, entries: Optional[List[dict]], source: str,
                          info: Optional[dict] = None):
        """把题目、方案、红/蓝数量与去向一起发到看板。"""
        self.publish_task_text(problem, entries, source, info=info)
        counts = {"red": 0, "blue": 0}
        zones = {"red": "", "blue": ""}
        for entry in entries or []:
            color = normalize_color(entry.get("color", ""))
            if color is None:
                continue
            counts[color] += int(entry.get("count", 0))
            zones[color] = zone_display_name(entry.get("zone", ""))
        for color in ("red", "blue"):
            label = "红色" if color == "red" else "蓝色"
            self.count_pubs[color].publish(Int32(data=int(counts[color])))
            if counts[color] > 0 and zones[color]:
                self.zone_pubs[color].publish(String(data=f"{label}到{zones[color]}"))
            else:
                self.zone_pubs[color].publish(String(data=f"{label}：无"))

    def execute_plan(self, entries: List[dict], problem: str = "", source: str = "",
                     info: Optional[dict] = None) -> bool:
        """按方案顺序逐个颜色执行 deliver_many，进度跨颜色连续累计。"""
        cleaned = []
        for entry in entries:
            color = normalize_color(entry.get("color", ""))
            zone = normalize_zone(entry.get("zone", ""))
            count = parse_int(entry.get("count", 0))
            if color and zone and count and 1 <= count <= 5:
                cleaned.append({"color": color, "count": count, "zone": zone})
        if not cleaned:
            self.get_logger().error("[题目] 没有可执行的搬运项")
            return False

        plan_text = format_plan(cleaned)
        # 执行顺序：先蓝色（全部搬完），再红色；同色内部由 available_cubes 按 1~5 编号依次搬
        execution = sorted(cleaned, key=lambda e: 0 if e["color"] == "blue" else 1)
        self._task_text = plan_text
        self._total = sum(int(e["count"]) for e in cleaned)
        self._index = 0
        self.publish_task_info(problem, cleaned, source, info=info)
        self.publish_status("task", f"开始搬运 {plan_text}", announce=False,
                            plan=plan_text, problem=problem)
        self.current_pub.publish(String(data="开始执行方案"))
        self.progress_pub.publish(String(data=f"0/{self._total} 个"))
        self.state_pub.publish(String(data=STATE_TEXT["deliver"]))

        for entry in execution:
            if not self.deliver_many(entry["color"], entry["count"], entry["zone"],
                                     task=plan_text, update_total=False):
                self.current_pub.publish(String(data="任务中断"))
                self.publish_status("error", f"任务中断于 {format_plan([entry])}", success=False)
                return False
        self.current_pub.publish(String(data="全部完成"))
        self.progress_pub.publish(String(data=f"{self._index}/{self._total} 个"))
        self.publish_status("done", f"出题任务全部完成 {plan_text}", announce=False,
                            plan=plan_text, success=True)
        return True

    def run_task(self, problem: str) -> bool:
        """出题程序入口：两阶段大模型解题（先 x/y，再套映射规则）→ 校验 → 执行。"""
        self._task_text = ""
        self.publish_status("task", "收到出题题目，开始解析", announce=False)
        self.current_pub.publish(String(data="大模型解题中…"))
        self.progress_pub.publish(String(data="解析中"))

        entries, info = self.solve_task_with_llm(problem)
        if not entries:
            message = info.get("error") or "解题失败"
            self.get_logger().error(f"[ERROR] {message}")
            self.publish_task_text(problem, None, "", info=info)
            self.state_pub.publish(String(data=STATE_TEXT["error"]))
            self.publish_status("error", message, success=False)
            self.current_pub.publish(String(data="解题失败"))
            self.progress_pub.publish(String(data="—"))
            return False

        plan_text = format_plan(entries)
        # 终端只打印最终方案，例如 [red,4,A;blue,1,C]；x/y 只留在 info 里不输出
        source = "大模型"
        self.get_logger().info(plan_text)
        total = sum(int(e["count"]) for e in entries)
        self.current_pub.publish(String(data="方案已解析"))
        self.progress_pub.publish(String(data=f"0/{total} 个"))
        if not TASK_AUTO_RUN:
            self.publish_task_info(problem, entries, source, info=info)
            self.current_pub.publish(String(data="方案已解析，等待指令"))
            self.publish_status("idle", "方案已解析，等待逐条指令（TASK_AUTO_RUN=0）")
            return True
        return self.execute_plan(entries, problem, source, info=info)

    def available_cubes(self, color: str) -> List[str]:
        """还没搬走、仍在地面上的同色方块，严格按编号 1~5 排序。"""
        result = []
        for index in range(1, 6):
            name = f"{color}_cube_{index}"
            pose = self.get_object_pose(name)
            if pose is None or pose.z > 0.10 or name in self._delivered:
                continue
            result.append(name)
        return result

    def safe_state(self):
        try:
            self.stop_base()
        except Exception:
            pass
        try:
            self.move_arm(POSE_CARRY, "安全收臂")
            # 吸着方块时不要张爪，否则会把它从夹爪侧面拖出来
            if self._attached_cube is None:
                self.move_gripper(GRIPPER_OPEN, "张开夹爪")
        except Exception as exc:
            self.get_logger().warn(f"安全复位失败: {exc}")

    # ---------------- 大模型与工具分发 ----------------
    def llm_plan(self, text: str) -> List[Tuple[str, dict]]:
        if self._http is None:
            reason = "未安装 requests" if not HAVE_REQUESTS else "未设置 DEEPSEEK_API_KEY"
            self.get_logger().error(f"{CONN_ERROR}：{reason}")
            return []
        payload = {
            "model": MODEL_NAME,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
            "tools": TOOLS,
            "tool_choice": "auto",
            "temperature": 0.0,
            "max_tokens": LLM_MAX_TOKENS,
        }
        headers = {"Authorization": f"Bearer {DEEPSEEK_API_KEY}",
                   "Content-Type": "application/json"}
        try:
            response = self._http.post(DEEPSEEK_URL, headers=headers, json=payload,
                                       timeout=LLM_TIMEOUT_COMMAND)
            response.raise_for_status()
            message = response.json()["choices"][0]["message"]
        except Exception as exc:
            self.get_logger().error(f"{CONN_ERROR}：大模型调用失败（{exc}）")
            return []

        calls = []
        for tool_call in message.get("tool_calls") or []:
            function = tool_call.get("function", {})
            try:
                args = json.loads(function.get("arguments") or "{}")
            except json.JSONDecodeError:
                args = {}
            calls.append((function.get("name", ""), args))
        if not calls and message.get("content"):
            self.get_logger().info(f"大模型回复: {message['content']}")
        return calls

    def execute_tool(self, name: str, args: dict) -> bool:
        self.get_logger().info(f"执行动作: {name} {args}")
        if name == "deliver_cube":
            color = normalize_color(args.get("color", ""))
            zone = normalize_zone(args.get("zone", ""))
            cube_id = parse_int(args.get("id", 0))
            if color is None or zone is None or not cube_id:
                self.get_logger().error(f"参数不完整: {args}")
                return False
            cube_id = int(clamp(cube_id, 1, 5))
            self._total = 1
            self._index = 0
            if args.get("task"):
                self._task_text = str(args["task"])
            ok = self.deliver(f"{color}_cube_{cube_id}", zone)
            if ok:
                self.publish_status("done", "指令完成", success=True)
            return ok

        if name == "deliver_cubes":
            color = normalize_color(args.get("color", ""))
            zone = normalize_zone(args.get("zone", ""))
            count = parse_int(args.get("count", 0))
            if color is None or zone is None or not count:
                self.get_logger().error(f"参数不完整: {args}")
                return False
            ok = self.deliver_many(color, count, zone, str(args.get("task", "")))
            if ok:
                self.publish_status("done", "指令完成", success=True)
            return ok

        if name == "navigate_to":
            target = str(args.get("target", ""))
            pose = self.get_object_pose(target)
            if pose is None:
                self.get_logger().error(f"未知导航目标: {target}")
                return False
            ok = self.navigate_near(target, pose)
            if ok:
                self._settle()
            return ok

        if name == "move_to_pose":
            return self.move_arm_named(str(args.get("pose", "")))

        if name == "control_gripper":
            if args.get("action") == "close":
                return self.move_gripper(GRIPPER_CLOSE, "闭合夹爪")
            return self.move_gripper(GRIPPER_OPEN, "张开夹爪")

        if name == "attach_cube":
            color = normalize_color(args.get("color", ""))
            cube_id = parse_int(args.get("id", 0))
            if color is None or not cube_id:
                return False
            cube = f"{color}_cube_{int(clamp(cube_id, 1, 5))}"
            return self.attach_link("attach" if args.get("op") == "attach" else "detach", cube)

        self.get_logger().warn(f"未知动作: {name}")
        return False

    def initialize_initial_pose(self):
        """用 Gazebo 真值给 AMCL 一个初始位姿，替代 RViz 里手动点 2D Pose Estimate。"""
        self.get_logger().info("等待 Gazebo 真值以初始化 AMCL 位姿 ...")
        for _ in range(30):
            self.spin_for(0.2)
            base = self.get_base_pose()
            if base is not None:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose.position.x = base.x
                msg.pose.pose.position.y = base.y
                msg.pose.pose.orientation = yaw_to_quat(base.yaw)
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.068
                for _ in range(3):
                    self.initpose_pub.publish(msg)
                    self.spin_for(0.1)
                self.get_logger().info(
                    f"已发布 /initialpose: ({base.x:.2f}, {base.y:.2f}, {math.degrees(base.yaw):.0f}°)")
                return
        self.get_logger().warn("拿不到 Gazebo 真值，请仍用 RViz 的 2D Pose Estimate")


# ============================================================================
# 9. 入口
# ============================================================================
BANNER = """
====================================================================
 大模型搬运控制器  ·  导航 → 抓取 → 搬运 → 放置  全流程
--------------------------------------------------------------------
 示例指令：
   将红方块1送到A区域
   把蓝色方块3搬到B区
   搬运2个红色方块到A区
 命令：help 帮助 | exit 退出
 出题程序：输入"题目"后粘贴题目+映射规则（空行结束）
   或直接：plan [red,4,A;blue,1,C]
====================================================================
"""


def print_help():
    print(BANNER)
    print("可用工具（调试）：")
    print("  move_to_pose(home/lift/carry/pick/place)")
    print("  control_gripper(close/open)")
    print("  attach_cube(attach/detach, color, id)")
    print("  navigate_to(red_cube_1 / blue_cube_2 / zone_a ...)")
    print("  plan [red,4,A;blue,1,C]     直接执行解析好的方案")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description="大模型全流程搬运控制器")
    parser.add_argument("--no-dock", action="store_true", help="关闭厘米级精停靠")
    parser.add_argument("--init-pose", action="store_true", help="用真值自动发布 /initialpose")
    args, _ = parser.parse_known_args()

    print(BANNER)
    # 所有自然语言理解都走大模型 API：连接不可用就直接退出，不做本地兜底
    if not HAVE_REQUESTS:
        print(f"[启动失败] {CONN_ERROR}：未安装 requests（pip3 install requests）")
        return 3
    if not DEEPSEEK_API_KEY:
        print(f"[启动失败] {CONN_ERROR}：未设置环境变量 DEEPSEEK_API_KEY")
        return 3
    if POSITIONS_SOURCE:
        print(f"[坐标] 使用 {POSITIONS_SOURCE}")
    else:
        print("[坐标] 未找到 positions.py，使用内置坐标（请先运行 scan_positions.py）")
    print(f"[坐标] 方块 {len(BLOCK_POSITIONS)} 个，放置区 {len(ZONE_POSITIONS)} 个")

    rclpy.init()
    try:
        node = MissionNode(use_dock=not args.no_dock)
    except Exception as exc:
        print(f"[启动失败] {exc}")
        rclpy.shutdown()
        return 1

    try:
        if args.init_pose:
            node.initialize_initial_pose()
        node.spin_for(0.5)
        node.publish_startup_panels()
        base = node.get_base_pose()
        if base is not None:
            print(f"[定位] 机器人当前位置 ({base.x:.2f}, {base.y:.2f}, {math.degrees(base.yaw):.0f}°)\n")
        for zone in ("zone_a", "zone_b", "zone_c"):
            pose = node.get_object_pose(zone)
            if pose is not None:
                print(f"[场景] {zone}: ({pose.x:.2f}, {pose.y:.2f})")
        print()

        while rclpy.ok():
            try:
                text = input("请输入指令 (help/exit): ").strip()
            except EOFError:
                break
            if not text:
                continue
            if text.lower() in ("exit", "quit", "q", "退出"):
                break
            if text.lower() in ("help", "?", "帮助"):
                print_help()
                continue

            # 出题程序的题目可能很长/多行：先输入"题目"，再整段粘贴
            if text.lower() in ("题目", "题目:", "task", "出题", "problem"):
                print("请粘贴题目和映射规则（可多行），空行结束：")
                lines = []
                while True:
                    try:
                        line = input()
                    except EOFError:
                        break
                    if not line.strip():
                        break
                    lines.append(line.strip())
                text = " ".join(lines).strip()
                if not text:
                    continue

            # 手动执行已经解析好的方案，例如：plan [red,4,A;blue,1,C]
            if text.lower().startswith(("plan ", "方案 ", "执行方案 ")):
                entries = parse_plan_text(text)
                if not entries:
                    node.get_logger().warn("方案格式不对，示例：plan [red,4,A;blue,1,C]")
                    continue
                node.execute_plan(entries, problem="(手动输入方案)", source="手动")
                continue

            # 出题程序：题目 + 映射规则 → 大模型解题 → 解析 → 批量搬运
            if is_task_problem(text):
                node.run_task(text)
                continue

            # 简单指令：只走大模型 API，失败即报错，不做本地兜底
            calls = node.llm_plan(text)
            if not calls:
                node.get_logger().error(
                    "大模型未能解析该指令（请检查网络/API Key），"
                    "或说明方块颜色/编号和目标区域，例如：将红方块1送到A区域")
                continue

            for name, tool_args in calls:
                if not node.execute_tool(name, tool_args):
                    node.get_logger().warn(f"动作 {name} 执行失败，停止本次任务")
                    break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，退出")
    except Exception as exc:
        node.get_logger().error(f"运行出错: {exc}")
    finally:
        try:
            node.stop_base()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
