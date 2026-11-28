#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务解析链路的离线验证（不启动 ROS / Gazebo，也不需要 API Key）。

被测对象：arm_llm/arm_llm/deepseek_all_controller.py 里的纯逻辑函数
    - extract_seed / clean_problem_text / extract_mapping_rule
    - is_task_problem / parse_plan_text / format_plan
    - MissionNode.publish_task_text 的输出格式

做法：用最小桩替换 rclpy 等 ROS 模块后再导入控制器，只调纯函数。
运行：python3 test_task_pipeline.py
退出码：0 全部通过，1 有失败。
"""

import importlib.util
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
# 优先用 portfolio/src 里的源码副本（上传到 GitHub 后只有这份可读），
# 本地开发时回退到原工程根目录。
_CANDIDATES = [HERE.parent / "src", HERE.parents[2]]
ROOT = next(
    (p for p in _CANDIDATES if (p / "arm_llm" / "arm_llm").is_dir()),
    _CANDIDATES[0])
CONTROLLER = ROOT / "arm_llm" / "arm_llm" / "deepseek_all_controller.py"


# --------------------------------------------------------------------------
# 最小 ROS 桩：只为了让控制器能被 import，不模拟任何行为
# --------------------------------------------------------------------------
class _Dummy:
    def __init__(self, *args, **kwargs):
        pass


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(m, key, value)
        sys.modules[name] = m

    class _Msg:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class _Node:
        def __init__(self, *args, **kwargs):
            pass

    mod("rclpy")
    mod("rclpy.node", Node=_Node)
    mod("rclpy.action", ActionClient=_Dummy)
    mod("rclpy.time", Time=_Dummy)
    sys.modules["rclpy"].spin_until_future_complete = lambda *a, **k: None
    sys.modules["rclpy"].spin_once = lambda *a, **k: None
    sys.modules["rclpy"].ok = lambda: True
    mod("action_msgs")
    mod("action_msgs.msg", GoalStatus=type("G", (), {"STATUS_SUCCEEDED": 4}))
    mod("builtin_interfaces")
    mod("builtin_interfaces.msg", Duration=_Dummy)
    mod("control_msgs")
    mod("control_msgs.action", FollowJointTrajectory=_Dummy)
    mod("gazebo_msgs")
    mod("gazebo_msgs.msg", ModelStates=_Dummy)
    mod("geometry_msgs")
    mod("geometry_msgs.msg", Quaternion=_Dummy, Twist=_Dummy,
        PoseWithCovarianceStamped=_Dummy)
    mod("linkattacher_msgs")
    mod("linkattacher_msgs.srv", AttachLink=_Dummy, DetachLink=_Dummy)
    mod("nav2_msgs")
    mod("nav2_msgs.action", NavigateToPose=_Dummy)
    mod("nav2_msgs.srv", ClearEntireCostmap=type(
        "C", (), {"Request": staticmethod(lambda: None)}))
    mod("sensor_msgs")
    mod("sensor_msgs.msg", JointState=_Dummy)
    mod("std_msgs")
    mod("std_msgs.msg", Int32=_Msg, String=_Msg)
    mod("trajectory_msgs")
    mod("trajectory_msgs.msg", JointTrajectoryPoint=_Dummy)

    # requests 也打成桩，避免本机未安装时导入失败
    fake = types.ModuleType("requests")
    fake.Session = type("Session", (), {})
    fake.exceptions = types.SimpleNamespace(
        Timeout=type("T", (Exception,), {}),
        ConnectionError=type("C", (Exception,), {}),
        HTTPError=type("H", (Exception,), {}))
    sys.modules["requests"] = fake


def load_controller():
    _install_stubs()
    spec = importlib.util.spec_from_file_location("controller", CONTROLLER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------------------
# 测试用例
# --------------------------------------------------------------------------
SAMPLE = """使用种子: 997072
------------------------------------------------------------
题目: 每个礼品盒有5个糖果;有软糖、硬糖两种礼品盒;小红想要1个软糖,5个硬糖;小明、小刚都想要1个软糖;小玉想要1个软糖,5个硬糖;小兰想要7个硬糖。设软糖礼盒数量为x,硬糖礼盒数量为y,计算x、y分别为多少。映射规则1. "x 代表红色,y 代表蓝色;数值大于等于 3 的去 A 区,其他颜色去 C 区" """

RULES = {
    "规则1": 'x 代表红色,y 代表蓝色;数值大于等于 3 的去 A 区,其他颜色去 C 区',
    "规则2": 'x y 中,数值大于等于 3 值的代表红色，否则代表蓝色；x 去 B 区,y 去 A 区',
    "规则3": '蓝色去 B 区,红色去 C 区; x y 中,数值大的是蓝色，数值少的是红色',
}


def run():
    print(f"源码路径: {CONTROLLER}")
    ctrl = load_controller()
    failures = []

    def check(cond, msg):
        print(("  PASS  " if cond else "  FAIL  ") + msg)
        if not cond:
            failures.append(msg)

    print("== 1. 种子 / 分隔线剥离 ==")
    seed = ctrl.extract_seed(SAMPLE)
    check(seed == "997072", f"extract_seed -> {seed!r}")
    cleaned = ctrl.clean_problem_text(SAMPLE)
    check("使用种子" not in cleaned, "clean_problem_text 去掉了种子行")
    check("----" not in cleaned, "clean_problem_text 去掉了分隔线")
    check(cleaned.startswith("每个礼品盒"), f"正文起始 -> {cleaned[:16]!r}")

    print("\n== 2. 题目 / 映射规则分离 ==")
    body, rule = ctrl.extract_mapping_rule(SAMPLE)
    check(body.startswith("每个礼品盒"), f"题目 -> {body[:16]!r}...")
    check(rule.startswith("x 代表红色"), f"规则 -> {rule[:24]!r}...")
    check("使用种子" not in body and "----" not in body, "题目里不含种子/分隔线")

    print("\n== 3. 三种公布映射规则的提取（编号+引号+无前缀） ==")
    for name, text in RULES.items():
        payload = f"题目: 某题。映射规则{name[-1]}. \"{text}\""
        _, got = ctrl.extract_mapping_rule(payload)
        ok = got.replace(" ", "") == text.replace(" ", "")
        check(ok, f"{name} 提取 -> {got[:30]!r}")

    print("\n== 4. 任务题 / 简单指令判别 ==")
    check(ctrl.is_task_problem(SAMPLE) is True, "出题输入判为任务题")
    check(ctrl.is_task_problem("将红方块1送到A区域") is False, "简单指令不误判为任务题")
    check(ctrl.is_task_problem("搬运2个蓝色方块到B区") is False, "第二条简单指令不误判")

    print("\n== 5. 方案解析（容忍模型啰嗦/带示例） ==")
    noisy = ("比如 [red,2,C;blue,3,A] 只是格式示例。\n"
             "本题 x=4>=3 所以红色去 A。\n答案: [red,4,A;blue,1,C]")
    entries = ctrl.parse_plan_text(noisy)
    plan = ctrl.format_plan(entries)
    check(plan == "[red,4,A;blue,1,C]", f"取末尾答案 -> {plan}")
    check(len(entries) == 2, f"解析出 2 项，实得 {len(entries)}")

    print("\n== 6. /mission/task 发布格式 ==")

    class Pub:
        def __init__(self):
            self.msgs = []

        def publish(self, msg):
            self.msgs.append(getattr(msg, "data", msg))

    node = object.__new__(ctrl.MissionNode)
    node.task_pub = Pub()
    node.publish_task_text(SAMPLE, entries, "大模型")
    text = node.task_pub.msgs[-1]
    lines = text.splitlines()
    check(lines[0] == "种子: 997072", f"首行 -> {lines[0]!r}")
    check(lines[1].startswith("题目: 每个礼品盒"), "第二行为题目")
    check(lines[2].startswith("映射规则: x 代表红色"), "第三行为映射规则")
    check(lines[3] == "方案: [red,4,A;blue,1,C]", f"末行 -> {lines[3]!r}")
    check(len(lines) == 4, f"共 4 行，实得 {len(lines)}")

    print("\n== 7. 投放点居中 ==")
    check(ctrl.DROP_OFFSET == (0.0, 0.0), f"DROP_OFFSET -> {ctrl.DROP_OFFSET}")

    print("\n" + "=" * 52)
    if failures:
        print(f"结果: {len(failures)} 项失败")
        return 1
    print("结果: 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(run())
