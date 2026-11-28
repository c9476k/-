# LLM 驱动的移动抓取机器人（ROS 2 + Gazebo + DeepSeek）

> 自然语言 / 随机应用题 → 结构化搬运方案 → Nav2 自主导航 → 六轴机械臂抓取 → 精准放置 → 可视化看板

一个在 **ROS 2 Humble + Gazebo Classic 11** 上跑通的移动操作（mobile manipulation）系统：
用大模型把"人话"或"随机数学题"翻译成可执行的搬运方案，再由导航与机械臂完成闭环执行。

**English abstract** — An LLM-driven mobile manipulator on ROS 2 Humble. A two-stage LLM pipeline
turns a randomized word problem plus a mapping rule into a structured plan
(`[red,4,A;blue,1,C]`), which a Nav2 + 6-DOF-arm controller executes end-to-end in Gazebo.
Includes unit-tested parsing logic, geometry validation, and documented debugging of TF, physics,
and controller-tuning failures.



---

## 1. 它解决什么问题

比赛/演示场景是：出题程序随机生成应用题和映射规则（例如"x 代表红色，数值 ≥3 去 A 区"），
机器人需要 **自己解题、自己决定搬什么颜色几个、搬到哪里，然后真的搬过去**。

本项目的目标是把这条链路做成闭环：

```
随机题目 + 映射规则
        │
        ▼  （两阶段大模型）
   [red,4,A;blue,1,C]
        │
        ▼  （任务执行器）
 Nav2 导航 → 精停靠 0.34m → 六轴 IK 抓取 → 吸附 → 导航到区域 → 下放 → 释放
        │
        ▼
 Foxglove 看板（种子 / 题目 / 规则 / 方案 / 进度）
```

## 2. 我做了什么（范围与归属）

诚实说明，便于评估真实贡献：

| 部分 | 来源 |
|---|---|
| 底盘/机械臂 URDF、Nav2 与 MoveIt 配置、LinkAttacher 插件 | 赛事提供的 `yzbot` 工程（第三方基础） |
| **LLM 控制层** `deepseek_all_controller.py`（2051 行） | 本人实现：任务解析、两阶段解题、工具调用、执行编排、看板 |
| **独立解题器** `ai.py`（648 行） | 本人实现：仅依赖 API 的题目求解与结构化输出 |
| **坐标自动扫描** `scan_positions.py` | 本人实现：从 Gazebo 真值生成 `positions.py` |
| **动态障碍** `moving_obstacle.py` | 本人实现：基于 `/gazebo/set_entity_state` 的正弦往复 |
| **场景/参数调优**（world、Nav2 参数、TF 修复、物理参数） | 本人排查与修改，详见 [调试记录](docs/debugging-log.md) |
| **验证脚本与证据** `portfolio/verification/` | 本人编写，可复现 |

开发方式：采用 AI 辅助编程（AI pair programming），本人负责需求拆解、方案决策、
问题定位与验证闭环。所有结论均以可复现的脚本或日志为依据。

## 3. 技术栈

- **机器人**：ROS 2 Humble、Gazebo Classic 11、ros2_control、MoveIt 2
- **导航**：Nav2（AMCL + MPPI + costmap 三层）、robot_localization EKF
- **操作**：6 自由度机械臂 + 平移夹爪 + Gazebo LinkAttacher 吸附
- **大模型**：DeepSeek Chat（function calling）+ 可选推理模型
- **可视化**：coStudio（coScene，经 coBridge 连接，默认端口 21274）订阅 `/mission/*` 自定义话题；RViz2 辅助调试
- **语言**：Python（控制与解析）、C++（LinkAttacher 插件修补）

## 4. 可量化的结果

| 指标 | 数值 | 证据 |
|---|---|---|
| 方块两两间距 ≥ 2 m | **45 / 45 对，最小 2.454 m** | [test_geometry.txt](verification/results/test_geometry.txt) |
| 任务解析正确性（种子/题目/规则/方案） | **全部用例通过** | [test_task_pipeline.txt](verification/results/test_task_pipeline.txt) |
| 映射规则格式兼容 | 3 种公布规则 × 编号/引号/无前缀写法 | 同上 |
| 导航精度问题定位 | 目标容差 0.15 m 而车停在 0.19 m → 单次等 90 s 超时 | [调试记录 §5](docs/debugging-log.md) |
| 底盘起步加速度 | 轮加速度 1.0 → 3.0 rad/s²（线加速度 0.065 → 0.195 m/s²） | [调试记录 §6](docs/debugging-log.md) |
| LLM 调用次数 | 简单指令 **1 次**，出题 **2 次**（未为省 token 减少调用） | [llm-solver.md](docs/llm-solver.md) |

## 5. 证据索引

| 想证明什么 | 看哪里 |
|---|---|
| 系统怎么跑起来的 | [docs/architecture.md](docs/architecture.md) |
| 大模型解题与映射规则怎么设计 | [docs/llm-solver.md](docs/llm-solver.md) |
| 我怎么定位真实工程问题 | [docs/debugging-log.md](docs/debugging-log.md) |
| 简历怎么写、面试怎么答 | [docs/resume-bullets.md](docs/resume-bullets.md) |
| 代码逻辑是否真的能用 | [verification/](verification/)（可直接运行） |
| 截图/录屏该拍什么 | [media/README.md](media/README.md) |

## 6. 自己跑一遍验证（不需要 ROS）

脚本会自动读取 `src/` 下的源码副本（找不到时回退到主工程目录），
因此**只上传本文件夹也能跑通**。

```bash
cd portfolio/verification
python3 run_checks.py
```

预期输出：

```
[PASS] test_task_pipeline.py
[PASS] test_geometry.py
总体结果: 全部通过
```

这两个脚本只校验纯逻辑与场景几何，不启动 ROS/Gazebo，任何人可复现；
输出会存档到 `verification/results/`，作为仓库内的长期证据。

## 7. 已知限制（比"全部完成"更可信）

- 3 cm 方块在 **0.16 m 高**的单线激光下方，**激光看不到**，因此不会进入代价地图；
  这是几何限制，不是代码 bug，已记录取舍（[调试记录 §7](docs/debugging-log.md)）。
- 放置区底板是**纯视觉体（无碰撞）**，方块落地会陷进 2 cm；已确认并接受该表现。
- 动态障碍采用**位姿瞬移**（20 Hz 设置位姿），不是真实物理滑动。
- 大模型调用受网络影响，已实现"无法连接"快速失败（退出码 3），不做本地兜底求解。

## 8. 目录

```
portfolio/
├── README.md                 本文件（仓库首页）
├── src/                      源码副本（与主工程一致，可直接阅读/运行）
│   ├── arm_llm/              控制层（deepseek_all_controller.py、ai.py）
│   └── yzbot/                脚本、Nav2 参数、地图、场景、URDF
├── docs/
│   ├── architecture.md       系统架构与数据流
│   ├── llm-solver.md         两阶段解题与映射规则
│   ├── debugging-log.md      7 个真实工程问题与修复
│   └── resume-bullets.md     简历 bullet 与面试问答
├── verification/
│   ├── run_checks.py         一键跑校验
│   ├── test_task_pipeline.py 解析链路测试
│   ├── test_geometry.py      场景几何测试
│   └── results/              真实运行输出（长期证据）
├── media/
│   └── README.md             录屏/截图清单与命名规范
└── SECURITY.md               密钥泄露处理（重要）
```
