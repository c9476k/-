# 系统架构与数据流

## 1. 分层

```
┌──────────────────────────────────────────────────────────────┐
│  交互层       终端输入 / coStudio 看板（coBridge）              │
│               /mission/{task,state,current,progress,...}      │
├──────────────────────────────────────────────────────────────┤
│  决策层       deepseek_all_controller.py（2051 行）            │
│               · 指令识别（任务题 vs 简单指令）                  │
│               · 两阶段 LLM：解 x/y → 套映射规则                │
│               · 方案解析 [red,4,A;blue,1,C]                    │
│               · 执行编排（抓 → 运 → 放，逐个方块）              │
├──────────────────────────────────────────────────────────────┤
│  技能层       navigate / dock / pick / place / deliver_many    │
│               IK 求解、吸附校验、失败重试与安全收臂              │
├──────────────────────────────────────────────────────────────┤
│  中间件       Nav2（AMCL+MPPI+costmap）   robot_localization   │
│               ros2_control   LinkAttacher   Gazebo 插件        │
├──────────────────────────────────────────────────────────────┤
│  仿真层       Gazebo Classic 11（gazebo_world3.new）           │
│               差速底盘 0.48×0.48m、6 轴臂、2D 激光 0.16m       │
└──────────────────────────────────────────────────────────────┘
```

## 2. 一个任务的完整时序

```
用户输入 ──► is_task_problem() 判定
                │
        ┌───────┴────────┐
        │任务题           │简单指令
        ▼                ▼
  阶段一 LLM          LLM function calling
  解出 x,y           deliver_cube(color,id,zone)
        │                │
        ▼                │
  阶段二 LLM             │
  套映射规则             │
        │                │
        ▼                ▼
  [red,4,A;blue,1,C] ──► execute_plan()
                             │
                             ▼
                    ┌── 对每个颜色：deliver_many ──┐
                    │  对每个方块：deliver(cube,zone) │
                    │      pick()                    │
                    │        navigate_near()         │
                    │        dock_to_object()        │
                    │        plan_ik() → move_arm()  │
                    │        attach_link("attach")   │
                    │      place()                   │
                    │        navigate_near()         │
                    │        dock_to_object()        │
                    │        move_arm() → detach     │
                    └────────────────────────────────┘
                             │
                             ▼
                    看板刷新 + 日志 + 下一块
```

## 3. 关键接口

| 类型 | 名称 | 消息/服务 |
|---|---|---|
| Action | `/navigate_to_pose` | `nav2_msgs/NavigateToPose` |
| Action | `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |
| Action | `/gripper_controller/follow_joint_trajectory` | 同上 |
| Service | `/ATTACHLINK`、`/DETACHLINK` | `linkattacher_msgs/AttachLink`、`DetachLink` |
| Topic | `/gazebo/model_states` | `gazebo_msgs/ModelStates`（真值来源） |
| Topic | `/scan`、`/odom`、`/joint_states`、`/tf` | 标准 |
| Topic | `/cmd_vel`、`/cmd_vel_smoothed` | `geometry_msgs/Twist`（精停靠直发） |

自定义看板话题（`std_msgs/String`，数量为 `Int32`）：

| 话题 | 内容 |
|---|---|
| `/mission/task` | 种子 / 题目 / 映射规则 / 方案（四行） |
| `/mission/state` | 待命 / 导航中 / 抓取中 / 放置中 / 任务完成 |
| `/mission/current` | 当前任务，如"红方块1 → A区" |
| `/mission/progress` | 第 2/5 个 · 剩余 4.3m |
| `/mission/count/red`、`/mission/count/blue` | 数量（`Int32`） |
| `/mission/zone/red`、`/mission/zone/blue` | 去向 |
| `/mission/status` | 以上信息的 JSON 合集（调试） |

## 4. 三个关键设计决策

### 4.1 为什么用"真值 + 直发 cmd_vel"做精停靠

Nav2 的 `xy_goal_tolerance` 默认 0.25 m，即使收到 0.15 m，也远大于机械臂需要的精度
（臂展窗口 0.27~0.45 m，目标 0.34 m）。因此导航只负责"送大概位置"，
最后几十厘米由 `dock_to_object()` 用 Gazebo 真值 + `/cmd_vel` 闭环收敛到 `0.34 ± 0.02 m`。

这条分工后来救了一次：车停在离目标 **0.19 m** 处不满足 Nav2 的 0.15 m 容差，
导致每次导航空等 90 s 超时。修复方式是"进入 0.30 m 即提前接管交给精停靠"，
详见 [调试记录 §5](debugging-log.md)。

### 4.2 为什么坐标要自动扫描

方块会被机器人撞飞、被人手动挪动，写死坐标必然失效。`scan_positions.py` 从
`/gazebo/model_states` 读取真值，自动生成 `positions.py`，让"场景改了就重扫"成为标准动作。
实测中一次场景调整就让方块坐标偏离 1.5 m、区域坐标偏离 6 m，手写坐标根本无法维护。

### 4.3 为什么保留 `ai.py` 独立解题器

控制器的执行依赖 ROS 环境；而"这道题解得对不对"应该能脱离 ROS 单独验证。
`ai.py` 是纯 Python、无 ROS 依赖的解题器，用于快速回归题目解析与映射逻辑。

## 5. 主要参数

| 参数 | 值 | 含义 |
|---|---|---|
| `NAV_STANDOFF` | 0.34 m | 车停在目标正前方多远 |
| `DOCK_TOL` | 0.02 m | 精停靠距离容差 |
| `DOCK_YAW_TOL` | 0.08 rad | 精停靠朝向容差 |
| `MIN_REACH` / `MAX_REACH` | 0.27 / 0.45 m | 允许 IK 求解的臂展窗口 |
| `PICK_GRASP_Z` | 0.05 m | 抓取高度（夹爪中心离地） |
| `GRIPPER_CLOSE` | 0.020 m | 夹爪闭合量（留 7.5 mm/侧安全间隙） |
| `NAV_ARRIVE_EPS` | 0.30 m | 提前接管半径 |
| `SETTLE_TIME` | 0.8 s | 导航结束等待 |
| `DROP_OFFSET` | (0, 0) | 放置点相对区域中心偏移（居中） |

参数与代码的对应关系见 `arm_llm/arm_llm/deepseek_all_controller.py` 顶部"配置区"。
