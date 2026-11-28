# 工程调试记录（7 个真实故障）

这些不是教程案例，是项目里**实际发生并修复**的问题。每条按"现象 → 根因 → 修复 → 验证"记录，
适合面试时展开讲。

---

## 1. LinkAttacher 插件在 1 自由度关节上越界

**现象**：Gazebo 启动即报错

```
[Err] ODEJoint::SetDamping: index[1] is out of bounds (DOF() = 1)
```

**根因**：`gazebo_link_attacher.cpp` 对夹爪的平移关节调用 `joint->SetDamping(1, 1.0)`，
而 `finger_joint1` 是**单自由度平移关节**，合法索引只有 `0`。

**修复**：改为 `joint->SetDamping(0, 1.0)`，重新编译 `ros2_linkattacher` 并重启 Gazebo。

**教训**：第三方插件默认假设关节是多自由度，接自有 URDF 时要核对 DOF。

---

## 2. 吸附成功后机器人"自旋飞行"

**现象**：夹爪闭合 + 吸附成功，抬臂瞬间机器人整体乱转、被弹飞。

**根因（两处叠加）**：

1. 夹爪闭合量设为 `0.035 m`，正好等于方块 3 cm 的宽度 —— 手指**实实在在夹住了方块**，
   与 LinkAttacher 的固定约束互相较劲；
2. 方块 `inertia` 写的是 `1`（kg·m²），而方块质量只有 **5 g**，
   相当于惯性放大约 1.3×10⁶ 倍。

**修复**：

| 项 | 修复前 | 修复后 |
|---|---|---|
| `GRIPPER_CLOSE` | 0.035 | **0.020**（每侧留 7.5 mm 间隙，只做"合拢动作"不接触） |
| 方块 `inertia` | 1 | **7.5e-7** |
| 手指关节 `effort` | 默认 | 30 |
| 手指关节 `damping` | 默认 | 2.0 |

**验证**：抬臂后方块随夹爪稳定上升，机器人不再异常旋转。

---

## 3. RViz 里机器人持续抖动，且 Gazebo 朝向偏了 30°

**现象**：设置 2D Pose Estimate 后，RViz 中小车持续小幅抖动；Gazebo 里车头也不是正方向。

**根因（三重）**：

1. **TF 重复发布**：Gazebo 差速插件 `publish_odom_tf: true`，同时 `robot_localization`
   的 EKF 也 `publish_tf: true`，两者都在发 `odom → base_footprint`，
   数值略有差异 → RViz 解析到的位姿在两者间来回跳。
2. **world 里内嵌了一整个机器人**：`gazebo_world3.new` 中有 1135 行的 `six_arm` 模型，
   还带着自己的差速驱动插件 → 又多一份 `/odom` 和 TF。这是之前"保存世界"时把机器人一起存进去了。
3. **保存的位姿 yaw = -0.524 rad（-30°）**：`<state>` 里 six_arm 的位姿如此，
   所以每次加载世界车头都是歪的，而不是 spawn 参数里的 `-Y 0`。

**修复**：

- 从 world 删除内嵌 `six_arm`（模型定义 1135 行 + state 条目 64 行），文件 5456 → 4257 行；
- 关闭 Gazebo 的 TF 输出，交给 EKF 独占：

```xml
<publish_odom>true</publish_odom>         <!-- /odom 话题保留，EKF 需要 -->
<publish_odom_tf>false</publish_odom_tf>
<publish_wheel_tf>false</publish_wheel_tf>
```

**验证**：`ros2 topic info /odom` 只有 1 个发布者；TF 树中 `odom → base_footprint` 只来自 EKF；
抖动消失，车头恢复 +x。

**教训**：一条 TF 边只能有一个发布者。`map→odom` 归 AMCL，`odom→base_footprint` 归 EKF，
`base_footprint→各连杆` 归 robot_state_publisher，这个分工要守住。

---

## 4. 提速后 MPPI 频繁 `Optimizer fail to compute path`

**现象**：为提高效率把 Nav2 速度整体提到 1.5 倍后，`controller_server` 报

```
[ERROR] [controller_server]: Optimizer fail to compute path
[WARN]  [follow_path] Aborting handle.
[ERROR] [bt_navigator]: Goal failed
```

任务在 `[blue,3,A]` 处中断。

**根因**：MPPI 在采样轨迹里**找不到任何一条不碰撞的**。我同时动了几项会互相放大的参数：

| 参数 | 原值 | 我改成 | 影响 |
|---|---|---|---|
| `vx_std` | 0.3 | 0.45 | 采样更"野"，贴障碍时大量样本直接撞墙 |
| `wz_std` | 0.4 | 0.6 | 同上 |
| `ax_max/az_max` | 1.5 | 2.25 | 单步速度变化大，贴墙时更难拉回 |
| `collision_monitor.time_before_collision` | 1.2 s | 0.8 s | 允许贴得更近，进入膨胀区后全部轨迹判碰撞 |

**修复**：保留 `vx_max: 1.5` 的提速目标，但把"探索性"参数收回合理区间：

| 参数 | 修复后 |
|---|---|
| `vx_std` / `wz_std` | 0.35 / 0.45 |
| `ax_max` / `az_max` | 2.0 |
| `batch_size` | 1000 → **1200**（多采样，提高找到可行解的概率） |
| `time_before_collision` | 0.8 → **1.2** |

**教训**：提速不是只改 `vx_max`。采样方差、加速度上限、避障前瞻必须同步调整，
否则控制器会"有速度但无解"。

---

## 5. 差 4 cm 判不到达，每次空等 90 s

**现象**：日志

```
[导航] 超过 90s 未到达，取消目标
[导航] zone_a 第 1 次失败，清理代价地图后重试
[导航] 超过 90s 未到达，取消目标
[导航] zone_a Nav2 连续 2 次失败，放弃（当前位姿 (4.60,-5.77,2°)，距目标 0.19m）
[error] blue_cube_3 放置失败
```

**根因**：车已经停在离目标 **0.19 m**、朝向只差 2°，但 Nav2 的 `xy_goal_tolerance` 是
**0.15 m** —— 差 4 cm 不满足判定，于是永远不返回成功，一直等到 90 s 超时；重试再来 90 s。

**修复（两层）**：

1. 放宽 Nav2 容差到 `xy_goal_tolerance: 0.25`、`yaw_goal_tolerance: 0.20`；
2. 控制器增加"提前接管"：只要进入目标 **0.30 m** 并保持 1 s，就主动取消 Nav2 目标，
   交给 `dock_to_object()` 做厘米级精停靠；Nav2 报失败但车已在 0.30 m 内时同样接管。

**为什么可以这样**：机械臂需要的精度（0.34 ± 0.02 m）本来就靠精停靠实现，
Nav2 不需要 15 cm 精度。这套分工让"导航差几厘米"不再变成"任务失败"。

**验证**：桩测试覆盖 5 个场景（提前接管 / 远距离超时 / 失败但靠近 / 失败且远 / 正常成功），全部通过。

---

## 6. 提速的真正瓶颈：轮加速度只有 0.065 m/s²

**现象**：把 Nav2 速度上限从 1.0 提到 1.5 m/s 后，实际观感几乎没变。

**根因**：Gazebo 差速插件的

```xml
<max_wheel_acceleration>1.0</max_wheel_acceleration>
```

单位是 **rad/s²**。轮半径 0.065 m，折算线加速度只有 `1.0 × 0.065 = 0.065 m/s²`。
也就是说：即使把目标速度设到 1.5 m/s，车也要爬二十多秒才到速，Nav2 的速度上限根本用不上。

**修复**：`max_wheel_acceleration: 1.0 → 3.0`（线加速度 0.195 m/s²），
轮扭矩 20 → 25 保证加速时不打滑。

**教训**：调"车速"要区分**速度上限**（Nav2）与**加速度上限**（仿真插件/驱动）。
短距离搬运时，加速度往往才是主要瓶颈。

---

## 7. 3 cm 方块对激光"隐形"

**现象**：导航完全不避让地面上的小方块，会直接碾过去。

**根因（几何限制）**：

| 项目 | 数值 | 来源 |
|---|---|---|
| 激光高度 | **0.16 m** | `base_footprint→base_link` 0.10 + `base_link→laser` 0.06 |
| 方块高度 | **0.03 m** | world 中 `<size>0.03 0.03 0.03</size>` |
| 激光最小测距 | **0.389 m** | `range_min = robot_radius(0.339) + safety_margin(0.05)` |

激光是单线 2D 扫描，平面在 16 cm，从 3 cm 方块**上方**扫过 → 永远测不到。
另外 39 cm 内是盲区。

**决策**（不是修 bug，而是接受约束）：

- 不把这些方块画进静态地图（画了也没意义，SLAM 同样扫不到）；
- 抓取靠 `NAV_STANDOFF = 0.34 m` 停在方块前，再用机械臂够，不需要车压过去；
- 后来新增的 **0.5 m 高障碍块**能被激光看到，因此会被避让——同一套机制，差别只在高度。

**衍生结论**：如果希望机器人"绕开"，物体必须高于激光平面，或改传感器（本项目有 RGB-D，
但挂在机械臂末端，不适合做导航避障）。

---

## 8. 其他已记录的取舍

| 问题 | 结论 |
|---|---|
| 放置区底板是纯视觉体（无碰撞） | 方块落地会陷进 2 cm；选择"位置正确"优先，保持现状 |
| 动态障碍用位姿瞬移（20 Hz `set_entity_state`） | 实现简单、可控；不是真实物理滑动 |
| Gazebo 提示 `Queue limit reached for topic /gazebo/.../pose/local/info` | 20 Hz × 2 link 的位姿同步把 GUI 队列打满，仅影响显示，不影响物理/导航 |
| 大模型提示词污染导致抄答案 | 见 [llm-solver.md §2](llm-solver.md) |
