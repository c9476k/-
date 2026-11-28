# 源码副本说明

这里的代码是从主工程**原样复制**过来的，目的是让本文件夹单独上传到 GitHub 后，
招聘方仍能直接阅读源码、跑通 `verification/` 里的验证脚本。

## 目录对应关系

| 本目录 | 主工程中的路径 | 说明 |
|---|---|---|
| `arm_llm/arm_llm/deepseek_all_controller.py` | 同路径 | **主控制器**（2051 行）：任务解析、两阶段解题、工具调用、导航/抓取/放置编排 |
| `arm_llm/arm_llm/ai.py` | 同路径 | **独立解题器**（648 行）：仅依赖 API，无 ROS 依赖 |
| `arm_llm/arm_llm/cube_detector.py` | 同路径 | OpenCV 方块检测（辅助） |
| `arm_llm/package.xml`、`setup.py`、`setup.cfg` | 同路径 | ROS 2 包定义与入口 |
| `yzbot/scripts/scan_positions.py` | 同路径 | 从 Gazebo 真值自动生成坐标文件 |
| `yzbot/scripts/positions.py` | 同路径 | 生成的方块/区域坐标（被控制器导入） |
| `yzbot/scripts/moving_obstacle.py` | 同路径 | 动态障碍：正弦往复（`/gazebo/set_entity_state`） |
| `yzbot/bot_navigation/param/originbot_nav2_2.yaml` | 同路径 | Nav2 参数（AMCL/MPPI/代价地图/速度平滑） |
| `yzbot/bot_navigation/maps/mapn3.yaml`、`.pgm` | 同路径 | 静态栅格地图（几何校验用） |
| `yzbot/mybot_description/worlds/gazebo_world3.new` | 同路径 | Gazebo 场景（已移除内嵌机器人） |
| `yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro` | 同路径 | 差速插件参数（轮加速度、TF 开关） |

## 归属说明

- `arm_llm/arm_llm/` 下的控制层与解题器、`yzbot/scripts/` 下三个脚本、Nav2 参数调优、
  场景与 URDF 的修改：**本人实现**。
- 底盘/机械臂的完整 URDF、MoveIt 与 Nav2 包结构、LinkAttacher 插件：来自赛事提供的
  基础工程（`yzbot`），本目录只保留**被修改或与结论直接相关**的文件。

## 这不是完整工作区

这里**不能直接 `colcon build`**：只保留了与作品证据相关的文件，
缺少 `mybot`、`hello_moveit`、`IFRA_LinkAttacher` 等完整包与其余 URDF/xacro 依赖。
完整工程结构见主仓库；本目录的定位是"可读、可验证"，不是"可编译"。

## 一致性

本目录代码与主工程一致（复制时未做任何修改）。
`verification/` 下的测试默认读取本目录，因此结论可与源码一一对应。
