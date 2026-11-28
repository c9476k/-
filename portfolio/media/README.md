# 录屏与截图清单（需要你本人补拍）

文档和脚本能证明"代码逻辑对"，但**机器人真的动了**只能靠影像证明。
按下面清单拍，拍完把文件放进本目录。

## 一、必拍 5 段

| 序号 | 内容 | 建议时长 | 文件名 |
|---|---|---|---|
| 1 | 完整一单：输入题目 → 输出方案 → 车动 → 抓取 → 放置 | 60~120 s | `01_full_run.mp4` |
| 2 | 抓取特写：精停靠 → 下放机械臂 → 夹爪闭合 → 吸附 → 抬起 | 15~20 s | `02_pick.mp4` |
| 3 | 放置特写：到区域 → 下放 → 释放 → 方块落在颜色区域内 | 15~20 s | `03_place.mp4` |
| 4 | Foxglove 看板：`/mission/task` 四行 + 进度/数量 | 截图 | `04_foxglove.png` |
| 5 | 失败与恢复（可选但很加分）：导航失败 → 清代价地图重试 → 成功 | 30~60 s | `05_recovery.mp4` |

### 第 1 段的完整操作顺序

```bash
# 终端 1
ros2 launch mybot gazebo_world2.launch.py

# 终端 2
ros2 launch bot_navigation nav_bringup_gazebo2.launch.py

# 终端 3（coStudio 桥接：coBridge）
ros2 launch cobridge cobridge_launch.xml
# 默认端口 21274；coStudio 里选「coScene Websocket」，
# 地址填 ws://192.168.70.218:21274（同机可填 ws://localhost:21274）

# 终端 4（控制器）
export DEEPSEEK_API_KEY=sk-xxxx        # ← 录屏时务必打码/遮住这一行
python3 ~/demo1_ws/src/arm_llm/arm_llm/deepseek_all_controller.py
# 输入：题目
# 粘贴题干 + 映射规则，空行结束
```

录屏时**画面里同时出现**：Gazebo 3D 视图 + 控制器终端。

### 第 4 段 coStudio 面板配置

coStudio 的面板体系与 Foxglove 同源，下面按通用叫法给（若你的版本名称略有差异，按括号里的英文找）：

| 面板（英文） | 话题 | 类型 |
|---|---|---|
| 原始消息 / Raw Message | `/mission/task` | String |
| 原始消息 / Raw Message | `/mission/state` | String |
| 原始消息 / Raw Message | `/mission/current` | String |
| 原始消息 / Raw Message | `/mission/progress` | String |
| 指标 / Indicator | `/mission/count/red.data` | Int32 |
| 指标 / Indicator | `/mission/count/blue.data` | Int32 |
| 三维 / 3D | `/scan`、`/map`、`/tf`、`/plan` | 场景与路径 |

注意两点：

1. 字符串话题（`/mission/task` 等）必须用**原始消息**面板，指标面板只显示数值；
2. 指标面板要填 `/mission/count/red.data`，只填话题名会一直显示 0。

## 二、拍摄注意

1. **不要露出 API Key**：终端里 `export DEEPSEEK_API_KEY=...` 那行要裁掉或打码。
2. **IP 可打码**：`192.168.x.x` 建议遮一下。
3. **分辨率**：≥ 1280×720；Gazebo 尽量拉大窗口，别把面板挤成一团。
4. **节奏**：完整版不要剪太碎，保留"等待"的真实感（可信度更高）；
   同时另存一个 30~60 s 的剪辑版放简历/README 顶部。
5. **失败镜头也留着**：调参前后的对比、失败重试，比只放成功更有说服力。

## 三、压缩与转 GIF（可选）

```bash
# 压缩到 720p
ffmpeg -i 01_full_run.mp4 -vf scale=-2:720 -c:v libx264 -crf 26 -an 01_full_run_720p.mp4

# 转 GIF（短片段）
ffmpeg -i 03_place.mp4 -vf "fps=12,scale=640:-1" -loop 0 03_place.gif
```

GitHub 仓库里 GIF 超过 ~10 MB 会拖慢加载，建议：`README` 顶部放 GIF，完整视频放 Release 或外链。

## 四、目录结构

```
media/
├── README.md            本文件
├── 01_full_run.mp4      完整流程
├── 01_full_run_720p.mp4 压缩版（放仓库）
├── 02_pick.mp4
├── 03_place.mp4
├── 04_foxglove.png
└── 05_recovery.mp4
```

拍完后，在根 [README.md](../README.md) 第 4、5 节的证据表里补上链接，
例如 `[完整流程录屏](media/01_full_run_720p.mp4)`。
