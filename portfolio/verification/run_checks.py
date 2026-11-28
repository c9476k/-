#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键跑完所有离线校验，并把输出存档到 results/。

用法：
    python3 run_checks.py

产物：
    results/<测试名>.txt   每个测试的完整输出
    results/summary.txt    汇总（时间、Python 版本、每项结果）

这些结果文件可以直接提交到仓库，作为"可复现"的证据。
"""

import datetime
import platform
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
TESTS = ["test_task_pipeline.py", "test_geometry.py"]


def main():
    RESULTS.mkdir(exist_ok=True)
    summary = []
    started = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    for name in TESTS:
        path = HERE / name
        if not path.exists():
            summary.append((name, "MISSING", ""))
            continue
        proc = subprocess.run(
            [sys.executable, str(path)],
            capture_output=True, text=True, encoding="utf-8", errors="replace")
        output = (proc.stdout or "") + (proc.stderr or "")
        (RESULTS / f"{name.replace('.py', '')}.txt").write_text(
            output, encoding="utf-8")
        status = "PASS" if proc.returncode == 0 else f"FAIL({proc.returncode})"
        summary.append((name, status, output))
        print(f"[{status}] {name}")

    lines = [
        "离线校验汇总",
        "=" * 60,
        f"时间      : {started}",
        f"Python    : {platform.python_version()}",
        f"平台      : {platform.system()} {platform.release()}",
        "",
    ]
    for name, status, _ in summary:
        lines.append(f"{status:12s} {name}")
    ok = all(s == "PASS" for _, s, _ in summary)
    lines += ["", f"总体结果: {'全部通过' if ok else '存在失败'}", ""]
    lines.append("说明：这些测试只校验纯逻辑与场景几何，不启动 ROS/Gazebo，")
    lines.append("      因此任何人都能在本地复现。")
    (RESULTS / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\n" + "\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
