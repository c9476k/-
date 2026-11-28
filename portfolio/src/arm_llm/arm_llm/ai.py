#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机题目求解器（仅调用大模型 API 解题，不含 ROS / 导航 / 机械臂）。

解题完全依赖 DeepSeek API，分两阶段：
    阶段一  让大模型逐步列算式解出 x、y（可启用推理模型深度思考）
    阶段二  把已确定的 x、y 连同本次映射规则交给大模型，翻译成
            [color,count,zone;...] 供程序解析

【不做任何本地兜底求解】：API 不可用（未装 requests / 未设 Key / 网络不通 /
超时 / HTTP 报错）时直接报错「无法连接」并以退出码 3 结束，避免用本地猜测
掩盖接口故障——你要验证的就是大模型本身能否解题。

已覆盖你公布的三种映射规则及其措辞变体：
    规则1  x代表红色,y代表蓝色; 数值大于等于3的去A区, 其他颜色去C区
    规则2  x/y中数值大于等于3的代表红色,否则代表蓝色; x去B区, y去A区
    规则3  蓝色去B区,红色去C区; x/y中数值大的是蓝色,数值少的是红色

规则写法兼容：带"映射规则:"前缀 / 单独一行 / 编号列表(1. 2、 (1)) /
引号包裹(" " “ ” 『 』) / 紧跟题干句号后；全角半角标点均可。

运行：
    export DEEPSEEK_API_KEY=sk-xxxx                 # 必需，否则报"无法连接"
    export DEEPSEEK_REASON_MODEL=deepseek-reasoner  # 可选：阶段一用推理模型
    python3 ai.py --selftest          # 用 API 跑 11 条内置用例，验证解题准确率
    python3 ai.py "题干... 映射规则..."
    python3 ai.py                     # 交互粘贴，空行结束
    python3 ai.py --quiet "..."       # 只输出方案行，便于脚本取值
    python3 ai.py --json "..."        # JSON 输出（含 x/y、推理过程、方案）
    python3 ai.py --show-reasoning    # 打印大模型的完整演算过程

退出码：0=成功  2=未能解析出方案  3=无法连接 API

作为模块复用（例如接回搬运控制器）：
    from ai import solve_problem
    entries, info = solve_problem(text)
    if info.get("error"): ...
"""

import argparse
import json
import os
import re
import sys
from typing import List, Optional, Sequence, Tuple

try:
    import requests
    HAVE_REQUESTS = True
except Exception:                       # 没装 requests 就是"无法连接"
    HAVE_REQUESTS = False


# ============================================================================
# 1. 大模型配置
# ============================================================================
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")
DEEPSEEK_URL = os.environ.get("DEEPSEEK_URL", "https://api.deepseek.com/v1/chat/completions")
MODEL_NAME = os.environ.get("DEEPSEEK_MODEL", "deepseek-chat")
# 阶段一解应用题需要更强的推理时可指定，例如 deepseek-reasoner
# 该模型会额外返回 reasoning_content（思考过程），更慢但更会算
REASON_MODEL_NAME = os.environ.get("DEEPSEEK_REASON_MODEL", "")
LLM_TIMEOUT = float(os.environ.get("LLM_TIMEOUT_TASK", "60"))
LLM_MAX_TOKENS = int(os.environ.get("LLM_MAX_TOKENS_TASK", "128"))
# 解 x/y 要列算式，给足输出空间
LLM_MAX_TOKENS_XY = int(os.environ.get("LLM_MAX_TOKENS_XY", "1500"))

# 出题程序保证 x>=1、y>=1、x+y=5，用它校验大模型有没有算错
EXPECTED_TOTAL = 5

SUPPORTED_COLORS = ("red", "blue")
COLOR_LABELS = {"red": "红色", "blue": "蓝色"}
ZONE_LETTERS = {"zone_a": "A", "zone_b": "B", "zone_c": "C"}

# 无法连接时的统一错误前缀
CONN_ERROR = "无法连接"


# ============================================================================
# 2. 提示词（刻意不给"某规则→某答案"的示例，避免模型照抄而不真算）
# ============================================================================
XY_SYSTEM_PROMPT = (
    "你是应用题求解器。请真正逐步计算，禁止猜测、禁止套用任何记忆中的答案。\n"
    "解题步骤（必须逐步执行）：\n"
    "1. 找出题目中两种物品的名称，以及『每个容器有 N 个/件…』里的 N。\n"
    "2. 逐句累加每个人对每种物品的需求量，得到两个总量 totalX、totalY。\n"
    "   特别注意：『甲、乙都需要 n 个 X』表示这两个人各 n 个，必须乘以人数；\n"
    "   『设…数量为x』『有A、B两种…』这类定义句不是需求，不要计入。\n"
    "3. x = ceil(totalX / N)，y = ceil(totalY / N)（向上取整，且至少为 1）。\n"
    f"4. 用题目保证的 x>=1、y>=1、x+y={EXPECTED_TOTAL} 验算；"
    "若不满足，回到第 2 步重新检查累加。\n"
    "输出：先写清每一项的累加与取整过程，最后一行严格写成：\n"
    "x=<整数>,y=<整数>\n"
)

MAP_SYSTEM_PROMPT = (
    "你是规则套用器。用户会给你已经算好的 x、y 数值，以及本次的映射规则。\n"
    "你只需严格按【本次规则】把数值翻译成搬运方案，不要重新解题、不要改动 x/y。\n"
    "规则可能规定：颜色由 x/y 直接指定、或由数值阈值决定、或由两数大小排名决定；\n"
    "区域可能按颜色指定、按 x/y 指定、或按数值阈值指定。请逐条对照本次规则判断，\n"
    "不要假设 x 一定是红色，也不要假设任何固定的区域。\n"
    "输出要求：只输出一行，不要解释、不要 Markdown、不要代码块：\n"
    "[color,count,zone;...]  其中 color 只能是 red 或 blue，zone 只能是 A/B/C，\n"
    "count 是该颜色的数量（就是 x 或 y 的值），分号分隔，每种颜色最多出现一次。\n"
    "若本次规则没有给出颜色或区域映射，输出：[ERROR] 缺少映射\n"
)


# ============================================================================
# 3. 解析与格式化工具
# ============================================================================
PLAN_ITEM_RE = re.compile(
    r"(red|blue|红|蓝)\s*[,:;\s]+\s*(\d+)\s*[,:;\s]+\s*([abcABC])(?![a-zA-Z])",
    re.IGNORECASE)

XY_ANSWER_RE = re.compile(
    r"x\s*[=＝:：]\s*(\d+)[^\d\n]{0,20}?y\s*[=＝:：]\s*(\d+)", re.IGNORECASE)

CN_DIGITS = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def parse_int(value) -> Optional[int]:
    """把 "3" / 3 / "三" / "两" 统一成整数。"""
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
    if text in ("a", "b", "c"):
        return "zone_" + text
    match = re.search(r"zone[_ ]?([abc])|([abc])\s*区|(?:区域|区)\s*([abc])", text)
    if match:
        return "zone_" + (match.group(1) or match.group(2) or match.group(3))
    return None


def zone_letter(zone) -> str:
    return ZONE_LETTERS.get(str(zone), "?")


def order_entries(entries: Sequence[dict]) -> List[dict]:
    """统一输出顺序：红在前、蓝在后，便于和标准答案逐字对比。"""
    return sorted(entries, key=lambda e: 0 if str(e.get("color")) == "red" else 1)


def format_plan(entries: Sequence[dict]) -> str:
    """[{"color":"red","count":4,"zone":"zone_a"}, ...] -> "[red,4,A;blue,1,C]" """
    if not entries:
        return ""
    return "[" + ";".join(
        f"{e.get('color')},{int(e.get('count', 0))},{zone_letter(e.get('zone'))}"
        for e in order_entries(entries)) + "]"


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


def parse_plan_text(text: str) -> List[dict]:
    """从大模型回复里抠出 [red,4,A;blue,1,C]。

    模型带思考过程时推理里可能出现别的方括号，所以优先取"答案:"标记之后、
    或最后一个能解析出条目的 [...] 片段（最终答案通常在末尾）。
    """
    if not text:
        return []
    normalized = (str(text).replace("；", ";").replace("，", ",")
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


def clean_problem_text(problem: str) -> str:
    """去掉开头的"题目:"/"题干:"等前缀，避免报告里前缀重复。"""
    return re.sub(r"^\s*(题目|题干|题|task|problem|出题)\s*[:：]\s*", "",
                  str(problem), flags=re.IGNORECASE).strip()


# ---- 从输入里分离出映射规则（兼容编号/引号/换行/无标记等写法）----
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
    """一个片段是否像映射规则：必须同时提到颜色和（区域或映射动词）。"""
    if not RULE_COLOR_RE.search(segment):
        return False
    return bool(RULE_ZONE_RE.search(segment) or RULE_MAP_RE.search(segment))


def extract_mapping_rule(problem: str) -> Tuple[str, str]:
    """从输入中分离出映射规则，返回 (题干, 规则)。规则找不到时返回空串。"""
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


# ============================================================================
# 4. API 连接
# ============================================================================
def check_connection() -> str:
    """检查是否具备调用条件。可用返回空串，否则返回"无法连接"的具体原因。"""
    if not HAVE_REQUESTS:
        return f"{CONN_ERROR}：未安装 requests（pip3 install requests）"
    if not DEEPSEEK_API_KEY:
        return f"{CONN_ERROR}：未设置环境变量 DEEPSEEK_API_KEY"
    return ""


def _llm_call(messages: List[dict], max_tokens: int, model: str = "") -> Tuple[str, str, str]:
    """单次对话调用，返回 (content, reasoning_content, 错误)。

    错误统一以"无法连接"开头，便于上层直接展示。
    """
    problem = check_connection()
    if problem:
        return "", "", problem
    payload = {
        "model": model or MODEL_NAME,
        "messages": messages,
        "temperature": 0.0,
        "max_tokens": max_tokens,
    }
    headers = {"Authorization": f"Bearer {DEEPSEEK_API_KEY}",
               "Content-Type": "application/json"}
    try:
        response = requests.post(DEEPSEEK_URL, headers=headers, json=payload, timeout=LLM_TIMEOUT)
        response.raise_for_status()
        message = response.json()["choices"][0]["message"]
        return ((message.get("content") or "").strip(),
                (message.get("reasoning_content") or "").strip(), "")
    except requests.exceptions.Timeout:
        return "", "", f"{CONN_ERROR}：请求超时（{LLM_TIMEOUT:.0f}s，可调 LLM_TIMEOUT_TASK）"
    except requests.exceptions.ConnectionError as exc:
        return "", "", f"{CONN_ERROR}：网络不可达（{exc.__class__.__name__}）"
    except requests.exceptions.HTTPError as exc:
        return "", "", f"{CONN_ERROR}：接口返回错误（{exc}）"
    except Exception as exc:
        return "", "", f"{CONN_ERROR}：{exc.__class__.__name__}: {exc}"


def _parse_xy(text: str) -> Optional[Tuple[int, int]]:
    """从回复里取最后一处 x=<数>,y=<数>。"""
    matches = XY_ANSWER_RE.findall(text or "")
    if not matches:
        return None
    return int(matches[-1][0]), int(matches[-1][1])


# ============================================================================
# 5. 两阶段解题
# ============================================================================
def solve_xy_with_llm(problem: str) -> Tuple[Optional[int], Optional[int], str, str]:
    """阶段一：让大模型真正列算式解出 x、y。

    返回 (x, y, 演算文本, 错误)。若首次结果不满足 x+y=5，会带着约束追问一次。
    """
    model = REASON_MODEL_NAME or MODEL_NAME
    messages = [
        {"role": "system", "content": XY_SYSTEM_PROMPT},
        {"role": "user", "content": clean_problem_text(problem)},
    ]
    content, reasoning, error = _llm_call(messages, LLM_MAX_TOKENS_XY, model)
    if error:
        return None, None, "", error
    text = content or reasoning
    pair = _parse_xy(text)
    if pair is None:
        return None, None, text, f"{CONN_ERROR}：大模型回复中找不到 x=<数>,y=<数>"

    if pair[0] + pair[1] != EXPECTED_TOTAL:
        # 不满足题目约束，带着约束让它重算一次
        messages.append({"role": "assistant", "content": content})
        messages.append({"role": "user", "content": (
            f"你给出 x={pair[0]}, y={pair[1]}，但 x+y={pair[0] + pair[1]}，"
            f"而题目保证 x+y={EXPECTED_TOTAL} 且 x>=1、y>=1。"
            "请重新逐句累加每种物品的总需求量（注意『甲、乙都需要 n 个』要乘以人数），"
            f"重新按 ceil(总量/N) 计算，最后只输出一行：x=<整数>,y=<整数>")})
        content2, reasoning2, error2 = _llm_call(messages, LLM_MAX_TOKENS_XY, model)
        if not error2:
            text2 = content2 or reasoning2
            pair2 = _parse_xy(text2)
            if pair2 is not None:
                text = f"{text}\n--- 追问修正 ---\n{text2}"
                pair = pair2
        else:
            text = f"{text}\n--- 追问失败: {error2} ---"

    return pair[0], pair[1], text, ""


def apply_mapping_with_llm(x: int, y: int, rule: str, problem: str) -> Tuple[List[dict], str, str]:
    """阶段二：把已确定的 x、y 交给大模型套用本次映射规则。

    返回 (方案, 原始回复, 错误)。规则没分离出来时退化为把整题给它自行判断。
    """
    context = rule if rule else clean_problem_text(problem)
    user = (f"已知：x={x}, y={y}\n"
            f"本次映射规则：{context}\n"
            "请严格按本次规则输出方案，只输出一行。")
    content, _reasoning, error = _llm_call(
        [{"role": "system", "content": MAP_SYSTEM_PROMPT},
         {"role": "user", "content": user}], LLM_MAX_TOKENS)
    if error:
        return [], "", error
    entries = parse_plan_text(content)
    if not entries:
        return [], content, "大模型未给出可解析的方案（回复里没有 [color,count,zone] 形式）"
    return entries, content, ""


def validate_plan(entries: Sequence[dict], x: int, y: int) -> List[str]:
    """一致性校验（不重新解题）：方案里的数量应当正好是 x 和 y。"""
    warnings = []
    counts = sorted(int(e["count"]) for e in entries)
    if counts != sorted([x, y]):
        warnings.append(
            f"方案数量 {counts} 与解得的 x={x},y={y} 不一致，请人工核对")
    if len(entries) != 2:
        warnings.append(f"方案含 {len(entries)} 项，预期 2 项（红、蓝各一）")
    if len({e["zone"] for e in entries}) != len(entries):
        warnings.append("两种颜色被分到了同一个区域，请核对映射规则")
    return warnings


def solve_problem(problem: str) -> Tuple[List[dict], dict]:
    """唯一解题入口：仅通过大模型 API 完成。

    返回 (方案, 诊断信息)。info["error"] 非空即失败；
    API 不可用时 error 以"无法连接"开头。
    """
    body, rule = extract_mapping_rule(problem)
    info: dict = {"body": body, "rule": rule, "x": None, "y": None,
                  "reasoning": "", "llm_raw": "", "warnings": [], "error": ""}

    conn = check_connection()
    if conn:
        info["error"] = conn
        return [], info

    if not rule:
        info["warnings"].append("未从输入中分离出映射规则，将把整题交给大模型自行判断")

    # 阶段一：解 x、y
    x, y, reasoning, error = solve_xy_with_llm(problem)
    info["reasoning"] = reasoning
    if error:
        info["error"] = error
        return [], info
    info["x"], info["y"] = x, y
    if x + y != EXPECTED_TOTAL:
        info["warnings"].append(
            f"x+y={x + y}，不等于题目保证的 {EXPECTED_TOTAL}，大模型可能算错")

    # 阶段二：套映射规则
    entries, raw, error = apply_mapping_with_llm(x, y, rule, problem)
    info["llm_raw"] = raw
    if error:
        info["error"] = error
        return [], info

    info["warnings"].extend(validate_plan(entries, x, y))
    info["plan"] = format_plan(entries)
    return entries, info


# ============================================================================
# 6. 自检用例（全部走真实 API）
# ============================================================================
SELFTEST_CASES = [
    # (题目, 期望方案, 说明)
    (
        "题干: 每个衣柜有5件衣物；有T恤、裤子两种衣柜；小美需要2件T恤，1条裤子；"
        "小帅、小丽都需要7件T恤；小娜需要2 件T恤，1条裤子；小杰需要1条裤子。"
        "设T恤衣柜数量为x，裤子衣柜数量为y，计算x、y分别为多少。"
        "1. \"x 代表红色,y 代表蓝色;数值大于等于 3 的去 A 区,其他颜色去 C 区\"",
        "[red,4,A;blue,1,C]",
        "规则1（编号+引号、无映射规则前缀、全角标点）",
    ),
    (
        "题目: 每个文具盒有5个文具;有铅笔、橡皮两种文具盒;小明领取2个铅笔,5个橡皮;"
        "小红、小兰都领取2个铅笔;小刚领取3个铅笔,5个橡皮;小芳领取4个橡皮。"
        "设铅笔文具盒数量为x,橡皮文具盒数量为y,计算x、y分别为多少。"
        "映射规则: x代表红色,y代表蓝色,数量大于等于3的颜色去A区，其他颜色去C区",
        "[red,2,C;blue,3,A]",
        "规则1（x=2 y=3）",
    ),
    (
        "题目: 每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
        "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
        "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
        "映射规则: x代表红色,y代表蓝色,数值大于等于3的去A区,其他颜色去C区",
        "[red,4,A;blue,1,C]",
        "规则1（x=4 y=1）",
    ),
    (
        "题目: 每个书架有5本书;有小说、漫画两种书架;小明借走3本小说,5本漫画;"
        "小红、小兰都借走4本小说;小刚借走2本漫画;小芳借走3本漫画。"
        "设小说书架数量为x,漫画书架数量为y,计算x、y分别为多少。"
        "映射规则: x y 中,数值大于等于 3 值的代表红色，否则代表蓝色；x 去 B 区,y 去 A 区",
        "[red,3,B;blue,2,A]",
        "规则2（x=3 红色去B，y=2 蓝色去A）",
    ),
    (
        "题目: 每个书架有5本书;有小说、漫画两种书架;小明借走3本小说,5本漫画;"
        "小红、小兰都借走2本小说;小刚借走2本小说,4本漫画;小芳借走3本漫画。"
        "设小说书架数量为x,漫画书架数量为y,计算x、y分别为多少。"
        "映射规则: x y 中,数值大于等于 3 值的代表红色，否则代表蓝色；x 去 B 区,y 去 A 区",
        "[red,3,A;blue,2,B]",
        "规则2（y=3 红色去A，x=2 蓝色去B）",
    ),
    (
        "题目: 每个文具盒有5个文具;有铅笔、橡皮两种文具盒;小明领取2个铅笔,5个橡皮;"
        "小红、小兰都领取2个铅笔;小刚领取3个铅笔,5个橡皮;小芳领取4个橡皮。"
        "设铅笔文具盒数量为x,橡皮文具盒数量为y,计算x、y分别为多少。"
        "映射规则: 蓝色去 B 区,红色去 C 区; x y 中,数值大的是蓝色，数值少的是红色",
        "[red,2,C;blue,3,B]",
        "规则3（y=3 大→蓝色去B，x=2 少→红色去C）",
    ),
    (
        "题目: 每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
        "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
        "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
        "映射规则: 蓝色去 B 区,红色去 C 区; x y 中,数值大的是蓝色，数值少的是红色",
        "[red,1,C;blue,4,B]",
        "规则3（x=4 大→蓝色去B，y=1 少→红色去C）",
    ),
    (
        "题目: 每个工具箱有5个工具;有螺丝刀、螺母两种工具箱;小赵拿走4个螺丝刀,2个螺母;"
        "小钱、小孙都拿走3个螺丝刀;小李拿走3个螺丝刀,4个螺母;小王拿走3个螺母。"
        "设螺丝刀工具箱数量为x,螺母工具箱数量为y,计算x、y分别为多少。"
        "映射规则: x对应红色,y对应蓝色,数值大于等于3的去A区,其他颜色去C区",
        "[red,3,A;blue,2,C]",
        "规则1 变体（工具箱/拿走/对应）",
    ),
    (
        "题目: 每个收纳盒有5个零件;有齿轮、弹簧两种收纳盒;小张需要6个齿轮,2个弹簧;"
        "小刘、小陈都需要4个齿轮;小杨需要3个弹簧;小周需要4个弹簧。"
        "设齿轮收纳盒数量为x,弹簧收纳盒数量为y,计算x、y分别为多少。"
        "映射规则: x y 中,数值不少于3的代表红色，否则代表蓝色;x去A区,y去C区",
        "[red,3,A;blue,2,C]",
        "规则2 变体（不少于3/x→A y→C）",
    ),
    (
        "题目: 每个袋子里有5个水果;有苹果、橘子两种袋子;小吴买了3个苹果,5个橘子;"
        "小郑、小王都买了2个苹果;小冯买了4个橘子;小褚买了6个橘子。"
        "设苹果袋子数量为x,橘子袋子数量为y,计算x、y分别为多少。"
        "映射规则: 蓝色去A区,红色去B区; x y 中,数值较多的是蓝色，数值较少的是红色",
        "[red,2,B;blue,3,A]",
        "规则3 变体（较多的/较少的）",
    ),
    (
        "题目: 每个药箱有5盒药;有感冒药、胃药两种药箱;A组领取4盒感冒药,3盒胃药;"
        "B组、C组都领取3盒感冒药;D组领取2盒胃药;E组领取4盒胃药。"
        "设感冒药药箱数量为x,胃药药箱数量为y,计算x、y分别为多少。"
        "映射规则: 蓝色去 B 区,红色去 C 区; x y 中,数值大的是蓝色，数值少的是红色",
        "[red,2,C;blue,3,B]",
        "规则3 变体（非『小X』人名/药箱题型）",
    ),
]


def run_selftest(show_reasoning: bool = False) -> int:
    conn = check_connection()
    if conn:
        print(f"错误: {conn}")
        return 3
    print("=" * 74)
    print(f"大模型解题自检（模型: {REASON_MODEL_NAME or MODEL_NAME}，共 {len(SELFTEST_CASES)} 条）")
    print("=" * 74)
    failures = 0
    for index, (problem, expected, label) in enumerate(SELFTEST_CASES, 1):
        entries, info = solve_problem(problem)
        actual = format_plan(entries)
        ok = actual == expected
        failures += 0 if ok else 1
        print(f"\n{'✓' if ok else '✗'} [{index}/{len(SELFTEST_CASES)}] {label}")
        print(f"    x={info.get('x')} y={info.get('y')}")
        print(f"    期望 {expected} / 实得 {actual or '(空)'}")
        if info.get("error"):
            print(f"    错误 {info['error']}")
            if str(info["error"]).startswith(CONN_ERROR):
                print("\nAPI 不可用，剩余用例跳过")
                return 3
        for warning in info.get("warnings", []):
            print(f"    警告 {warning}")
        if show_reasoning and info.get("reasoning"):
            print("    --- 大模型演算 ---")
            for line in str(info["reasoning"]).splitlines():
                print(f"    {line}")
    print("\n" + "=" * 74)
    print(f"结果: {len(SELFTEST_CASES) - failures}/{len(SELFTEST_CASES)} 通过")
    print("=" * 74)
    return 0 if failures == 0 else 1


# ============================================================================
# 7. 命令行
# ============================================================================
def print_report(problem: str, entries: List[dict], info: dict,
                 quiet: bool = False, show_reasoning: bool = False):
    plan = format_plan(entries)
    if quiet:
        print(plan if plan else f"[ERROR] {info.get('error', '解析失败')}")
        return
    print("-" * 74)
    if info.get("body"):
        print(f"题干: {info['body']}")
    if info.get("rule"):
        print(f"映射规则: {info['rule']}")
    if info.get("x") is not None:
        print(f"大模型求解: x={info['x']} y={info['y']} (x+y={info['x'] + info['y']})")
    if show_reasoning and info.get("reasoning"):
        print("--- 大模型演算过程 ---")
        print(info["reasoning"])
        print("-" * 74)
    if info.get("llm_raw") and show_reasoning:
        print(f"大模型映射回复: {info['llm_raw']}")
    for warning in info.get("warnings", []):
        print(f"警告: {warning}")
    if info.get("error"):
        print(f"错误: {info['error']}")
    print(f"方案: {plan if plan else '[ERROR] ' + str(info.get('error', '解析失败'))}")
    print("-" * 74)


def read_multiline() -> str:
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
    return " ".join(lines).strip()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="随机题目求解器（仅调用大模型 API）：题目+映射规则 → [red,4,A;blue,1,C]",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=('示例:\n'
                '  export DEEPSEEK_API_KEY=sk-xxxx\n'
                '  python3 ai.py --selftest\n'
                '  python3 ai.py "题目: ... 映射规则: ..."\n'
                '退出码: 0=成功  2=未能解析  3=无法连接\n'))
    parser.add_argument("problem", nargs="*", help="题目文本（不填则进入交互粘贴）")
    parser.add_argument("--selftest", action="store_true", help="用 API 跑内置用例自检")
    parser.add_argument("--quiet", action="store_true", help="只输出方案行")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果")
    parser.add_argument("--show-reasoning", action="store_true", help="打印大模型演算过程")
    args = parser.parse_args()

    if args.selftest:
        return run_selftest(show_reasoning=args.show_reasoning)

    conn = check_connection()
    if conn:
        print(f"错误: {conn}")
        return 3

    problem = " ".join(args.problem).strip() or read_multiline()
    if not problem:
        print("没有输入题目")
        return 1

    entries, info = solve_problem(problem)

    if args.json:
        print(json.dumps({
            "plan": format_plan(entries),
            "entries": entries,
            "x": info.get("x"),
            "y": info.get("y"),
            "rule": info.get("rule", ""),
            "reasoning": info.get("reasoning", "") if args.show_reasoning else "",
            "llm_raw": info.get("llm_raw", ""),
            "warnings": info.get("warnings", []),
            "error": info.get("error", ""),
        }, ensure_ascii=False, indent=2))
    else:
        print_report(problem, entries, info, quiet=args.quiet,
                     show_reasoning=args.show_reasoning)

    if not entries:
        return 3 if str(info.get("error", "")).startswith(CONN_ERROR) else 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
