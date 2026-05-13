#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "docs/combined_fault_policy_arbiter_design.html"


def load_svg_renderer():
    spec = importlib.util.spec_from_file_location(
        "render_mermaid_svg", ROOT / "scripts/render_mermaid_svg.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def find_div_end(text: str, start: int) -> int:
    pos = start
    depth = 0
    while True:
        next_open = text.find("<div", pos)
        next_close = text.find("</div>", pos)
        if next_close == -1:
            raise ValueError("unclosed div")
        if next_open != -1 and next_open < next_close:
            depth += 1
            pos = next_open + 4
            continue
        depth -= 1
        pos = next_close + len("</div>")
        if depth == 0:
            return pos


def replace_div_containing(text: str, needle: str, replacement: str) -> str:
    start = text.find(needle)
    if start == -1:
        raise ValueError(f"missing block: {needle}")
    div_start = text.rfind("<div", 0, start)
    if div_start == -1:
        raise ValueError(f"missing div start for: {needle}")
    div_end = find_div_end(text, div_start)
    return text[:div_start] + replacement + text[div_end:]


def replace_section_diagram(text: str, section_id: str, replacement: str) -> str:
    section_start = text.find(f'<section id="{section_id}"')
    if section_start == -1:
        raise ValueError(f"missing section: {section_id}")
    section_end = text.find("<section", section_start + 1)
    if section_end == -1:
        section_end = text.find("</main>", section_start)
    if section_end == -1:
        raise ValueError(f"missing section end: {section_id}")

    for needle in ('<div class="svg-diagram"', '<div class="diagram"', '<div class="timeline"'):
        div_start = text.find(needle, section_start, section_end)
        if div_start != -1:
            div_end = find_div_end(text, div_start)
            return text[:div_start] + replacement + text[div_end:]
    raise ValueError(f"missing diagram block in section: {section_id}")


def main() -> None:
    renderer = load_svg_renderer()
    text = TARGET.read_text(encoding="utf-8")

    diagrams = {
        "position": """
flowchart LR
  Detect["事件发现器\n只发布事实 active / inactive"] --> Facts["事件事实集\nactive_events / EventFact"]
  Facts --> Arbiter["事件仲裁器\n法典规则 + 执行意图仲裁"]
  Arbiter --> Report["触犯法典信息\nrule hit / reason / severity"]
  Arbiter --> Plan["执行计划\nintent / target / priority"]
  Report --> Reporter["上报器\n法典触发 + 执行结果 + 人工接管"]
  Plan --> Executor["事件执行层\n软端 nodemanager\n硬件 safety"]
""",
        "dataflow": """
flowchart LR
  subgraph Source["输入事实"]
    A["事件 A\n导航 topic 低频"]
    B["事件 B\n碰撞区触发"]
    C["事件 C\n底盘反馈异常"]
  end
  subgraph Rules["法典规则"]
    All["when_all\n全部条件必须命中"]
    Any["when_any\n至少一个条件命中"]
    Count["min_match_count\n命中数量阈值"]
  end
  subgraph Arbiter["事件仲裁器"]
    Match["match_result\n规则匹配结果"]
    State["runtime_state\n进入 / 保持 / 恢复"]
    Winner["winner_rules\n交叉策略赢家"]
    Plan["execution_plan\n统一执行计划"]
  end
  subgraph Output["输出"]
    Report["report_payload\n法典触发信息"]
    Execute["event_executor_cmd\n软硬执行通道"]
  end
  A --> Match
  B --> Match
  C --> Match
  All --> Match
  Any --> Match
  Count --> Match
  Match --> State --> Winner --> Plan
  Plan --> Execute
  Winner --> Report
  A --> Report
  B --> Report
  C --> Report
""",
        "runtime": """
flowchart TB
  Start["当前周期 active_events"] --> Build["建立事件事实快照\n按 event_key 去重"]
  Build --> Match["匹配法典规则\nwhen_all / when_any / min_match_count"]
  Match --> Hit{"法典条件满足?"}
  Hit -- 是 --> Enter["进入确认\n等待 enter_hold_s"]
  Enter --> Active["规则 ACTIVE\n生成执行候选"]
  Hit -- 否 --> Clear["恢复确认\n等待 clear_hold_s + min_hold_s"]
  Clear --> Recover["规则 RECOVER\n重新计算剩余规则"]
  Active --> Cross["交叉仲裁\npriority / execution_level / 覆盖数"]
  Recover --> Cross
  Cross --> Plan["生成统一执行计划\n不让发现器直接执行"]
  Plan --> Output["输出\nreport_payload + event_executor_cmd"]
""",
        "same-time": """
flowchart LR
  Same["同周期收到 A+B"] --> Facts["active_events = A+B"]
  Facts --> Match{"法典组合规则匹配?"}
  Match -- 是 --> Hold{"enter_hold_s = 0?"}
  Hold -- 是 --> Active["直接生成组合执行计划"]
  Hold -- 否 --> Confirm["短暂确认\n过滤单帧噪声"]
  Confirm --> Pass{"确认通过?"}
  Pass -- 是 --> Active
  Pass -- 否 --> Single["重新匹配单事件规则"]
  Match -- 否 --> Single
  Active --> Execute["交给事件执行层\n软硬通道独立执行"]
""",
        "conflict": """
flowchart LR
  A["A\n导航异常"] --> R1["规则1\nA+B -> 软停"]
  B["B\n碰撞风险"] --> R1
  B --> R2["规则2\nB+C -> 急停"]
  C["C\n底盘异常"] --> R2
  A --> R3["规则3\nA+B+C -> 事件执行计划"]
  B --> R3
  C --> R3
  R1 --> Judge["优先级仲裁\npriority / execution_level / 覆盖数"]
  R2 --> Judge
  R3 --> Judge
  Judge --> Winner["赢家组合"]
  Winner --> Executor["事件执行层\nnodemanager + safety"]
""",
        "recover": """
stateDiagram-v2
  [*] --> INACTIVE
  INACTIVE --> PENDING_ACTIVE: 条件满足
  PENDING_ACTIVE --> ACTIVE: 满足 enter_hold_s
  PENDING_ACTIVE --> INACTIVE: 条件消失
  ACTIVE --> PENDING_CLEAR: 条件不满足
  PENDING_CLEAR --> ACTIVE: 条件重新满足
  PENDING_CLEAR --> INACTIVE: 满足 clear_hold_s + min_hold_s
""",
    }

    for section_id, mermaid in diagrams.items():
        text = replace_section_diagram(text, section_id, renderer.render_mermaid(mermaid.strip()))

    gradual_sequence = """
sequenceDiagram
  participant Time as 时间
  participant Facts as 事件事实集
  participant Arb as 事件仲裁器
  participant Exec as 事件执行层
  Time->>Facts: A 出现
  Facts->>Arb: active_events = A
  Arb->>Exec: 匹配 A 的法典规则后输出执行计划
  Time->>Facts: B 后续出现
  Facts->>Arb: active_events = A+B
  Arb->>Arb: A+B 进入确认
  Arb->>Exec: enter_hold_s 未满足时保持上一轮执行计划
  Arb->>Exec: enter_hold_s 满足后输出组合执行计划
  Time->>Facts: B 恢复但 A 还在
  Arb->>Exec: 组合 clear 后按剩余事件重新生成计划
"""
    text = replace_section_diagram(
        text,
        "gradual",
        renderer.render_mermaid(gradual_sequence.strip()))

    text = renderer.inject_svg_css(text)
    TARGET.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
