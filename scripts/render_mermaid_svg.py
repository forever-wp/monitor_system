#!/usr/bin/env python3
from __future__ import annotations

import html
import importlib.util
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FONT = "Noto Sans CJK SC, Microsoft YaHei, Arial"

TARGETS = {
    "docs/project_architecture.md": "docs/project_architecture.html",
    "docs/monitor_modular_isolation_design.md": "docs/monitor_modular_isolation_design.html",
    "src/nav2_monitor/docs/architecture_diagrams.md": "src/nav2_monitor/docs/architecture_diagrams.html",
}

SVG_CSS = """

    /* SVG diagrams generated from Mermaid blocks. Offline, no CDN/runtime dependency. */
    .svg-diagram {
      margin: 20px 0;
      padding: 20px;
      border: 1px solid #cfe0d8;
      border-radius: 22px;
      background:
        radial-gradient(circle at 10% 8%, rgb(255 255 255 / 92%), transparent 30%),
        linear-gradient(135deg, #fbfefd 0%, #eef7f3 100%);
      overflow-x: auto;
      box-shadow: 0 14px 30px rgb(31 42 44 / 8%);
    }
    .svg-diagram-title {
      display: inline-flex;
      align-items: center;
      margin-bottom: 14px;
      padding: 6px 12px;
      border-radius: 999px;
      background: #dff0e8;
      color: #1f6f50;
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.02em;
    }
    .svg-diagram-legend {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: -2px 0 14px;
      color: #5b6c67;
      font-size: 12px;
    }
    .svg-diagram-legend span {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      padding: 3px 8px;
      border-radius: 999px;
      background: rgb(255 255 255 / 68%);
      border: 1px solid #d6e5df;
    }
    .svg-diagram-legend i {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      display: inline-block;
    }
    .svg-diagram svg {
      display: block;
      max-width: 100%;
      height: auto;
      min-width: 720px;
    }
"""


def semantic_colors(label: str, key: str = "") -> tuple[str, str, str]:
    text = f"{label} {key}".lower()
    red_keys = ("急停", "碰撞", "风险", "异常", "故障", "失败", "丢失", "emergency", "collision", "risk", "fault", "error")
    gold_keys = ("策略", "仲裁", "规则", "判断", "确认", "切换", "winner", "match", "policy", "arbiter", "judge")
    blue_keys = ("输入", "数据", "topic", "source", "sensor", "camera", "雷达", "点云", "scan", "tf", "odom", "cmd_vel")
    green_keys = ("输出", "上报", "状态", "恢复", "健康", "status", "event", "report", "clear", "monitor")
    violet_keys = ("执行", "动作", "安全", "节点", "nodemanager", "supervisor", "safety", "action", "execute", "cmd")

    if any(key_word in text for key_word in red_keys):
        return "#fff1ec", "#d26a56", "#7c2f25"
    if any(key_word in text for key_word in violet_keys):
        return "#f1efff", "#8d7ed3", "#463a88"
    if any(key_word in text for key_word in gold_keys):
        return "#fff6db", "#d2a235", "#6c4d00"
    if any(key_word in text for key_word in blue_keys):
        return "#edf6ff", "#6ba2d7", "#265b89"
    if any(key_word in text for key_word in green_keys):
        return "#eaf7ef", "#61a87d", "#1f6f50"
    return "#ffffff", "#86b6a3", "#1f2a2c"


def flow_node_dot(key: str, label: str, shape: str) -> str:
    fill, stroke, font = semantic_colors(label, key)
    dot_shape = "diamond" if shape == "decision" else "ellipse" if shape == "round" else "box"
    style = "filled" if shape == "decision" else "rounded,filled"
    margin = "0.16,0.10" if shape == "decision" else "0.13,0.08"
    return (
        f"{dot_quote(key)} ["
        f"label={dot_quote(label)}, "
        f"shape={dot_quote(dot_shape)}, "
        f"style={dot_quote(style)}, "
        f"fillcolor={dot_quote(fill)}, "
        f"color={dot_quote(stroke)}, "
        f"fontcolor={dot_quote(font)}, "
        "penwidth=1.45, "
        f"margin={dot_quote(margin)}"
        "];"
    )


def load_convert_module():
    spec = importlib.util.spec_from_file_location(
        "convert_docs_to_html", ROOT / "scripts/convert_docs_to_html.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def dot_quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def clean_label(value: str | None) -> str:
    if not value:
        return ""
    value = html.unescape(value).strip()
    value = value.strip('"').strip("'")
    value = value.replace("<br/>", "\n").replace("<br>", "\n").replace("\\n", "\n")
    return value.strip()


def mermaid_logical_lines(raw: str) -> list[str]:
    """Merge physical lines that are part of the same Mermaid node label."""
    lines: list[str] = []
    buffer = ""
    balance = 0
    quote_open = False
    escape = False

    def feed(text: str) -> None:
        nonlocal balance, quote_open, escape
        for char in text:
            if escape:
                escape = False
                continue
            if char == "\\":
                escape = True
                continue
            if char == '"':
                quote_open = not quote_open
                continue
            if quote_open:
                continue
            if char in "[{(":
                balance += 1
            elif char in "]})" and balance > 0:
                balance -= 1

    for raw_line in raw.splitlines():
        stripped = raw_line.strip()
        if not stripped:
            continue
        if buffer:
            buffer += "\\n" + stripped
        else:
            buffer = stripped
            balance = 0
            quote_open = False
            escape = False
        feed(stripped)
        if balance == 0 and not quote_open:
            lines.append(buffer)
            buffer = ""

    if buffer:
        lines.append(buffer)
    return lines


def strip_node_token(token: str) -> str:
    token = token.strip()
    token = re.sub(r"^[|:]+|[|:]+$", "", token).strip()
    token = re.sub(r"[\[{(].*$", "", token).strip()
    return token


def extract_node(token: str) -> tuple[str, str, str]:
    token = token.strip()
    patterns = [
        (r'^([A-Za-z_][\w]*)\s*\[\s*"([\s\S]*?)"\s*\]$', "box"),
        (r'^([A-Za-z_][\w]*)\s*\{\s*"([\s\S]*?)"\s*\}$', "decision"),
        (r'^([A-Za-z_][\w]*)\s*\(\s*"([\s\S]*?)"\s*\)$', "round"),
        (r"^([A-Za-z_][\w]*)\s*\[\s*'([\s\S]*?)'\s*\]$", "box"),
        (r"^([A-Za-z_][\w]*)\s*\{\s*'([\s\S]*?)'\s*\}$", "decision"),
        (r"^([A-Za-z_][\w]*)\s*\(\s*'([\s\S]*?)'\s*\)$", "round"),
        (r'^([A-Za-z_][\w]*)\s*\[\s*([\s\S]*?)\s*\]$', "box"),
        (r'^([A-Za-z_][\w]*)\s*\{\s*([\s\S]*?)\s*\}$', "decision"),
        (r'^([A-Za-z_][\w]*)\s*\(\s*([\s\S]*?)\s*\)$', "round"),
    ]
    for pattern, shape in patterns:
        match = re.match(pattern, token, flags=re.S)
        if match:
            return match.group(1), clean_label(match.group(2)), shape
    key = strip_node_token(token)
    return key, key, "box"


class FlowGraph:
    def __init__(self, direction: str) -> None:
        self.direction = direction
        self.nodes: dict[str, str] = {}
        self.node_shapes: dict[str, str] = {}
        self.groups: dict[str, list[str]] = {}
        self.node_group: dict[str, str] = {}
        self.edges: list[tuple[str, str, str]] = []

    def add_node(self, token: str, group: str | None = None) -> str:
        key, label, shape = extract_node(token)
        if not key:
            return key
        existing = self.nodes.get(key)
        if existing is None or (existing == key and label and label != key):
            self.nodes[key] = label or key
        if key not in self.node_shapes or (self.node_shapes[key] == "box" and shape != "box"):
            self.node_shapes[key] = shape
        if group:
            self.groups.setdefault(group, [])
            if key not in self.groups[group]:
                self.groups[group].append(key)
            self.node_group.setdefault(key, group)
        return key

    def add_edge(self, left: str, right: str, label: str = "") -> None:
        src = self.add_node(left)
        dst = self.add_node(right)
        if src and dst:
            self.edges.append((src, dst, label))


def parse_flowchart(raw: str) -> FlowGraph:
    lines = mermaid_logical_lines(raw)
    first = lines[0] if lines else "flowchart TB"
    direction = "LR" if re.search(r"\bLR\b", first) else "TB"
    graph = FlowGraph(direction)
    current_group: str | None = None

    for line in lines[1:]:
        if line.startswith("subgraph "):
            match = re.match(r"subgraph\s+\w+\s*\[\s*\"?(.*?)\"?\s*\]", line)
            current_group = clean_label(match.group(1)) if match else clean_label(line.replace("subgraph", "", 1))
            graph.groups.setdefault(current_group, [])
            continue
        if line == "end":
            current_group = None
            continue

        if line.count("-->") >= 2 and "|" not in line:
            parts = [part.strip() for part in line.split("-->")]
            keys = [graph.add_node(part, current_group) for part in parts]
            for src, dst in zip(keys, keys[1:]):
                graph.edges.append((src, dst, ""))
            continue

        match = re.match(r"(.+?)\s*-+>\|(.+?)\|\s*(.+)$", line)
        if match:
            left, label, right = match.group(1), clean_label(match.group(2)), match.group(3)
            graph.add_node(left, current_group)
            graph.add_node(right, current_group)
            graph.add_edge(left, right, label)
            continue

        match = re.match(r"(.+?)\s*--\s*(.+?)\s*-->\s*(.+)$", line)
        if match:
            left, label, right = match.group(1), clean_label(match.group(2)), match.group(3)
            graph.add_node(left, current_group)
            graph.add_node(right, current_group)
            graph.add_edge(left, right, label)
            continue

        match = re.match(r"(.+?)\s*-->\s*(.+)$", line)
        if match:
            left, right = match.group(1), match.group(2)
            graph.add_node(left, current_group)
            graph.add_node(right, current_group)
            graph.add_edge(left, right)
            continue

        if "[" in line or "{" in line or "(" in line:
            graph.add_node(line, current_group)

    return graph


def run_dot(dot: str) -> str:
    result = subprocess.run(
        ["dot", "-Tsvg"],
        input=dot,
        text=True,
        capture_output=True,
        check=True,
    )
    svg = result.stdout
    start = svg.find("<svg")
    if start >= 0:
        svg = svg[start:]
    svg = re.sub(r"<title>.*?</title>", "", svg, flags=re.S)
    return svg


def flow_to_svg(raw: str) -> str:
    graph = parse_flowchart(raw)
    rankdir = "LR" if graph.direction == "LR" else "TB"
    lines = [
        "digraph G {",
        f"graph [rankdir={rankdir}, bgcolor=\"transparent\", pad=\"0.25\", nodesep=\"0.45\", ranksep=\"0.62\", splines=ortho, outputorder=edgesfirst];",
        f"node [shape=box, style=\"rounded,filled\", fillcolor=\"#ffffff\", color=\"#86b6a3\", penwidth=1.35, fontname={dot_quote(FONT)}, fontsize=12, margin=\"0.13,0.08\"];",
        f"edge [color=\"#1f6f50\", arrowsize=0.78, penwidth=1.45, fontname={dot_quote(FONT)}, fontsize=10];",
    ]

    clustered = set()
    for idx, (group, keys) in enumerate(graph.groups.items()):
        if not keys:
            continue
        lines.extend([
            f"subgraph cluster_{idx} {{",
            f"label={dot_quote(group)};",
            f"fontname={dot_quote(FONT)};",
            "fontsize=14;",
            "fontcolor=\"#1f6f50\";",
            "style=\"rounded,filled\";",
            "color=\"#c8ded4\";",
            "fillcolor=\"#f7fbf9\";",
        ])
        for key in keys:
            clustered.add(key)
            label = graph.nodes.get(key, key)
            lines.append(flow_node_dot(key, label, graph.node_shapes.get(key, "box")))
        lines.append("}")

    for key, label in graph.nodes.items():
        if key not in clustered:
            lines.append(flow_node_dot(key, label, graph.node_shapes.get(key, "box")))

    for src, dst, label in graph.edges:
        attr = f" [label={dot_quote(label)}, color=\"#5b8f79\"]" if label else ""
        lines.append(f"{dot_quote(src)} -> {dot_quote(dst)}{attr};")

    lines.append("}")
    return wrap_svg(run_dot("\n".join(lines)), "可视化流程图")


def parse_sequence(raw: str):
    participants: dict[str, str] = {}
    messages: list[tuple[str, str, str, str]] = []
    for line in raw.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        match = re.match(r"participant\s+(\w+)\s+as\s+(.+)$", line)
        if match:
            participants[match.group(1)] = clean_label(match.group(2))
            continue
        match = re.match(r"(\w+)\s*(-+>>|--+>>|->>|-->>)\s*(\w+)\s*:\s*(.+)$", line)
        if match:
            src, arrow, dst, label = match.groups()
            participants.setdefault(src, src)
            participants.setdefault(dst, dst)
            messages.append((src, arrow, dst, clean_label(label)))
    return participants, messages


def wrap_svg_text(text: str, width: int = 18) -> list[str]:
    text = clean_label(text)
    if len(text) <= width:
        return [text]
    chunks: list[str] = []
    line = ""
    for char in text:
        line += char
        if len(line) >= width:
            chunks.append(line)
            line = ""
    if line:
        chunks.append(line)
    return chunks[:3]


def sequence_to_svg(raw: str) -> str:
    participants, messages = parse_sequence(raw)
    actor_keys = list(participants.keys())
    col_w = 170
    left = 55
    top = 72
    step_h = 76
    width = max(860, left * 2 + max(1, len(actor_keys) - 1) * col_w + 120)
    height = top + len(messages) * step_h + 85
    xs = {key: left + idx * col_w + 70 for idx, key in enumerate(actor_keys)}

    body: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img">',
        "<defs>",
        '<marker id="arrow-seq" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L0,6 L9,3 z" fill="#1f6f50"/></marker>',
        "</defs>",
        f'<rect width="{width}" height="{height}" rx="18" fill="#fbfefd"/>',
    ]

    for key, x in xs.items():
        label = participants[key]
        fill, stroke, font = semantic_colors(label, key)
        body.append(f'<rect x="{x - 65}" y="18" width="130" height="38" rx="10" fill="{fill}" stroke="{stroke}" stroke-width="1.3"/>')
        body.append(f'<text x="{x}" y="42" text-anchor="middle" font-family="{FONT}" font-size="12" fill="{font}" font-weight="700">{html.escape(label)}</text>')
        body.append(f'<line x1="{x}" y1="58" x2="{x}" y2="{height - 32}" stroke="#c7d8d0" stroke-width="1.2" stroke-dasharray="5 5"/>')

    for idx, (src, arrow, dst, label) in enumerate(messages, start=1):
        y = top + (idx - 1) * step_h
        x1 = xs[src]
        x2 = xs[dst]
        dashed = ' stroke-dasharray="6 5"' if "--" in arrow else ""
        _, stroke, font = semantic_colors(label, f"{src} {dst}")
        if src == dst:
            loop_w = 72
            body.append(f'<path d="M{x1},{y} C{x1 + loop_w},{y} {x1 + loop_w},{y + 32} {x1},{y + 32}" fill="none" stroke="{stroke}" stroke-width="1.7" marker-end="url(#arrow-seq)"{dashed}/>')
            label_x = x1 + loop_w + 8
            label_anchor = "start"
        else:
            start = x1 + (8 if x2 > x1 else -8)
            end = x2 - (8 if x2 > x1 else -8)
            body.append(f'<line x1="{start}" y1="{y}" x2="{end}" y2="{y}" stroke="{stroke}" stroke-width="1.7" marker-end="url(#arrow-seq)"{dashed}/>')
            label_x = (x1 + x2) / 2
            label_anchor = "middle"
        body.append(f'<circle cx="{x1}" cy="{y}" r="12" fill="#ffffff" stroke="{stroke}"/>')
        body.append(f'<text x="{x1}" y="{y + 4}" text-anchor="middle" font-family="{FONT}" font-size="10" fill="{font}" font-weight="700">{idx}</text>')
        for line_no, chunk in enumerate(wrap_svg_text(label), start=0):
            body.append(f'<text x="{label_x}" y="{y - 10 + line_no * 15}" text-anchor="{label_anchor}" font-family="{FONT}" font-size="12" fill="{font}">{html.escape(chunk)}</text>')

    body.append("</svg>")
    return wrap_svg("\n".join(body), "可视化时序图")


def state_to_svg(raw: str) -> str:
    states: set[str] = set()
    edges: list[tuple[str, str, str]] = []
    for line in raw.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        match = re.match(r"(.+?)\s*-->\s*(.+?)(?::\s*(.+))?$", line)
        if not match:
            continue
        src, dst, label = clean_label(match.group(1)), clean_label(match.group(2)), clean_label(match.group(3) or "")
        states.add(src)
        states.add(dst)
        edges.append((src, dst, label))

    lines = [
        "digraph G {",
        "graph [rankdir=LR, bgcolor=\"transparent\", pad=\"0.25\", nodesep=\"0.5\", ranksep=\"0.72\", splines=true];",
        f"node [shape=box, style=\"rounded,filled\", fillcolor=\"#ffffff\", color=\"#86b6a3\", penwidth=1.35, fontname={dot_quote(FONT)}, fontsize=12, margin=\"0.13,0.08\"];",
        f"edge [color=\"#1f6f50\", arrowsize=0.78, penwidth=1.45, fontname={dot_quote(FONT)}, fontsize=10];",
    ]
    for state in sorted(states):
        if state == "[*]":
            lines.append(f"{dot_quote(state)} [shape=circle, label=\"\", width=0.22, fillcolor=\"#1f6f50\", color=\"#1f6f50\"];")
        else:
            fill, stroke, font = semantic_colors(state)
            lines.append(
                f"{dot_quote(state)} [label={dot_quote(state)}, "
                f"fillcolor={dot_quote(fill)}, color={dot_quote(stroke)}, "
                f"fontcolor={dot_quote(font)}, penwidth=1.45];")
    for src, dst, label in edges:
        attr = f" [label={dot_quote(label)}]" if label else ""
        lines.append(f"{dot_quote(src)} -> {dot_quote(dst)}{attr};")
    lines.append("}")
    return wrap_svg(run_dot("\n".join(lines)), "可视化状态机")


def wrap_svg(svg: str, title: str) -> str:
    legend = (
        '<div class="svg-diagram-legend">'
        '<span><i style="background:#6ba2d7"></i>输入 / 数据源</span>'
        '<span><i style="background:#d2a235"></i>判断 / 策略</span>'
        '<span><i style="background:#61a87d"></i>状态 / 输出</span>'
        '<span><i style="background:#d26a56"></i>故障 / 风险</span>'
        '<span><i style="background:#8d7ed3"></i>执行 / 安全动作</span>'
        '</div>'
    )
    return f'<div class="svg-diagram"><div class="svg-diagram-title">{html.escape(title)}</div>{legend}{svg}</div>'


def render_mermaid(raw: str) -> str:
    raw = html.unescape(raw).strip()
    first = raw.splitlines()[0].strip() if raw else ""
    if first.startswith("sequenceDiagram"):
        return sequence_to_svg(raw)
    if first.startswith("stateDiagram"):
        return state_to_svg(raw)
    if first.startswith("flowchart") or first.startswith("graph"):
        return flow_to_svg(raw)
    return f'<pre><code class="language-text">{html.escape(raw)}</code></pre>'


def inject_svg_css(text: str) -> str:
    marker = "/* SVG diagrams generated from Mermaid blocks. Offline, no CDN/runtime dependency. */"
    start = text.find(marker)
    if start != -1:
        line_start = text.rfind("\n", 0, start)
        end = text.find("</style>", start)
        if line_start != -1 and end != -1:
            return text[:line_start + 1] + SVG_CSS + "\n" + text[end:]
    return text.replace("</style>", SVG_CSS + "\n  </style>", 1)


def render_html_diagrams(text: str) -> str:
    pattern = re.compile(r'<pre><code class="language-mermaid">\n?(.*?)</code></pre>', re.S)
    text = pattern.sub(lambda match: render_mermaid(match.group(1)), text)
    return inject_svg_css(text)


def rebuild_html_from_head(src: str, dst: str, convert) -> None:
    raw = subprocess.check_output(["git", "show", f"HEAD:{src}"], cwd=ROOT, text=True)
    raw = convert.update_links_in_text(raw)
    if src == "docs/project_architecture.md" and "combined_fault_policy_arbiter_design.html" not in raw:
        marker = "后续模块独立化、可靠数据联通、QoS 分层、CPU 满载降级与上层确认边界的完整方案见：[监控系统模块独立化与可靠数据联通设计方案](monitor_modular_isolation_design.html)。"
        raw = raw.replace(
            marker,
            marker + "\n\n多错误组合、动作覆盖、交叉策略和组合恢复的独立仲裁层设计见：[多错误组合策略仲裁层设计页](combined_fault_policy_arbiter_design.html)。",
        )
    title = Path(src).stem.replace("_", " ")
    text = convert.wrap_html(title, convert.to_html_body(raw))
    text = render_html_diagrams(text)
    (ROOT / dst).write_text(text, encoding="utf-8")


def main() -> None:
    convert = load_convert_module()
    for src, dst in TARGETS.items():
        rebuild_html_from_head(src, dst, convert)
        print(dst)


if __name__ == "__main__":
    main()
