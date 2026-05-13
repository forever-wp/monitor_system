#!/usr/bin/env python3
from __future__ import annotations

import html
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

DIAGRAM_CSS = """

    /* HTML diagram renderer: generated from Mermaid blocks, no external dependency. */
    .viz-diagram {
      margin: 18px 0;
      padding: 18px;
      border: 1px solid #d6e4dd;
      border-radius: 16px;
      background: linear-gradient(135deg, #fbfefd 0%, #f2f8f5 100%);
      overflow-x: auto;
    }
    .viz-heading {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 14px;
      padding: 5px 10px;
      border-radius: 999px;
      background: #e7f3ee;
      color: #1f6f50;
      font-size: 12px;
      font-weight: 700;
    }
    .viz-clusters {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
      gap: 12px;
      min-width: 680px;
    }
    .viz-cluster {
      padding: 12px;
      border: 1px dashed #b9cec5;
      border-radius: 14px;
      background: rgb(255 255 255 / 72%);
    }
    .viz-cluster-title {
      margin-bottom: 8px;
      color: #1f6f50;
      font-size: 13px;
      font-weight: 700;
    }
    .viz-node-grid {
      display: grid;
      gap: 8px;
    }
    .viz-node {
      padding: 9px 11px;
      border: 1px solid #dbe4e8;
      border-radius: 11px;
      background: #fff;
      box-shadow: 0 6px 14px rgb(31 42 44 / 6%);
      color: #1f2b2d;
      font-size: 13px;
      line-height: 1.38;
    }
    .viz-node strong {
      display: block;
      color: #163f31;
      font-size: 12px;
      margin-bottom: 2px;
      opacity: .78;
    }
    .viz-edges {
      display: grid;
      gap: 8px;
      margin-top: 14px;
      min-width: 680px;
    }
    .viz-edge {
      display: grid;
      grid-template-columns: minmax(150px, 1fr) auto minmax(70px, auto) auto minmax(150px, 1fr);
      gap: 8px;
      align-items: center;
      padding: 8px;
      border: 1px solid #e1e8eb;
      border-radius: 12px;
      background: #fff;
    }
    .viz-edge-end {
      padding: 8px 10px;
      border-radius: 10px;
      background: #f6f9fa;
      border: 1px solid #e1e8eb;
      font-size: 13px;
    }
    .viz-arrow {
      color: #1f6f50;
      font-weight: 800;
      text-align: center;
    }
    .viz-edge-label {
      padding: 4px 8px;
      border-radius: 999px;
      background: #f6ebc7;
      color: #6c4c0f;
      font-size: 12px;
      text-align: center;
      white-space: nowrap;
    }
    .viz-edge-label:empty {
      display: none;
    }
    .sequence-viz .sequence-lanes {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(135px, 1fr));
      gap: 8px;
      min-width: 680px;
      margin-bottom: 14px;
    }
    .sequence-viz .sequence-lane {
      padding: 9px 10px;
      border: 1px solid #cfe0d8;
      border-radius: 12px;
      background: #fff;
      color: #1f6f50;
      font-weight: 700;
      text-align: center;
      font-size: 13px;
    }
    .sequence-viz .sequence-steps {
      display: grid;
      gap: 8px;
      min-width: 680px;
    }
    .sequence-viz .sequence-step {
      display: grid;
      grid-template-columns: minmax(130px, 1fr) auto minmax(130px, 1fr) minmax(220px, 2fr);
      gap: 8px;
      align-items: center;
      padding: 9px;
      border: 1px solid #e1e8eb;
      border-radius: 12px;
      background: #fff;
    }
    .sequence-viz .sequence-actor {
      padding: 7px 9px;
      border-radius: 10px;
      background: #f6f9fa;
      border: 1px solid #e1e8eb;
      font-size: 13px;
      text-align: center;
    }
    .sequence-viz .sequence-message {
      color: #5f6f73;
      font-size: 13px;
    }
    .state-viz .state-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 10px;
      min-width: 620px;
      margin-bottom: 14px;
    }
    .state-viz .state-node {
      padding: 12px;
      border-radius: 13px;
      border: 1px solid #cfe0d8;
      background: #fff;
      color: #1f6f50;
      font-weight: 700;
      text-align: center;
    }
    .state-viz .state-transitions {
      display: grid;
      gap: 8px;
      min-width: 620px;
    }
    .state-viz .state-transition {
      display: grid;
      grid-template-columns: minmax(135px, 1fr) auto minmax(120px, auto) auto minmax(135px, 1fr);
      gap: 8px;
      align-items: center;
      padding: 8px;
      border: 1px solid #e1e8eb;
      border-radius: 12px;
      background: #fff;
    }
"""


def clean_label(value: str) -> str:
    value = html.unescape(value).strip()
    value = value.strip('"').strip("'")
    value = value.replace("<br/>", "\n").replace("<br>", "\n").replace("\\n", "\n")
    return value.strip()


def strip_node_token(token: str) -> str:
    token = token.strip()
    token = re.sub(r"^[|:]+|[|:]+$", "", token).strip()
    token = re.sub(r"[\[{(].*$", "", token).strip()
    return token


def extract_node(token: str) -> tuple[str, str]:
    token = token.strip()
    patterns = [
        r'^([A-Za-z_][\w]*)\s*\[\s*"?(.*?)"?\s*\]$',
        r'^([A-Za-z_][\w]*)\s*\{\s*"?(.*?)"?\s*\}$',
        r'^([A-Za-z_][\w]*)\s*\[\s*/?(.*?)/?\s*\]$',
    ]
    for pattern in patterns:
        m = re.match(pattern, token)
        if m:
            return m.group(1), clean_label(m.group(2))
    key = strip_node_token(token)
    return key, key


def html_text(value: str) -> str:
    return html.escape(value).replace("\n", "<br>")


def parse_flowchart(raw: str) -> str:
    lines = [line.strip() for line in raw.splitlines() if line.strip()]
    nodes: dict[str, str] = {}
    groups: dict[str, list[str]] = {}
    current_group = "节点"
    groups[current_group] = []
    edges: list[tuple[str, str, str]] = []

    def add_node(token: str, group: str | None = None) -> str:
        key, label = extract_node(token)
        if not key:
            return key
        nodes.setdefault(key, label or key)
        target_group = group or current_group
        groups.setdefault(target_group, [])
        if key not in groups[target_group]:
            groups[target_group].append(key)
        return key

    for line in lines[1:]:
        if line.startswith("subgraph "):
            m = re.match(r"subgraph\s+\w+\s*\[\s*\"?(.*?)\"?\s*\]", line)
            current_group = clean_label(m.group(1)) if m else clean_label(line.replace("subgraph", "", 1))
            groups.setdefault(current_group, [])
            continue
        if line == "end":
            current_group = "节点"
            groups.setdefault(current_group, [])
            continue

        # Chain first: A --> B --> C
        if line.count("-->") >= 2 and "|" not in line:
            parts = [p.strip() for p in line.split("-->")]
            keys = [add_node(part) for part in parts]
            for src, dst in zip(keys, keys[1:]):
                edges.append((src, "", dst))
            continue

        # Edge with |label|
        m = re.match(r"(.+?)\s*-+>\|(.+?)\|\s*(.+)$", line)
        if m:
            left, label, right = m.group(1), clean_label(m.group(2)), m.group(3)
            src = add_node(left)
            dst = add_node(right)
            edges.append((src, label, dst))
            continue

        # Edge with text between dashes: A -- yes --> B
        m = re.match(r"(.+?)\s*--\s*(.+?)\s*-->\s*(.+)$", line)
        if m:
            left, label, right = m.group(1), clean_label(m.group(2)), m.group(3)
            src = add_node(left)
            dst = add_node(right)
            edges.append((src, label, dst))
            continue

        m = re.match(r"(.+?)\s*-->\s*(.+)$", line)
        if m:
            left, right = m.group(1), m.group(2)
            src = add_node(left)
            dst = add_node(right)
            edges.append((src, "", dst))
            continue

        if "[" in line or "{" in line:
            add_node(line)

    if not groups.get("节点"):
        groups.pop("节点", None)

    clusters_html = []
    for group, keys in groups.items():
        if not keys:
            continue
        node_items = "".join(
            f'<div class="viz-node"><strong>{html.escape(key)}</strong>{html_text(nodes.get(key, key))}</div>'
            for key in keys
        )
        clusters_html.append(
            f'<div class="viz-cluster"><div class="viz-cluster-title">{html_text(group)}</div>'
            f'<div class="viz-node-grid">{node_items}</div></div>'
        )

    edge_html = "".join(
        '<div class="viz-edge">'
        f'<div class="viz-edge-end">{html_text(nodes.get(src, src))}</div>'
        '<div class="viz-arrow">→</div>'
        f'<div class="viz-edge-label">{html_text(label)}</div>'
        '<div class="viz-arrow">→</div>'
        f'<div class="viz-edge-end">{html_text(nodes.get(dst, dst))}</div>'
        '</div>'
        for src, label, dst in edges
    )

    return (
        '<div class="viz-diagram flow-viz">'
        '<div class="viz-heading">HTML 可视化流程图</div>'
        f'<div class="viz-clusters">{"".join(clusters_html)}</div>'
        f'<div class="viz-edges">{edge_html}</div>'
        '</div>'
    )


def parse_sequence(raw: str) -> str:
    participants: dict[str, str] = {}
    messages: list[tuple[str, str, str, str]] = []

    for line in raw.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        m = re.match(r"participant\s+(\w+)\s+as\s+(.+)$", line)
        if m:
            participants[m.group(1)] = clean_label(m.group(2))
            continue
        m = re.match(r"(\w+)\s*(-+>>|--+>>|->>|-->>)\s*(\w+)\s*:\s*(.+)$", line)
        if m:
            src, arrow, dst, label = m.groups()
            participants.setdefault(src, src)
            participants.setdefault(dst, dst)
            messages.append((src, arrow, dst, clean_label(label)))

    lanes = "".join(f'<div class="sequence-lane">{html_text(name)}</div>' for name in participants.values())
    steps = "".join(
        '<div class="sequence-step">'
        f'<div class="sequence-actor">{html_text(participants.get(src, src))}</div>'
        f'<div class="viz-arrow">{"⇢" if "--" in arrow else "→"}</div>'
        f'<div class="sequence-actor">{html_text(participants.get(dst, dst))}</div>'
        f'<div class="sequence-message">{html_text(label)}</div>'
        '</div>'
        for src, arrow, dst, label in messages
    )
    return (
        '<div class="viz-diagram sequence-viz">'
        '<div class="viz-heading">HTML 可视化时序图</div>'
        f'<div class="sequence-lanes">{lanes}</div>'
        f'<div class="sequence-steps">{steps}</div>'
        '</div>'
    )


def parse_state(raw: str) -> str:
    transitions: list[tuple[str, str, str]] = []
    states: list[str] = []

    def add_state(name: str) -> None:
        if name and name not in {"[*]"} and name not in states:
            states.append(name)

    for line in raw.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        m = re.match(r"(.+?)\s*-->\s*(.+?)(?::\s*(.+))?$", line)
        if m:
            src, dst, label = clean_label(m.group(1)), clean_label(m.group(2)), clean_label(m.group(3) or "")
            add_state(src)
            add_state(dst)
            transitions.append((src, label, dst))

    state_html = "".join(f'<div class="state-node">{html_text(state)}</div>' for state in states)
    trans_html = "".join(
        '<div class="state-transition">'
        f'<div class="viz-edge-end">{html_text(src)}</div>'
        '<div class="viz-arrow">→</div>'
        f'<div class="viz-edge-label">{html_text(label)}</div>'
        '<div class="viz-arrow">→</div>'
        f'<div class="viz-edge-end">{html_text(dst)}</div>'
        '</div>'
        for src, label, dst in transitions
    )
    return (
        '<div class="viz-diagram state-viz">'
        '<div class="viz-heading">HTML 可视化状态机</div>'
        f'<div class="state-grid">{state_html}</div>'
        f'<div class="state-transitions">{trans_html}</div>'
        '</div>'
    )


def render_mermaid(raw: str) -> str:
    raw = html.unescape(raw).strip()
    first = raw.splitlines()[0].strip() if raw else ""
    if first.startswith("sequenceDiagram"):
        return parse_sequence(raw)
    if first.startswith("stateDiagram"):
        return parse_state(raw)
    if first.startswith("flowchart") or first.startswith("graph"):
        return parse_flowchart(raw)
    return f'<pre><code class="language-text">{html.escape(raw)}</code></pre>'


def inject_css(text: str) -> str:
    if "HTML diagram renderer: generated from Mermaid blocks" in text:
        return text
    return text.replace("</style>", DIAGRAM_CSS + "\n  </style>", 1)


def process_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(r'<pre><code class="language-mermaid">\n?(.*?)</code></pre>', re.S)
    if not pattern.search(text):
      return False
    new_text = pattern.sub(lambda m: render_mermaid(m.group(1)), text)
    new_text = inject_css(new_text)
    path.write_text(new_text, encoding="utf-8")
    return True


def main() -> None:
    changed = []
    for path in ROOT.rglob("*.html"):
        if any(part in {".git", "build", "install", "log"} for part in path.parts):
            continue
        if process_file(path):
            changed.append(path.relative_to(ROOT))
    for path in changed:
        print(path)


if __name__ == "__main__":
    main()
