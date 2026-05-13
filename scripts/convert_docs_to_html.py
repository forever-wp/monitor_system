#!/usr/bin/env python3
from __future__ import annotations

import html
import os
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def is_readme(path: Path) -> bool:
    name = path.name.lower()
    return name == "readme.md"


def collect_non_readme_markdown() -> list[Path]:
    files: list[Path] = []
    for path in ROOT.rglob("*.md"):
        if ".git" in path.parts:
            continue
        if "build" in path.parts or "install" in path.parts or "log" in path.parts:
            continue
        if is_readme(path):
            continue
        files.append(path)
    return sorted(files)


def slugify(text: str) -> str:
    text = text.strip().lower()
    text = re.sub(r"[`*_~\[\](){}<>]", "", text)
    text = re.sub(r"[^\w\u4e00-\u9fff\- ]+", "", text, flags=re.UNICODE)
    text = re.sub(r"\s+", "-", text)
    return text or "section"


def convert_inline(text: str) -> str:
    text = html.escape(text, quote=False)
    text = re.sub(r"`([^`]+)`", r"<code>\1</code>", text)
    text = re.sub(r"\*\*([^\*]+)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"\*([^\*]+)\*", r"<em>\1</em>", text)

    def repl_link(m: re.Match[str]) -> str:
        label = m.group(1)
        href = m.group(2)
        if href.endswith(".md") and Path(href).name.lower() != "readme.md":
            href = href[:-3] + ".html"
        return f'<a href="{html.escape(href, quote=True)}">{label}</a>'

    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", repl_link, text)
    return text


def flush_paragraph(out: list[str], buf: list[str]) -> None:
    if not buf:
        return
    merged = " ".join(s.strip() for s in buf if s.strip())
    if merged:
        out.append(f"<p>{convert_inline(merged)}</p>")
    buf.clear()


def to_html_body(md_text: str) -> str:
    lines = md_text.splitlines()
    out: list[str] = []

    in_code = False
    code_lang = ""
    paragraph: list[str] = []
    in_ul = False
    in_ol = False
    in_table = False
    table_rows: list[list[str]] = []
    headings: list[tuple[str, str]] = []

    def close_lists() -> None:
        nonlocal in_ul, in_ol
        if in_ul:
            out.append("</ul>")
            in_ul = False
        if in_ol:
            out.append("</ol>")
            in_ol = False

    def flush_table() -> None:
        nonlocal in_table, table_rows
        if not in_table or not table_rows:
            in_table = False
            table_rows = []
            return
        out.append('<div class="table-wrap"><table>')
        header = table_rows[0]
        out.append("<thead><tr>" + "".join(f"<th>{convert_inline(c.strip())}</th>" for c in header) + "</tr></thead>")
        body_rows = table_rows[1:]
        if body_rows and all(re.fullmatch(r"[:\- ]+", c.strip()) for c in body_rows[0]):
            body_rows = body_rows[1:]
        out.append("<tbody>")
        for row in body_rows:
            out.append("<tr>" + "".join(f"<td>{convert_inline(c.strip())}</td>" for c in row) + "</tr>")
        out.append("</tbody></table></div>")
        in_table = False
        table_rows = []

    for raw in lines:
        line = raw.rstrip("\n")
        stripped = line.strip()

        if in_code:
            if stripped.startswith("```"):
                out.append("</code></pre>")
                in_code = False
                code_lang = ""
            else:
                out.append(html.escape(line))
            continue

        if stripped.startswith("```"):
            flush_paragraph(out, paragraph)
            close_lists()
            flush_table()
            code_lang = stripped[3:].strip()
            class_attr = f' class="language-{html.escape(code_lang, quote=True)}"' if code_lang else ""
            out.append(f"<pre><code{class_attr}>")
            in_code = True
            continue

        if not stripped:
            flush_paragraph(out, paragraph)
            close_lists()
            flush_table()
            continue

        if stripped.startswith("#"):
            flush_paragraph(out, paragraph)
            close_lists()
            flush_table()
            m = re.match(r"^(#{1,6})\s+(.*)$", stripped)
            if m:
                level = len(m.group(1))
                title = m.group(2).strip()
                hid = slugify(title)
                headings.append((hid, title))
                out.append(f'<h{level} id="{hid}">{convert_inline(title)}</h{level}>')
                continue

        if stripped.startswith(">"):
            flush_paragraph(out, paragraph)
            close_lists()
            flush_table()
            quote = stripped[1:].strip()
            out.append(f"<blockquote>{convert_inline(quote)}</blockquote>")
            continue

        if "|" in stripped and stripped.startswith("|") and stripped.endswith("|"):
            flush_paragraph(out, paragraph)
            close_lists()
            cols = [c.strip() for c in stripped.strip("|").split("|")]
            table_rows.append(cols)
            in_table = True
            continue
        elif in_table:
            flush_table()

        ul = re.match(r"^[-*]\s+(.*)$", stripped)
        if ul:
            flush_paragraph(out, paragraph)
            close_lists()
            if not in_ul:
                out.append("<ul>")
                in_ul = True
            out.append(f"<li>{convert_inline(ul.group(1))}</li>")
            continue

        ol = re.match(r"^\d+\.\s+(.*)$", stripped)
        if ol:
            flush_paragraph(out, paragraph)
            close_lists()
            if not in_ol:
                out.append("<ol>")
                in_ol = True
            out.append(f"<li>{convert_inline(ol.group(1))}</li>")
            continue

        paragraph.append(stripped)

    flush_paragraph(out, paragraph)
    close_lists()
    flush_table()

    toc = ""
    if headings:
        items = "".join(f'<li><a href="#{hid}">{html.escape(title)}</a></li>' for hid, title in headings if title)
        toc = f'<aside class="toc"><strong>目录</strong><ul>{items}</ul></aside>'

    return toc + "\n" + "\n".join(out)


def wrap_html(title: str, body: str) -> str:
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      --bg: #f6f7f8;
      --paper: #ffffff;
      --ink: #1f2a2c;
      --muted: #5f6f73;
      --line: #dbe3e6;
      --brand: #1f6f50;
      --brand-soft: #e7f3ee;
      --shadow: 0 10px 28px rgb(31 42 44 / 8%);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      background: linear-gradient(160deg, #f6f7f8 0%, #edf3f5 100%);
      font-family: "Noto Sans SC", "Microsoft YaHei", Arial, sans-serif;
      line-height: 1.72;
    }}
    .page {{
      width: min(1100px, calc(100% - 28px));
      margin: 0 auto;
      padding: 28px 0 48px;
    }}
    .card {{
      background: var(--paper);
      border: 1px solid var(--line);
      border-radius: 18px;
      box-shadow: var(--shadow);
      padding: 28px;
    }}
    h1, h2, h3, h4, h5, h6 {{ line-height: 1.28; margin: 22px 0 10px; }}
    h1 {{ margin-top: 6px; font-size: clamp(28px, 4vw, 44px); }}
    h2 {{ font-size: clamp(21px, 2.7vw, 31px); }}
    h3 {{ font-size: clamp(18px, 2.1vw, 24px); }}
    p {{ margin: 10px 0; }}
    code {{
      padding: 2px 6px;
      border-radius: 6px;
      background: var(--brand-soft);
      color: #154933;
      font-family: "JetBrains Mono", Consolas, monospace;
      font-size: 0.93em;
    }}
    pre {{
      margin: 14px 0;
      padding: 14px;
      border-radius: 12px;
      background: #152124;
      color: #e6f0f2;
      overflow-x: auto;
      border: 1px solid #26363a;
    }}
    pre code {{
      padding: 0;
      background: transparent;
      color: inherit;
      font-size: 0.9em;
    }}
    ul, ol {{ padding-left: 22px; margin: 8px 0 14px; }}
    li {{ margin: 4px 0; }}
    blockquote {{
      margin: 14px 0;
      padding: 10px 14px;
      border-left: 4px solid var(--brand);
      background: var(--brand-soft);
      color: #1f4f3c;
    }}
    a {{ color: #1f5f8c; text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    .toc {{
      margin-bottom: 18px;
      padding: 14px 16px;
      border: 1px solid var(--line);
      border-radius: 12px;
      background: #f9fcfd;
    }}
    .toc strong {{ display: block; margin-bottom: 8px; color: var(--brand); }}
    .toc ul {{ margin: 0; padding-left: 18px; }}
    .table-wrap {{
      overflow-x: auto;
      margin: 12px 0;
      border: 1px solid var(--line);
      border-radius: 12px;
      background: #fff;
    }}
    table {{ border-collapse: collapse; width: 100%; min-width: 520px; }}
    th, td {{
      border-bottom: 1px solid var(--line);
      padding: 10px 12px;
      text-align: left;
      vertical-align: top;
    }}
    thead th {{
      color: var(--brand);
      background: var(--brand-soft);
    }}
    tbody tr:last-child td {{ border-bottom: none; }}
    .meta {{
      margin-top: 20px;
      color: var(--muted);
      font-size: 13px;
      border-top: 1px dashed var(--line);
      padding-top: 12px;
    }}
    @media (max-width: 720px) {{
      .card {{ padding: 20px; }}
    }}
  </style>
</head>
<body>
  <div class="page">
    <article class="card">
      {body}
      <div class="meta">本文档已统一为 HTML 形式，便于浏览、打印和后续可视化扩展。</div>
    </article>
  </div>
</body>
</html>
"""


def update_links_in_text(text: str) -> str:
    def should_convert(path_text: str) -> bool:
        name = Path(path_text).name.lower()
        return name != "readme.md"

    def repl_markdown_link(m: re.Match[str]) -> str:
        prefix = m.group(1)
        href = m.group(2)
        suffix = m.group(3)
        if should_convert(href):
            href = href[:-3] + ".html"
        return prefix + href + suffix

    def repl_plain_path(m: re.Match[str]) -> str:
        path_text = m.group(1)
        if should_convert(path_text + ".md"):
            return path_text + ".html"
        return path_text + ".md"

    text = re.sub(r"(\[[^\]]+\]\()([^)]+\.md)(\))", repl_markdown_link, text)
    text = re.sub(r"(?<!\w)([A-Za-z0-9_./-]+)\.md(?!\w)", repl_plain_path, text)
    return text


def convert_file(md_path: Path) -> Path:
    src = md_path.read_text(encoding="utf-8")
    src = update_links_in_text(src)
    title = md_path.stem.replace("_", " ")
    body = to_html_body(src)
    html_text = wrap_html(title, body)
    target = md_path.with_suffix(".html")
    target.write_text(html_text, encoding="utf-8")
    md_path.unlink()
    return target


def update_readme_links() -> None:
    for md in ROOT.rglob("*.md"):
        if ".git" in md.parts:
            continue
        if "build" in md.parts or "install" in md.parts or "log" in md.parts:
            continue
        if not is_readme(md):
            continue
        text = md.read_text(encoding="utf-8")
        new = update_links_in_text(text)
        if new != text:
            md.write_text(new, encoding="utf-8")


def main() -> None:
    files = collect_non_readme_markdown()
    for md in files:
        convert_file(md)
    update_readme_links()


if __name__ == "__main__":
    main()
