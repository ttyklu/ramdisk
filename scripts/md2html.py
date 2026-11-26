#!/usr/bin/env python3
"""md2html.py - render the repo's Markdown docs to standalone HTML (no dependencies).

    python3 scripts/md2html.py README.md docs/compression_spec.md

Writes <name>.html next to each input. Covers the Markdown these docs use: ATX headings
(with GitHub-style anchors), paragraphs, fenced code, pipe tables, nested bullet / numbered
lists, inline code, bold, italics and links (.md links are rewritten to .html).
"""
import html, os, re, sys

CSS = """
:root { --fg:#1f2328; --bg:#fff; --mute:#59636e; --line:#d1d9e0; --code:#f6f8fa; --link:#0969da; }
@media (prefers-color-scheme: dark) {
  :root { --fg:#e6edf3; --bg:#0d1117; --mute:#9198a1; --line:#3d444d; --code:#151b23; --link:#4493f8; }
}
html { background: var(--bg); color: var(--fg); }
body { font: 15px/1.55 -apple-system, "Segoe UI", Helvetica, Arial, sans-serif;
       max-width: 980px; margin: 0 auto; padding: 32px 24px 64px; }
h1, h2, h3 { line-height: 1.25; margin: 1.6em 0 .6em; }
h1 { font-size: 1.9em; border-bottom: 1px solid var(--line); padding-bottom: .3em; margin-top: 0; }
h2 { font-size: 1.45em; border-bottom: 1px solid var(--line); padding-bottom: .25em; }
h3 { font-size: 1.15em; }
a { color: var(--link); text-decoration: none; } a:hover { text-decoration: underline; }
code, pre { font: 13px/1.45 ui-monospace, "SF Mono", Menlo, Consolas, monospace; }
code { background: var(--code); padding: .15em .35em; border-radius: 4px; }
pre { background: var(--code); padding: 14px 16px; border-radius: 6px; overflow-x: auto; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; margin: 1em 0; display: block; overflow-x: auto; }
th, td { border: 1px solid var(--line); padding: 6px 12px; text-align: left; vertical-align: top; }
th { background: var(--code); }
td { font-variant-numeric: tabular-nums; }
ul, ol { padding-left: 1.8em; } li { margin: .2em 0; }
footer { color: var(--mute); font-size: 13px; margin-top: 48px; border-top: 1px solid var(--line); padding-top: 12px; }
"""


def slug(text):
    s = re.sub(r"<[^>]+>", "", text).strip().lower()
    s = re.sub(r"[^\w\- ]", "", s)
    return s.replace(" ", "-")


def inline(t):
    parts = re.split(r"(`[^`]+`)", t)
    out = []
    for p in parts:
        if p.startswith("`") and p.endswith("`") and len(p) > 1:
            out.append("<code>" + html.escape(p[1:-1]) + "</code>")
            continue
        p = html.escape(p, quote=False)

        def link(m):
            href = re.sub(r"\.md(#|$)", r".html\1", m.group(2))
            return f'<a href="{html.escape(href)}">{m.group(1)}</a>'
        p = re.sub(r"\[([^\]]+)\]\(([^)\s]+)\)", link, p)
        p = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", p)
        p = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<em>\1</em>", p)
        out.append(p)
    return "".join(out)


ITEM = re.compile(r"^(\s*)([*-]|\d+\.)\s+(.*)$")


def render_list(lines):
    """lines of one list block -> html, nesting by indentation"""
    items = []                                   # (indent, ordered, text)
    for l in lines:
        m = ITEM.match(l)
        if m:
            items.append([len(m.group(1)), m.group(2)[0].isdigit(), m.group(3)])
        elif items:
            items[-1][2] += " " + l.strip()
    out, stack = [], []                          # stack of (indent, tag)
    for ind, ordered, text in items:
        while stack and ind < stack[-1][0]:
            out.append(f"</li></{stack.pop()[1]}>")
        if stack and ind == stack[-1][0]:
            out.append("</li>")
        else:
            tag = "ol" if ordered else "ul"
            stack.append((ind, tag))
            out.append(f"<{tag}>")
        out.append("<li>" + inline(text))
    while stack:
        out.append(f"</li></{stack.pop()[1]}>")
    return "".join(out)


def cells(row):
    row = row.strip()
    if row.startswith("|"):
        row = row[1:]
    if row.endswith("|"):
        row = row[:-1]
    return [c.strip() for c in row.split("|")]


def convert(md):
    lines = md.split("\n")
    out, i, title = [], 0, None
    while i < len(lines):
        l = lines[i]
        if l.startswith("```"):
            j = i + 1
            while j < len(lines) and not lines[j].startswith("```"):
                j += 1
            out.append("<pre><code>" + html.escape("\n".join(lines[i + 1:j])) + "</code></pre>")
            i = j + 1
            continue
        m = re.match(r"^(#{1,6})\s+(.*)$", l)
        if m:
            n, text = len(m.group(1)), inline(m.group(2))
            title = title or re.sub(r"<[^>]+>", "", text)
            out.append(f'<h{n} id="{slug(text)}">{text}</h{n}>')
            i += 1
            continue
        if l.lstrip().startswith("|") and i + 1 < len(lines) and re.match(r"^\s*\|[\s:|-]+\|\s*$", lines[i + 1]):
            head = cells(l)
            j, rows = i + 2, []
            while j < len(lines) and lines[j].lstrip().startswith("|"):
                rows.append(cells(lines[j]))
                j += 1
            t = ["<table><thead><tr>" + "".join(f"<th>{inline(c)}</th>" for c in head) + "</tr></thead><tbody>"]
            for r in rows:
                t.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>")
            out.append("".join(t) + "</tbody></table>")
            i = j
            continue
        if ITEM.match(l):
            j = i
            block = []
            while j < len(lines):
                lj = lines[j]
                if not lj.strip():
                    # a blank line ends the list unless the next line continues it
                    if j + 1 < len(lines) and (ITEM.match(lines[j + 1]) and lines[j + 1].startswith(" ")):
                        j += 1
                        continue
                    break
                if not ITEM.match(lj) and not lj.startswith(" "):
                    break
                block.append(lj)
                j += 1
            out.append(render_list(block))
            i = j
            continue
        if not l.strip():
            i += 1
            continue
        j, para = i, []
        while j < len(lines) and lines[j].strip() and not lines[j].startswith(("```", "#", "|")) \
                and not ITEM.match(lines[j]):
            para.append(lines[j].strip())
            j += 1
        out.append("<p>" + inline(" ".join(para)) + "</p>")
        i = j
    return title or "Document", "\n".join(out)


def main():
    for src in sys.argv[1:]:
        title, body = convert(open(src, encoding="utf-8").read())
        dst = os.path.splitext(src)[0] + ".html"
        with open(dst, "w", encoding="utf-8") as f:
            f.write(f"<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
                    f"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                    f"<title>{html.escape(title)}</title>\n<style>{CSS}</style>\n</head>\n<body>\n"
                    f"{body}\n<footer>Generated from <code>{html.escape(os.path.basename(src))}</code> "
                    f"by <code>scripts/md2html.py</code>.</footer>\n</body>\n</html>\n")
        print(f"{src} -> {dst}")


if __name__ == "__main__":
    main()
