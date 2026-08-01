#!/usr/bin/env python3
"""Turn roadmap data files into standalone HTML dashboards.

A roadmap lives in one YAML (or JSON) file under ``doc/roadmap/``. This
script validates that file and writes a single self-contained HTML page
that needs no web server, no network access, and no build step -- open it
from disk and it works.

The page itself is ``roadmap_template.html`` in this directory. The script
only replaces the data block inside it, so the whole visual design can be
edited in that one file without touching any Python.

Typical use::

    python3 tools/roadmap/generate_roadmap.py

which reads every roadmap file in ``doc/roadmap/`` and writes the pages
plus an index into ``doc/roadmap/site/``.

Validate without writing anything (useful in CI)::

    python3 tools/roadmap/generate_roadmap.py --check

See ``tools/roadmap/README.md`` for the data format and the update flow.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
DEFAULT_SOURCE_DIR = REPO_ROOT / "doc" / "roadmap"
DEFAULT_OUTPUT_DIR = DEFAULT_SOURCE_DIR / "site"
TEMPLATE_PATH = HERE / "roadmap_template.html"

DATA_BEGIN = "<!-- ROADMAP-DATA:BEGIN -->"
DATA_END = "<!-- ROADMAP-DATA:END -->"
STYLE_BEGIN = "<!-- HEAD-STYLE:BEGIN -->"
STYLE_END = "<!-- HEAD-STYLE:END -->"
BODY_BEGIN = "<!-- BODY:BEGIN -->"
BODY_END = "<!-- BODY:END -->"

VALID_STATUSES = ("planned", "active", "done", "at-risk", "blocked")
REQUIRED_ITEM_FIELDS = ("id", "title", "description", "rationale", "progress")
OPTIONAL_ITEM_FIELDS = ("status", "owner", "effort", "target", "tags", "links")


class RoadmapError(Exception):
    """Raised when a roadmap file cannot be read or does not validate."""


@dataclass
class Roadmap:
    """A validated roadmap, ready to be written out as a page."""

    path: Path
    data: dict[str, Any]
    slug: str
    items: list[dict[str, Any]] = field(default_factory=list)

    @property
    def title(self) -> str:
        return str(self.data.get("title", self.slug))

    @property
    def subtitle(self) -> str:
        return str(self.data.get("subtitle", ""))

    @property
    def updated(self) -> str:
        return str(self.data.get("updated", ""))

    @property
    def overall(self) -> int:
        if not self.items:
            return 0
        return round(sum(float(i["progress"]) for i in self.items) / len(self.items))

    def count_by_state(self) -> dict[str, int]:
        counts = {state: 0 for state in VALID_STATUSES}
        for item in self.items:
            counts[derive_status(item)] += 1
        return counts


def derive_status(item: dict[str, Any]) -> str:
    """Return the item's state, deriving it from progress when unset."""
    status = item.get("status")
    if status in VALID_STATUSES:
        return str(status)
    progress = float(item.get("progress", 0))
    if progress >= 100:
        return "done"
    return "active" if progress > 0 else "planned"


def load_data_file(path: Path) -> dict[str, Any]:
    """Read a roadmap file. YAML needs PyYAML; JSON needs nothing."""
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".yaml", ".yml"):
        try:
            import yaml
        except ImportError as exc:
            raise RoadmapError(
                f"{path.name}: reading YAML needs PyYAML. Install it with "
                "'pip install pyyaml', or write the roadmap as .json instead."
            ) from exc
        try:
            loaded = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise RoadmapError(f"{path.name}: could not parse YAML -- {exc}") from exc
    else:
        try:
            loaded = json.loads(text)
        except json.JSONDecodeError as exc:
            raise RoadmapError(f"{path.name}: could not parse JSON -- {exc}") from exc

    if not isinstance(loaded, dict):
        raise RoadmapError(f"{path.name}: the file must contain a mapping at the top level.")
    return loaded


def validate(path: Path, data: dict[str, Any]) -> list[dict[str, Any]]:
    """Check one roadmap and return its flattened item list.

    Every problem found is reported together, so a single run tells the
    author everything that needs fixing.
    """
    problems: list[str] = []
    items: list[dict[str, Any]] = []

    if not str(data.get("title", "")).strip():
        problems.append("the roadmap needs a 'title'.")

    streams = data.get("streams")
    if not isinstance(streams, list) or not streams:
        problems.append("the roadmap needs a non-empty 'streams' list.")
        streams = []

    seen_ids: dict[str, str] = {}
    for stream_index, stream in enumerate(streams):
        where = f"stream {stream_index + 1}"
        if not isinstance(stream, dict):
            problems.append(f"{where}: each stream must be a mapping.")
            continue
        if not str(stream.get("name", "")).strip():
            problems.append(f"{where}: needs a 'name'.")
        code = str(stream.get("code", "")).strip()
        if code:
            where = f"stream '{code}'"

        stream_items = stream.get("items")
        if not isinstance(stream_items, list) or not stream_items:
            problems.append(f"{where}: needs a non-empty 'items' list.")
            continue

        for item_index, item in enumerate(stream_items):
            item_where = f"{where}, item {item_index + 1}"
            if not isinstance(item, dict):
                problems.append(f"{item_where}: each item must be a mapping.")
                continue

            item_id = str(item.get("id", "")).strip()
            if item_id:
                item_where = f"{where}, item '{item_id}'"
                if item_id in seen_ids:
                    problems.append(
                        f"{item_where}: this id is already used in {seen_ids[item_id]}. "
                        "Item ids must be unique within a roadmap."
                    )
                seen_ids[item_id] = where

            for required in REQUIRED_ITEM_FIELDS:
                value = item.get(required)
                if required == "progress":
                    if value is None:
                        problems.append(f"{item_where}: needs 'progress' (a number from 0 to 100).")
                    elif not isinstance(value, (int, float)) or isinstance(value, bool):
                        problems.append(f"{item_where}: 'progress' must be a number, not {value!r}.")
                    elif not 0 <= float(value) <= 100:
                        problems.append(f"{item_where}: 'progress' is {value}; it must be 0 to 100.")
                elif not str(value or "").strip():
                    problems.append(f"{item_where}: needs a '{required}'.")

            status = item.get("status")
            if status is not None and status not in VALID_STATUSES:
                problems.append(
                    f"{item_where}: status {status!r} is not one of "
                    f"{', '.join(VALID_STATUSES)}."
                )

            links = item.get("links", [])
            if links and not isinstance(links, list):
                problems.append(f"{item_where}: 'links' must be a list of label/url mappings.")
            elif isinstance(links, list):
                for link in links:
                    if not isinstance(link, dict) or "url" not in link:
                        problems.append(f"{item_where}: every link needs at least a 'url'.")

            unknown = set(item) - set(REQUIRED_ITEM_FIELDS) - set(OPTIONAL_ITEM_FIELDS)
            if unknown:
                problems.append(
                    f"{item_where}: unknown field(s) {', '.join(sorted(unknown))}. "
                    f"Known fields are {', '.join(REQUIRED_ITEM_FIELDS + OPTIONAL_ITEM_FIELDS)}."
                )

            items.append(item)

    if problems:
        listed = "\n".join(f"  - {p}" for p in problems)
        raise RoadmapError(f"{path.name} has {len(problems)} problem(s):\n{listed}")

    return items


def display_path(path: Path) -> str:
    """Return the path relative to the repository when it sits inside it."""
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT)).replace("\\", "/")
    except ValueError:
        return str(resolved)


def load_roadmap(path: Path) -> Roadmap:
    data = load_data_file(path)
    items = validate(path, data)
    data.setdefault("source", display_path(path))
    return Roadmap(path=path, data=data, slug=path.stem, items=items)


def embed_json(data: dict[str, Any]) -> str:
    """Serialise data for safe inlining in a <script> element."""
    payload = json.dumps(data, indent=2, ensure_ascii=False)
    return payload.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")


def section(template: str, begin: str, end: str, what: str) -> str:
    start = template.find(begin)
    stop = template.find(end)
    if start == -1 or stop == -1 or stop < start:
        raise RoadmapError(
            f"{TEMPLATE_PATH.name} is missing its {what} markers "
            f"({begin} ... {end}). Restore them before generating."
        )
    return template[start + len(begin):stop]


def render_page(template: str, roadmap: Roadmap, body_only: bool) -> str:
    """Return the finished HTML for one roadmap."""
    block = (
        '\n<script id="roadmap-data" type="application/json">\n'
        + embed_json(roadmap.data)
        + "\n</script>\n"
    )
    start = template.find(DATA_BEGIN)
    stop = template.find(DATA_END)
    if start == -1 or stop == -1:
        raise RoadmapError(
            f"{TEMPLATE_PATH.name} is missing its data markers "
            f"({DATA_BEGIN} ... {DATA_END}). Restore them before generating."
        )
    page = template[: start + len(DATA_BEGIN)] + block + template[stop:]

    if not body_only:
        return page

    style = section(page, STYLE_BEGIN, STYLE_END, "style")
    body = section(page, BODY_BEGIN, BODY_END, "body")
    return f"<title>{roadmap.title} — roadmap</title>\n{style}\n{body}"


def render_index(template: str, roadmaps: list[Roadmap]) -> str:
    """Return a small landing page linking every generated dashboard."""
    style = section(template, STYLE_BEGIN, STYLE_END, "style")
    cards = []
    for roadmap in roadmaps:
        counts = roadmap.count_by_state()
        attention = counts["at-risk"] + counts["blocked"]
        cards.append(
            f"""      <li class="item" style="--state-hue: var(--state-active); --gauge-hue: var(--accent)">
        <div class="item__mark">
          <span class="item__id">{escape(roadmap.slug.upper())}</span>
        </div>
        <div class="item__body">
          <h3 class="item__title"><a href="{escape(roadmap.slug)}.html">{escape(roadmap.title)}</a></h3>
          <p class="item__desc">{escape(roadmap.subtitle)}</p>
          <div class="item__facts" style="margin-top:12px">
            <span>Items <b>{len(roadmap.items)}</b></span>
            <span>Complete <b>{counts['done']}</b></span>
            <span>In flight <b>{counts['active']}</b></span>
            <span>Needs attention <b>{attention}</b></span>
          </div>
        </div>
        <div class="item__gauge">
          <span class="item__pct">{roadmap.overall}<span>%</span></span>
          <div class="gauge" style="--pct:{roadmap.overall}%"><div class="gauge__fill"></div></div>
          <div class="item__facts"><span>Updated <b>{escape(roadmap.updated or 'not stated')}</b></span></div>
        </div>
      </li>"""
        )

    joined = "\n".join(cards)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Roadmaps</title>
{style}
</head>
<body>
<header class="masthead">
  <div class="shell masthead__inner">
    <div>
      <h1 class="masthead__title">Roadmaps</h1>
      <p class="masthead__sub">Each dashboard is generated from a roadmap file in
        <code>doc/roadmap/</code>. Progress figures are the plain mean of item progress.</p>
    </div>
    <div class="masthead__meta">
      <span>Dashboards <b>{len(roadmaps)}</b></span>
      <button class="themetoggle" type="button" onclick="
        var r=document.documentElement,
            d=r.getAttribute('data-theme')
              ? r.getAttribute('data-theme')==='dark'
              : matchMedia('(prefers-color-scheme: dark)').matches;
        r.setAttribute('data-theme', d ? 'light' : 'dark');">Switch theme</button>
    </div>
  </div>
</header>
<main class="shell">
  <div class="streams">
    <ol class="items" style="margin-top:28px">
{joined}
    </ol>
  </div>
</main>
</body>
</html>
"""


def escape(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def collect_sources(inputs: Iterable[Path]) -> list[Path]:
    found: list[Path] = []
    for item in inputs:
        if item.is_dir():
            for suffix in ("*.yaml", "*.yml", "*.json"):
                found.extend(sorted(item.glob(suffix)))
        elif item.exists():
            found.append(item)
        else:
            raise RoadmapError(f"{item} does not exist.")
    return found


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate roadmap dashboards from roadmap data files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        default=[DEFAULT_SOURCE_DIR],
        help=f"roadmap files or directories (default: {DEFAULT_SOURCE_DIR})",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"directory to write pages into (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate the roadmap files and write nothing",
    )
    parser.add_argument(
        "--no-index",
        action="store_true",
        help="skip the index page that links the dashboards",
    )
    parser.add_argument(
        "--body-only",
        action="store_true",
        help="emit the style and body without the surrounding page skeleton, "
        "for embedding the dashboard in another page",
    )
    args = parser.parse_args(argv)

    try:
        sources = collect_sources(args.inputs)
        if not sources:
            raise RoadmapError(
                f"No roadmap files found in {', '.join(str(i) for i in args.inputs)}. "
                "Add a .yaml or .json file describing the roadmap."
            )

        template = TEMPLATE_PATH.read_text(encoding="utf-8")
        roadmaps = [load_roadmap(path) for path in sources]

        if args.check:
            for roadmap in roadmaps:
                print(
                    f"ok  {roadmap.path.name}: {len(roadmap.items)} items, "
                    f"{roadmap.overall}% overall"
                )
            return 0

        args.output.mkdir(parents=True, exist_ok=True)
        for roadmap in roadmaps:
            suffix = ".body.html" if args.body_only else ".html"
            out = args.output / (roadmap.slug + suffix)
            out.write_text(render_page(template, roadmap, args.body_only), encoding="utf-8")
            print(f"wrote {display_path(out)}  ({len(roadmap.items)} items, {roadmap.overall}%)")

        if not args.no_index and not args.body_only:
            index = args.output / "index.html"
            index.write_text(render_index(template, roadmaps), encoding="utf-8")
            print(f"wrote {display_path(index)}  ({len(roadmaps)} dashboards)")

    except RoadmapError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
