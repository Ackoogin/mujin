# Roadmap dashboards

This directory turns roadmap data into standalone HTML dashboards. Each
dashboard shows every roadmap item with its title, description, rationale,
and progress, and lets a reader filter by state, search the text, and see at
a glance how much of each stream is finished.

There are three files:

| File | What it is |
|------|-----------|
| `roadmap_template.html` | The dashboard itself: layout, styles, and the code that draws the page. Edit this to change how dashboards look. |
| `generate_roadmap.py` | Validates roadmap files and writes one HTML page per roadmap, plus an index. Standard library only, except that YAML input needs PyYAML. |
| `README.md` | This file. |

The roadmap data lives in [`doc/roadmap/`](../../doc/roadmap/), and the
generated pages are written to `doc/roadmap/site/`.

## The update flow

1. Edit the roadmap file, for example `doc/roadmap/ame.yaml`.
2. Regenerate:

   ```bat
   python3 tools/roadmap/generate_roadmap.py
   ```

3. Open `doc/roadmap/site/index.html` in a browser. The pages are entirely
   self-contained, so this works from a local clone with no web server and
   no network access.
4. Commit both the roadmap file and the regenerated pages, so anyone with a
   clone can open the dashboard without running the generator.

To check the data without writing any files, which is what a continuous
integration job should run:

```bat
python3 tools/roadmap/generate_roadmap.py --check
```

Other options:

| Option | Effect |
|--------|--------|
| `-o DIR`, `--output DIR` | Write pages somewhere other than `doc/roadmap/site/` |
| `--check` | Validate only; write nothing. Exits non-zero on any problem |
| `--no-index` | Skip the index page |
| `--body-only` | Emit the styles and body without the surrounding page skeleton, for embedding a dashboard in another page |

Positional arguments are roadmap files or directories. With none given, the
generator reads every `.yaml`, `.yml`, and `.json` file in `doc/roadmap/`.

## Editing the template directly

`roadmap_template.html` is also a working dashboard on its own. It carries a
small example roadmap in a `<script id="roadmap-data">` block near the
bottom, between these two markers:

```html
<!-- ROADMAP-DATA:BEGIN -->
<!-- ROADMAP-DATA:END -->
```

The generator replaces everything between those markers and changes nothing
else, so you can restyle the page freely as long as the markers stay put.
Two other marker pairs, `HEAD-STYLE` and `BODY`, are what `--body-only`
uses; leave those in place as well.

If you would rather keep a dashboard as a single hand-edited file than run
the generator, that works too: copy the template, replace the JSON block
with your own data, and edit it in place. You lose the validation the
generator does, so check the browser console if something does not appear.

## The data format

A roadmap file is a mapping with a title and a list of streams, and each
stream holds a list of items.

```yaml
title: AME — Autonomous Mission Engine
subtitle: One or two sentences on what this roadmap covers.
updated: "2026-08-01"
source: doc/todo/AME/TODO.md      # where the underlying detail lives

streams:
  - code: PCL                      # short label shown in the filter chips
    name: PCL integration compliance
    summary: One sentence of context for the stream.
    items:
      - id: PCL-1                  # unique within the roadmap; used for deep links
        title: Fix the ExecutorNode ingress race
        description: >-
          What the work is, in plain sentences.
        rationale: >-
          Why it is worth doing, and what goes wrong if it is skipped.
        progress: 0                # 0 to 100
        status: at-risk            # optional; see below
        effort: Small              # optional
        owner: Unassigned          # optional
        target: 2026 Q4            # optional
        tags: [safety]             # optional; searchable
        links:                     # optional
          - label: Migration plan
            url: ../../plans/AME/pcl_migration_plan.md
```

Every item must have `id`, `title`, `description`, `rationale`, and
`progress`. The generator reports all missing or malformed fields at once,
naming the stream and item, rather than stopping at the first one.

Text in the `description`, `rationale`, and stream `summary` fields may mark
identifiers with `` `backticks` ``; they are drawn as inline code. Nothing
else in those fields is interpreted as markup.

### States

An item's state decides its colour, its pill, and which filter it answers
to. Set `status` only when progress alone does not tell the story;
otherwise it is derived:

| Status | When it applies | Derived from progress? |
|--------|-----------------|------------------------|
| `done` | Finished | Yes, at 100 |
| `active` | Under way | Yes, above 0 |
| `planned` | Agreed but not started | Yes, at 0 |
| `at-risk` | Under way or waiting, and something is wrong | No — set it yourself |
| `blocked` | Cannot proceed | No — set it yourself |

Finished and unstarted work is drawn in neutral greys, unstarted work with a
hatch, so the colour on the page belongs to work that is live or in trouble.

### How progress is calculated

The overall figure and each stream figure are the plain mean of the progress
of the items involved, so every item counts the same regardless of size. The
page says so in its footer. `progress` is a number a person maintains; the
generator does not infer it from anything.

## Adding a new dashboard

Add a `.yaml` file to `doc/roadmap/` and regenerate. The file name becomes
the page name, so `doc/roadmap/pyramid.yaml` produces
`doc/roadmap/site/pyramid.html`, and the index picks it up automatically.

## Design notes

The palette is inherited from the AME Dev Environment theme in
[`subprojects/AME/tools/devenv/ui/theme.py`](../../subprojects/AME/tools/devenv/ui/theme.py):
a near-black ground, a vivid cyan accent, panels separated by a background
lift rather than borders, and no corner rounding. Light mode is the
printed-briefing counterpart of the same palette, and is what the page
prints in.

Both themes are defined as custom properties at the top of the template's
style block, so components never name a colour directly. The page follows
the reader's system preference and the theme button overrides it.
