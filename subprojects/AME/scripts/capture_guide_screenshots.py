#!/usr/bin/env python3
"""Take the screenshots used by the authoring tool user guide.

The pictures in `doc/guides/authoring_tool_user_guide.md` are the interface
itself, not drawings of it, so they have to be retaken whenever the interface
changes. This script drives `ame_authoring_tool --capture`, which opens a saved
project, walks the whole workflow, and writes one PNG per step.

The tool needs no display: on Linux, SDL's offscreen video driver is used, which
is set for you below. On Windows the window is created hidden.

Usage:

    python subprojects/AME/scripts/capture_guide_screenshots.py \
        --tool build/src/ame_authoring_tool

With no --tool, the usual build directories are searched. With no --out, the
pictures land in `subprojects/AME/doc/guides/images/authoring/`.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

AME_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = AME_ROOT / "doc" / "guides" / "images" / "authoring"
DEFAULT_PROJECT = AME_ROOT / "domains" / "first_survey" / "first-survey.ameproj.json"

# One entry per run of the tool. The sets differ only in window size: the
# workflow screens want a wide window, the sentence editor a tall one, and the
# diagnostics page a very tall one, because each is photographed whole rather
# than scrolled.
CAPTURE_SETS = [
    ("workflow", "1600x1000"),
    ("editor", "1100x2000"),
    ("reports", "1500x2600"),
]

CANDIDATE_TOOLS = [
    AME_ROOT / "build" / "src" / "ame_authoring_tool",
    AME_ROOT / "build" / "src" / "Release" / "ame_authoring_tool.exe",
    AME_ROOT.parent.parent / "build-authoring" / "subprojects" / "AME" / "src" /
    "ame_authoring_tool",
    AME_ROOT.parent.parent / "build-authoring" / "subprojects" / "AME" / "src" /
    "Release" / "ame_authoring_tool.exe",
]


def find_tool(explicit):
    if explicit:
        tool = Path(explicit).resolve()
        if not tool.exists():
            sys.exit(f"no authoring tool at {tool}")
        return tool
    for candidate in CANDIDATE_TOOLS:
        if candidate.exists():
            return candidate
    sys.exit(
        "the authoring tool was not found. Build it first:\n"
        "  cmake --preset authoring\n"
        "  cmake --build --preset authoring-release --target ame_authoring_tool\n"
        "then pass --tool <path> if it is somewhere unusual.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tool", help="path to ame_authoring_tool")
    parser.add_argument("--project", default=str(DEFAULT_PROJECT),
                        help="the .ameproj.json to photograph")
    parser.add_argument("--out", default=str(DEFAULT_OUT),
                        help="where the PNG files are written")
    args = parser.parse_args()

    tool = find_tool(args.tool)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    environment = dict(os.environ)
    if sys.platform.startswith("linux"):
        environment.setdefault("SDL_VIDEODRIVER", "offscreen")

    captured = []
    # The graph canvas saves its scroll position in NodeEditor.json beside the
    # working directory. Running from a temporary directory keeps that file out
    # of the repository, and means every run starts from the same view.
    with tempfile.TemporaryDirectory() as working_directory:
        for name, size in CAPTURE_SETS:
            result = subprocess.run(
                [str(tool), "--capture", str(out.resolve()),
                 "--project", str(Path(args.project).resolve()),
                 "--set", name, "--size", size],
                cwd=working_directory, env=environment,
                capture_output=True, text=True)
            if result.returncode != 0:
                sys.stderr.write(result.stdout)
                sys.stderr.write(result.stderr)
                sys.exit(f"the {name} screenshots could not be taken")
            report = json.loads(result.stdout)
            captured.extend(report["screenshots"])

    manifest = out / "screenshots.json"
    manifest.write_text(json.dumps(captured, indent=2) + "\n")
    for shot in captured:
        print(f"{shot['file']}: {shot['caption']}")
    print(f"{len(captured)} screenshots in {out}")
    if shutil.which("optipng"):
        print("tip: optipng *.png makes these smaller before committing")


if __name__ == "__main__":
    main()
