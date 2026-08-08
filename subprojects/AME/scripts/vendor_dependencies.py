#!/usr/bin/env python3
"""Copy AME's third-party dependencies into subprojects/AME/external.

AME is built in air-gapped environments, where nothing can be downloaded at
configure time. Its dependencies are therefore checked into the repository the
way PCL and PYRAMID already are, and this script is what puts them there.

Only the files AME actually compiles or includes are kept. Upstream test
suites, documentation, demo programs and ports for platforms AME does not
target are left out, which is the difference between roughly 36 MB and roughly
190 MB. Each dependency below says what is kept and why, so that the next
person to update one can see whether their new files fall inside the slice.

Usage:

    # Take the sources from a build tree that has already fetched them.
    python3 vendor_dependencies.py --from-build-dir ../../../build-authoring

    # Or let CMake fetch them into a scratch directory first.
    python3 vendor_dependencies.py --fetch

    # Check the checked-in copies match what the manifest says, and change
    # nothing. This is what continuous integration runs.
    python3 vendor_dependencies.py --verify

Run it from anywhere; paths are worked out from this file's location.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

AME_ROOT = Path(__file__).resolve().parent.parent
EXTERNAL = AME_ROOT / "external"
MANIFEST = EXTERNAL / "manifest.json"


@dataclass
class Dependency:
    """One third-party library, and the slice of it AME keeps."""

    #: Directory name under external/.
    name: str
    #: Name FetchContent gives its source directory under _deps.
    fetch_name: str
    repository: str
    tag: str
    #: Why AME needs it, in one line, for the external/README.md table.
    purpose: str
    #: Directories copied whole, relative to the upstream root.
    directories: list[str] = field(default_factory=list)
    #: Individual files copied, relative to the upstream root.
    files: list[str] = field(default_factory=list)
    #: Copy every regular file sitting at the upstream root. Use this when a
    #: dependency's own CMakeLists reads templates beside it and listing them
    #: one at a time would be a guessing game.
    top_level_files: bool = False
    #: Glob patterns removed after copying, relative to the vendored root.
    prune: list[str] = field(default_factory=list)
    #: Set when the copy is maintained by hand rather than from an upstream
    #: checkout, so --fetch and --from-build-dir leave it alone.
    manual: bool = False


DEPENDENCIES: list[Dependency] = [
    Dependency(
        name="nlohmann_json",
        fetch_name="",
        repository="https://github.com/nlohmann/json",
        tag="v3.11.3",
        purpose="JSON for the authoring project format, run records and reports",
        manual=True,
    ),
    Dependency(
        name="behaviortree_cpp",
        fetch_name="behaviortree_cpp",
        repository="https://github.com/BehaviorTree/BehaviorTree.CPP.git",
        tag="4.6.2",
        purpose="Behaviour tree engine that runs compiled plans",
        directories=["src", "include", "3rdparty", "cmake"],
        files=["CMakeLists.txt", "LICENSE"],
        # The upstream build only compiles these when its own options are on,
        # and AME turns all of them off.
        prune=["examples", "tests", "sample_nodes", "docs"],
    ),
    Dependency(
        name="lapkt",
        fetch_name="lapkt_src",
        repository="https://github.com/LAPKT-dev/LAPKT-public.git",
        tag="Devel2.0",
        purpose="Classical planner used by ame_core's Planner",
        # AME compiles seventeen .cxx files out of this tree and includes from
        # seven directories; everything else upstream ships (external solvers,
        # Python bindings, benchmark problems) is unused. Keeping src alone
        # takes it from 93 MB to about 1 MB.
        directories=["src"],
        files=["LICENSE.txt"],
        prune=["src/**/*.py", "src/interfaces"],
    ),
    Dependency(
        name="googletest",
        fetch_name="googletest",
        repository="https://github.com/google/googletest.git",
        tag="v1.14.0",
        purpose="Test framework for every AME test binary",
        directories=["googletest"],
        files=["CMakeLists.txt", "LICENSE"],
        # AME sets BUILD_GMOCK=OFF, so the mock library is never configured.
        prune=["googletest/test", "googletest/samples", "googletest/docs"],
    ),
    Dependency(
        name="imgui",
        fetch_name="imgui",
        repository="https://github.com/ocornut/imgui",
        tag="v1.91.6-docking",
        purpose="Immediate-mode user interface for the authoring tool",
        files=[
            "imgui.cpp",
            "imgui.h",
            "imgui_draw.cpp",
            "imgui_tables.cpp",
            "imgui_widgets.cpp",
            "imgui_demo.cpp",
            "imgui_internal.h",
            "imstb_rectpack.h",
            "imstb_textedit.h",
            "imstb_truetype.h",
            "imconfig.h",
            "LICENSE.txt",
            # Only the two backends the tool builds against.
            "backends/imgui_impl_sdl2.cpp",
            "backends/imgui_impl_sdl2.h",
            "backends/imgui_impl_opengl3.cpp",
            "backends/imgui_impl_opengl3.h",
            "backends/imgui_impl_opengl3_loader.h",
        ],
    ),
    Dependency(
        name="imgui_node_editor",
        fetch_name="imgui_node_editor",
        repository="https://github.com/thedmd/imgui-node-editor",
        tag="develop",
        purpose="Canvas widget behind the domain, plan and behaviour-tree views",
        files=[
            "imgui_node_editor.cpp",
            "imgui_node_editor.h",
            "imgui_node_editor_api.cpp",
            "imgui_node_editor_internal.h",
            "imgui_node_editor_internal.inl",
            "imgui_canvas.cpp",
            "imgui_canvas.h",
            "imgui_bezier_math.h",
            "imgui_bezier_math.inl",
            "imgui_extra_math.h",
            "imgui_extra_math.inl",
            "crude_json.cpp",
            "crude_json.h",
            "LICENSE",
        ],
    ),
    Dependency(
        name="sdl2",
        fetch_name="sdl2",
        repository="https://github.com/libsdl-org/SDL",
        tag="release-2.30.10",
        purpose="Window, input and OpenGL context for the authoring tool",
        directories=["src", "include", "cmake", "wayland-protocols"],
        # SDL2's CMakeLists reads several of the templates beside it, so the
        # whole of the top level comes across. It is a few hundred kilobytes.
        top_level_files=True,
        # Upstream ships tests, docs and Xcode/Android/VisualC project files
        # that no CMake build reads.
        prune=[
            "test",
            "docs",
            "Xcode*",
            "VisualC*",
            "android-project",
            "src/hidapi/**/*.sln",
        ],
    ),
    Dependency(
        name="stb",
        fetch_name="stb",
        repository="https://github.com/nothings/stb",
        tag="5736b15f7ea0ffb08dd38af21067c314d6a3aae9",
        purpose="Writing the self-test screenshot as a PNG",
        files=["stb_image_write.h", "LICENSE"],
    ),
    Dependency(
        name="tinyfiledialogs",
        fetch_name="tinyfiledialogs",
        repository="https://git.code.sf.net/p/tinyfiledialogs/code",
        tag="master",
        purpose="Native open and save dialogs",
        files=["tinyfiledialogs.c", "tinyfiledialogs.h", "README.txt"],
    ),
    Dependency(
        name="jetbrains_mono",
        fetch_name="jetbrains_mono",
        repository="https://github.com/JetBrains/JetBrainsMono",
        tag="v2.304",
        purpose="The authoring tool's interface font",
        # One weight is loaded at runtime. The upstream repository carries every
        # weight, the web fonts and the design sources, which is 21 MB for a
        # 200 KB file.
        files=["fonts/ttf/JetBrainsMono-Regular.ttf", "OFL.txt"],
    ),
    Dependency(
        name="websocketpp",
        fetch_name="websocketpp",
        repository="https://github.com/zaphoyd/websocketpp.git",
        tag="0.8.2",
        purpose="Foxglove bridge WebSocket server (only when AME_FOXGLOVE is on)",
        directories=["websocketpp"],
        files=["COPYING"],
    ),
    Dependency(
        name="asio",
        fetch_name="asio",
        repository="https://github.com/chriskohlhoff/asio.git",
        tag="asio-1-28-0",
        purpose="Standalone Asio, which websocketpp needs",
        directories=["asio/include"],
        files=["asio/LICENSE_1_0.txt"],
        prune=["asio/include/**/*.am"],
    ),
]


def log(message: str) -> None:
    print(f"[vendor] {message}", flush=True)


def prune_paths(root: Path, patterns: list[str]) -> None:
    for pattern in patterns:
        for path in sorted(root.glob(pattern), reverse=True):
            if path.is_dir():
                shutil.rmtree(path, ignore_errors=True)
            elif path.exists():
                path.unlink()


def copy_dependency(dep: Dependency, source_root: Path) -> None:
    target = EXTERNAL / dep.name
    if target.exists():
        shutil.rmtree(target)
    target.mkdir(parents=True)

    for directory in dep.directories:
        src = source_root / directory
        if not src.is_dir():
            raise SystemExit(f"{dep.name}: expected directory {src}")
        shutil.copytree(src, target / directory, symlinks=True)

    names = list(dep.files)
    if dep.top_level_files:
        names += [p.name for p in source_root.iterdir() if p.is_file()]

    for name in dict.fromkeys(names):
        src = source_root / name
        if not src.is_file():
            raise SystemExit(f"{dep.name}: expected file {src}")
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, destination)

    prune_paths(target, dep.prune)
    # Upstream version-control metadata is never wanted; the manifest records
    # which commit each copy came from instead.
    prune_paths(target, [".git", ".github", "**/.git"])


def tree_digest(root: Path) -> str:
    """A digest over every file in a vendored copy, so drift is detectable."""
    digest = hashlib.sha256()
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def write_manifest() -> None:
    entries = []
    for dep in DEPENDENCIES:
        target = EXTERNAL / dep.name
        if not target.is_dir():
            continue
        files = [p for p in target.rglob("*") if p.is_file()]
        entries.append(
            {
                "name": dep.name,
                "repository": dep.repository,
                "tag": dep.tag,
                "purpose": dep.purpose,
                "files": len(files),
                "bytes": sum(p.stat().st_size for p in files),
                "sha256": tree_digest(target),
            }
        )
    MANIFEST.write_text(json.dumps({"dependencies": entries}, indent=2) + "\n")
    log(f"wrote {MANIFEST.relative_to(AME_ROOT)}")


def verify() -> int:
    if not MANIFEST.is_file():
        log("no manifest; run this script without --verify first")
        return 1
    recorded = {d["name"]: d for d in json.loads(MANIFEST.read_text())["dependencies"]}
    problems = 0
    for dep in DEPENDENCIES:
        target = EXTERNAL / dep.name
        if not target.is_dir():
            log(f"MISSING {dep.name}")
            problems += 1
            continue
        if dep.name not in recorded:
            log(f"NOT IN MANIFEST {dep.name}")
            problems += 1
            continue
        if tree_digest(target) != recorded[dep.name]["sha256"]:
            log(f"CHANGED {dep.name}")
            problems += 1
    if problems == 0:
        log(f"all {len(DEPENDENCIES)} vendored dependencies match the manifest")
    return 1 if problems else 0


def fetch_into(scratch: Path) -> Path:
    """Clone each dependency at its pinned tag into a scratch directory."""
    scratch.mkdir(parents=True, exist_ok=True)
    for dep in DEPENDENCIES:
        if dep.manual:
            continue
        destination = scratch / dep.fetch_name
        if destination.is_dir():
            continue
        log(f"cloning {dep.name} at {dep.tag}")
        subprocess.run(
            [
                "git", "clone", "--depth", "1", "--branch", dep.tag,
                dep.repository, str(destination),
            ],
            check=True,
        )
    return scratch


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--from-build-dir",
        type=Path,
        help="A build tree whose _deps directory already holds the sources",
    )
    group.add_argument(
        "--fetch", action="store_true",
        help="Clone each dependency at its pinned tag first (needs network)",
    )
    group.add_argument(
        "--verify", action="store_true",
        help="Check the checked-in copies against the manifest and change nothing",
    )
    parser.add_argument(
        "--scratch", type=Path, default=Path("/tmp/ame-vendor-sources"),
        help="Where --fetch clones to",
    )
    args = parser.parse_args()

    if args.verify:
        return verify()

    if args.fetch:
        source_base = fetch_into(args.scratch)
        suffix = ""
    else:
        source_base = args.from_build_dir.resolve() / "_deps"
        if not source_base.is_dir():
            raise SystemExit(f"no _deps directory under {args.from_build_dir}")
        suffix = "-src"

    EXTERNAL.mkdir(exist_ok=True)
    for dep in DEPENDENCIES:
        if dep.manual:
            log(f"{dep.name}: maintained by hand, left alone")
            continue
        source_root = source_base / f"{dep.fetch_name}{suffix}"
        if not source_root.is_dir():
            raise SystemExit(f"{dep.name}: no source at {source_root}")
        log(f"vendoring {dep.name}")
        copy_dependency(dep, source_root)

    write_manifest()
    total = sum(
        p.stat().st_size for p in EXTERNAL.rglob("*") if p.is_file()
    )
    log(f"external/ is now {total / (1024 * 1024):.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
