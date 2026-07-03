#!/usr/bin/env python3
"""Byte-identical generation baseline helper for the codegen refactor.

Generates the two reference trees the regression bar in
doc/plans/PYRAMID/generator_refactor_plan.md requires:

  * ``legacy``  -- ``proto/`` tree, ``--contract-layout pyramid``, all backends
  * ``generic`` -- ``pim/test/`` tree, ``--contract-layout generic``, all backends

Usage:
    # Capture a baseline (typically once, from main, before refactoring):
    python3 tests/generation_baseline.py generate <baseline-dir>

    # Re-generate with the current working tree and compare:
    python3 tests/generation_baseline.py check <baseline-dir> [<scratch-dir>]

``check`` exits non-zero and prints the differing paths if the current
generator output is not byte-identical to the baseline.

An alternative generator checkout (e.g. a git worktree pinned at the
baseline commit) can be selected with ``--pyramid-root``.
"""

from __future__ import annotations

import argparse
import filecmp
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PYRAMID_ROOT = Path(__file__).resolve().parents[1]

TREES = {
    'legacy': ('proto', 'pyramid'),
    'generic': ('pim/test', 'generic'),
}


def generate_tree(name: str, out_root: Path,
                  pyramid_root: Path = PYRAMID_ROOT) -> Path:
    proto_rel, layout = TREES[name]
    out_dir = out_root / name
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    cmd = [
        sys.executable,
        str(pyramid_root / 'pim' / 'generate_bindings.py'),
        str(pyramid_root / proto_rel),
        str(out_dir),
        '--contract-layout', layout,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise RuntimeError(f'generation failed for tree {name!r}')
    return out_dir


def generate_all(out_root: Path, pyramid_root: Path = PYRAMID_ROOT) -> None:
    for name in TREES:
        generate_tree(name, out_root, pyramid_root)


def diff_dirs(baseline: Path, candidate: Path) -> list[str]:
    """Recursively compare two dirs; return human-readable differences."""
    problems: list[str] = []

    def walk(cmp: filecmp.dircmp) -> None:
        for missing in cmp.left_only:
            problems.append(f'only in baseline: {cmp.left}/{missing}')
        for extra in cmp.right_only:
            problems.append(f'only in candidate: {cmp.right}/{extra}')
        # dircmp default compare is shallow (stat); force content compare.
        (_, mismatch, errors) = filecmp.cmpfiles(
            cmp.left, cmp.right, cmp.common_files, shallow=False)
        for name in mismatch:
            problems.append(f'differs: {cmp.left}/{name}')
        for name in errors:
            problems.append(f'uncomparable: {cmp.left}/{name}')
        for sub in cmp.subdirs.values():
            walk(sub)

    walk(filecmp.dircmp(baseline, candidate))
    return problems


def check(baseline_root: Path, scratch_root: Path,
          pyramid_root: Path = PYRAMID_ROOT) -> list[str]:
    problems: list[str] = []
    for name in TREES:
        base = baseline_root / name
        if not base.is_dir():
            problems.append(f'missing baseline tree: {base}')
            continue
        cand = generate_tree(name, scratch_root, pyramid_root)
        problems.extend(diff_dirs(base, cand))
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command', choices=('generate', 'check'))
    ap.add_argument('baseline_dir', type=Path)
    ap.add_argument('scratch_dir', nargs='?', type=Path)
    ap.add_argument('--pyramid-root', type=Path, default=PYRAMID_ROOT)
    args = ap.parse_args()

    if args.command == 'generate':
        generate_all(args.baseline_dir, args.pyramid_root)
        print(f'baselines written to {args.baseline_dir}')
        return 0

    scratch = args.scratch_dir
    tmp = None
    if scratch is None:
        tmp = tempfile.mkdtemp(prefix='pyramid-baseline-check-')
        scratch = Path(tmp)
    try:
        problems = check(args.baseline_dir, scratch, args.pyramid_root)
    finally:
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)
    if problems:
        for p in problems:
            print(p)
        print(f'{len(problems)} difference(s) vs baseline')
        return 1
    print('byte-identical to baseline')
    return 0


if __name__ == '__main__':
    sys.exit(main())
