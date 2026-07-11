#!/usr/bin/env python3
"""Fetch and pin the XSD schema drops consumed by xsd2proto.py.

Phase 0 of doc/plans/PYRAMID/uci_mms_conversion_plan.md.  The XSDs are not
vendored into the repo until their redistribution posture is confirmed (see
README.md here); this script materialises them under ``pim/schemas/dl/<drop>/``
from, in order of preference:

  1. an environment-variable override (file or directory), e.g. the
     ``UCI_XSD_PATH`` variable the LA-CAL interop driver already uses;
  2. a well-known location inside the git-ignored ``external/`` checkout
     (the persistent Kitty Hawk stack contains the UCI 2.5 XSD);
  3. a pinned HTTPS URL (the public a-gra release).

Every obtained file is sha256-verified against ``schema_manifest.json``:

  * pin matches            -> OK, file is usable;
  * pin mismatch           -> hard failure, the file is removed;
  * pin is null (bootstrap)-> the file is kept but reported UNVERIFIED, and
    the computed hash is printed so it can be committed into the manifest
    (trust-on-first-use, done once, in writing).

Exit codes: 0 = all requested drops present and pinned; 3 = present but at
least one UNVERIFIED (bootstrap pending); 1 = failure.  Consumers that need
the XSDs (tests, xsd2proto runs) should SKIP with a printed reason when this
script cannot produce them -- the established Sleet/GNAT pattern.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import shutil
import sys
import urllib.error
import urllib.request
from pathlib import Path

SCHEMAS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCHEMAS_DIR.parents[3]
MANIFEST_PATH = SCHEMAS_DIR / "schema_manifest.json"
DL_DIR = SCHEMAS_DIR / "dl"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _resolve_env_source(source: dict, name: str) -> Path | None:
    import os

    env = source.get("env")
    value = os.environ.get(env or "")
    if not value:
        return None
    p = Path(value)
    if p.is_file():
        # A file-valued override must actually be the requested file.
        return p if p.name == name else None
    if p.is_dir():
        candidate = p / name
        if candidate.is_file():
            return candidate
        hits = sorted(p.rglob(name))
        if hits:
            return hits[0]
    return None


def _resolve_checkout_source(source: dict) -> Path | None:
    pattern = source.get("glob")
    if not pattern:
        return None
    hits = sorted(glob.glob(str(REPO_ROOT / pattern), recursive=True))
    return Path(hits[0]) if hits else None


def _resolve_url_source(source: dict, dest: Path) -> Path | None:
    url = source.get("url")
    if not url:
        return None
    try:
        with urllib.request.urlopen(url, timeout=60) as resp:
            dest.parent.mkdir(parents=True, exist_ok=True)
            tmp = dest.with_suffix(dest.suffix + ".part")
            with tmp.open("wb") as f:
                shutil.copyfileobj(resp, f)
            tmp.replace(dest)
            return dest
    except (urllib.error.URLError, OSError) as exc:
        print(f"  url source failed ({url}): {exc}")
        return None


def fetch_file(drop: str, spec: dict, force: bool) -> str:
    """Return 'pinned', 'unverified', or 'missing' for one manifest file."""
    name = spec["name"]
    pin = spec.get("sha256")
    dest = DL_DIR / drop / name

    if dest.is_file() and not force:
        digest = _sha256(dest)
        if pin and digest == pin:
            print(f"  {name}: present, pin OK")
            return "pinned"
        if pin:
            print(f"  {name}: PIN MISMATCH (expected {pin}, got {digest}); removing")
            dest.unlink()
        else:
            print(f"  {name}: present but UNVERIFIED (no pin recorded)")
            print(f"    bootstrap: set \"sha256\": \"{digest}\" in schema_manifest.json and commit")
            return "unverified"

    for source in spec.get("sources", []):
        kind = source.get("kind")
        found: Path | None = None
        if kind == "env":
            found = _resolve_env_source(source, name)
            if found:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(found, dest)
                print(f"  {name}: copied from ${source.get('env')} ({found})")
        elif kind == "checkout":
            found = _resolve_checkout_source(source)
            if found:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(found, dest)
                print(f"  {name}: copied from checkout ({found})")
        elif kind == "url":
            found = _resolve_url_source(source, dest)
            if found:
                print(f"  {name}: downloaded from {source.get('url')}")
        if not found:
            continue

        digest = _sha256(dest)
        if pin:
            if digest == pin:
                print(f"    pin OK ({digest[:12]}...)")
                return "pinned"
            print(f"    PIN MISMATCH: expected {pin}, got {digest}; removing")
            dest.unlink()
            continue  # a later source might carry the pinned bytes
        print(f"    UNVERIFIED (no pin recorded)")
        print(f"    bootstrap: set \"sha256\": \"{digest}\" in schema_manifest.json and commit")
        return "unverified"

    print(f"  {name}: MISSING (no source produced it)")
    return "missing"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("drops", nargs="*", help="drop names from schema_manifest.json (default: all)")
    parser.add_argument("--force", action="store_true", help="re-fetch even if a file is already present")
    args = parser.parse_args()

    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    drops = manifest["drops"]
    requested = args.drops or sorted(drops)
    unknown = [d for d in requested if d not in drops]
    if unknown:
        print(f"unknown drop(s): {', '.join(unknown)}; manifest has: {', '.join(sorted(drops))}")
        return 1

    results: list[str] = []
    for drop in requested:
        print(f"drop {drop} (schema_version {drops[drop].get('schema_version')}):")
        for spec in drops[drop]["files"]:
            results.append(fetch_file(drop, spec, args.force))

    if "missing" in results:
        print("FAIL: at least one schema file could not be obtained")
        return 1
    if "unverified" in results:
        print("UNVERIFIED: obtained, but at least one sha256 pin is not yet recorded")
        return 3
    print("OK: all requested schema files present and pinned")
    return 0


if __name__ == "__main__":
    sys.exit(main())
