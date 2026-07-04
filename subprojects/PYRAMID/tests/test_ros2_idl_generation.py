#!/usr/bin/env python3
"""Validate the native ROS2 IDL generator against ROS2 interface rules.

This is a fast, toolchain-free regression check: it generates the .msg/.srv from
the real .proto contracts and asserts the output obeys the rosidl rules that
would otherwise only surface in a full colcon build (CamelCase interface names,
valid primitive/array types, resolvable type references, valid field/constant
names). The full rosidl build is exercised separately under the ament package.

Run directly:  python3 test_ros2_idl_generation.py [proto_dir]
"""

from __future__ import annotations

import re
import sys
import tempfile
from pathlib import Path

PIM = Path(__file__).resolve().parents[1] / "pim"
sys.path.insert(0, str(PIM))

from proto_parser import (  # noqa: E402
    parse_proto_tree, ProtoTypeIndex, is_binding_proto,
)
from ros2_idl_codegen import generate_ros2_idl, IDL_EXTERNAL_DEPENDENCIES  # noqa: E402

# rosidl interface names: CamelCase, start uppercase, alnum only.
_IFACE_RE = re.compile(r"^[A-Z][A-Za-z0-9]*$")
# rosidl field names: lowercase, snake_case, no leading/trailing/double underscore.
_FIELD_RE = re.compile(r"^[a-z](_?[a-z0-9])*$")
# rosidl constant names: uppercase letters/digits/underscore.
_CONST_RE = re.compile(r"^[A-Z](_?[A-Z0-9])*$")

_PRIMITIVES = {
    "bool", "byte", "char",
    "float32", "float64",
    "int8", "uint8", "int16", "uint16",
    "int32", "uint32", "int64", "uint64",
    "string", "wstring",
}

_failures: list[str] = []


def _check(cond: bool, msg: str) -> None:
    if not cond:
        _failures.append(msg)


def _base_type(token: str) -> str:
    """Strip an array suffix: 'Foo[]' -> 'Foo', 'uint8[]' -> 'uint8'."""
    return re.sub(r"\[\d*\]$", "", token)


def _parse_idl_line(line: str):
    """Return ('const', type, name) / ('field', type, name) / None for a line."""
    line = line.split("#", 1)[0].strip()
    if not line:
        return None
    # constant: TYPE NAME=VALUE
    m = re.match(r"^(\S+)\s+([A-Za-z0-9_]+)\s*=\s*(.+)$", line)
    if m:
        return ("const", m.group(1), m.group(2))
    m = re.match(r"^(\S+)\s+([A-Za-z0-9_]+)$", line)
    if m:
        return ("field", m.group(1), m.group(2))
    return ("unparsed", line, "")


def main(argv: list[str]) -> int:
    proto_dir = Path(argv[1]) if len(argv) > 1 else (
        Path(__file__).resolve().parents[1] / "proto")
    # Mirror the production index: generate_bindings.py excludes
    # compiler-extension protos (pyramid.options) before any generator runs.
    index = ProtoTypeIndex(
        [pf for pf in parse_proto_tree(proto_dir) if is_binding_proto(pf)])

    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        written = generate_ros2_idl(index, out)
        msg_files = sorted((out / "msg").glob("*.msg"))
        srv_files = sorted((out / "srv").glob("*.srv"))

        _check(len(written) == len(msg_files) + len(srv_files),
               "returned path count != files on disk")
        _check(len(msg_files) > 0 and len(srv_files) > 0,
               "expected both .msg and .srv output")

        # Every generated message name is a legal, resolvable ROS2 type.
        known_types = {p.stem for p in msg_files}
        external = set()
        for dep in IDL_EXTERNAL_DEPENDENCIES:
            # builtin_interfaces/Time etc. — accept any pkg/Type from a declared dep.
            external.add(dep)

        def _type_ok(token: str) -> bool:
            base = _base_type(token)
            if base in _PRIMITIVES:
                return True
            if "/" in base:  # pkg/Type from an external dependency
                pkg = base.split("/", 1)[0]
                return pkg in external
            return base in known_types

        for f in msg_files + srv_files:
            _check(bool(_IFACE_RE.match(f.stem)),
                   f"{f.name}: interface name not CamelCase")

        # Field/constant/type validation across every section.
        for f in msg_files + srv_files:
            text = f.read_text(encoding="utf-8")
            for raw in text.splitlines():
                if raw.strip() == "---":  # service section separator
                    continue
                parsed = _parse_idl_line(raw)
                if parsed is None:
                    continue
                kind, typ, name = parsed
                _check(kind != "unparsed", f"{f.name}: unparseable line {typ!r}")
                if kind == "field":
                    _check(_type_ok(typ), f"{f.name}: bad/unresolved type {typ!r}")
                    _check(bool(_FIELD_RE.match(name)),
                           f"{f.name}: bad field name {name!r}")
                elif kind == "const":
                    _check(_type_ok(typ), f"{f.name}: bad const type {typ!r}")
                    _check(bool(_CONST_RE.match(name)),
                           f"{f.name}: bad const name {name!r}")

        # Spot-check the domain-aligned structural mappings the generator
        # promises (mirrors pyramid::domain_model: aliases collapsed, base
        # inlined, optional non-string -> has_ companion, optional string plain).
        entity = (out / "msg" / "Entity.msg").read_text(encoding="utf-8")
        _check("bool has_update_time" in entity and "float64 update_time" in entity,
               "Entity.msg missing collapsed optional Timestamp (has_update_time/float64)")
        _check("string id" in entity and "string source" in entity,
               "Entity.msg missing collapsed Identifier->string id/source")
        # Scalar-wrapper aliases must NOT be emitted as ROS2 types.
        for alias in ("Identifier", "Angle", "Length", "Timestamp", "Percentage"):
            _check(not (out / "msg" / f"{alias}.msg").exists(),
                   f"{alias}.msg should not be emitted (it is a scalar alias)")
        # Compiler-extension option types must NOT be emitted as ROS2 types.
        for opt_type in ("Interaction", "Qos"):
            _check(not (out / "msg" / f"{opt_type}.msg").exists(),
                   f"{opt_type}.msg should not be emitted (pyramid.options is "
                   "not part of the SDK surface)")

        query = (out / "msg" / "Query.msg").read_text(encoding="utf-8")
        _check("string[] id" in query, "Query.msg missing repeated string id (Identifier collapsed)")

        # Base inlining: ObjectMatch inlines Entity's fields directly.
        om = (out / "msg" / "ObjectMatch.msg").read_text(encoding="utf-8")
        _check("string matching_object_id" in om, "ObjectMatch.msg missing own field")
        _check("string id" in om and "float64 update_time" in om,
               "ObjectMatch.msg missing inlined Entity base fields")

        # oneof members become has_ + value (mirrors domain tl::optional members).
        goal = (out / "msg" / "PlanningGoal.msg").read_text(encoding="utf-8")
        _check("bool has_expression" in goal and "string expression" in goal,
               "PlanningGoal.msg missing oneof member presence companion")

        # A streaming rpc yields an open .srv + a typed frame .msg.
        frame = out / "msg" / "TacticalObjectsProvidedMatchingObjectsReadMatchFrame.msg"
        srv = out / "srv" / "TacticalObjectsProvidedMatchingObjectsReadMatch.srv"
        _check(frame.exists(), "streaming frame .msg not generated")
        _check(srv.exists(), "streaming open .srv not generated")
        if frame.exists():
            ftext = frame.read_text(encoding="utf-8")
            _check("ObjectMatch data" in ftext, "frame missing typed payload field")
            _check("bool end_of_stream" in ftext, "frame missing end_of_stream")

    if _failures:
        print(f"FAIL: {len(_failures)} ROS2 IDL check(s) failed:")
        for msg in _failures:
            print(f"  - {msg}")
        return 1
    print(f"OK: ROS2 IDL valid ({len(msg_files)} msg, {len(srv_files)} srv)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
