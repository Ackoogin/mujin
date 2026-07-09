#!/usr/bin/env python3
"""Build a PCL routing manifest from generated binding interactions.

Phase 1 (D5, doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md):
`binding_manifest.json` carries an `interactions` section -- one Interaction
per Request/Information port, each leg listing its two mutually-exclusive
sides (side A: rpc/service endpoint(s); side B: the one topic endpoint).
This script picks the first Request-shape interaction realized from the
*consumed* role (the consumer-side test component every existing caller of
this script exercises), and for its `request`/`requirement` legs emits:

  - a `transport` line per peer (`svc_rpc` for the rpc side, `topic_pubsub`
    for the pub/sub side -- always both, regardless of which side is
    routed, so a --realize override can switch sides without a manifest
    edit);
  - one PCL `exclusive` group per leg (declared before any route line for
    one of its members, per the grammar's declare-before-use rule);
  - route lines for exactly one side of each leg -- default the rpc/service
    side ("rpc") for both legs, with a per-leg override via --realize.

    D5/D1's `pattern` stamp does not resolve to a single side cleanly (a
    request leg's stamp differs by role, and the requirement leg's stamp is
    the request leg's inverse -- there is no one reading of "the stamp" that
    picks one side per leg without also depending on which role generated
    the manifest). Absent an unambiguous reading, "rpc" is the conservative
    default: it is the transport every existing PCL harness in this tree
    validates first, needs no correlated-pair bookkeeping to exercise, and
    keeps this script's synchronous validate-and-exit shape. Before Phase 1
    this script instead free-routed one rpc endpoint *and* both pub/sub
    topics together for the same interaction -- precisely the unreconciled
    dual-seam pattern D5 makes a compose-time error; picking one side per
    leg here is a deliberate, required behaviour change, not a regression
    (see the Phase 1 ledger entry in the plan doc).

The positional CLI interface (binding_manifest, plugin_path, out_manifest)
is unchanged from before Phase 1: existing callers
(build_contract_routing_test.sh/.bat) invoke this with exactly those three
arguments and no flags, and keep working unmodified.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

_LEG_NAMES = ("request", "requirement", "information")


def _reliability_floors(endpoint_requirements: list) -> dict:
    """Map (endpoint_name, kind) -> reliability floor ("reliable" /
    "best_effort" / ""), from `binding_manifest.json`'s `endpoint_requirements`
    -- the same per-endpoint QoS source `EndpointRequirement`/
    `_build_endpoint_requirements` already compute (binding_contract.py).
    `interactions` (this script's own primary source) does not carry
    reliability on its `InteractionEndpoint` entries, so route lines must
    join back here rather than silently dropping the floor -- the PCL
    manifest loader only enforces QoS from an explicit reliability token on
    the `route` line itself (see FailsClosedWhenQosFloorUnmet)."""
    floors: dict = {}
    for req in endpoint_requirements:
        floors[(req["endpoint_name"], req["kind"])] = req.get("reliability", "")
    return floors


def _route_line(endpoint: dict, peer: str, floors: dict) -> str:
    line = f"route {endpoint['endpoint_name']} {endpoint['kind']} {peer}"
    reliability = floors.get((endpoint["endpoint_name"], endpoint["kind"]), "")
    if reliability:
        line += f" {reliability}"
    return line


def _find_request_interaction(interactions: list) -> dict:
    """The first Request-shape interaction (`port_kind == "request"`, both a
    `request` and a `requirement` leg present) whose request leg's side A
    (the rpc/service side) is the *consumed* role -- the consumer-side
    component shape every existing caller of this script exercises."""
    for interaction in interactions:
        if interaction.get("port_kind") != "request":
            continue
        legs_by_name = {leg["name"]: leg for leg in interaction.get("legs", [])}
        request_leg = legs_by_name.get("request")
        requirement_leg = legs_by_name.get("requirement")
        if not request_leg or not requirement_leg or not request_leg.get("side_a"):
            continue
        if request_leg["side_a"][0].get("kind") != "consumed":
            continue
        return interaction
    raise SystemExit("no consumed request-shape interaction in manifest")


def _parse_realize_overrides(specs: list) -> dict:
    overrides = {}
    for spec in specs:
        if "=" not in spec:
            raise SystemExit(f"--realize expects <leg>=rpc|pubsub, got {spec!r}")
        leg, side = spec.split("=", 1)
        if leg not in _LEG_NAMES:
            raise SystemExit(
                f"--realize leg must be one of {_LEG_NAMES}, got {leg!r}"
            )
        if side not in ("rpc", "pubsub"):
            raise SystemExit(f"--realize side must be rpc or pubsub, got {side!r}")
        overrides[leg] = side
    return overrides


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("binding_manifest")
    parser.add_argument("plugin_path")
    parser.add_argument("out_manifest")
    parser.add_argument(
        "--realize",
        action="append",
        default=[],
        metavar="<leg>=rpc|pubsub",
        help="override the default realization (rpc) of one leg "
             "(request|requirement|information); repeatable, e.g. "
             "--realize request=pubsub --realize requirement=pubsub",
    )
    args = parser.parse_args()
    overrides = _parse_realize_overrides(args.realize)

    data = json.loads(Path(args.binding_manifest).read_text(encoding="utf-8"))
    interaction = _find_request_interaction(data.get("interactions", []))
    legs_by_name = {leg["name"]: leg for leg in interaction["legs"]}
    floors = _reliability_floors(data.get("endpoint_requirements", []))

    plugin = Path(args.plugin_path).resolve().as_posix()
    lines = [
        f'transport svc_rpc {plugin} {{"mode":"rpc"}}',
        f'transport topic_pubsub {plugin} {{"mode":"pubsub"}}',
    ]

    summary = []
    for leg_name in ("request", "requirement"):
        leg = legs_by_name[leg_name]
        side_choice = overrides.get(leg_name, "rpc")
        routed_side = leg["side_a"] if side_choice == "rpc" else leg["side_b"]
        peer = "svc_rpc" if side_choice == "rpc" else "topic_pubsub"

        side_a_names = ",".join(ep["endpoint_name"] for ep in leg["side_a"])
        side_b_names = ",".join(ep["endpoint_name"] for ep in leg["side_b"])
        lines.append(f'exclusive {leg["group_name"]} {side_a_names} {side_b_names}')
        for ep in routed_side:
            lines.append(_route_line(ep, peer, floors))
            summary.append(f"{leg_name}_leg_route={ep['endpoint_name']}:{ep['kind']}")
    lines.append("")

    Path(args.out_manifest).write_text("\n".join(lines), encoding="utf-8")
    print(f"interaction={interaction['service_key']}")
    for line in summary:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
