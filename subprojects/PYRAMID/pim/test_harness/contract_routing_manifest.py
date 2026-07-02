#!/usr/bin/env python3
"""Build a PCL routing manifest from generated binding endpoint requirements."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _route_line(req: dict, peer: str) -> str:
    rel = req.get("reliability", "unspecified")
    suffix = "" if rel == "unspecified" else f" {rel}"
    return f"route {req['endpoint_name']} {req['kind']} {peer}{suffix}"


def _find_correlated_topics(reqs: list[dict]) -> tuple[dict, dict]:
    publishers = [
        r for r in reqs
        if r.get("source") == "topic"
        and r.get("kind") == "publisher"
        and r.get("endpoint_name", "").endswith(".request")
    ]
    subscribers = [
        r for r in reqs
        if r.get("source") == "topic"
        and r.get("kind") == "subscriber"
        and r.get("endpoint_name", "").endswith(".requirement")
    ]
    by_endpoint = {r["endpoint_name"]: r for r in subscribers}
    for pub in publishers:
        base = pub["endpoint_name"][: -len(".request")]
        sub = by_endpoint.get(base + ".requirement")
        if sub:
            return pub, sub
    raise SystemExit("no correlated request/requirement topic pair in manifest")


def _find_consumed_service(reqs: list[dict]) -> dict:
    for rpc_name in ("Create", "Update", "Cancel"):
        for req in reqs:
            if (
                req.get("source") == "service"
                and req.get("kind") == "consumed"
                and req.get("rpc_name") == rpc_name
            ):
                return req
    raise SystemExit("no consumed unary request endpoint in manifest")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("binding_manifest")
    parser.add_argument("plugin_path")
    parser.add_argument("out_manifest")
    args = parser.parse_args()

    data = json.loads(Path(args.binding_manifest).read_text(encoding="utf-8"))
    reqs = data.get("endpoint_requirements", [])
    service_req = _find_consumed_service(reqs)
    request_topic, requirement_topic = _find_correlated_topics(reqs)

    plugin = Path(args.plugin_path).resolve().as_posix()
    lines = [
        f'transport svc_rpc {plugin} {{"mode":"rpc"}}',
        f'transport topic_pubsub {plugin} {{"mode":"pubsub"}}',
        _route_line(service_req, "svc_rpc"),
        _route_line(request_topic, "topic_pubsub"),
        _route_line(requirement_topic, "topic_pubsub"),
        "",
    ]
    Path(args.out_manifest).write_text("\n".join(lines), encoding="utf-8")
    print(f"service_route={service_req['endpoint_name']}:{service_req['kind']}")
    print(f"request_topic_route={request_topic['endpoint_name']}:{request_topic['kind']}")
    print(
        f"requirement_topic_route="
        f"{requirement_topic['endpoint_name']}:{requirement_topic['kind']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
