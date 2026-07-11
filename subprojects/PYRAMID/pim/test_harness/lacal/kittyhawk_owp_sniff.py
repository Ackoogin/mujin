#!/usr/bin/env python3
"""Read-only OWP observer for a running AMS GRA Kitty Hawk stack.

Connects to a live Sleet instance as a registered scratch service
(`ame-sniffer` — see `sleet/services.d.local/ame-sniffer.toml` in the local
`external/ams-gra/` bring-up, documented in
`subprojects/PYRAMID/doc/guides/ams_gra_starter_kit_bringup.md`) and
subscribes to the Kitty Hawk demo's UCI topics, printing the first couple of
payloads per topic and a running count. Useful for confirming the stack is
actually producing traffic (PositionReport/ObservationMeasurementReport/
SignalReport/ServiceStatus) without needing a full PCL harness, and for
capturing sample payloads to spec the AME MS-leg bridge against.

Requires `pip install --user websocket-client`; no other PYRAMID/PCL
dependency (same "no PCL dependency" spirit as the `owp` client core).
"""

from __future__ import annotations

import argparse
import json
import time

import websocket

DEFAULT_TOPICS = [
    ("sub1", "SignalReport", "mission.signal-report"),
    ("sub2", "ObservationMeasurementReport", "mission.observation-measurement-report"),
    ("sub3", "PositionReport", "mission.position-report"),
    ("sub4", "ServiceStatus", "mission.service-status"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:21402/owp")
    parser.add_argument("--service-id", default="ame-sniffer")
    parser.add_argument("--schema", default="002.5.0")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument(
        "--samples-per-topic",
        type=int,
        default=2,
        help="full payloads to print per subscription before only counting",
    )
    args = parser.parse_args()

    ws = websocket.WebSocket()
    ws.connect(args.url, subprotocols=["owp"])
    init_payload = {"versions": ["1.0"], "schema": args.schema, "service_id": args.service_id}
    ws.send("INIT " + json.dumps(init_payload))
    print("INIT ->", ws.recv())

    for sid, msg_name, topic in DEFAULT_TOPICS:
        ws.send(f"SUB {sid} {msg_name} {topic}")
        print(f"SUB {sid} {msg_name} {topic} ->", ws.recv())

    deadline = time.time() + args.duration
    counts: dict[str, int] = {}
    while time.time() < deadline:
        ws.settimeout(2.0)
        try:
            message = ws.recv()
        except Exception:
            continue
        if not message:
            continue
        parts = message.split(" ", 2)
        if len(parts) == 3 and parts[0] == "MSG":
            sid = parts[1]
            counts[sid] = counts.get(sid, 0) + 1
            if counts[sid] <= args.samples_per_topic:
                print(f"--- {sid} sample #{counts[sid]} ---")
                print(parts[2])
        else:
            print("OTHER:", message[:200])

    print("counts:", counts)


if __name__ == "__main__":
    main()
