#!/usr/bin/env python3
"""Foreign-peer LA-CAL interop driver for Phase 4.

Drives the *independently authored* AMS GRA `la-cal-harness` OWP client and
its XSD-derived OMS JSON generator against a running Sleet, opposite a PCL
process (`lacal_e2e_test`). Two directions:

  publish   -- foreign peer PUBs a PositionReport; a PCL subscriber consumes
               and typed-decodes it (proves "we consume foreign").
  subscribe -- foreign peer SUBs; a PCL publisher PUBs; this validates the
               received OMS JSON against the expected shape and values
               (proves "foreign consumes us").

The message shape is the harness generator's own XSD-derived PositionReport
(same global-element structure our codec derives), with identity/position
fields pinned to the values the PCL binary asserts so the cross-process
comparison is deterministic. `$type` JSON annotations the generator adds are
stripped: they are not UCI content and Sleet validates against the XSD.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys

from la_cal_harness.oms_json.generator import OMSJsonGenerator
from la_cal_harness.owp.client import OWPClient
from la_cal_harness.owp.models import Err, Info, Init, Msg, Ok, Pub, Sub

# Values the PCL lacal_e2e_test asserts (fill_position / on_position).
LATITUDE_RAD = 0.8989737191417272
LONGITUDE_RAD = -0.002230530784048753
ALTITUDE_M = 1250.0
ALTITUDE_REFERENCE = "WGS_HAE"
SYSTEM_UUID = "550e8400-e29b-41d4-a716-446655440000"
SERVICE_UUID = "6eefc2b6-08d4-4c39-8267-b1f21745bc90"
TIMESTAMP = "2026-07-11T12:00:00Z"

TOPIC = "mission.position-report"
MESSAGE = "PositionReport"
SCHEMA = "002.5.0"


def _strip_annotations(value):
    """Remove generator `$type` keys recursively (not UCI wire content)."""
    if isinstance(value, dict):
        return {k: _strip_annotations(v) for k, v in value.items() if k != "$type"}
    if isinstance(value, list):
        return [_strip_annotations(v) for v in value]
    return value


def build_position_report(xsd_path: str) -> dict:
    """Foreign, XSD-derived PositionReport with pinned identity/position."""
    doc = _strip_annotations(OMSJsonGenerator(xsd_path).generate_minimal(MESSAGE))
    report = doc[MESSAGE]

    header = report["MessageHeader"]
    header["SystemID"] = {"UUID": SYSTEM_UUID}
    header["ServiceID"] = {"UUID": SERVICE_UUID}
    header["Timestamp"] = TIMESTAMP
    header["SchemaVersion"] = SCHEMA
    header["Mode"] = "LIVE"

    report["SecurityInformation"] = {
        "Classification": "U",
        "OwnerProducer": [{"GovernmentIdentifier": "USA"}],
    }

    data = report["MessageData"]
    data["SystemID"] = {"UUID": SYSTEM_UUID}
    data["Source"] = "ACTUAL"
    data["CurrentOperatingDomain"] = "AIR"
    data["InertialState"] = {
        "Position": {
            "Latitude": LATITUDE_RAD,
            "Longitude": LONGITUDE_RAD,
            "Altitude": ALTITUDE_M,
            "Timestamp": TIMESTAMP,
            "AltitudeReference": ALTITUDE_REFERENCE,
        }
    }
    return doc


def _require(condition: bool, message: str) -> None:
    # Explicit raise (not `assert`) so the oracle survives python -O / PYTHONOPTIMIZE.
    if not condition:
        raise ValueError(message)


def validate_received(body: str) -> None:
    """Check a PCL-published PositionReport matches the expected shape and values.

    Covers the header identity, message-data identity/enums, and the full
    Position numerics -- i.e. the fields the PCL publisher sets deterministically
    -- so a PASS reflects field-level fidelity, not just schema acceptance.
    """
    doc = json.loads(body)
    report = doc[MESSAGE]

    header = report["MessageHeader"]
    _require(header["SystemID"]["UUID"] == SYSTEM_UUID, f"SystemID: {header}")
    _require(header["ServiceID"]["UUID"] == SERVICE_UUID, f"ServiceID: {header}")
    _require(header["Timestamp"] == TIMESTAMP, f"Timestamp: {header}")
    _require(header["SchemaVersion"] == SCHEMA, f"SchemaVersion: {header}")
    _require(header["Mode"] == "LIVE", f"Mode: {header}")

    security = report["SecurityInformation"]
    _require(security["Classification"] == "U", f"Classification: {security}")
    _require(
        security["OwnerProducer"][0]["GovernmentIdentifier"] == "USA",
        f"OwnerProducer: {security}",
    )

    data = report["MessageData"]
    _require(data["SystemID"]["UUID"] == SYSTEM_UUID, f"MessageData SystemID: {data}")
    _require(data["Source"] == "ACTUAL", f"Source: {data}")
    _require(data["CurrentOperatingDomain"] == "AIR", f"Domain: {data}")

    position = data["InertialState"]["Position"]
    _require(abs(position["Latitude"] - LATITUDE_RAD) < 1e-9, f"Latitude: {position}")
    _require(abs(position["Longitude"] - LONGITUDE_RAD) < 1e-9, f"Longitude: {position}")
    _require(abs(position["Altitude"] - ALTITUDE_M) < 1e-9, f"Altitude: {position}")
    _require(
        position["AltitudeReference"] == ALTITUDE_REFERENCE,
        f"AltitudeReference: {position}",
    )
    _require(position["Timestamp"] == TIMESTAMP, f"Position Timestamp: {position}")


async def _handshake(client: OWPClient, service_id: str, verbose: bool) -> Info:
    _, info = await client.handshake(
        Init(versions=["1.0"], schema=SCHEMA, service_id=service_id, verbose=verbose)
    )
    return info


async def run_publish(url: str, service_id: str, xsd_path: str) -> int:
    body = json.dumps(build_position_report(xsd_path))
    client = await OWPClient.connect(url)
    # Verbose PUB draws a +OK / -ERR so schema rejection surfaces loudly.
    await _handshake(client, service_id, verbose=True)
    await client.send(Pub(topic=TOPIC, message=body))
    reply = await client.recv(timeout=5.0)
    if isinstance(reply, Err):
        print(f"interop-publish: PUB rejected: {reply.error} {reply.details or ''}",
              file=sys.stderr)
        return 2
    if not isinstance(reply, Ok):
        print(f"interop-publish: expected +OK for PUB, got {reply}", file=sys.stderr)
        return 2
    print(f"interop-publish: PUB {TOPIC} {MESSAGE} accepted", flush=True)
    return 0


async def run_subscribe(url: str, service_id: str, ready_path: str, timeout: float) -> int:
    client = await OWPClient.connect(url)
    # Verbose so the SUB is +OK-acked before we signal readiness: the peer's
    # PUB must not race ahead of an active subscription (BEST_EFFORT, no
    # broker-side retention).
    await _handshake(client, service_id, verbose=True)
    await client.send(Sub(subscription_id="1", message_name=MESSAGE, topic=TOPIC))
    ack = await client.recv(timeout=timeout)
    if isinstance(ack, Err):
        print(f"interop-subscribe: SUB rejected: {ack.error}", file=sys.stderr)
        return 2
    if not isinstance(ack, Ok):
        print(f"interop-subscribe: expected +OK for SUB, got {ack}", file=sys.stderr)
        return 2
    with open(ready_path, "w", encoding="utf-8") as handle:
        handle.write("ready\n")

    message = await client.recv(timeout=timeout)
    if not isinstance(message, Msg):
        print(f"interop-subscribe: expected MSG, got {message}", file=sys.stderr)
        return 2
    validate_received(message.message)
    print("interop-subscribe: received schema-valid PositionReport", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("publish")
    p.add_argument("--url", required=True)
    p.add_argument("--service-id", default="lacal-harness")
    p.add_argument("--xsd", required=True)

    s = sub.add_parser("subscribe")
    s.add_argument("--url", required=True)
    s.add_argument("--service-id", default="lacal-harness")
    s.add_argument("--ready", required=True)
    s.add_argument("--timeout", type=float, default=10.0)

    args = parser.parse_args()
    if args.mode == "publish":
        return asyncio.run(run_publish(args.url, args.service_id, args.xsd))
    return asyncio.run(run_subscribe(args.url, args.service_id, args.ready, args.timeout))


if __name__ == "__main__":
    raise SystemExit(main())
