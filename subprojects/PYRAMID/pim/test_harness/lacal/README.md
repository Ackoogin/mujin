# PYRAMID LA-CAL harness

This directory contains the pinned OWP grammar, Sleet service registrations,
and the cross-process Phase 3 proof.

Run against an already configured Sleet instance:

```bash
SLEET_URL=ws://127.0.0.1:21402 ./build_lacal_e2e_test.sh
```

Copy the two files under `services/` into the Sleet `services_dir` before
starting that server. The service IDs, topic names, and UCI message bindings
must match exactly.

The script can also launch a local Sleet binary when its pinned UCI schema is
available:

```bash
SLEET_BIN=/path/to/sleet \
SLEET_SCHEMA_PATH=/path/to/UCI_MessageDefinitions_v2_5_0.xsd \
./build_lacal_e2e_test.sh
```

When neither mode is available, or the configured URL is unreachable, the
script prints a `SKIP:` reason and exits successfully. A configured server that
accepts connections but rejects initialization, subscriptions, schema-valid
payloads, or routing is a test failure rather than a skip.

The proof starts two separate `lacal_e2e_test` processes. Each process loads
`libpyramid_lacal_transport_plugin` via `pcl_transport_routing_load`. The
subscriber routes `mission.position-report`, registers its PCL subscriber, and
waits. The publisher loads the OMS JSON codec plugin, encodes the frozen UCI
PositionReport C struct, and publishes it. Success requires the subscriber to
decode and verify the typed position after Sleet routes it.

## Phase 4 foreign-peer interop (`build_lacal_interop_test.sh`)

Runs the PCL transport against the **independently authored** AMS GRA
`la-cal-harness` as the opposite peer, proving both interop directions through
Sleet:

- **A (we consume foreign):** the harness OWP client publishes an XSD-derived
  OMS JSON `PositionReport`; the PCL subscriber typed-decodes it.
- **B (foreign consumes us):** the PCL publisher publishes; the harness OWP
  client subscribes, receives, and validates the OMS JSON.

`lacal_interop_driver.py` is the foreign peer: it drives the harness's own
`OWPClient` and `OMSJsonGenerator`, pinning identity/position fields to the
values the PCL binary asserts so the cross-process comparison is deterministic.
The `lacal-harness` service must be registered (see `services/`).

Install the harness peer into a Python 3.12+ venv, then run:

```bash
python3.12 -m venv .venv
.venv/bin/pip install -e /path/to/la-cal-harness
SLEET_URL=ws://127.0.0.1:21402 \
LACAL_HARNESS_PYTHON=$PWD/.venv/bin/python \
UCI_XSD_PATH=/path/to/UCI_MessageDefinitions_v2_5_0.xsd \
./build_lacal_interop_test.sh
```

The script SKIPs (exit 0) when `SLEET_URL` is unreachable, no harness-capable
Python is given, or the XSD is absent; a reachable Sleet that then rejects any
step is a FAIL. A captured PASS run is committed at `interop_run.log`.

## Phase 5 request/entity seam (`build_lacal_seam_test.sh`)

Demonstrates the interaction seam's **pubsub-works over the real broker** leg
with a UCI vocabulary Sleet validates: a correlated request/entity
interaction, both legs realized pub/sub over LA-CAL, using the
`ActionCommand` / `ActionCommandStatus` pair (correlation key `CommandID.UUID`).

- provider (`seam-ma`) subscribes the request topic; on an `ActionCommand` it
  publishes correlated `ActionCommandStatus` transitions (`RECEIVED`,
  `ACCEPTED`) on the entity topic;
- consumer (`seam-c2`) publishes the `ActionCommand`, subscribes the
  entity topic, and asserts it collects both correlated transitions.

```bash
SLEET_URL=ws://127.0.0.1:21402 ./build_lacal_seam_test.sh
```

Needs the `seam-ma` / `seam-c2` service registrations (`services/`) loaded by
Sleet. SKIPs when Sleet is absent/unreachable. Captured PASS at `seam_run.log`.
The rpc-impossible half of the matrix is the unit test
`owp.LacalPlugin.RpcEndpointOverPubsubPeerFailsClosed`.

The positive leg drives the PCL pub/sub primitives directly (like
`lacal_e2e_test`), not the generated `MaactionRequestPort` interaction facade;
running that facade over LA-CAL additionally needs OMS-JSON codecs for the
generated seam structs (rung-3 `oms_json_backend.py`).
