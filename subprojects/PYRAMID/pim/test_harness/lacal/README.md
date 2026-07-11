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
