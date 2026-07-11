# LA-CAL Integration Plan — Rung 1 of the OMS CAL Join

**Status:** in progress — Phases 0–4 are implemented and verified on Linux.
Phase 3 (real-Sleet cross-process delivery) and Phase 4 (OMS JSON codec +
foreign-peer interop, both directions, against the AMS GRA `la-cal-harness`)
are complete against the pinned Sleet container. Phases 5 (interaction seam
over LA-CAL) and 6 (stretch AME MS-leg demo) remain.
**Date:** 2026-07-11
**Design source:**
[`doc/research/AME/ams_gra_oms_cal_join.md`](../../research/AME/ams_gra_oms_cal_join.md)
§5 (the LA-CAL form of the direct join) and §6 (rungs 1–3). This plan makes
rung 1 concrete: a PCL transport plugin speaking the **OMS WebSocket
Protocol (`owp`, OMSC-SPC-013)** plus an **OMS JSON** codec, proven against
the public Sleet server and LA-CAL test harness from the AMS GRA starter
kit.
**Companions:**
[`ams_gra_starter_kit_review.md`](../../research/AME/ams_gra_starter_kit_review.md)
(Sleet, registration, Kitty Hawk scenario),
[`pubsub_interaction_guide.md`](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md)
(the interaction seam this rides),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(plugin ABI, capability model).

---

## 1. Goal and non-goals

**Goal.** A PYRAMID component — unchanged component code, contract-derived
bindings — publishes and subscribes UCI-shaped messages on an OMS Abstract
Service Bus through its LA-CAL entry point, with the Request-port
interaction running in its pub/sub realization over that transport, and
the whole path proven against independently-authored infrastructure
(Sleet + `la-cal-harness`), not just our own loopback.

**Non-goals (later rungs / other plans):**

- No `xsd2proto` converter or full A-GRA profile — the demo vocabulary is
  hand-authored, exactly like `pim/agra_example` (which stays the model).
- No provider C++ CAL plugin (rung 4; blocked on an SDK existing).
- No EXI/DMS offboard stack, no OMS compliance claim of any kind.
- No semantic bridging — `agra_c2_bridge` remains future work; everything
  here moves already-shaped messages ("the CAL join carries no
  semantics").

---

## 2. Deliverables

| # | Artifact | Location | New/extends |
|---|----------|----------|-------------|
| 1 | `owp` client core (WebSocket + protocol state machine, no PCL deps) | `subprojects/PYRAMID/src/owp/` → static lib `pyramid_owp` | new |
| 2 | LA-CAL transport plugin (`PUBSUB`-only, fail-closed INIT) | `subprojects/PYRAMID/plugins/pyramid_lacal_transport_plugin.cpp` → `libpyramid_lacal_transport_plugin.so` | new; sibling of the coupled plugins |
| 3 | OMS JSON codec for the demo vocabulary (`application/oms-json`) | hand-written first: `subprojects/PYRAMID/plugins/pyramid_oms_json_codec_agra.cpp`; end-state: `pim/backends/oms_json_backend.py` | new |
| 4 | Sleet-backed E2E harnesses (+ mock-server unit tests) | `subprojects/PYRAMID/tests/test_owp_client.cpp`, `pim/test_harness/lacal_*` | new; follows `agra_*` harness conventions |
| 5 | Interaction-facade-over-LA-CAL proof (rpc/pubsub interchange, third transport) | `pim/test_harness/lacal_seam_test.cpp` (variant of `agra_seam_interchange_test.cpp`) | extends |
| 6 | Capability-matrix + guide updates | `transport_codec_plugin_system.md` (new row), `pubsub_interaction_guide.md` §7 (new examples) | doc |

---

## 3. Design decisions (fixed up front)

- **D1 — Decoupled transport + codec, not a coupled `.so` (initially).**
  `owp` carries opaque UTF-8 JSON bodies addressed by topic, so the
  existing decoupled pattern fits: the codec produces the exact OMS JSON
  text, the transport frames it into `PUB`. The plugin **guards the
  content type** (config declares `"content_type":"application/oms-json"`;
  publish with anything else fails `PCL_ERR_STATE`) so mispairing fails
  closed rather than putting proto3-JSON on the bus. A single-`.so`
  coupled packaging (gRPC/ROS2 pattern) can follow for deploy convenience;
  it is not needed for correctness.
- **D2 — Caps `PCL_CAP_PUBSUB` only** (`serve`/`invoke_*`/`stream_*` slots
  NULL, so vtable derivation agrees with the declaration). QoS declaration
  is a **Phase-0 exit question**: default `BEST_EFFORT` unless the
  official OMSC-SPC-013 text supports declaring `RELIABLE` (buffering/
  redelivery is the server's; WebSocket alone doesn't earn the claim). A
  config override (`"declare_reliability":"reliable"`) makes the
  deploy-time decision explicit, mirroring the UDP downgrade convention.
- **D3 — Fail-closed initialization.** `pcl_transport_plugin_entry`
  performs the `INIT` → `INFO` handshake synchronously with a bounded
  timeout and **returns NULL if `INFO` does not arrive** — this is the
  loud surface for Sleet's silent rejection of unregistered services
  (starter-kit review §4.2). Diagnostics name the URL and `service_id`.
- **D4 — Threading per the PCL transport contract.** One plugin-owned
  asio/websocketpp worker thread. `publish` validates, copies, enqueues,
  returns (never blocks on the socket). Inbound `MSG` frames are
  deep-copied and posted via `pcl_executor_post_remote_incoming` from the
  worker. `pcl_transport_plugin_teardown` stops and joins the worker
  before `dlclose` (UDP plugin precedent).
- **D5 — Topic discipline: topic == UCI message name.** The demo contract
  tree stamps OMS-style topic names via `pyramid.options.pyramid_op`
  (expressible today; `agra_example/README.md` "Topic naming" note). The
  transport passes topics through verbatim; `subscribe(topic, type_name)`
  already receives the message name the `SUB` operation needs.
- **D6 — Identity from `INFO`, not invented.** The System/Service UUIDs
  returned in `INFO` are recorded on the adapter and exposed via a small
  getter (`pyramid_lacal_transport_get_identity()`), for the future
  bridge's header population. UUIDv5 derivation remains the fallback where
  no CAL assigns identity (per the starter-kit review recommendation).
- **D7 — Wrapper handling deferred by construction.** Phase-level demos
  use Information ports (no wrapper on the wire) and Request ports whose
  wire shape we control end-to-end (both ends are ours or the harness's).
  Per-variant unwrap onto per-type topics (research doc §3.2) is **not**
  built in this plan — it becomes necessary only when a non-PYRAMID peer
  must consume our Request traffic, and is noted as a follow-on.

---

## 4. Phases

### Phase 0 — Recon and pinning (small)

**Progress:** complete (2026-07-11). See
`subprojects/PYRAMID/pim/test_harness/FINDINGS.md` and
`pim/test_harness/lacal/owp_grammar.md`.

The one phase with no code deliverable; everything after it keys off its
answers.

1. Clone/run the starter kit locally: Sleet
   (`ams-gra-hello-world-sk-infra-sleet`, Rust) standalone first, full
   `podman-compose` Kitty Hawk second (optional). Register a scratch
   service via the `.toml` mechanism.
2. Drive it with `ams-gra-hello-world-sk-test-la-cal-harness` (Python) and
   **capture real `owp` frames** (both the harness's and the kit skills').
3. Pin, from the **official** OMSC-SPC-013 `.docx` (not the markdown
   conversion) cross-checked against Sleet's source:
   - exact `INIT`/`INFO`/`SUB`/`UNSUB`/`PUB`/`MSG`/`+OK`/`-ERR` grammar
     (field order, delimiters, subscription-id rules, group semantics);
   - whether `PUB` carries the message name or it is implied by the
     payload's global-element key;
   - the QoS/buffering text → resolve **D2**;
   - the `INIT.schema` value(s) Sleet accepts, and whether Sleet
     validates payloads against the schema or routes opaque JSON by
     topic — this decides whether Phase 3 can run our `agra_example`
     vocabulary through Sleet or must adopt kit-native UCI messages.
4. Record findings in `pim/test_harness/FINDINGS.md`; confirm Apache-2.0
   posture for any harness fixtures we vendor.

**Exit gate:** frame grammar + schema/validation behaviour documented;
D2 resolved; demo-vocabulary decision made (own contract vs kit UCI
messages — plan assumes *own contract* below and flags the deltas where
the other branch differs).

### Phase 1 — `owp` client core (small/medium)

**Progress:** complete (2026-07-11); mock-server and golden-frame suite green.

A plain C++17 library with **no PCL dependency**, so it unit-tests in
isolation and could serve non-PCL uses later.

1. `subprojects/PYRAMID/src/owp/owp_client.{hpp,cpp}`: websocketpp/asio
   client (both already CMake-fetched at the repo root; today consumed
   only by `ame_foxglove`) with:
   - connect with `Sec-WebSocket-Protocol: owp`;
   - `init(versions, schema, service_id, verbose)` → parsed `Info`
     (UUIDs, server id, negotiated version) or timeout error;
   - `subscribe(sub_id, message_name, topic, group?)` / `unsubscribe`;
   - `publish(topic, body)`;
   - `MSG` dispatch to a caller-supplied callback (documented: fires on
     the worker thread);
   - `+OK`/`-ERR` correlation for verbose mode; `-ERR` carried as status;
   - explicit `close()`; **no auto-reconnect in v1** — a dropped
     connection surfaces as an error state (reconnect policy is a
     deployment concern; noted follow-on).
2. Frame parser/serializer as free functions (space/tab delimiter rules
   from Phase 0) — table-driven and separately testable.
3. `subprojects/PYRAMID/tests/test_owp_client.cpp`: in-process
   websocketpp **mock server** covering handshake happy path, INIT
   timeout, `-ERR` propagation, subprotocol rejection, frame-grammar edge
   cases from the Phase-0 capture corpus.

**Exit gate:** unit suite green with the mock; client round-trips the
captured real-Sleet frames byte-for-byte.

### Phase 2 — LA-CAL transport plugin (medium)

**Progress:** implemented and verified on Linux (2026-07-11). The plugin load,
INIT failure, identity, executor/broker round-trip, content-type guard, and
manifest QoS rollback tests are green. Windows compilation remains to be run
in the Windows CI/toolchain environment.

`subprojects/PYRAMID/plugins/pyramid_lacal_transport_plugin.cpp`, exported
symbols exactly per the ABI (`pcl_plugin.h`), with
`pcl_transport_udp_plugin.c` as the structural model (it is the existing
PUBSUB-only plugin):

1. `pcl_transport_abi_version` (=1), `pcl_transport_plugin_entry`,
   `pcl_transport_plugin_caps` → `PCL_CAP_PUBSUB`,
   `pcl_transport_plugin_qos` → per D2, `pcl_transport_plugin_teardown`.
2. `config_json`:
   `{"url","service_id","schema","verbose","content_type",`
   `"connect_timeout_ms","peer_id","executor"}` — same opaque-JSON
   convention and `executor`-pointer threading as socket/shm/udp.
3. Vtable: `publish` → content-type guard + enqueue `PUB`; `subscribe` →
   `SUB(sub_id, type_name, topic)`; everything else NULL. Ingress: `MSG`
   → deep-copy → `pcl_executor_post_remote_incoming(e, peer_id, topic,
   msg)`.
4. Entry performs D3's fail-closed INIT; identity from `INFO` stored per
   D6.
5. Tests:
   - `test_lacal_transport_plugin_load.cpp` (dlopen + ABI + caps/QoS
     declaration, mirroring `test_grpc_coupled_plugin_load.cpp`);
   - plugin ↔ mock-server round trip through a real executor (publish
     from one executor, deliver into a second's subscriber callback);
   - negative: unregistered `service_id` (mock replies nothing / `-ERR`)
     ⇒ entry returns NULL; RELIABLE-floored route over a BEST_EFFORT
     declaration ⇒ `pcl_transport_routing_load` fails closed (reuses the
     UDP negative pattern from `agra_udp_proof_test`).

**Exit gate:** load test + loopback-via-mock green on Linux; Windows
build compiles (websocketpp/MSVC parity note carried like the other
harness `.bat` files).

### Phase 3 — E2E through real Sleet, kit-native vocabulary (medium)

**Progress:** complete (2026-07-11). The cross-process harness, scratch
service registrations, build integration, and graceful no-Sleet/dead-Sleet
SKIPs are implemented. Against the pinned Sleet `v2026.06.01` container (UCI
2.5 schema loaded), both services INIT-register and the publisher's
schema-valid `PositionReport` (radians + `WGS_HAE`) is routed to and decoded
by the subscriber: `PASS: cross-process PCL -> Sleet -> PCL PositionReport
delivery`. Dead-Sleet and absent-config SKIPs, plus the fail-closed negatives
(unanswered/rejected INIT → NULL entry; RELIABLE floor over BEST_EFFORT →
routing fails closed) are all green. See FINDINGS.md "Phase 3 exit
(2026-07-11)".

Two PCL processes joined **only** by a locally-running Sleet:

1. Register two scratch services (publisher/subscriber sides) in Sleet's
   `.toml`; check a committed example config into
   `pim/test_harness/lacal/` with a README (Sleet itself is *not*
   vendored — the harness locates it via `SLEET_BIN`/`SLEET_URL` env and
   **SKIPs gracefully when absent**, the Ada-E2E precedent).
2. `lacal_shm_analogue` test: the `agra_example` **information topic**
   (`MAActionPlan_Service`, BEST_EFFORT) published from process A,
   received in process B, both routed via a `pcl_transport_routing`
   manifest line
   (`transport asb libpyramid_lacal_transport_plugin.so {...}` +
   `route <topic> publisher|subscriber asb`).
   - If Phase 0 found Sleet validates message names against the UCI
     schema: swap the vocabulary for the kit-native message set chosen in
     Phase 0 (contract tree hand-authored beside `agra_example`,
     same grammar/options discipline) — the test structure is unchanged.
3. Payloads use the existing `application/json` codec **only if** Phase 0
   confirmed Sleet routes opaque bodies; otherwise Phase 4's codec moves
   before this phase. (The content-type guard makes the choice explicit
   either way.)

**Exit gate:** cross-process PCL↔Sleet↔PCL delivery, dead-Sleet SKIP
behaviour, and the fail-closed negatives all demonstrated by
`build_lacal_e2e_test.sh` in `pim/test_harness/` + FINDINGS.md entry.

### Phase 4 — OMS JSON codec + foreign-peer interop (medium)

**Progress:** complete (2026-07-11). The frozen C ABI, `application/oms-json`
plugin, and upstream-derived `SignalReport`/`PositionReport` golden tests are
green (codec prerequisite). Foreign-peer interop now passes **both directions**
against the independently-authored AMS GRA `la-cal-harness` through the pinned
Sleet container: `PASS A` (harness publishes XSD-derived OMS JSON → PCL
typed-decodes) and `PASS B` (PCL publishes → harness subscribes and validates),
driven by `pim/test_harness/lacal/lacal_interop_driver.py` +
`build_lacal_interop_test.sh` (SKIP-safe on absent Sleet/harness/XSD). The
harness's XSD-derived generator and our hand-written codec agree on the wire
shape — the result that earns "interop". See FINDINGS.md "Phase 4 exit
(2026-07-11)".

The step that earns "interop" rather than "loopback through a broker":

1. Implement `application/oms-json` for the demo vocabulary — hand-written
   codec plugin (`pcl_codec_plugin_entry`, ABI v2, operating on the frozen
   `pyramid_<T>_c` structs so C++ and Ada share it), implementing the
   OMSC-SPC-013 mapping: global-element key rule (bare name in the
   `…/programs/oam` namespace), choice representation, numeric edge cases
   (`NaN`/`Infinity` as strings), UUIDs as RFC 4122 strings.
   Structure the tables so a later `pim/backends/oms_json_backend.py`
   (sibling of `json_backend.py`, emitters in `pim/cpp/` mirroring
   `json_codec_gen.py`) can generate them — but **do not build the
   generator in this plan**; two-to-four messages don't justify it, and
   the real generator wants `xsd2proto`'s XSD parse (rung 3).
2. Golden-fixture tests: encode/decode against captured harness payloads
   (Phase 0 corpus) — byte-compare where the spec fixes ordering,
   semantic-compare where it doesn't.
3. Interop test A (we consume foreign): subscribe to a message the Python
   `la-cal-harness` (or, full-kit variant, `rf-fm-demod`'s signal report)
   publishes; decode into typed bindings; assert field fidelity.
4. Interop test B (foreign consumes us): publish from PCL; assert the
   harness's subscriber receives schema-valid OMS JSON.

**Exit gate:** both interop directions green against
independently-authored peers; codec round-trip suite green; SKIP-safe.

### Phase 5 — The interaction seam over LA-CAL (small/medium)

The payoff phase: demonstrate that rung 1 composes with the seam that
motivated it.

1. `lacal_seam_test`: the `agra_seam_interchange_test.cpp` component pair,
   unmodified component code, with the **pub/sub realization of both legs
   routed over the LA-CAL transport** (request + requirement topics
   through Sleet), driven by `contract_routing_manifest.py`-generated
   manifests (`--realize request=pubsub --realize requirement=pubsub`).
2. Mixed-transport variant (the `agra_mixed_route_test` pattern):
   correlated pair over LA-CAL, information topic over UDP — proving
   per-endpoint routing across the new transport.
3. Negative: attempt to realize a leg as RPC over the LA-CAL peer ⇒
   compose-time `PCL_ERR_STATE` (`needs RPC_UNARY but peer "asb" (lacal)
   provides {PUBSUB}`), confirming the capability row.

**Exit gate:** rpc-impossible/pubsub-works matrix demonstrated over the
real broker; `pubsub_interaction_guide.md` §7 table gains the row.

### Phase 6 (stretch, AME-facing) — MS-leg demo

Starter-kit review §5.2 executed over the new plugin instead of a bespoke
spike: kit signal-report/track messages → typed decode → `StateUpdate` →
WorldModel fact grounding, evaluated truth-vs-perceived against the Kitty
Hawk DIS channel. Separately schedulable; it is an AME demo, not
transport work, and must not gate rungs.

---

## 5. Test and harness conventions

- Everything Sleet-dependent lives in `pim/test_harness/` with
  `build_lacal_*.sh` (+ `.bat` parity notes) like the `agra_*` proofs, and
  **SKIPs** — with a printed reason — when `SLEET_URL` is unset or
  unreachable. Mock-server unit tests (Phases 1–2) are CTest-registered
  and unconditional.
- The Phase-0 frame-capture corpus is checked in as fixtures (small,
  text) with provenance comments; licence per Phase-0 check.
- FINDINGS.md gains one dated section per phase exit, same as the SHM/UDP
  proving work.

## 6. Risks and open questions

| Risk | Handling |
|---|---|
| Sleet validates topics/messages against a UCI schema our hand-authored vocabulary isn't in | Phase-0 gate; fallback is the kit-native message set (Phase 3 step 2) — structure unchanged |
| `owp` grammar details differ between the markdown conversion, the official `.docx`, and Sleet's implementation | Phase 0 pins against `.docx` + captured frames; Sleet source is Apache-2.0 and readable |
| QoS honesty (D2) | Phase-0 exit decision from official text; default BEST_EFFORT + explicit route-floor downgrade is always safe |
| Reconnect-less v1 client turns broker restarts into hard failures | Accepted for v1 (fail closed beats silent gaps); reconnect + resubscribe is a listed follow-on |
| Known routing gap: manifest `peer` id not threaded into transports (`transport_codec_plugin_system.md` §known gaps) | Same workaround as the `agra_*` harnesses: `peer_id` in the plugin's own `config_json`; the structural fix stays in TODO WS-D |
| Kit/schema drift (starter kit is "early development", A-GRA/OMS schema versions are siblings) | Pin exact upstream commits + `INIT.schema` value in FINDINGS; surface mismatch, don't coerce |
| Windows parity (podman/Sleet tooling is Linux-first) | Mock-server tests are cross-platform; Sleet E2E documented Linux-only, like the other harness `.sh` proofs |

## 7. Follow-ons deliberately outside this plan

Per-variant Request-wrapper unwrap onto per-type topics (research doc
§3.2) for non-PYRAMID consumers of our Request traffic; the
`oms_json_backend.py` generator (wants `xsd2proto`, rung 3); coupled
single-`.so` packaging; client reconnect/resubscribe policy; transport
health surfacing (CAL connection-status analogue); the C++ CAL plugin
(rung 4) — whose vtable mapping this plugin's Phase-2 code defines.
