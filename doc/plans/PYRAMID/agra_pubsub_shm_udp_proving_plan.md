# A-GRA Example Contract over SHM + UDP — Status Review and Proving Plan

**Scope:** (1) A fresh, evidence-backed status review of
[`pubsub_contract_generation_plan.md`](pubsub_contract_generation_plan.md)
(executed 2026-07-02) against the tree; (2) the gap analysis between that
plan's end state and a *data-plane* proof over real decoupled transports;
(3) a phased plan that ends with an **example A-GRA-vocabulary proto
contract published/subscribed over the shared-memory and UDP transports,
proven end-to-end** — the correlated request/requirement pair and an
information topic actually crossing process and socket boundaries, not just
validating at compose time.

**Date:** 2026-07-08
**Follows on from:** [`pubsub_contract_generation_plan.md`](pubsub_contract_generation_plan.md)
**Related:** [`a_gra_standard_review.md`](../../research/AME/a_gra_standard_review.md),
[`a_gra_e2e_worked_example.md`](../../research/AME/a_gra_e2e_worked_example.md),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md),
`subprojects/PYRAMID/pim/test_harness/FINDINGS.md`,
[`TODO.md`](../../todo/PYRAMID/TODO.md) (WS-E, D-list).

---

## 1. Status review — pubsub plan phases 0–5, re-verified 2026-07-08

Every ledger claim in the executed plan was re-checked against the tree and,
where runnable, re-run in a clean Linux checkout (workspace configured
`-DUNMANNED_BUILD_AME=OFF`, `pcl_core` + `flatc` built). All green:

| Phase | Ledger claim | Re-verification (2026-07-08) |
|---|---|---|
| 0 | Options proto in both trees; parser captures `Interaction`; classifier advisory | `pyramid/options/pyramid.options.proto` present in `proto/` and `pim/test/`; `proto_parser.py` parses `(pyramid.options.pyramid_op)` blocks; `python3 -m unittest subprojects.PYRAMID.pim.test_proto_parser` — 3 tests, OK |
| 1 | MBSE generator stamps options per §5.1 | `proto_generator.py` `_interaction_topic`/option emission confirmed; generated tree carries `pattern`/`topic`/`qos` on every port rpc; regeneration of `pim/test/` from `mbse/test.json` **drift-free** (`git status` clean) |
| 2 | Topics from contract options, side-table scoped to legacy; leak fixed | `viability_check.sh` PASS (48 protos → 201 messages, 14 enums, 40 services; 646 files); `kTopicEntityMatches` **absent** from regenerated output; `binding_manifest.json` carries 76 endpoint requirements, 34 of them contract-derived topics with QoS |
| 3 | Comms harness: information topic + correlated pair + cancel, JSON + FlatBuffers | `build_comms_test.sh` PASS (0 failures), including the sequence-conformance checks (request → requirement transition; cancel → requirement transition) |
| 4 | Contract-derived endpoint requirements; mixed-route compose-time validation | `build_contract_routing_test.sh` PASS — `contract_routing_validation=pass transports=2`; `build_plugin_load_test.sh` PASS |
| 5 | Side-table frozen compat; docs + sequence checks landed | `standard_topics.py` consulted only for the legacy compat layout; TODO.md marks the plan executed; follow-ups B2/B3 closed 2026-07-04 |

**Verdict: the executed plan's ledger is accurate.** The contract layer,
option stamping, topic derivation, manifests, and the in-process runtime
proof all hold up under fresh regeneration and execution. Counts have
drifted slightly from FINDINGS.md's snapshot (203→201 messages, 42→40
services) due to interim tree evolution; behaviour-affecting claims all
reproduce.

## 2. Gap analysis — what "proven over SHM and UDP" still requires

The executed plan proved the *contract* and the *compose-time* story. The
data plane over real decoupled transports is exactly the part still open:

1. **Phase 4's mixed-route proof moved no bytes.** The routing validation
   uses `contract_transport_plugin.c` — a NULL-vtable stub that declares
   `PUBSUB`/`RPC_UNARY` caps from its config and RELIABLE QoS. It proves
   the capability/QoS wiring and fail-closed validation, and nothing else.
   `contract_routing_validation=pass transports=2` is a compose-time
   statement.
2. **The comms test is in-process.** Both containers sit in one executor;
   topic delivery is local dispatch. Contract-generated topic payloads have
   never crossed the decoupled SHM or UDP transports.
3. **SHM/UDP are proven only at the PCL unit level, with raw messages.**
   `test_pcl_shared_memory_transport.cpp` covers real cross-process pub/sub
   (fork/CreateProcess helper, REQ_PCL_186/187), fan-out atomicity, peer
   filters, backpressure; `test_pcl_udp_transport.cpp` covers loopback
   datagram publish/subscribe, peer-filter ingress drops, malformed-datagram
   robustness, and `NoServiceRpcSupport`. None of it uses generated typed
   helpers, codec plugins, or contract-derived routes.
4. **QoS reconciliation has never met a genuinely BEST_EFFORT transport.**
   The MBSE generator stamps `reliability: RELIABLE` on every topic (role
   defaults); the UDP plugin declares `PCL_QOS_RELIABILITY_BEST_EFFORT`.
   Routing any currently-stamped topic over UDP must fail closed — correct
   behaviour, but it means the UDP leg of this plan *requires* a contract
   whose information topic is legitimately BEST_EFFORT (§5.4's "unless the
   model marks it telemetry-like" branch, which was specified but never
   exercised) or an explicit deploy-time floor decision in the manifest.
5. **No A-GRA-vocabulary contract exists.** `pim/test/` speaks PIM
   Osprey/Seaspray. The A-GRA bridge argument (pubsub plan §7 — "mostly
   renaming, not re-plumbing") has no concrete contract artefact behind it
   yet.
6. **Windows parity for the newer harnesses is authored but unproven**
   (carried note from the executed plan; this Linux environment cannot
   discharge it either — it stays a carried note).

Everything needed to close these already exists as machinery: the routing
manifest loader injects the executor pointer into plugin configs and stands
up real plugins (`pcl_transport_routing.c` `inject_executor`), the SHM
plugin takes `bus_name`/`participant` config with
`PUBSUB|RPC_UNARY|RPC_STREAM` caps and RELIABLE QoS, the UDP plugin is
PUBSUB-only BEST_EFFORT, and `contract_routing_manifest.py` already derives
route lines from `binding_manifest.json`. The remaining work is a contract
plus harnesses.

## 3. Design decisions for the example contract

### 3.1 Source form: hand-authored, grammar-conforming, options-stamped

The example contract is a **hand-authored proto tree**
`subprojects/PYRAMID/pim/agra_example/`, not an MBSE model extension:

- `mbse/test.json` is exported from the SysML model; hand-inserting an
  A-GRA package there would fake a model round-trip we don't own. A
  hand-written tree is a supported pipeline input (the legacy tree is one),
  and it faithfully mirrors the real situation an `agra_c2_bridge` drop
  would be in: an external-standard-shaped contract entering our toolchain.
- The tree conforms to the port grammar (4-rpc Request shape / 1-rpc
  Information shape) **and** carries explicitly stamped
  `pyramid.options.pyramid_op` options — so it exercises the authoritative
  Layer-2 path, while a strip-the-options variant in the parser tests
  exercises the classifier fallback on the same services.
- It vendors byte-identical copies of `pyramid/options/pyramid.options.proto`
  and the `pyramid/data_model/{base,common}` protos it needs (`Ack`,
  `Query`, `Identifier`, `Requirement`/`Achievement` with the acceptance
  layer), guarded by a checksum test against the `pim/test/` originals so
  the copies cannot drift silently. The tree stays independently
  consumable, like a contract drop should be.

If/when the SysML model grows an A-GRA package, the tree regenerates from
it via the normal MBSE path and this section's decision is superseded.

### 3.2 Vocabulary and shape

One provider component (`agra.mission_autonomy`, the MA side) and one
consumer (`agra.c2_station`), C2 ↔ MA per the worked example:

```proto
// Request port — Command-2/ActionRequest-2 correlated pair
service MAAction_Service {
  rpc Create(MAAction_Service_Request) returns (agra.data_model.common.Ack);
  rpc Read(agra.data_model.common.Query) returns (stream MAAction_Service_Requirement);
  rpc Update(MAAction_Service_Requirement) returns (agra.data_model.common.Ack);
  rpc Cancel(agra.data_model.base.Identifier) returns (agra.data_model.common.Ack);
}

// Information port — Data-1
service MAActionPlan_Service {
  rpc Read(google.protobuf.Empty) returns (stream MAActionPlan_Service_Information);
}
```

- `MA_Action` payload: `action_type` (FIND_SEARCH, …), `target_object`,
  `action_constraints` (EMCON/keep-out strings suffice for the proof) —
  the worked example's tasking message.
- The request wrapper's `cancel` variant is the `ActionCancelCommand`
  analogue; requirement transitions carry `Achievement` with the
  acceptance layer (RECEIVED/REJECTED + reason — the
  `ActionProcessingState` analogue) so `MA_ActionStatus` is a requirement
  transition, exactly the pubsub plan §3 mapping.
- `MA_ActionPlan` payload on the information topic: plan id, correlated
  action id, summary — the Data-1 publication C2 reads before approving.

### 3.3 Topics and QoS — the load-bearing choices

| Topic | Role | Stamped QoS | Intended transport |
|---|---|---|---|
| `agra.ma_action.request` | pair, C2→MA | RELIABLE, VOLATILE, depth 10 | SHM |
| `agra.ma_action.requirement` | pair, MA→C2 | RELIABLE, VOLATILE, depth 10 | SHM |
| `agra.ma_action_plan.information` | Data-1, MA→C2 | **BEST_EFFORT**, VOLATILE, depth 10 | UDP |

The BEST_EFFORT stamp on the information topic is deliberate and is what
makes the UDP leg contractually legitimate: it exercises the §5.4
reconciliation rule (contract floor ≤ transport ceiling) in the passing
direction against a real BEST_EFFORT transport, while the RELIABLE pair
over UDP is the negative test that must fail closed. Since the tree is
hand-authored, no generator change is needed to stamp it; teaching
`proto_generator.py` a telemetry-like model marking stays a recorded
follow-up for the MBSE path (TODO D-list).

A-GRA's own topic discipline (MA-L1-013: topic name == wrapped message
type name) differs from our `<project>.<interface>.<role>` scheme. The
contract option carries the topic verbatim, so an A-GRA-conformant name is
*expressible* today; we keep our scheme for the example (it is our
contract, not an OTA DMS artefact) and record the rename as a deploy-time
bridge concern. This is a one-line note in the example's README, not a
mechanism to build.

## 4. Phased plan

Standing requirements throughout: legacy tree and `pim/test/` outputs stay
byte-identical; every new harness has `.sh` + `.bat` parity authored; Ada
object-compile parity for anything the example generates; evidence recorded
in §6's ledger on completion of each phase.

### Phase A — routed-egress plumbing proof (enabler)

Prove that contract-generated typed topic helpers actually traverse a
*real* transport stood up from a routing manifest — using the existing
`pim/test/` contract, before any new contract exists. This de-risks the
one genuinely unknown seam: container port → executor route → plugin
egress → remote executor ingress (`pcl_executor_post_remote_incoming`) →
subscriber callback, with topic wire names as endpoint names end-to-end.

- Extend the routing harness: two executors (same process first), each
  loading `libpcl_transport_shared_memory_plugin.so` via
  `pcl_transport_routing_load` manifests generated by
  `contract_routing_manifest.py`, sharing a bus; drive the existing
  `pim_osprey` request/requirement pair through the generated
  `publish*`/`subscribe*` helpers with the JSON codec plugin.
- Then split provider and consumer into two processes (reuse the
  `pcl_shm_peer_helper.cpp` spawn pattern).
- Fix whatever this surfaces (expected candidates: local-vs-remote route
  modes on generated publisher ports, endpoint-name ↔ topic-wire-name
  mismatches in manifest emission, plugin teardown ordering) — each fix in
  PCL or the generators, never in the harness alone.
- **Accept:** contract-generated payloads observed crossing a real SHM bus
  between two processes; existing in-process comms test still green;
  no generated-output diffs outside the harness.

### Phase B — the A-GRA example contract

- Author `pim/agra_example/` per §3: vendored base/common/options protos
  (checksum-guarded), `agra.data_model` types, `mission_autonomy` provided
  + `c2_station` consumed service files, options stamped per §3.3, README
  documenting the A-GRA correspondence table (pubsub plan §3) and the
  MA-L1-013 note.
- Parser tests: both services classify correctly with options present;
  identically with options stripped (classifier fallback); the BEST_EFFORT
  stamp survives into `ProtoRpc.qos`.
- Generate bindings (`--contract-layout generic`); manifest must carry the
  three topics with their distinct reliability floors (`reliable`,
  `reliable`, `best_effort`).
- **Accept:** generation clean for C++ and Ada (object-compile bar);
  manifest endpoint requirements as above; legacy + `pim/test/`
  regeneration byte-identical throughout.

### Phase C — SHM proof: the correlated pair, cross-process

New harness `agra_shm_comms_test` (`.sh`/`.bat`): MissionAutonomy provider
process and C2Station consumer process, each with its own executor and
routing manifest loading the real SHM plugin on a shared, per-run-unique
bus name. Scenario, mirroring the worked example's sequence:

1. C2 publishes `MA_Action` (FIND_SEARCH, target, constraints) on
   `agra.ma_action.request`.
2. MA observes it, publishes requirement transitions on
   `agra.ma_action.requirement`: acceptance RECEIVED → progress →
   COMPLETE; C2 observes each.
3. C2 publishes the cancel variant (`ActionCancelCommand` analogue) for a
   second action; MA publishes the CANCELLED transition; C2 observes it.
4. Sequence-conformance checks (A-GRA-harness style, as in the existing
   comms test): every request followed by ≥1 correlated requirement
   transition; cancel followed by a further transition; correlation by
   `Entity.id`/`source`, flat topic space.
5. JSON codec plugin first, FlatBuffers second (same carried note as
   before: the FlatBuffers wrapper path bridges via the JSON closure).

- **Accept:** green on Linux from a clean generate; compose-time
  validation passes using only contract-derived endpoint requirements
  against the real plugin's declared caps/QoS; `.bat` parity authored.

### Phase D — UDP proof, including the fail-closed negative

1. **Negative gate first:** a manifest routing
   `agra.ma_action.requirement` (RELIABLE floor) over
   `libpcl_transport_udp_plugin.so` (BEST_EFFORT) must fail closed at
   `pcl_transport_routing_load` with the precise-diagnostic style — the
   first exercise of §5.4 against a real transport. Assert on the
   diagnostic text.
2. `agra.ma_action_plan.information` (BEST_EFFORT floor) over UDP between
   two processes on loopback ports: MA publishes plan objects, C2
   subscribes and decodes; peer-filter ingress respected.
3. Document (README + harness comment) the explicit deploy-time downgrade
   path — an operator writing `best_effort` as the route's reliability
   floor to carry the pair over UDP — as policy, demonstrated by one
   harness case, not silently defaulted.

- **Accept:** negative case fails closed with diagnostic; information
  topic round-trips over real UDP datagrams; `.bat` parity authored.

### Phase E — the endgame: one system, both transports, full scenario

New harness `agra_mixed_route_test` (`.sh`/`.bat`), the terminal
acceptance for the whole plan: MissionAutonomy and C2Station processes
each load **two** transports from one routing manifest —

```
transport pair_shm  libpcl_transport_shared_memory_plugin.so {"bus_name":"agra_<run>","participant":"ma"}
transport info_udp  libpcl_transport_udp_plugin.so           {"bind":"127.0.0.1:<port>", ...}

route agra.ma_action.request           subscriber pair_shm  reliable
route agra.ma_action.requirement       publisher  pair_shm  reliable
route agra.ma_action_plan.information  publisher  info_udp  best_effort
```

— and run the complete worked-example sequence: action tasking and status
over the SHM correlated pair, plan publication over UDP, cancel, all
sequence checks, both codecs. This is the A-GRA C2/MS split in miniature:
reliable command-and-status alongside best-effort data publication, every
topic, direction, and QoS floor derived from the contract.

- **Accept (plan-terminal):** `agra_mixed_route_test` green on Linux from
  a clean checkout (`generate → compose-time validation → run`), with the
  evidence ledger (§6) filled in: transports=2 *with bytes moved on both*,
  sequence checks pass, negative gate replayed, Ada parity held, legacy +
  `pim/test/` byte-identical. Update FINDINGS.md, TODO.md (WS/D-list
  rows), and the pubsub plan's ledger pointer.

### Risks

- **The Phase A seam is the real unknown.** If generated publisher ports
  only dispatch locally (route-mode gaps), the fix lands in PCL/executor
  routing, sized small-to-medium; Phase A exists to surface this before
  the contract work stacks on it.
- **UDP subscribe is receive-all with client-side topic dispatch** (the
  vtable `subscribe` is a no-op); fine on loopback, but the harness must
  bind per-run unique ports to survive parallel CI.
- **SHM bus naming collisions** across concurrent runs: per-run-unique
  bus names (pid/timestamp), as the PCL unit tests already do.
- **Vendored proto drift** in `agra_example/`: the checksum test is the
  guard; if it fires, re-vendor, never fork.
- **FlatBuffers wrapper bridge** still routes via the JSON closure
  (carried note); the example keeps JSON as the primary codec and
  FlatBuffers as the second witness, same as the executed plan.
- **Windows parity remains authored-not-run** in this environment; carried
  note, discharged next time a Windows checkout runs the harness set.

## 5. Relationship to the A-GRA adoption path

This plan turns the pubsub plan's §7 claim into an artefact: after Phase E
there exists a concrete, regenerable, A-GRA-vocabulary contract whose
correlated-pair and information topics demonstrably run over two real
transports with contract-derived routing and QoS. The `agra_c2_bridge`
(review §5.2/§7) then starts from a proven contract shape — its remaining
work is vocabulary mapping at the boundary and the approval gate, with the
DMS/DDS transport and EXI codec entering later as one more plugin pair
under the same routing manifests used here.

## 6. Evidence ledger

To be filled per phase on execution, in the style of the pubsub plan's
progress ledger (commands run, pass/fail, counts, diffs). Empty at
plan-authoring time (2026-07-08); §1 above records the pre-plan baseline
evidence.

### Phase A — executed 2026-07-08

New harness: `subprojects/PYRAMID/pim/test_harness/routed_egress_shm_test.cpp`
+ `build_routed_egress_test.sh`/`.bat`. Reuses the existing `pim/test`
contract's `pim_osprey.sensor_products` request/requirement/information
topics (the same vehicle `components_comms_test.cpp` already exercises
in-process) driven through the generated `publish*`/`subscribe*` helpers and
the JSON codec, but routed via `pcl_transport_routing_load` loading the real
`libpcl_transport_shared_memory_plugin.so` from a hand-authored manifest
(`contract_routing_manifest.py` is tuned for the NULL-vtable
`contract_transport_plugin.c` stub and doesn't emit `bus_name`/
`participant_id` config, so this harness writes its own manifest text
per `pcl_transport_routing.h`'s documented grammar).

Two scenarios, both green on Linux from a clean build
(`bash build_routed_egress_test.sh`):

1. **Two executors, one process, one SHM bus.** Each executor loads its own
   instance of the SHM plugin via `pcl_transport_routing_load`, sharing a
   `bus_name`. Full request/requirement/information round trip plus the
   cancel-transition conformance check, all delivered over the real bus (not
   local dispatch) — verified by asserting on both sides' observed state, not
   just the publish-call return code.
2. **Same routing setup split across two OS processes**, `fork()` +
   `execv()` of the harness binary itself with `--role=provider|consumer`
   (mirroring `pcl_shm_peer_helper.cpp`'s spawn/ready-file/result-file
   synchronization pattern, but driving the manifest-routed generated
   helpers instead of the direct transport API). Both processes exit 0 and
   their result files confirm the request, information, and both
   requirement transitions (create + cancel) crossed the real inter-process
   SHM bus.

**Seam bug found and root-caused (harness/manifest-authoring level, not a
PCL defect):** naming both sides' local transport peer alias `shm_peer`
made the same-process scenario silently drop every subscriber-side message.
`pcl_executor_validate_endpoint_route` requires a route's peer names to
match a *locally-registered* transport peer (confirmed intentional and
pinned by `test_pcl_capabilities.cpp`'s `ValidateRouteFailsClosedOnMissingPeer`/
`ValidatePubsubEndpointOverPubsubTransport`), while the executor's runtime
ingress filter (`peer_is_allowed` in `pcl_executor.c`) compares route peer
names against the *remote sender's* participant identity, since the SHM
plugin posts `pcl_executor_post_remote_incoming(exec, frame->source_id,
topic, msg)`. Those are two different namespaces that happen to share one
field. The fix (in the harness's manifest generation, not PCL): alias each
side's local transport peer name after its *counterpart's* actual
`participant_id` (e.g. the consumer's manifest registers its SHM transport
under the local alias `"provider"`), which satisfies both checks at once for
a two-party correlated pair. Recorded here as a real, load-bearing
authoring convention for any future manifest (hand-authored or generated)
that routes a shared multi-participant bus transport (SHM today; would also
apply to any future N:1 pub/sub bus plugin) — `contract_routing_manifest.py`
does not need to change for this plan's remaining phases since Phase B/C's
manifests will be hand/harness-authored the same way, but this is a D-list
candidate if/when route derivation is taught to target SHM.

**Regression check:** `build_contract_routing_test.sh`
(`contract_routing_validation=pass transports=2`) and `build_comms_test.sh`
(JSON + FlatBuffers rounds, 0 failures) both re-ran green after this change;
no generated-output diffs outside the new harness's own scratch
directories (`generated_routed_egress/`, `routed_egress_scratch/`, both
gitignored).

**Accept criteria met:** contract-generated payloads (via the existing
`pim_osprey` contract, not a new one) observed crossing a real SHM bus
between two processes, loaded from a routing manifest driving the real
plugin; existing in-process comms test still green; no generated-output
diffs outside the harness. Phase A's purpose — de-risking the
container-port -> executor-route -> plugin-egress -> remote-executor-ingress
-> subscriber-callback seam before the new A-GRA contract work stacks on
it — is discharged: the seam works via the manifest-routing path, provided
the peer-alias convention above is followed.

**Carried note:** Windows parity (`build_routed_egress_test.bat`) authored,
not run, in this Linux environment — same carried note as the rest of this
plan's Windows story.

### Phase B — executed 2026-07-08

New tree: `subprojects/PYRAMID/pim/agra_example/` (5 proto files: vendored
`pyramid.options`/`pyramid.data_model.{base,common}`, new
`pyramid.data_model.agra` with `MA_Action`/`MA_ActionPlan`, and the
`agra.mission_autonomy` provided / `agra.c2_station` consumed service
files per §3.2/§3.3) + `README.md` (A-GRA correspondence table, MA-L1-013
topic-naming note) + `pim/test_agra_example.py` (checksum guard + parser
classification tests).

`python3 pim/generate_bindings.py pim/agra_example <out> --languages cpp
--backends json` -- clean: 5 proto files, 38 messages, 7 enums, 4 services,
16 files generated, `binding_manifest.json`'s `topics`/`endpoint_requirements`
carry exactly the three topics from §3.3 with the right floors:

```
agra.ma_action.request            reliable
agra.ma_action.requirement         reliable
agra.ma_action_plan.information    best_effort
```

`--languages ada` generation also clean (13 files); object-compiling those
requires GNAT/gprbuild, not installed in this environment -- carried note,
same as the plan's other Ada/Windows items (matches the existing
`pyramid_ada_all` carried-note convention in `CLAUDE.md`).

`python3 -m unittest subprojects.PYRAMID.pim.test_proto_parser
subprojects.PYRAMID.pim.test_agra_example` -- 6 tests, OK: both services
classify correctly with options present (request/information, correct
topic + QoS incl. the BEST_EFFORT stamp surviving into `ProtoRpc.qos`);
identically with a copy of the provided-side proto stripped of every
`pyramid_op` option block (classifier fallback); the three vendored files
are byte-identical (sha256) to their `pim/test/` originals.

**Accept criteria met:** generation clean for C++ and Ada (object-compile
bar carried as a note, per above); manifest endpoint requirements as
specified; legacy (`proto/`) and `pim/test/` untouched and regenerate
byte-identical throughout (this phase only added new files under
`pim/agra_example/` and `pim/test_agra_example.py` -- `git status` confirms
no modification to any existing generated-output-affecting file).
