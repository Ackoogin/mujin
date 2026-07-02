# Pub/Sub Contract Generation Plan — A-GRA-Informed Alignment of the PCL/PYRAMID Bindings

**Scope:** (1) A close reading of A-GRA's service-over-pub/sub mechanism and
why it matters to us; (2) an honest inventory of where PCL/PYRAMID pub/sub
bindings actually stand (weakly defined today); (3) options — including
skewing toward pub/sub at the MBSE layer itself — and a concrete, phased plan
to generate pub/sub bindings from the new-shape proto contracts
(`subprojects/PYRAMID/pim/test/`), whose port grammar is adequately congruent
with A-GRA's interaction primitives.

**Date:** 2026-07-02
**Split out from:** [`a_gra_e2e_worked_example.md`](../../research/AME/a_gra_e2e_worked_example.md)
§8 (now a pointer here).
**Related:** [`a_gra_standard_review.md`](../../research/AME/a_gra_standard_review.md) §2.3/§6.3,
[`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md),
[`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md),
`pim/test_harness/FINDINGS.md`.

---

## 1. A-GRA's service-over-pub/sub mechanism

A-GRA has **no RPC anywhere**. Its 841-message surface reduces to six
interaction primitives (review §2.3), every one realised as published data
objects on DDS topics. The mechanics, in detail:

### 1.1 Request/response as correlated object pairs

- **Command-2 / ActionRequest-2 / DataRequest-2** are each a *pair of
  published objects*: the request (`PO_Command`, `MA_ApprovalRequest`,
  `MA_MissionPlanCommand`, …) and a separate status message
  (`PO_CommandStatus`, `MA_ApprovalRequestStatus`, …) that carries the
  request's ID plus a processing-state enum and a reason
  (`ApprovalRequestProcessingState`, `CommandProcessingStateReason`, …).
  There is no synchronous ack: acceptance, rejection, progress, and
  completion are all state transitions on the published status object.
- **Requests are first-class, addressable, persistent objects.** A command
  has its own lifecycle, survives its issuer, can be observed by any
  authorised subscriber, and is cancelled by *another message*
  (`ActionCancelCommand`) with explicitly specified race semantics.
  Cancelled objects are retained for history/debrief.
- **Query results are just more publications** (Data-1) correlated by the
  query's ID.

### 1.2 Topic and envelope discipline

- **Topic name == wrapped message type name, exactly** (compliance rule
  MA-L1-013). No per-instance or per-connection topics.
- **One mandated generic wrapper** (`MA_DataPayloadWrapper`, MA-L1-012)
  carries every payload over DDS; EXI-encoded UCI inside.
- **Instance disambiguation is by header fields, not topics**: every message
  carries `SystemID`/`ServiceID`/`MissionID` in its `MessageHeader`, and
  consumers filter. This is the load-bearing trick that keeps the topic
  space flat and contract-derivable — multiple providers of the same
  capability share a topic and are told apart by ID.
- **CRUD without RPC:** the `ObjectState` NEW/UPDATED/REMOVED enum on
  published objects signals create/update/delete from the publisher's side.

### 1.3 Why this matters to us

- It is a **proof at CCA-program scale** that a complete service surface —
  planning, approval, capability command, status — can be expressed as
  correlated topic pairs over a pub/sub-only middleware.
- Our own plugin capability model
  ([`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md))
  already records that **DDS/MQTT and UDP provide `PUBSUB` but not
  `RPC_UNARY`**, and defers an `RPC_UNARY over PUBSUB` adapter as future
  opt-in work. A-GRA's correlated-pair pattern *is* the specification for
  that adapter.
- Everything published is observable and replayable — sequence-level
  conformance testing (the A-GRA L1 harness approach) and debrief fall out
  of the transport model instead of being bolted on.
- For the A-GRA bridge itself (review §5.2/§7): if EntityActions has a
  native correlated-topic projection, `agra_c2_bridge` becomes mostly
  *renaming* (`PlanningRequirement` → `MA_Action` vocabulary), not
  *re-plumbing* (RPC ↔ pub/sub adaptation).

Costs A-GRA accepts, which we must too if we adopt the pattern: retention/GC
policy for request objects (A-GRA deliberately keeps cancelled objects),
idempotent re-publication semantics, correlation-ID discipline in every
requirement type, and per-topic QoS decisions.

---

## 2. Where PCL/PYRAMID pub/sub bindings actually are (and why "weakly defined" is fair)

### 2.1 The runtime primitives are minimal

PCL's pub/sub surface is two untyped calls:
`pcl_container_add_subscriber(container, topic, callback, …)` and
`pcl_port_publish(port, pcl_msg_t*)`
(`subprojects/PCL/include/pcl/pcl_container.h`). Content-type aware, codec
agnostic, but no typed callback surface and no contract linkage — everything
above it is the generator's job.

### 2.2 Topics are not in the contract — and the side-table leaks

Topic wire names, payload types, and service↔topic bindings live in a JSON
side-table (`pim/topic_metadata/tactical_objects_topics.json`, loaded by
`pim/standard_topics.py`), matched to services by **substring** of the
package name (`package_match: "tactical_objects"`). Only the Tactical
Objects compatibility set exists.

This is not hypothetical weakness. The substring match **leaks across proto
trees**: generating the new PIM tree produces, in
`pyramid_services_pim_osprey_tactical_objects_provided.hpp`,

```cpp
constexpr const char* kTopicEntityMatches = "standard.entity_matches";
using pyramid::domain_model::common_pim_components::tactical_objects::ObjectMatch;
```

— legacy wire names and topic helpers stamped into `pim_osprey`'s
tactical-objects bindings, resolving against a *different* `ObjectMatch`
type that happens to share the short name. It compiles by coincidence, not
by contract.

Further gaps: no QoS anywhere in the topic path; topic direction
(publish vs subscribe) is asserted by the side-table rather than derived
from provided/consumed; the plugin capability model validates
endpoint↔transport fit at compose time but the *generator* never checks
that a contract's topics and services can be carried by the intended
transport mix.

### 2.3 The interaction-semantics proposal — right idea, partially overtaken

[`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md)
correctly diagnosed the split source of truth and proposed a two-layer
convention (signature inference + authoritative `pyramid.options.Interaction`
method option carrying `pattern`/`topic`/`qos`). Two developments have
overtaken parts of it:

1. **The new contract shape makes patterns derivable without annotation.**
   The proposal treats `rpc Read(Empty) returns (stream T)` as *ambiguous*
   (subscribe vs argument-less server-stream) and requires an option. That
   is true for free-form services — but the MBSE-generated tree
   (`pim/test/`, emitted by `pim/mbse/proto_generator.py`) has a strict
   **port grammar** (§3) in which that shape is the *canonical, unambiguous*
   Information port. Pattern inference over the port grammar needs no
   per-method annotation at all.
2. **The plugin system landed.** The proposal's transport-projection table
   predates the capability model (`PUBSUB`/`RPC_UNARY`/`RPC_STREAM` caps,
   per-endpoint routing manifests, compose-time fail-closed validation) and
   QoS now has two candidate homes — the contract option and the plugin QoS
   profile — that need one reconciliation rule (§5.4).

Its migration plan ("author `PublishX`/`SubscribeX` topic methods for each
side-table entry, then retire `standard_topics.py`") remains valid **for the
legacy tree only**; the new tree should never grow hand-authored topic rpcs.

---

## 3. The new contract shape, and its congruence with A-GRA

`pim/mbse/proto_generator.py` (SysML XMI → JSON → proto) emits, per
component, `services.provided` / `services.consumed` files whose services
come from model **ports** in exactly two shapes
(`_write_one_port_service`):

```proto
// Request port 'pSENRequirement' (provided) refining SENRequirement
service SENRequirement_Service {
  rpc Create(SENRequirement_Service_Request) returns (common.Ack);
  rpc Read(common.Query) returns (stream SENRequirement_Service_Requirement);
  rpc Update(SENRequirement_Service_Requirement) returns (common.Ack);
  rpc Cancel(base.Identifier) returns (common.Ack);
}

// Information port 'pCapability' (provided) refining Capability
service Capability_Service {
  rpc Read(google.protobuf.Empty) returns (stream generic.Capabilities);
}
```

with service-local `oneof` wrapper messages
(`X_Service_Request`, `X_Service_Requirement`, `X_Service_Information`)
when the port's payload has refined variants. The grammar is closed: **every
service is either a Request port (4 fixed rpcs) or an Information port
(1 fixed rpc)**, and direction comes from the file (`provided`/`consumed`).

That is adequately similar to A-GRA's primitive set for a mechanical mapping:

| PIM port grammar | A-GRA primitive | Pub/sub realisation |
|---|---|---|
| Information port, provided | Data-1 / Status-1 | provider publishes payload stream on the information topic |
| Information port, consumed | Data-1 / Status-1 (consumer end) | subscribe to the same topic |
| Request port `Create`/`Update` | Command-2 / ActionRequest-2 (request half) | publish request object on the request topic |
| Request port `Read(Query) → stream Requirement` | Command-2 status half + DataRequest-2 | subscribe to the requirement topic; requirement objects carry embedded status (`Requirement.base.status: Achievement`) — the `*Status`/`*RequestStatus` analogue |
| Request port `Cancel(Identifier)` | `*CancelCommand` | publish cancel object on the request topic (or a `cancel` role topic); confirmation is a requirement-object transition |
| `Ack` return | (none — A-GRA has no sync ack) | derived on RPC transports; **replaced** by a requirement/status transition on pub/sub transports |
| `oneof` payload wrappers | UCI abstract-type choice groups | wrapper message is the topic payload type |
| `Entity.id`/`source` | `MessageHeader.SystemID` + object IDs | correlation and instance filtering by ID, flat topic space |

The one semantic addition pub/sub needs that the RPC shape gets for free:
an **acceptance state**. `Achievement.Progress` starts at execution; A-GRA's
ProcessingState covers RECEIVED/REJECTED-with-reason before that. A small
extension (either an `AcceptanceState` on `Achievement`, or reserving
`PROGRESS_*` values) closes it — after which the `Ack` rpc return is fully
derivable.

---

## 4. Options

### Option A — derive pub/sub at the binding-generation layer

Teach `pim/proto_parser.py` a port-grammar classifier (a service matching
the 4-rpc Request shape or the 1-rpc Information shape gets a `port_kind`),
and have `cpp_codegen.py`/`ada_codegen.py` synthesise topics from
`port_kind` × `provided|consumed` — replacing the JSON side-table for
grammar-conforming trees.

- *Pros:* no contract or MBSE change; works today on `pim/test/` as-is;
  the old tree keeps the side-table.
- *Cons:* the pattern is *inferred* at every consumer of the contract
  (our generator, but also any third party reading the `.proto`); a
  free-form service that accidentally matches the shape gets topics it
  didn't ask for; topic names/QoS have no contract home.

### Option B — annotate the contract (Layer-2 options, hand-maintained)

Implement `pyramid.options.Interaction` per the interaction-semantics doc
and hand-annotate services.

- *Pros:* explicit, self-describing contract; covers free-form services.
- *Cons:* hand-maintenance across ~42 generated services (and growing) is
  exactly the side-table problem re-homed; the generated tree would be
  edited post-generation, which the pipeline forbids (generated protos are
  regenerated, not patched).

### Option C — skew toward pub/sub at the MBSE layer (`pim/mbse`)

The MBSE layer is where port kind and direction are **ground truth** — the
generator does not infer them, it *knows* them from the SysML port
stereotype before any proto exists. Two sub-options:

- **C1 — MBSE stamps interaction options.** `proto_generator.py`'s
  `_write_one_port_service` emits the Layer-2 option block mechanically on
  every rpc it writes: `pattern` (PUBLISH/SUBSCRIBE/COMMAND role),
  `topic` (derived per §5.1), `qos` (role defaults). The contract stays
  RPC-shaped — stock `protoc`/gRPC consumers are untouched (custom options
  are transparent metadata) — but now carries authoritative pub/sub
  semantics that no downstream layer has to guess.
- **C2 — MBSE emits a pub/sub-first contract.** Drop the rpc form for
  ports altogether: emit topic-pair declarations (request topic message +
  requirement topic message + naming/QoS manifest), A-GRA style, and make
  RPC the derived projection.

Compatibility assessment (the criterion the option was raised on):

| Consumer | A (infer) | B (hand-annotate) | C1 (MBSE-stamped options) | C2 (pub/sub-first) |
|---|---|---|---|---|
| Stock gRPC client, `.proto` as-is | ✓ | ✓ | ✓ (options transparent) | ✗ (no service to consume — needs a generated RPC projection) |
| PCL runtime (socket/shm/in-proc) | ✓ | ✓ | ✓ | ✓ |
| ROS2 envelope + native IDL | ✓ (inferred topics) | ✓ | ✓ (authoritative topic/QoS) | ✓ |
| DDS/MQTT/UDP (PUBSUB-only transports) | ✓ | ✓ | ✓ | ✓ (native shape) |
| A-GRA/DMS bridging | adapter | adapter | adapter, spec'd by options | most direct |
| Third-party reading the contract | must re-implement inference | explicit | explicit | explicit but non-standard (no proto idiom for topics) |
| Existing bindings/tests (`pim/test_harness` green suite) | unchanged | unchanged | unchanged output modulo options | breaks; full regeneration of every backend |

**C2 is rejected for now**: it maximises pub/sub purity but forfeits the
"stock consumer uses the contract as-is" property that is the whole reason
pub/sub was modelled as rpc (interaction-semantics doc, *Direct Consumer
Compatibility*), and it invalidates the entire verified v1 surface
(FINDINGS.md: 592/592 green) for no capability we cannot get from C1.
Revisit only if a hard DMS/DDS compliance target makes the pub/sub-first
form the delivered artefact.

### Recommendation: **C1 + A**, layered

MBSE stamps options (C1) as the authoritative layer for everything it
generates; the binding generator carries the port-grammar classifier (A) as
the inference fallback for contracts that lack options (hand-written protos,
older drops). This is exactly the Layer-1/Layer-2 resolution rule the
interaction-semantics doc already defines — with Layer-2 now *machine-
written at the MBSE layer* instead of hand-authored, which dissolves that
doc's main adoption cost. The legacy tree keeps the JSON side-table until
its own migration (§6, Phase 5).

---

## 5. Topic model for the port grammar

### 5.1 Naming — derived from the IDL, nothing hardcoded

Rule (consistent with the repo-wide "generators never hardcode domain
names" constraint): the topic base is derived from the **interface-block
identity**, which is the one name both ends of a wired port pair share
(the service name, minus `_Service`), scoped by project:

```
<project>.<interface_snake>.<role>
role ∈ { request, requirement, information }
```

Examples from `pim/test/`:

| Port | Topics |
|---|---|
| `SENRequirement_Service` (pim_osprey.sensors, Request port) | `pim_osprey.sen_requirement.request`, `pim_osprey.sen_requirement.requirement` |
| `Capability_Service` (pim_osprey.sensors, Information port, provided) | `pim_osprey.capability.information` (publish) |
| `Capability_Evidence_Service` (pim_osprey.sensors, consumed) | `pim_osprey.capability_evidence.information` (subscribe) |

Direction table (who publishes / who subscribes):

| Port kind × side | request topic | requirement topic | information topic |
|---|---|---|---|
| Request, provided | subscribe | publish | — |
| Request, consumed | publish | subscribe | — |
| Information, provided | — | — | publish |
| Information, consumed | — | — | subscribe |

`Cancel` publishes on the request topic (an Identifier-bearing cancel
object; the wrapper message gains a `cancel` variant), mirroring A-GRA's
cancel-as-message; alternatively a fourth `cancel` role topic if mixing
payload kinds on one topic proves awkward in Phase 3.

**Instance scoping.** Like A-GRA (flat topics, filter by `SystemID`), the
default is one topic per interface, with consumers filtering by
`Entity.source`/`id`. Where a deployment genuinely needs per-instance
isolation, the PCL routing manifest already names peers/endpoints — a
deploy-time topic-prefix override belongs there, **not** in the contract.

### 5.2 Payload types

The topic payload is exactly the rpc payload: the service-local `oneof`
wrapper where one exists (`SENRequirement_Service_Request`), the bare
data-model type otherwise. No new envelope — the existing PCL
`content_type` + codec-plugin model carries it, and the transport envelope
(`correlation_id`, `status`, `end_of_stream`) already has the fields the
correlated-pair pattern needs.

### 5.3 Status/acceptance semantics

- Requirement objects carry `Requirement.base.status : Achievement` —
  progress/quality/feasibility already modelled.
- Add an acceptance layer (RECEIVED / REJECTED + reason — A-GRA
  ProcessingState analogue) so that on pub/sub transports the `Ack` return
  is replaced by a requirement transition, and on RPC transports the `Ack`
  is derived from the same transition. One semantic, two projections.

### 5.4 QoS — one rule for two homes

Contract options carry **intent** (role defaults: request/requirement =
RELIABLE, information = RELIABLE unless the model marks it telemetry-like);
plugin QoS profiles carry **transport capability**. Compose-time validation
(already implemented for capabilities) extends to: contract QoS floor must
be ≤ transport QoS ceiling, else fail closed with the existing style of
precise diagnostic.

---

## 6. Concrete implementation plan

Proving path: `pim/test/` end to end, alongside the untouched legacy tree.
Each phase has a hard acceptance gate; diff-stability of regenerated
artefacts is a standing requirement throughout.

### Progress ledger — 2026-07-02

Phases 0-3 have been implemented using the recommended **C1 + A,
layered** option.

- **Phase 0 complete:** `pyramid.options.pyramid_op` is present in both
  proto import roots; the parser captures `Interaction{pattern, topic,
  qos}` into `ProtoRpc`; the service-shape classifier is present and
  advisory. Legacy parsing was compared against the HEAD parser with the
  new advisory fields stripped: `legacy_parse_identical=1`, 13 files,
  16 services.
- **Phase 1 complete:** the MBSE proto generator stamps options on every
  generated port RPC. Topics follow §5.1, including acronym-safe interface
  splitting (`SPRRequirement` -> `spr_requirement`), with default QoS
  `RELIABLE`, `VOLATILE`, `depth=10`.
- **Phase 2 complete:** binding generation resolves topics from contract
  options first, falls back to the port grammar for unannotated contracts,
  and consults `standard_topics.py` only for the legacy compat service
  layout. New-tree manifests record 58 contract-derived topics. Legacy
  bindings regenerate byte-identically, excluding only the manifest's new
  topic records; the `kTopicEntityMatches` leak is absent from
  `pim_osprey.tactical_objects`.
- **Phase 3 complete:** `Achievement` now carries an acceptance layer
  (`RECEIVED` / `REJECTED` plus optional reason); request wrappers include
  a `cancel` variant on the request topic; the comms harness exercises an
  Information topic and a correlated Request/Requirement pair, including
  Cancel, over JSON and FlatBuffers.

Verification evidence from the implementation pass:

- `python3 -m unittest subprojects/PYRAMID/pim/test_proto_parser.py`
  passed.
- `python3 subprojects/PYRAMID/pim/mbse/proto_generator.py
  subprojects/PYRAMID/pim/mbse/test.json subprojects/PYRAMID/pim/test`
  was drift-free against the current generated proto tree.
- `subprojects/PYRAMID/pim/test_harness/viability_check.sh` passed.
- `subprojects/PYRAMID/pim/test_harness/build_comms_test.sh` passed,
  including JSON and FlatBuffers topic round-trips.
- `bash subprojects/PYRAMID/pim/test_harness/build_plugin_load_test.sh`
  passed. The script lacks an executable bit in this checkout, so it was
  invoked via `bash`.
- Legacy binding regeneration using HEAD vs current generators passed
  `diff -ru --exclude=binding_manifest.json`.
- Generated Ada object compilation passed with `gnatgcc -c -gnat2020` for
  both trees: new tree 112/112 `.adb`, legacy tree 18/18 `.adb`.

Implementation notes carried forward:

- `pyramid.options` is parsed as compiler-extension metadata but excluded
  from generated SDK artifact surfaces; otherwise non-application
  descriptor-extension schemas are projected into ROS2/codec backends.
- FlatBuffers codec plugins currently require the generated JSON codec
  closure as a bridge for wrapper conversion. This is acceptable for the
  Phase 3 proof, but it should be revisited before treating binary codecs
  as fully independent plugin artifacts.
- Windows `.bat` parity has been authored for the updated comms and
  plugin-load scripts, but was not run in the Linux implementation
  environment.
- Generated binding output remained out of git; committed artifacts are
  source generators, proto specs, tests, harness scripts, and docs.

### Phase 0 — options proto + parser (enabler)

- Add `pyramid/options/pyramid.options.proto` (`Interaction`: `pattern`,
  `topic`, `qos`; field number in the 50000–99999 internal range) to both
  proto trees' import roots.
- Extend `pim/proto_parser.py`: capture method-option blocks into
  `ProtoRpc` (`pattern`, `topic`, `qos`); add the **port-grammar
  classifier** (service shape → `port_kind ∈ {request, information, none}`)
  as the inference fallback.
- **Accept:** parser unit tests over both trees; legacy tree parses
  byte-identically (no options present, classifier is advisory only).

### Phase 1 — MBSE stamping (Option C1)

- `pim/mbse/proto_generator.py` `_write_one_port_service`: emit the option
  block on every port rpc — topic per §5.1 (derived from interface name +
  project segment, both already in hand), role pattern, role-default QoS.
- Regenerate `pim/test/`.
- **Accept:** regeneration diff shows *only* option additions; the
  `test_harness` viability + comms + plugin-load suites remain green
  unmodified (options are inert to every existing backend).

### Phase 2 — binding generation from the contract

- `cpp_codegen.py` / `ada_codegen.py`: build the per-service topic set from
  parsed options (falling back to the classifier), not from
  `standard_topics.py`, whenever the contract yields any. Emit the existing
  typed helper surface (`kTopic*`, `subscribe*`, `publish*`, `encode*`,
  `decode*`) for port topics, including oneof-wrapper payload helpers.
- **Scope the legacy side-table to the legacy layout only** — this fixes
  the §2.2 substring leak: `topics_for_service()` is consulted only for
  the `pyramid` compat layout, never for MBSE/generic layouts.
- Record topics in `binding_contract.py`/the binding manifest so transport
  bundles and conformance tooling can enumerate them.
- **Accept:** legacy Tactical Objects bindings regenerate **byte-identical**
  (the migration correctness proof from the interaction-semantics doc);
  new-tree bindings gain topic helpers; the leaked `kTopicEntityMatches`
  block disappears from `pim_osprey` output.

### Phase 3 — runtime proof over PCL

- Extend `pim/test_harness/components_comms_test.cpp`: provider publishes
  an Information topic (`capability.information`); a consumer subscribes;
  a Request port exercised as a correlated pair (publish request →
  requirement transitions observed on the requirement topic, including a
  Cancel). JSON codec plugin, then one decoupled binary codec.
- Add the acceptance-state extension (§5.3) to `common.proto`
  (`Achievement`) in the MBSE type definitions, regenerate.
- **Accept:** comms test green on Linux + Windows (`.sh`/`.bat` parity per
  harness convention); Ada bindings for the new topic helpers
  object-compile (parity with FINDINGS.md bar).

### Phase 4 — transport projections

- **ROS2:** map port topics through the existing canonical naming
  (`/pyramid/topic/<wire_name with '.'→'/'>`); QoS from contract options
  (closing the QoS gap noted in `ros2_transport_semantics.md`).
- **PUBSUB-only transports:** route a Request port over the UDP or DDS
  path as the correlated pair — this is the deferred
  `RPC_UNARY over PUBSUB` adapter from the plugin-system doc, now specified
  by the contract options instead of ad hoc. Behind explicit config, per
  that doc's rule that adapters advertise derived capabilities.
- **Accept:** capability-model compose-time validation passes for a
  mixed route (services over socket/gRPC, topics over UDP/DDS) using only
  contract-derived endpoint requirements.

### Phase 5 — cleanup and retirement

- Legacy tree: either author its topic options into the hand-written
  contract protos (small: one topic set) and delete
  `standard_topics.py` + `topic_metadata/`, or explicitly mark the
  side-table *frozen compat, new consumers forbidden*. Decide on the
  evidence of Phase 2's diff.
- Update `generated_bindings.md`, `pyramid_interaction_semantics.md`
  status tables; conformance suite gains sequence-level checks
  (request must be followed by ≥1 correlated requirement transition —
  the A-GRA-harness-style check from review §5.4).

### Risks

- **Oneof wrappers as topic payloads** touch every codec backend's
  oneof handling on a new path; Phase 3 deliberately runs one text and one
  binary codec early. The FlatBuffers nested-package follow-up in
  FINDINGS.md predates this and lands in the same area — sequence them.
- **Interface-name topic collisions** across projects sharing common
  interface blocks: the project segment in §5.1 covers the observed tree;
  the classifier must fail closed (build error) on a genuine collision
  rather than silently merging topics.
- **Ada surface growth**: topic helpers per port across 42 services is
  bounded (~2–3 helpers each) but Ada codegen has been the historic pain
  point; Phase 3's object-compile gate is the control.
- **Dual QoS homes** drifting: §5.4's single reconciliation rule must be
  implemented in one place (compose-time validation), not per transport.

---

## 7. Relationship to the A-GRA adoption path

This plan is independent of, but multiplies, the A-GRA work
(review §7): Phase 2 of the A-GRA path (an `agra_c2_bridge` fronting
`autonomy_backend`) inherits a contract whose services already project onto
correlated topic pairs, so the bridge reduces to vocabulary translation plus
the approval gate; Phase 4 of the A-GRA path (EXI codec + DMS-conformant DDS
transport) becomes *a codec plugin and a transport plugin* in the existing
decoupled plugin model, with the topic discipline (name-per-type, generic
wrapper, ID-based instance filtering) already native to our bindings rather
than bridged.
