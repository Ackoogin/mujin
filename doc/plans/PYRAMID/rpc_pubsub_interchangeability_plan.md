# RPC / Pub-Sub Seam Interchangeability — the Interaction Facade Plan

**Scope:** Resolve the RPC/pub-sub seam duality found during the A-GRA
proving plan's Phase C: today every `Create/Read/Update/Cancel` rpc on a
4-rpc Request-shape service generates **two independent, coexisting,
independently routable bindings** (an RPC seam and a pub/sub seam) with no
declared relationship between them. The goal is to make the two seams
**interchangeable realizations of one contract element** — the same
transaction can be carried as RPC mechanics or as correlated-pair pub/sub
mechanics, chosen per deployment (and per *leg*, see §2.3) at compose
time, with handwritten component code identical in every case. This is
the property AMS-GRA's CAL provides for OMS services ("insulating OMS
Services from transport specifics",
[`ams_gra_starter_kit_review.md`](../../research/AME/ams_gra_starter_kit_review.md)
§4): client code writes to one abstraction; the message-plane realization
is a deployment decision behind it.

**Date:** 2026-07-09
**Follows on from:**
[`agra_pubsub_shm_udp_proving_plan.md`](agra_pubsub_shm_udp_proving_plan.md)
(Phase C follow-up: "the RPC/pub-sub seam duality"),
[`pubsub_contract_generation_plan.md`](pubsub_contract_generation_plan.md)
(§1 correlated-pair pattern, §5 topic derivation).
**Related:** [`TODO.md`](../../todo/PYRAMID/TODO.md) (WS-D rows: seam
duality; opt-in capability adapters), WS-E facade closure
(`configureTransport`/`configurePubSubTransport` — the pattern this plan
generalizes),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(capability model, §5.4 QoS reconciliation),
[`a_gra_standard_review.md`](../../research/AME/a_gra_standard_review.md).

---

## 1. Problem statement — what the duality is, precisely

Evidence base: the proving plan's Phase C follow-up note, re-confirmed
against the tree at plan-authoring time.

For every rpc in a grammar-conforming Request-shape service, the generator
emits **both**:

1. **RPC seam** — `invoke*`/`dispatch*` typed helpers,
   `pcl_container_add_service` / `pcl_executor_invoke_async`, endpoint
   kinds `consumed`/`provided`/`stream_provided`, keyed by the rpc wire
   name (e.g. `ma_action.create`), capability `RPC_UNARY`/`RPC_STREAM`.
2. **Pub/sub seam** — `publish*`/`subscribe*` typed helpers,
   `pcl_container_add_publisher`/`add_subscriber` /
   `pcl_executor_publish_port`, endpoint kinds `publisher`/`subscriber`,
   keyed by the contract topic (e.g. `agra.ma_action.request`),
   capability `PUBSUB`.

Both appear in `binding_manifest.json` for the identical rpc
(`binding_contract.py`'s `_build_endpoint_requirements` emits the
`source="service"` row *and* the `source="topic"` rows unconditionally).
The `pyramid_op` `pattern` stamp (`SUBSCRIBE`/`PUBLISH` vs `UNARY`) is
advisory: it drives topic derivation but disables nothing, and nothing
anywhere reconciles a routing manifest that routes **both** seams for the
same rpc. Every existing harness exercises exactly one seam per rpc; the
other stays generated, dormant, and untested. Component code is written
*against a seam*, not against the contract element — a component that
calls `createAsync()` cannot be redeployed onto a pub/sub-only link
without rewriting it to `publish*`/`subscribe*` + hand-rolled correlation
(which is precisely what `agra_shm_comms_test.cpp` hand-rolls today).

Consequences:

- **Client code is coupled to the backend choice** — the opposite of the
  CAL property, and the blocker the A-GRA bridge argument (pubsub plan §7)
  quietly depends on not being there.
- **A misconfigured deployment can route both seams** for one rpc —
  duplicate delivery paths with divergent QoS — and compose-time
  validation waves it through.
- **The WS-D "capability adapter" row** (`RPC_UNARY over PUBSUB`) stays
  unbuildable as specified, because there is no single API surface for an
  adapter to sit behind.

## 2. Target model — the interaction facade

### 2.1 One contract element, one API, two realizations

The unit of contract is the **interaction**, not the rpc:

- A **Request port** is one interaction: *submit* (create/update/cancel
  commands), a correlated *requirement transition stream*, and
  correlation by `Entity.id`/`source` on a flat namespace. Its six
  generated artefacts today (4 rpc endpoints + 2 topics) are six views of
  this one element.
- An **Information port** is one interaction: a *publication stream*
  (1 rpc endpoint + 1 topic today).

The generated facade grows a **transaction-shaped API surface** (the
"interaction facade") that component authors code against, with the
existing seam-specific APIs retained underneath (see D7). The facade has
exactly two backing realizations, selected by opaque config in the WS-E
`configureTransport` style:

| Facade operation | RPC realization | Pub/sub realization |
|---|---|---|
| consumer `submit(command)` | unary invoke of `Create`/`Update`/`Cancel`, remote `Ack` | wrap command in the `_Request` wrapper variant, publish on `<...>.request` |
| consumer `transitions(query, on_transition)` | server-streaming invoke of `Read` | subscribe `<...>.requirement`, client-side filter by `query.id`, honour `one_shot` |
| consumer `cancelTransitions()` | `pcl_stream_cancel` | local unsubscribe |
| provider `onCommand(...)` handler | unary dispatch | subscriber callback → same handler |
| provider `TransitionWriter.send()` | fan-out to all open `Read` streams | publish on `<...>.requirement` |
| information consumer `subscribe(on_msg)` | streaming invoke of `Read` | topic subscribe |
| information provider `publish(msg)` | fan-out to open `Read` streams | topic publish |

Handwritten component logic sees only the left column's *names*; which
column executes is configuration.

### 2.2 What "interchangeable" means — the exact claim

Two ends of a link must agree on the **wire realization per leg** (that
is what the shared routing manifest is for). Interchangeability is the
claim that:

1. **Component code is realization-independent.** The same compiled
   component object runs unmodified whether its deployment routes the
   interaction as RPC, as pub/sub, or mixed per leg (§2.3).
2. **The API shape at each end is free.** Because the facade *is* the
   projection, one end's code can be "RPC-shaped" in its own terms
   (futures, acks, stream handles) while the wire — and therefore what
   the other end sees — is pub/sub, and vice versa. A consumer calling
   `submit()` over a pub/sub-routed leg and a provider whose handler is
   fed by a subscriber are the user-visible form of "the same transaction
   is RPC at one end and pub/sub at the other".
3. **The relationship between the seams is declared and enforced.**
   Routing both realizations of one leg is a compose-time error, not a
   silent double-path (§3, D5).

This is follow-up option **(b)** from the proving plan's Phase C note
("both seams generated but explicitly interchangeable via a single facade
method"), executed in a way that also delivers the substance of option
(a): the facade always presents service/transaction semantics; projection
to pub/sub happens below the API line.

### 2.3 Legs, not just links

A Request-port interaction has two independent directed legs:

- **request leg** (consumer → provider): `Create`/`Update`/`Cancel`
  commands, or the `.request` topic.
- **requirement leg** (provider → consumer): `Read` stream frames, or the
  `.requirement` topic.

Each leg's realization is chosen independently in the routing manifest.
This is not a generalization for its own sake — it is the A-GRA C2/MS
deployment shape in miniature: commands as RPC (synchronous transport
acks, point-to-point) with status as pub/sub (observable by any
authorised subscriber, replayable for debrief), or the full A-GRA
pub/sub-everywhere shape, from the same code.

## 3. Design decisions

### D1 — The facade is transaction-shaped; seam choice is opaque config

New generated classes per Request port: `RequestPortClient` (consumer)
and `RequestPortProvider` (provider); per Information port:
`InformationPortSource` / `InformationPortSink`. Naming deliberately
avoids "Service"/"Topic" — the whole point is that neither leaks.
Selection API mirrors WS-E:
`configureInteractionBinding(config_json)` with shapes
`{"binding":"rpc"}`, `{"binding":"pubsub"}`, and per-leg overrides
`{"request_leg":"rpc","requirement_leg":"pubsub"}`; unset falls back to
the contract's `pattern` stamp (the stamp finally becomes the *default
realization* rather than dead metadata). The existing
`ProvidedService`/`ConsumedService`/`publish*`/`subscribe*` surfaces are
unchanged and remain the primitives the facade composes.

### D2 — Canonical wire rule per leg, and the wrapper-variant conformance check

The pub/sub realization of the request leg has one payload: the service's
`_Request` wrapper message on the `.request` topic. The projection rule
is:

- a command rpc whose request type **is** the wrapper publishes it as-is
  (`Create` today);
- a command rpc whose request type **matches a wrapper `oneof` variant's
  type** is wrapped into that variant (`Cancel` → `cancel` variant);
- anything else is a **contract conformance error** at generation time.

A command that satisfies neither clause is **not projectable**: it exists
in the RPC realization only.

Grammar-wide fact (checked at plan-authoring time): `Update`'s request
type is the `_Service_Requirement` message in **every** contract in the
tree — A-GRA example and all of `pim/test/` — and **no** Request wrapper
carries a variant for it (`pim/test/` wrappers hold request variants +
`cancel`; the A-GRA wrapper holds `ma_action` + `cancel`). So today
`Update` is uniformly non-projectable, and the first-rpc-wins
`setdefault` in `topics_for_proto_service` silently hides the
payload-type ambiguity. The consequences, made explicit rather than
discovered by deployers:

- Projectability is computed per command and recorded in the manifest's
  `interactions` section (Phase 1). Selecting a pub/sub realization for
  a request leg is valid; *using* a non-projectable command through the
  facade on a pub/sub-realized leg fails at the facade with a precise
  error (`PCL_ERR_STATE`-style, naming the command and the missing
  wrapper variant) — fail closed at the earliest layer that can know.
- The A-GRA example, being hand-authored, gains an `update` variant in
  Phase 0 so the terminal proof exercises a fully-projectable port.
- `pim/test/` and the MBSE generator are **not** changed by this plan
  (byte-identical stance); teaching `proto_generator.py` to emit an
  update variant — making the whole grammar projectable — is a recorded
  follow-up, gated on the same MBSE-model decision as the existing
  telemetry-marking follow-up.

### D3 — Ack semantics: the portable API promises only what the weaker realization can honour

A-GRA's own rule ("there is no synchronous ack" — pubsub plan §1.1)
decides this. The facade's `submit()` returns a `SubmitResult` meaning
**accepted for transfer** (RPC: the remote `Ack`; pub/sub: local publish
success). *Acceptance* — RECEIVED/REJECTED plus reason — is authoritative
only as a requirement transition (`Achievement.acceptance`, already in
the contract for exactly this purpose). The RPC realization does not
synthesize acceptance transitions from `Ack`s, and the pub/sub
realization does not fake remote `Ack`s; both surfaces the same honest
minimum. Components needing the RPC-only stronger guarantee read
`SubmitResult::remoteAck()` (an `optional<Ack>` — populated only under
RPC realization) and are thereby *visibly* realization-dependent — the
dependence is in the type system, not implicit.

### D4 — `Read`/`Query` over pub/sub: client-side filter, documented late-join gap

Pub/sub realization of `transitions(query, ...)`: subscribe the
`.requirement` topic, filter frames client-side by `query.id` (empty =
all), complete after the first delivery batch when `one_shot` is set.
Two semantic gaps versus RPC `Read` are documented, not papered over:

- **Late join:** an RPC `Read` can serve current state from the
  provider's store; a VOLATILE subscription sees only future
  publications. The provider facade keeps a bounded per-id
  latest-transition snapshot (needed anyway for RPC-mode `Read`, see D6)
  and — pub/sub mode — **re-publishes the current snapshot for an id when
  a command for that id arrives**, giving idempotent-re-publication
  semantics (the cost A-GRA accepts, pubsub plan §1.3). Durability
  QoS/history replay is explicitly out of scope (deferred with the
  DDS transport).
- **`one_shot` over pub/sub** is best-effort ("what is observable now"),
  and says so in the generated doc comment.

### D5 — Compose-time exclusivity: one realization per leg, fail closed

**The naive "at most one member routed" rule is wrong** — a request leg's
RPC realization is *multiple* service endpoints (`Create`, `Update`,
`Cancel` each have their own endpoint name), all of which are legitimately
routed together when the leg runs as RPC. The exclusivity constraint is
between the two **sides** (RPC-side vs. pub/sub-side), not between
individual members: any number of same-side endpoints may be routed
together; the violation is routing anything from *both* sides for the
same leg.

Two enforcement layers:

1. **PCL routing grammar** gains an `exclusive` stanza with two
   comma-separated endpoint lists — the two realizations of one leg, not a
   flat member list:
   ```
   exclusive <group_name> <side_a_endpoint>[,<side_a_endpoint>...] <side_b_endpoint>[,<side_b_endpoint>...]
   ```
   e.g. `exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request`.
   `pcl_transport_routing_load` fails closed (precise-diagnostic style, like
   the §5.4 QoS failure) if at least one endpoint from **each** side has a
   route installed; any number of same-side endpoints may be routed
   together freely. Contract-agnostic, small, and useful beyond this plan.
2. **`contract_routing_manifest.py`** emits one `exclusive` group per
   interaction leg from `binding_manifest.json` (which gains an
   `interactions` section grouping each leg's service endpoint(s) — side A
   — with its projected topic endpoint — side B), and derives route lines
   for exactly one side per leg (default: the `pattern` stamp; overridable
   per deployment).

Hand-authored manifests that omit `exclusive` stanzas remain valid (PCL
stays contract-agnostic); generated manifests always carry them.

### D6 — Provider-side transition fan-out and the snapshot store

`TransitionWriter.send(t)` is the provider's single way to emit a
transition. Under RPC realization it fans out to every open `Read`
stream whose query matches `t`'s id (the facade owns the open-stream
registry the hand-written harnesses have no equivalent of today); under
pub/sub it publishes on the `.requirement` topic. In both modes the
facade updates the bounded per-id snapshot (latest transition per
`Entity.id`, LRU-bounded, size configurable) that serves RPC `Read`
initial state and D4's re-publication. This store is deliberately
minimal — it is not the A-GRA retention/history policy, which stays a
recorded follow-up.

### D7 — Strictly additive; seams stay; scope is grammar-conforming ports

- The new facade is layered on the existing generated primitives; no
  existing generated symbol changes or disappears. Existing harnesses and
  the WS-E facade tests keep passing unmodified — that is the regression
  bar for every phase.
- **Free-form (non-grammar) services are out of scope**: they get no
  interaction facade and no pub/sub projection; routing their service
  endpoints stays the only option, and stays fail-closed against
  PUBSUB-only transports. (WS-D's "opt-in capability adapters for
  free-form services" row is unchanged by this plan.)
- The transport-level `RPC_UNARY over PUBSUB` adapter idea is
  **subsumed** for grammar-conforming ports: "run this interaction over a
  PUBSUB-only transport" is now just a route-line realization choice, no
  transport adapter involved. The WS-D row gets annotated accordingly in
  Phase 5.

## 4. Phased plan

Standing requirements throughout (as the proving plan): legacy `proto/`
and `pim/test/` regeneration stays byte-identical **except** where a
phase explicitly extends `binding_manifest.json` (Phase 1's
`interactions` section — additive key, existing keys byte-identical);
`.sh` + `.bat` parity authored for every new harness; Ada object-compile
parity for generated output (carried note where GNAT is absent, per
`CLAUDE.md` convention); evidence recorded in §7's ledger per phase.

### Phase 0 — Contract groundwork: per-command projectability

- Add the missing `update` variant
  (`MAAction_Service_Requirement`) to `MAAction_Service_Request` in the
  A-GRA example contract (provided + consumed files); update
  `test_agra_example.py`.
- Implement D2's projectability computation in the generator (beside
  `topics_for_proto_service`): each command rpc of a Request-shape
  service is classified projectable (is-wrapper / matches-variant) or
  not, replacing the silent first-rpc-wins payload ambiguity with an
  explicit record. Parser/generator tests cover: fully-projectable
  (A-GRA example post-fix), the uniform `pim/test/` shape (`Update`
  non-projectable, everything else projectable), and a
  conflicting-variant negative.
- **Accept:** projectability computed and tested; A-GRA example
  regenerates clean with the new variant and classifies fully
  projectable; `pim/test/` + legacy byte-identical (classification is
  internal until Phase 1's additive manifest section);
  `build_agra_shm_comms_test.sh` (Phase C harness) still green.

### Phase 1 — Declared interactions + compose-time exclusivity

- `binding_contract.py`: build an `Interaction` model (Request port →
  request leg + requirement leg; Information port → one leg), each leg
  listing its **two sides** — service-endpoint(s) (RPC side) and
  topic-endpoint (pub/sub side) — and each command's Phase 0
  projectability flag; emit as an additive `interactions` section in
  `binding_manifest.json`.
- PCL: two-sided `exclusive <group> <side_a_endpoints> <side_b_endpoints>`
  stanza (D5) in `pcl_transport_routing.h` grammar; loader fails closed
  (`PCL_ERR_STATE`, diagnostic naming the group and the routed endpoint
  from each side) when at least one endpoint from *each* side is routed;
  any number of same-side endpoints may be routed together freely; unit
  tests in `test_pcl_transport_routing.cpp` beside the existing
  `FailsClosedWhenTransportLacksRequiredCap`/QoS-floor tests.
- `contract_routing_manifest.py`: emit `exclusive` groups from
  `interactions`; derive route lines for exactly one side per leg
  (default = `pattern` stamp; `--realize <leg>=rpc|pubsub` override).
- **Accept:** dual-routing a leg (one endpoint from each side) fails
  closed at `pcl_transport_routing_load` with the precise diagnostic;
  routing multiple same-side endpoints together (e.g. all three RPC
  commands) succeeds; generated manifests carry the groups; all existing
  routing/egress harnesses re-run green (their manifests route one side
  and gain nothing).

### Phase 2 — C++ consumer facade (`RequestPortClient`, `InformationPortSink`)

- Emit the transaction API (§2.1 table, consumer rows) in the components
  facade header, composed from the existing `invoke*`/`subscribe*`
  primitives; `configureInteractionBinding(config_json)` per D1;
  `SubmitResult` with `remoteAck()` per D3; query filtering and
  `one_shot` per D4.
- Generated-binding tests in the WS-E style
  (`PclGeneratedComponentFacade.*`): same test component body run under
  `{"binding":"rpc"}` and `{"binding":"pubsub"}` against an in-process
  provider driven through today's primitives; assertions on delivered
  payloads, correlation filtering, `remoteAck()` presence/absence, and
  the D2 non-projectable-command failure (submit of a variant-less
  command on a pub/sub-realized leg errors precisely, exercised against
  a `pim/test/`-shaped contract).
- **Accept:** one component source, both bindings, all tests green; no
  raw `pcl_*` calls in the test's handwritten logic; generator tests pin
  the emitted API.

### Phase 3 — C++ provider facade (`RequestPortProvider`, `InformationPortSource`)

- One handler surface (`onCommand` variants + `TransitionWriter`), backed
  by unary/stream dispatch (RPC) or subscriber callbacks (pub/sub);
  fan-out registry and bounded snapshot store per D6; snapshot
  re-publication on command arrival (pub/sub) per D4.
- Mirror of Phase 2's tests from the provider side, including: two
  concurrent `Read` streams with disjoint queries each receiving only
  their transitions (RPC mode); late-command snapshot re-publication
  observed (pub/sub mode).
- **Accept:** as Phase 2, provider-side; Phase 2's consumer tests re-run
  against a facade-built provider (facade↔facade, both bindings, one
  executor).

### Phase 4 — Ada parity

- Generated Ada packages gain the same interaction surface
  (`Submit`, `Transitions`, `Configure_Interaction_Binding`, provider
  handler + `Transition_Writer`), composed from the existing generated
  Ada primitives, to the **object-compile bar** (`pyramid_ada_all`
  convention); runtime proof is a carried note where GNAT/gprbuild is
  absent, consistent with every prior plan.
- **Accept:** Ada generation clean for the A-GRA example + `pim/test/`;
  object-compile where toolchain present, else carried; no C++
  regressions.

### Phase 5 — Interchange proof harness (plan-terminal)

New harness `agra_seam_interchange_test` (`.sh`/`.bat`), same two-process
MissionAutonomy/C2Station shape and SHM bus as the proving plan's Phase C
(SHM declares `PUBSUB|RPC_UNARY|RPC_STREAM` — one transport carries every
realization, isolating the seam variable). **One compiled component
translation unit per side, byte-identical across all runs** (enforced by
building it once and relinking), driven through the interaction facade
only; the run matrix is pure manifest/config:

| Run | request leg | requirement leg | What it proves |
|---|---|---|---|
| 1 | rpc | rpc | facade over classic RPC, cross-process |
| 2 | pubsub | pubsub | Phase C's scenario via the facade — no hand-rolled correlation left |
| 3 | rpc | pubsub | mixed legs: synchronous-acked commands + observable status (§2.3) |
| 4 | *dual-routed* | — | negative: Phase 1 exclusivity fails closed, diagnostic asserted |

Full worked-example sequence per run (task → RECEIVED → IN_PROGRESS →
COMPLETED; concurrent second action cancelled; correlation and
non-conflation checks reused from Phase C), information port routed
per-run alongside, JSON codec primary + FlatBuffers second witness.

- **Accept (plan-terminal):** all four runs green on Linux from a clean
  generate with **identical component sources/objects** across runs 1–3;
  run 3 demonstrates the same transaction carried RPC on one leg and
  pub/sub on the other with neither end's code aware; run 4's diagnostic
  asserted; every prior harness
  (`build_comms_test`, `build_routed_egress_test`,
  `build_agra_shm_comms_test`, `build_agra_udp_proof_test`,
  `build_agra_mixed_route_test`, `build_contract_routing_test`) re-runs
  green; legacy + `pim/test/` byte-identical modulo the Phase 1 additive
  manifest section. Update FINDINGS.md, TODO.md (WS-D seam-duality row →
  resolved-by pointer; capability-adapter row → annotated per D7), and
  the proving plan's follow-up section with a pointer here.

### Follow-ups recorded, not executed here

- UDP variant of run 3 (request leg must *refuse* rpc realization over a
  PUBSUB-only transport — today's capability validation already covers
  this; a harness case would be evidence, not new mechanism).
- Retention/history policy for the snapshot store (A-GRA keeps cancelled
  objects; we bound and evict).
- DDS/EXI transport pair and gRPC mapping of the interaction facade.
- Free-form service adapters (unchanged WS-D row).
- MBSE generator emitting per-leg realization defaults from model
  markings (extends the existing telemetry-like marking follow-up).

## 5. Risks

- **Provider-side fan-out is the largest new runtime surface** (open-
  stream registry + snapshot store, D6). Mitigation: it is facade-layer
  C++ over existing primitives — no PCL/executor changes anticipated;
  Phase 3's concurrent-streams test is written first.
- **Semantic honesty drift**: the temptation to make pub/sub `submit()`
  "feel" like RPC (synthetic acks) or vice versa. D3/D4 are the guard;
  review against them explicitly at each phase.
- **Manifest-format additivity**: the `interactions` section must not
  perturb existing consumers of `binding_manifest.json`. Guard: byte
  comparison of all pre-existing keys in Phase 1's accept.
- **`exclusive` stanza scope creep** in PCL: keep it a dumb name-set
  check; interaction knowledge stays in the Python tooling.
- **Ada facade size**: if the Ada emission balloons, Phase 4 falls back
  to spec-only (`Submit`/`Transitions` spec + pragma-stubbed bodies) with
  an explicit carried note rather than delaying the terminal proof —
  decided at Phase 4 start, recorded in the ledger either way.
- **Windows parity** authored-not-run: same carried note discipline as
  the proving plan.

## 6. Relationship to the A-GRA / AMS-GRA adoption path

After Phase 5, the repository has a working CAL-analogue for
grammar-conforming ports: component code bound to contract semantics,
message-plane realization (RPC vs correlated-pair pub/sub, per leg)
chosen at compose time, exclusivity enforced, proven with byte-identical
component objects across realizations. The `agra_c2_bridge` then targets
the interaction facade instead of a seam: A-GRA's pub/sub-everywhere
discipline is run 2's configuration, not a code shape — and a future
DMS/DDS transport plugin (or an LA-CAL/OWP bridge from the starter-kit
review, its item 2) slots in as a transport under run-2-shaped manifests
with no component changes. The pubsub plan's §7 "mostly renaming, not
re-plumbing" claim stops resting on an unreconciled dual seam.

## 7. Evidence ledger

To be filled per phase on execution, in the proving plan's ledger style
(commands run, pass/fail, counts, diffs, carried notes). Empty at
plan-authoring time (2026-07-09); §1's duality description and D2's
missing-`update`-variant finding constitute the pre-plan baseline
evidence.

### Phase 0 — executed 2026-07-09

**A-GRA example contract fix.** Added an `update` oneof variant of type
`MAAction_Service_Requirement` (field 3) to `MAAction_Service_Request` in
both `pim/agra_example/pyramid/components/pyramid.components.agra.mission_autonomy.services.provided.proto`
and the mirrored `pyramid.components.agra.c2_station.services.consumed.proto`
(the two files carry byte-identical message bodies for `MAAction_Service_Request`/
`MAAction_Service_Requirement` today -- literal copy-paste, no import between
them -- so the same field was added to both by hand, keeping them in sync).
`test_agra_example.py` has no assertion on the wrapper's exact variant set
(checked: its two classification tests assert `port_kind`/`topic`/`qos`
only), so no test edit was needed there; `classify_port_service`'s
`update.request_type == read.response_type` invariant is untouched by the
new variant (`Update`'s request type was already `MAAction_Service_Requirement`),
so `MAAction_Service.port_kind` stays `"request"`.

**D2 projectability computation.** Added to `binding_contract.py`, next to
`_topic_from_rpc`/`topics_for_proto_service`:
- `CommandProjectability` (frozen dataclass): `service_name`, `rpc_name`,
  `request_type`, `wrapper_type`, `projectable`, `reason`, `variant_field`.
- `command_projectability_for_service(index, pf, service)` /
  `command_projectability_for_file(index, pf)`: classify each
  Create/Update/Cancel rpc of a `port_kind == "request"` service. The
  wrapper is resolved by the grammar-wide naming convention
  `<ServiceName>_Request` (verified against every service in the tree, not
  assumed), not by trusting `Create`'s own request type -- so a
  wrapper-name mismatch is itself detected (`reason="no_wrapper"`) rather
  than silently reusing whatever `Create` happens to point at. A request
  type matching more than one `oneof` variant field of the same type
  classifies `reason="ambiguous_variant"` (not projectable) rather than
  picking one arbitrarily -- the "matches exactly one variant" reading of
  D2/Phase 0's task text.
- `BindingContract` gained an additive `command_projectability: Dict[str,
  Tuple[CommandProjectability, ...]]` field (default `{}`), populated by a
  new `_build_command_projectability()` helper called from `build_contract()`.
  `topics_for_proto_service`, `topics_for_proto_file`,
  `TopicSpecResolver.topics_for_proto` are untouched -- same signatures,
  same first-rpc-wins topic derivation as before Phase 0; classification is
  computed alongside, not folded into, the existing topic dict. Confirmed
  no other caller constructs `BindingContract` positionally (only
  `build_contract()` does) and no `topics_for_proto*` caller needed changes
  (`cpp/components_gen.py`, `cpp/service_header_gen.py`,
  `cpp/service_impl_gen.py`, `cpp/codec_plugin_gen.py`, `ada/naming.py`,
  `backends/ros2_backend.py` all go through `TopicSpecResolver.topics_for_proto`,
  whose signature/behaviour is unchanged).

**Design choice not fully pinned by the plan text, made explicitly:** the
wrapper message is resolved by name convention
(`_request_wrapper_type_name` = `<ServiceName>_Request`), not by "whatever
type `Create`'s rpc uses." Both give the same answer on every
grammar-conforming service seen (`Create`'s request type equals the
convention-derived wrapper name in A-GRA and all of `pim/test/`), but the
convention-derived approach also gives a distinct, honest
`reason="no_wrapper"` outcome if `Create`'s own request type were ever *not*
the service's `_Request` message (a genuine grammar violation) -- treated as
a wrapper-resolution failure for every command, per D2's "wrapper doesn't
exist/parse as expected" clause, rather than as a vacuous truth via
`Create`'s own type. No contract in the tree exercises this distinction
today; noted for anyone reusing this function.

**Tests** (`subprojects/PYRAMID/pim/test_binding_contract.py`, new module,
7 test cases across 3 classes):
- `AgraExampleProjectabilityTest`: both `MAAction_Service` definitions
  (provided + consumed) classify fully projectable post-fix (`Create` ->
  `is_wrapper`, `Update` -> `matches_variant`/`update`, `Cancel` ->
  `matches_variant`/`cancel`); `build_contract()` records it under the
  expected `command_projectability` key.
- `PimTestUniformProjectabilityTest`: `SPRRequirement_Service`
  (`pim/test/.../pyramid.components.pim_osprey.sensor_products.services.provided.proto`,
  the fixture named in this task) classifies `Create`/`Cancel` projectable,
  `Update` not (`reason="no_match"`); a second test sweeps every
  `port_kind == "request"` service across the whole `pim/test/` tree and
  confirms the same Update-non-projectable / Create+Cancel-projectable
  pattern holds uniformly (D2's grammar-wide claim, checked rather than
  trusted).
- `ConflictingVariantProjectabilityTest`: synthetic inline `.proto` fixture
  (tempfile, parsed via `parse_proto`, same convention as
  `test_proto_parser.py`) with a wrapper carrying two `Identifier`-typed
  oneof variants (`cancel`, `cancel_dup`); `Cancel`'s request type collides
  with both, classifies `reason="ambiguous_variant"`, not projectable. A
  second case renames the wrapper message so the naming convention no
  longer resolves it; every command classifies `reason="no_wrapper"`.

`python3 -m unittest subprojects.PYRAMID.pim.test_proto_parser
subprojects.PYRAMID.pim.test_agra_example
subprojects.PYRAMID.pim.test_binding_contract` -- run from the repo root
per the proving plan's own invocation convention (`generate_bindings.py`
carries no separate test runner; `python3 -m unittest` against dotted
module paths is the established command, confirmed in the proving plan's
Phase 0/B ledger entries). 13 tests total, all OK: 3 (`test_proto_parser`)
+ 3 (`test_agra_example`) + 7 (`test_binding_contract`, the new module).

**Regeneration byte-identity.** Captured a before/after pair (this session
had no prior generated-output tree checked in, so "before" = a scratch
generation from the pre-Phase-0 tree, "after" = the same generation
post-Phase-0):
- `pim/test/` -> `--languages cpp,ada --backends json,flatbuffers`: 388
  files both before and after; `diff -rq` clean (0 differences).
- legacy `proto/` -> same flags: 65 files both before and after; `diff -rq`
  clean (0 differences).
- `pim/agra_example/` -> same flags: 54 files generated cleanly (exit 0);
  `binding_manifest.json`'s `topics` section still carries exactly the
  three A-GRA topics from Phase B (`agra.ma_action.request`/`.requirement`
  RELIABLE, `agra.ma_action_plan.information` BEST_EFFORT) -- unchanged by
  Phase 0, confirming the `interactions` manifest section stays Phase 1's
  job. Spot-checked the generated C++ struct: `MAAction_Service_Request`
  now carries `tl::optional<MAAction_Service_Requirement> update;`
  alongside the existing `ma_action`/`cancel` fields.

**Phase C harness regression.** Built `pcl_core`,
`pcl_transport_shared_memory_plugin`, and `flatc` from a fresh
`cmake --preset flatbuffers-only` configure at the repo root (no prior
build tree existed in this environment; build completed in a few minutes,
no long-running fetches beyond flatbuffers/googletest already resolved at
configure time). `bash build_agra_shm_comms_test.sh` -- clean generate, all
four codec `.so`s and the harness built cleanly, JSON and FlatBuffers runs
both green (`PASS (0 failure(s))`), same correlation/non-conflation
results as the proving plan's Phase C (`action-1`: 3 transitions
RECEIVED/IN_PROGRESS/COMPLETED; `action-2`: 1 transition CANCELLED).
Regression check: `bash build_contract_routing_test.sh` (Phase A) re-ran
green (`contract_routing_validation=pass transports=2`).

**Carried note:** GNAT/gprbuild not installed in this environment, so Ada
object-compile for the regenerated `pim/agra_example/` tree is carried,
not run -- same convention as Phases B/C of the proving plan and
`CLAUDE.md`'s `pyramid_ada_all` note. Ada generation itself (13 -> now
included in the combined 54-file cpp+ada run above) is clean.

**Accept criteria met:** projectability computed and tested (13 new
tests); A-GRA example regenerates clean with the new `update` variant and
both `MAAction_Service` definitions classify fully projectable;
`pim/test/` (388 files) and legacy `proto/` (65 files) regenerate
byte-identical to pre-Phase-0 output; `build_agra_shm_comms_test.sh`
re-ran green after the contract change. No changes outside this phase's
scope: `git status` after all regeneration and test runs shows only the
two A-GRA proto files, `binding_contract.py`, and the new
`test_binding_contract.py` modified/added (build directories and harness
scratch output are all pre-existing `.gitignore` entries).

### Phase 1 — executed 2026-07-09

**Part A -- PCL `exclusive` routing grammar.** Extended the manifest
line-based parser (`subprojects/PCL/src/pcl_transport_routing.c`) with a
`handle_exclusive_line()` following `handle_transport_line`/
`handle_route_line`'s exact conventions: `next_token`/`trim` tokenizing,
`set_diag` diagnostics, fail-closed-with-rollback via the existing
`pcl_transport_routing_load` loop (verified by reading it -- any non-`PCL_OK`
return from a line handler breaks the loop and calls
`pcl_transport_routing_destroy`, which now also frees the new group array).
Storage is a new growable `pcl_routing_exclusive_group_t` array
(`groups`/`group_count`/`group_capacity`) on `pcl_transport_routing_t`,
realloc-doubled exactly like `routing_push`/`routing_push_route`; each group
holds two bounded endpoint-name lists (`side_a`/`side_b`,
`PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS = 16`) parsed by a new
`split_endpoint_list()` helper shared by both sides, which fails closed on
an empty list member (a doubled comma) or an oversized list rather than
silently truncating.

Enforcement (**as fixed in review** -- see the design-choices note below for
the original, order-dependent version this replaced) is a single
`validate_exclusivity()` pass called from `pcl_transport_routing_load()`
once the whole manifest has been parsed and every line has succeeded: for
each declared group it checks, via `route_matching_list()` (a linear scan
over the final `r->routes` array -- no new executor API, exactly as
scoped), whether *any* routed endpoint appears on side A and *any* routed
endpoint appears on side B; if both are non-empty, fails closed with
`PCL_ERR_STATE` naming the group and one conflicting endpoint from each
side. Because the check runs once against the fully-populated route/group
state, it is independent of whether a manifest's `route` and `exclusive`
lines for one group appear in any particular order. Any number of
same-side endpoints route together with zero effect on other groups;
declared groups are never registered on the executor (pure manifest
bookkeeping), so `pcl_transport_routing_destroy` just frees the array --
no unregister step needed, unlike transports/routes.

Header (`subprojects/PCL/include/pcl/pcl_transport_routing.h`): grammar
`\code` block extended with the `exclusive <group_name> <side_a_endpoints>
<side_b_endpoints>` directive, the two-sided semantics, and the
declare-before-use ordering requirement, plus a worked example routing all
three A-GRA command endpoints together and showing the rejected pub/sub
route as a comment. `PCL_ERR_STATE`'s doc-comment gained the exclusivity
case.

Six new tests in `subprojects/PCL/tests/test_pcl_transport_routing.cpp`
(a seventh, `ExclusiveGroupConflictDetectedRegardlessOfDeclarationOrder`,
was added by the same-day review fix below -- see the design-choices note)
(`ExclusiveGroup*`, following the file's `WriteManifest`/diag-buffer/
`EXPECT_NE(...find(...))` style): two-sided conflict fails closed naming
group + both endpoints; three same-side command endpoints route together
successfully; an ungrouped endpoint is unaffected by a group routed
alongside it; a malformed `exclusive` line (missing side B) fails closed;
a doubled comma (empty list member) fails closed -- note a *trailing* comma
alone does not manufacture a phantom empty member with this tokenizer (the
loop condition stops before processing text past the last comma), so the
malformed-input case that actually exercises the empty-member branch is a
doubled comma mid-list, not a trailing one, documented in the test; and a
rollback case (two side-A routes installed, then an unrelated malformed
route line) proving the group's routes are cleared along with everything
else, mirroring `REQ_PCL_421`'s existing coverage for the general case.

**Part B -- requirements traceability.** `REQ_PCL_463`-`REQ_PCL_468` added to
`subprojects/PCL/doc/requirements/LLR.md` as a new "## 37. Manifest Routing
Exclusivity" section (appended at the end of the file rather than
renumbering sections 32-36 -- LLR.md's `## NN.` headers are prose-only
organisation the coverage generator does not parse, so out-of-sequence
placement carries no tooling risk; verified by reading
`gen_hlr_coverage.py`, which only tracks section boundaries from `HLR.md`).
`PCL.069`/`PCL.070` (checked against `HLR.md`) cover the manifest grammar
and general fail-closed atomicity but do not state the two-sided-mutual-
exclusivity rule itself, so per `doc/standards/requirements_standard.md`
a new HLR was added: **`PCL.077` - Manifest Exclusive Realization Groups**,
placed physically after `PCL.070` in `HLR.md` (same "Manifest-Driven
Endpoint Routing" section) but keeping the next-free number `077` --
following the existing `PCL.057` precedent of a later-numbered HLR
inserted earlier in the document. Traces to `D8`/`D9` (added to the Design
Decision Traceability table), the same pair `PCL.069`/`PCL.070` trace to --
D8 because which realization runs is manifest data, D9 because a dual-
routed leg is exactly the "unprovable configuration must fail at compose
time" case D9 already states generally. All six new LLRs trace to
`PCL.077`; the three that are also malformed-input/rollback cases
additionally trace to `PCL.070`, matching the existing pattern (e.g.
`REQ_PCL_420` traces `PCL.070, PCL.063`).
`grep -rhoE "REQ_PCL_[0-9]+" subprojects/PCL | sort -t_ -k3 -n -u | tail -3`
showed `REQ_PCL_462` as the highest allocated on this branch at execution
time (462, not the 463 the task text anticipated -- re-checked per the
task's own instruction not to assume), so `463`-`468` were the six free
numbers used. `python3 subprojects/PCL/scripts/gen_hlr_coverage.py --check`
regenerated `doc/reports/PCL/HLR_COVERAGE.md` cleanly: 91 HLRs (was 90), 383
LLRs (was 377), zero trace gaps. `doc/reports/PCL/COVERAGE_REPORT.md` (the
gcovr statement-coverage snapshot) was **not** regenerated: `gcovr` is not
installed in this environment (`gcov` is present, `gcovr` is not), and
regenerating it is a separate full-coverage build across all 21 PCL test
binaries per `cmake/pcl_coverage/CMakeLists.txt`, out of scope for this
phase's build request -- carried, consistent with the GNAT-absent carried-
note convention elsewhere in this plan's ledger.

**Part C -- declared interactions + manifest emission.**
`subprojects/PYRAMID/pim/binding_contract.py` gained `InteractionEndpoint`/
`InteractionLeg`/`Interaction` frozen dataclasses and an
`interaction_for_service()`/`_build_interactions()` pair (mirroring
`command_projectability_for_service`/`_build_command_projectability`'s
structure exactly), wired into `BindingContract.interactions` (additive,
default `{}`) via `build_contract()`. `interaction_for_service()` is purely
a grouping over `topics_for_proto_service()` (topic derivation) and
`command_projectability_for_service()` (Phase 0's per-command flags) --
neither is recomputed. A Request-shape service yields up to two legs
(`request`: side A = `Create`/`Update`/`Cancel` service endpoints with their
Phase 0 projectability, side B = the `.request` topic; `requirement`: side
A = the `Read` service endpoint, side B = the `.requirement` topic); an
Information-shape service yields one `information` leg (side A = `Read`,
side B = the `.information` topic). Group names are
`f"{service_key}.{leg_name}_leg"` (`_leg_group_name()`), documented as the
single source of truth for the convention so `contract_routing_manifest.py`
(and any future manifest generator) can reproduce it from
`binding_manifest.json` data alone without a stored group name.

`generate_bindings.py`: `'interactions': []` added to
`BindingArtifactManifest.__init__`'s `_data` dict alongside
`'endpoint_requirements'`; a new `add_interactions()` method (same
plain-dict-entries pattern as `add_endpoint_requirements`, conditionally
including `rpc_name`/`projectable`/`reason` only when meaningful) called
immediately after `manifest.add_endpoint_requirements(contract)` in
`main()`. (First pass wired the constructor key and the method but missed
the call site; caught by the byte-identity regeneration check below, which
showed zero `interactions` entries in every generated manifest until the
call was added -- worth recording since it is exactly the kind of gap a
regeneration diff is supposed to catch.)

`test_harness/contract_routing_manifest.py` rewritten to read `interactions`
from `binding_manifest.json` instead of scanning the flat
`endpoint_requirements` list. It selects the first Request-shape
interaction whose request leg's side A is the *consumed* role (matching
what every existing caller of this script exercises -- a consumer-side test
component), emits one `exclusive` group per leg (declared before that leg's
route lines), and routes exactly one side per leg. **Default-side design
choice** (the plan text's "default = the pattern stamp" was ambiguous here,
per the task's own framing): D1/D5's `pattern` stamp does not resolve to a
single side cleanly -- a request leg's stamp differs by role (`PUBLISH` for
a consumed-role component, `SUBSCRIBE` for provided) and the requirement
leg's stamp is the request leg's inverse, so "the stamp" alone does not pick
one side per leg independent of which role generated the manifest. Resolved
to the conservative choice named in the task: route the rpc/service side
("rpc") by default for both legs, overridable per leg with a new repeatable
`--realize <leg>=rpc|pubsub` flag (`<leg>` in `request`/`requirement`/
`information`). This *is* a behaviour change from before Phase 1: the old
script free-routed one rpc endpoint (only the first of `Create`/`Update`/
`Cancel` it found, via a flat scan) **and** both pub/sub topics for the same
interaction simultaneously -- exactly the unreconciled dual-seam pattern D5
makes a compose-time error. Picking one side per leg is required, not
incidental; noted explicitly in the script's module docstring. The
three-positional-argument CLI (`binding_manifest plugin_path out_manifest`)
is unchanged; `--realize` is purely additive.

**Tests.** `subprojects/PYRAMID/pim/test_binding_contract.py` gained an
`InteractionModelTest` class (5 new tests): the A-GRA example's
`MAAction_Service` yields exactly `request`+`requirement` legs with the
expected side-A/side-B membership (all three command endpoints on side A,
correct `group_name`, projectability flags all `True` post-Phase-0) and the
correct single `information` leg for `MAActionPlan_Service`;
`build_contract()` records interactions keyed like `service_topics`; a
`pim/test/` contract's `SPRRequirement_Service` (the fixture named in this
task) shows `Update` present in the request leg's side A with
`projectable=False, reason="no_match"` alongside `Create`/`Cancel` at
`projectable=True` -- proving projectability gates pub/sub-side safety
without removing the command from the rpc side; and a service with no
recognised `port_kind` yields no `Interaction` at all (D7 scope: free-form
services untouched).

`python3 -m unittest subprojects.PYRAMID.pim.test_proto_parser
subprojects.PYRAMID.pim.test_agra_example
subprojects.PYRAMID.pim.test_binding_contract` -- 18 tests, all OK (13
carried from Phase 0 + 5 new).

**Regeneration byte-identity.** Same before/after methodology as Phase 0
(`git stash` of the Part C Python changes for the "before" generation, `git
stash pop` for "after", since no generated-output tree is checked into this
repo): `pim/test/` (884 files), legacy `proto/` (158 files), and
`pim/agra_example/` (114 files), each with `--languages cpp,ada --backends
json,flatbuffers`. `diff -rq` before/after showed exactly one differing file
per tree, `binding_manifest.json`; a key-by-key JSON comparison confirmed
every other key byte-identical and the only new key is `interactions`
(`pim/test/`: 40 interaction entries; `proto/`: 0, since the legacy tree has
no grammar-conforming `port_kind` services; `pim/agra_example/`: 4). One
unrelated pre-existing nondeterminism was found and ruled out during this
check: two Ada/C generic-CABI files
(`pyramid-data_model-generic_pim-generic_pkg-cabi.ads`,
`pyramid_data_model_generic_pim_generic_cabi.h`) reordered two struct
declarations between runs; reproduced by generating the **same** pre-Phase-1
code twice and diffing those two runs against each other (no Phase 1 code
involved), confirming it is unrelated set-iteration-order nondeterminism in
the existing generator, not a regression introduced here -- noted, not
fixed (out of this phase's scope).

**PCL build + tests.** `cmake --preset flatbuffers-only` (fresh configure,
no prior build tree in this environment) then `cmake --build --preset
flatbuffers-only-release --parallel $(nproc)` -- full build green, no
warnings in `pcl_transport_routing.c`. `./test_pcl_transport_routing
--gtest_filter="*Exclusive*"` -- 6/6 new tests pass. Full binary re-run --
**23/23 tests pass** (17 pre-existing + 6 new), zero regressions.

**Harness regression.** `bash
subprojects/PYRAMID/pim/test_harness/build_contract_routing_test.sh` --
green (`contract_routing_validation=pass transports=2`); inspected the
generated `contract_routes.pcl` and confirmed the `exclusive` line precedes
its members' `route` lines and all three `sdi_data_processing_dependency`
command endpoints route together under one group with no conflict. Manually
exercised `--realize request=pubsub` against the same generated
`binding_manifest.json` and re-ran the compiled validation harness directly
-- also `PCL_OK`, confirming the override selects the other side correctly.
`bash subprojects/PYRAMID/pim/test_harness/build_agra_shm_comms_test.sh`
(Phase C, unaffected by Part C's script changes but re-run per the task's
regression bar) -- green, same JSON/FlatBuffers, two-process,
correlation/non-conflation results as Phase 0's ledger entry.

**Design choices recorded (beyond the default-side call above):**
- ~~`exclusive` group declaration order enforcement (declare-before-use) is
  documented but not separately mechanically enforced~~ **fixed in review,
  same day:** the flagged gap was real -- a manifest with both conflicting
  `route` lines appearing *before* the `exclusive` declaration loaded
  successfully, since the per-line `check_exclusivity()` call only ever
  consulted whatever groups had been declared so far. Replaced with a
  single `validate_exclusivity()` pass run once after the entire manifest
  is parsed (checking, per group, whether *any* routed endpoint appears on
  each side, via the existing `route_matching_list()` helper), removing
  the ordering dependency entirely rather than special-casing it. Updated
  the header grammar doc and `PCL.077`'s HLR text (both previously stated
  the now-false declare-before-use requirement) and added
  `REQ_PCL_469 - Exclusive Group Conflict Detected Regardless Of
  Declaration Order` (`ExclusiveGroupConflictDetectedRegardlessOfDeclarationOrder`)
  proving the reordered case now fails closed. `HLR_COVERAGE.md`
  regenerated (91 HLRs, **384** LLRs, zero gaps); full
  `test_pcl_transport_routing` re-run, **24/24** pass;
  `build_contract_routing_test.sh` and `build_agra_shm_comms_test.sh`
  both re-ran green after the fix.
- `contract_routing_manifest.py`'s interaction selection (first
  Request-shape interaction with a *consumed*-role request leg) is a
  judgement call preserving the old script's implicit "consumer-side test
  component" framing as closely as a coherent single-interaction pick
  allows; the old script's two independent flat-list scans could (and in
  the `pim/test/` tree, did) pick a service endpoint and a topic pair from
  *different* logical interactions, which the new interactions-based
  approach cannot do by construction.

**Accept criteria met:** dual-routing a leg fails closed at
`pcl_transport_routing_load()` with a diagnostic naming the group and both
endpoints (`ExclusiveGroupTwoSidedConflictFailsClosed`); routing all three
RPC command endpoints together succeeds
(`ExclusiveGroupMultipleSameSideEndpointsRouteTogether`); generated
manifests carry `interactions` (40/0/4 entries across the three trees) and
`contract_routing_manifest.py` emits the corresponding `exclusive` groups;
`build_contract_routing_test.sh` and `build_agra_shm_comms_test.sh` both
re-ran green. `git status` after all builds/regeneration/tests shows only
the ten files listed above as Part A/B/C changes (build trees and harness
scratch output are pre-existing `.gitignore` entries; `build-flatbuffers-only/`
retained on disk, gitignored, in case it is useful for review).

### Phase 2 — executed 2026-07-09

New generator module `subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py`
(`InteractionFacadeEmitterMixin`, ~620 lines), wired into
`CppServiceGenerator` (`service_gen.py`) and invoked from
`components_gen.py::_write_components_header` (3-line hook, strictly
additive -- appends after the existing `ProvidedHandler`/`ProvidedService`/
`ConsumedService`/`StreamWriter`/`StreamHandle` text, changes none of it).
For every service with a full `Interaction` (both legs present, from
Phase 1's `interaction_for_service`), emits:

- **`RequestPortClient`** (Request-shape services): one `submit()` overload
  per command rpc, composed from the existing `<op>Async()` primitive (RPC
  realization) or the D2 wrapper-projection rule (pub/sub realization, publish
  via the existing `publish<Topic>()` primitive); `transitions(query, ...)`
  composed from `<op>Streaming()` (RPC) or a client-side `query.id` filter
  over a shared `subscribe<Topic>()` registration (pub/sub, D4); a
  `SubmitResult{accepted, status, ack}` struct per D3 (`remoteAck()`
  populated only under RPC); `configureInteractionBinding(config_json)`
  selecting `{"binding":"rpc"|"pubsub"}` or per-leg
  `{"request_leg":...,"requirement_leg":...}`, default `"rpc"` for both legs
  (same conservative default Phase 1's `contract_routing_manifest.py`
  settled on, for the same reason: the `pattern` stamp does not resolve
  unambiguously to one side per leg independent of role).
- **`InformationPortSink`** (Information-shape services): `subscribe(on_msg)`
  composed the same way, one leg only.
- A shared `SubscriptionHandle` (move-only, wraps either a live
  `StreamHandle` (RPC) or a `shared_ptr<bool>` liveness flag (pub/sub) so
  `transitions()`/`subscribe()` return one type under either realization)
  and `InteractionBinding` enum, emitted once per file ahead of the classes
  that use them.

**Design choices (plan text under-specified, resolved and recorded per the
task's own instruction):**
- **`submit()` returns `std::future<SubmitResult>`**, not a bare
  `SubmitResult` -- the plan's §2.1 table just shows `submit(command)`.
  Chosen for consistency with `ConsumedService`'s existing uniformly-async
  `<op>Async()`/`std::future<Result<T>>` idiom; the RPC branch wraps the
  already-eagerly-invoked `<op>Async` future in a `std::launch::deferred`
  continuation (dispatch happens synchronously either way under local
  routing, matching the WS-E test's own "Local routing is synchronous"
  comment), the pub/sub branch returns an already-satisfied
  `std::promise`-backed future.
- **D2 non-projectable-command failure surfaces at `submit()`'s pub/sub
  branch**, not as a missing overload: every command still gets a
  `submit()` overload (needed for the RPC realization regardless of
  projectability -- `Update` must still be callable when the leg is
  RPC-realized), but the pub/sub branch of a non-projectable overload
  short-circuits to `SubmitResult{accepted=false, status=PCL_ERR_STATE}`
  without touching the wire at all, per the plan's exact D2 Phase 2 note
  ("fails at the facade with a precise error... only the *use* fails at
  runtime, not generation").
- **Correlation-field resolution for D4's pub/sub `query.id` filter** is
  best-effort structural discovery (`_requirement_correlation_field`):
  resolves the `Read` stream frame type's single-oneof-single-variant
  payload's `id` field (directly or one level of `Entity`-style `base`
  inlining, matching `types_gen.py`'s own inlining convention). Every
  contract in the tree at Phase 2 authoring time fits this shape (confirmed
  by generating clean against `pim/test/` and legacy `proto/` -- see
  below), so the filter is live everywhere it currently could be; a
  requirement frame type that doesn't fit the shape falls back to
  accept-all with a generated comment explaining why, per D4's "documented
  gap, not a silently-wrong filter" instruction, rather than failing
  generation.

**Test infrastructure and the account-interruption recovery.** This phase's
implementing agent was cut off by an account-level API spend limit right
after reporting "Builds cleanly. Let's run it." -- the generator code above,
`subprojects/PYRAMID/tests/test_pcl_generated_interaction_facade.cpp`
(574 lines), and the CMake wiring for a new `interaction_facade_fixture/`
test-owned proto tree (D2's non-projectable-command fixture, `Widget_Service`
with a variant-less `Update`) were all present and compiling, but 8 of 9
new tests failed. Diagnosed and fixed directly (not re-delegated, to avoid
the same interruption):

- **Root cause (not a Phase 2 generator bug):** every generated
  `invoke*`/`publish*` helper routes its wire payload through
  `pyramid_try_registry_encode`/`decode`, which consults
  `pcl_codec_registry_default()` with **no static-codec fallback** (fail-
  closed by design, `transport_codec_plugin_system.md`). The interrupted
  agent's CMake target compiled the raw generated `*_codec.cpp`
  (`toJson`/`fromJson` free functions) directly into the test executable --
  which defines the functions but registers nothing -- and never loaded
  any of the three packages' JSON codec *plugin* `.so`s
  (`pcl_plugin_load_codec`) the way every real deployment (and
  `agra_shm_comms_test.cpp`, Phase C of the proving plan) does. Confirmed
  by bisection: instrumenting `pcl_executor_invoke_async`/`find_service` in
  `pcl_transport_routing.c`'s neighbour `pcl_executor.c` with temporary
  `fprintf` diagnostics (reverted before commit; not part of this phase's
  diff) showed the executor-level dispatch code was **never reached** --
  the failing call returned `PCL_ERR_NOT_FOUND` from
  `invokeMaactionCreate`'s own registry-encode check, several layers above
  routing.
- **Fix:** three new `add_library(... MODULE ...)` targets in
  `tests/CMakeLists.txt` (`pyramid_codec_json_agra_mission_autonomy`,
  `pyramid_codec_json_agra_c2_station`,
  `pyramid_codec_json_interaction_facade_fixture_widget`), each built from
  its package's generated `*_json_codec_plugin.cpp` plus the shared
  data-model + package codec/cabi-marshal sources it needs (mirroring
  `agra_shm_comms_test.cpp`'s per-package `.so` split, needed for the same
  reason: each package's plugin exports the identical
  `extern "C" pcl_codec_plugin_entry` symbol, so they cannot be statically
  linked together). A new `CodecLoaderEnvironment : ::testing::Environment`
  in the test file `pcl_plugin_load_codec()`s all three `.so` paths
  (threaded in via `target_compile_definitions`, the same
  `CAPTURE_PLUGIN_PATH`-style pattern `subprojects/PCL/tests/CMakeLists.txt`
  already uses) into the shared default registry before any test runs,
  registered via a global `::testing::AddGlobalTestEnvironment(...)` so it
  fires ahead of `RUN_ALL_TESTS()` regardless of translation-unit order.
  This resolved 8 of 9 failures immediately.
- **Ninth failure, a genuine test-design gap:**
  `SubmitProjectableCreateUnderPubsubStillWorksOnTheSameFixture` published
  to the fixture's `.request` topic with no local subscriber at all in the
  test -- `pcl_executor_publish_port`'s `PCL_ROUTE_LOCAL` branch correctly
  reports `PCL_ERR_NOT_FOUND` when nobody local is listening (mirroring
  `find_service`'s behaviour for unary RPC), so the publish's "success"
  never had anywhere to be delivered. Added `WidgetProbeComponent`, a
  second hand-wired `widget_cons::ConsumedService` instance subscribing the
  fixture's `.request` topic (reusing the role-symmetric
  `subscribeInteractionFacadeFixtureWidgetRequest()` primitive every
  generated `ConsumedService` carries, per `interaction_facade_gen.py`'s
  own comment on why this symmetry exists) as the fixture's sole listener.

**Verification.** `./test_pcl_generated_interaction_facade` -- **9/9 pass**,
stable across 3 repeated runs. Full regression sweep re-run after the fix
(all green, no changes needed elsewhere): `test_pcl_generated_component_stream_handle`
(7/7, the existing WS-E facade test, confirming Phase 2's additive hook
changed none of its behaviour), `test_pcl_transport_routing` (24/24, Phase 1's
suite, confirming the unrelated `pcl_executor.c` debug instrumentation used
for diagnosis was fully reverted and left no trace),
`python3 -m unittest ... test_binding_contract` (18/18, unchanged --
Phase 2 touched no Python contract-model code, only the C++ generator),
`build_contract_routing_test.sh`, `build_agra_shm_comms_test.sh` (Phase C),
`build_agra_udp_proof_test.sh` (Phase D), and `build_agra_mixed_route_test.sh`
(Phase E) all re-ran green. Regenerated `pim/test/` (388 files) and legacy
`proto/` (65 files) directly (not via `git stash`/diff -- no generated
output is git-tracked, confirmed via `git status` before and after showing
only this phase's 6 intended source files) to confirm the new
per-file `_write_interaction_facade` hook does not crash or error against
every existing contract in the tree, not just the two Phase 2 fixtures:
both regenerate cleanly (exit 0), and the facade classes appear in 16 of
`pim/test/`'s generated files (every service with a complete two-leg
`Interaction`) versus 0 in legacy `proto/` (no grammar-conforming ports
there, consistent with Phase 1's finding) -- confirming the correlation-field
structural-discovery fallback (previous bullet) never needed to fire
outside its documented accept-all path. Ada generation (`--languages ada`)
included in the same regeneration runs; object-compile stays a carried note
(no GNAT/gprbuild in this environment, per the established convention).

**Accept criteria met:** one component compiles against the same generated
facade classes under both `{"binding":"rpc"}` and `{"binding":"pubsub"}`
(`RpcAndPubsub/InteractionFacadeBindingTest.SubmitCreateRoundTrip/{0,1}`,
parameterized over the identical test body); `remoteAck()` populated under
RPC and empty under pub/sub for the same command; the D2 non-projectable
case fails at the facade with `PCL_ERR_STATE` before any wire attempt;
`transitions()` delivers correctly under both realizations with pub/sub-mode
`query.id` filtering proven to actually filter (two concurrent ids, only
the queried one observed); `InformationPortSink` proven under pub/sub;
existing generated-binding tests, PCL routing tests, Python tests, and
every proving-plan harness re-run green. Phase 3 (provider-side facade) is
untouched, as scoped.

### Phase 3 — executed 2026-07-09

Extended `subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py` (no new
file this phase -- appended to Phase 2's module) with the provider-side
half of the facade, wired into the same `_write_interaction_facade` hook
alongside the Phase 2 consumer classes:

- **`RequestPortHandler`** (per Request-shape service): one `on<Command>()`
  virtual method per `Create`/`Update`/`Cancel` rpc, returning `Ack` --
  deliberately *no* `onRead` method. Under the interaction facade, `Read`
  is entirely facade-internal (the open-stream registry, below); component
  authors implement only the command callbacks.
- **`RequestPortProvider`**: owns RPC dispatch for *just this service's*
  four rpcs, independent of any file-wide `ProvidedService` a component
  might separately construct for other services in the same file --
  `ProvidedService`/`ProvidedHandler` (components_gen.py, unmodified) are
  genuinely file-wide (one shared handler bound via one `all_rpcs` list
  spanning every service in the file), which does not compose with a
  per-service provider facade. Resolved by having `RequestPortProvider`
  emit its *own* narrow `Bridge : ServiceHandler` (overriding only this
  service's four methods; every other rpc in the file keeps
  `ServiceHandler`'s existing default stub body, unreferenced) and its own
  `addUnaryBinding`/`addStreamBinding`/`unaryDispatch`/`streamDispatch`
  scoped to `service.rpcs` -- mechanically the same shape
  `ProvidedService` already uses, narrowed, not a new dispatch mechanism.
  Also owns:
  - a pub/sub probe (a plain `ConsumedService`, matching Phase 2's own
    "role-symmetric primitives" pattern) subscribing the request topic and
    publishing the requirement topic;
  - the **open-Read-stream registry** (D6): `Bridge`'s stream-rpc override
    registers each incoming `Read`'s `StreamWriter` + `Query` instead of
    exposing it to the component author;
  - the **bounded snapshot store** (D6): an insertion-order-evicted
    `std::list`+`std::unordered_map` pair (`kSnapshotCapacity = 64`),
    updated on every `sendTransition()` regardless of realization;
  - `configureInteractionBinding()`, selecting only the **requirement
    leg's** realization -- **design decision**: the request leg has no
    provider-side binding to configure at all. Both the RPC service ports
    and the pub/sub probe's subscriber are *always* bound and listening;
    which one a deployment actually delivers through is a routing-manifest
    decision (D5's `exclusive` groups), not an application-level choice on
    the provider, since the provider must be able to receive a command via
    whichever side the manifest actually connects. Documented in `bind()`'s
    doc comment and accepted (not silently dropped) in
    `configureInteractionBinding()`'s JSON if a caller passes
    `"request_leg"` anyway.
  - **`TransitionWriter`** (D6): a thin proxy (`transitionWriter()` returns
    one by value) forwarding `send()` to `RequestPortProvider::sendTransition()`,
    which records the snapshot, then either fans out to every open Read
    stream whose `query.id` matches (RPC) or publishes the requirement
    topic (pub/sub) -- reusing Phase 2's exact `_requirement_correlation_field`
    resolver for both the snapshot key and the RPC fan-out's `query.id`
    match, not a new correlation mechanism.
- **`dispatchPubsubCommand`** (D4 late-join mitigation): the pub/sub
  probe's request-topic subscriber callback. Unwraps the shared wrapper's
  oneof, dispatches to the same `on<Command>()` methods the RPC realization
  uses (return value discarded, per D3 -- pub/sub has no synchronous ack),
  then re-publishes the current snapshot for the command's id if one
  exists. Correlation extraction for the **command** side (distinct from
  the **transition** side's existing resolver) needed new machinery:
  `_command_correlation_accessor()` (direct `Entity`-inlined `.id` for
  `Create`'s `MA_Action`, the `Identifier` scalar-alias-is-the-id-itself
  case for `Cancel`, one level of oneof-unwrap for `Update`'s
  `MAAction_Service_Requirement`) plus `_leftover_wrapper_variant()` for
  the `is_wrapper` command specifically: since `Create`'s own rpc parameter
  type *is* the wrapper (D2), it has no dedicated oneof field the way
  `matches_variant` commands do, so its correlation payload is found *by
  elimination* -- the one wrapper oneof field no `matches_variant` command
  already claims.

**A real bug found and fixed during verification (not by inspection --
by a failing test and a debugging session that needed generator-source
instrumentation + full reconfigure to actually observe, see below):**
`republishSnapshotFor()`'s first draft passed a *reference* into the
stored snapshot value (`sendTransition(it->second->second)`) straight
into `sendTransition()`, which unconditionally calls `recordSnapshot()`
first. `recordSnapshot()`'s existing-id branch (needed for the normal
update-in-place case) does `snapshot_order_.erase(existing->second)` --
which, when re-recording the *same* id via a reference that already
aliases that exact list node, erases the node the caller's reference
points at, then copies from the now-dangling reference into the
replacement node: a use-after-free with no crash (the freed `std::list`
node's memory was quickly reused by the immediately-following
`push_back`, so the corrupted copy silently carried whichever bytes
happened to land there rather than segfaulting) that manifested as the
re-published transition's `id` field no longer matching `query.id` on the
consumer side, so `TransitionsUnderPubsubFiltersByQueryId`-style matching
silently failed. Fixed by making `republishSnapshotFor()` copy the
snapshot value out **before** calling `sendTransition()`:
```cpp
const MAAction_Service_Requirement snapshot_copy = it->second->second;
sendTransition(snapshot_copy);
```
Diagnosed by regenerating with temporary `fprintf` tracing added directly
to `interaction_facade_gen.py` (not hand-edited into build artifacts --
an earlier attempt to do the latter produced no observable output for
reasons not fully root-caused, possibly a stale generated-tree directory
that a subsequent `cmake --preset` silently regenerated over; switching
to source-level instrumentation with an explicit
`rm -rf .../generated_agra_example && cmake --preset flatbuffers-only`
before every rebuild made the tracing reliable) at four call sites
(`dispatchPubsubTransition`'s entry and per-iteration match state,
`dispatchPubsubCommand`'s entry, `republishSnapshotFor`'s lookup result,
`sendTransition`'s publish return code) -- all instrumentation removed
before this commit.

**Tests** (`subprojects/PYRAMID/tests/test_pcl_generated_interaction_facade.cpp`,
extended, no new file): a facade-built provider (`FacadeHandler` +
`FacadeProviderComponent`, using `MaactionRequestPortProvider`) added
*alongside* Phase 2's existing hand-wired `RecordingHandler`/
`ProviderComponent` stand-in (not replacing it, to keep the Phase 2
regression bar zero-risk under this phase's time budget) plus four new
tests:
- `FacadeSubmitCreateRoundTripUnderRpc` / `...UnderPubsub` -- the plan's
  "facade<->facade, both bindings, one executor" accept criterion, a
  smaller-scope stand-in for a full parameterized re-run of every Phase 2
  scenario against the facade provider (a fuller re-run is possible future
  work, not required to meet this phase's stated accept bar).
- `TransitionWriterFansOutToConcurrentReadStreamsByQuery` -- the plan's
  explicit "two concurrent Read streams with disjoint queries each
  receiving only their transitions (RPC mode)" criterion: two consumers,
  disjoint single-id queries, one `TransitionWriter::send()` per id,
  each consumer receiving exactly its own.
- `LateCommandTriggersSnapshotRepublicationUnderPubsub` -- the plan's
  explicit "late-command snapshot re-publication observed (pub/sub mode)"
  criterion: consumer subscribes and observes an original send (count 1);
  a later `Cancel` command for the same id triggers `dispatchPubsubCommand`'s
  re-publication, observed as a second, distinct delivery (count 2) --
  this is the test that caught the use-after-free above; also confirmed
  (via a fixed, then-reverted version of the test) that a naive
  "send-before-subscribing" ordering is *not* buffered/replayed by PCL's
  local pub/sub (VOLATILE, no history), which is why the test's baseline
  send happens after the consumer subscribes rather than trying to prove
  a stronger late-join guarantee than D4 actually promises.

`./test_pcl_generated_interaction_facade` -- **13/13 pass** (9 carried
from Phase 2 + 4 new), stable across 3 repeated runs. Full regression
sweep: `test_pcl_generated_component_stream_handle` (7/7, WS-E facade,
untouched), `test_pcl_transport_routing` (24/24, Phase 1, untouched),
`python3 -m unittest ... test_binding_contract` (18/18, untouched --
Phase 3 is C++-generator-only), `build_contract_routing_test.sh`,
`build_agra_shm_comms_test.sh` (Phase C), `build_agra_udp_proof_test.sh`
(Phase D), `build_agra_mixed_route_test.sh` (Phase E) all re-ran green.
Regenerated `pim/test/` (388 files, `--languages cpp,ada`) directly to
confirm the new provider-side per-file hook doesn't crash or error
against any existing contract, not just the two Phase 2/3 fixtures --
clean, exit 0 (no git-tracked generated output exists to diff against, so
this is a crash/error check, not a byte-identity check, consistent with
Phase 2's ledger methodology). Ada object-compile stays a carried note
(no GNAT/gprbuild in this environment).

**Accept criteria met:** `RequestPortHandler`/`RequestPortProvider`
implemented, composed from existing primitives per D6 (no existing
generated symbol changed); two concurrent Read streams with disjoint
queries each receive only their own transitions (RPC mode, explicit plan
criterion); late-command snapshot re-publication observed (pub/sub mode,
explicit plan criterion); a facade-built provider interoperates correctly
with Phase 2's facade-built consumer under both bindings on one executor
(the "facade<->facade" criterion, via the four new tests rather than a
full re-run of every Phase 2 scenario -- see the design-choices note
above on scope); every existing test and harness re-runs green. Phase 4
(Ada parity) and Phase 5 (terminal interchange harness) remain untouched,
as scoped.

### Phase 4 — executed 2026-07-09

**Decision (recorded per the plan's own risk-note instruction, "decided at
Phase 4 start, recorded in the ledger either way"): spec-only fallback,
not full functional parity.** Rationale, established before writing any
Ada code:

- This build environment has no GNAT/gprbuild at all (`which gnatmake
  gprbuild gnat` all resolve to nothing; only plain `gcc` is present, and
  it lacks the `gnat1` backend -- confirmed by re-running the pre-existing
  `test_generic_ada.py` object-compile tests, which already fail with
  `cannot execute 'gnat1'` on the base branch, before this phase's changes).
  Runtime *and* compile-time proof are both unavailable regardless of which
  approach is chosen, so the usual mitigant for "large generated-code
  change" -- compile and run it -- does not apply to either option here.
- Surveyed the existing Ada generator (`pim/ada/`: `naming.py`,
  `service_spec_gen.py`, `service_body_gen.py`, ~2900 lines together) before
  deciding. It is a flat procedural surface -- one shared `Service_Handlers`
  record of `access procedure`/`access function` callbacks, one
  `Service_Channel` enum, one `Dispatch` case statement -- with no
  tagged-type hierarchy, no dynamic dispatch, no RAII/controlled-type
  idiom anywhere in it. The C++ facade's runtime behaviour (D1's per-leg
  live binding switch, `SubscriptionHandle` unifying an RPC `StreamHandle`
  and a pub/sub liveness flag under one type, D6's fan-out over an
  open-stream registry plus a bounded LRU-ish snapshot store, D2's
  oneof-variant routing at dispatch time) has no existing Ada analogue to
  extend -- it would mean introducing a materially new architectural layer
  into a generator that has never needed one, entirely unverifiable in this
  environment. That is precisely the "Ada facade size balloons" condition
  the plan's risk note names.
- Spec-only keeps the *declared* surface -- the same shape client/provider
  code would program against (`Submit_<Command>`, `Transitions`,
  `Configure_Interaction_Binding`) -- consistent across languages and ready
  for a future phase to wire real dispatch behind, without committing to
  unverifiable dispatch logic now.

**Implementation.** New file
`subprojects/PYRAMID/pim/ada/interaction_facade_gen.py`
(`InteractionFacadeSpecMixin`), mixed into `AdaServiceGenerator`
(`service_gen.py`). Threaded the per-file proto `Path` (`pf`, already
computed correctly in `generate()`'s loop) through `_write_spec`/
`_write_body` as a new trailing optional parameter (default `None`, so
every other existing caller/signature is unaffected) so the hook can build
a `ProtoTypeIndex` and call `binding_contract.interaction_for_service` --
the same Phase 0/1 classification function the C++ facade calls -- rather
than re-deriving a parallel Request-shape detection rule in Ada. For each
Request-shape `Interaction`:

- `<Prefix>_Configure_Interaction_Binding (Config_Json : String)` -- spec
  declaration matching D1's `config_json` shape (`request_leg`/
  `requirement_leg` keys), documented in a doc comment.
- `<Prefix>_Submit_<Command>` per `Create`/`Update`/`Cancel` rpc actually
  present on the service (not a fixed three -- the widget fixture-style
  case of a service missing a command is handled by only emitting what
  `interaction_for_service`'s `request` leg actually lists), with an
  explicit `Result_Has_Ack : out Boolean` alongside `Result_Ack : out Ack`
  in the declaration -- naming D3's honest-ack-semantics contract directly
  in the spec rather than leaving it implicit.
- `<Prefix>_Transition_Callback` (access-procedure type) and
  `<Prefix>_Transitions (Filter : ...; Callback : ...)` when a
  `requirement` leg is present, with a doc comment recording D4's late-join
  caveat and pointing at D6's provider-side snapshot store as the
  mitigation a real implementation must call through to.
- Bodies (`.adb`) are pragma-stubbed: `pragma Unreferenced` on every `in`
  parameter, then `raise Program_Error with "<Name>: Phase 4 spec-only
  fallback -- ..."`, following this codebase's own pre-existing
  not-yet-implemented convention (`service_body_gen.py` already raises
  `Program_Error` the same way for its own unfinished paths -- not a new
  idiom introduced by this phase).

**Bug caught before generating anything for real:** the first draft
declared `Transitions (Query : Query; ...)` -- a formal parameter named
identically to its own type mark (the `Query` message type). Ada's own
homograph rule likely accepts this syntactically (the type mark on a
parameter is resolved in the scope *before* the parameter's own name comes
into scope), but this codebase already has an explicit guard against
exactly this collision class elsewhere (`naming.py::_ada_field_name`'s
`if ada_type is not None and name == ada_type: name = 'Val_' + name`,
and the pre-existing `Invoke_*` procedures uniformly name this parameter
`Request`, never after its own type) -- strong precedent that this
exact pattern has caused real problems before. Renamed the parameter to
`Filter` before generating anything, rather than relying on untestable
Ada semantics to save it.

**Verification** (Python-level only, per the decision above -- no
GNAT/gprbuild in this environment to compile-verify either the new
declarations or any pre-existing Ada output):

- Ran `generate_bindings.py ... --languages ada` against both
  `pim/agra_example` (13 files) and `pim/test/` (98 files) directly --
  clean, exit 0, no exceptions. Inspected the emitted `.ads`/`.adb` for
  `MAAction_Service` (agra_example, provided side) and
  `SDI_Data_Provision_Dependency_Service` (`pim/test`, consumed side) by
  hand: correct per-service command set, correct Ada types threaded
  through from `_ada_req_type`/the response-type resolver, correct
  `Filter`/`Query` type-vs-name disambiguation, matching spec/body
  procedure signatures.
- `tests/test_generic_ada.py`: 4 passed / 2 failed / 1 skipped, byte-identical
  to the same file re-run against the pre-Phase-4 tree (confirmed via
  `git stash`) -- the 2 failures are the pre-existing `gnat1`-absent
  object-compile tests, present on the base branch, untouched by this
  phase; 0 new failures.
- Full non-Ada-object-compile Python suite (`tests/` + `pim/`, i.e.
  everything except `test_generic_ada.py`'s two toolchain-gated cases):
  52/52 passed, including `pim/test_binding_contract.py` (unmodified,
  proves `interaction_for_service` reuse didn't perturb Phase 0/1's own
  behaviour) and `pim/test_agra_example.py`.
- No C++ generator source (`pim/cpp/`) touched by this phase -- confirmed
  via `git status`; C++ build/test suite not re-run (nothing in its input
  changed).

**Accept criteria met (spec-only variant, as pre-authorized by the plan's
own risk note):** Ada generation clean for the A-GRA example + `pim/test/`;
object-compile carried as a note (no GNAT/gprbuild present, consistent
with every prior phase's carried-note convention -- this is an existing
environment limitation, not new to this phase); no C++ regressions. Full
functional runtime parity with the C++ facade (D1's live dispatch, D6's
snapshot store, etc.) is explicitly *not* delivered this phase --
recorded here as scoped-out, not silently dropped, per the plan's own
instruction to record the decision "either way."

### Phase 5 — executed 2026-07-09 (plan-terminal)

New harness `pim/test_harness/agra_seam_interchange_test.cpp` +
`build_agra_seam_interchange_test.sh`, modeled on the prior proving plan's
Phase C/E harnesses (`agra_shm_comms_test.cpp`, `agra_mixed_route_test.cpp`
-- same two-process MissionAutonomy/C2Station split, same real
`libpcl_transport_shared_memory_plugin.so` bus, same cross-process
child-process-pair pattern) but driving `MaactionRequestPortProvider` /
`MaactionRequestPortClient` (the Phase 2/3 interaction facade) exclusively
-- no hand-rolled `publish*`/`subscribe*`/`decode*` calls anywhere in the
new harness. One binary, built once, re-executed as both roles across every
run with only `--request-leg=`/`--requirement-leg=` CLI args and a
freshly-generated manifest differing per run, so "byte-identical component
objects across runs" holds trivially (literally the same file).

Run matrix, each a real two-process cross-process run over SHM, JSON then
FlatBuffers:

| Run | request leg | requirement leg | Result |
|---|---|---|---|
| negative | dual-routed (both realizations of the request leg) | -- | fails closed, compose-time only, no processes spawned |
| 1 | rpc | rpc | PASS |
| 2 | pubsub | pubsub | PASS |
| 3 | rpc | pubsub | PASS |

Each of runs 1-3 replays the full worked-example sequence (Create ->
RECEIVED/IN_PROGRESS/COMPLETED, a second action Cancelled, correlation +
non-conflation checks) through `submit()`/`transitions()`/`TransitionWriter`
only, plus a new D3 check unique to this phase: `remoteAck()` is asserted
populated when the request leg is rpc-realized and asserted empty when
pubsub-realized, for both the create and the cancel command -- proving the
facade's honest-ack promise holds under real cross-process RPC, not just
the in-process fixture Phase 2/3 verified it against.

**Two real, pre-existing PCL-level gaps found and fixed** -- this is the
first place in the entire codebase anything has driven a `provided`/
`stream_provided`-kind endpoint through `pcl_transport_routing_load()` and
carried real cross-process traffic over it; every earlier harness (this
plan's Phases 0-4, the prior proving plan's Phases A-E) only ever routed
pub/sub topics through that path. Both were found by direct evidence
(a hung cross-process run, traced via targeted `dprintf` instrumentation in
`pcl_transport_shared_memory.c`, rebuilding just the affected static
lib/plugin targets each iteration -- removed before the final commit; see
methodology precedent in the Phase 3 ledger entry) before either was
understood as a real bug rather than a harness bug:

1. **Manifest-driven streaming invoke never dispatched remotely.**
   `pcl_executor_invoke_async()` (unary) already checks the per-endpoint
   route table `pcl_transport_routing_load()` populates before falling back
   to the legacy single executor-wide transport; `pcl_executor_invoke_stream()`
   (streaming) never had the equivalent check -- it went straight to the
   legacy `e->has_transport`/`e->transport` field, which a routing-manifest
   deployment never sets, so every manifest-routed streaming invoke silently
   fell through to the always-empty intra-process fallback.
   `transitions()`'s underlying `maactionReadStreaming()` call returned an
   invalid `StreamHandle` every time under the rpc realization -- run 1
   deadlocked (see the future-lifecycle bug below, which this same symptom
   also triggered). Fixed in `subprojects/PCL/src/pcl_executor.c` by adding
   the identical per-endpoint-route-then-legacy-fallback structure
   `pcl_executor_invoke_async()` already has, verbatim in shape.
2. **A `provided`/`stream_provided`-serving component's inbound requests
   were silently dropped.** The shared-memory (and socket) transport's
   documented architecture (`pcl_transport_shared_memory.h`) requires a
   provider to retrieve its transport's "gateway" container and
   `configure()`/`activate()`/`pcl_executor_add()` it itself -- inbound
   `SVC_REQ`/`STREAM_REQ` frames are re-posted onto an internal topic only
   the gateway subscribes to. This is a real, established, precedented
   requirement (`subprojects/PYRAMID/tactical_objects/src/apps/tactical_objects_main.cpp`
   already does this manually for its own `pcl_executor_set_transport()`
   -based setup), but `pcl_transport_routing_t` exposed no way to retrieve a
   loaded peer's gateway container at all -- the manifest-driven path had no
   route to this existing, working mechanism. Added
   `pcl_transport_routing_get_gateway(routing, peer_id, out_gateway)` to
   `pcl_transport_routing.c`/`.h` (purely additive: one new function,
   resolves the same optional `pcl_shm_transport_plugin_gateway`/
   `pcl_socket_transport_plugin_gateway` symbols `tactical_objects_main.cpp`
   already resolves manually, via the same `pcl_plugin_symbol()` public
   API -- no transport plugin touched, no vtable change). The harness's MA
   role now calls it once, right after `pcl_transport_routing_load()`,
   before constructing its component.

Both fixes are HLR/LLR-documented (`PCL.078`, `REQ_PCL_470`/`471` in
`subprojects/PCL/doc/requirements/`) with dedicated gtest coverage added to
`test_pcl_transport_routing.cpp` (`StreamingInvokeDispatchesThroughRoutedTransport`;
`GetGatewayReturnsUsableContainerForShmPeer`,
`GetGatewayReturnsNullForPluginWithoutGateway`,
`GetGatewayFailsClosedOnUnknownPeerAndBadArgs` -- the last two using a
minimal `invoke_stream` stub added to the existing test-only capture
transport plugin), not just the integration harness -- `gen_hlr_coverage.py`
reports 0 gaps (92 HLRs, 386 LLRs) after regeneration.

Also fixed, in the harness itself before the two PCL bugs were even
diagnosed: `MaactionRequestPortClient::submit()` returns a
`std::launch::deferred` `std::future<SubmitResult>` -- `wait_for()` on such
a future *always* reports `std::future_status::deferred`, never `ready`,
regardless of whether the underlying (genuinely async) response has
arrived, so a poll loop written as "spin while `wait_for(0) != ready`, then
`.get()`" spins uselessly for the whole timeout and then calls a *for-real*
blocking `.get()` with nothing left driving the executor -- worse, a
`std::launch::async` future's destructor also blocks until its thread
finishes, so even abandoning the wait after a deadline doesn't bound the
hang. Fixed by running the actual blocking `.get()` on a `std::launch::async`
background thread (whose future *does* report `ready`/`timeout` correctly)
while the calling thread keeps spinning the executor -- `await_submit()` in
the harness, a small, generically-typed helper. Not a facade bug -- `submit()`'s
own design already does the safe (eager-invoke, deferred-wait) split
correctly; this was purely about how a *caller* wanting a bounded wait must
compose with it.

**Verification**: full run (negative gate + 3 runs x 2 codecs) green,
`PASS (0 failure(s))`, stable. Full PCL regression suite re-run after each
of the two core fixes and once more clean at the end: 732/732 CTest cases
green (up from 728 -- the 4 new gtest cases), including every
`PclSharedMemoryTransport.*`/`PclTransportRouting.*` case pre-existing
before this phase. Every prior harness re-ran green:
`build_comms_test.sh`, `build_routed_egress_test.sh`,
`build_agra_shm_comms_test.sh`, `build_agra_udp_proof_test.sh`,
`build_agra_mixed_route_test.sh`, `build_contract_routing_test.sh`.
`gen_hlr_coverage.py`: 0 gaps.

**Accept criteria met (plan-terminal):** all four runs (negative + 3
positive) green on Linux from a clean generate, with identical component
objects across runs 1-3 (the same compiled binary, trivially); run 3
demonstrates the same transaction carried rpc on the request leg and
pub/sub on the requirement leg with neither `MaactionRequestPortProvider`
nor `MaactionRequestPortClient` aware which; run 4's (the negative gate's)
diagnostic asserted (names the `request_leg` group and "opposite sides");
every prior harness re-runs green. `doc/todo/PYRAMID/TODO.md`'s WS-D
seam-duality row is updated to **Resolved** with a pointer here, and its
capability-adapters row annotated per D7 (scope now explicitly free-form
services only); `pim/test_harness/FINDINGS.md`'s "Remaining follow-ups"
section gets a matching resolved-pointer entry. This plan
(`rpc_pubsub_interchangeability_plan.md`) is itself the pointer artifact both
of those updates point back to; the relationship to the broader adoption
path is already recorded in §6 above.

This closes the plan: Phases 0-5 are all executed, evidenced, and green.

### Phase 5 addendum — PR review, executed 2026-07-09

Two Codex review comments on the Phase 5 PR, both real correctness gaps in
that phase's own fix, not pre-existing bugs:

- **P1**: routing a streaming client invoke under the pre-existing
  `consumed` kind meant `pcl_endpoint_required_caps()` only ever demanded
  `PCL_CAP_RPC_UNARY` for it (the same requirement a *unary* consumed route
  has) — a manifest could compose successfully against a unary-only peer
  for a streaming endpoint, failing only once the stream was actually
  invoked. Fixed by adding a new `PCL_ENDPOINT_STREAM_CONSUMED` endpoint
  kind (manifest token `stream_consumed`), symmetric with the
  provider-side `PCL_ENDPOINT_STREAM_PROVIDED`/`stream_provided` that
  already existed: `pcl_endpoint_required_caps(STREAM_CONSUMED)` requires
  `PCL_CAP_RPC_STREAM`; `pcl_executor_invoke_stream()`'s route-table lookup
  (this phase's own REQ_PCL_470 fix) now keys on it instead of `CONSUMED`.
  `binding_contract.py::_service_endpoint_kind()` emits `stream_consumed`
  for a consumed-role streaming rpc (mirroring its existing
  `stream_provided` branch for the provided role); `agra_seam_interchange_test.cpp`'s
  hand-written manifest updated to match.  New `PCL_ENDPOINT_STREAM_CONSUMED`
  requires touching: `pcl_types.h` (enum), `pcl_capabilities.c` (required
  caps), `pcl_transport_routing.c`/`.h` (grammar token + doc), `pcl_executor.c`
  (route lookup kind + the existing LOCAL|REMOTE-forbidden guard extended to
  the new kind too). Documented as `REQ_PCL_472`, with a new negative gtest
  (`StreamingRouteRejectsStreamIncapableTransport`, routed against the
  existing unary-only `pcl_transport_nocaps_plugin`) proving the exact
  scenario the reviewer described now fails closed at compose time.
- **P2**: `contract_routing_manifest.py`'s route-line writer read
  `endpoint_name`/`kind` off `InteractionEndpoint` but never carried a
  reliability floor, silently dropping the QoS token the old
  `endpoint_requirements`-based path used to append — a generated manifest
  could route a contract-marked-`RELIABLE` endpoint to a `best_effort`-only
  transport and compose successfully. Fixed by joining back to
  `binding_manifest.json`'s `endpoint_requirements` (keyed by
  `(endpoint_name, kind)`) for the floor, since `interactions` itself
  carries no QoS field. Fixing P1 and P2 together changed
  `build_contract_routing_test.sh`'s generated manifest for the first time
  in a way that mattered: the `*.read` consumed-side route now correctly
  emits `stream_consumed reliable` instead of `consumed` (no floor), which
  then correctly failed closed against `contract_transport_plugin.c`'s
  `"mode":"rpc"` stub (declared only `RPC_UNARY`) until that stub's
  declared caps were updated to include `RPC_STREAM` too — a real
  demonstration of P1's fix working end-to-end through the actual
  generator path, not just the new unit tests.

**Verification**: `test_pcl_transport_routing.cpp`
(`StreamingInvokeDispatchesThroughRoutedTransport` updated to route
`stream_consumed`; new `StreamingRouteRejectsStreamIncapableTransport`),
`test_pcl_capabilities.cpp` (`EndpointRequiredCaps` extended;
`LoaderUsesExplicitCapsSymbol` updated for the capture plugin's now-honest
`RPC_STREAM` declaration). Full PCL suite: 733/733 (up from 732 — the one
new negative test). Full Phase 5 run matrix re-run clean:
`PASS (0 failure(s))`. All prior harnesses re-run green, including
`build_contract_routing_test.sh` (now genuinely exercises and validates the
capability gate rather than silently passing). `gen_hlr_coverage.py`: 0
gaps (92 HLRs, 387 LLRs). `PCL.078`/`REQ_PCL_470`-`472` updated to reflect
the corrected design.

### Phase 5 addendum 2 — further PR review, executed 2026-07-09

A second Codex review pass (on a9dba96, the first addendum's own fix)
found three more real gaps, all downstream consumers of the new
`PCL_ENDPOINT_STREAM_CONSUMED` kind that the first addendum introduced but
did not itself update:

- The QoS-floor join added a real bug of its own:
  `EndpointRequirement.reliability` is the sentinel string `"unspecified"`
  (not empty) for an endpoint with no declared QoS floor, and the fix's
  truthy check passed that straight through as a manifest token --
  `pcl_transport_routing.c` only accepts `reliable`/`best_effort`, so this
  would have broken the *common* case (most endpoints have no explicit
  floor) rather than just failing to preserve one. Fixed by only emitting
  the token for the two real values.
- `subprojects/PYRAMID/pim/cpp/components_gen.py`'s `ConsumedService`
  (the file-wide facade the interaction facade's `consumedService()`
  accessor exposes) still hardcoded `PCL_ENDPOINT_CONSUMED` for every rpc
  uniformly in `routeAllRemote()`/`routeAllLocal()` -- a real component
  calling `consumedService().routeAllRemote("peer")` for a streaming `Read`
  (exactly the interaction facade's own requirement leg) would install the
  route under the wrong kind and `pcl_executor_invoke_stream()` would
  silently miss it, falling back to local/intra-process dispatch instead of
  the configured peer. Fixed to select the stream kind per-rpc from
  `rpc.server_streaming`; new `pim/test_stream_consumed_routing.py` pins
  the generated route calls.
- The same hardcoded-kind gap existed in the Ada generator
  (`service_body_gen.py`'s `Configure_Consumed_Transport`) and in the
  hand-maintained Ada mirror of the C enum
  (`subprojects/PCL/bindings/ada/pcl_bindings.ads`, missing the new literal
  entirely). Fixed analogously; verified by clean Python-level
  regeneration only, consistent with Phase 4's carried Ada
  compile-verification limitation (no GNAT/gprbuild in this environment).

**Verification**: full PCL suite 733/733 (unchanged count -- these three
fixes are Python generator + one Ada spec literal, no new PCL test);
`pim/test_stream_consumed_routing.py` new and passing; full Python suite
53/53; Phase 5 harness re-run `PASS (0 failure(s))`; every prior harness
including `build_contract_routing_test.sh` re-run green (its generated
manifest now correctly reads `...read stream_consumed ... reliable`).
`gen_hlr_coverage.py`: 0 gaps. `REQ_PCL_472`'s ledger entry extended with
these follow-on fixes' detail rather than filed as a separate LLR, since
none of them are PCL library behaviour -- they are PYRAMID generator
correctness for consumers of PCL.078's already-documented contract.

### Phase 5 addendum 3 — further PR review, executed 2026-07-09

A third review pass (on 410dac9, the second addendum's own fix) found two
more real gaps:

- Both `pcl_executor_invoke_async()` (pre-existing, unaffected by this PR
  directly) and `pcl_executor_invoke_stream()` (this PR's new code,
  faithfully mirroring the older function's structure) let an explicit
  `PCL_ROUTE_LOCAL` route fall through a "handled below" comment straight
  into the legacy executor-wide transport fallback (`e->has_transport &&
  e->transport.invoke_async`/`invoke_stream`) *before* ever reaching
  intra-process dispatch -- so a deliberate local-route override was
  silently ignored whenever a default transport also happened to be
  registered on the executor. Confirmed this predates the PR in
  `invoke_async`; fixed both functions symmetrically (leaving the
  inconsistency between two near-identical functions would be worse, and
  the reviewer's comment landed on code in the file this PR was already
  modifying) via a `route_forces_local` flag set when `route->route_mode
  == PCL_ROUTE_LOCAL`, gating the fallback check with
  `!route_forces_local &&` rather than restructuring the fallback's
  position. New `REQ_PCL_473`, traced to PCL.078; two new gtests
  (`PclExecutor.InvokeAsyncExplicitLocalRouteBypassesDefaultTransport`,
  `PclStreaming.InvokeStreamExplicitLocalRouteBypassesDefaultTransport`)
  register a default transport with a spy `invoke_async`/`invoke_stream`
  alongside an explicit `PCL_ROUTE_LOCAL` route and assert the transport
  is never called.
- `subprojects/PYRAMID/pim/cpp/components_gen.py`'s generated
  `*_components.hpp` preamble never emitted `#include <unordered_map>`,
  despite `interaction_facade_gen.py` generating a `std::unordered_map`
  member (`snapshot_index_`, the D6 snapshot store) whenever a
  `RequestPortProvider` is emitted -- a latent compile-fragility bug for
  any consumer lacking a transitive include. Fixed by adding the include
  to the preamble.

Also: GNAT and gprbuild were installed in this environment during this
round (`apt-get install gnat gprbuild`), retroactively closing the "no
GNAT/gprbuild in this environment" compile-verification gap both Phase 4
and Phase 5 addendum 2 carried for Ada codegen. `pcl_bindings.ads`
(including addendum 2's new `PCL_ENDPOINT_STREAM_CONSUMED` literal) now
compile-checks cleanly (`gcc -c -gnat2020 -gnatc`, exit 0), and the full
`test_generic_ada.py` suite (7/7, including the two Ada object-compile
tests that previously failed for lack of a compiler) passes.

**Verification**: full PCL suite 735/735 (net +2 for the two new
`route_forces_local` tests); full Python suite across `pim/` and
`PYRAMID/tests/` 60/60 (up from the previously-reported 53, now including
the newly-passing Ada compile tests); `pcl_bindings.ads` direct
`gnatc` check passes; generated `*_components.hpp` spot-checked to confirm
`#include <unordered_map>` now precedes the `snapshot_index_` member's
use of it; Phase 5 harness re-run `PASS (0 failure(s))`; every sibling
harness (`build_comms_test.sh`, `build_routed_egress_test.sh`,
`build_agra_shm_comms_test.sh`, `build_agra_udp_proof_test.sh`,
`build_agra_mixed_route_test.sh`, `build_contract_routing_test.sh`)
re-run green. `gen_hlr_coverage.py`: 92 HLRs / 388 LLRs, 0 gaps.

### Phase 5 addendum 4 — Codex review, executed 2026-07-09

An automated Codex review pass (on 539b83a, addendum 3's own fix) found two
more real gaps, both scoped fail-closed/late-join guarantees this PR's own
work had already established for the common case but not yet for a less
obvious variant of it:

- `validate_exclusivity()` (`pcl_transport_routing.c`, D5's `exclusive`
  group enforcement) only scanned `r->routes` -- the endpoint routes the
  *current* `pcl_transport_routing_load()` call itself installed -- when
  checking whether both sides of a declared group were routed. A group
  conflict split across two loads into the same executor (or a manifest
  routing only one side against a side already routed
  programmatically) composed successfully and left both realizations
  active, since the check never saw the pre-existing side at all. Fixed
  by adding `pcl_executor_endpoint_route_exists_any_kind()` (a
  kind-agnostic sibling of the existing per-kind
  `pcl_executor_endpoint_route_exists()`, since a group's side members
  are plain names with no fixed kind) and querying the live executor
  directly -- which, since every route this load installs is itself
  already applied to the executor by the time `validate_exclusivity()`
  runs, subsumes the old manifest-load-scoped check without needing to
  keep both. New `REQ_PCL_474`, traced to PCL.077; new gtest
  `PclTransportRouting.ExclusiveGroupConflictDetectedAgainstPreExistingRoute`
  installs a programmatic side-A route before the manifest load, routes
  only side B from the manifest, and asserts the load fails closed
  naming both endpoints.
- `interaction_facade_gen.py`'s generated `RequestPortProvider`
  (`registerOpenStream()`) never consulted the D6 snapshot store
  (`snapshot_index_`) when a new RPC Read stream opened -- unlike the
  pub/sub path's `dispatchPubsubCommand()`, which already calls
  `republishSnapshotFor()` after every command as its own late-join
  mitigation. A client opening an RPC `transitions()` stream for an id
  already in a terminal state (e.g. COMPLETED/CANCELLED, with no further
  transition ever coming) would wait forever, since `fanOutRpc()` only
  ever delivers *future* transitions to *currently open* streams. Fixed
  by a new `replayStoredSnapshots(OpenStream&)` helper, called once from
  `registerOpenStream()` right after a stream is registered: it sends
  every snapshot matching the stream's query id filter (or every stored
  snapshot, when the filter is empty -- mirroring `fanOutRpc()`'s same
  "empty id list matches everything" semantics) directly to that
  stream's own writer, deliberately not through
  `sendTransition()`/`fanOutRpc()`, which would also re-deliver to every
  other already-open matching stream. New compiled gtest
  `PclGeneratedInteractionFacadeProvider.LateRpcStreamReplaysStoredSnapshot`
  (`test_pcl_generated_interaction_facade.cpp`) sends a transition
  *before* any Read stream is open, then subscribes and asserts the
  stored snapshot is delivered immediately -- the RPC-mode sibling of
  the file's existing pub/sub late-command republication test. This is
  PYRAMID generator behaviour, not PCL library behaviour, so (consistent
  with how the non-PCL follow-on fixes in addenda 2-3 were handled) it
  gets no PCL LLR entry of its own; the dedicated regression test is the
  record.

**Verification**: full CTest suite 737/737 among tests already running in
this environment before this round (net +2 for the two new tests above);
installing GNAT in this environment during addendum 3 also surfaced 12
newly-appearing Ada-target CTest entries this build tree had never
attempted before, one of which (`pyramid_ada_build_artifacts`) fails on a
pre-existing, unrelated `service_body_gen.py` bug -- an empty
`begin...end` block in a `sensor_data_interpretation` consumed-service
procedure with no publisher endpoints to configure, invalid Ada syntax
that predates this PR entirely and touches no file this PR modifies;
left out of scope (no reviewer raised it, and fixing it is unrelated to
this PR's actual review comments). Full Python suite (`pim/` +
`PYRAMID/tests/`) 60/60; Phase 5 harness re-run `PASS (0 failure(s))`;
every sibling harness re-run green. `gen_hlr_coverage.py`: 92 HLRs / 389
LLRs, 0 gaps.
