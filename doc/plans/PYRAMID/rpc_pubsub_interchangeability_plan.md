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
