# High-Efficiency In-Process Port Bindings — Gap Analysis & TODO

**Status: first cut scheduled (2026-07-21). Scope decisions recorded below.**

## Decisions for the first cut (2026-07-21)

These choices were made to keep the first cut small, provably safe, and
strictly additive. They resolve the open questions the TODO items raise. Each
decision names the item it settles.

1. **Scope — Tier A only (N3a, N4, N5).** The first cut implements the
   same-executor, zero-copy pointer handoff (Tier A). Tier B (native handoff
   across sibling executor threads, N3b) is deferred to a follow-up: it needs a
   new in-process sibling-executor route in PCL and cross-thread ownership
   transfer that Tier A does not. Where a component requests `native` but the
   peer is on a different executor, the runtime falls back to the serialized
   path rather than failing to route. Decisions 5 and 6 below (Tier-B ownership
   mechanism, Tier-B native services) are therefore recorded as the intended
   follow-up direction, not built now.
2. **Provenance guard — reject at transport ingress (N2b, option a).** A
   transport-originated message carrying the native sentinel is refused at the
   ingress choke point. Same-executor dispatch does not pass through transport
   ingress, so a native sentinel reaching a trampoline is guaranteed to be a
   live local object. The subscriber/handler callback ABI is left unchanged
   (option b is not taken).
3. **Mixed local + remote fan-out — forbid at compose time (N6).** A native
   publish on a route that includes `PCL_ROUTE_REMOTE` is rejected before any
   local delivery occurs — this covers a mixed `PCL_ROUTE_LOCAL|PCL_ROUTE_REMOTE`
   route *and* a route with **several** remote peers (fan-out is one-to-many;
   see the fan-out analysis below). Mixed-route topics use the serialized path.
   The encode-once-for-remote-plus-native-for-local option is not built in the
   first cut.
4. **Cross-language — C++-to-C++ only (N7).** Native handoff is C++-to-C++
   only. C++ to Ada in-process native handoff is out of scope for the first cut;
   it needs a struct-ABI-compatibility decision, not just plumbing. Any
   cross-language leg falls back to serialization.
5. **(Follow-up, Tier B) ownership mechanism — reference-counted handle (N1,
   N3b).** When Tier B is built, a published native object is held by shared
   ownership (a reference-counted handle) that keeps it alive until every
   subscriber on **every** receiving executor has run, rather than a single
   move into one queue. Pub/sub fans one object out to several subscribers,
   possibly spread across several sibling executors, so a single move cannot
   serve them — shared `const` ownership referenced once per receiving-executor
   queue is the model the fan-out analysis below requires. Not built in the
   first cut.
6. **(Follow-up, Tier B) native services scope — pub/sub only first (N3b, N4).**
   When Tier B is built, the first Tier-B increment covers native pub/sub only;
   sibling-executor native RPC (request/response/stream) is a later increment
   and falls back to serialized until then. Not built in the first cut.
7. **Deployment surface — route-JSON field (N5).** The native opt-in is a field
   on the existing route JSON, `{"transport":"local","payload":"native"}`, not a
   new `.ports` payload column. The default stays serialized (`payload` absent
   or `"serialized"`).

## First-cut implementation status (2026-07-21)

What is built and verified, and what is deliberately deferred within the
Tier-A-only scope.

| Item | Status | Notes |
|------|--------|-------|
| **N1** native payload contract | **Done** | `PCL_NATIVE_CONTENT_TYPE` sentinel + `pcl_msg_is_native()` in [`pcl_types.h`](../../../subprojects/PCL/include/pcl/pcl_types.h); normative contract in [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md). |
| **N2** fail-closed egress guard | **Done** | Rejects native on publish, service request, deferred response, and stream frame across the executor, container, and shared-memory gateway paths. |
| **N2b** inbound provenance guard | **Done** | Reject-at-ingress (option a): the cross-thread pub/sub, service-request, and response queues, and the shared-memory gateway's direct handler dispatch, all refuse a native sentinel. |
| **Port abstraction** | **Done** | Native is first-class on the standard `pcl::Port` surface: `pcl::Port::publishNative(value, schema)` and `pcl::nativePayload<T>(msg)` in [`component.hpp`](../../../subprojects/PCL/include/pcl/component.hpp), usable without the generated facade. |
| **N3a** generator Tier-A native pub/sub | **Done** | Opt-in `--native-inprocess` flag emits `publish<Topic>Native()` (delegating to `pcl::Port::publishNative`) and a native subscriber trampoline (via `pcl::nativePayload`). Default output byte-for-byte unchanged (verified by `diff -qr`). |
| **N5** deployment surface | **Done (pub/sub)** | `configurePubSubTransport({"transport":"local","payload":"native"})` selects native; rejected on a remote route. |
| **N6** mixed local+remote fan-out | **Done (runtime)** | Runtime refuses a native publish on any REMOTE-inclusive route *before* the local leg runs, so no local side effect leaks. This is the "rejected before local dispatch" arm of the N6 decision; a separate compose-time validator is not added because the runtime already fails closed. |
| **N4** generator native services | **Runtime done; codegen deferred** | The PCL runtime for native services is complete and tested: a native unary/deferred response bypasses the executor-default transport and returns to the local callback ([`pcl_service_respond`](../../../subprojects/PCL/src/pcl_container.c)), and a native local invoke via an explicit LOCAL route skips the transport. The **generator emission** of native request/response/stream trampolines and the owned response slot is deferred to the next increment. |
| **N7** benchmarks + Ada scope | **Ada scope recorded; benchmark deferred** | C++-to-C++ only recorded (Decision 4). The microbenchmark/parity harness is a follow-up. |

Tests: [`subprojects/PCL/tests/test_pcl_native_binding.cpp`](../../../subprojects/PCL/tests/test_pcl_native_binding.cpp)
covers the sentinel, the Tier-A local publish delivering by pointer, the
mixed-route refusal before local delivery, all four ingress guards, and both
native-service response paths.

## Fan-out (port multiplicity) and the deferred native items

The deferred items (Tier B, and the "support mixed native" arm of N6) cannot be
scoped against the single "one local subscriber plus one remote subscriber"
picture the original write-up used. A pub/sub port fans out to *many*
destinations, and the native work has to account for that multiplicity from the
start. This section records how fan-out works today and what each deferred item
must add.

### How fan-out works today

- **Pub/sub is one-to-many; RPC and streaming are one-to-one.** A publisher
  delivers to every matching subscriber. A *consumed* service or stream
  endpoint is restricted to at most one peer — `pcl_executor_invoke_async` and
  `pcl_executor_invoke_stream` both reject `peer_count > 1`
  (`subprojects/PCL/src/pcl_executor.c`). So the multiplicity problem below is a
  **pub/sub** problem only; the deferred native *services* work stays
  point-to-point and does not inherit it.
- **Local fan-out.** `dispatch_incoming_now` loops the containers on the
  publisher's executor and calls each matching subscriber with the **same**
  `pcl_msg_t`. For a native publish this already means one `const` object
  pointer is shared by all local subscribers (the read-only-sharing rule,
  constraint 3) — local fan-out needs nothing new for Tier A.
- **Remote fan-out.** `pcl_executor_publish_port` iterates the port's named
  peers — up to `PCL_MAX_ENDPOINT_PEERS` (8) — and calls each peer transport's
  `publish` with the same message; a peer_count of zero uses the single default
  transport. Crucially the payload is **encoded once** (the generated
  `publish<Topic>` helper serializes into one buffer before calling
  `pcl_port_publish`), and that one buffer is reused across all peers. Remote
  fan-out is already encode-once, not encode-per-peer.
- **Mixed.** When a route has both bits, the local leg runs first, then the
  per-peer remote loop.
- **Durability.** There is **no** retained/last-value/transient-local mechanism
  in PCL today, so the durability concern (constraint 6) is forward-looking:
  there is nothing to retain yet, and nothing the native path breaks today.

### The general shape the deferred work must handle

A single pub/sub publish can, in general, fan out across up to four destination
classes **at once**, each wanting a different payload representation:

| Destination class | Multiplicity | Representation the native design needs |
|-------------------|--------------|----------------------------------------|
| Same-executor local subscribers (Tier A) | N | one **borrowed** `const` pointer, shared |
| Sibling-executor subscribers (Tier B) | across M sibling executors | one **shared-owned** native object (reference-counted), referenced once per receiving-executor queue |
| Remote peers (serialized) | up to 8 named peers, or the default | **one** encoded buffer, reused across all peers |
| Durable/retained snapshot (if durability is ever added) | 1 | one **owned** copy (encoded or native) |

The single most important consequence: the "one local + one remote" framing is a
special case. The mixed native publish is really *native-for-all-local* +
*native-shared-for-all-siblings* + *one-encode-shared-across-all-remote-peers*.
So the deferred work must produce **at most one** encoded buffer per publish (not
one per peer) and **at most one** native representation (shared, not moved), then
route each destination class to the representation it needs.

### What each deferred item adds

- **Tier B sibling-executor native (N3b).** This is where multiplicity forces
  the ownership decision. A single move into one queue cannot serve N local
  borrowers plus M sibling-executor queues; the object must be held by a
  reference-counted handle, incremented once per sibling-executor queue entry
  and released when that executor has drained it. The handle's lifetime spans
  the *maximum* over all receiving executors, not one. Decision 5 is updated
  accordingly. Local Tier-A subscribers still borrow synchronously and take no
  reference.
- **Mixed local+remote native (the "support" arm of N6).** The publish must
  compute the encoded buffer once — only when there is at least one remote peer
  (or a durable snapshot to back) — hand the borrowed/shared native
  representation to the local and sibling legs, and reuse the single encoded
  buffer across all K remote peers. Because the local leg currently runs before
  the remote legs, the native/encoded split has to be decided **up front**
  (before any dispatch), so a per-peer encode failure cannot leave a native
  local delivery already committed. Until this is built, the first cut keeps the
  simpler rule: any route with a remote leg (one peer or several) refuses native
  and uses serialization (Decision 3).
- **RPC/streaming native (N4 codegen).** No fan-out change: these stay
  one-to-one by the `peer_count > 1` rejection above, so the native request /
  response / stream-frame work does not need a multi-peer representation split.
  Recording this so that work is not over-built.
- **Durability.** Out of scope until a retained-value feature exists at all. If
  one is added, its snapshot is simply a fifth consumer that shares the single
  encoded buffer (a borrowed native pointer cannot back a retained value —
  constraint 6).

### Multiple port instances — another axis of multiplicity

The classes above assumed one port per topic. A component may define **several
ports on the same topic name**, and this multiplies the picture again — and it
is directly relevant to native, because different port instances can carry
different payload representations.

**What is supported today (verified in code and test):**

- **Multiple ports, same name — yes.** `validate_port_definition` imposes no
  uniqueness check; a container may add up to `PCL_MAX_PORTS` (64) ports,
  including several publishers or subscribers on the same topic.
- **Per-port routes — yes, programmatically.** Each `pcl_port_t` carries its own
  `route_mode` / `route_configured`, set by `pcl_port_set_route`
  (`pcl::Port::setRoute` / `routeLocal` / `routeRemote`). Publish operates on a
  specific port handle, and `dispatch_incoming_now` evaluates each subscriber
  port's own route, so two same-named ports genuinely route independently. The
  test `MultiplePortInstancesPerTopicCarryDifferentRepresentations` builds a
  native-local publisher and a serialized-remote publisher on one topic and
  shows each behaves on its own route.
- **The `.ports` / manifest surface cannot — it collapses by (name, kind).** The
  endpoint-route table (`pcl_executor_set_endpoint_route`, used by the manifest
  loader and by the generated *consumed-service* facade) is keyed by
  `(endpoint_name, endpoint_kind)`, and `port_route_mode` consults it **before**
  the per-port route and lets it **win for every port of that name and kind**.
  So the deployment-surface path cannot give two same-named same-kind ports
  different routes, and installing any manifest route for a name overrides all
  per-port routes for it. Pub/sub facade routing (`routeAllPublishersLocal`,
  etc.) uses the per-port path, so it is not subject to this collapse; the
  manifest path and consumed-service routing are.

**Why this matters for native:**

- **Different ports can hold different representations.** A native-local
  publisher port plus a serialized-remote publisher port on one topic together
  express the mixed local+remote fan-out that the single-port N6 rule forbids —
  the author publishes the live object on the native port and the encoded value
  on the remote port, with no double local delivery (only the local port has a
  local leg). This is a legitimate pattern **today** and a lighter alternative
  to the deferred single-port encode-once split. The same holds on the
  subscriber side: a LOCAL-only subscriber instance receives native while a
  REMOTE-from-peer instance of the same topic receives serialized, and
  `route_accepts` steers each ingress to the right one.
- **Native selection is per-port (implemented for publishers).** The port
  abstraction always supported per-instance selection (`publishNative` on a
  specific port); the generated facade now does too. Each publisher topic has
  its own native flag and a `configure<Topic>Transport(json)` that routes that
  one publisher and sets its native preference independently, so two publishers
  on one facade can differ (one native-local, one serialized-remote). The bulk
  `configurePubSubTransport` remains available for the uniform case and clears
  the native flags when it routes remote. Subscribers need no per-port switch —
  the native trampoline recognises a native message itself. The one piece still
  outstanding is the **manifest**: because the endpoint-route table keys by
  `(name, kind)` it cannot carry a per-instance native/route choice for two
  same-named ports, so a manifest-driven per-instance selection still needs the
  discriminator described below. Programmatic and JSON-config selection are
  done.
- **Deployment-surface gap to close with the deferred work.** If a deployment
  wants to express "native local + serialized remote" as two same-name ports,
  the manifest route table cannot currently give them distinct routes (the
  (name, kind) collapse above). The deferred mixed-native design therefore has a
  fork to record: either (a) extend the endpoint-route table with a per-instance
  discriminator so same-named ports can be routed separately, or (b) keep one
  port per name and do the encode-once/native split inside a single publish.
  Option (b) avoids touching the routing key but requires the single-port split;
  option (a) enables the lighter two-port pattern under the manifest.

### Does anything need adding to fan-out *now*?

No change to the first cut. Tier-A local native fan-out already works (one shared
`const` pointer to N subscribers), multiple same-named ports already route
independently through the port abstraction, and every remote-inclusive route —
one peer or eight — is uniformly refused for native. Per-port native selection on
the generated facade is already in place (a per-topic flag plus
`configure<Topic>Transport`), so publishers can differ within one facade today.
The multiplicity work that remains lands with Tier B and the mixed-native arm,
which must add: (1) reference-counted shared ownership across all receiving
executors; (2) a single encoded buffer reused across all remote peers in one
publish; and (3) for the **manifest** path only, a per-instance discriminator on
the endpoint-route table so two same-named ports can carry different routes /
native choices (the programmatic and JSON-config paths already can). The
decisions above are updated so those items are built against the one-to-many,
many-ports model rather than a one-to-one special case.

## Motivating deployment: A-GRA Objectives → Tasks, onboard and peer

The concrete driver for the mixed-route native work is an A-GRA (Autonomy
Government Reference Architecture) deployment. A-GRA's Mission Autonomy
decomposes Effect → Action → Task; its **Objectives (COMP-039)** and **Tasks
(COMP-062)** components are exactly the AME planning/execution role (see
[`doc/research/AME/a_gra_e2e_worked_example.md`](../../research/AME/a_gra_e2e_worked_example.md)
§7–8 and [`a_gra_standard_review.md`](../../research/AME/a_gra_standard_review.md)).
An Objectives component must drive a Tasks component in up to three places at
once:

1. **Onboard, same process, same executor** — a co-located Tasks. The Tier-A
   native path: hand the live task command over by pointer, zero copy.
2. **Onboard, same host, separate process** — a Tasks in a sibling process,
   reached over the shared-memory transport. A *local* deployment but a
   *transport* route: it serializes (native is refused on a transport route, by
   design), though it stays on-box with no network. (A same-process sibling
   *executor* would be Tier B once built; a separate process is SHM.)
3. **Peer ACP** — a Tasks on another platform, over the A-GRA **Peer-to-Peer
   (P2P)** L1 interface (UCI/EXI over DDS). A remote route through the P2P
   transport plugin; serialized.

Two facts from A-GRA shape the model (worked example §8):

- **A-GRA is pub/sub end to end — there is no RPC.** Every "service" is a
  correlated command/`*Status` object pair over a topic. So Objectives → Tasks is
  a *topic*, inherently one-to-many, not a point-to-point call. The one-to-one
  RPC constraint in PCL does not bind A-GRA traffic; fan-out to the onboard Tasks
  **and** one or more peers is the normal case, not an edge case.
- **Instances are disambiguated by in-band header IDs, not by per-instance topic
  names.** The topic name equals the message name and is *shared* across the
  onboard and peer Tasks; which instance a command targets is carried in the
  message header (IDs), and which instances receive it is a matter of *routing*
  (which transports/peers the topic is bound to). A-GRA does **not** mint a
  distinct topic per Tasks instance.

The routing consequence is the four-destination fan-out from the analysis below,
made concrete: a single Objectives publish of one task command wants to reach
(1) the co-located Tasks as a **native pointer**, and (2)+(3) the SHM-sibling and
P2P-peer Tasks as **one serialized buffer**, encoded once and reused across both
transports. That is precisely the deferred "support mixed native" arm of N6 — so
A-GRA makes that arm the **primary** target of the next increment, not an
optional extra.

### What this means for the earlier open questions

- **One reusable facade, realization by annotation — do not split the contract.**
  The pub/sub projection is already expressed as `pattern` / `topic` annotations
  on each rpc of the single service contract. In
  `MA_MissionPlanCommand_Service`, for example, `Create` / `Update` / `Cancel`
  carry `pattern: SUBSCRIBE topic: "MA_MissionPlanCommand"` and `Read` carries
  `pattern: PUBLISH topic: "MA_MissionPlanCommandStatus"`
  ([`pim/agra_p2_seam`](../../../subprojects/PYRAMID/proofs/contracts/agra_p2_seam) overlay).
  The generator turns those annotated operations into the facade's
  `subscribe<Topic>` / `publish<Topic>` ports directly, so **one** facade already
  presents the interaction with its pub/sub realization built in — that is the
  whole point of the port binding being a single reusable facade. The
  mixed-native fan-out therefore extends *that* facade's `publish<Topic>` to
  deliver native-local and serialized-peer in one call, on the ports already
  generated. It must **not** fork the contract into a parallel pub/sub binding or
  a separate facade — doing so would defeat the reuse the facade exists for. (An
  earlier note in this file's history suggested sequencing the mixed-native work
  behind a separate pub/sub projection; that was wrong. The projection is
  annotation-driven on the same contract, and the per-port routing / native
  switch already attach to these generated topic ports.)
- **Port naming / per-instance topics.** A-GRA answers this the opposite way from
  a "distinct name per instance" design. The onboard and peer Tasks share **one**
  topic identity; they are told apart by route and by header IDs, not by
  different port names. So what matters is a per-instance *route* (one
  shared-named port carrying a native-local leg and a peer-remote leg), not
  per-instance *naming* / remapping — that is not the A-GRA mechanism and need
  not be built for this use case.
- **What works today.** The per-port routing and native switch already landed let
  an Objectives component realise this use case **now** with two publisher ports
  on the shared topic: one routed local and published native
  (`publish<Task>Native` / `configure<Task>Transport({"payload":"native"})`) for
  the onboard leg, and one routed remote to the peer and published serialized for
  the P2P (or SHM) leg. It costs two publish calls and two ports, but it is fully
  supported and tested.
- **What the next increment buys.** The single-port mixed fan-out collapses those
  two into one publish: encode once for the SHM/P2P peers, hand the native
  pointer to the onboard leg, with the fail-closed guard moved to compose time
  (per the N6 decision). That is the ergonomic A-GRA realisation and the concrete
  goal for the mixed-native (and Tier-B) work.

## Motivating deployment: PYRAMID bridge (consumed ↔ provided)

The second driver is the PYRAMID **bridge** pattern. In PYRAMID a service is
defined once; a consumer gets a generated `Consumed` facade and a provider a
`Provided` facade from that one contract, so **components do not depend on each
other's service definitions**. A *bridge* wires one component's consumed
endpoint to another's provided endpoint — often translating vocabulary between
two independently-defined contracts (the planned `agra_c2_bridge` translating
`PlanningRequirement` ↔ `MA_Action` is exactly this; the intra-process
[`pcl_bridge`](../../../subprojects/PCL/include/pcl/pcl_bridge.h) is the
same idea at the topic level: subscribe, transform, dispatch).

The efficiency case the plan must serve: the bridge is deployed **local to the
consumer** — `component 1 → bridge` is native (co-located), while `bridge →
component 2` is remote (serialized). The important observation is that **the
bridge is the representation-change boundary**, and it decomposes the mixed
local+remote problem into two hops that each carry a *single* representation:

- **c1 → bridge: native, local.** c1 hands the live object to the bridge by
  pointer (Tier A). The bridge reads it through the native subscriber/handler.
- **bridge → c2: serialized, remote.** The bridge encodes and sends over the
  transport. Because a native payload is borrowed and read-only for the callback
  only, the bridge must **encode inside that callback** (a transform does this
  synchronously) — or take an owned copy first if it defers the forward. The
  fail-closed guard makes this correct-by-construction: the bridge *cannot*
  route the borrowed native pointer onto its remote leg (native on a transport
  route is refused), so it is forced to serialize at the boundary.

Two consequences:

- **The bridge topology does not need the single-port mixed fan-out.** Each of
  its two ports has one route and one representation, which the per-port routing
  and native switch already landed handle directly: the c1-facing port is
  native-local, the c2-facing port is serialized-remote. So for the *decoupled
  point-to-point* case, the deferred mixed-native arm is unnecessary — the bridge
  puts the encode where it belongs and keeps every port simple.
- **The two motivating deployments split cleanly.** The A-GRA broadcast case
  (one Objectives publish reaching a co-located Tasks natively *and* peers over
  P2P, on one shared topic) is the genuine 1:N mixed fan-out that still wants the
  single-port encode-once path. The bridge case is decoupled point-to-point and
  is fully served today by two single-representation ports. Knowing which
  topology a deployment uses tells you whether it needs the deferred mixed-native
  arm at all.

## What this is about

When two PYRAMID components run in the **same operating-system process**, they
can exchange a message as the native typed object (the generated C++ struct)
without ever serializing it to a wire format. This document calls that
**high-efficiency in-process binding**. The generated **port-style bindings**
(the typed publish/subscribe and service facades described in
[`cpp_component_authoring.md`](../../../subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md)
and [`component_skeletons.md`](../../../subprojects/PYRAMID/doc/architecture/component_skeletons.md))
do not offer this: even when a port is routed entirely in-process, the
generated code encodes the typed value into a serialized buffer on the way out
and decodes it again on the way in. This document explains the gap precisely
and lists the work needed to close it.

### "In-process" is not one case — it is two

An important distinction, easy to lose: two components in the same process may
or may not share the same PCL executor (executor = one worker thread that ticks
components and runs their callbacks). The efficient path is different in each
case, and a solution has to handle both:

| Tier | Where | What is possible | What is still needed |
|------|-------|------------------|----------------------|
| **A — same executor** | Same process, **same executor thread** | True **zero-copy**: hand the subscriber a pointer to the live object, synchronously, on the one thread. No copy, no queue, no serialization. | Nothing beyond a borrow that lasts the dispatch call. This is PCL.022 direct dispatch. |
| **B — different executor, same process** | Same process, **different executor threads** | No **wire serialization** — the data stays in-process memory. But the two threads do not share a call stack. | A cross-thread **queue** and a thread-safe **ownership transfer** (a native struct copy or move, or shared ownership) — a "minimal data change", not zero-copy, and not synchronous. |
| **C — different process / host** | Separate processes or machines | — | Full serialization + a transport adapter. Out of scope here; this is what the bindings already do. |

Tier A is the headline win (zero-copy). Tier B still avoids the encode/decode
CPU and the wire-format round trip; it trades the *pointer* handoff for a
*native-object* handoff across the executor's ingress queue. Both are "high
efficiency" relative to today's always-serialize behaviour; only Tier A is
literally zero-copy. The plan below keeps them as separate acceptance cases so
Tier B's threading and lifetime rules are not smuggled in under Tier A's
simpler borrow contract.

A note on the term: "high efficiency" is used here as a plain description of
the zero-serialization local path, not as a formal term from the PYRAMID
Technical Standard. The wire-efficient FlatBuffers path
([`standard_alignment.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/tactical_objects/standard_alignment.md))
is a *smaller* serialized form; it is not the same thing as skipping
serialization entirely.

## The two claims, checked against the code

### Claim 1 — PCL supports zero-copy, no-serialization in-process handoff

Yes. The PCL runtime is deliberately format-agnostic and already carries a
raw object pointer through a port:

- **Tier A — direct dispatch (same executor).** With no transport adapter
  set, the executor routes a published message straight to matching
  subscribers by pointer, on the single executor thread. This is requirement
  **PCL.022 — Intra-Process Direct Dispatch** ("zero-copy pointer handoff",
  [`subprojects/PCL/doc/requirements/HLR.md`](../../../subprojects/PCL/doc/requirements/HLR.md)),
  and design principle **P5 — Zero-copy where possible**
  ([`pcl-component-system.md`](../../../subprojects/PCL/doc/architecture/pcl-component-system.md)).
- **Tier B — cross-thread ingress (different executor, same process).** An
  external thread hands a message to an executor with
  `pcl_executor_post_incoming()` (requirements **PCL.025 / PCL.026**); the
  executor dispatches it to subscriber callbacks on its own thread during the
  next spin cycle. This is the substrate for two executors in one process
  talking to each other. **Caveat that shapes the work below:** this function
  today *deep-copies the topic, type name, and payload bytes* before
  returning (`subprojects/PCL/include/pcl/pcl_executor.h`, lines 127-144), so
  a native Tier-B path needs a variant that copies or moves the **native
  struct** rather than re-serializing it — the copy is unavoidable across the
  thread boundary, but the wire encode/decode is not.
- **The message envelope already allows a raw struct.** `pcl_msg_t`
  (`subprojects/PCL/include/pcl/pcl_types.h`) is `{ const void* data;
  uint32_t size; const char* type_name; }`. The `data` field's own comment
  reads "serialized (or raw struct) payload" and `type_name` is a free-form
  string. Nothing in PCL inspects the bytes; a publisher may legitimately set
  `data` to the address of a live C++ object and `type_name` to a chosen
  native tag. On Tier A the executor hands that pointer to each local
  subscriber unchanged; on Tier B the object must instead be owned by / copied
  into the ingress queue because the publisher's stack object will be gone by
  the time the receiving executor drains the queue.

So the substrate is present for both tiers. What is missing is a **generated
binding** that uses it.

### Claim 2 — the port-style bindings do not use it; they always serialize

Confirmed. The generated topic and service facades encode on send and decode
on receive **unconditionally**, independent of the route:

- **Send always encodes.** The generated `publish<Topic>(payload,
  content_type)` helper builds a serialized buffer first
  (`subprojects/PYRAMID/pim/cpp/service_impl_gen.py`, around lines 673-698):
  it calls `encode<Topic>(payload, content_type, &wire_payload)` — which
  runs `pyramid_try_registry_encode(...)` against the JSON / FlatBuffers /
  Protobuf codec — and only then fills a `pcl_msg_t` whose `data` points at
  the encoded `wire_payload` string. The component facade's
  `publish<Topic>(...)` wrapper
  (`subprojects/PYRAMID/pim/cpp/components_gen.py`, around lines 655-664)
  passes `content_type_` straight through, so there is no branch that could
  skip the encode.
- **Receive always decodes.** The generated subscriber trampoline
  (`components_gen.py`, around lines 836-847) calls `decode<Topic>(msg,
  &payload)` before invoking the typed callback. The service-request and
  RPC-response trampolines decode the same way.
- **"Local" routing does not help.** `configureTransport({"transport":
  "local"})` / `configurePubSubTransport(...)` and the `routeAll*Local()`
  helpers (`components_gen.py` around lines 666-707) only choose **not to use
  a transport adapter**. They select the PCL direct-dispatch path — but the
  buffer that direct dispatch hands over has *already been serialized* by the
  encode step above, and the receiver still decodes it. So the PCL "zero-copy
  pointer handoff" here is zero-copy of the **encoded buffer**, not of the
  **native object**. Encode plus decode still runs on every local message.

The `.ports` deployment file reflects the same limitation. Its per-port
`MODE` column is `rpc` or `pubsub`
([`component_skeletons.md`](../../../subprojects/PYRAMID/doc/architecture/component_skeletons.md)) —
these choose the interaction realization, not the payload path. There is no
value that selects a native, non-serializing in-process handoff.

## Why this matters

- **It undercuts PCL's P5 design principle for exactly the components that
  should benefit.** Two typed PYRAMID components co-located in one process pay
  full encode + decode cost, even though PCL is built to avoid it.
- **It is on the hot path.** High-rate intra-process data (sensor products,
  tactical-object updates) is the case where FlatBuffers was adopted to keep
  the serialized form small. Skipping serialization altogether removes the
  CPU and the per-message heap allocation, not just shrinks the buffer.
- **Co-location is a first-class deployment.** The skeleton model is
  one-component-per-process today, but single-process multi-component
  executors are explicitly supported (`routeAllLocal`, the in-process
  showcase, and AME's combined ROS2 executor all rely on it) — both the
  same-executor (Tier A) and sibling-executor (Tier B) shapes. Those
  deployments get none of the efficiency PCL can offer through the typed
  bindings.

## Design constraints a native path must respect

A native handoff is only safe under conditions the serialized path does not
have to worry about. Any solution has to enforce these, not assume them. Note
that constraint 2 (lifetime) differs by tier — this is where the same-executor
and cross-executor cases must not be conflated:

1. **Same process, same compiled types.** Publisher and subscriber must share
   the identical struct layout (same generated headers, same build /
   ABI). Cross-process and cross-language (C++ ↔ Ada) handoff cannot use a
   raw pointer; those legs must fall back to serialization.
2. **Lifetime — different rule per tier.**
   - *Tier A (same executor):* `pcl_msg_t` pointers are borrowed for the
     duration of the synchronous dispatch call (see the type's own doc
     comment). The native object outlives the call and the subscriber must not
     retain the pointer past its callback. A borrow is enough — **with one
     exception, the unary service response** (next point).
   - *Tier B (different executor):* dispatch is asynchronous — the publisher
     runs on past the publish call, so a borrow of its stack object is a
     use-after-free by the time the receiving executor drains its queue. Tier
     B must **transfer ownership** into the queue: move the native object in,
     or hold it by shared ownership (for example a reference-counted handle)
     that keeps it alive until every subscriber on that executor has run.
   - *Unary service response, even on Tier A:* the response is **not** consumed
     inside the handler's stack frame. PCL calls the service handler, and only
     after it returns does it invoke the client response callback
     (`pcl_executor_invoke_async` fills a local `response`, the handler returns
     `PCL_OK`, then `callback(&response, ...)` runs —
     `subprojects/PCL/src/pcl_executor.c`). A native response whose `data`
     points at an object local to the generated dispatch would dangle by the
     time that callback casts it. The native path therefore needs an **owned
     response slot** whose lifetime spans past the handler return until the
     response callback runs — the analogue of the serialized path's per-channel
     `response_storage`. The same applies to a deferred (`PCL_PENDING`) reply
     completed later via `pcl_service_respond`.
3. **Read-only sharing.** One publish fans out to several subscribers sharing
   one `const` object. The receiver must treat it as immutable; a subscriber
   that needs to mutate or keep the value must copy it itself. Under Tier B's
   shared-ownership option this also means no in-place mutation of a value
   others still hold.
4. **Fail closed if it could reach the wire — and early enough.** A native
   pointer must never be handed to a transport adapter — that would ship a raw
   address off-box. The runtime must reject a native-tagged message on any
   non-local route rather than sending garbage. Crucially, the rejection must
   sit **before any local dispatch**, not at the transport hook: for a mixed
   `PCL_ROUTE_LOCAL|PCL_ROUTE_REMOTE` route, `pcl_executor_publish_port`
   delivers the local leg *first* and only then invokes remote transports
   (`subprojects/PCL/src/pcl_executor.c`). A guard that fires at the transport
   hook runs after the native local leg has already produced side effects, so a
   native publish on a mixed route must be refused up front (or the N6
   split/compose-time policy must already be in force).
5. **Mixed local + remote fan-out.** If one topic has both an in-process
   native subscriber and a remote subscriber, the message cannot be
   *only* a native pointer. Either the publish encodes for the remote leg and
   hands the native object to the local leg, or such a mix is refused at
   compose time. This has to be decided, not left implicit.
6. **Durability / late join.** Pub/sub durability (retained last value for a
   late-joining subscriber) needs a stored copy. A borrowed native pointer
   cannot back a retained snapshot, so durable topics either keep an encoded
   retained value or an owned copy.
7. **Default output must not change.** The generated serialized path is the
   default and stays byte-for-byte identical (the standing regression bar in
   [`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md)); the native path
   is strictly additive and opt-in.

## TODO

Suggested order. Each item keeps the serialized path as the default and
adds the native path behind an explicit opt-in.

### N1. Decide and document the native payload contract — **S**

- Define a native content-type sentinel, e.g. `application/x-pcl-native`
  carrying the schema short-name, distinct from any wire content type.
- Write the same-ABI, read-only, and **per-tier lifetime** rules from the
  section above as the normative contract for the native path — explicitly
  the Tier-A borrow versus the Tier-B ownership transfer.
- Decide the Tier-B ownership mechanism (move into the ingress queue vs. a
  reference-counted handle) so N3b/N4/N5 build against one model.
- **Accept:** a short spec section (add to
  [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
  or a new architecture page) that N2–N7 implement against.

### N2. Runtime fail-closed guard in PCL — **S/M**

- Ensure the executor/transport layer rejects a native-tagged `pcl_msg_t` on
  any route that would reach a transport adapter, returning a clear error
  rather than transmitting a pointer. Reuse or extend the transport
  capability model (`pcl_capabilities.h`, `PCL_CAP_*`).
- The guard must cover **every** path that can hand a `pcl_msg_t` to a
  transport hook, not just publish: pub/sub publish, service **request**
  (`pcl_executor_invoke_async` / `pcl_executor_invoke_stream`,
  `pcl_transport.h`), service **response** (`pcl_service_respond`), and
  **stream frame** (`pcl_stream_send`, `pcl_container.h`). A native-tagged
  request, response, or stream frame on a remote RPC route can otherwise leak
  a raw pointer onto the wire.
- **Accept:** fail-closed tests exercise a native-tagged message on a remote
  route for each of the four paths above (publish, request, response, stream
  frame); the same messages on a local route still deliver.

### N2b. Inbound provenance — a native sentinel a trampoline sees must be locally originated — **S/M**

This is a PCL-side prerequisite for N3a/N4, not a generator task, and the
plan cannot leave the same-executor guard to the generator alone. The
subscriber callback ABI does not carry provenance: `dispatch_incoming_now()`
knows `source_route_mode` / `source_peer_id` and filters on them
(`subprojects/PCL/src/pcl_executor.c`), but `pcl_sub_callback_t`
(`subprojects/PCL/include/pcl/pcl_container.h`) receives only
`(container, msg, user_data)`. A generated trampoline therefore cannot tell a
same-executor borrowed object from a remote frame that arrived through
`pcl_executor_post_remote_incoming()` and merely carries the native sentinel
in `msg->type_name`; casting on the sentinel alone would reinterpret remote
bytes as a live C++ object.

- The same hole exists on the **service** paths, not just pub/sub. N4 enables
  native request / response / stream-frame trampolines, and their generated
  callbacks also see only a `pcl_msg_t`.
- **State the guard as a property over three provenance states, not two.** The
  native sentinel alone is not enough to decide safety, because there are
  *three* origins, not just "same executor" versus "everything else":
  1. **same-executor borrowed** (Tier A) — safe to cast;
  2. **trusted in-process owned-native from a sibling executor** (Tier B, once
     N3b exists) — also safe to cast, and it deliberately crosses executors, so
     a blanket "reject anything not same-executor" would reject exactly the
     case the plan is adding;
  3. **wire/transport-originated** — must be rejected; its bytes are not a live
     object.
  So the invariant is: *a native-casting trampoline only ever runs on origin 1
  or 2; origin 3 never reaches it carrying the native sentinel.* Origins 1 and
  2 need a distinct trusted-provenance marker on the in-process owned-native
  queue entry that transport ingress cannot forge.
- **State it as a property, not a list of entry points.** An enumerated list
  of ingress functions is fragile because transport-specific gateways call
  handlers directly, bypassing the generic `post_*` helpers. Two examples
  already in the tree:
  - the generic cross-thread helpers `pcl_executor_post_remote_incoming`
    (pub/sub), `pcl_executor_post_service_request_remote` (RPC request), and
    `pcl_executor_post_response_msg` (RPC response/frame)
    (`subprojects/PCL/include/pcl/pcl_executor.h`); **and**
  - the shared-memory gateway, which unpacks the frame and calls
    `service_port->svc_handler(...)` / `stream_port->stream_handler(...)`
    **directly** (`pcl_shm_gateway_svc_sub_cb` /
    `pcl_shm_gateway_stream_sub_cb`,
    `subprojects/PCL/src/pcl_transport_shared_memory.c`), never touching the
    `post_*` helpers above.
  A future transport will add more such paths, so the guard should live at a
  single choke point (or a shared provenanced/rejecting helper every transport
  ingress must route through), not a hand-maintained list.
- Pick one of two mechanisms and implement it in PCL at that choke point. Both
  must reject origin 3 while still admitting origins 1 and 2:
  - **(a) Reject on entry from any transport; trust the in-process queue.** A
    transport-originated message tagged with the native sentinel is refused at
    ingress. Same-executor dispatch (origin 1) and the Tier-B in-process
    owned-native queue (origin 2) do not pass through transport ingress, so a
    native sentinel reaching a trampoline is guaranteed to be a live object.
    Simplest; the trusted-provenance boundary is "did this come from a
    transport or from the in-process path".
  - **(b) Pass provenance to the callback.** Extend the subscriber/handler/
    response callbacks (or a side channel) with a provenance tag the trampoline
    checks before it casts — distinguishing same-executor borrowed and
    in-process owned-native (cast) from transport-originated (never cast). More
    invasive to the ABI; needed if a native sentinel must legitimately share an
    ingress path with transport traffic.
- **Accept:** a wire/transport-originated native-tagged message never reaches a
  native-casting trampoline as a live object (asan/stress proves no
  reinterpretation) — exercised for pub/sub frame, service request, unary
  response, and **server-stream response frame** (delivered via
  `pcl_executor_post_response_msg` into `StreamPushState::trampoline`, so it is
  a cast path too), and **including a shared-memory gateway RPC path** to prove
  the guard is not tied to the generic `post_*` helpers; same-executor
  (origin 1) and, once N3b lands, sibling-executor owned-native (origin 2)
  delivery both still cast and deliver.

### N3a. Generator: Tier-A native fast path for topic pub/sub (same executor) — **M**

- In `components_gen.py` / `service_impl_gen.py`, emit an opt-in native
  publish that sets `msg.data = &payload`, `msg.size = sizeof/0`, and
  `msg.type_name = <native sentinel>`, plus a subscriber trampoline that,
  on the native sentinel, casts `msg->data` back to the typed pointer and
  calls the callback with no decode.
- **Depends on N2b.** The trampoline may cast on the native sentinel only
  because N2b guarantees such a sentinel is locally originated; the guard is
  not implementable in the generator alone (the callback ABI carries no
  provenance). Where N2b is not yet in place, the native branch must stay off.
- **Accept:** default (serialized) generated output unchanged by `diff -qr`;
  the Tier-A path compiles and delivers the correct typed value with zero
  copies on one executor; N2b's ingress guard is in place.

### N3b. Tier-B native transfer across executors (same process) — **M/L**

- **First close a routing gap: PCL has no named in-process sibling executor.**
  Local dispatch only visits containers on the *calling* executor
  (`dispatch_incoming_now()` loops `e->containers`), local service lookup
  searches that same executor, and `pcl_executor_set_endpoint_route()` rejects
  a peer id unless the route includes `PCL_ROUTE_REMOTE`
  (`subprojects/PCL/include/pcl/pcl_transport.h`). So today a message to a
  second executor in the same process has nowhere to go except a
  remote-style transport/bridge, which serializes. Tier B needs a new
  **in-process sibling-executor route** (name a sibling executor as a local
  peer, or an in-process bridge that carries native objects) before there is a
  destination to send to. This is the substance of the item, not a detail.
- Add PCL cross-thread queue variants that carry a native object by the
  ownership mechanism chosen in N1 instead of deep-copying wire bytes, and
  route them over the sibling-executor path above so the native struct is
  moved/shared into the receiving executor's queue with no wire encode. This
  must cover **all** cross-thread queues PCL uses, not only pub/sub: today
  `pcl_executor_post_incoming` (pub/sub), `pcl_executor_post_service_request` /
  `_remote` (RPC request), and `pcl_executor_post_response_msg` (RPC response
  and stream frames) each deep-copy bytes
  (`subprojects/PCL/include/pcl/pcl_executor.h`). If only the pub/sub queue
  gets a native variant, the RPC half stays serialized and N4/N5's
  "same native request works for a sibling executor" cannot be met.
- **Decision to record:** either provide native-owned request/response/stream
  queue variants as above, or explicitly scope Tier-B **native services** out
  of the first cut (Tier-B native pub/sub only; sibling-executor services fall
  back to serialized) so N4/N5 do not promise something N3b does not deliver.
- **Accept:** two executors in one process can be wired as native local peers;
  a single-process test delivers the correct typed value across the two
  executor threads for every interaction the decision keeps in scope (pub/sub,
  and RPC request/response/stream if not scoped out) with no codec calls and no
  use-after-free under a stress/asan run; default output still unchanged.

### N4. Generator: native fast path for services (unary + streaming) — **M**

- Same treatment for `ProvidedService` / `ConsumedService` request,
  response, and stream-frame trampolines: skip encode/decode when both ends
  are local and native is selected. Keep stream-end and cancellation
  semantics identical to the serialized path.
- **Own the response past the handler return.** Per constraint 2, a native
  unary response cannot borrow an object local to the generated dispatch —
  PCL invokes the client callback *after* the handler returns. Give the native
  path a per-channel owned response slot (the analogue of the serialized
  `response_storage`), covering both the immediate `PCL_OK` reply and the
  deferred `PCL_PENDING` / `pcl_service_respond` reply.
- **Keep deferred local replies off the transport.** `pcl_service_respond()`
  checks `ctx->executor->transport.respond` *before* the saved local callback
  (`subprojects/PCL/src/pcl_container.c`), so on an executor that also has a
  default transport installed, a native local deferred (`PCL_PENDING`) reply
  would be handed to that transport — and then rejected by N2b's fail-closed
  guard — instead of returning to the local caller. The service context must
  carry local/native provenance and route a deferred native reply straight to
  the saved callback, bypassing the executor-default transport. Owned response
  storage alone does not fix this.
- **Bypass the bind-time codec check on native/local routes.** Generated
  `bind()` calls `supportsContentType(content_type_)` and returns
  `PCL_ERR_INVALID` when the codec registry is empty
  (`components_gen.py`, `interaction_facade_gen.py`). A native/local route uses
  no codec, so that check must be skipped or deferred for it — otherwise the
  "no codec registered, still succeeds locally" acceptance below can never
  pass even when every trampoline skips encode/decode.
- **Depends on N2b** for the same reason as N3a: the request/response/frame
  trampolines cast on the native sentinel, which is only sound once inbound
  provenance guarantees the sentinel is locally originated.
- **Same-executor (Tier A) always; sibling-executor (Tier B) only if N3b keeps
  native services in scope.** For a Tier-B peer, native services work only when
  N3b provides the native request/response/stream queue variants; if N3b scopes
  them out, a sibling-executor service call falls back to serialized.
- **Accept:** the `examples/cpp` Tactical Objects showcase runs unchanged
  under serialized routing and, under native routing, produces identical
  results with no codec calls (verify by codec-registry call count or a
  no-codec-registered run that still succeeds locally); a unary-response
  lifetime test (handler-scope object destroyed after return) passes under
  asan, proving the owned response slot.

### N5. Deployment surface — selecting the native realization — **S/M**

- Extend `configureTransport` / `configurePubSubTransport` to accept the
  native selection (for example `{"transport":"local","payload":"native"}`),
  and thread an equivalent choice through the interaction facade so
  realization-independent components can opt in.
- Native payloads are **strictly opt-in**; the default stays serialized so the
  regression baseline (constraint 7) and the new lifetime rules never apply
  without a deliberate choice. Do **not** infer native purely from "route is
  local and ABI matches" — that would silently switch existing local
  deployments from serialized buffers to borrowed/owned native objects. If a
  convenience shorthand is wanted (for example a project-wide "prefer native
  where possible" switch), it is itself an explicit opt-in flag, and any
  inference happens only underneath that flag.
- Decide whether the opt-in is a new `.ports` payload column or a field on the
  existing route JSON (for example `{"transport":"local","payload":"native"}`);
  document the decision. Default remains serialized either way.
- The component asks for `native`; the runtime picks Tier A or Tier B by
  whether the peer shares the executor. The deployment surface should not make
  the author choose the tier by hand. Note this automatic selection depends on
  N3b's sibling-executor route existing — until it does, `native` can only be
  honoured for a same-executor (Tier A) peer, and a sibling-executor peer must
  fall back to serialized rather than silently failing to route.
- **Accept:** a component can request native locally without editing business
  logic; handler signatures unchanged; the same request works whether the peer
  is same-executor or a sibling executor.

### N6. Mixed local/remote fan-out and durability policy — **M**

- Implement (or explicitly forbid at compose time) a topic that has both a
  native local subscriber and a remote subscriber: encode once for the remote
  leg, hand the native object to the local leg.
- **Order matters (see constraint 4).** Because `pcl_executor_publish_port`
  dispatches the local leg before the remote leg, a native publish on a mixed
  route cannot rely on the transport-hook guard to stop it — the local delivery
  has already happened by then. Either this compose-time policy must be in
  force before native pub/sub is enabled on a mixed route, or the native
  publish must be rejected before the local dispatch runs.
- Define how durable/retained topics back their snapshot when the live path
  is native (retain an encoded or owned copy).
- **Accept:** compose-time validation covers the mixed case per the decision;
  a **mixed-route native publish is rejected before any local delivery occurs**
  (no local side effect leaks past the guard); a durable topic with a late
  joiner still delivers the last value under native routing.

### N7. Benchmarks, correctness parity, and cross-language boundary — **M**

- Microbenchmark all three payload paths on a local publish/echo: serialized,
  Tier-A native (zero copy), and Tier-B native (one struct copy/move, no wire
  encode), reporting encode/decode elimination and allocations avoided for
  each. The point is to show Tier B still beats serialized even though it is
  not zero-copy.
- Parity test: native and serialized paths deliver equal typed values for the
  same inputs across the showcase RPCs and topics.
- Scope the cross-language (C++ ↔ Ada in one process) question: start
  C++-to-C++ only and document Ada native handoff as out of scope for the
  first cut (it needs an ABI-compatibility decision, not just plumbing),
  mirroring how F1 handled Ada remote routing as a follow-up.
- **Accept:** benchmark and parity tests wired into the standing regression
  bar; Ada scope decision recorded.

## Key files

| Area | Files |
|------|-------|
| PCL direct dispatch / envelope (Tier A) | `subprojects/PCL/include/pcl/pcl_executor.h`, `subprojects/PCL/include/pcl/pcl_types.h` (`pcl_msg_t`), `subprojects/PCL/doc/requirements/HLR.md` (PCL.022) |
| PCL cross-thread queues (Tier B) | `subprojects/PCL/include/pcl/pcl_executor.h` — pub/sub `pcl_executor_post_incoming` (lines 111-144), RPC request `pcl_executor_post_service_request` / `_remote`, RPC response/frame `pcl_executor_post_response_msg`; `subprojects/PCL/doc/requirements/HLR.md` (PCL.025 / PCL.026) |
| Subscriber callback ABI + dispatch provenance (N2b) | `subprojects/PCL/include/pcl/pcl_container.h` (`pcl_sub_callback_t`), `subprojects/PCL/src/pcl_executor.c` (`dispatch_incoming_now`, `route_accepts`, `pcl_executor_invoke_async` response order), `subprojects/PCL/include/pcl/pcl_transport.h` (`pcl_executor_post_remote_incoming`, `pcl_executor_set_endpoint_route`), `subprojects/PCL/src/pcl_transport_shared_memory.c` (`pcl_shm_gateway_svc_sub_cb` / `pcl_shm_gateway_stream_sub_cb` direct handler dispatch) |
| PCL capability/route model | `subprojects/PCL/include/pcl/pcl_capabilities.h`, `subprojects/PCL/include/pcl/pcl_transport_routing.h` |
| Native service response lifetime + deferred routing (N4) | `subprojects/PCL/src/pcl_container.c` (`pcl_service_respond` transport-before-callback order), `subprojects/PCL/src/pcl_executor.c` (`pcl_executor_invoke_async` immediate-reply order) |
| Generated topic pub/sub + routing + bind codec check | `subprojects/PYRAMID/pim/cpp/components_gen.py` (publish/subscribe helpers, `routeAll*Local`, trampolines, `supportsContentType` bind check ~L265), `subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py` |
| Generated encode/decode/publish primitives | `subprojects/PYRAMID/pim/cpp/service_impl_gen.py` (`encode*`, `decode*`, `publish*`) |
| Component-authoring surface | `subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md`, `subprojects/PYRAMID/doc/architecture/component_skeletons.md` |
| Publish dispatch ordering (local-before-remote, N6/constraint 4) | `subprojects/PCL/src/pcl_executor.c` (`pcl_executor_publish_port`, local leg lines ~1367-1371 then remote ~1373-1394) |
| Interaction realization model | `subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md`, `subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md` |
