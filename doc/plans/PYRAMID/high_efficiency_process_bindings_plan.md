# High-Efficiency In-Process Port Bindings — Gap Analysis & TODO

**Status: proposed (2026-07-19). Not yet scheduled.**

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
([`standard_alignment.md`](../../../subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md))
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
  ([`component_container_design.md`](../../../subprojects/PCL/doc/architecture/component_container_design.md)).
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
     retain the pointer past its callback. A borrow is enough.
   - *Tier B (different executor):* dispatch is asynchronous — the publisher
     runs on past the publish call, so a borrow of its stack object is a
     use-after-free by the time the receiving executor drains its queue. Tier
     B must **transfer ownership** into the queue: move the native object in,
     or hold it by shared ownership (for example a reference-counted handle)
     that keeps it alive until every subscriber on that executor has run.
3. **Read-only sharing.** One publish fans out to several subscribers sharing
   one `const` object. The receiver must treat it as immutable; a subscriber
   that needs to mutate or keep the value must copy it itself. Under Tier B's
   shared-ownership option this also means no in-place mutation of a value
   others still hold.
4. **Fail closed if it could reach the wire.** A native pointer must never be
   handed to a transport adapter — that would ship a raw address off-box. The
   runtime must reject a native-tagged message on any non-local route rather
   than sending garbage.
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

- Pick one of two mechanisms and implement it in PCL:
  - **(a) Reject at ingress.** `pcl_executor_post_remote_incoming()` (and any
    transport ingress) refuses a message tagged with the native sentinel, so
    any native sentinel that reaches a subscriber is guaranteed local origin.
    Simplest; makes the trampoline cast sound by construction.
  - **(b) Pass provenance to the callback.** Extend the subscriber callback (or
    a side channel) so the trampoline can check "same-executor, borrowed"
    before it casts. More invasive to the ABI; only needed if a native
    sentinel must legitimately travel over ingress (it should not for Tier A).
- **Accept:** a remote/native-tagged frame delivered via
  `pcl_executor_post_remote_incoming()` never reaches a native-casting
  trampoline as a live object (asan/stress proves no reinterpretation);
  same-executor native delivery is unaffected.

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
- Add a PCL cross-thread ingress variant that carries a native object by the
  ownership mechanism chosen in N1 instead of deep-copying wire bytes
  (`pcl_executor_post_incoming` deep-copies today — see Claim 1), and route it
  over the sibling-executor path above so the native struct is moved/shared
  into the receiving executor's queue with no wire encode.
- **Accept:** two executors in one process can be wired as native local peers;
  a single-process test delivers the correct typed value across the two
  executor threads with no codec calls and no use-after-free under a
  stress/asan run; default output still unchanged.

### N4. Generator: native fast path for services (unary + streaming) — **M**

- Same treatment for `ProvidedService` / `ConsumedService` request,
  response, and stream-frame trampolines: skip encode/decode when both ends
  are local and native is selected. Keep stream-end and cancellation
  semantics identical to the serialized path.
- **Depends on N2b** for the same reason as N3a: the request/response/frame
  trampolines cast on the native sentinel, which is only sound once inbound
  provenance guarantees the sentinel is locally originated.
- **Accept:** the `examples/cpp` Tactical Objects showcase runs unchanged
  under serialized routing and, under native routing, produces identical
  results with no codec calls (verify by codec-registry call count or a
  no-codec-registered run that still succeeds locally).

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
- Define how durable/retained topics back their snapshot when the live path
  is native (retain an encoded or owned copy).
- **Accept:** compose-time validation covers the mixed case per the decision;
  a durable topic with a late joiner still delivers the last value under
  native routing.

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
| PCL cross-thread ingress (Tier B) | `subprojects/PCL/include/pcl/pcl_executor.h` (`pcl_executor_post_incoming`, lines 111-144), `subprojects/PCL/doc/requirements/HLR.md` (PCL.025 / PCL.026) |
| Subscriber callback ABI + dispatch provenance (N2b) | `subprojects/PCL/include/pcl/pcl_container.h` (`pcl_sub_callback_t`), `subprojects/PCL/src/pcl_executor.c` (`dispatch_incoming_now`, `route_accepts`), `subprojects/PCL/include/pcl/pcl_transport.h` (`pcl_executor_post_remote_incoming`, `pcl_executor_set_endpoint_route`) |
| PCL capability/route model | `subprojects/PCL/include/pcl/pcl_capabilities.h`, `subprojects/PCL/include/pcl/pcl_transport_routing.h` |
| Generated topic pub/sub + routing | `subprojects/PYRAMID/pim/cpp/components_gen.py` (publish/subscribe helpers, `routeAll*Local`, trampolines) |
| Generated encode/decode/publish primitives | `subprojects/PYRAMID/pim/cpp/service_impl_gen.py` (`encode*`, `decode*`, `publish*`) |
| Component-authoring surface | `subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md`, `subprojects/PYRAMID/doc/architecture/component_skeletons.md` |
| Interaction realization model | `subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md`, `subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md` |
