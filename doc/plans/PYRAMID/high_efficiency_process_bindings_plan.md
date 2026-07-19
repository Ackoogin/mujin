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
  reinterpretation) — exercised for pub/sub frame, service request, and
  response, and **including a shared-memory gateway RPC path** to prove the
  guard is not tied to the generic `post_*` helpers; same-executor (origin 1)
  and, once N3b lands, sibling-executor owned-native (origin 2) delivery both
  still cast and deliver.

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
| PCL cross-thread queues (Tier B) | `subprojects/PCL/include/pcl/pcl_executor.h` — pub/sub `pcl_executor_post_incoming` (lines 111-144), RPC request `pcl_executor_post_service_request` / `_remote`, RPC response/frame `pcl_executor_post_response_msg`; `subprojects/PCL/doc/requirements/HLR.md` (PCL.025 / PCL.026) |
| Subscriber callback ABI + dispatch provenance (N2b) | `subprojects/PCL/include/pcl/pcl_container.h` (`pcl_sub_callback_t`), `subprojects/PCL/src/pcl_executor.c` (`dispatch_incoming_now`, `route_accepts`, `pcl_executor_invoke_async` response order), `subprojects/PCL/include/pcl/pcl_transport.h` (`pcl_executor_post_remote_incoming`, `pcl_executor_set_endpoint_route`), `subprojects/PCL/src/pcl_transport_shared_memory.c` (`pcl_shm_gateway_svc_sub_cb` / `pcl_shm_gateway_stream_sub_cb` direct handler dispatch) |
| PCL capability/route model | `subprojects/PCL/include/pcl/pcl_capabilities.h`, `subprojects/PCL/include/pcl/pcl_transport_routing.h` |
| Native service response lifetime + deferred routing (N4) | `subprojects/PCL/src/pcl_container.c` (`pcl_service_respond` transport-before-callback order), `subprojects/PCL/src/pcl_executor.c` (`pcl_executor_invoke_async` immediate-reply order) |
| Generated topic pub/sub + routing + bind codec check | `subprojects/PYRAMID/pim/cpp/components_gen.py` (publish/subscribe helpers, `routeAll*Local`, trampolines, `supportsContentType` bind check ~L265), `subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py` |
| Generated encode/decode/publish primitives | `subprojects/PYRAMID/pim/cpp/service_impl_gen.py` (`encode*`, `decode*`, `publish*`) |
| Component-authoring surface | `subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md`, `subprojects/PYRAMID/doc/architecture/component_skeletons.md` |
| Interaction realization model | `subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md`, `subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md` |
