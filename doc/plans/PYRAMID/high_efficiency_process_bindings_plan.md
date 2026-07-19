# High-Efficiency In-Process Port Bindings — Gap Analysis & TODO

**Status: proposed (2026-07-19). Not yet scheduled.**

## What this is about

When two PYRAMID components run in the **same operating-system process** on
the same PCL executor, they can in principle exchange a message by passing a
pointer to the native typed object (the generated C++ struct) straight from
the publisher to the subscriber. No serialization, no byte buffer, no
allocation on the hot path. This document calls that path **high-efficiency
in-process binding**: the sender hands over the live object and the receiver
reads it in place.

PCL already supports this at the runtime level. The generated **port-style
bindings** (the typed publish/subscribe and service facades described in
[`cpp_component_authoring.md`](../../../subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md)
and [`component_skeletons.md`](../../../subprojects/PYRAMID/doc/architecture/component_skeletons.md))
do not. Even when a port is routed entirely in-process, the generated code
encodes the typed value into a serialized buffer on the way out and decodes
it again on the way in. This document explains the gap precisely and lists
the work needed to close it.

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

- **Direct dispatch.** With no transport adapter set, the executor routes a
  published message straight to matching subscribers by pointer, on the
  single executor thread. This is requirement **PCL.022 — Intra-Process
  Direct Dispatch** ("zero-copy pointer handoff",
  [`subprojects/PCL/doc/requirements/HLR.md`](../../../subprojects/PCL/doc/requirements/HLR.md)),
  and design principle **P5 — Zero-copy where possible**
  ([`component_container_design.md`](../../../subprojects/PCL/doc/architecture/component_container_design.md)).
- **The message envelope already allows a raw struct.** `pcl_msg_t`
  (`subprojects/PCL/include/pcl/pcl_types.h`) is `{ const void* data;
  uint32_t size; const char* type_name; }`. The `data` field's own comment
  reads "serialized (or raw struct) payload" and `type_name` is a free-form
  string. Nothing in PCL inspects the bytes; a publisher may legitimately set
  `data` to the address of a live C++ object and `type_name` to a chosen
  native tag, and the executor will hand that pointer to each local
  subscriber unchanged.

So the substrate is present. What is missing is a **generated binding** that
uses it.

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
  showcase, and AME's combined ROS2 executor all rely on it). Those
  deployments get none of the efficiency PCL can offer through the typed
  bindings.

## Design constraints a native path must respect

A native pointer handoff is only safe under conditions the serialized path
does not have to worry about. Any solution has to enforce these, not assume
them:

1. **Same process, same compiled types.** Publisher and subscriber must share
   the identical struct layout (same generated headers, same build /
   ABI). Cross-process and cross-language (C++ ↔ Ada) handoff cannot use a
   raw pointer; those legs must fall back to serialization.
2. **Borrow semantics / lifetime.** `pcl_msg_t` pointers are borrowed for the
   duration of the call (see the type's own doc comment). The native object
   must outlive the synchronous dispatch and the subscriber must not retain
   the pointer past its callback. This matches the existing single-threaded
   executor contract but must be stated and enforced.
3. **Read-only sharing.** One publish fans out to several subscribers sharing
   one `const` pointer. The receiver must treat the object as immutable; a
   subscriber that needs to mutate or keep the value must copy it itself.
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
- Write the borrow/lifetime, read-only, and same-ABI rules from the section
  above as the normative contract for the native path.
- **Accept:** a short spec section (add to
  [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
  or a new architecture page) that N2–N7 implement against.

### N2. Runtime fail-closed guard in PCL — **S/M**

- Ensure the executor/transport layer rejects a native-tagged `pcl_msg_t` on
  any route that would reach a transport adapter, returning a clear error
  rather than transmitting a pointer. Reuse or extend the transport
  capability model (`pcl_capabilities.h`, `PCL_CAP_*`).
- **Accept:** a test that publishing a native-tagged message on a remote
  route fails closed; local route still delivers.

### N3. Generator: native fast path for topic pub/sub — **M**

- In `components_gen.py` / `service_impl_gen.py`, emit an opt-in native
  publish that sets `msg.data = &payload`, `msg.size = sizeof/0`, and
  `msg.type_name = <native sentinel>`, plus a subscriber trampoline that,
  on the native sentinel, casts `msg->data` back to the typed pointer and
  calls the callback with no decode. Guard both so the native branch is taken
  only on a proven-local route; otherwise the existing encode/decode runs.
- **Accept:** default (serialized) generated output unchanged by `diff -qr`;
  the native path compiles and delivers the correct typed value in-process.

### N4. Generator: native fast path for services (unary + streaming) — **M**

- Same treatment for `ProvidedService` / `ConsumedService` request,
  response, and stream-frame trampolines: skip encode/decode when both ends
  are local and native is selected. Keep stream-end and cancellation
  semantics identical to the serialized path.
- **Accept:** the `examples/cpp` Tactical Objects showcase runs unchanged
  under serialized routing and, under native routing, produces identical
  results with no codec calls (verify by codec-registry call count or a
  no-codec-registered run that still succeeds locally).

### N5. Deployment surface — selecting the native realization — **S/M**

- Extend `configureTransport` / `configurePubSubTransport` to accept the
  native selection (for example `{"transport":"local","payload":"native"}`),
  and thread an equivalent choice through the interaction facade so
  realization-independent components can opt in.
- Decide whether the `.ports` file gains a payload column or whether native
  is inferred whenever a route is local and both ends share the same compiled
  ABI; document the decision. Default remains serialized.
- **Accept:** a component can request native locally without editing business
  logic; handler signatures unchanged.

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

- Microbenchmark: native vs. serialized local publish/echo, reporting
  encode/decode elimination and allocations avoided.
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
| PCL direct dispatch / envelope | `subprojects/PCL/include/pcl/pcl_executor.h`, `subprojects/PCL/include/pcl/pcl_types.h` (`pcl_msg_t`), `subprojects/PCL/doc/requirements/HLR.md` (PCL.022) |
| PCL capability/route model | `subprojects/PCL/include/pcl/pcl_capabilities.h`, `subprojects/PCL/include/pcl/pcl_transport_routing.h` |
| Generated topic pub/sub + routing | `subprojects/PYRAMID/pim/cpp/components_gen.py` (publish/subscribe helpers, `routeAll*Local`, trampolines) |
| Generated encode/decode/publish primitives | `subprojects/PYRAMID/pim/cpp/service_impl_gen.py` (`encode*`, `decode*`, `publish*`) |
| Component-authoring surface | `subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md`, `subprojects/PYRAMID/doc/architecture/component_skeletons.md` |
| Interaction realization model | `subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md`, `subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md` |
