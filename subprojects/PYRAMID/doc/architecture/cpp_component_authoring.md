# Authoring C++ Components Against Generated PYRAMID Bindings

This guide is the reference for writing a C++ component on top of the PYRAMID
generated service bindings. It is the companion to
[`generated_bindings.md`](generated_bindings.md) and uses the
**service-binding facade** emitted alongside the low-level binding
(`*_components.hpp`).

If you only need the low-level invoke/dispatch/encode/decode primitives — for
custom transports, codec dispatch tests, or framework code — read the binding
guide instead. This page is for application authors writing components.

## What the generator emits

For each `pyramid.components.<name>.services.{provided,consumed}` proto, the
generator produces two layers under `${binaryDir}/generated/pyramid_cpp_bindings/`:

| Artifact | Surface | Use |
|----------|---------|-----|
| `<prefix>.{hpp,cpp}` | `ServiceHandler`, `invoke*`, `dispatch`, `encode*`, `decode*` | Low-level. Transports, codec round-trip tests. |
| `<prefix>_components.hpp` | `ProvidedHandler`, `ProvidedService`, `ConsumedService`, `Result<T>`, `StreamHandle` | Service bindings you attach to your own `pcl::Component`. |

`ProvidedService` and `ConsumedService` are **bindings**, not components. A
PYRAMID component is a deployable unit that may host several services. You
write a `pcl::Component` subclass and compose one binding per proto as a
member; that lets a single deployable component own (for example) the
provided side of Tactical Objects and the consumed side of Sensor Data
Interpretation in the same class.

## The pieces you write against

### `ProvidedHandler` — typed callbacks

Subclass and override `on<Op>` methods. There is one method per RPC, with
typed request and reply.

- Unary RPC `Op(Req) returns (Reply)` → `Reply onOp(const Req&)`.
- Server-streaming RPC `Op(Req) returns (stream Item)` →
  `void onOp(const Req&, StreamWriter<Item> writer)`.

For streaming RPCs the handler is handed a typed `StreamWriter<Item>` it can
keep and pump frames into over time:

- `writer.send(frame)` — emit one typed frame on the wire.
- `writer.end(status = PCL_OK)` — close (or abort) the stream. Idempotent.
- `writer.cancelled()` — true if the client has asked to cancel; servers
  emitting long streams should poll this and stop.
- Dropping the writer without calling `end()` aborts the stream with
  `PCL_ERR_STATE` (the destructor handles this for you).

The writer is move-only; capture it into a per-stream state if you need to
emit across tick boundaries.

```cpp
class InterestStore final : public svc::ProvidedHandler {
  Identifier onObjectOfInterestCreateRequirement(
      const ObjectInterestRequirement& r) override;

  void onObjectOfInterestReadRequirement(             // streaming RPC
      const Query& q,
      svc::StreamWriter<ObjectInterestRequirement> writer) override {
    for (auto& it : results_for(q)) {
      if (writer.cancelled()) break;
      writer.send(it);
    }
    writer.end();
  }

  Ack onObjectOfInterestDeleteRequirement(const Identifier&) override;
};
```

### `ProvidedService` — attach to your component

`ProvidedService` is a binding object. You declare it as a member of your
own `pcl::Component`, hand it the host component plus the executor plus the
handler at construction, then call `bind()` from `on_configure()` to install
the RPC ports on the host.

```cpp
class TacticalObjectsComponent : public pcl::Component {
public:
  TacticalObjectsComponent(pcl::Executor& exec, std::string content_type)
      : pcl::Component("tactical_objects"),
        tobj_provided_(*this, exec, store_, std::move(content_type)) {}

  pcl_status_t routeProvidedTo(std::string_view peer) {
    return tobj_provided_.routeAllRemote(peer);
  }

protected:
  pcl_status_t on_configure() override { return tobj_provided_.bind(); }

private:
  InterestStore store_;
  svc::ProvidedService tobj_provided_;
};
```

Two constructors are emitted, matching whichever ownership you prefer:

- `(pcl::Component&, pcl::Executor&, ProvidedHandler&, std::string content_type)` —
  caller owns the handler.
- `(pcl::Component&, pcl::Executor&, std::unique_ptr<ProvidedHandler>, std::string content_type)` —
  the binding takes ownership.

`bind()`:

- Validates the content type.
- Installs each RPC port (unary `addService` / streaming `addStreamService`)
  on the host component.
- Stores per-channel response-buffer state so dispatch is reentrant.

`routeAllRemote(peer)` restricts every advertised RPC to a single peer.
Stream-end handling is internal — you never call `pcl_stream_end` directly,
just `writer.end()` from your handler.

Composing several `ProvidedService` (or mixing provided and consumed) inside
one component is just adding more members and another `binding.bind()` call
in `on_configure`.

### `ConsumedService` — async-shaped typed client

`ConsumedService` is the client-side binding. Like `ProvidedService` it
attaches to your component and takes the executor; routing happens through
the executor's transport.

```cpp
class HmiClientComponent : public pcl::Component {
public:
  HmiClientComponent(pcl::Executor& exec, std::string content_type)
      : pcl::Component("hmi_client"),
        tobj_consumed_(*this, exec, std::move(content_type)) {}

  pcl_status_t routeProvidedDefault() {
    return tobj_consumed_.routeAllRemote();      // executor's default transport
  }
  pcl_status_t routeProvidedTo(std::string_view peer) {
    return tobj_consumed_.routeAllRemote(peer);  // named peer
  }

  std::future<svc::Result<Identifier>>
  createRequirementAsync(const ObjectInterestRequirement& req) {
    return tobj_consumed_.objectOfInterestCreateRequirementAsync(req);
  }

  svc::StreamHandle
  streamReadRequirement(
      const Query& q,
      std::function<void(const ObjectInterestRequirement&)> on_frame,
      std::function<void(pcl_status_t)> on_end = {}) {
    return tobj_consumed_.objectOfInterestReadRequirementStreaming(
        q, std::move(on_frame), std::move(on_end));
  }

private:
  svc::ConsumedService tobj_consumed_;
};
```

The per-RPC API:

- **Unary RPC** → `<op>Async(req)` returns `std::future<Result<T>>`. The
  future resolves on the executor thread once the response trampoline runs;
  callers spin the executor (typically by ticking it from the main loop)
  until the future is ready.
- **Streaming RPC** → `<op>Streaming(req, on_frame, on_end)` returns a
  `StreamHandle`. Both callbacks fire on the executor thread and together
  cover the stream lifetime; there is no future-returning collected variant.
  Call `handle.cancel()` to stop receiving frames — subsequent on_frame
  callbacks are suppressed, the underlying stream context is cancelled, and
  `on_end` fires with `PCL_ERR_CANCELLED`.

`Result<T>` is a generated struct: `.status`, `.value`, `.ok()`. There is no
synchronous `Result<T> create(...)` form — every unary call is async-shaped
to make executor-thread coupling explicit.

## Single-threaded executor model

The recommended driving model is single-threaded: the main thread owns the
executor and drives it with `executor.spinOnce(timeout_ms)` in the
application's loop. All `pcl::Component` callbacks (lifecycle, subscribers,
service handlers) and all binding callbacks (unary trampolines, streaming
`on_frame` / `on_end`) run on that thread. Component-local state therefore
does not need atomics or other synchronization.

`pcl::SpinThread` exists for cases where you want a background drive thread
instead, and `pcl::await` / `pcl::awaitValue` block a non-executor thread on
a future; both belong to the multi-threaded pattern and are not needed for
the single-threaded showcase.

## PCL helpers used to wire it all together

```cpp
#include <pcl/shared_memory_participant.hpp>   // bus, executor, gateway
#include <pcl/spin_thread.hpp>                 // (optional) background spin
#include <pcl/await.hpp>                       // (optional) future + spin
```

| Helper | Replaces |
|--------|----------|
| `pcl::SharedMemoryParticipant{bus, id, with_gateway}` | `pcl_shared_memory_transport_create` + `executor.setTransport` + gateway configure/activate/add boilerplate |
| `pcl::SpinThread{executor}` | `std::thread` + atomic stop-flag + `spinOnce(10) + yield` loop |
| `pcl::await(executor, future, timeout)` / `pcl::awaitValue(...)` | Hand-rolled spin-and-wait on `std::atomic<bool>` |

## Full showcase

The canonical showcase is in `subprojects/PYRAMID/examples/cpp/`:

- `tobj_interest_store.{hpp,cpp}` — typed business logic
  (`ProvidedHandler` subclass).
- `tactical_objects_component.hpp` — hand-written `pcl::Component` that
  composes one `ProvidedService` binding plus the store.
- `hmi_client_component.hpp` — hand-written `pcl::Component` that composes
  one `ConsumedService` binding and exposes typed async accessors.
- `tobj_shared_memory_example.cpp` — bus bring-up, component wiring,
  single-threaded `spinOnce` loop, demonstration sequence.

The showcase exercises Create / Read (streaming, server-ended) / Read
(streaming, client-cancelled) / Delete with no `pcl_msg_t`, no codec
branching, no manual stream plumbing, no background thread, and no
`pcl::await`.

## When to drop down to the low-level surface

The component facade calls into the low-level `invoke*` / `dispatch` /
`encode*` / `decode*` helpers in `<prefix>.hpp`. Use those directly when:

- You need full control of `pcl_endpoint_route_t` per call.
- You're writing a transport adapter or codec round-trip test.
- You need to plug the binding into a custom executor lifecycle (e.g. another
  framework's event loop).

For everything else, prefer the binding facade — it owns the boilerplate.

## See also

- [`generated_bindings.md`](generated_bindings.md) — binding architecture overview.
- [`pcl_pyramid_binding_generation_overview.md`](pcl_pyramid_binding_generation_overview.md) — short engineer-facing description of the binding layer.
- [`PYRAMID_COMPONENT_RESPONSIBILITIES.md`](PYRAMID_COMPONENT_RESPONSIBILITIES.md) — canonical component responsibilities from the technical standard.
