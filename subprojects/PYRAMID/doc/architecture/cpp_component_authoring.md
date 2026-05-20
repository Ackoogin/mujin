# Authoring C++ Components Against Generated PYRAMID Bindings

This guide is the v1 reference for writing a C++ component on top of the
PYRAMID generated service bindings. It is the companion to
[`generated_bindings.md`](generated_bindings.md) and uses the **component-shaped
facade** emitted alongside the low-level binding (`*_components.hpp`).

If you only need the low-level invoke/dispatch/encode/decode primitives — for
custom transports, codec dispatch tests, or framework code — read the v1
binding guide instead. This page is for application authors writing services
and clients.

## What the generator emits

For each `pyramid.components.<name>.services.{provided,consumed}` proto, the
generator produces two layers under `${binaryDir}/generated/pyramid_cpp_bindings/`:

| Artifact | Surface | Use |
|----------|---------|-----|
| `<prefix>.{hpp,cpp}` | `ServiceHandler`, `invoke*`, `dispatch`, `encode*`, `decode*` | Low-level. Transports, codec round-trip tests. |
| `<prefix>_components.hpp` | `ProvidedHandler`, `ProvidedComponent`, `ConsumedComponent`, `Result<T>` | Component code. Header-only; layers on top of the low-level surface. |

The component-shaped facade is the v1 application authoring surface. Component
code should not have to type `pcl_msg_t`, `pcl_stream_context_t`, or any
codec name.

## The three classes you write against

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

### `ProvidedComponent` — one PCL component, every service

`ProvidedComponent` is a `pcl::Component` that hosts every advertised service
in the package. Pass it your executor and your handler.

```cpp
InterestStore store;
svc::ProvidedComponent provider{exec, store, content_type};
provider.start();                       // configure + activate + add
provider.routeAllRemote("client");      // restrict callers (optional)
```

Two constructors are emitted, matching whichever ownership you prefer:

- `(pcl::Executor&, ProvidedHandler&, std::string content_type)` — caller owns
  the handler.
- `(pcl::Executor&, std::unique_ptr<ProvidedHandler>, std::string content_type)`
  — the component takes ownership.

The component:

- Registers every RPC port on `start()`.
- Routes each port to the supplied peer (or to anyone, if you don't call
  `routeAllRemote`).
- Owns per-channel response-buffer storage so dispatch is reentrant.
- Drains its deferred-stream-end queue from `on_tick` — you never call
  `pcl_stream_end` directly.

### `ConsumedComponent` — async-shaped typed client

Every RPC has typed async entry points. Unary returns
`std::future<Result<Reply>>`; streaming offers two flavours.

```cpp
svc::ConsumedComponent consumer{exec, content_type};
consumer.start();
consumer.routeAllRemote();              // use the executor's default transport
// or: consumer.routeAllRemote("server");   // use a named peer transport

// Unary RPC -- async future
std::future<Result<Identifier>> f = consumer.objectOfInterestCreateRequirementAsync(req);

// Streaming RPC -- collected vector via async future
std::future<Result<std::vector<ObjectInterestRequirement>>>
    fr = consumer.objectOfInterestReadRequirementAsync(query);

// Streaming RPC -- per-frame push callbacks
consumer.objectOfInterestReadRequirementStreaming(
    query,
    [](const ObjectInterestRequirement& frame) { ... },
    [](pcl_status_t status)                    { ... });    // on_end (optional)
```

`Result<T>` is a generated struct: `.status`, `.value`, `.ok()`. The future
resolves on the executor thread that processes the response — drive that
executor yourself (`spinOnce` / `pcl::SpinThread`) and block on the future
with `pcl::await` / `pcl::awaitValue`.

There is no synchronous `Result<T> create(...)` form. Every call is
async-shaped to make the cost of executor-thread coupling explicit.

## PCL helpers used to wire it all together

Three small header-only helpers in PCL remove the recurring shared-memory
plumbing patterns:

```cpp
#include <pcl/shared_memory_participant.hpp>   // bus, executor, gateway
#include <pcl/spin_thread.hpp>                 // background spin loop
#include <pcl/await.hpp>                       // block on future + spin
```

| Helper | Replaces |
|--------|----------|
| `pcl::SharedMemoryParticipant{bus, id, with_gateway}` | `pcl_shared_memory_transport_create` + `executor.setTransport` + gateway configure/activate/add boilerplate |
| `pcl::SpinThread{executor}` | `std::thread` + atomic stop-flag + `spinOnce(10) + yield` loop |
| `pcl::await(executor, future, timeout)` / `pcl::awaitValue(...)` | Hand-rolled spin-and-wait on `std::atomic<bool>` |

## Full showcase

The canonical showcase is `subprojects/PYRAMID/examples/cpp/tobj_shared_memory_example.cpp`.
It brings up a two-participant shared-memory bus, advertises the Tactical
Objects services on one side, drives the consumer side from `main()` via
`pcl::await`, and exercises the full Create / Read (streaming) / Delete cycle.
The user code is one file with no `pcl_msg_t`, no codec branching, and no
manual streaming plumbing.

## When to drop down to the low-level surface

The component facade calls into the low-level `invoke*` / `dispatch` /
`encode*` / `decode*` helpers in `<prefix>.hpp`. Use those directly when:

- You need full control of `pcl_endpoint_route_t` per call.
- You're writing a transport adapter or codec round-trip test.
- You need to plug the binding into a custom executor lifecycle (e.g. another
  framework's event loop).

For everything else, prefer the component facade — it owns the boilerplate.

## See also

- [`generated_bindings.md`](generated_bindings.md) — v1 binding architecture overview.
- [`pcl_pyramid_binding_generation_overview.md`](pcl_pyramid_binding_generation_overview.md) — short engineer-facing description of the binding layer.
- [`PYRAMID_COMPONENT_RESPONSIBILITIES.md`](PYRAMID_COMPONENT_RESPONSIBILITIES.md) — canonical component responsibilities from the technical standard.
