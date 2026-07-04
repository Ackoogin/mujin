# PYRAMID In-Process Service/PubSub Facade TODO Report

## Purpose

Review the PYRAMID generated binding and PCL facade surfaces for ordinary
component-to-component communication when the peer is local to the process, and
record the work needed to make that path a first-class option for service and
pub/sub use.

Use this page to answer:

- what already works for same-process PCL dispatch
- where the generated facade still makes component authors drop to lower-level
  PCL calls
- which TODO items close the local-peer service/pub/sub gaps

This report is tracked from
[`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md).

## Scope

Reviewed surfaces:

- Generated C++ component facade:
  [`subprojects/PYRAMID/pim/cpp/components_gen.py`](../../../subprojects/PYRAMID/pim/cpp/components_gen.py)
- Low-level generated service/topic API:
  [`subprojects/PYRAMID/pim/cpp/service_header_gen.py`](../../../subprojects/PYRAMID/pim/cpp/service_header_gen.py) and
  [`service_impl_gen.py`](../../../subprojects/PYRAMID/pim/cpp/service_impl_gen.py)
- Component authoring guide:
  [`cpp_component_authoring.md`](../../../subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md)
- PCL component/executor model:
  [`08-pcl-component-system.md`](../../../subprojects/PCL/doc/architecture/08-pcl-component-system.md)
- Tactical Objects bridge and examples under
  [`subprojects/PYRAMID/tactical_objects/`](../../../subprojects/PYRAMID/tactical_objects/) and
  [`subprojects/PYRAMID/examples/cpp/`](../../../subprojects/PYRAMID/examples/cpp/)

## Current baseline

PCL already has the runtime primitive needed for local peers:

- With no transport adapter, publisher/subscriber traffic stays in-process and
  dispatches directly through the executor.
- `pcl_executor_invoke_service(...)` finds services through the local route.
- PCL route metadata can explicitly mark ports or endpoint routes as
  `PCL_ROUTE_LOCAL`.

The generated C++ service facade is close, but not yet uniform:

- `ConsumedService` already emits `routeAllLocal()` for RPC endpoints, plus
  remote/default transport helpers.
- `ProvidedService` binds typed service handlers and can restrict exposure to a
  remote peer with `routeAllRemote(peer_id)`, but it does not emit a symmetric
  `routeAllLocal()` helper to make "local peer only" an explicit provider-side
  option.
- Same-executor service use is therefore possible, but the copied examples
  emphasize shared-memory/remote routing rather than local component-to-component
  use.

The generated topic surface is less complete at component-facade level:

- Low-level generated helpers can encode/decode, subscribe, and publish typed
  topic payloads.
- `ConsumedService` emits typed subscriber convenience methods for generated
  subscribe topics.
- Component-facing publish ownership is still manual: component code must create
  a publisher port and call the low-level `publish*` helper with that port.
  This leaks `pcl_port_t*` ownership and lower-level topic wiring into ordinary
  component code.

`StandardBridge` remains a mixed facade/framework layer:

- It uses generated dispatch, encode/decode, and publish helpers.
- It still registers services, subscribers, and publisher ports directly with
  raw PCL calls. That may be acceptable for a framework bridge, but it should not
  remain the pattern copied by normal component authors.

## Gaps

| Gap | Impact |
|-----|--------|
| Local service routing is implicit on the provider side | A component cannot ask the generated provider facade to expose only local callers in the same way the consumer can route to local providers. |
| No canonical same-executor service facade proof | Existing tests prove PCL local dispatch and generated streaming behavior, but not the full generated component facade in a single executor with no transport. |
| Topic publishing is not owned by the component facade | Component authors still need `addPublisher`, `pcl_port_t*`, or low-level publish helpers for ordinary generated pub/sub. |
| No canonical same-executor pub/sub facade proof | The in-process pub/sub path is proven at lower layers, but not through a facade-only component-to-component example/test. |
| `StandardBridge` is a raw-PCL exception without a crisp policy | Reviewers cannot tell whether raw service/topic registration there is a deliberate adapter boundary or a facade gap still to close. |
| Local-peer docs lag the runtime capability | The authoring guide documents the single-threaded executor model, but does not show "two local components, one executor, no transport" as the standard facade deployment option. |

## TODO items

### E1. Make local-peer service routing explicit on the C++ facade

Add provider-side local routing to the generated component facade and make the
service routing vocabulary symmetric.

- **Plan:** emit `ProvidedService::routeAllLocal()` and document the local
  service route alongside `ConsumedService::routeAllLocal()`,
  `routeAllRemote()`, and `routeAllRemote(peer_id)`. Preserve the existing
  default behavior, but give component authors an explicit local-only call when
  a peer is in the same executor/process.
- **Accept:** generated providers and consumers can both be configured for local
  service-only communication through facade methods; remote route behavior is
  unchanged; generator tests pin the emitted method names.

### E2. Prove facade-only same-executor service communication

Add a canonical regression for ordinary local component-to-component RPC and
streaming.

- **Plan:** add a generated-binding test that composes a provider component and
  a consumer component into one `pcl::Executor`, sets local routing through the
  facade, does not install any transport adapter, then exercises unary and
  server-streaming calls.
- **Accept:** the test passes with no socket/shared-memory/gRPC/ROS2 transport
  configured; component code uses generated `ProvidedService` and
  `ConsumedService` only, with no raw `pcl_executor_*` service calls.

### E3. Promote generated pub/sub to component-facade ownership

Close the remaining topic facade gap so ordinary components do not own raw PCL
publisher ports for generated topics.

- **Plan:** for generated publish topics, emit component-facade methods that
  create and retain publisher ports during component configuration and expose
  typed `publish<Topic>(payload)` methods. Keep the low-level `publish*`
  helpers for framework/adapters/tests, but make the component facade the copied
  API for application code.
- **Accept:** a component can publish every generated topic through the facade
  without touching `pcl_port_t*`; content type validation and publisher
  lifecycle are handled by the binding; selected codec behavior remains
  byte-for-byte compatible except for the intended component facade header
  changes.

### E4. Prove facade-only same-executor pub/sub communication

Add local pub/sub coverage at the same abstraction level as service facade
coverage.

- **Plan:** create a test with two components in one executor and no transport:
  one publishes a generated topic through the component facade, the other
  subscribes through the component facade, and the executor delivers the typed
  payload locally.
- **Accept:** the test uses no raw `pcl_container_add_subscriber`,
  `pcl_container_add_publisher`, `pcl_port_publish`, or `pcl_msg_t` in the
  handwritten component logic; the received typed payload matches the published
  value for JSON and at least one binary content type.

### E5. Classify or migrate `StandardBridge` raw PCL wiring

Decide whether `StandardBridge` is a framework adapter that may keep raw PCL
wiring, or migrate it to the component facade once pub/sub ownership exists.

- **Plan:** after E1-E4, review `StandardBridge` service and topic wiring.
  Prefer composing generated `ProvidedService` / `ConsumedService` and the new
  topic facade where it does not obscure bridge-specific behavior. If any raw
  PCL calls remain, document them as framework-adapter exceptions and add a
  source guard so examples and normal components do not copy that pattern.
- **Accept:** reviewers can distinguish deliberate adapter code from ordinary
  component code; raw generated-service/topic PCL calls are either removed from
  `StandardBridge` or allowlisted with a documented reason and static coverage.

### E6. Document the local-peer deployment option

Make local component-to-component communication visible as a standard PYRAMID
deployment shape.

- **Plan:** update the component authoring guide with a same-executor example:
  provider and consumer in one process, local service routes, generated topic
  publish/subscribe through the facade, single-threaded `spinOnce`, and no
  transport adapter.
- **Accept:** docs show local peer, shared-memory peer, and remote transport as
  selectable deployment options without changing handler signatures or business
  component code.

## Regression bar for this workstream

Each implementation item should satisfy the standing PYRAMID generator bar from
the consolidated TODO, plus these local-peer checks:

1. Generated output changes are limited to intentional facade/API additions.
2. `python3 -m pytest subprojects/PYRAMID/tests -q` remains green.
3. Generated C++ binding tests include a no-transport same-executor service
   proof and a no-transport same-executor pub/sub proof.
4. The component authoring guide uses the component facade as the copied example
   for both service and pub/sub local-peer communication.

