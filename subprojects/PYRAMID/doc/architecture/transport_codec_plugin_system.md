# Transport & Codec Plugin System

## Purpose

PYRAMID components and clients run against the generic PCL runtime through
**runtime-loaded plugins**. A client links only the framework (`pcl_core`) and
the generated contract (typed facade + native↔C-struct marshalling). The
production plugin path covers JSON / FlatBuffers / Protobuf codecs and the
socket, shared-memory, and UDP transports. gRPC is generated and runtime-tested
as a direct C++ transport library, and its loadable coupled plugin
(`pyramid_grpc_coupled_plugin`) is a real runtime plugin implementing the
transport contract in both directions (server-mode ingress and client-mode
`invoke_async`/`invoke_stream`). ROS2 has a real rclcpp-backed coupled plugin
that is also complete both ways (pub/sub, consumed unary + streaming, safe
lifecycle, reproducible ament build). The transport **capability model** (caps
declaration, compose-time validation, QoS floor, manifest-driven per-endpoint
routing) is in place. Native ROS2 IDL (`.msg`/`.srv`) and the `domain_model`↔ROS2
marshalling/wire codec are generated and round-trip-verified; switching the live
ROS2 transport from the pass-through envelope to that typed codec is the headline
remaining item. Current capability and remaining work are tracked in
`doc/plans/PYRAMID/transport_plugins.md`.

This isolates churn (a wire-format or transport change ships as a new plugin, not
a client rebuild), keeps client binaries minimal, and lets one cross-language
codec `.so` serve both C++ and Ada over a frozen C-ABI struct boundary.

See also [generated_bindings.md](generated_bindings.md) and
[pcl_pyramid_binding_generation_overview.md](pcl_pyramid_binding_generation_overview.md).

## Runtime composition

```mermaid
flowchart TB
  subgraph Client["CLIENT / COMPONENT  (links: pcl_core + generated marshalling ONLY)"]
    Facade["Generated facade<br/>native types in/out"]
    Marshal["native ↔ pyramid_&lt;T&gt;_c<br/>(pyramid_marshal_&lt;module&gt;)"]
    Facade --> Marshal
  end

  subgraph Registry["PCL runtime (pcl_core)"]
    CodecReg["codec registry<br/>(content_type → vtable)"]
    Exec["executor<br/>setTransport(vtable)"]
    Loader["plugin loader<br/>dlopen + ABI check"]
  end

  subgraph Plugins["Runtime .so plugins (composed at load time)"]
    CodecSO["CODEC .so<br/>json | flatbuffers"]
    TransSO["TRANSPORT .so<br/>socket | shm | udp"]
    Grpc["coupled gRPC target<br/>both-ways (gated)"]
    Ros2["coupled ROS2 target<br/>both-ways, envelope (gated)"]
  end

  Marshal -- "pyramid_&lt;T&gt;_c" --> CodecReg
  CodecReg -- selects --> CodecSO
  Exec --> TransSO
  Loader -- "config_json (opaque)" --> CodecSO
  Loader -- "config_json (opaque)" --> TransSO
  Loader --> Grpc
  Loader --> Ros2
```

Key properties:

- **Fail closed.** With no codec registered for a `content_type`, encode/decode
  return an error (C++) or raise (Ada) rather than falling back to a built-in
  codec; likewise with no transport plugin. This holds for **both languages**,
  including Ada scalar aliases (`Identifier` etc.).
- **Uniform config pass-through.** A client switch flows as an opaque
  `config_json` string through the loader into both codec and transport entry
  points (transport carries role/host/port/bus + the executor pointer).
- **One cross-language codec.** The codec consumes the frozen `pyramid_<T>_c`
  C struct, so the same `.so` is loaded from C++ and Ada.

## Plugin directionality — the core contract

A transport plugin **must implement the full PCL transport contract in both
directions**. It is not enough to implement "certain bits" (e.g. only inbound
server ingress): a plugin shall provide everything a component needs to use
**both provided and consumed** services.

```
  PCL provider  <- plugin (server / ingress) <-  remote peer (client)
  PCL consumer  -> plugin (client / egress)  ->  remote peer (server)
```

The **core requirement is `PCL <-> plugin <-> PCL` both ways**: a PYRAMID
component on each side, with the plugin bridging the chosen middleware. Concretely
a complete plugin implements:

- **Provided (ingress):** accept inbound requests/streams/publishes from the wire
  and dispatch them to the executor's service/subscriber handlers (`serve` /
  `subscribe`, or a middleware server such as the gRPC/ROS2 coupled plugins start
  on load).
- **Consumed (egress):** route a component's outbound calls to a remote peer
  (`invoke_async` for unary, `invoke_stream` for server-streaming, `publish` for
  topics).

A plugin instance may be configured for one side (e.g. the socket plugin's
`role: server|client`, the gRPC plugin's `mode: server|client`), but the plugin
as a whole must cover both so a deployment can compose providers and consumers
freely.

### One-sided interop is also supported

Because each side is just the chosen middleware on the wire, a plugin can equally
bridge a PYRAMID component to a **native, non-PCL** peer — only one side is PCL:

```
  PCL consumer  -> gRPC plugin (client) -> native gRPC server (non-PCL)
  native gRPC client -> gRPC plugin (server) -> PCL provider
```

These one-sided interop cases are first-class use cases, but the **bidirectional
`PCL <-> plugin <-> PCL` path is the core contract** every transport plugin must
satisfy.

### Heterogeneous middleware capabilities (the capability model)

Middleware differ in which interaction primitives they provide. The capability
model makes that explicit so the framework can validate endpoint↔transport fit at
compose time (fail closed, early, precise) and deployments can mix middleware
per endpoint.

**Capabilities.** A capability is a named bundle of transport-vtable slots an
endpoint kind requires:

| Capability | Endpoint kinds | Vtable slots |
|------------|----------------|--------------|
| `PUBSUB`     | PUBLISHER / SUBSCRIBER          | `publish` / `subscribe` |
| `RPC_UNARY`  | PROVIDED / CONSUMED (service)   | server ingress + `respond` / `invoke_async` |
| `RPC_STREAM` | STREAM_PROVIDED / stream consume | server stream + `stream_send`/`stream_end` / `invoke_stream` (+ `stream_cancel`) |
| `RPC_ACTION` | (ROS2 actions; future)          | goal + feedback stream + result + cancel |

Orthogonal **QoS** (reliability `UNSPECIFIED < BEST_EFFORT < RELIABLE`, ordering,
durability, max size) is part of the profile, not a capability gate; an endpoint
may pin a minimum (e.g. a command service must be `RELIABLE`).

**Per-middleware matrix** (declared by each plugin; the test oracle):

| Transport | `PUBSUB` | `RPC_UNARY` | `RPC_STREAM` | `RPC_ACTION` | Reliability | Codec |
|-----------|:---:|:---:|:---:|:---:|---|---|
| gRPC      | ✗ | ✓ | ✓ | ⟂ | RELIABLE | coupled (protobuf) |
| ROS2      | ✓ | ✓ | ✓ | ✓ (native) | configurable QoS | coupled (ROS2 IDL) |
| TCP socket| ✓ | ✓ | ✗ | ✗ | RELIABLE | decoupled |
| shared mem| ✓ | ✓ | ✓ | ✗ | RELIABLE | decoupled |
| UDP       | ✓ | ✗ | ✗ | ✗ | BEST_EFFORT | decoupled |
| DDS/MQTT  | ✓ | ⟂/✗ | ✗ | ✗ | configurable | decoupled |

✓ native · ⟂ via an explicit adapter (not in v1) · ✗ unsupported. A component
whose contract has both topics and services cannot be carried by gRPC *or* UDP
alone — it needs ROS2/socket/shm, or per-endpoint routing across two transports.

**Declaration.** A plugin exports `pcl_transport_plugin_caps(config_json)` (a
`PCL_CAP_*` bitmask) and optional `pcl_transport_plugin_qos`; the loader exposes
them via `pcl_plugin_transport_caps()` / `_qos()`. If the symbol is absent the
loader derives the mask from non-NULL vtable slots (declaration is authoritative,
derivation is the fallback — coupled plugins whose vtables carry fail-closed stubs
**must** declare).

**Compose-time validation.** `pcl_executor_validate_endpoint_route()` checks each
routed endpoint's required cap (`pcl_endpoint_required_caps`) and QoS floor against
the routed transport's recorded caps/QoS, failing closed (`PCL_ERR_NOT_FOUND`
missing peer, `PCL_ERR_STATE` missing cap/QoS) with a precise diagnostic like
`endpoint "x" needs RPC_UNARY but peer "telemetry" (udp) provides {PUBSUB}`.

**Mixed-middleware routing.** The `pcl_transport_routing` manifest (line-based:
`transport <peer> <plugin> [config]` / `route <endpoint> <kind> <peers>
[reliability]`) loads each transport plugin (injecting the executor), records its
caps + QoS, registers it as a named peer, then installs and validates every route.
So one component can route its services over gRPC and its telemetry topics over
DDS/UDP. Capability *gaps* are **fail-closed by default**; opt-in adapters
(`PUBSUB over RPC_STREAM`, `RPC_UNARY over PUBSUB`) are not in v1 and, when added,
each advertises its derived capability behind explicit config.

## Code mechanism — ABI contracts

| Contract | Symbol | Signature | Header |
|----------|--------|-----------|--------|
| Codec | `pcl_codec_plugin_entry` | `const pcl_codec_t* (const char* config_json)` | `pcl/pcl_codec.h` (`PCL_CODEC_ABI_VERSION = 2`) |
| Transport | `pcl_transport_abi_version` + `pcl_transport_plugin_entry` | `uint32_t (void)` + `const pcl_transport_t* (const char* config_json)` | `pcl/pcl_plugin.h` (`PCL_TRANSPORT_ABI_VERSION = 1`) |
| Loader | `pcl_plugin_load_codec` / `pcl_plugin_load_transport` | `(path, config_json, …)` | `pcl/pcl_plugin_loader.h` |

```mermaid
sequenceDiagram
  participant App as client/app
  participant Ld as pcl_plugin_loader
  participant So as plugin .so
  participant Reg as codec registry / executor
  App->>Ld: load_codec(path, config_json, registry)
  Ld->>So: dlopen + resolve pcl_codec_plugin_entry
  Ld->>So: entry(config_json)
  So-->>Ld: const pcl_codec_t*  (abi_version checked == 2)
  Ld->>Reg: register(content_type → vtable)
  App->>Ld: load_transport(path, config_json)
  Ld->>So: abi_version() == 1 ? entry(config_json)
  So-->>Ld: const pcl_transport_t*
  App->>Reg: executor.setTransport(vtable)
```

The opaque `config_json` is stored and exposed to the codec via `codec_ctx`
(generated plugins) or consumed directly by the transport (e.g. the socket
plugin reads `{"role","host","port","executor"}`; shm reads
`{"bus_name","participant_id","executor"}`; the coupled ROS2 plugin reads
`{"node_name","executor"}` and stands up an rclcpp node + spin thread). Coupled
targets are intended to be loadable twice against the same `.so` -- once via
`pcl_plugin_load_transport`, once via `pcl_plugin_load_codec` -- presenting both
vtables under one `content_type`. Both coupled targets implement this
functionally: the gRPC plugin adapts the `pyramid_grpc_transport` runtime
(server + `invoke_async`/`invoke_stream`), and the ROS2 plugin wraps the
`RclcppRuntimeAdapter`. Default-plugin auto-load is
config-driven: `pcl_codec_registry_load_plugins_from_manifest(registry, path)`
loads every codec listed in a manifest (transport entries are skipped).

## Build mechanism

```mermaid
flowchart LR
  Proto["proto/**/*.proto"] --> Gen["generate_bindings.py<br/>(cpp_codegen / ada_codegen)"]
  Gen --> Facade["facade .cpp/.adb"]
  Gen --> MarshalSrc["pyramid_data_model_&lt;module&gt;_cabi_marshal.cpp"]
  Gen --> CodecSrc["*_codec_plugin.cpp (json/fb)"]

  MarshalSrc --> MObj["pyramid_marshal_&lt;module&gt;_obj (OBJECT)"]
  MObj --> MLib["pyramid_marshal_&lt;module&gt;.a (per module)"]
  MObj --> Agg["pyramid_generated_marshal.a (aggregate)"]

  CodecSrc --> CodecMod["libpyramid_codec_&lt;codec&gt;_&lt;component&gt;.so (MODULE)"]

  subgraph PCL["subprojects/PCL/src"]
    Sock["pcl_transport_socket_plugin.so (MODULE)"]
    Shm["pcl_transport_shared_memory_plugin.so (MODULE)"]
    Udp["pcl_transport_udp_plugin.so (MODULE)"]
    Core["pcl_core.a"]
  end

  Core --> ClientBin["client/app binary"]
  MLib --> ClientBin
```

- Codec plugins and transport plugins are CMake `MODULE` libraries (`.so`).
- `pyramid_grpc_transport` is built as a static library when gRPC is enabled.
  `pyramid_grpc_coupled_plugin` (PIC MODULE) is a real both-ways plugin: a config
  contract (`role`/`address`/aggregated component set), lifecycle (`shutdown` +
  private destroy), real server + `invoke_async`/`invoke_stream` wiring, and
  plugin-loaded round-trip tests (`test_grpc_coupled_plugin_e2e`).
- Marshalling is compiled **once per data-model module** as an `OBJECT` library,
  exposed both as a standalone `pyramid_marshal_<module>.a` (for per-component
  deployment) and bundled into the aggregate `pyramid_generated_marshal.a` (the
  name C++ and Ada consumers link by, unchanged).
- C++ and Ada bindings are regenerated into the build tree on configure/build:
  `${binaryDir}/generated/pyramid_cpp_bindings` and
  `${binaryDir}/generated/pyramid_ada_bindings`. Generated source-tree binding
  outputs are ignored and are not the source of truth.
- **End-to-end proto→plugins:** `scripts/build_plugins.sh` runs the whole
  pipeline (regenerate bindings from `.proto`, then build every codec/transport
  `.so` via the CMake `pyramid_plugins` aggregate target), suitable as a CI/CD
  stage or a manual engineer step. `--grpc` additionally builds protobuf + the
  coupled gRPC target plugin; `--stage` chains `stage_plugin_deploy.sh`.
- **Ada consumes, it does not produce, plugins.** The codec/transport `.so`s are
  language-neutral C-ABI artifacts; Ada clients and the Ada bridge load the same
  files at run time (`PYRAMID_CODEC_PLUGINS` / `PCL_TRANSPORT_PLUGIN`).
  `scripts/build_ada.sh` is the Ada counterpart to `build_plugins.sh`: it builds
  the Ada *binaries* via GNAT `gprbuild` (the `pyramid_ada_all` target, which also
  builds the GNAT FlatBuffers archive) plus `pyramid_plugins` so the `.so`s they
  load are present. `--regen` refreshes the build-local Ada binding tree from
  `.proto`.

## Deployment

`scripts/stage_plugin_deploy.sh` stages a per-component deployment dir:

```
<out>/<component>/
  plugins/                codec .so(s) + transport .so(s)
  include/  src/          PCL headers + generated facade the client compiles
  lib/                    libpcl_core.a + libpyramid_marshal_<module>.a  (closure only)
  codec_manifest.txt      codec .so paths  → PCL_CODEC_MANIFEST auto-loads them
  transport_manifest.txt  transport .so paths → pass via --transport-plugin
  MANIFEST.txt  README.md
```

```mermaid
flowchart TB
  subgraph Deploy["dist/plugin_deploy/tactical_objects"]
    Lib["lib/<br/>libpcl_core.a<br/>libpyramid_marshal_common.a<br/>libpyramid_marshal_tactical.a"]
    Plug["plugins/<br/>libpyramid_codec_json_tactical_objects.so<br/>libpyramid_codec_flatbuffers_tactical_objects.so<br/>libpcl_transport_socket_plugin.so<br/>libpcl_transport_shared_memory_plugin.so<br/>libpcl_transport_udp_plugin.so"]
    Man["codec_manifest.txt / transport_manifest.txt"]
  end
  Run["PCL_CODEC_MANIFEST=codec_manifest.txt ./client \\<br/>--transport-plugin &lt;socket|shm&gt;.so"] --> Plug
  Run --> Lib
```

**Module-closure staging (churn isolation).** Only the data-model modules a
component actually marshals are staged (derived from the component's codec
plugin's `*_cabi_marshal.hpp` includes): `tactical_objects` → `common`,
`tactical`; `autonomy_backend` → `common`, `autonomy`. An edit to an unrelated
module (`sensors`, `radar`, …) therefore produces no diff in an unrelated
component's deployment dir.

## Using & extending the plugin system

**1. Author a component/client.** Compile once against the generated typed facade;
link only `pcl_core` + `pyramid_generated_marshal`. Call the typed
provided/consumed APIs with native types — no `pcl_msg_t`, codec, or transport in
the call path. Select a payload `content_type` (e.g. `application/json`) when
configuring ports/services.

**2. Configure a deployment.** Load a codec and a transport plugin at runtime. The
directional/wiring knobs travel as one opaque `config_json` per plugin:

| Plugin | Key `config_json` fields |
|--------|--------------------------|
| socket | `role: provided\|consumed` (aliases `server`/`client`), `host`, `port`, `executor` |
| shared memory | `bus_name`, `participant_id`, `executor` |
| udp | `host`, `port`, `executor` (symmetric pub/sub) |
| gRPC coupled | `role: provided\|consumed` (alias `mode: server\|client`), `address`, aggregated component set, `executor` |
| ROS2 coupled | `role: provided\|consumed`, `node_name`, `executor` |

Codecs auto-load from a manifest (`PCL_CODEC_MANIFEST` →
`pcl_codec_registry_load_plugins_from_manifest`); transports are passed via
`--transport-plugin` or a `pcl_transport_routing` manifest for mixed middleware.
`scripts/stage_plugin_deploy.sh` produces a ready per-component deploy dir.

**3. Author a new plugin.** Build a CMake `MODULE` (`.so`) exporting the ABI
symbols above:
- **Codec:** `pcl_codec_plugin_entry(config_json) -> const pcl_codec_t*`
  (`PCL_CODEC_ABI_VERSION == 2`). Operate on the frozen `pyramid_<T>_c` struct so
  one `.so` serves C++ and Ada.
- **Transport:** `pcl_transport_abi_version() == 1` +
  `pcl_transport_plugin_entry(config_json) -> const pcl_transport_t*`. Implement
  both directions (§ directionality). Declare `pcl_transport_plugin_caps` (+
  `pcl_transport_plugin_qos`) so compose-time validation is accurate; export
  `pcl_transport_plugin_teardown` if the plugin owns threads/contexts to release
  before `dlclose`.

**4. ROS2 native typing.** Generate the ROS2 IDL + wire codec
(`PYRAMID_ENABLE_ROS2=ON`; `scripts/build_ros2_transport.sh`); see
[ros2_transport_semantics.md](ros2_transport_semantics.md) for the canonical
mapping and the `pyramid_msgs` interface package.

## Status

| Capability | C++ | Ada |
|------------|-----|-----|
| Client links core libs only (no transport/codec/wire deps) | ✅ | ✅ |
| Transport via runtime plugin (socket + shm + udp) | ✅ | ✅ |
| Codec via runtime plugin (cross-language `.so`) | ✅ | ✅ |
| Codec config pass-through (`config_json`) | ✅ | ✅ |
| Fail closed with no transport plugin | ✅ | ✅ |
| Fail closed with no codec plugin (incl. scalar aliases) | ✅ | ✅ |
| Coupled target plugin (codec+transport, one content_type) | gRPC both-ways; ROS2 both-ways (envelope) | — |
| Transport capability model (caps + compose-time validation + QoS + manifest routing) | ✅ | ✅ |

**Plugin binding v1 is complete**: clients link core libs only, transport + codec
are runtime plugins with uniform `config_json`, default-plugin manifests,
per-module marshalling, and both languages fail closed. The **direct generated
gRPC C++ transport** is built and
runtime-verified on Linux (`test_grpc_transport_smoke`,
`BindingPerformanceTest.Grpc_Tcp`); enable with `-DPYRAMID_ENABLE_GRPC=ON
-DPYRAMID_ENABLE_PROTOBUF=ON` or `scripts/build_plugins.sh --grpc`. The
loadable **coupled gRPC target plugin** is now a real runtime plugin implementing
the transport contract **both ways**: server mode starts a gRPC server (via a
proto-driven aggregator, `pyramid_grpc_plugin_aggregator`) that routes inbound
RPCs to the executor; client mode dials a remote endpoint and implements
`invoke_async` (unary) and `invoke_stream` (server-streaming) through the
generated typed stubs. `test_grpc_coupled_plugin_e2e` proves both directions
through the dlopen'd `.so` (PCL ↔ plugin ↔ plugin ↔ PCL).

The **coupled ROS2 target plugin** (`libpyramid_ros2_coupled_plugin`, content type
`application/ros2`) follows the same one-`.so`-two-vtables contract, with its
transport vtable wired to the real `RclcppRuntimeAdapter`. Loading it constructs
an rclcpp node and background spin thread; the vtable implements `publish` /
`subscribe`, consumed `invoke_async` (unary) and `invoke_stream`
(server-streaming, via the open-stream service + frame topic), and a process-safe
`spin_once` shutdown. Production closure is **complete**: reproducible fresh-tree
ament build (`scripts/build_ros2_transport.sh`, no committed generated files),
plugin-loaded live E2E (`colcon test` green), and `pcl_transport_plugin_teardown`
unload discipline. **Native ROS2 IDL + marshalling are done**: real `.msg`/`.srv`
(the `pyramid_msgs` package) and a `domain_model`↔`pyramid_msgs` `rclcpp` wire
codec (`pyramid_ros2_codec.hpp`), round-trip verified for every type. The live
ROS2 transport still carries the *pass-through envelope* (`PclEnvelope` /
`PclService` / `PclOpenStream`); switching it to the typed codec (codec-plugin
registration + typed adapter), plus ROS2 actions, is the remaining transport work
— see `doc/plans/PYRAMID/transport_plugins.md`.

**Architectural note — gRPC protobuf marshalling and Ada.** Ada consumes gRPC
the same way it consumes socket/shm: the gRPC coupled plugin is loaded as the PCL
transport, and the `application/protobuf` codec plugin is loaded through the
codec registry. There is no Ada-specific gRPC JSON shim in the active path.
`application/grpc` remains a wire-level gRPC transport detail; the Ada facade
uses the standard generated service API with `application/protobuf` payload
encoding.

**Current capability and remaining transport work** (putting the typed ROS2 codec
on the live wire, ROS2 actions, deferred capability adapters) are tracked in:
[`doc/plans/PYRAMID/transport_plugins.md`](../../../../doc/plans/PYRAMID/transport_plugins.md).
