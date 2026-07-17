# Component Skeletons: Generated Boilerplate, User-Owned Business Logic

This guide is for application authors who want a runnable PYRAMID component
process without writing the port wiring by hand. The generator produces a
**skeleton** (a base class that owns and connects every typed port) and a
one-time **scaffold** (the files you edit: handler classes, a thin lifecycle
class, a `main`, build wiring, and deployment configuration templates).

The split follows the *generation gap* pattern:

- **Generated files are regenerated on every run and must never be edited.**
  They carry the banner `Auto-generated component skeleton. Regenerated on
  every run.`
- **Scaffolded files are written only if they do not exist yet.** Once
  created they belong to you; re-running the generator never overwrites
  them. This is where all business logic lives.

If you want to compose port bindings into your own `pcl::Component` by hand
instead, read
[`cpp_component_authoring.md`](cpp_component_authoring.md). This page is the
higher-level path: the skeleton does that composition for you.

Throughout this guide the worked example is the `pim_osprey.sensors`
component from the MBSE-derived contract tree in `pim/test/pyramid`. It has
two provided request ports (Authorisation Dependency and SEN Requirement),
one provided information port (Capability), and one consumed information
port (Capability Evidence).

## Generating skeletons and scaffolds

Skeleton generation is off by default; nothing changes for existing users
until you ask for it.

From a packaged SDK:

```bash
<sdk-root>/scripts/generate_bindings.sh \
  --cpp --ada --backends json \
  --proto-dir <contract-tree> \
  --skeletons --scaffold-dir <your-workspace>
```

Directly through the generator:

```bash
python pim/generate_bindings.py <contract-tree> <output-dir> \
  --languages cpp,ada --backends json \
  --component-skeletons \
  --components pim_osprey.sensors,pim_osprey.sensor_products \
  --scaffold-dir <your-workspace>
```

- `--component-skeletons` turns the feature on. It requires the `pyramid`
  contract layout (the default).
- `--components` limits generation to a comma-separated list of
  `<project>.<component>` keys. Omit it to generate every component that has
  ports in the contract tree. Unknown keys are reported as errors.
- `--scaffold-dir` is where the one-time user files are written. It must be
  **outside** the generator output directory, so scaffolds can never be
  confused with regenerable output (the generator enforces this).
- In the monorepo build, `-DPYRAMID_GENERATE_COMPONENT_SKELETONS=ON` adds
  the flag to the codegen custom targets.

## What lands where

For each selected component the generator writes:

| File | Where | Ownership |
|------|-------|-----------|
| `pyramid_component_skeleton_support.hpp` | generator output dir | **Generated** — shared C++ handler ownership support; do not edit |
| `pyramid_component_<proj>_<comp>_skeleton.{hpp,cpp}` | generator output dir, next to `pyramid_services_*` | **Generated** — do not edit |
| `pyramid-skeletons-<proj>-<comp>.{ads,adb}` | generator output dir (Ada out dir in the SDK wrapper) | **Generated** — do not edit |
| `src/<comp>_handlers.{hpp,cpp}` | `<scaffold-dir>/<proj>_<comp>/` | **Yours** — port business logic goes here |
| `include/<proj>_<comp>/<comp>_component.hpp`, `src/<comp>_component.cpp` | `<scaffold-dir>/<proj>_<comp>/` | Yours — lifecycle logic goes here |
| `src/<comp>_main.cpp` | `<scaffold-dir>/<proj>_<comp>/` | Yours (rarely needs editing) |
| `CMakeLists.txt` | `<scaffold-dir>/<proj>_<comp>/` | Yours |
| `configs/{linux,windows}/{tcp,shared_memory}/<comp>.ports` | `<scaffold-dir>/<proj>_<comp>/` | **Yours — must be edited before first run** |
| `ada/src/pyramid-skeletons-<proj>-<comp>-impl.{ads,adb}`, `ada/src/<comp>_main.adb`, `ada/<comp>.gpr` | `<scaffold-dir>/<proj>_<comp>/ada/` | Yours — Ada business logic goes in the `Impl` package |

To re-create a single scaffold file from the template (for example after
deleting an experiment), delete just that file and re-run the generator: it
recreates missing files and prints `scaffold: kept <path>` for every file it
left alone.

## Where business logic goes (C++)

Put request handling in `src/<comp>_handlers.cpp`. The scaffold creates one
class per provided request port, and each class implements the existing
generated facade handler interface for that port:

```cpp
class SenrequirementHandler final
    : public services::provided::SenrequirementRequestPortHandler {
public:
  services::provided::Ack onCreate(
      const services::provided::SENRequirement_Service_Request&) override;
  services::provided::Ack onUpdate(
      const services::provided::SENRequirement_Service_Entity&) override;
  services::provided::Ack onCancel(
      const services::provided::Identifier&) override;
};
```

The generated skeleton constructor takes a `Handlers` aggregate. Its fields
follow contract port order:

| Your port | `Handlers` field | Required? |
|-----------|------------------|-----------|
| Provided request port | `HandlerSlot<<Port>RequestPortHandler> <port_key>` | Yes; pass a handler object by reference or a `std::unique_ptr` |
| Consumed information port | `std::function<void(const Frame&)> <port_key>` | Yes; `on_configure()` reports an empty slot and returns `PCL_ERR_STATE` |
| Consumed request port | `std::function<void(const Frame&)> <port_key>_transitions` | No; it defaults to empty and no transition subscription is created |
| Provided information port | No handler field | Not applicable; it is an outgoing port |

The main scaffold constructs the handler objects and injects them with
positional aggregate initialization (the project builds as C++17, which does
not have designated initializers). The generated comments name each field so
the positional list stays readable, and the order is the contract port order
used everywhere else by the generator:

```cpp
// scaffolded handler classes, from <comp>_handlers.hpp
sensors::AuthorisationDependencyHandler authorisation_handler;
sensors::SenrequirementHandler sen_requirement_handler;

sensors::SensorsSkeleton::Handlers handlers{
    /* .authorisation_dependency_request = */ authorisation_handler,
    /* .capability_evidence_information = */
    [](const services::consumed::Capabilities& item) {
      // sink business logic
    },
    /* .sen_requirement_request = */ sen_requirement_handler,
};
sensors::SensorsComponent component(runtime.executor(), std::move(handlers));
```

`HandlerSlot<T>` accepts either `T&` for caller-owned handlers or
`std::unique_ptr<T>` when the skeleton should own the handler.

Because the slots are typed by the **facade** handler interfaces, reusable
business logic binds to a port as a unit: a class that implements
`SenrequirementRequestPortHandler` can be handed to any component's matching
slot, in this process or another, without per-method plumbing.

The skeleton does not flatten port traffic into component hooks and does not
add named forwarding helpers. Its protected port accessors are the outbound
API:

| To do this | Call |
|------------|------|
| Publish on a provided information port | `capabilityInformationPort().publish(item)` |
| Report command progress on a provided request port | `senRequirementRequestPort().transitionWriter().send(item)` |
| Send a command through a consumed request port | `someRequestPort().submit(request)` |
| Use any other facade operation | Call it through the corresponding `<portKey>Port()` accessor |

Everything else — constructing the ports, `bind()`-ing them in
`on_configure()`, subscribing the sinks, and routing incoming commands to
your injected handlers is inside the generated skeleton. The derived
`<Comp>Component` remains only for lifecycle work such as `on_tick`. If you
need custom configuration, override `on_user_configure()` rather than
`on_configure()`; the skeleton calls it last, after all ports are bound.

If you generated a scaffold using the earlier hook-based design,
write-if-absent protects your existing `<comp>_component.*` files. They will
fail to compile against the new constructor. That is the migration signal to
move port logic into handler classes and inject a `Handlers` aggregate.

## Where business logic goes (Ada)

The generated package `Pyramid.Skeletons.<Proj>.<Comp>` declares a `Handlers`
record and a `Bind` procedure. The scaffolded `Impl` child is a plain package
of facade-compatible handler functions and sink procedures:

```ada
package Pyramid.Skeletons.Pim_Osprey.Sensors.Impl is
   function On_SEN_Requirement_Create
     (Request : SEN_Requirement_Service_Request) return Ack;
   procedure On_Capability_Evidence (Item : Capabilities);
   ...
end Pyramid.Skeletons.Pim_Osprey.Sensors.Impl;
```

The scaffolded main builds the generated record. Provided request fields are
the facade's `<Prefix>_Interaction_Handlers` records; required information
sinks are access-to-procedure values made with `'Access`; consumed request
transition callbacks are optional. `Runtime_Container.On_Configure` passes
that record to `Bind`. `On_Tick` remains lifecycle code owned by the
scaffolded runtime container.

There are no generated Ada forwarding helpers. Use the existing service
facade operations directly for outgoing work, such as
`<Prefix>_Send_Transition`, `Publish_<Topic>`, and `<Prefix>_Submit_<Rpc>`.

Two Ada-specific rules:

- One binding per component package per process. A package-level `Bound` flag
  makes a second `Bind` raise `Program_Error`, matching the underlying Ada
  facades. `Bind` also rejects null required sink procedures.
- Where the current Ada facade layer does not expose a decoder for a
  local-wrapper information topic, `Bind` **fails closed**: it raises
  `Program_Error` with an explanatory message instead of silently dropping
  data, and the bind site carries a comment saying so.

## Configuration: what you must edit before running

### The `.ports` deployment files

The process takes a `.ports` file as its first command-line argument. The
scaffold provides four templates under `configs/` (Linux and Windows plugin
names, TCP and shared-memory transports), one line per logical port:

```
# port NAME MODE PEER PLUGIN PLUGIN_CONFIG
port sen_requirement_request rpc <peer-process> plugins/pcl_transport_socket_plugin.so {"role":"<provided-or-consumed>","host":"127.0.0.1","port":<port>}
```

Every `<...>` placeholder must be replaced before the first run — the file
is a template, and the runtime rejects a file that does not configure every
generated port exactly once.

- `NAME` is the port key. It must match an entry of the generated
  `deploymentPorts()` list; the template already contains one correct line
  per port, so keep the names as they are.
- `MODE` is `rpc` or `pubsub` and selects which realization of the port is
  routed. The same compiled binary supports both; nothing in your code
  changes when you flip this. The template defaults request ports to `rpc`
  and information ports to `pubsub`.
- `PEER` is the name the *other* process uses for itself on this
  connection. All lines that share a peer must use the same plugin and
  plugin configuration.
- `PLUGIN` is the transport plugin path, relative to the SDK root.
- `PLUGIN_CONFIG` is passed verbatim to the plugin: host/port JSON for the
  socket transport, bus name and participant id for shared memory.

### The codec plugin path

The generated `main` loads one JSON codec plugin whose path arrives through
the compile definition `PYRAMID_COMPONENT_CODEC_PLUGIN_PATH`. The scaffolded
`CMakeLists.txt` already sets it to the SDK-built plugin for your component
(`pyramid_codec_json_<proj>_<comp>`), so there is nothing to edit unless you
relocate the built plugin at deploy time.

### CMake cache variables

The scaffolded `CMakeLists.txt` builds against a packaged SDK:

- `PYRAMID_SDK` (required): path to the SDK root.
- `PYRAMID_SDK_COMPONENTS`: pre-set to your component key; the SDK project
  compiles the matching skeleton into the `pyramid_component_skeletons`
  library and pulls in the right service bindings automatically.
- The skeleton `.cpp` must exist in the SDK's generated tree; if it does
  not, CMake stops with an error telling you to re-run the generator with
  `--component-skeletons`.

### Run duration

```bash
./pim_osprey_sensors configs/linux/tcp/sensors.ports              # run until SIGINT/SIGTERM
./pim_osprey_sensors configs/linux/tcp/sensors.ports --duration-seconds=15   # timed run (tests, demos)
```

## How your component gets wired in

There is no registry or factory. Constructing the component and passing it to
`ProcessRuntime::run()` *is* the registration, in three stages — all visible
in the scaffolded `main`:

```cpp
pcl::ProcessRuntime runtime(
    argc, argv, {PYRAMID_COMPONENT_CODEC_PLUGIN_PATH},
    SensorsSkeleton::deploymentPorts());          // 1. routes loaded
SensorsComponent component(
    runtime.executor(), std::move(handlers));     // 2. ports self-attach
return runtime.run(component);                    // 3. lifecycle + executor
```

1. **Route setup.** Before any component exists, `ProcessRuntime` reads the
   `.ports` file and matches each line against the static
   `deploymentPorts()` list, loading transport routes and activating the
   per-peer gateways.
2. **Construction is the attachment point.** The skeleton constructor stores
   your `Handlers`, then constructs every port facade with
   `(*this, executor, ...)` — each port attaches itself to the component and
   the executor, and provider ports capture your injected handler object, so
   incoming commands dispatch straight to it.
3. **`runtime.run(component)`** drives the PCL lifecycle: `configure` (the
   skeleton binds every port against the loaded routes and subscribes your
   sink functions, then calls `on_user_configure()`), `activate`, then the
   component joins the executor spin loop and starts receiving
   `on_tick(dt)`. On exit it symmetrically removes, deactivates, and cleans
   up.

One component instance per process is the scaffold's model; a system of
several components is several processes (see
`examples/agra/p3_three_process`). On the Ada side the registration *is*
explicit: the scaffolded main calls the generated
`Bind (Container, Exec, Binding)` from its container's `On_Configure`, and a
second `Bind` in the same process raises `Program_Error`.

## Regeneration rules

Safe at any time: re-running the generator. Skeletons are emitted
byte-identically for an unchanged contract (this is pinned by golden tests),
and scaffolds are never rewritten.

When the contract itself changes (a port is added, or a message type
changes), the regenerated `Handlers` aggregate picks the change up
automatically. A new provider port fails to compile until you supply its
handler (`HandlerSlot` has no default state), and a new sink port that you
leave uninitialized is caught at startup: `on_configure()` reports the empty
slot by name and returns `PCL_ERR_STATE`. Both are the intended signal that
new business logic is required.

## Verifying an installation

- `pim/test_harness/build_component_skeleton_smoke.sh --sdk-root <sdk>`
  generates and compiles the untouched sensors scaffold (C++ and, when
  `gprbuild` is available, Ada).
- `pim/test_harness/component_skeleton_e2e.sh --sdk-root <sdk>` runs a
  filled-in sensors provider against a client harness over shared memory in
  both `rpc` and `pubsub` modes.

Both harnesses generate into a scratch directory and leave the SDK's own
generated contract tree untouched.
