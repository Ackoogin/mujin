# A-GRA P3 three-process SDK example

This example uses the generated bindings from `pcl_pyramid_sdk_agra_p3` in
three separate processes representing the C2, Mission Autonomy, and Mission
System roles. Each process contains one application component.
The executable `main` files only compose a `ProcessRuntime` and their component
class; port behavior remains in the component classes.

This is a transport and binding reference, not a claim of A-GRA compliance.

## Components and ports

| Process | Request port | Separate Information port |
|---|---|---|
| `c2` | Consumes `MA_ActionCommand`; sends Create and receives its typed `Ack` response | Subscribes to `MA_Action` from Mission Autonomy |
| `mission_autonomy` | Provides `MA_ActionCommand` to C2; consumes `MA_TaskCommand` from the Mission System | Publishes `MA_Action` to C2 and `MA_Task` to the Mission System |
| `mission_system` | Provides `MA_TaskCommand`; returns its typed `Ack` response | Subscribes to `MA_Task` from Mission Autonomy |

The flow follows the interface directions recorded from Table 3-1 of the MA L1
Compliance Document. C2 sends a typed `MA_ActionCommand_Service_Request` to
Mission Autonomy. Mission Autonomy returns an `Ack` on that Request port, then
sends a typed `MA_TaskCommand_Service_Request` to the Mission System. The
Mission System returns its own `Ack` on the downstream Request port.

The `MA_Action` and `MA_Task` Information ports are independent of those
Request/response interactions. They demonstrate one-way typed publications;
they are not request payloads or response channels.

The component classes use only SDK-generated facade and message types. They do
not handle `pcl_msg_t`, response-buffer ownership, or JSON serialization. The
generated binding layer selects the configured codec and hides the encoded
bytes. The deployment files select where the typed Request and Information
ports move, keeping transport and codec details out of the components.

Both providers treat the command ID as an idempotency key. A transport may
redeliver a request, so duplicate delivery returns the same positive `Ack`
without applying the example business action twice.

## Deployment configurations

Two complete Windows configurations are under `configs/shared_memory` and
`configs/tcp`:

- `shared_memory` loads `pcl_transport_shared_memory_plugin.dll`. It uses one
  named shared-memory bus for the C2-to-Mission-Autonomy link and another for
  the Mission-Autonomy-to-Mission-System link.
- `tcp` loads `pcl_transport_socket_plugin.dll`. Mission Autonomy listens on
  port 19401 and the Mission System listens on port 19402. Mission Autonomy
  provides the C2 connection and consumes the Mission System connection.

The matching Linux configurations are under `configs/linux`. They load the
same transport plugins by their Linux `.so` names. Each process receives its
`.ports` file as its first argument. The runtime also discovers and activates
any RPC gateway containers exposed by the selected transports.

All plugin paths are relative to the SDK root. The launchers therefore use the
selected SDK distribution as each process's working directory.

Each non-comment line configures one logical generated port:

```text
port NAME MODE PEER PLUGIN PLUGIN_CONFIG
```

- `NAME` is the port's local configuration key.
- `MODE` is `rpc` or `pubsub`.
- `PEER` identifies the other process on this connection.
- `PLUGIN` is the transport plugin library.
- `PLUGIN_CONFIG` is passed unchanged to that plugin. For example, it contains
  a shared-memory bus and participant identity, or a TCP role and endpoint.

The generated facade supplies the individual RPC or topic endpoint names and
their directions through `deploymentDescriptor()`. The example runtime expands
the short port record into PCL's validated endpoint routes internally, including
mutual exclusion and the reliable QoS floor. Components therefore neither
repeat low-level routes nor hard-code `rpc` or `pubsub`.

The example selects RPC for each Request port and pub/sub for each separate
Information port. A typed Request response returns on the same RPC interaction
and has no response port or second configuration line.

## Generate the P3 SDK bindings

Create the P3 SDK distribution as described in the repository's PYRAMID SDK
guide, then run its binding generator once. The package includes this example
under `examples/agra/p3_three_process`. The example needs C++ JSON bindings
only. On Linux, from the SDK root:

```bash
scripts/generate_bindings.sh --cpp --backends json
```

From a Visual Studio 2022 x64 developer prompt, start in the SDK root:

```bat
cd /d D:\Dev\repo\mujin\dist\pcl_pyramid_sdk_agra_p3
scripts\generate_bindings.bat --cpp --backends json
```

The SDK generator writes `sdk_project/generated`. This example deliberately
uses that directory rather than any generated files from the monorepo build.

## Build

On Linux, from the SDK root:

```bash
cmake -S examples/agra/p3_three_process \
  -B build-agra-p3-example \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYRAMID_P3_SDK="$PWD"
cmake --build build-agra-p3-example --parallel 4
```

On Windows, from the SDK root:

```bat
cmake -S examples\agra\p3_three_process ^
  -B build-agra-p3-example ^
  -DPYRAMID_P3_SDK=%CD%
cmake --build build-agra-p3-example --config Release --parallel 4
```

The example disables the SDK FlatBuffers backend because the formal P3 seam
contains unions that the current SDK FlatBuffers backend does not support. It
builds and loads the generated JSON codecs for the P3 C2 and Mission System
interfaces. Components still see only generated C++ types; the codec layer
owns JSON encoding and decoding.

## Run three separate processes

The helper starts the Mission System, Mission Autonomy, and C2 as separate
operating-system processes. Run either deployment from the SDK root. On Linux:

```bash
examples/agra/p3_three_process/run_three_processes.sh \
  --sdk-root "$PWD" \
  --build-dir build-agra-p3-example \
  --profile shared_memory
```

```bash
examples/agra/p3_three_process/run_three_processes.sh \
  --sdk-root "$PWD" \
  --build-dir build-agra-p3-example \
  --profile tcp
```

On Windows:

```powershell
examples\agra\p3_three_process\run_three_processes.ps1 `
  -SdkRoot $PWD `
  -BuildDir "$PWD\build-agra-p3-example" `
  -Profile shared_memory
```

```powershell
examples\agra\p3_three_process\run_three_processes.ps1 `
  -SdkRoot $PWD `
  -BuildDir "$PWD\build-agra-p3-example" `
  -Profile tcp
```

The output should show C2's action Create request acknowledged by Mission
Autonomy, followed by Mission Autonomy's task Create request acknowledged by
the Mission System. It should separately show `MA_Action` Information arriving
at C2 and `MA_Task` Information arriving at the Mission System. Use
`--duration-seconds N` on Linux or `-DurationSeconds N` on Windows to change
the default 15-second run.

You can also run the executables in three terminals. Make the P3 SDK root the
working directory so the relative `plugins/...` paths in the port files
resolve, then pass the matching `.ports` file to each executable.
