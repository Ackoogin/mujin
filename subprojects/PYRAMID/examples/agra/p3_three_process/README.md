# A-GRA P3 three-process SDK example

This example uses the generated bindings from `pcl_pyramid_sdk_agra_p3` in
three separate processes. Each process contains one application component.
The executable `main` files only compose a `ProcessRuntime` and their component
class; port behavior remains in the component classes.

This is a transport and binding reference, not a claim of A-GRA compliance.

## Components and ports

| Process | Request port | Information port |
|---|---|---|
| `mission_coordinator` | Provides `ma_action_command.create` | Publishes and subscribes to `MA_Action` |
| `sensor_planner` | Consumes `ma_action_command.create` | Publishes and subscribes to `MA_Action` |
| `vehicle_planner` | Consumes `ma_action_command.create` | Publishes and subscribes to `MA_Action` |

The planners each send one typed `MA_ActionCommand_Service_Request`. The
coordinator handles both requests and returns a typed `Ack`. All three
components also publish typed `MA_Action_Service_Information` messages and read
messages sent by the other side of their link.

The component classes use the SDK-generated P3 types and native JSON binding
functions at their boundaries. They send the encoded bytes through PCL's RPC
and pub/sub ports. The deployment files select where those endpoints move.
This keeps transport details out of the component classes.

The coordinator treats the command ID as an idempotency key. A transport may
redeliver a request, so duplicate delivery returns the same positive `Ack`
without applying the example business action twice.

## Deployment configurations

Two complete Windows configurations are under `configs/shared_memory` and
`configs/tcp`:

- `shared_memory` loads `pcl_transport_shared_memory_plugin.dll`. It uses one
  named shared-memory bus for each planner-to-coordinator link.
- `tcp` loads `pcl_transport_socket_plugin.dll`. The coordinator listens on
  ports 19401 and 19402 because the socket transport owns one connection per
  transport instance.

The matching Linux configurations are under `configs/linux`. They load the
same transport plugins by their Linux `.so` names. Each process receives its
`.routes` file as its first argument. A file both
loads its transport plugin and maps the request and information endpoints to
named peers. The runtime also discovers and activates any RPC gateway
containers exposed by the selected transports.

All plugin paths are relative to the SDK root. The launchers therefore use the
selected SDK distribution as each process's working directory.

## Generate the P3 SDK bindings

Create the P3 SDK distribution as described in the
[PYRAMID SDK guide](../../../doc/architecture/sdk_packaging.md), then run its
binding generator once. The example needs C++ JSON bindings only. On Linux,
from the repository root:

```bash
dist/pcl_pyramid_sdk_agra_p3/scripts/generate_bindings.sh \
  --cpp --backends json
```

From a Visual Studio 2022 x64 developer prompt:

```bat
cd /d D:\Dev\repo\mujin\dist\pcl_pyramid_sdk_agra_p3
scripts\generate_bindings.bat --cpp --backends json
```

The SDK generator writes `sdk_project/generated`. This example deliberately
uses that directory rather than any generated files from the monorepo build.

## Build

On Linux, from the repository root:

```bash
cmake -S subprojects/PYRAMID/examples/agra/p3_three_process \
  -B build-agra-p3-example \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYRAMID_P3_SDK="$PWD/dist/pcl_pyramid_sdk_agra_p3"
cmake --build build-agra-p3-example --parallel 4
```

On Windows, from the repository root:

```bat
cmake -S subprojects\PYRAMID\examples\agra\p3_three_process ^
  -B build-agra-p3-example ^
  -DPYRAMID_P3_SDK=D:\Dev\repo\mujin\dist\pcl_pyramid_sdk_agra_p3
cmake --build build-agra-p3-example --config Release --parallel 4
```

The example disables the SDK FlatBuffers backend because the formal P3 seam
contains unions that the current SDK FlatBuffers backend does not support. It
builds and loads the generated JSON codec for the P3 C2 component.

## Run three separate processes

The helper starts the coordinator first, then launches both planners as
separate operating-system processes. Run either deployment from the repository
root. On Linux:

```bash
subprojects/PYRAMID/examples/agra/p3_three_process/run_three_processes.sh \
  --sdk-root dist/pcl_pyramid_sdk_agra_p3 \
  --build-dir build-agra-p3-example \
  --profile shared_memory
```

```bash
subprojects/PYRAMID/examples/agra/p3_three_process/run_three_processes.sh \
  --sdk-root dist/pcl_pyramid_sdk_agra_p3 \
  --build-dir build-agra-p3-example \
  --profile tcp
```

On Windows:

```powershell
subprojects\PYRAMID\examples\agra\p3_three_process\run_three_processes.ps1 `
  -SdkRoot D:\Dev\repo\mujin\dist\pcl_pyramid_sdk_agra_p3 `
  -BuildDir D:\Dev\repo\mujin\build-agra-p3-example `
  -Profile shared_memory
```

```powershell
subprojects\PYRAMID\examples\agra\p3_three_process\run_three_processes.ps1 `
  -SdkRoot D:\Dev\repo\mujin\dist\pcl_pyramid_sdk_agra_p3 `
  -BuildDir D:\Dev\repo\mujin\build-agra-p3-example `
  -Profile tcp
```

The output should show each planner's Create request being acknowledged by the
coordinator. It should also show `MA_Action` publications arriving in the other
processes. Use `--duration-seconds N` on Linux or `-DurationSeconds N` on
Windows to change the default 15-second run.

You can also run the executables in three terminals. Make the P3 SDK root the
working directory so the relative `plugins/...` paths in the routing files
resolve, then pass the matching `.routes` file to each executable.
