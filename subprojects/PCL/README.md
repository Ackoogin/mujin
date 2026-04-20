# PCL

`subprojects/PCL` contains the PYRAMID Container Library runtime, public headers, examples, and PCL-owned tests.

This directory is intended to be a future repo boundary for the low-level container/runtime layer.

## What It Provides

- C lifecycle container API with configure/activate/tick/deactivate/cleanup/shutdown states
- Single-threaded executor with queued external ingress and deferred service responses
- Publisher, subscriber, service, consumed-service, and stream-service endpoints
- Local, socket, UDP, and shared-memory transport support
- C++ RAII wrappers in `include/pcl/component.hpp` and `include/pcl/executor.hpp`
- Bridge helpers for transform/filter forwarding between endpoints

## Build And Test

From the workspace root:

```bat
cmake --preset default
cmake --build build --config Release --target pcl_core pcl_transport_socket pcl_transport_udp pcl_transport_shared_memory
ctest --test-dir build --output-on-failure -C Release -R "^Pcl"
```

For GNAT/Ada consumers, build GCC-compatible static libraries with:

```bat
subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat
```

## Directory Map

| Path | Contents |
|------|----------|
| `include/pcl/` | Public C ABI and C++ wrappers |
| `src/` | Core runtime, transports, logging, and bridge implementation |
| `tests/` | PCL lifecycle, executor, transport, robustness, threading, and wrapper tests |
| `examples/` | Minimal C examples |
| `scripts/` | GNAT static-library and coverage helpers |
| `doc/architecture/` | Design and component-system references |
| `doc/guides/` | Peer/transport configuration guide |
| `doc/requirements/` | PCL HLR and LLR requirements |

## Documentation

- [High-level requirements](doc/requirements/HLR.md)
- [Low-level requirements](doc/requirements/LLR.md)
- [Component container design](doc/architecture/component_container_design.md)
- [Component system overview](doc/architecture/08-pcl-component-system.md)
- [Peer and transport configuration guide](doc/guides/peer_transport_configuration.md)
- [Coverage report](../../doc/reports/PCL/COVERAGE_REPORT.md)
- [HLR coverage matrix](../../doc/reports/PCL/HLR_COVERAGE.md)
