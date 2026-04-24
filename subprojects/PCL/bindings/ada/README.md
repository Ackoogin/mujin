# Ada PCL Wrapper

This directory contains the canonical Ada binding surface for PCL.

Packages:

- `Pcl_Bindings`: thin import layer for the public C ABI
- `Pcl_Component`: OO wrapper for components, executors, ports, params, and routing
- `Pcl_Transports`: OO wrappers for socket, UDP, and shared-memory transports
- `Pcl_Content_Types`: named wire-format helpers
- `Pcl_Typed_Ports`: generic helper for typed publish/decode flows built on an existing codec pair

This Ada layer lives in `subprojects/PCL` so it can evolve with the C ABI and the
C++ wrappers instead of drifting inside downstream examples.

Start with the user guide:

- [`../../doc/guides/pcl_user_guide.md`](../../doc/guides/pcl_user_guide.md)
