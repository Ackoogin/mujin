# HLR Requirements Coverage

> **Generated file.** Regenerate with `python3 subprojects/PCL/scripts/gen_hlr_coverage.py`.
> Do not hand-edit; fix `HLR.md`/`LLR.md` or the test requirement tags and regenerate.

Generated: 2026-07-09

End-to-end traceability: **Test -> LLR -> HLR** for all PCL requirements.

Each test carries `///< REQ_PCL_NNN` tags; each LLR's **Traces** field names its parent HLR(s). The matrix below is derived from those two sources plus the **Verification** pointers in `LLR.md`.

## Container Lifecycle

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.001 | Lifecycle State Machine | 001, 003 | test_pcl_integration, test_pcl_lifecycle |
| PCL.002 | Lifecycle Transitions | 004, 005, 006 | test_pcl_integration, test_pcl_lifecycle |
| PCL.003 | Transition Callback Invocation | 004, 008 | test_pcl_integration, test_pcl_lifecycle |
| PCL.004 | Callback Failure Aborts Transition | 007, 009, 010, 011 | test_pcl_integration, test_pcl_lifecycle, test_pcl_robustness |
| PCL.005 | Container Identity | 001, 114 | test_pcl_integration, test_pcl_lifecycle, test_pcl_robustness |
## Port Management

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.006 | Port Types | 019, 022 | test_pcl_lifecycle, test_pcl_robustness |
| PCL.007 | Port Creation During Configure Only | 012, 019, 020 | test_pcl_lifecycle, test_pcl_robustness |
| PCL.008 | Port Capacity Limit | 021 | test_pcl_robustness |
| PCL.009 | Publish Constraint | 023, 024, 026, 027, 106, 107 | test_pcl_proto_bindings, test_pcl_robustness |
| PCL.010 | Subscriber Callback Dispatch | 044 | test_pcl_executor |
| PCL.011 | Service Handler Dispatch | 040 | test_pcl_robustness |
| PCL.011a | Async Service Invocation | 164, 165 | test_pcl_robustness, test_pcl_socket_transport |
| PCL.011b | Deferred Service Response | 459, 460, 461 | test_pcl_executor, test_pcl_robustness |
| PCL.011c | Streaming Service Response | 167, 168, 169, 170, 171, 172, 311, 312 | test_pcl_shared_memory_transport, test_pcl_streaming |
## Parameters

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.012 | Typed Parameter Storage | 013, 014, 015 | test_pcl_lifecycle |
| PCL.013 | Parameter Capacity Limit | 016 | test_pcl_robustness |
| PCL.014 | Parameter Default Values | 013, 017 | test_pcl_lifecycle, test_pcl_robustness |
## Tick Rate and Periodic Execution

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.015 | Configurable Tick Rate | 028, 029, 092 | test_pcl_lifecycle, test_pcl_robustness |
| PCL.016 | Tick Rate Validation | 030 | test_pcl_lifecycle |
| PCL.017 | Delta Time Reporting | 043 | test_pcl_robustness |
## Executor

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.018 | Multi-Container Execution | 031, 032, 033, 034, 084, 091, 093 | test_pcl_dining, test_pcl_executor, test_pcl_robustness |
| PCL.019 | Spin and Spin-Once | 032 | test_pcl_executor |
| PCL.020 | Shutdown Request | 035, 055 | test_pcl_executor, test_pcl_robustness |
| PCL.021 | Graceful Shutdown | 036, 041, 042 | test_pcl_executor, test_pcl_robustness |
| PCL.022 | Intra-Process Direct Dispatch | 027, 044, 045, 047, 165 | test_pcl_executor, test_pcl_robustness |
| PCL.023 | Service Invocation | 040 | test_pcl_robustness |
| PCL.024 | Container Add and Remove | 037, 039 | test_pcl_robustness |
## Cross-Thread Ingress

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.025 | Cross-Thread Message Posting | 049, 050, 051, 052, 053, 054, 055, 056 | test_pcl_executor, test_pcl_robustness |
| PCL.026 | Ingress Queue Drain | 054 | test_pcl_robustness |
| PCL.027 | Async Response Delivery | 057, 058, 059, 060, 061 | test_pcl_robustness |
## Transport Adapter

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.028 | Transport Adapter Interface | 048 | test_pcl_robustness |
| PCL.029 | Transport Wiring | 048, 062, 063, 384 | test_pcl_plugin_loader, test_pcl_robustness |
| PCL.030 | Dispatch Incoming From Transport | 044 | test_pcl_executor |
| PCL.030a | Transport Client Service Invocation | 164, 165, 166 | test_pcl_robustness, test_pcl_socket_transport |
| PCL.030b | Endpoint Routing Configuration | 173, 174, 175 | test_pcl_executor |
| PCL.030c | Named Peer Transport Registration | 174, 175, 178 | test_pcl_executor |
| PCL.030d | Remote Ingress Filtering | 173, 201, 306, 311, 314, 327, 438 | test_pcl_executor, test_pcl_shared_memory_transport, test_pcl_template_transport, test_pcl_udp_transport |
## TCP Socket Transport

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.031 | Server Mode | 115, 116, 158, 159, 193, 195, 197 | test_pcl_socket_transport |
| PCL.032 | Client Mode | 117, 118, 119, 190, 191, 192, 193, 194, 195, 196, 197 | test_pcl_socket_transport |
| PCL.033 | Wire Protocol | 120, 121, 176, 412, 413 | test_pcl_socket_faults, test_pcl_socket_transport |
| PCL.034 | Gateway Container | 122, 123, 124, 163, 177, 409 | test_pcl_socket_faults, test_pcl_socket_transport |
| PCL.035 | Non-Blocking Send | 125, 161 | test_pcl_socket_transport |
| PCL.036 | Async Remote Service Invocation | 126, 127, 128, 160, 162, 163, 177, 410, 411 | test_pcl_socket_faults, test_pcl_socket_transport |
| PCL.036a | Remote Peer Identity | 176, 177, 179, 199 | test_pcl_socket_transport, test_pcl_udp_transport |
| PCL.036b | Inter-Process Shared Memory Bus | 186, 187, 188, 305, 306, 307, 310, 313, 314, 315, 316, 326, 327 | test_pcl_shared_memory_transport |
| PCL.036c | Shared Memory Publish Fan-Out | 186, 187, 211 | test_pcl_shared_memory_transport |
| PCL.036d | Shared Memory Async Remote Service Invocation | 188, 189, 308, 309 | test_pcl_shared_memory_transport |
| PCL.036e | Robust Client Connect Semantics | 190, 191, 192, 193, 194, 195, 196, 197, 204, 205, 414, 415 | test_pcl_socket_faults, test_pcl_socket_transport |
| PCL.036f | UDP Datagram Transport (Pub/Sub Only) | 198, 199, 200, 201, 202, 203, 301, 302, 303, 304 | test_pcl_udp_transport |
| PCL.036g | Shared Memory Atomic Fan-Out And Topic Backpressure | 211, 212, 457 | test_pcl_shared_memory_transport, test_pcl_transport_threading |
## Logging

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.037 | Printf-Style Logging | 064, 065, 067, 071, 073, 074 | test_pcl_log, test_pcl_robustness |
| PCL.038 | Pluggable Log Handler | 064, 068, 072 | test_pcl_log, test_pcl_robustness |
| PCL.039 | Log Level Filtering | 066, 070 | test_pcl_log, test_pcl_robustness |
| PCL.040 | Log Levels | 069 | test_pcl_robustness |
## Bridge

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.041 | Bridge Creation | 075, 081, 082, 083, 328 | test_pcl_bridge, test_pcl_dining |
| PCL.042 | Bridge Transform Dispatch | 079, 084 | test_pcl_bridge, test_pcl_dining |
| PCL.043 | Bridge Message Suppression | 080 | test_pcl_bridge, test_pcl_dining |
| PCL.044 | Bridge Null Safety | 076, 077, 078 | test_pcl_bridge, test_pcl_dining |
## Robustness and Error Handling

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.045 | Null Handle Safety | 002, 018, 022, 025, 038, 046, 050, 060, 063, 111, 112, 113, 114, 116, 118, 123, 127, 129, 130, 166, 170, 184, 189, 192, 193, 198, 212, 216, 298, 302, 317, 340, 347, 353, 362, 377, 378, 381, 388, 390, 399, 401, 404, 406, 408, 429, 431, 432, 460 | test_pcl_alloc, test_pcl_apos_transport, test_pcl_capabilities, test_pcl_codec_registry, test_pcl_executor, test_pcl_integration, test_pcl_lifecycle, test_pcl_plugin_loader, test_pcl_robustness, test_pcl_shared_memory_transport, test_pcl_socket_transport, test_pcl_streaming, test_pcl_template_transport, test_pcl_transport_routing, test_pcl_udp_transport |
| PCL.046 | Allocation Failure Handling | 016, 021, 037, 052, 061, 082, 085, 086, 087, 088, 089, 090, 185, 309, 310, 318, 319, 409, 410, 411, 413 | test_pcl_bridge, test_pcl_dining, test_pcl_oom, test_pcl_robustness, test_pcl_shared_memory_transport, test_pcl_socket_faults, test_pcl_transport_routing |
| PCL.047 | Status Codes | 462 | test_pcl_lifecycle, test_pcl_robustness |
## C++ Wrappers

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.048 | Component Base Class | 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 157, 217 | test_pcl_cpp_wrappers |
| PCL.049 | Executor Wrapper | 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157 | test_pcl_cpp_wrappers |
## Service Bindings (Proto/Generated)

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.050 | Wire-Name Constants | 094, 096 | test_pcl_proto_bindings |
| PCL.051 | Topic Constants | 095, 097 | test_pcl_proto_bindings |
| PCL.052 | Service Handler Base Class | 102, 103, 110 | test_pcl_proto_bindings |
| PCL.053 | JSON Builder Functions | 098, 099, 100, 101 | test_pcl_proto_bindings |
| PCL.054 | Subscribe Wrappers | 104, 105 | test_pcl_proto_bindings |
| PCL.055 | Dispatch Function | 108, 109 | test_pcl_proto_bindings |
| PCL.056 | Public Route Configuration API | 179 | test_pcl_socket_transport |
## Cross-Thread Ingress

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.057 | Cross-Thread Local Service Request Queuing | 180, 181, 182, 183, 184, 185 | test_pcl_robustness |
## Codecs

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.058 | Codec Abstraction | 346 | test_pcl_codec_registry |
| PCL.059 | Codec Registry | 337, 338, 339, 340, 341, 342, 343, 344, 345, 385, 395 | test_pcl_codec_registry, test_pcl_plugin_loader |
## Transport Capabilities and QoS

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.060 | Transport Capability Declaration | 347, 348, 349, 350, 351, 352, 354, 355, 364 | test_pcl_capabilities |
| PCL.061 | Endpoint Capability Requirements | 363, 375 | test_pcl_capabilities |
| PCL.062 | QoS Declaration and Floor Semantics | 359, 360, 361, 370, 371, 372, 373, 374, 376, 377, 378, 426 | test_pcl_capabilities, test_pcl_transport_routing |
| PCL.063 | Compose-Time Route Validation | 365, 366, 367, 368, 369, 371, 372, 373, 374, 379, 380, 420 | test_pcl_capabilities, test_pcl_transport_routing |
## Plugin ABI and Loader

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.064 | Transport Plugin ABI | 354, 355, 383, 384, 397 | test_pcl_capabilities, test_pcl_plugin_loader |
| PCL.065 | Plugin Loader Fail-Closed Behaviour | 353, 362, 386, 393, 394, 395, 396, 397, 398, 402 | test_pcl_capabilities, test_pcl_plugin_loader |
| PCL.066 | Codec Plugin Batch Loading | 385, 391, 392, 399, 400, 401, 405 | test_pcl_plugin_loader |
| PCL.067 | Safe Teardown-Then-Unload | 381, 382, 403, 407 | test_pcl_plugin_loader |
| PCL.068 | Reference Transport Plugins | 356, 357, 358, 359, 360, 361, 387, 388, 389, 390, 403, 404, 406, 407, 408 | test_pcl_capabilities, test_pcl_plugin_loader |
## Manifest-Driven Endpoint Routing

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.069 | Routing Manifest | 317, 416, 417, 418, 419, 425 | test_pcl_transport_routing |
| PCL.070 | Routing Atomicity and Fail-Closed Diagnostics | 275, 318, 319, 416, 420, 421, 422, 423, 424, 425, 426, 427, 428, 466, 467, 468 | test_pcl_transport_routing |
## Transport Template and Conformance

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.071 | Transport Template Scaffold | 286, 288, 290, 292, 293, 294, 295, 320, 321, 429, 430, 431, 432, 434, 435, 436, 437, 438, 439, 440 | test_pcl_template_transport |
| PCL.072 | Transport Conformance Suite | 433, 441, 442, 443, 445, 446, 447 | test_pcl_apos_transport, test_pcl_template_transport |
| PCL.073 | APOS LVC Transport | 297, 298, 300, 322, 323, 324, 325, 444, 445, 446, 447 | test_pcl_apos_transport |
## Portable Allocator

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.074 | Portable Allocator | 213, 214, 215, 216 | test_pcl_alloc |
## Transport Threading Model

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.075 | Transport Threading-Model Contract | 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458 | test_pcl_transport_threading |
| PCL.076 | Transport Threading-Model Conformance Suite | 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458 | test_pcl_transport_threading |
## Manifest-Driven Endpoint Routing

| HLR | Description | LLR(s) | Test File(s) |
|-----|-------------|--------|--------------|
| PCL.077 | Manifest Exclusive Realization Groups | 463, 464, 465, 466, 467, 468, 469 | test_pcl_transport_routing |
| PCL.078 | Manifest-Driven Remote Streaming Invoke and Gateway Discovery | 470, 471 | test_pcl_transport_routing |


## Summary

| Metric | Count |
|--------|-------|
| HLRs | 92 |
| LLRs | 386 |
| LLRs with at least one verifying test file | 386 |
| Distinct test files carrying REQ_PCL tags | 17 |
| HLRs with no tracing LLR | 0 |
| LLRs with no test evidence | 0 |
| LLRs with no parent HLR | 0 |
| Test tags naming unknown LLRs | 0 |

## Trace Gaps

### HLRs with no tracing LLR

None.

### LLRs with no test evidence

None.

### LLRs with no parent HLR trace

None.

### LLR references to unknown HLRs

None.

### Test tags naming unknown LLRs

None.

## Trace Tag Format

Tests use LLR requirement tags in comments:

```
///< REQ_PCL_NNN: Brief description. PCL.0XX.
```

Each LLR in `LLR.md` has a **Traces** field listing the parent HLR(s), completing the chain:

```
Test (code tag) -> LLR (LLR.md) -> HLR (HLR.md)
```

