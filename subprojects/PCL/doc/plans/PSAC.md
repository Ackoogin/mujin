# PCL Plan for Software Aspects of Certification (PSAC)

| Field | Value |
|-------|-------|
| Document | PCL-PSAC |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL` (PYRAMID Composition Library) |
| Approval | Pending certification liaison; no certification authority engagement has occurred yet |

This PSAC follows the DO-178C section 11.1 content outline. It is an
engineering draft: it records the intended certification approach so the other
life-cycle documents have a fixed frame of reference. It has not been
submitted to any certification authority.

## 1. System Overview

PCL is a deployable component container framework for autonomous mission
systems. It provides lifecycle management (UNCONFIGURED → CONFIGURED → ACTIVE
→ FINALIZED), deterministic single-threaded execution of component business
logic, publisher/subscriber/service/stream inter-component communication, and
a pluggable transport abstraction (local, TCP socket, UDP, shared memory,
APOS Local Virtual Channels).

PCL is infrastructure software: it hosts application components but contains
no domain business logic. System-level requirements, the system safety
assessment, and the failure-condition categorisation belong to each embedding
programme; see section 4.

## 2. Software Overview

- Language: strict C17, zero external dependencies (design decision D1 in
  `../requirements/HLR.md`).
- Concurrency model: all user callbacks execute on a single executor thread
  (D2); transport I/O threads communicate only through deep-copied queue
  ingress (D5, PCL.075).
- Failure philosophy: fail-closed compose-time validation of all endpoint
  routing (D9); bounded port/parameter capacities; consistent status codes.

## 3. Certification Considerations

### 3.1 Software Level

Target software level: **DAL C**, with the DAL B delta tracked separately in
`../../../../doc/reports/PCL/DO178C_GAP_ANALYSIS.md`. The level is a target
assumption, not an allocated level: PCL has no parent system safety
assessment yet. When a programme embeds PCL, the programme's PSAC governs and
this document becomes an input to it.

### 3.2 Certification Boundary

The airborne-software boundary for the certified configuration is:

| Item | In boundary | Notes |
|------|-------------|-------|
| `src/pcl_alloc.c`, `pcl_container.c`, `pcl_executor.c`, `pcl_log.c`, `pcl_bridge.c` | Yes | Core runtime |
| `src/pcl_capabilities.c`, `pcl_transport_routing.c` | Yes | Compose-time validation |
| Reference transports (`pcl_transport_socket.c`, `_udp.c`, `_shared_memory.c`, `_template.c`, `_apos.c`) | Per deployment | Only the transports a deployment routes to are airborne software; the rest are excluded from the certified part list |
| `src/pcl_codec_registry.c`, `src/pcl_plugin_loader.c` + `*_plugin.c` shims | **Deactivated in the certified configuration** | Runtime `dlopen` composition is for non-certified deployments; the certified configuration requires a static-registration build (see 3.3) |
| C++ wrappers (`include/pcl/*.hpp`) | No | Separate item; consumers carry their own assurance argument |
| Ada binding (`bindings/ada/`) | No | Separate item, same rationale |
| Tests, examples, scripts, template/conformance harnesses | No | Verification environment, not airborne software |

### 3.3 Special Considerations

1. **Field-loadable software / dynamic linking.** The plugin loader
   (PCL.064–PCL.070) is a certification complication. Planned mitigation: a
   static-registration build option that links and registers transports at
   build time and compiles the loader out, so the `dlopen` path is
   deactivated code in the airborne configuration. This build option does not
   exist yet (open item, gap analysis section 4.2).
2. **Dynamic memory during ACTIVE operation.** Message ingress deep-copies
   and queue nodes allocate from the heap. The bounded-memory argument, or a
   bounded-arena mode behind `pcl_alloc`, is an open item (gap analysis
   section 4.1).
3. **Target environment.** All current verification evidence is host-based
   (Linux/GCC, Windows/MSVC + GNAT). Credit-seeking testing must be repeated
   on the target computer or a justified target-representative environment
   (the APOS binding indicates an ASAAC/APOS RTOS target).
4. **Tool qualification.** gcovr (structural coverage) and
   GoogleTest/CTest (test pass/fail) outputs are used for verification
   credit; both need TQL-5 qualification or documented independent checking
   of their output (see SVP section 7).

## 4. Software Life Cycle and Life-Cycle Data

Processes are defined in the companion plans, all in this directory:

| Plan | Document |
|------|----------|
| Development | `SDP.md` |
| Verification | `SVP.md` |
| Configuration management | `SCMP.md` |
| Quality assurance | `SQAP.md` |

Standards: `../standards/requirements_standard.md`,
`../standards/design_standard.md`, `../standards/c_coding_standard.md`.

Life-cycle data inventory and current status is maintained in
`../../../../doc/reports/PCL/DO178C_GAP_ANALYSIS.md` section 1
(the evidence inventory table).

## 5. Schedule

No certification schedule exists. Sequencing follows the gap-analysis
roadmap: Phase 1 (DAL C readiness) then Phase 2 (DAL B delta).

## 6. Additional Considerations

- **Previously developed software**: none. PCL is developed in-repo from
  scratch with full git history.
- **COTS**: none in the airborne boundary (D1). GoogleTest/CMake/gcovr are
  verification-environment tools only.
- **Alternative methods**: none proposed.
