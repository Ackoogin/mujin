# PCL Mutex Audit

Companion to `COVERAGE_REPORT.md`. Walks every mutex acquisition in the PCL
runtime to confirm lock-ordering, identify any deadlock hazards, and flag
state that is *not* mutex-protected so the implicit single-threaded-mutator
contract is visible.

Generated: 2026-05-21

Scope: `subprojects/PCL/src/pcl_executor.c`, `pcl_transport_socket.c`,
`pcl_transport_shared_memory.c`, `pcl_transport_template.c`,
`pcl_transport_udp.c`. `pcl_container.c`, `pcl_bridge.c`, `pcl_log.c`,
`pcl_transport_apos.c` -- and the later plugin/routing-era sources
`pcl_codec_registry.c`, `pcl_plugin_loader.c`, `pcl_transport_routing.c`,
`pcl_capabilities.c`, `pcl_alloc.c` -- use no synchronisation primitives of
their own.

## Inventory

| Module | Lock | Type | Protects |
|--------|------|------|----------|
| executor | `containers_lock` | `pcl_mutex_t` (CRITICAL_SECTION / pthread_mutex) | `containers[]`, `container_count` |
| executor | `incoming_lock` | `pcl_mutex_t` | `incoming_head`, `incoming_tail` |
| executor | `resp_cb_lock` | `pcl_mutex_t` | `resp_cb_head`, `resp_cb_tail` |
| executor | `svc_req_lock` | `pcl_mutex_t` | `svc_req_head`, `svc_req_tail` |
| socket transport | `send_lock` + `send_cond` / `send_event` | producer-consumer | `send_head/tail` outbound queue |
| socket transport | `pending_lock` | mutex | `pending_head` correlation table, `next_seq_id` |
| SHM transport | `pending_lock` | process-local mutex | `pending_head`, `pending_stream_head`, `active_stream_targets`, `next_seq_id` |
| SHM transport | `lock_sem` / `lock_handle` (bus_lock) | inter-process semaphore / named mutex | `region->slots[]`, `region->participant_count` |
| template transport | `send_lock` + `send_cond` / `send_event` | producer-consumer | `send_head/tail` outbound queue |
| template transport | `pending_lock` | mutex | `pending_head` correlation, `peer_aliases`, `peer_id` buffer, `next_seq_id` |
| UDP transport | `recv_stop`, `recv_running` | `volatile int` flags | recv-thread stop signaling |

`pcl_transport_udp.c` is single-flag synchronised: it has one receiver thread
and a `volatile int recv_stop` flag, no critical sections, no condvars. The
recv thread polls the flag between blocking `recvfrom` calls (made bounded
via `SO_RCVTIMEO`). Safe by inspection.

## Executor Lock Ordering

The executor never nests its four locks. Each lock is acquired, the bounded
critical section runs, and the lock is released before any callback fires:

```
pcl_executor_add/remove        : containers_lock                   (no other lock held)
drain_incoming_queue           : incoming_lock                     (dispatch runs after release)
drain_resp_cb_queue            : resp_cb_lock                      (cb fires after release)
drain_svc_req_queue            : svc_req_lock                      (find_service runs after release)
pcl_executor_destroy           : containers_lock, then            (each released before the next)
                                 incoming_lock, then resp_cb_lock,
                                 then svc_req_lock                 (strict sequential, never nested)
```

All four destroy-time acquisitions are pure dequeue passes that null out the
head/tail pointers under the lock, then walk the detached linked list with
no lock held. Free order is therefore independent of acquisition order, and
no callback or transport entry point can observe a partially-freed queue.

## Transport Lock Orderings

### Socket transport

Internally only two locks; never nested.

```
invoke_async:
  pending_lock  -> register entry -> release
  send_lock     -> enqueue frame  -> release        (never under pending_lock)

invoke_async rollback (enqueue failed):
  pending_lock  -> remove entry   -> release        (send_lock already released)

recv_thread (SVC_RESP path):
  pending_lock  -> take entry     -> release
  -- callback dispatched via pcl_executor_post_response_msg (executor lock)

destroy:
  recv_stop/send_stop assignments first
  pending_lock and send_lock individually held only for short windows
```

There is no path where `pending_lock` and `send_lock` are both held at the
same time. The send queue drain in destroy (`pcl_transport_socket.c:1352`)
runs after both threads have joined, so no concurrent access.

### Shared-memory transport

Two locks plus the bus mutex; never nested across categories.

```
invoke_async:
  pending_lock  -> register entry              -> release
  loop { bus_lock -> find_provider_slot         -> release }
  bus_lock      -> enqueue STREAM_REQ frame    -> release   (never under pending_lock)

invoke_stream: same pattern as invoke_async with the stream variant.

recv_thread main loop:
  sync_local_services:
    containers_lock -> collect ports          -> release    (line 593-624)
    bus_lock        -> publish slot summary    -> release    (line 635-651)
  bus_lock          -> dequeue one frame       -> release    (line 1713-1727)
  handle_frame: dispatches to executor queues, never under bus_lock.

gateway_sub_cb (executor thread):
  find_remote_service: containers_lock -> walk ports -> release
  optional bus_lock for send_response   -> release
  (containers_lock and bus_lock acquired sequentially, never nested)

destroy:
  recv_stop=1, join recv_thread.
  bus_lock -> clear our slot -> release.
  pcl_shm_active_streams_abort:
    pending_lock -> detach list -> release
    iterate list with no lock: stream_send_frame takes bus_lock individually.
```

`pcl_shm_collect_local_services` (line 583) is the only place that takes
`containers_lock`; it is always released before the surrounding code
acquires `bus_lock`. The `active_streams_abort` path likewise detaches the
list under `pending_lock` then walks the *detached* list with no lock held,
so `bus_lock` -> `pending_lock` can never form a cycle.

### Template transport

Identical shape to the socket transport, plus a peer-alias list under
`pending_lock`:

```
set_peer_id:
  pending_lock -> snapshot peer_id, snprintf into peer_id buffer,
                  tpl_alias_remember_locked, restore on OOM -> release
  (this was the data race fixed in commit e3a9b2c -- now an atomic
   single-mutex sequence)

recv_thread main loop:
  recv_blocking (no lock).
  pending_lock -> snapshot peer_id into stack buffer -> release
  PUBLISH:   pcl_executor_post_remote_incoming        (executor queue lock)
  SVC_REQ:   pcl_executor_post_service_request_remote (executor queue lock)
  SVC_RESP:  pending_lock -> take entry -> release;
             pcl_executor_post_response_msg            (executor queue lock)

destroy:
  Iterates peer_aliases via pending_lock, then releases before invoking
  pcl_executor_register_transport(... NULL) per alias. Same defensive
  release-then-call pattern as SHM destroy.
```

`send_lock` and `pending_lock` are never both held. The destroy alias walk
runs entirely outside `pending_lock` (the list is detached first), so calls
into the executor cannot reenter back through any transport lock.

## Cross-Module Lock-Order Graph

Drawing every "if I hold X, can I take Y" edge across all five modules:

```
                          (none -- no nested locks across modules)

executor.containers_lock  ----->  (nothing -- iterations under it never call transports)
executor.incoming_lock    ----->  (nothing)
executor.resp_cb_lock     ----->  (nothing)
executor.svc_req_lock     ----->  (nothing)
socket.send_lock          ----->  (nothing)
socket.pending_lock       ----->  (nothing)
shm.pending_lock          ----->  (nothing)
shm.bus_lock              ----->  (nothing)
template.send_lock        ----->  (nothing)
template.pending_lock     ----->  (nothing)
```

Every lock is acquired around a bounded, lockless-callbacks critical section
and released before any external entry point is invoked. With an empty
edge set in this graph, **no lock-order inversion can occur, and no deadlock
between two PCL mutexes is possible.**

## Recv-Thread vs Main-Thread Synchronisation

The executor's `containers_lock` is the only cross-thread synchronisation
visible to transports. The lock comment in `pcl_internal.h` lines 135-138
documents the intent:

```
/// Guards \ref containers and \ref container_count against concurrent
/// mutation by the main thread (pcl_executor_add / pcl_executor_remove /
/// pcl_container_destroy via auto-detach) and iteration by transport recv
/// threads (e.g. shared-memory service lookups).
```

The lock is acquired by:

- `pcl_executor_add` / `pcl_executor_remove` (`pcl_executor.c:347, 361`) on
  whichever thread the user calls them from -- typically the main thread,
  occasionally a transport teardown thread.
- `pcl_container_destroy` via auto-detach (`pcl_container.c:94`).
- Transport recv threads via the exposed
  `pcl_executor_containers_lock/unlock` (SHM transport uses these in
  `find_remote_service`, `find_remote_stream_service`, and
  `collect_local_services`).

The lock is **not** acquired by the executor's own spin-thread iterations
(`find_service`, `find_stream_service`, `dispatch_incoming_now`,
`drain_svc_req_queue`'s container walk). The implicit contract is that the
spin thread is *also* the only thread expected to mutate the container
list, so iteration on the spin thread races nothing.

This is documented and intentional, but worth restating: **callers must not
invoke `pcl_executor_add` or `pcl_executor_remove` from a different thread
while `pcl_executor_spin_once` is running on the spin thread.** The
transport recv threads honour this by using the public lock helpers; user
code that follows the same pattern (lock, mutate, unlock) is also safe.

## Unprotected State (relies on the user-contract)

Three fields are touched from more than one thread but have no mutex of
their own. These are real concurrency surfaces -- code that mutates them
outside the documented quiesce-before-teardown contract will race:

1. **`pcl_executor_t::transports[]`** and `transport_count`
   (`pcl_executor.c:761-783`). Mutated by `pcl_executor_register_transport`;
   read by `find_named_transport` from the executor spin thread (e.g. inside
   `drain_svc_req_queue` -> `find_service` paths that consult it). Both
   transport teardown (`pcl_socket_transport_destroy:1291`,
   `pcl_shared_memory_transport_destroy:1880`,
   `pcl_transport_template_destroy:957`) and user-driven registration call
   this. The contract is: register on the main thread before spin starts,
   and only unregister via transport `destroy` *after* the executor has
   stopped. The aliasing fix in commit `403f9b3` made unregister rebind-safe
   but did not add a mutex.

2. **`pcl_executor_t::transport`** and `has_transport`
   (`pcl_executor.c:730-740`). Same pattern; same contract.

3. **`pcl_executor_t::endpoint_routes[]`** and `endpoint_route_count`
   (`pcl_executor.c:802-823`). Mutated by `pcl_executor_set_endpoint_route`;
   read by `port_route_mode`, `port_peer_count`, `port_peer_id_at` on the
   spin thread. Same contract.

These three could be hardened with a single `routing_lock` covering all
three arrays, taken read-side around `find_named_transport`,
`port_route_mode`, and friends. The change is non-trivial (the spin thread
reads them under the hot path of every dispatch), so the contract was
preferred. **Recommended hardening when scaling beyond a single
configurator thread:** introduce that lock, or freeze routing config at
container-configure time.

## Recv Threads in Particular

Each transport runs at least one background thread whose teardown sequence
deserves attention because misordered shutdown is a classic source of
use-after-free more than deadlock:

- `pcl_socket_transport_destroy` clears the executor slots first, sets stop
  flags, then closes sockets (which unblocks blocking recv/send), then
  joins, then destroys mutexes. **Correct order.**
- `pcl_shared_memory_transport_destroy` clears the executor slots, sets
  `recv_stop`, joins the recv thread *before* tearing down the gateway
  container or the bus mapping. `pcl_shm_active_streams_abort` then runs
  after recv has stopped, so no concurrent stream registration is possible.
  **Correct order.**
- `pcl_transport_template_destroy` (lines 919-1021) clears default + alias
  executor slots, signals `send_stop`, calls `hooks.wake` so the engineer's
  `recv_blocking` returns, joins both threads, drains pending entries
  under `pending_lock`, destroys mutexes, calls `hooks.close` *after* both
  threads are joined. **Correct order**, and the engineer-facing contract
  ("close runs on a quiescent system") is the same as the other transports.
- `pcl_transport_udp_destroy` sets `recv_stop`, closes the UDP socket to
  wake the blocked recv, joins. No mutex to tear down. **Correct.**

## Recent Fixes Audited (commits 18b7236, e3a9b2c, 403f9b3)

The three template-transport fixes that landed just before this audit each
addressed a real concurrency bug:

- `18b7236` (set_peer_id atomic + create OOM): `set_peer_id` now snapshots
  the original peer_id before the alias-list write so an OOM mid-rename
  rolls the field back atomically. **Verified clean.**
- `e3a9b2c` (peer_id / recv_thread race): both reads and writes of
  `ctx->peer_id` now happen under `pending_lock`. The recv thread
  snapshots into a stack buffer before dispatching. **Verified clean.**
- `403f9b3` (alias unregistration of rebound peers): destroy now consults
  `pcl_executor_get_transport_for_peer` and only clears slots that still
  point at this adapter. **Verified clean and covered by the new
  `test_pcl_executor.cpp::GetTransportForPeerResolvesRegisteredAdapter`
  test.**

No regressions detected.

## Findings Summary

| Finding | Severity | Action |
|---------|----------|--------|
| No deadlock between any pair of PCL mutexes | -- | None required |
| Lock-order graph is acyclic (in fact empty -- no nesting anywhere) | -- | None required |
| `e->transports[]`, `e->transport`, `e->endpoint_routes[]` rely on single-thread-mutator contract | Low / documented | Re-audit if multi-threaded re-registration is ever introduced |
| `containers_lock` not taken by executor spin's own iteration | Low / documented | Same |
| Recent set_peer_id + recv_thread race | Fixed (e3a9b2c) | Verified |
| Alias-rebinding clobber on destroy | Fixed (403f9b3) | Verified, covered by new test |
| set_peer_id partial-write under OOM | Fixed (18b7236) | Verified |

No code changes are recommended off the back of this audit. The two
unprotected routing tables are deliberate design choices; if they become a
problem when scaling to multi-configurator scenarios, a single read-write
lock covering all three is the minimum hardening.
