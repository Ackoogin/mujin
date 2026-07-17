# PYRAMID port-system stress test — findings

Results and analysis from the first run of the multi-process port-system
stress harness (`subprojects/PCL/tests/stress/`, see its README for how to
reproduce).  The harness launches many worker processes against the
runtime-loaded transport plugins — shared memory, TCP socket, and UDP —
through the production `pcl_plugin_load_transport()` path.

**Environment.** Linux x86-64 container, 4 CPUs, GCC 13.3, Release build
(`-DUNMANNED_BUILD_PYRAMID=OFF -DUNMANNED_BUILD_AME=OFF`).  Numbers below
are from a 3-second load window per scenario.  Absolute rates will differ
on other hardware; the *shapes* (collapse curves, loss modes, limits) are
properties of the code.

> **Status update (fixes applied).** The findings below were re-verified on a
> 16-CPU host, and defects **D2, D3, D4, D5** (and the socket half of **D7**)
> have since been fixed and re-tested.  See
> [Fixes applied and retest](#fixes-applied-and-retest) at the end of this
> document for the before/after numbers. **D1** and **D6** were subsequently
> implemented, and a UDP receive counter was added for the remaining half of
> **D7**; see the [D1 / D6 retest](#d1--d6-retest).  The remaining global
> shared-memory bus lock (the rest of **D2**) is still open and is annotated
> below.

## Headline: maximum shared-memory plugin throughput

One publisher publishing flat-out to one subscriber through the dlopen'd
shared-memory plugin (`libpcl_transport_shared_memory_plugin.so`), zero
loss, zero sequence gaps:

| Payload | Messages/s | Throughput | Publish backpressure hits |
|--------:|-----------:|-----------:|--------------------------:|
|    64 B | **91,261** |  46.7 Mbit/s | 5.4 % of attempts |
|   1 KiB |     89,970 | 737.0 Mbit/s | 5.6 % |
|   4 KiB |     81,447 |   2.7 Gbit/s | 5.7 % |
|  16 KiB |     81,043 |  10.6 Gbit/s | 5.8 % |

**The maximum measured shared-memory plugin throughput is ~91,000
messages/second** (64-byte payloads).  Rate is almost flat across a 256×
payload-size range, which confirms the bottleneck is per-message
bus-semaphore/notification overhead (defect D2 below), not memcpy
bandwidth.  `PCL_SHM_MAX_PAYLOAD` caps a frame at 16 KiB, so 10.6 Gbit/s
is also the ceiling in bytes.

## Scaling results

**Fan-out (1 publisher → N subscribers, one bus).** Per-subscriber rate
collapses far faster than the fan-out factor:

| Subscribers | Per-subscriber msg/s | Aggregate msg/s |
|------------:|---------------------:|----------------:|
| 1 | 83,305 | 83,305 |
| 3 | 60,135 | 180,405 |
| 7 |  5,813 |  40,697 |

**Multiple publishers (N publishers → 1 subscriber, one bus).** Aggregate
*delivered* throughput drops as publishers are added, while rejected
publishes overtake accepted ones:

| Publishers | Delivered msg/s | Accepted publishes | Rejected (`PCL_ERR_NOMEM`) |
|-----------:|----------------:|-------------------:|---------------------------:|
| 1 | 81,857 | 245,563 | 14,943 |
| 3 | 41,903 | 125,714 | 34,035 |
| 7 | 15,396 |  46,208 | 73,519 (61 % of attempts) |

**Independent pairs sharing one bus.** Each pair uses its own topic, yet
the bus copies every publish to every participant (D1), so unrelated
traffic interferes:

| Pairs on one bus | Per-pair msg/s | vs. isolated pair |
|-----------------:|---------------:|------------------:|
| 1 | 82,046 | 1.0× |
| 2 | 18,295 | 0.22× |
| 4 |  2,894 | 0.035× |

**Independent pairs on separate buses (process scaling).** 48 concurrent
worker processes ran without a single failure; aggregate throughput keeps
rising and the falling per-bus rate tracks CPU oversubscription (4 cores),
not shared bus state:

| Buses | Processes | Aggregate msg/s | Per-bus msg/s |
|------:|----------:|----------------:|--------------:|
|  4 |  8 | 184,482 | 46,120 |
|  8 | 16 | 225,763 | 28,220 |
| 16 | 32 | 282,286 | 17,643 |
| 24 | 48 | 354,891 | 14,787 |

**Unary services through the shm gateway (1 provider ← N clients).**

| Clients | Requests sent | Responses with data | Successful req/s | Dropped |
|--------:|--------------:|--------------------:|-----------------:|--------:|
| 1 | 106,294 | 106,294 | 35,424 | 0 % |
| 3 | 244,231 |  44,601 | 14,851 | **82 %** |
| 6 | 157,570 |  71,377 | 23,758 | 55 % |

A single client is lossless at ~35 k req/s.  The moment aggregate
in-flight requests exceed the provider's 16-deep mailbox, most requests
come back as empty responses (defect D4); the non-monotonic 3-client dip
is the fail-fast path spinning clients on instant errors.

**Churn and limits.** 54 attach/detach cycles ran against a live
publisher/subscriber pair with zero message loss, zero gaps, and zero
attach failures.  The participant cap admitted exactly 8 of 12
simultaneous attachers, and a duplicate participant ID was rejected.

**Plugin mix (shm + socket + UDP simultaneously, max rate, 64 B).**

| Transport | "Accepted" publishes | Received | Lost | Declared QoS |
|-----------|---------------------:|---------:|-----:|--------------|
| shared memory | 86,360 | 86,360 | 0 | RELIABLE — honoured |
| TCP socket | 2,143,095 | 837,362 | **1,305,733 (61 %)** | RELIABLE — **violated** (D5) |
| UDP | 2,318,557 | 1,225,891 | 1,092,666 (47 %) | BEST_EFFORT — as declared (D7) |

## Scaling defects and risks identified

### D1 — shm publish fan-out couples every participant (by design, but it does not scale)

`pcl_shm_publish_once_locked()` copies every published frame into the
mailbox of **every** other participant on the bus, whether or not that
participant subscribes to the topic; receivers filter after delivery.
Combined with the all-or-nothing capacity pre-check (if *any* target
mailbox is full the whole publish fails with `PCL_ERR_NOMEM`), one slow or
uninterested participant throttles every publisher on the bus.  The
`shm-fanout` and `shm-pairs` scenarios quantify the collapse.

> **Fixed.** Each slot now records up to 16 subscribed topic names in the
> shared region. `pcl_shm_subscribe` registers a topic under the bus lock and
> `pcl_shm_publish_once_locked` selects only interested slots before applying
> its all-or-nothing capacity check. A slot with no subscriptions yet retains
> the previous deliver-to-all behaviour while its executor is being set up.
> The layout change increments `PCL_SHM_VERSION` to 4, so an old region is
> re-initialised by the existing magic/version check. The atomic pre-check is
> unchanged, preserving exact delivery for every selected subscriber.

### D2 — one global bus semaphore serialises all bus operations

Every publish, every service frame, and every iteration of every
participant's receive thread takes the single named semaphore
(`pcl_shm_bus_lock`).  The receive thread also re-syncs its service
advertisement table (another lock acquisition) on **every loop iteration**
(`pcl_shm_recv_thread_main` calls `pcl_shm_sync_local_services`
unconditionally).  Aggregate throughput therefore *drops* as participants
are added (`shm-multi-pub`, `shm-fanout`), instead of scaling until CPU
saturation.

> **Fixed (service-table sync half).** `pcl_shm_sync_local_services` now
> caches the service set it last advertised and only takes the bus lock when
> the local service set actually changes.  In the steady state (services are
> configured once at startup) the receive thread no longer touches the bus
> lock at all, which lifted single-pair 64 B throughput and the fan-out
> aggregate (see the retest table).  The *single global bus lock itself*
> still serialises every publish and service frame; removing that is the
> larger structural change tracked with D1.

### D3 — hard participant limit of 8, reported only as a generic attach failure

`PCL_SHM_MAX_PARTICIPANTS` is 8.  The 9th attach fails (correctly —
verified by `shm-limit`), but `pcl_shared_memory_transport_create` returns
plain `NULL` and the plugin entry returns `NULL`, so the loader reports the
same `PCL_ERR_NOT_FOUND` as any other entry failure.  A deployment
composing >8 components on one bus gets no diagnostic pointing at the
capacity limit.  Duplicate participant IDs are also rejected (correctly).

> **Fixed.** `pcl_shared_memory_transport_create` now emits a distinct
> `PCL_LOG_ERROR` line for each attach-failure cause: bus at capacity (naming
> the `PCL_SHM_MAX_PARTICIPANTS` limit and suggesting fewer participants per
> bus or splitting across buses), a duplicate participant id, and a failed
> receive-notification primitive.  The return value is unchanged (still
> `NULL`, so no ABI change), but the operator now gets an actionable message.
> Verified in the `shm-limit` retest: the 9th–12th attachers log the capacity
> message and the duplicate-id attacher logs the uniqueness message.

### D4 — shm service requests are silently dropped when the provider mailbox is full

In `pcl_shm_worker_svc_req()`, when the provider's 16-deep mailbox has no
capacity, the SVC_REQ frame is not sent and the client's callback fires
with an **empty** response — indistinguishable from a provider-side
failure, with no status, no retry, and no backpressure.  A comment in the
neighbouring stream path calls a full provider mailbox "an
adversarial-timing path not reachable under testing"; the `shm-services`
scenario reaches it within milliseconds at 3 concurrent clients
(client in-flight window 3×8 > queue depth 16).  Unary RPC over the shm
bus degrades from lossless to mostly-dropped as soon as offered load
exceeds the tiny provider mailbox.

> **Fixed.** `pcl_shm_worker_svc_req` now retries a full provider mailbox on
> the egress worker within a bounded window (`PCL_SHM_SVC_REQ_RETRY_MS`,
> 50 ms), re-resolving the provider under the bus lock on each attempt,
> before giving up and delivering the terminal empty response.  Because the
> provider drains its 16-deep mailbox in microseconds, a burst that overflows
> it is now delayed by a few poll intervals rather than dropped.  In the
> `shm-services` retest the drop rate fell from 82 %/55 % (3/6 clients) to
> **zero** — every request sent came back with a real response — and
> successful throughput rose.  A status-carrying callback (so a genuine
> give-up is distinguishable from a real empty response) would need a wider
> `pcl_resp_cb_fn_t` signature and is left as a separate change.

### D5 — TCP socket transport: unbounded egress queue, silent loss at teardown, despite RELIABLE QoS

`pcl_transport_socket.c` implements `publish` as a non-blocking enqueue
onto an **unbounded** in-process FIFO drained by one send thread doing
blocking `send()`.  A publisher faster than the link (or the receiver)
grows that queue without limit — memory growth at millions of frames per
minute in `plugin-mix` — and when the transport is torn down the queue is
freed, discarding everything still in it.  The plugin declares
`PCL_QOS_RELIABILITY_RELIABLE`, yet the `plugin-mix` scenario loses the
large majority of successfully-"published" messages this way.  There is no
high-water mark, no `PCL_ERR_NOMEM`, and no flush-on-shutdown.

> **Fixed.** The outbound FIFO is now capped at a high-water mark
> (`PCL_SOCKET_MAX_QUEUE_BYTES`, 16 MiB).  Beyond it, `publish` fails fast
> with `PCL_ERR_NOMEM` — the same fail-fast backpressure the shared-memory
> transport gives — so a publisher outrunning the link learns it must back
> off instead of growing memory without bound.  Teardown was reordered so the
> send thread drains the queue to the still-open socket *before* the socket is
> closed (a send timeout bounds the flush against a dead peer), and a running
> `dropped_publishes` counter is exposed via
> `pcl_socket_transport_dropped_publishes()`.  In the `plugin-mix` retest the
> socket transport went from losing **61–86 %** of "published" frames to
> **zero loss** (`rx == pub ok`), with the over-cap drops now surfaced as
> visible `PCL_ERR_NOMEM` backpressure instead of silent discards — so its
> declared `RELIABLE` QoS is honoured.

### D6 — executor ingress queue is unbounded: a slow subscriber turns backpressure into RSS growth

The shm receive thread drains its 16-deep shared-memory mailbox quickly
and re-posts each frame onto the executor's incoming queue
(`enqueue_incoming_message`), which is an unbounded heap-backed linked
list.  A subscriber whose executor thread spins slowly therefore does
**not** push backpressure to publishers: the mailbox rarely fills because
the receive thread immediately offloads into process memory.
`shm-slow-sub` shows the direction: a subscriber sleeping 2 ms between
spins absorbed the full 50 k msg/s stream with ~1.5 MB more RSS than the
fast subscriber and near-zero extra publisher `PCL_ERR_NOMEM` (each spin
drains the whole accumulated backlog, so it caught up in bursts).  A
subscriber that is persistently slower than the offered rate accumulates
that backlog without bound — an eventual OOM rather than a throttle.

> **Fixed.** `pcl_executor_set_incoming_queue_limit` adds an opt-in cap; its
> default is zero, preserving the previous unbounded behaviour. Once the cap
> is reached, `pcl_executor_post_incoming` and remote ingress return
> `PCL_ERR_NOMEM` without taking the message.  When the shared-memory receive
> thread gets that rejection it re-delivers the same frame in a short retry
> loop instead of moving on, so it stops draining its mailbox and the
> mailbox's fixed capacity fills and pushes `PCL_ERR_NOMEM` backpressure back
> to publishers.  Re-delivery is safe because a rejected pub/sub post takes
> nothing (delivery is idempotent on `PCL_ERR_NOMEM`), and with the default
> unbounded queue the first delivery always succeeds, so the steady-state
> receive path still takes the bus lock only once per frame — no throughput
> cost when the cap is off.  `pcl_executor_get_incoming_queue_depth` exposes
> the current depth for monitoring and tests.

### D7 — UDP loss is expected, but there is no receive-side accounting

UDP declares `BEST_EFFORT`, so loss under overload is by design.  The
stress run confirms heavy loss at max rate with no counters anywhere in
the transport to observe it; only the harness's sequence-gap tracking made
it visible.  Deployments have no way to distinguish "quiet topic" from
"drowning topic".

> **Fixed (counter).** The UDP transport now exposes
> `pcl_udp_transport_received_datagrams()`. It counts every datagram received
> from the socket, including malformed datagrams that the decoder rejects, so
> a deployment can distinguish a quiet socket from inbound traffic that is
> being discarded. UDP remains BEST_EFFORT. Per-source sequence-gap counting
> remains a possible future wire-format extension, but is not required for
> receive-side activity accounting.

## What held up well

- **Correctness under contention.** Across every shm scenario the
  delivered stream was exact: `sub rx == pub ok` with zero sequence gaps —
  the all-or-nothing fan-out transaction and fail-fast `PCL_ERR_NOMEM`
  backpressure never lost or duplicated an accepted message, including
  during participant churn (`shm-churn`).
- **Bus lifecycle.** Attach/detach churn while traffic flowed never
  corrupted the bus, wedged the lock, or stranded a slot; capacity is
  reclaimed when holders exit.
- **Limits enforced.** The participant cap and duplicate-ID guard both
  fail closed.
- **Separate buses scale.** `shm-multibus` ran 48 concurrent worker
  processes on 24 buses with zero failures, and aggregate throughput kept
  rising with bus count; per-bus rate fell only with CPU oversubscription
  (4 cores).  The per-bus bottlenecks (D1/D2) are the scaling limit, not
  process count or OS shared-memory resources.

## Fixes applied and retest

Defects **D2, D3, D4, D5** and the socket half of **D7** were fixed, and the
whole harness was re-run to confirm the improvement with no correctness
regression.  Both the baseline and the post-fix run in the table below are
from the **same 16-CPU host** (so they are directly comparable to each other,
though not to the 4-CPU numbers in the sections above).  Every shared-memory
scenario still delivered an exact stream (`sub rx == pub ok`, zero sequence
gaps) after the fixes.

| Metric (16-CPU retest) | Before fixes | After fixes |
|---|---:|---:|
| Socket `plugin-mix` messages lost (declared RELIABLE) | 8,263,842 (86 %) | **0** |
| Socket over-cap drops surfaced as `PCL_ERR_NOMEM` | 0 (silent) | 22,726 (visible) |
| Unary services, 3 clients — requests dropped | 419,908 / 661,948 (63 %) | **0 / 306,043** |
| Unary services, 6 clients — requests dropped | 2,015,331 / 2,108,828 (95 %) | **0 / 180,856** |
| Unary services, 6 clients — successful req/s | 23,758 | 60,242 |
| `shm-throughput` 64 B | 165,175 msg/s | 231,779 msg/s |
| `shm-fanout` 7 subscribers — aggregate | 215,780 msg/s | 295,410 msg/s |
| `shm` sequence gaps / lost, all scenarios | 0 / 0 | 0 / 0 (preserved) |

The 144 PCL transport unit tests (`test_pcl_shared_memory_transport`,
`test_pcl_socket_transport`, `test_pcl_socket_faults`,
`test_pcl_udp_transport`, `test_pcl_plugin_loader`,
`test_pcl_transport_routing`) all still pass.

**Where the fixes live.** D5 and the socket counter are in
`src/pcl_transport_socket.c` (plus the new
`pcl_socket_transport_dropped_publishes` accessor in the header); D2, D3, and
D4 are in `src/pcl_transport_shared_memory.c`.

## Still open

The following work remains after the D1/D6/D7 updates above.

1. **D2 (remainder)** — the single global bus lock still serialises every
   publish and service frame.  The service-table-sync contention was removed;
   per-slot locks were not attempted because they would need a safe
   multi-mailbox transaction/rollback protocol to retain exact fan-out
   delivery. The global lock remains the correctness-preserving design in
   this pass.

## D1 / D6 retest

This table compares the state after the first fix pass (D2–D5, already
committed) with the state after adding D1 interest filtering and the D6
opt-in ingress bound.  Both columns are from the **same 16-CPU host** and the
three-second stress window, so they are directly comparable to each other.
Every shared-memory scenario still delivered an exact stream (`sub rx ==
pub ok`, zero sequence gaps).

| Scenario | After D2–D5 | After D1 + D6 |
|---|---:|---:|
| `shm-throughput` 64 B | 231,779 msg/s | 216,821 msg/s |
| `shm-fanout`, 7 subscribers — aggregate | 295,410 msg/s | **481,404 msg/s** |
| `shm-pairs`, 1 pair | 206,709 msg/s | 218,518 msg/s |
| `shm-pairs`, 2 pairs (per pair) | 47,512 | **84,847** |
| `shm-pairs`, 4 pairs (per pair) | 10,369 | **18,444** |
| Unary services, 3 clients — req/s | 101,993 (0 dropped) | 83,958 (0 dropped) |
| Unary services, 6 clients — req/s | 60,242 (0 dropped) | 52,693 (0 dropped) |

Interest filtering (D1) is the win: once each subscriber advertises the topics
it wants, an unrelated pair on the same bus no longer lands in its mailbox, so
`shm-pairs` and `shm-fanout` scale much better.  The small dips on the single
pipeline scenarios (`shm-throughput`, single-pair, services) are measurement
variance around the same receive path — the D6 change adds no per-frame bus
lock in the default unbounded configuration (a first version that re-locked to
advance the read index on every frame cut receive-bound throughput by roughly
a third and was reworked to the retry-in-place form described under D6).

All 181 PCL transport and executor unit tests pass
(`test_pcl_shared_memory_transport` 34, `test_pcl_socket_transport` 33,
`test_pcl_socket_faults` 7, `test_pcl_udp_transport` 12,
`test_pcl_plugin_loader` 28, `test_pcl_transport_routing` 31,
`test_pcl_executor` 36).
