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

### D2 — one global bus semaphore serialises all bus operations

Every publish, every service frame, and every iteration of every
participant's receive thread takes the single named semaphore
(`pcl_shm_bus_lock`).  The receive thread also re-syncs its service
advertisement table (another lock acquisition) on **every loop iteration**
(`pcl_shm_recv_thread_main` calls `pcl_shm_sync_local_services`
unconditionally).  Aggregate throughput therefore *drops* as participants
are added (`shm-multi-pub`, `shm-fanout`), instead of scaling until CPU
saturation.

### D3 — hard participant limit of 8, reported only as a generic attach failure

`PCL_SHM_MAX_PARTICIPANTS` is 8.  The 9th attach fails (correctly —
verified by `shm-limit`), but `pcl_shared_memory_transport_create` returns
plain `NULL` and the plugin entry returns `NULL`, so the loader reports the
same `PCL_ERR_NOT_FOUND` as any other entry failure.  A deployment
composing >8 components on one bus gets no diagnostic pointing at the
capacity limit.  Duplicate participant IDs are also rejected (correctly).

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

### D7 — UDP loss is expected, but there is no receive-side accounting

UDP declares `BEST_EFFORT`, so loss under overload is by design.  The
stress run confirms heavy loss at max rate with no counters anywhere in
the transport to observe it; only the harness's sequence-gap tracking made
it visible.  Deployments have no way to distinguish "quiet topic" from
"drowning topic".

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

## Suggested follow-ups (not implemented here)

1. Report a distinct status (or at least a log line) for "bus full" vs
   other attach failures (D3).
2. Deliver an error status to the client callback instead of an empty
   response when a SVC_REQ cannot be enqueued, and consider a bounded
   retry window on the egress worker like the publish backpressure path
   (D4).
3. Add a high-water mark to the socket egress FIFO (fail `publish` with
   `PCL_ERR_NOMEM` beyond it) and drain the queue before teardown closes
   the socket (D5).
4. Bound the executor incoming queue (fail the post, so the shm mailbox
   fills and publishers see backpressure) or make the bound configurable
   (D6).
5. Add drop/overflow counters to the UDP and socket transports (D7).
6. Consider per-topic interest filtering on the shm bus so fan-out only
   targets subscribed participants (D1), and move service-table sync off
   the receive hot loop (D2).
