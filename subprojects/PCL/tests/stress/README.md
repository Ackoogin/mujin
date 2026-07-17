# PYRAMID port-system stress test

A multi-process stress harness for the PCL transport plugin system ("the
pyramid port system").  It launches many worker processes against the
runtime-loaded transport plugins — shared memory, TCP socket, and UDP — to
find scaling defects and to measure the maximum shared-memory plugin
throughput in messages per second.

The harness has two parts:

- **`pcl_port_stress_worker`** (built by CMake, not registered as a CTest
  test): one process playing a single role — publisher, subscriber, service
  provider, service client, or attach-and-hold.  The transport always comes
  in through `pcl_plugin_load_transport()` on a dlopen'd plugin `.so`, so
  the stress path is the production plugin path.
- **`run_port_stress.py`**: the driver.  It spawns workers, coordinates
  them with marker files (ready / start / stop), waits for them to exit,
  and aggregates the one-line JSON result every worker writes.

## Running

Build the workspace (PCL targets are enough), then:

```bash
python3 subprojects/PCL/tests/stress/run_port_stress.py \
    --build-dir build \
    --out-dir /tmp/port_stress_results
```

Useful flags:

- `--quick` — shorter runs and smaller matrices (a smoke pass).
- `--scenario NAME` (repeatable) — run only the named scenario(s).
- `--duration-ms N` — load-generation window per scenario (default 3000).

Raw per-worker JSON, per-worker logs, and a machine-readable
`summary.json` land in `--out-dir`.

## Scenarios

| Scenario | What it stresses |
|----------|------------------|
| `shm-throughput` | 1 publisher → 1 subscriber across payload sizes; the peak messages/s is the headline shared-memory throughput number. |
| `shm-fanout` | 1 publisher → N subscribers on one bus (publish fans out to every participant). |
| `shm-multi-pub` | N publishers → 1 subscriber (bus-lock contention, mailbox pressure). |
| `shm-pairs` | Independent pub/sub pairs sharing **one** bus; quantifies fan-out interference, because the bus copies every publish to every participant whether subscribed or not. |
| `shm-multibus` | Independent pub/sub pairs on **separate** buses; many simultaneous processes with no shared bus state. |
| `shm-slow-sub` | One fast and one deliberately slow subscriber on the same bus (head-of-line blocking and ingress-queue memory growth). |
| `shm-limit` | Attaches participants past the compiled-in bus capacity (`PCL_SHM_MAX_PARTICIPANTS`, 8) and checks the duplicate-participant-id guard. |
| `shm-services` | 1 unary service provider ← N clients through the shared-memory gateway. |
| `shm-churn` | Steady pub/sub traffic while other participants attach and detach repeatedly. |
| `plugin-mix` | Shared-memory, TCP socket, and UDP plugin workers all running at once. |

## Interpreting results

Each subscriber reports its receive count and a rate computed over its own
first→last receive window; publishers report per-status publish counts
(`ok`, `err_nomem` for fail-fast mailbox backpressure, …).  Subscribers
embed an 8-byte sequence number check (`gaps`) so silent loss is visible
even when counts look plausible.  Workers also report their final `VmRSS`
so unbounded queue growth shows up as memory.

Findings from the initial run of this harness are written up in
[`../../doc/reports/port_stress_findings.md`](../../doc/reports/port_stress_findings.md).
