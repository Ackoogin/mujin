#!/usr/bin/env python3
"""Multi-process stress driver for the PYRAMID port system (PCL transports).

Launches many pcl_port_stress_worker processes against the runtime-loaded
transport plugins (shared memory, TCP socket, UDP) and aggregates the JSON
result each worker writes.  The goal is to find scaling defects (contention,
head-of-line blocking, participant limits, message loss, memory growth) and
to measure the maximum shared-memory plugin throughput in messages/second.

Usage:
    python3 run_port_stress.py --build-dir <workspace build dir> \
        [--out-dir <results dir>] [--scenario NAME ...] [--quick]

Scenarios (default: all):
    shm-throughput    1 publisher -> 1 subscriber across payload sizes;
                      reports the peak messages/s (the headline number).
    shm-fanout        1 publisher -> N subscribers on one bus.
    shm-multi-pub     N publishers -> 1 subscriber on one bus.
    shm-pairs         Independent pub/sub pairs sharing ONE bus (fan-out
                      interference: every publish is copied to every
                      participant, subscribed or not).
    shm-multibus      Independent pub/sub pairs on SEPARATE buses -- many
                      processes, no shared bus state.
    shm-slow-sub      A fast and a deliberately slow subscriber on one bus
                      (head-of-line blocking + unbounded ingress queue).
    shm-limit         Attach participants past the compiled-in bus capacity
                      and report where and how attach fails.
    shm-services      1 service provider <- N clients (unary RPC rate).
    shm-churn         Publisher/subscriber traffic while other participants
                      attach and detach repeatedly.
    plugin-mix        shm + socket + udp workers running simultaneously.

Each scenario writes raw worker JSON under --out-dir and the driver prints
a summary table; a machine-readable summary lands in summary.json.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
import uuid

MAX_PARTICIPANTS = 8      # PCL_SHM_MAX_PARTICIPANTS in pcl_transport_shared_memory.c
QUEUE_DEPTH = 16          # PCL_SHM_QUEUE_DEPTH
MAX_PAYLOAD = 16384       # PCL_SHM_MAX_PAYLOAD


def ns_to_s(ns):
    return ns / 1e9


class Runner:
    def __init__(self, worker, plugins, out_dir, verbose=False):
        self.worker = worker
        self.plugins = plugins
        self.out_dir = out_dir
        self.verbose = verbose
        self.summary = {}

    def scenario_dir(self, name):
        d = os.path.join(self.out_dir, name)
        os.makedirs(d, exist_ok=True)
        return d

    def spawn(self, sdir, tag, mode, transport, extra):
        """Start one worker; returns (process, result_file, tag)."""
        result_file = os.path.join(sdir, tag + ".json")
        cmd = [
            self.worker,
            "--mode=" + mode,
            "--transport=" + transport,
            "--plugin=" + self.plugins[transport],
            "--participant=" + tag,
            "--result-file=" + result_file,
        ] + extra
        log = open(os.path.join(sdir, tag + ".log"), "w")
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        return (proc, result_file, tag)

    def wait_files(self, files, timeout_s=30):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(os.path.exists(f) for f in files):
                return True
            time.sleep(0.01)
        return False

    def collect(self, procs, timeout_s=90):
        """Wait for workers and parse their result JSON."""
        results = {}
        deadline = time.monotonic() + timeout_s
        for proc, result_file, tag in procs:
            budget = max(1, deadline - time.monotonic())
            try:
                proc.wait(timeout=budget)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                results[tag] = {"error": "timeout"}
                continue
            if os.path.exists(result_file):
                with open(result_file) as f:
                    data = json.load(f)
                data["exit_code"] = proc.returncode
                results[tag] = data
            else:
                results[tag] = {"error": "no result", "exit_code": proc.returncode}
        return results

    # -- shared-memory pub/sub scenarios ------------------------------------

    def run_pubsub(self, name, bus, n_pub, n_sub, size, duration_ms,
                   sub_spin_sleep_us=0, slow_subs=0, slow_spin_us=2000):
        """Generic N publishers -> M subscribers on one shm bus."""
        sdir = self.scenario_dir(name)
        start = os.path.join(sdir, "start")
        stop = os.path.join(sdir, "stop")
        procs = []
        ready = []

        for i in range(n_sub):
            tag = "sub%d" % i
            spin = slow_spin_us if i < slow_subs else sub_spin_sleep_us
            rf = os.path.join(sdir, tag + ".ready")
            ready.append(rf)
            procs.append(self.spawn(sdir, tag, "sub", "shm", [
                "--bus=" + bus, "--topic=t0",
                "--ready-file=" + rf, "--stop-file=" + stop,
                "--spin-sleep-us=%d" % spin,
                "--timeout-ms=%d" % (duration_ms + 20000),
            ]))
        for i in range(n_pub):
            tag = "pub%d" % i
            rf = os.path.join(sdir, tag + ".ready")
            ready.append(rf)
            procs.append(self.spawn(sdir, tag, "pub", "shm", [
                "--bus=" + bus, "--topic=t0",
                "--ready-file=" + rf, "--start-file=" + start,
                "--size=%d" % size, "--duration-ms=%d" % duration_ms,
            ]))

        if not self.wait_files(ready):
            print("  WARNING: not all workers became ready")
        open(start, "w").close()

        # Publishers exit on their own; then tell subscribers to drain.
        for proc, _, tag in procs:
            if tag.startswith("pub"):
                proc.wait(timeout=duration_ms / 1000 + 60)
        open(stop, "w").close()
        return self.collect(procs)

    @staticmethod
    def pub_totals(results):
        pubs = [r for r in results.values() if r.get("mode") == "pub"]
        return {
            "attempts": sum(r["attempts"] for r in pubs),
            "ok": sum(r["ok"] for r in pubs),
            "nomem": sum(r["err_nomem"] for r in pubs),
            "not_found": sum(r["err_not_found"] for r in pubs),
            "other": sum(r["err_other"] for r in pubs),
        }

    @staticmethod
    def sub_rate(r):
        """Messages/s over a subscriber's own first->last receive window."""
        if r.get("received", 0) < 2:
            return 0.0
        window = ns_to_s(r["last_rx_ns"] - r["first_rx_ns"])
        if window <= 0:
            return 0.0
        return (r["received"] - 1) / window

    # -- scenarios ----------------------------------------------------------

    def shm_throughput(self, duration_ms, sizes):
        rows = []
        for size in sizes:
            bus = "thr_%s_%d" % (uuid.uuid4().hex[:6], size)
            results = self.run_pubsub("shm-throughput-%d" % size, bus,
                                      1, 1, size, duration_ms)
            sub = results.get("sub0", {})
            pub = results.get("pub0", {})
            rate = self.sub_rate(sub)
            mbps = rate * size * 8 / 1e6
            rows.append({
                "payload_bytes": size,
                "published_ok": pub.get("ok", 0),
                "publish_nomem": pub.get("err_nomem", 0),
                "received": sub.get("received", 0),
                "gaps": sub.get("gaps", 0),
                "msgs_per_s": round(rate, 1),
                "mbit_per_s": round(mbps, 1),
                "lost": pub.get("ok", 0) - sub.get("received", 0),
            })
            print("  payload %6dB: %9.0f msg/s  %8.1f Mbit/s  "
                  "(pub ok=%d nomem=%d, sub rx=%d gaps=%d lost=%d)" % (
                      size, rate, mbps, pub.get("ok", 0),
                      pub.get("err_nomem", 0), sub.get("received", 0),
                      sub.get("gaps", 0),
                      pub.get("ok", 0) - sub.get("received", 0)))
        peak = max(rows, key=lambda r: r["msgs_per_s"]) if rows else None
        self.summary["shm-throughput"] = {"rows": rows, "peak": peak}

    def shm_fanout(self, duration_ms, size, sub_counts):
        rows = []
        for n_sub in sub_counts:
            bus = "fan_%s_%d" % (uuid.uuid4().hex[:6], n_sub)
            results = self.run_pubsub("shm-fanout-%d" % n_sub, bus,
                                      1, n_sub, size, duration_ms)
            pub = results.get("pub0", {})
            subs = [r for k, r in results.items() if k.startswith("sub")]
            rates = [self.sub_rate(r) for r in subs]
            rx = [r.get("received", 0) for r in subs]
            rows.append({
                "subscribers": n_sub,
                "published_ok": pub.get("ok", 0),
                "per_sub_rate": [round(x, 1) for x in rates],
                "per_sub_received": rx,
                "aggregate_msgs_per_s": round(sum(rates), 1),
                "publish_nomem": pub.get("err_nomem", 0),
            })
            print("  %d subs: pub ok=%d nomem=%d, per-sub rate=%s agg=%.0f" % (
                n_sub, pub.get("ok", 0), pub.get("err_nomem", 0),
                ["%.0f" % x for x in rates], sum(rates)))
        self.summary["shm-fanout"] = rows

    def shm_multi_pub(self, duration_ms, size, pub_counts):
        rows = []
        for n_pub in pub_counts:
            bus = "mp_%s_%d" % (uuid.uuid4().hex[:6], n_pub)
            results = self.run_pubsub("shm-multi-pub-%d" % n_pub, bus,
                                      n_pub, 1, size, duration_ms)
            totals = self.pub_totals(results)
            sub = results.get("sub0", {})
            rows.append({
                "publishers": n_pub,
                "published_ok": totals["ok"],
                "publish_nomem": totals["nomem"],
                "received": sub.get("received", 0),
                "sub_msgs_per_s": round(self.sub_rate(sub), 1),
            })
            print("  %d pubs: total ok=%d nomem=%d, sub rx=%d (%.0f msg/s)" % (
                n_pub, totals["ok"], totals["nomem"], sub.get("received", 0),
                self.sub_rate(sub)))
        self.summary["shm-multi-pub"] = rows

    def shm_pairs(self, duration_ms, size, pair_counts):
        """Independent pub->sub pairs, all sharing one bus.  Because shm
        publish fans out to every participant regardless of subscription,
        pairs on one bus interfere; this quantifies the cost."""
        rows = []
        for n_pairs in pair_counts:
            bus = "pair_%s_%d" % (uuid.uuid4().hex[:6], n_pairs)
            sdir = self.scenario_dir("shm-pairs-%d" % n_pairs)
            start = os.path.join(sdir, "start")
            stop = os.path.join(sdir, "stop")
            procs = []
            ready = []
            for i in range(n_pairs):
                stag = "sub%d" % i
                rf = os.path.join(sdir, stag + ".ready")
                ready.append(rf)
                procs.append(self.spawn(sdir, stag, "sub", "shm", [
                    "--bus=" + bus, "--topic=t%d" % i,
                    "--ready-file=" + rf, "--stop-file=" + stop,
                    "--timeout-ms=%d" % (duration_ms + 20000),
                ]))
                ptag = "pub%d" % i
                rf = os.path.join(sdir, ptag + ".ready")
                ready.append(rf)
                procs.append(self.spawn(sdir, ptag, "pub", "shm", [
                    "--bus=" + bus, "--topic=t%d" % i,
                    "--ready-file=" + rf, "--start-file=" + start,
                    "--size=%d" % size, "--duration-ms=%d" % duration_ms,
                ]))
            if not self.wait_files(ready):
                print("  WARNING: not all workers became ready")
            open(start, "w").close()
            for proc, _, tag in procs:
                if tag.startswith("pub"):
                    proc.wait(timeout=duration_ms / 1000 + 60)
            open(stop, "w").close()
            results = self.collect(procs)
            totals = self.pub_totals(results)
            subs = [r for k, r in results.items() if k.startswith("sub")]
            agg = sum(self.sub_rate(r) for r in subs)
            rows.append({
                "pairs": n_pairs,
                "published_ok": totals["ok"],
                "publish_nomem": totals["nomem"],
                "aggregate_msgs_per_s": round(agg, 1),
                "per_pair_msgs_per_s": round(agg / n_pairs, 1) if n_pairs else 0,
            })
            print("  %d pairs (1 bus): total ok=%d nomem=%d, agg=%.0f msg/s "
                  "(%.0f per pair)" % (n_pairs, totals["ok"], totals["nomem"],
                                       agg, agg / n_pairs if n_pairs else 0))
        self.summary["shm-pairs"] = rows

    def shm_multibus(self, duration_ms, size, bus_counts):
        rows = []
        for n_bus in bus_counts:
            sdir = self.scenario_dir("shm-multibus-%d" % n_bus)
            start = os.path.join(sdir, "start")
            stop = os.path.join(sdir, "stop")
            procs = []
            ready = []
            for i in range(n_bus):
                bus = "mb_%s_%d" % (uuid.uuid4().hex[:6], i)
                stag = "sub%d" % i
                rf = os.path.join(sdir, stag + ".ready")
                ready.append(rf)
                procs.append(self.spawn(sdir, stag, "sub", "shm", [
                    "--bus=" + bus, "--topic=t0",
                    "--ready-file=" + rf, "--stop-file=" + stop,
                    "--timeout-ms=%d" % (duration_ms + 30000),
                ]))
                ptag = "pub%d" % i
                rf = os.path.join(sdir, ptag + ".ready")
                ready.append(rf)
                procs.append(self.spawn(sdir, ptag, "pub", "shm", [
                    "--bus=" + bus, "--topic=t0",
                    "--ready-file=" + rf, "--start-file=" + start,
                    "--size=%d" % size, "--duration-ms=%d" % duration_ms,
                ]))
            if not self.wait_files(ready, timeout_s=60):
                print("  WARNING: not all workers became ready")
            open(start, "w").close()
            for proc, _, tag in procs:
                if tag.startswith("pub"):
                    proc.wait(timeout=duration_ms / 1000 + 90)
            open(stop, "w").close()
            results = self.collect(procs, timeout_s=120)
            totals = self.pub_totals(results)
            subs = [r for k, r in results.items() if k.startswith("sub")]
            agg = sum(self.sub_rate(r) for r in subs)
            failed = [k for k, r in results.items() if "error" in r
                      or r.get("exit_code", 0) != 0]
            rows.append({
                "buses": n_bus,
                "processes": 2 * n_bus,
                "published_ok": totals["ok"],
                "aggregate_msgs_per_s": round(agg, 1),
                "per_bus_msgs_per_s": round(agg / n_bus, 1) if n_bus else 0,
                "failed_workers": failed,
            })
            print("  %d buses (%d procs): total ok=%d, agg=%.0f msg/s "
                  "(%.0f per bus)%s" % (
                      n_bus, 2 * n_bus, totals["ok"], agg,
                      agg / n_bus if n_bus else 0,
                      "  FAILED: %s" % failed if failed else ""))
        self.summary["shm-multibus"] = rows

    def shm_slow_sub(self, duration_ms, size):
        bus = "slow_%s" % uuid.uuid4().hex[:6]
        results = self.run_pubsub("shm-slow-sub", bus, 1, 2, size,
                                  duration_ms, slow_subs=1, slow_spin_us=2000)
        pub = results.get("pub0", {})
        slow = results.get("sub0", {})   # sub0 is the slow one
        fast = results.get("sub1", {})
        row = {
            "published_ok": pub.get("ok", 0),
            "publish_nomem": pub.get("err_nomem", 0),
            "fast_sub_received": fast.get("received", 0),
            "fast_sub_rate": round(self.sub_rate(fast), 1),
            "slow_sub_received": slow.get("received", 0),
            "slow_sub_rate": round(self.sub_rate(slow), 1),
            "slow_sub_rss_kb": slow.get("rss_kb", 0),
            "fast_sub_rss_kb": fast.get("rss_kb", 0),
        }
        print("  pub ok=%d nomem=%d | fast rx=%d (%.0f/s, rss=%dK) | "
              "slow rx=%d (%.0f/s, rss=%dK)" % (
                  row["published_ok"], row["publish_nomem"],
                  row["fast_sub_received"], row["fast_sub_rate"],
                  row["fast_sub_rss_kb"], row["slow_sub_received"],
                  row["slow_sub_rate"], row["slow_sub_rss_kb"]))
        self.summary["shm-slow-sub"] = row

    def shm_limit(self, attempts=12):
        sdir = self.scenario_dir("shm-limit")
        bus = "lim_%s" % uuid.uuid4().hex[:6]
        procs = []
        for i in range(attempts):
            tag = "att%02d" % i
            procs.append(self.spawn(sdir, tag, "attach", "shm", [
                "--bus=" + bus, "--hold-ms=4000",
            ]))
            time.sleep(0.15)  # serialise attach order
        results = self.collect(procs)
        attached = sorted(k for k, r in results.items()
                          if r.get("mode") == "attach" and r.get("ok") == 1)
        rejected = sorted(k for k, r in results.items()
                          if r.get("ok") != 1)
        row = {"attempted": attempts, "attached": len(attached),
               "rejected": len(rejected), "rejected_tags": rejected}
        print("  attempted=%d attached=%d rejected=%d (capacity limit %d)" % (
            attempts, len(attached), len(rejected), MAX_PARTICIPANTS))
        # Duplicate-id probe: two workers with the same participant id.
        dup_dir = self.scenario_dir("shm-limit-dup")
        dbus = "dup_%s" % uuid.uuid4().hex[:6]
        p1 = self.spawn(dup_dir, "dupA", "attach", "shm",
                        ["--bus=" + dbus, "--hold-ms=2000"])
        time.sleep(0.3)
        # Same participant tag on purpose: result file must differ.
        result_file = os.path.join(dup_dir, "dupB.json")
        cmd = [self.worker, "--mode=attach", "--transport=shm",
               "--plugin=" + self.plugins["shm"], "--participant=dupA",
               "--result-file=" + result_file,
               "--bus=" + dbus, "--hold-ms=500"]
        log = open(os.path.join(dup_dir, "dupB.log"), "w")
        p2 = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        dup_results = self.collect([p1, (p2, result_file, "dupB")])
        row["duplicate_id_rejected"] = dup_results.get("dupB", {}).get("ok") != 1
        print("  duplicate participant id rejected: %s" %
              row["duplicate_id_rejected"])
        self.summary["shm-limit"] = row

    def shm_services(self, duration_ms, client_counts):
        rows = []
        for n_cli in client_counts:
            sdir = self.scenario_dir("shm-services-%d" % n_cli)
            bus = "svc_%s_%d" % (uuid.uuid4().hex[:6], n_cli)
            start = os.path.join(sdir, "start")
            stop = os.path.join(sdir, "stop")
            procs = []
            rf = os.path.join(sdir, "svc.ready")
            procs.append(self.spawn(sdir, "svc", "svc", "shm", [
                "--bus=" + bus, "--service=echo",
                "--ready-file=" + rf, "--stop-file=" + stop,
                "--timeout-ms=%d" % (duration_ms + 20000),
            ]))
            ready = [rf]
            for i in range(n_cli):
                tag = "cli%d" % i
                rf = os.path.join(sdir, tag + ".ready")
                ready.append(rf)
                procs.append(self.spawn(sdir, tag, "client", "shm", [
                    "--bus=" + bus, "--service=echo",
                    "--ready-file=" + rf, "--start-file=" + start,
                    "--size=64", "--duration-ms=%d" % duration_ms,
                    "--inflight=8",
                ]))
            if not self.wait_files(ready):
                print("  WARNING: not all workers became ready")
            open(start, "w").close()
            for proc, _, tag in procs:
                if tag.startswith("cli"):
                    proc.wait(timeout=duration_ms / 1000 + 60)
            open(stop, "w").close()
            results = self.collect(procs)
            svc = results.get("svc", {})
            clis = [r for k, r in results.items() if k.startswith("cli")]
            sent = sum(r.get("attempts", 0) for r in clis)
            done = sum(r.get("ok", 0) for r in clis)
            window = max((r.get("active_ns", 0) for r in clis), default=0)
            rate = done / ns_to_s(window) if window else 0.0
            rows.append({
                "clients": n_cli,
                "requests_sent": sent,
                "responses_ok": done,
                "svc_invocations": svc.get("received", 0),
                "req_per_s": round(rate, 1),
            })
            print("  %d clients: sent=%d ok=%d svc_saw=%d -> %.0f req/s" % (
                n_cli, sent, done, svc.get("received", 0), rate))
        self.summary["shm-services"] = rows

    def shm_churn(self, duration_ms, size, churners=3):
        sdir = self.scenario_dir("shm-churn")
        bus = "churn_%s" % uuid.uuid4().hex[:6]
        start = os.path.join(sdir, "start")
        stop = os.path.join(sdir, "stop")
        procs = []
        ready = []
        rf = os.path.join(sdir, "sub0.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "sub0", "sub", "shm", [
            "--bus=" + bus, "--topic=t0",
            "--ready-file=" + rf, "--stop-file=" + stop,
            "--timeout-ms=%d" % (duration_ms + 20000),
        ]))
        rf = os.path.join(sdir, "pub0.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "pub0", "pub", "shm", [
            "--bus=" + bus, "--topic=t0",
            "--ready-file=" + rf, "--start-file=" + start,
            "--size=%d" % size, "--duration-ms=%d" % duration_ms,
        ]))
        if not self.wait_files(ready):
            print("  WARNING: pub/sub not ready")
        open(start, "w").close()

        # Attach/detach churners while traffic flows.
        churn_ok = 0
        churn_fail = 0
        t_end = time.monotonic() + duration_ms / 1000
        k = 0
        while time.monotonic() < t_end:
            wave = []
            for c in range(churners):
                tag = "churn%d_%d" % (k, c)
                wave.append(self.spawn(sdir, tag, "attach", "shm", [
                    "--bus=" + bus, "--hold-ms=120",
                ]))
            wave_results = self.collect(wave, timeout_s=30)
            for r in wave_results.values():
                if r.get("ok") == 1:
                    churn_ok += 1
                else:
                    churn_fail += 1
            k += 1

        for proc, _, tag in procs:
            if tag.startswith("pub"):
                proc.wait(timeout=duration_ms / 1000 + 60)
        open(stop, "w").close()
        results = self.collect(procs)
        pub = results.get("pub0", {})
        sub = results.get("sub0", {})
        row = {
            "churn_attach_ok": churn_ok,
            "churn_attach_fail": churn_fail,
            "published_ok": pub.get("ok", 0),
            "publish_nomem": pub.get("err_nomem", 0),
            "publish_other": pub.get("err_other", 0),
            "received": sub.get("received", 0),
            "gaps": sub.get("gaps", 0),
            "lost": pub.get("ok", 0) - sub.get("received", 0),
            "sub_msgs_per_s": round(self.sub_rate(sub), 1),
        }
        print("  churn ok=%d fail=%d | pub ok=%d nomem=%d other=%d | "
              "sub rx=%d gaps=%d lost=%d (%.0f msg/s)" % (
                  churn_ok, churn_fail, row["published_ok"],
                  row["publish_nomem"], row["publish_other"],
                  row["received"], row["gaps"], row["lost"],
                  row["sub_msgs_per_s"]))
        self.summary["shm-churn"] = row

    def plugin_mix(self, duration_ms, size):
        sdir = self.scenario_dir("plugin-mix")
        start = os.path.join(sdir, "start")
        stop = os.path.join(sdir, "stop")
        procs = []
        ready = []

        # shm pair
        bus = "mix_%s" % uuid.uuid4().hex[:6]
        rf = os.path.join(sdir, "shm_sub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "shm_sub", "sub", "shm", [
            "--bus=" + bus, "--topic=t0", "--ready-file=" + rf,
            "--stop-file=" + stop, "--timeout-ms=%d" % (duration_ms + 20000),
        ]))
        rf = os.path.join(sdir, "shm_pub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "shm_pub", "pub", "shm", [
            "--bus=" + bus, "--topic=t0", "--ready-file=" + rf,
            "--start-file=" + start, "--size=%d" % size,
            "--duration-ms=%d" % duration_ms,
        ]))

        # socket pair (server hosts the subscriber; client publishes to it)
        rf = os.path.join(sdir, "sock_sub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "sock_sub", "sub", "socket", [
            "--role=server", "--port=47401", "--topic=t0",
            "--ready-file=" + rf, "--stop-file=" + stop,
            "--timeout-ms=%d" % (duration_ms + 20000),
        ]))
        time.sleep(0.3)  # let the server bind before the client connects
        rf = os.path.join(sdir, "sock_pub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "sock_pub", "pub", "socket", [
            "--role=client", "--host=127.0.0.1", "--port=47401", "--topic=t0",
            "--ready-file=" + rf, "--start-file=" + start,
            "--size=%d" % size, "--duration-ms=%d" % duration_ms,
        ]))

        # udp pair (symmetric endpoints)
        rf = os.path.join(sdir, "udp_sub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "udp_sub", "sub", "udp", [
            "--local-port=47402", "--remote-host=127.0.0.1",
            "--remote-port=47403", "--topic=t0",
            "--ready-file=" + rf, "--stop-file=" + stop,
            "--timeout-ms=%d" % (duration_ms + 20000),
        ]))
        rf = os.path.join(sdir, "udp_pub.ready")
        ready.append(rf)
        procs.append(self.spawn(sdir, "udp_pub", "pub", "udp", [
            "--local-port=47403", "--remote-host=127.0.0.1",
            "--remote-port=47402", "--topic=t0",
            "--ready-file=" + rf, "--start-file=" + start,
            "--size=%d" % size, "--duration-ms=%d" % duration_ms,
        ]))

        if not self.wait_files(ready):
            print("  WARNING: not all workers became ready")
        open(start, "w").close()
        for proc, _, tag in procs:
            if tag.endswith("pub"):
                proc.wait(timeout=duration_ms / 1000 + 60)
        open(stop, "w").close()
        results = self.collect(procs)

        row = {}
        for kind in ("shm", "sock", "udp"):
            pub = results.get(kind + "_pub", {})
            sub = results.get(kind + "_sub", {})
            rate = self.sub_rate(sub)
            row[kind] = {
                "published_ok": pub.get("ok", 0),
                "publish_errors": pub.get("err_nomem", 0)
                + pub.get("err_not_found", 0) + pub.get("err_other", 0),
                "received": sub.get("received", 0),
                "msgs_per_s": round(rate, 1),
                "lost": pub.get("ok", 0) - sub.get("received", 0),
            }
            print("  %-6s pub ok=%-8d errs=%-6d rx=%-8d rate=%9.0f/s lost=%d" % (
                kind, row[kind]["published_ok"], row[kind]["publish_errors"],
                row[kind]["received"], rate, row[kind]["lost"]))
        self.summary["plugin-mix"] = row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--build-dir", required=True,
                    help="workspace build dir containing subprojects/PCL")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--scenario", action="append", default=None,
                    help="run only the named scenario(s)")
    ap.add_argument("--duration-ms", type=int, default=3000)
    ap.add_argument("--quick", action="store_true",
                    help="shorter runs, smaller matrices")
    args = ap.parse_args()

    build = os.path.abspath(args.build_dir)
    pcl = os.path.join(build, "subprojects", "PCL")
    worker = os.path.join(pcl, "tests", "stress", "pcl_port_stress_worker")
    plugins = {
        "shm": os.path.join(pcl, "src", "libpcl_transport_shared_memory_plugin.so"),
        "socket": os.path.join(pcl, "src", "libpcl_transport_socket_plugin.so"),
        "udp": os.path.join(pcl, "src", "libpcl_transport_udp_plugin.so"),
    }
    for path in [worker] + list(plugins.values()):
        if not os.path.exists(path):
            sys.exit("missing binary: %s (build the workspace first)" % path)

    out_dir = args.out_dir or os.path.join(
        os.getcwd(), "port_stress_results_%s" % time.strftime("%Y%m%d_%H%M%S"))
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)

    dur = 1500 if args.quick else args.duration_ms
    runner = Runner(worker, plugins, out_dir)

    all_scenarios = ["shm-throughput", "shm-fanout", "shm-multi-pub",
                     "shm-pairs", "shm-multibus", "shm-slow-sub",
                     "shm-limit", "shm-services", "shm-churn", "plugin-mix"]
    chosen = args.scenario or all_scenarios

    t0 = time.monotonic()
    for name in chosen:
        print("== %s ==" % name)
        if name == "shm-throughput":
            sizes = [64, 4096] if args.quick else [64, 1024, 4096, 16384]
            runner.shm_throughput(dur, sizes)
        elif name == "shm-fanout":
            counts = [3] if args.quick else [1, 3, 7]
            runner.shm_fanout(dur, 64, counts)
        elif name == "shm-multi-pub":
            counts = [3] if args.quick else [1, 3, 7]
            runner.shm_multi_pub(dur, 64, counts)
        elif name == "shm-pairs":
            counts = [2] if args.quick else [1, 2, 4]
            runner.shm_pairs(dur, 64, counts)
        elif name == "shm-multibus":
            counts = [8] if args.quick else [4, 8, 16, 24]
            runner.shm_multibus(dur, 64, counts)
        elif name == "shm-slow-sub":
            runner.shm_slow_sub(dur, 64)
        elif name == "shm-limit":
            runner.shm_limit()
        elif name == "shm-services":
            counts = [3] if args.quick else [1, 3, 6]
            runner.shm_services(dur, counts)
        elif name == "shm-churn":
            runner.shm_churn(dur, 64)
        elif name == "plugin-mix":
            runner.plugin_mix(dur, 64)
        else:
            print("  unknown scenario, skipping")

    elapsed = time.monotonic() - t0
    runner.summary["_meta"] = {
        "duration_ms_per_scenario": dur,
        "wall_time_s": round(elapsed, 1),
        "worker": worker,
        "plugins": plugins,
    }
    summary_path = os.path.join(out_dir, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(runner.summary, f, indent=2)
    print("\nsummary written to %s (%.0fs total)" % (summary_path, elapsed))


if __name__ == "__main__":
    main()
