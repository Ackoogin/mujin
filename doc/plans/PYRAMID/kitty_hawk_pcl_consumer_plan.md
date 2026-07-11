# Kitty Hawk PCL Consumer Proof — PYRAMID-only Successor to LA-CAL Phase 6

**Status:** proposed, not yet scheduled.
**Date:** 2026-07-11.
**Supersedes:** the AME-facing scope of
[`la_cal_integration_plan.md`](la_cal_integration_plan.md) Phase 6
("stretch, AME-facing — MS-leg demo"). That phase named `StateUpdate` →
WorldModel fact grounding and a truth-vs-perceived evaluator against the
Kitty Hawk DIS channel as the payoff. **AME now lives in a separate repo**,
so that work — and any WorldModel-specific design — is out of scope here.
This plan captures what remains a legitimate PYRAMID/PCL deliverable once
the AME-side consumer is subtracted out.
**Design source:** [`ams_gra_starter_kit_review.md`](../../research/AME/ams_gra_starter_kit_review.md)
§5.2 (for context on what the original demo intended);
[`la_cal_integration_plan.md`](la_cal_integration_plan.md) Phases 1–5 (the
transport plugin, codec, and interaction-facade machinery this plan reuses
unchanged);
[`ams_gra_starter_kit_bringup.md`](../../../subprojects/PYRAMID/doc/guides/ams_gra_starter_kit_bringup.md)
(the live Kitty Hawk stack this plan targets — already stood up).

---

## 1. Goal and non-goals

**Goal.** Prove the LA-CAL transport plugin and OMS JSON codec against the
**full, live Kitty Hawk simulation** — not just the pinned `infra/sleet`
container run standalone (Phase 3) or the synthetic `la-cal-harness`
generator (Phase 4) — as a PCL-only harness with no AME/WorldModel
dependency. Concretely: a PCL process registers with the running Sleet,
subscribes the UCI messages genuinely produced by Squall's RF/IR sensor
simulation and the `rf-fm-demod`/`ir-search-and-track` Skills (independent,
non-PYRAMID-authored binaries, driving real physics via JSBSim and a real
RF/IR propagation model — not a scripted fixture), and typed-decodes them
with the existing frozen-C-struct codec layer.

This is a stronger interop claim than Phase 4's: Phase 4 proved interop
against `la-cal-harness`'s XSD-derived *generator*, a test tool. This plan
proves interop against the Skills that are themselves the intended
production-shaped OMS Services in the AMS GRA ecosystem, fed by a physics
and sensor simulation with genuine loss-of-track dynamics (occlusion on the
pentagon flight pattern, RF range/lock loss) — so decode fidelity has to
hold up against real jitter and edge cases (e.g. `ObservationMeasurement`
values swinging through the full az/el range, `PositionReport` updating at
sensor-sim rate), not just a generator's fixed sample set.

**Non-goals (now out of scope for this repo):**

- `StateUpdate` / WorldModel fact grounding of any kind — AME lives
  elsewhere now; if that repo wants this data, it consumes the LA-CAL
  transport plugin and generated bindings this repo ships, same as any
  other PYRAMID consumer.
- Truth-vs-perceived evaluation against Supercell's DIS `EntityStatePdu`
  broadcast. DIS ingestion is a truth/perception concept, not a PCL
  transport/codec concern; it stays with whichever repo does the
  evaluation.
- Any Worldview/Graupel visualization integration — those are the kit's own
  tools for a human to eyeball truth-vs-perceived; nothing here drives them.
- Standing up the stack itself — done; see the bring-up guide referenced
  above. This plan assumes a running Kitty Hawk stack (persistent local
  checkout under `external/ams-gra/`, or CI-equivalent later) and focuses
  on the PCL-side proof against it.

---

## 2. What's missing to do this today

1. **`ObservationMeasurementReport` has no codec.** The hand-written
   `pyramid_oms_json_codec_uci.cpp` (frozen `pyramid_<T>_c` structs, ABI v2)
   currently covers exactly four UCI messages:
   `SignalReport`, `PositionReport`, `ActionCommand`, `ActionCommandStatus`
   (`OmsJsonUciCodec` 8/8 = 4 messages × encode/decode). `ir-search-and-track`
   publishes `ObservationMeasurementReport` (LOS Az/El measurements,
   `ReferenceKinematics` carrying the cached ownship `PositionReport`),
   confirmed live this session (see `FINDINGS.md` "Phase 6 progress"). This
   needs a fifth message added to the codec, same pattern as the existing
   four: frozen C struct, encode/decode golden tests against the captured
   live payload shape.
2. **No Sleet registration for a PCL-side subscriber against this stack.**
   The `ame-sniffer` scratch registration (Python, read-only, this session)
   proved the topics are live; a PCL harness needs its own `service_id` in
   `sleet/services.d.local/` (same override mechanism documented in the
   bring-up guide).
3. **`SignalReport` intermittency is scenario-real, not a fixture.** Unlike
   Phase 3/4's fixtures, this harness cannot assume a `SignalReport` shows
   up in any fixed window — the pentagon pattern genuinely gains/loses RF
   lock on the FM tower. The harness needs to either run long enough to
   observe at least one, or treat "zero SignalReports in N seconds" as
   inconclusive-but-not-failing for that message type specifically (distinct
   from Phase 3/4's fail-closed-on-silence contract, which assumes a
   scripted, deterministic publisher).

## 3. Deliverables

| # | Artifact | Location | New/extends |
|---|----------|----------|--------------|
| 1 | `ObservationMeasurementReport` OMS JSON codec entry | `subprojects/PYRAMID/plugins/pyramid_oms_json_codec_uci.cpp` (+ frozen struct header) | extends |
| 2 | Golden fixture tests from the live-captured payload shape | `subprojects/PYRAMID/tests/test_oms_json_codec_uci.cpp` | extends |
| 3 | Kitty Hawk consumer harness (PCL process, no AME dep) | `pim/test_harness/lacal/kittyhawk_consumer_test.cpp` + `build_kittyhawk_consumer_test.sh` | new, follows `lacal_e2e_test`/`lacal_interop_driver.py` conventions |
| 4 | Sleet registration for the harness | `external/ams-gra/sleet/services.d.local/kittyhawk-consumer.toml` (local, git-ignored — see bring-up guide) | new |
| 5 | FINDINGS.md exit entry | `subprojects/PYRAMID/pim/test_harness/FINDINGS.md` | doc |

## 4. Harness design

1. Register `kittyhawk-consumer` with the running Sleet (`allowed_topics`:
   `mission.position-report`, `mission.signal-report`,
   `mission.observation-measurement-report`, `mission.service-status`).
2. Connect via the existing `pyramid_lacal_transport_plugin` (unchanged from
   Phases 2–5) and the extended OMS JSON codec (deliverable #1).
3. Subscribe all four topics; typed-decode each received message into its
   frozen `pyramid_<T>_c` struct.
4. Assertions:
   - `PositionReport` and `ObservationMeasurementReport`: decode without
     error, required fields non-null/finite, latitude/longitude in valid
     WGS84 ranges, at least N samples received within a bounded timeout
     (these are continuous per the scenario design — a real fail-closed
     check).
   - `ServiceStatus`: at least one `NORMAL` from each of `rf-fm-demod` and
     `ir-search-and-track` within a bounded timeout (real fail-closed
     check).
   - `SignalReport`: decode without error **if** any arrive within a longer
     bounded window; zero arrivals logs a clear "inconclusive — no RF lock
     observed in this run" rather than failing the harness (per §2 point 3).
5. SKIP-safe like the existing `lacal_*` harnesses: SKIP with a printed
   reason when the Kitty Hawk stack isn't reachable (env var, e.g.
   `KITTYHAWK_SLEET_URL`), FAIL on a reachable-but-rejecting/malformed-payload
   Sleet.

## 5. Test and harness conventions

Same as `la_cal_integration_plan.md` §5: lives under `pim/test_harness/`,
`build_kittyhawk_consumer_test.sh` + SKIP-safe behaviour, not CTest-registered
(needs the live multi-container stack, not just Sleet), captured PASS output
committed as a `.log` fixture, one dated FINDINGS.md section on exit.

## 6. Exit gate

Codec covers all four UCI messages genuinely observed on the wire from
independent AMS GRA producers; the harness decodes a real, multi-minute
run of live Kitty Hawk traffic (not a fixture) end to end with no decode
errors; `SignalReport` is handled as scenario-intermittent rather than
asserted present. FINDINGS.md gains the exit entry.

## 7. Cross-repo note

Whatever repo AME now lives in, if it wants this data it consumes: the
`pyramid_lacal_transport_plugin` (Phase 2), the OMS JSON UCI codec extended
by this plan (deliverable #1), and generated Request-port facades where
applicable (Phase 5) — the same way any other PYRAMID consumer would. No
WorldModel-specific hooks belong in this repo; this plan's harness is
intentionally a plain PCL consumer, not a stub for AME to inherit from.
