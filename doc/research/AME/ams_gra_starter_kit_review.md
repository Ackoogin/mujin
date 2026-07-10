# AMS GRA Hello World Starter Kit Review

**Scope:** Review of the `docs/` tree of
[github.com/open-arsenal/ams-gra-hello-world-sk-getting-started](https://github.com/open-arsenal/ams-gra-hello-world-sk-getting-started)
as *additional context* for the A-GRA integration review: what **AMS GRA**
(Agile Mission Suite Government Reference Architecture) is, how it relates to
**A-GRA** (Autonomy GRA), what the starter kit's runnable environment offers
our adoption path, and how its interfaces map onto PCL/PYRAMID.

**Date:** 2026-07-08
**Companion to:**
[`a_gra_standard_review.md`](a_gra_standard_review.md) (esp. §6.3 wire
stack, §7 adoption path, §8 risks) and
[`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md).
**Related:**
[`pubsub_contract_generation_plan.md`](../../plans/PYRAMID/README.md),
[`agra_pubsub_shm_udp_proving_plan.md`](../../plans/PYRAMID/README.md),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md).
**Inputs reviewed:** repo `README.md`; `docs/architecture.md`,
`contracts.md`, `decisions.md`, `maintainer-guide.md`, `ports.md`,
`visualization.md`; `docs/tutorials/0-overview.md` through
`4-swapping-skills.md`. Fetched 2026-07-08 from the GitHub mirror at `main`.
Note the mirror caveat in §7: GitLab is the primary forge, so the mirror may
lag; findings here should be re-checked against the GitLab origin before any
implementation work keys off exact upstream contract text.

---

## 1. Executive summary

1. **AMS GRA is not A-GRA.** Same `open-arsenal` publisher, same
   UCI/OMS ecosystem, different seam: AMS GRA standardises the
   **sensor/mission-equipment side** (Multi-Function Apertures, the MEL raw
   data interface, "Skill" data processors, MASI shared infrastructure),
   where A-GRA standardises the **mission-autonomy (MA) boundary** (C2, VI,
   MS, MP, MD, P2P). They meet on the onboard bus: A-GRA's MS/VI interfaces
   are OMS/CAL over an Abstract Service Bus (review §6.3), and AMS GRA
   Skills publish UCI products onto exactly that bus. An A-GRA MA and an
   AMS GRA Skill are peers on the same ASB, producer/consumer of the same
   UCI vocabulary.

2. **The chief value to us is a public, runnable OMS/CAL/UCI environment.**
   The a-gra review flagged "OMS/CAL access may be controlled" as an
   external-artefact risk (§8). The starter kit contains **Sleet**, an open
   LA-CAL (Language-Agnostic CAL) server speaking **WebSocket + JSON** with
   an OMS WebSocket Protocol (OWP) init/subscribe/publish flow — a de facto
   accessible OMS/CAL environment, plus a full simulation harness around it
   (DIS ground truth, simulated RF/IR sensors, truth-vs-perceived
   visualization). That substantially de-risks *prototyping* the onboard
   MS-interface leg of the A-GRA plan; it does **not** discharge compliance
   (§5.3).

3. **AMS GRA's dual-plane split independently validates the PCL/PYRAMID
   transport architecture.** MEL/ABB (strict C++ API, vendor `.so`
   plugins, VITA 49.2 over UDP, "2 Msps I/Q, 4K@60 FPS" workloads) is a
   high-bandwidth data plane deliberately separated from CAL/ASB (message
   plane). That is the same split as PCL stream/SHM/UDP transports vs
   contract-derived pub/sub, and it hands the SHM/UDP proving plan concrete,
   externally-sourced load targets and a genuinely BEST_EFFORT use case
   (§4.2, §5.4).

4. **Several small, adoptable specifics:** a deterministic
   UUIDv5-from-names identity scheme for system/subsystem/message IDs; a
   workspace port registry with bidirectional manifest validation;
   snapshot (not incremental) semantics for activity messages; and the
   truth-vs-perceived overlay as an E2E evaluation pattern for sensor
   chains (§4, §6).

5. **Limits:** the kit is self-described "early development" and not yet
   fully AMS GRA compliant; it exercises **no EXI, no DDS/DMS** — so it is a
   harness for the *onboard/MS* leg only, not the offboard C2/Peer wire
   stack; and it speaks sensor-domain UCI (signal/track reports,
   activities), not the `MA_*` planning set, so it complements rather than
   replaces the stub C2 client in adoption-path Phase 2 (§5.2).

---

## 2. What AMS GRA is, and how it relates to A-GRA

### 2.1 Terminology (as defined by the starter kit docs)

| Term | Expansion | Meaning |
|------|-----------|---------|
| AMS GRA | Agile Mission Suite Government Reference Architecture | Architecture volumes + compliance material for modular mission suites |
| OMS | Open Mission Systems | Government-owned open architecture for integrating modular components ("Units of Replaceability") |
| UCI | Universal Command and Control Interface | Government-owned M2M C2 messaging standard (same UCI family A-GRA extends) |
| CAL | Critical Abstraction Layer | Required API boundary insulating OMS Services from transport specifics |
| LA-CAL | Language-Agnostic CAL | CAL implementation pattern over WebSockets + JSON |
| ASB | Abstract Service Bus | Logical messaging network carrying UCI messages between services |
| MFA | Multi-Function Aperture | Sensor/payload hardware at the free-space boundary producing raw digital data |
| MEL | see note | Standardized strict **C++** interface between MFAs and data processors |
| ABB | Avionics Backplane Bus | High-speed logical network carrying minimally-processed aperture data |
| Skill | AMS GRA Skill / Data Processor | Domain algorithm binding to MEL for raw ingest, publishing UCI via CAL |
| MASI | Mission Agnostic Service Infrastructure | Shared routing/health/lifecycle infrastructure |

Note on MEL: the upstream docs expand it inconsistently — tutorial 0 says
"Mission Equipment Location Interface", tutorials 2–3 say "MFA
Encapsulation Layer". The technical content is consistent (strict C++ IDD
between MFA and Skill); worth confirming the official expansion against the
AMS GRA volumes proper before we cite it anywhere normative.

### 2.2 Positioning against A-GRA

| Aspect | A-GRA (reviewed 2026-07-02) | AMS GRA (this kit) |
|--------|------------------------------|--------------------|
| Seam | MA ↔ C2/vehicle/mission-systems/peers | MFA/sensors ↔ data processors ↔ OMS services |
| Primary artefact | UCI 2.3 XSD + `MA_*` extensions + interaction sequences | Architecture volumes + MEL C++ IDDs + OMS service contracts |
| Onboard bus | OMS/CAL over ASB (MS/VI interfaces) | OMS/CAL over ASB (Skill egress, service comms) |
| Offboard | EXI over DDS-based DMS | Not in scope of the starter kit |
| High-rate data | Out of scope (MA consumes reports) | MEL over ABB (VITA 49.2 over UDP in practice) |
| Swappable unit | The MA itself | Skills (data processors) and MFAs |

The complementarity matters for the AME story: A-GRA's MS volume has the MA
*consuming* sensor products (tracks, signal reports) from mission systems —
and AMS GRA is precisely the architecture on the *producing* side of those
products. In the a-gra review's integration shape (§5.2), the "MS-volume
track messages → `StateUpdate` → WorldModel" ingress arrow terminates, on
the platform side, in an AMS GRA Skill chain: MFA → MEL → Skill → UCI on
the ASB. The two standards compose into one end-to-end pipeline with AME's
WorldModel as the downstream consumer.

### 2.3 The starter kit's runnable environment

Hardware-free `podman-compose` deployment (Kitty Hawk scenario: one blue
ownship with RF + IR sensors, two bandits, an FM radio tower, a pentagon
flight pattern designed so sensors genuinely lose lock/range):

| Component | Role | AMS GRA/OMS concept |
|-----------|------|---------------------|
| Supercell | Environment sim: JSBSim flight dynamics, waypoints, emitters; publishes DIS ground truth | Physical truth source (DIS, ECEF) |
| Squall | Unified RF + IR propagation model synthesising raw sensor streams | The **MFA** (canonical MEL provider, both RF and IR) |
| Sleet | LA-CAL server: WebSocket + OWP routing of UCI messages, `SIMULATION` mode | **MASI / CAL / ASB** |
| rf-fm-demod | FM demodulation of raw I/Q, emits UCI signal reports | Reference **Skill** (RF) |
| ir-search-and-track | IR frame processing → tracks | Reference **Skill** (IR) |
| Graupel | Ingests DIS (truth) + UCI (perceived), normalises to CZML | Evaluation/observability layer |
| Worldview | CesiumJS web client with independent truth/perception toggles | Visualization |

A hardware-in-the-loop mode exists (HackRF transmit, RTL-SDR/HackRF
receive), so the same Skill code can front real RF hardware.

Operationally notable constraints the docs call out:

- **Coordinate frames:** DIS is ECEF + Euler angles; UCI is WGS84 geodetic
  with NED-relative attitude; they require "centralized ECEF ↔ WGS84
  conversion algorithms".
- **Snapshot semantics:** `ESM_Activity` and `PO_Activity` are "complete
  snapshots, not incremental updates".
- **Registration:** every Sleet-connected service needs explicit `.toml`
  registration (credentials + topic bindings); unregistered services face
  **silent rejection**.
- **Identity:** deterministic UUIDv5 derivation —
  `SystemID = UUIDv5(Realm, system_name)`,
  `SubsystemID = UUIDv5(Realm, subsystem_name)`,
  `MessageID = UUIDv5(SubsystemID, "<event_type>:<timestamp_or_seq>")`,
  with the realm from a `NAMESPACE_UUID` environment variable and a legacy
  direct-UUID override. Explicit goal: "deterministic correlation across
  the OMS/UCI C2 network without configuration drift".

---

## 3. The MEL: AMS GRA's data plane, read against PCL

The MEL is the most architecturally interesting artefact for us, because it
is the ecosystem's answer to the same problem PCL's stream/SHM/UDP work
addresses: CAL-style messaging "cannot support high-performance streaming
scenarios" ("raw Radio Frequency (I/Q) bytes at 2 Msps, or uncompressed 4K
Optical (IR) frames at 60 FPS").

Mechanics (tutorials 2–3):

- The MEL is a **strict C++ API**, published as government-owned IDDs
  (`interfaces/rf-mel`, `interfaces/ir-mel`) — deliberately *not* a
  language-agnostic protocol. It is defined **only from the Skill's
  perspective**; there is no codified "MFA side" interface.
- Vendors ship a shared object (`libmy_rf_mel.so`) exporting standard MEL
  factory symbols (`createC2MEL(...)` etc.). The Skill dynamically loads
  it; vendor code inside the process translates the standardized C++ calls
  into whatever proprietary transport (network, PCIe, serial) commands the
  hardware. The IDD "does not mandate specific network transports".
- The API splits into a **C2MEL** (control: request Virtual Apertures and
  Jobs) and a **DataMEL** (data: create a Product Rx Endpoint with a data
  format and buffer sizing, register async callbacks).
- The reference data plane packetizes with **VITA 49.2** and ships over
  **UDP**.
- A Skill is **dual-interface** by construction: MEL (raw ingress) + CAL
  (processed UCI egress).

Read against PCL/PYRAMID:

| MEL/ABB feature | PCL/PYRAMID counterpart | Observation |
|-----------------|-------------------------|-------------|
| Vendor `.so` + factory symbols, loaded into the consumer process | `pcl_plugin_loader` + transport plugin ABI | Same deployment shape. Interesting inversion: MEL hides the *transport* inside the vendor library and standardises the *typed API*; PCL standardises the transport ABI and generates the typed facades. Both end at "typed calls in, vendor/transport bytes out". |
| C2MEL / DataMEL split | Service + consumed-service endpoints vs stream-service endpoints (`pcl_container_add_stream_service`, `pcl_stream_send`) | Direct analogue: control-plane request/response plus a subscribed high-rate product stream. |
| ABB over UDP, minimally-processed data, fan-out to any processor | `pcl_transport_udp`, `pcl_transport_shared_memory` | The proving plan's missing "genuinely BEST_EFFORT information topic" (gap 4 in `agra_pubsub_shm_udp_proving_plan.md`) now has an externally-sourced archetype: raw sensor product streams are the canonical BEST_EFFORT payload. |
| VITA 49.2 packetization | Codec plugin layer | Candidate stream codec if we ever front real/simulated MFAs; a well-specified, widely-implemented framing for exactly the SHM/UDP payload class we are proving. |
| Rates: 2 Msps I/Q, 4K mono8 @ 60 FPS | (no stated targets today) | Concrete, citable load targets for the SHM/UDP data-plane proof — worth adopting as the benchmark profile rather than inventing our own numbers. |
| "No MFA-side interface codified" | PIM port grammar defines both provider and consumer | We are stricter; no action, but explains why MEL swapping is Skill-centric. |

The strategic read: AMS GRA arrived at **typed-C++-plugin-per-vendor** for
the data plane and **transport-abstracted pub/sub** for the message plane —
i.e. the same two-tier answer PCL/PYRAMID has, developed independently.
That is useful external validation for
`transport_codec_plugin_system.md`'s design, and a caution that the data
plane's *interface* (not just its transport) is where vendor competition
happens — which is what our generated typed facades already provide.

---

## 4. The CAL/LA-CAL: AMS GRA's message plane, read against PYRAMID

- An **OMS Service** is "a software-only capability that communicates
  exclusively over the CAL" in standard UCI messages — no hardware access.
  That is a PYRAMID component in our vocabulary, with the CAL playing the
  role of the generated service/topic bindings over transport plugins.
- **LA-CAL** connection flow: WebSocket to Sleet → OWP initialization
  message (versions, schema, service identifier) → subscription commands
  per UCI message type and topic → publish results. Message-type-scoped
  topics and pure pub/sub — the same **service-over-pub/sub** discipline
  the a-gra review found in DMS (topic == message name, correlation by ID,
  no RPC), already analysed in
  [`pubsub_contract_generation_plan.md`](../../plans/PYRAMID/README.md).
  The starter kit confirms the pattern holds across the ecosystem's
  dev-grade transport too, which strengthens that plan's premise that
  pub/sub projection is the load-bearing shape, with the concrete wire
  (WebSocket/JSON, DDS/EXI) a swappable backend.
- **Compliance artefacts per service:** an OMS Service Contract
  (subscribed/published messages + configuration), a `service.yml`
  deployment descriptor, RRD (resource requirements), SBOM, SAST/DAST
  evidence. The bundle system validates every declared port against a
  workspace `docs/ports.md` registry **bidirectionally** and ships OCI
  archives + `manifest.json` + `SHA256SUMS`.

PYRAMID mapping and gaps:

1. **JSON codec: have. WebSocket transport: don't.** Our codec set already
   round-trips JSON; nothing in-tree speaks WebSocket as a PCL transport
   (websocketpp is vendored, but only used by `ame_foxglove`). A thin
   LA-CAL/OWP client — whether as a PCL transport plugin or as bridge-local
   code inside an `oms_bridge`-style component — is the cheapest path to
   joining a real ASB (§5.2).
2. **Service contract ≈ binding manifest.** Their OMS Service Contract +
   `service.yml` + port-registry validation is the same compose-time,
   fail-closed discipline as our `binding_manifest.json` endpoint
   requirements and contract routing validation — except their runtime
   (Sleet) then *silently rejects* unregistered services, a pitfall our
   fail-closed validation deliberately avoids. Keep ours; if we integrate
   with Sleet, surface registration failures loudly in the client.
3. **Identity: adopt UUIDv5 derivation.** `pyramid_core` already has UUID
   helpers (`core/include/uuid/`). The starter kit's
   Realm→SystemID→SubsystemID→MessageID scheme is a concrete, proven
   convention for populating the A-GRA `MessageHeader.SystemID/ServiceID`
   fields (and correlating command/status pairs) deterministically from
   configuration names. Recommend writing it into the `agra_c2_bridge`
   design as the header-population rule.
4. **Packaging kinship.** `service.yml`/RRD/SBOM/bundle-manifest map onto
   `sdk_packaging.md` and our SDK template; if AMS GRA-adjacent delivery
   ever becomes a target, aligning our descriptor fields with their
   `service.yml` template is cheap early and painful late. The workspace
   port-registry idea (functional ranges, next-available tracking,
   bidirectional manifest validation) is worth adopting for our own
   multi-component deployments regardless.

---

## 5. What this changes for the A-GRA adoption path

### 5.1 Risk retired (partially): OMS/CAL availability

`a_gra_standard_review.md` §8 lists "OMS/CAL access may be controlled" as
an external dependency for any onboard (VI/MS) compliance claim. The
starter kit demonstrates a **public LA-CAL implementation** and a full
UCI-speaking simulation ecosystem around it. For *prototyping and demos*,
the dependency is retired: we can exercise the MS-interface leg (UCI
sensor products → `StateUpdate` → WorldModel grounding) against live,
independently-verifiable ground truth without access-controlled artefacts.

For *compliance*, nothing changes: LA-CAL is explicitly a dev-grade,
language-agnostic pattern; real platforms choose their CAL/ASB
implementations at integration time (MSI responsibility, per the ICD), and
the offboard C2/Peer stack (EXI + DMS/DDS) appears nowhere in this kit.

### 5.2 A concrete Phase-2 demo option (MS leg)

The adoption path's Phase 2 envisages a stub C2 client for the planning
loop. The starter kit adds a complementary option on the **MS ingress**
side, closer to the worked example's sensing chain:

1. Spike a LA-CAL/OWP WebSocket client (JSON payloads; websocketpp already
   in-tree) and register it with a Sleet instance via the `.toml`
   mechanism.
2. Subscribe to the kit's signal-report/track UCI messages
   (`rf-fm-demod` / `ir-search-and-track` output).
3. Translate to `StateUpdate`/WorldModel fact ingress — the same arrow as
   review §5.1's "MS-volume track messages" row — and drive an AME
   observe/monitor goal against the Kitty Hawk scenario.
4. Evaluate with their own pattern: perceived (UCI-derived WorldModel
   state) vs truth (DIS) divergence, including genuine loss-of-track on
   the pentagon pattern's occlusions — a ready-made, externally-authored
   test of AME's replan-on-state-change loop.

Scope caution: the kit speaks **sensor-domain UCI**, not the `MA_*`
planning/approval set. It cannot exercise the C2 planning loop
(Action/Plan/Approval), so it *complements* the Phase-2 stub C2 client
rather than replacing it. Its UCI dialect and schema version also need
checking against `A-GRA_MessageDefinitions_v5_0_a.xsd` before assuming the
converted `agra.*` protos parse its payloads — the kit predates/parallels
the ASK 5.0a drop and is "not yet fully compliant" by its own admission.

### 5.3 Duties confirmed or sharpened

- **Coordinate-frame ownership.** The kit's hard-won "centralized ECEF ↔
  WGS84 conversion" lesson lands on whatever bridge/ingress we build; our
  `pyramid.data_model` position types (common/sensors/tactical) should
  declare their frame explicitly, and conversion belongs in one shared
  utility, not per-component.
- **Snapshot ingress semantics.** UCI activity messages are complete
  snapshots — consistent with A-GRA's `ObjectState` publisher-signal
  model, and a direct instruction for WorldModel ingress: replace, don't
  merge, per activity object.
- **Wire-stack ladder, now with a bottom rung.** LA-CAL JSON/WebSocket
  (dev, public, runnable today) → OMS/CAL/ASB proper (onboard compliance,
  integrator-chosen) → EXI over DMS/DDS (offboard compliance). Phase
  ordering should climb that ladder rather than jump to EXI/DMS.

### 5.4 Data-plane targets for the SHM/UDP proving plan

The proving plan needs a legitimately BEST_EFFORT information topic and has
no external load profile. Adopt the MEL numbers: sustained 2 Msps
interleaved I/Q (≈ 8 MB/s at 16-bit) over UDP with VITA 49.2-style framing,
and mono8 frame streams, as the benchmark payload classes. If the example
A-GRA-vocabulary contract in that plan grows a raw-product stream, this is
the shape it should have.

---

## 6. Adoptable ideas (independent of any AMS GRA commitment)

- **UUIDv5 deterministic identity** (§4.3): adopt for `agra_c2_bridge`
  header population and command/status correlation.
- **Workspace port registry with bidirectional validation** (§4.4): adopt
  for multi-component deployment/compose validation.
- **Truth-vs-perceived overlay as acceptance metric**: "if algorithms are
  perfect and latency zero, perceived overlaps truth" is a crisp E2E
  quality statement; our Foxglove layer could render WorldModel-derived
  state against a sim-truth channel the same way.
- **MEL-style DK (development kit) discipline**: vendor deliverable =
  library + integration docs + test tools + SBOM. Matches and slightly
  extends our SDK packaging story; the "test tools ship with the plugin"
  element is the part we lack.

---

## 7. Governance and provenance cautions

- **GitLab is primary; the GitHub mirror is unmonitored** ("GitHub issues
  and pull requests will not be monitored"). Track upstream, file issues,
  and pin references against the GitLab origin; this review read the
  GitHub mirror.
- **Maturity:** self-described early development, "not yet fully AMS GRA
  compliant". Treat the kit as *reference for patterns and a test
  environment*, not as a normative statement of AMS GRA — the authoritative
  volumes live in the AMS GRA repositories proper (not yet reviewed; a
  volume-level review analogous to a-gra §6 is the natural follow-up if AMS
  GRA alignment becomes more than a test-harness interest).
- **Terminology instability:** the MEL expansion inconsistency (§2.1) and
  the acknowledged pending UCI-extension submission on the A-GRA side both
  signal ecosystem-wide churn; keep converted artefacts versioned against
  dated upstream drops, as already planned for `agra.*`.
- **Licensing:** federal open-source posture (Code.mil `INTENT.md`, DCO,
  public-domain carve-out for US government employee contributions).
  Check the licence artefacts before vendoring any MEL IDD headers or OWP
  protocol code into this repo.

---

## 8. Recommended actions

| # | Action | Where it lands | Size |
|---|--------|----------------|------|
| 1 | Record the OMS/CAL-availability risk mitigation and this doc as a companion in the a-gra review | `a_gra_standard_review.md` header/§9 (done with this commit) | trivial |
| 2 | LA-CAL/OWP WebSocket+JSON client spike; register against a local Sleet; subscribe signal reports → `StateUpdate` ingress demo (§5.2) | new spike under PYRAMID examples or the bridge component | small |
| 3 | Adopt MEL load profile (2 Msps I/Q, mono8 frames, VITA 49.2 framing) as the SHM/UDP proving plan's BEST_EFFORT payload class | `agra_pubsub_shm_udp_proving_plan.md` benchmark section | small |
| 4 | Write UUIDv5 Realm→System→Subsystem→Message derivation into the `agra_c2_bridge` design as the header-population rule | bridge design (a-gra review §5.2 successor doc) | small |
| 5 | Confirm MEL IDD licence + official expansion; assess `interfaces/rf-mel`, `interfaces/ir-mel` as candidate contract imports if PYRAMID sensor components ever front (simulated) MFAs | follow-up review | medium |
| 6 | Workspace port registry + `service.yml`-aligned deployment descriptor fields for our bundles | `sdk_packaging.md` / deployment tooling | medium |

---

## 9. References

- Starter kit (GitHub mirror, read 2026-07-08):
  https://github.com/open-arsenal/ams-gra-hello-world-sk-getting-started —
  `README.md`, `docs/architecture.md`, `docs/contracts.md`,
  `docs/decisions.md`, `docs/maintainer-guide.md`, `docs/ports.md`,
  `docs/visualization.md`, `docs/tutorials/0-overview.md` …
  `4-swapping-skills.md`
- A-GRA review + worked example:
  [`a_gra_standard_review.md`](a_gra_standard_review.md),
  [`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md)
- PCL/PYRAMID plans this feeds:
  [`pubsub_contract_generation_plan.md`](../../plans/PYRAMID/README.md),
  [`agra_pubsub_shm_udp_proving_plan.md`](../../plans/PYRAMID/README.md)
- In-repo anchors: `subprojects/PCL/include/pcl/pcl_container.h`
  (stream-service endpoints), `subprojects/PCL/include/pcl/pcl_plugin.h`
  (plugin ABI), `subprojects/PYRAMID/core/include/uuid/`,
  `subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md`,
  `subprojects/PYRAMID/doc/architecture/sdk_packaging.md`
