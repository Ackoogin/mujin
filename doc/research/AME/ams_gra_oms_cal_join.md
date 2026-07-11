# Joining PYRAMID's Binding Layer with the AMS GRA / OMS CAL

**Scope:** How PYRAMID's generated-binding stack (contracts → facades → codec +
transport plugins, and the new RPC/pub-sub interaction seam) can join an
AMS GRA / OMS **Critical Abstraction Layer (CAL)**. Answers three questions
raised after the interaction-facade work landed:

1. **How standard is the AMS GRA C++ CAL?** It looks like it has rules but
   not a concrete interface — is that right?
2. **Do we use a C++ OMS CAL as a plugin for PYRAMID?** If so, how do the
   types get mapped?
3. **Or should the PYRAMID side join the abstract service bus itself** —
   and doesn't that duplicate (or have to align with) the OMS provider's
   C++ CAL implementation?

**Date:** 2026-07-11
**Companions:**
[`a_gra_standard_review.md`](a_gra_standard_review.md) (§6.3 wire stack —
the onboard MS/VI legs this doc makes concrete),
[`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md) (§4
LA-CAL/Sleet, §5.2 MS-leg demo — extended here),
[`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md).
**In-repo anchors:**
[`pubsub_interaction_guide.md`](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md)
(the interaction facade / pub-sub seam),
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(plugin ABI + capability model),
[`agra_example/README.md`](../../../subprojects/PYRAMID/pim/agra_example/README.md).
**Inputs reviewed:** the OMS Standard v2.5 document set, published publicly
2026 in [github.com/open-arsenal/oms](https://github.com/open-arsenal/oms) —
specifically OMSC-STD-001 RevM (OMS Standard), OMSC-SPC-001 RevL (CAL
Specification), OMSC-SPC-008 RevK (C++ CAL Specification), OMSC-SPC-013 RevB
(Language-Agnostic CAL Specification) — read via the repo's
`docs_markdown_unofficial/` conversions (**verify quotes against the
official `.docx` in `docs_official/` before any normative use**); the
[open-arsenal/ams-gra](https://github.com/open-arsenal/ams-gra) volume
listing; the
[open-arsenal/oms-sk-cal](https://github.com/open-arsenal/oms-sk-cal)
starter-kit CAL; and the open-arsenal org repo inventory.

---

## 1. Executive summary

1. **"The AMS GRA C++ CAL" is the OMS C++ CAL, and it is more standard than
   it first looks — but only at the source level.** AMS GRA defines no CAL
   of its own: its Interface Description Documents are the two MEL IDDs
   (RF/IR data plane); for the message plane its volumes defer to OMS.
   The OMS CAL is specified in three layers: OMSC-SPC-001 pins the
   *semantics* (pure pub/sub, one topic ↔ one message type, QoS,
   lifecycle); OMSC-SPC-008 pins a **concrete C++ API** — normative
   namespaces, class names, and method prototypes
   (`uci_getAbstractServiceBusConnection()`, typed `Reader`/`Writer`/
   `Listener` per message type, factory create/destroy pairs, XSD→accessor
   type mapping). The stated goal is source portability: an application
   "developed and tested against a specific CAL Implementation, can be
   compiled against any given CAL Implementation and execute properly."
   What is **not** standardized: the ABI, the wire protocol, the middleware
   underneath, callback threading, and the accessor internals. Two CAL
   implementations do not interoperate on the wire — the ASB is defined as
   carrying messages "from one CAL instance to another **given that they
   are of the same CAL Implementation**". So: concrete interface, yes;
   concrete *implementation or wire*, no. (§2)

2. **The impression "rules but not a concrete interface" comes from
   reading AMS GRA instead of OMS** — AMS GRA's volumes state usage rules
   and compliance duties, and the *abstract* CAL spec is indeed rules; the
   concrete interface lives one repo over, in the OMS document set. That
   set (v2.5, Jan 2026) is now **fully public**, which retires the
   a-gra review's "OMS/CAL access may be controlled" risk outright — not
   just the prototyping slice the starter kit covered. (§2.1)

3. **Yes — a C++ OMS CAL wraps naturally as a PCL transport plugin, and
   the just-landed pub/sub seam is precisely what makes that viable.**
   The CAL is pub/sub-only ("OMS Messaging interactions between OMS
   participants must use the CAL-provided publish and subscribe
   interface") — its capability row is `PUBSUB ✓, RPC_UNARY ✗,
   RPC_STREAM ✗`, reliability configurable. Before the interaction facade,
   our Request-shape services required RPC caps and would have failed
   closed against any CAL-backed transport; now, realizing both legs as
   the correlated topic pair is a routing-manifest choice and the same
   component code runs unchanged. Type mapping is the real work, and it
   has a clean answer: **generate it** — both sides (our `agra.*` protos
   via the planned `xsd2proto` profile, and the CAL's accessor classes)
   derive from the same UCI XSD, so a `pyramid_<T>_c` ↔ `uci::type`
   accessor mapping emitter is mechanical, with the ROS2 coupled plugin's
   typed-wire mapping as the in-repo precedent. A serialized shortcut via
   the CAL's `Externalizer` exists where providers ship a text encoding.
   The unavoidable cost: source-level standardization means the plugin
   *source* is portable but the *binary* is per-platform — one plugin,
   recompiled against each provider's CAL SDK. (§4)

4. **"Joining the ASB itself" is only sound in one form: LA-CAL.** In
   general the bus has no standardized wire — "joining the bus" means
   *being* a CAL implementation, which means reverse-engineering a
   specific platform's middleware and aligning with a vendor artifact:
   exactly the duplication worry, and the reason to reject it. The
   exception is the **Language-Agnostic CAL (OMSC-SPC-013)**: a normative
   *wire protocol* (WebSocket subprotocol `owp`, INIT/INFO/SUB/PUB/
   UNSUB/MSG, OMS JSON payloads) with no client API mandated. An LA-CAL
   client does **not** duplicate the provider's C++ CAL — the platform's
   LA-CAL server is itself one of the CAL forms the platform delivers, and
   it bridges clients onto the same ASB. Alignment is with a public spec,
   not a vendor implementation. The open questions are availability
   (whether a given real platform ships an LA-CAL server is a
   Platform Description Document matter — guaranteed in the starter kit
   via Sleet, not guaranteed on real platforms) and rate (text WebSocket;
   fine for MA-class traffic, not for the MEL data plane). (§5)

5. **Recommendation — climb the ladder, plugin-shaped at every rung.**
   (1) Now: an **LA-CAL coupled plugin** (WebSocket/`owp` transport +
   OMS-JSON codec, one `.so`, the gRPC/ROS2 coupled-target pattern),
   proven against the public Sleet server and `la-cal-harness`. This
   upgrades the starter-kit review's "spike" (action #2) to a first-class
   transport. (2) Later, per target platform: a **C++ CAL plugin** — same
   vtable mapping, provider SDK underneath, generated typed mapping.
   (3) Throughout: keep **semantic** translation (A-GRA vocabulary ↔
   `autonomy_backend`) in the `agra_c2_bridge` component where it already
   belongs; the CAL join is *transport adaptation only* and must not grow
   semantics. A dual-homed gateway component remains the fallback if a
   provider CAL's lifecycle/licensing can't be shaped into the plugin ABI.
   (§6, §7)

---

## 2. What the specs actually pin down

### 2.1 The document set is now public

[github.com/open-arsenal/oms](https://github.com/open-arsenal/oms) hosts
the unclassified **OMS Standard v2.5** (January 2026): the standard
(OMSC-STD-001 RevM), the CAL Specification (OMSC-SPC-001 RevL), the
language bindings — Java (OMSC-SPC-007 RevK) and **C++ (OMSC-SPC-008
RevK)** — the **Language-Agnostic CAL** (OMSC-SPC-013 RevB), the OS Facade
spec (OMSC-SPC-005), and the full template/checklist set (Platform/Service
Description Documents, Service Contract, Mission Package
Worksheet/Deployment Descriptor). Both official `.docx` and unofficial
Markdown conversions are checked in. The sibling
[open-arsenal/uci](https://github.com/open-arsenal/uci) repo publishes the
UCI schema set.

This changes the risk picture materially: when
[`a_gra_standard_review.md`](a_gra_standard_review.md) §8 was written,
"OMS/CAL access may be controlled" was a live external-artefact risk, and
the starter-kit review could only retire the *prototyping* slice
(LA-CAL-by-example via Sleet). The normative CAL API surface — including
the C++ prototypes — is now public.

What remains **not** public: any C++ CAL *implementation*. The
starter-kit's [`oms-sk-cal`](https://github.com/open-arsenal/oms-sk-cal)
is Java (`uci-cal-api`, `uci-cal-jms`, `uci-externalizer-xml` 2.5.0 jars —
a JMS-backed CAL with an XML externalizer, binaries only). C++ CAL
implementations remain platform deliverables from the Mission Systems
Integrator, exactly as the a-gra review's §6.3 recorded.

### 2.2 CAL semantics (OMSC-SPC-001): pure pub/sub, topic ↔ message type

The abstract spec is where the "rules" live, and the rules are strict and
convenient for us:

- **Pub/sub only.** "OMS Messaging interactions between OMS participants
  must use the CAL-provided publish and subscribe interface." There is no
  request/response or RPC anywhere in the CAL — commands and their statuses
  are correlated pub/sub messages, the same Command-2/ActionRequest-2
  discipline the a-gra review documented for the message set, now
  confirmed at the API layer.
- **One topic ↔ exactly one message type.** "A CAL Implementation shall
  associate a topic with one and only one CAL Message type"; each
  `Reader`/`Writer` binds to exactly one topic. This is the same
  topic-per-message-type discipline as DMS's MA-L1-013, applied onboard.
- **QoS.** Two reliability levels — Best Effort ("does not guarantee
  message delivery and will not retransmit") and Reliable (resend while
  the message "remains available for retransmit", order preserved per
  connection) — plus per-topic receive buffering with configurable depth
  and oldest-first eviction, shelf-life expiration, and periodic-rate
  down-filtering on readers.
- **Identity & lifecycle.** A client initializes a CAL instance from a
  **Service Identifier**; the CAL returns the System / Service / Subsystem
  / Component / Capability **UUIDs** (RFC 4122). One CAL instance per
  (Service Identifier, ASB Identifier) pair. A **connection-status state
  machine** (INITIALIZING / NORMAL / DEGRADED / INOPERABLE / FAILED) is
  exposed via callbacks, fired immediately with current state on
  registration.
- **Thread safety** of readers, writers, and factories is normative.
- **The ASB is deliberately opaque.** It is "the collection of resources
  that at a minimum allow IP based network traffic, in the form of OMS
  Messages, from one CAL instance to another **given that they are of the
  same CAL Implementation**", and the middleware is unconstrained: "This
  document makes no restriction on the tools, libraries, or products that
  may be used." The platform "must make available an appropriate CAL
  Implementation that satisfies the CAL API in all languages with OMS
  Standard-defined XSD-to-API translations."

That last pair of statements is the crux for question 3: **wire-level
interoperability is only promised between endpoints of the same CAL
implementation.** The portable seam is the API above the CAL, not the bus
below it.

### 2.3 The C++ CAL (OMSC-SPC-008): a concrete, source-level API

The C++ spec is not rules — it is prototypes. The normative surface:

```cpp
// entry point — the one global function a service calls
uci::base::AbstractServiceBusConnection* uci_getAbstractServiceBusConnection();
//   .getMySystemUUID() / .getMyServiceUUID() / .shutdown()

// per message type M (generated from the XSD), typed endpoints:
MWriter*  conn->...createWriter(...);     void write(const M&);  void close();
MReader*  conn->...createReader(...);     void read(M&); void readNoWait(M&);
                                          void addListener(MListener&); void close();
struct MListener { void handleMessage(const M&); };   // override + register

// serialization escape hatch
ExternalizerLoader* uci_getExternalizerLoader();
Externalizer* ldr->getExternalizer(encoding);   // read/write M <-> stream/string
```

Message types are **generated accessor classes** (`uci::type::…`) from the
UCI XSD under a normative XSD→C++ translation: complex types become
accessors with getter/setters, enums become enum accessors, collections
become `SimpleList`/`BoundedList`, constructors are protected and all
instantiation goes through factories with paired `destroy*()` calls.

Normative vs implementation-defined splits exactly as the summary said:

| Normative (any conformant CAL) | Implementation-defined (per provider) |
|---|---|
| Namespaces, class names, method prototypes | Accessor internals / data layout ("the actual mechanism … is an implementation issue") |
| XSD→C++ type translation | Underlying middleware & wire |
| Factory/destroy memory discipline | Delivery/link model (shared vs static lib, build) |
| Topic↔type binding, QoS semantics | Listener callback threading (spec is silent) |
| Thread safety of readers/writers/factories | Externalizer encodings available |

So the honest characterization of "how standard": **as standard as POSIX,
not as standard as a wire protocol.** A service written to the spec
recompiles against any provider's CAL SDK; nothing about the artifact is
reusable across providers without recompilation, and nothing about the
bytes on the bus is portable at all.

### 2.4 The LA-CAL (OMSC-SPC-013): a concrete wire, no API

The Language-Agnostic CAL is the mirror image — a **protocol
specification**, not an API:

- Transport: WebSocket (RFC 6455), subprotocol **`owp`** (OMS WebSocket
  Protocol), UTF-8 text frames.
- Flow: client sends `INIT` `{versions, schema, service_id, verbose}` →
  server replies `INFO` (negotiated version, server id, the client's
  System/Service/… UUIDs, system label) → `SUB` (subscription id, message
  name, topic, optional group) / `PUB` / `UNSUB` / server-sent `MSG`, with
  `+OK`/`-ERR` acks on verbose connections.
- Payloads: **OMS JSON** — a normative XSD→JSON mapping (global element
  key is the bare name for the `https://www.vdl.afrl.af.mil/programs/oam`
  namespace, `{namespace}name` otherwise; typed rules for choice,
  numerics, `NaN`/`Infinity` as strings, UUIDs as RFC 4122 strings).
- Division of labour: "CAL Clients that use an LA-CAL are responsible for
  UUID generation and message construction"; "A Provider of an OMS
  Platform is responsible for delivering required CAL Server
  implementations." No authentication in the protocol — security is a
  deployment concern (PDD / Service Contract).

The starter kit's Sleet server and `.toml` registration flow (reviewed in
[`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md) §4) is an
implementation of exactly this spec — the kit was not inventing a pattern,
it was implementing OMSC-SPC-013.

### 2.5 AMS GRA's own position

The [`ams-gra`](https://github.com/open-arsenal/ams-gra) repo confirms the
starter-kit review's read at the source: sixteen Architecture Volumes
(Mission System, Software Architecture, OMS Service MPU, Mission Agnostic
Service Infrastructure, Multi-Function Aperture MPU, …), Compliance
Artifacts, and an `Interface_Description_Documents/` folder containing
**only** `RF_MEL_IDD.pdf` and `IR_MEL_IDD.pdf`. AMS GRA codifies its own
concrete interface where it owns one (the MEL data plane) and otherwise
**requires OMS Services to communicate via the OMS CAL** — rules about
using the CAL, with the concrete interface delegated to the OMS specs.
That resolves question 1's framing: both readings are right, about
different documents.

---

## 3. The two "CALs" side by side

The user-facing framing — "join the PYRAMID CAL (bindings) with an AMS GRA
one" — is apt: PYRAMID's generated-binding stack plays the CAL role for
PYRAMID components. But the two stacks standardize **opposite halves**, the
same inversion the starter-kit review found for the MEL (§3 there):

| Layer | PYRAMID bindings | OMS CAL |
|---|---|---|
| Typed app API | Generated facades + interaction facade (`submit`/`transitions`/`publish`/`subscribe`) — generated from `.proto`, ours | Generated accessors + `Reader`/`Writer`/`Listener` — generated from XSD, **normative prototypes** |
| Serialization | Codec plugins (JSON / FlatBuffers / Protobuf), decoupled, `content_type`-keyed | `Externalizer` (encoding-keyed, provider-supplied) or implicit in the CAL |
| Transport | Transport plugins behind a **frozen C ABI** (`pcl_transport_t`), swappable at run time | The CAL implementation itself — **no ABI, no wire standard**, swap = recompile |
| Wire interop | Per-plugin wire (socket/shm/udp framing, gRPC, ROS2) — both ends load the same plugin | Same-CAL-implementation only; LA-CAL is the one normative wire |
| Composition | Routing manifest, caps + QoS validated at compose time, fail closed | Platform integration (MSI), Service Contract + PDD checklists |

PCL standardizes the *plugin boundary* and lets the API be generated; OMS
standardizes the *API* and lets the implementation float. The joint is
therefore natural and singular: **an OMS CAL (either language-specific or
LA-CAL) slots in as a PCL transport implementation** — below our frozen
ABI, above their normative API or wire. Nothing inward of the plugin
boundary (facades, components, AME) is affected.

### 3.1 The capability row, and why the new seam is the enabler

The CAL's capability declaration in our model
([`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)):

| Transport | `PUBSUB` | `RPC_UNARY` | `RPC_STREAM` | Reliability | Codec |
|---|:---:|:---:|:---:|---|---|
| OMS C++ CAL (provider SDK) | ✓ | ✗ | ✗ | configurable (BE/REL per topic) | coupled (typed accessors) |
| LA-CAL (`owp` WebSocket) | ✓ | ✗ | ✗ | server-dependent¹ | coupled (OMS JSON) |

¹ The `owp` protocol itself rides a reliable WebSocket, but the CAL QoS
contract (buffering, shelf life) is the server's; what the plugin may
declare needs confirming against OMSC-SPC-013's QoS text per deployment.

This row is exactly the profile that used to be a dead end: an
`autonomy_backend`-style contract full of Request-shape services requires
`RPC_UNARY` + `RPC_STREAM`, so compose-time validation would fail closed
against a PUBSUB-only transport — correctly, and terminally. **The
interaction facade changes the conclusion, not the validation**: realizing
both legs of every Request port as the correlated
`.request`/`.requirement` topic pair
([`pubsub_interaction_guide.md`](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md))
needs only `PUBSUB`, is chosen per leg in the routing manifest, and runs
the same compiled component. The A-GRA-shaped correlated-pair semantics the
seam implements (flat topics, correlation by payload id, acceptance as a
status transition, no synthesized acks) are **the same semantics the CAL
enforces** — the seam was derived from this ecosystem's pattern, and the
CAL is where it cashes out.

### 3.2 Topic and wrapper discipline at the boundary

Two PYRAMID-isms need reconciling with the OMS/UCI wire, both already
anticipated by the `agra_example` notes:

1. **Topic names.** Our scheme is `<project>.<interface>.<role>`; the
   OMS/A-GRA discipline is *topic name == message type name*. Topic
   strings are contract options (`pyramid.options.pyramid_op`), so an
   OMS-facing contract tree simply stamps UCI type names — expressible
   today, no generator change
   ([`agra_example/README.md`](../../../subprojects/PYRAMID/pim/agra_example/README.md)).
2. **The `_Request` wrapper.** Our request topic carries the service's
   wrapper message (command variants + `cancel` in a oneof); the OMS wire
   carries each UCI message on its own topic (`MA_Action` on the
   `MA_Action` topic, `ActionCancelCommand` on its own). The wrapper is a
   contract-internal convenience and must not leak onto an OMS bus.
   Options, in preference order:
   - **Unwrap in the coupled plugin/codec** — the projectability rule
     already guarantees each projectable command maps to exactly one
     wrapper variant, so variant ↔ per-type topic is a bijection the
     plugin can apply mechanically (publish: strip wrapper, emit variant
     on its type topic; subscribe: wrap on ingress). Correlation by
     payload id is unchanged — it is how UCI correlates too.
   - Author the OMS-boundary contract 1:1 (one port per UCI message
     family) so no wrapper exists to strip. More contract surface, zero
     plugin cleverness; reasonable for a narrow first profile.

### 3.3 QoS and status mapping

- CAL Best Effort / Reliable ↔ PCL reliability floors: **direct** — the
  contract-floor vs transport-ceiling reconciliation applies unchanged,
  including the fail-closed RELIABLE-over-BEST_EFFORT negative.
- CAL buffering depth, shelf life, and rate filtering: extra per-topic
  knobs → plugin `config_json`, same opaque-JSON convention as every other
  plugin.
- CAL connection status (INITIALIZING/…/FAILED): PCL has no transport
  health surface today; minimally, map to plugin lifecycle (fail
  `on_configure` until NORMAL) and log transitions — a fuller
  health-signal seam is a noted gap, not a blocker.
- CAL init-time UUIDs (System/Service/…): the authoritative source for
  the `MessageHeader.SystemID/ServiceID` population that the starter-kit
  review's UUIDv5 recommendation covered — when connected via a real CAL,
  **take identity from the CAL**, and use UUIDv5 derivation only where no
  CAL assigns one (offboard/DMS legs, tests).

---

## 4. Question 2 — the C++ CAL as a PCL transport plugin

**Yes, and it is the right long-term shape for platform integration.** The
plugin translates the vtable's `publish`/`subscribe` slots to typed CAL
calls:

```
pcl publish(topic, encoded msg)
  → look up topic's UCI type → obtain/construct accessor → Writer::write()
pcl subscribe(topic)
  → createReader(topic) + Listener whose handleMessage() deep-copies,
    encodes, and posts to the executor queue (foreign-thread ingress rule)
```

Design notes, in decreasing order of consequence:

1. **The type-mapping problem — generate it.** The CAL API is typed
   (accessors), while the PCL boundary carries encoded payloads, so
   something must cross between `pyramid_<T>_c` structs and `uci::type`
   accessors. Both sides descend from the same UCI XSD: our `agra.*`
   protos via the planned `xsd2proto` profile
   ([`a_gra_standard_review.md`](a_gra_standard_review.md) §4, §6.2), the
   accessors via the OMS-normative XSD→C++ translation. A
   **profile-driven mapping emitter** (a third consumer of the same
   profile manifest, structurally like the existing `*_cabi_marshal`
   emitters) produces `toAccessor(const pyramid_<T>_c&, uci::type::T&)` /
   `fromAccessor` pairs — mechanical field-by-field code, linear in
   profile size, diff-stable if field order is taken from XSD declaration
   order exactly as `xsd2proto` already plans. **In-repo precedent:** the
   ROS2 coupled plugin's typed `pyramid_msgs` wire is this same move
   (generated proto↔IDL type mapping inside a coupled plugin), so both the
   architecture and the cost model are already proven here.
2. **The serialized shortcut — `Externalizer`, where available.** If the
   provider ships a text externalizer (the starter-kit Java CAL ships
   `uci-externalizer-xml`; encodings are provider-defined), the plugin can
   skip accessors: a codec emits the externalizer's encoding and the
   plugin crosses via `Externalizer::read/write`. Cheaper to build, but it
   trades a normative dependency (accessor prototypes) for a
   non-normative one (which encodings a provider ships) — treat it as a
   per-platform optimization, not the portable design.
3. **Coupled plugin, not decoupled.** Because the wire representation is
   the provider's and the API is typed, this is a **coupled
   codec+transport target** exactly like gRPC/ROS2: one `.so`, two
   vtables, one `content_type` (e.g. `application/oms-cal`).
4. **Threading.** The CAL spec mandates thread-safe readers/writers but is
   silent on listener callback threads — so the plugin must treat
   `handleMessage()` as a foreign thread unconditionally: deep-copy, post
   to the executor, never call back inline. That is already the PCL
   threading contract; no new rule, just no shortcuts.
5. **Portability economics.** Source-level standardization ⇒ **one plugin
   source, N platform binaries**, each compiled against a provider's CAL
   SDK (headers + lib, delivery model unspecified by the spec). The plugin
   source should therefore touch only OMSC-SPC-008-normative names — any
   provider extension goes behind `config_json`.
6. **Per-type endpoints.** Readers/writers are per-message-type, so the
   plugin needs a topic→typed-endpoint dispatch table. Generate it from
   `binding_manifest.json`'s topics (the manifest already carries every
   contract topic + QoS floor) alongside the mapping emitter's output.

**Blocker, honestly stated:** there is no public C++ CAL implementation to
build against today — `oms-sk-cal` is Java/JMS binaries. The C++ CAL plugin
is *specifiable* now (the API is public) but only *provable* against a
provider SDK or a self-written toy CAL. That ordering is what makes the
LA-CAL rung first (§6).

---

## 5. Question 3 — joining the abstract service bus directly

Split the question the way the specs do:

**The general form — speak the platform ASB's wire — is unsound.** The ASB
has no normative wire; interop is promised only between endpoints of the
same CAL implementation (§2.2). A PYRAMID transport plugin that joins the
raw bus is, by definition, a reimplementation of that platform's CAL over
that platform's middleware: per-platform reverse engineering, aligned with
a vendor artifact that can change under us, with no compliance cover —
precisely the "duplicates or has to align with the OMS provider's C++ CAL"
trap the question identified. Reject this form.

**The LA-CAL form is sound, and is not a duplication.** OMSC-SPC-013 is a
public, normative wire onto the bus, designed exactly so that clients
which cannot (or choose not to) link a platform CAL library can still
join. The platform's LA-CAL **server** is one of the CAL implementations
the platform delivers; our client aligns with the spec, not with any
vendor's C++ CAL, and the two coexist by design — a C++ CAL service and an
LA-CAL client are peers on the same ASB, bridged by platform
infrastructure. For PYRAMID this is:

- a **transport plugin** implementing `owp` over WebSocket (websocketpp +
  asio are already vendored, currently used only by `ame_foxglove`), doing
  INIT/INFO, SUB/UNSUB per routed topic, PUB on publish, MSG → executor
  ingress, `+OK`/`-ERR` mapped to `pcl_status_t`;
- an **OMS JSON codec** for the profile types. **Warning:** OMS JSON is
  *not* proto3 JSON of the converted `agra.*` protos — element naming,
  choice representation, and the namespace-keyed global-element rule
  differ. The codec must implement OMSC-SPC-013's mapping, and the right
  way to keep it honest is to generate it from the same XSD profile
  manifest (the `xsd2proto` tool already parses the XSD; emitting an
  OMS-JSON codec table from the same parse is incremental);
- packaged together as a **coupled plugin** (`application/oms-json`
  content type), same rationale as §4.3.

Caveats that keep this the *first rung* rather than the *answer*:

- **Availability on real platforms is a PDD matter.** The CAL spec obliges
  platforms to provide CAL implementations "in all languages with OMS
  Standard-defined XSD-to-API translations" — whether that clause now
  reaches OMSC-SPC-013 LA-CAL servers on any given platform needs
  confirming per program (open question; guaranteed only in the starter
  kit, where Sleet is the platform).
- **Rate.** UTF-8 JSON over WebSocket is the dev-grade rung; MA-class
  planning/command/status traffic fits comfortably, MEL-class data does
  not (that is the SHM/UDP data plane's job, per the starter-kit review
  §5.4).
- **Identity/registration friction.** Sleet's silent rejection of
  unregistered services (starter-kit review §4.2) is a client-UX hazard
  the plugin should surface loudly (fail `on_configure` when INIT is not
  answered by INFO).

---

## 6. Recommended shape

```
                       AME (unchanged)
                        │ IAutonomyBackend / IExecutionSink
                agra_c2_bridge            ← SEMANTIC translation only
                (A-GRA vocab ↔ autonomy_backend; approval,
                 header population, RBAC, status fan-out)
                        │ agra.* contract (xsd2proto profile)
        interaction facade, realization = pubsub per leg   ← the new seam
                        │ correlated topic pairs / information topics
      ┌─────────────────┴───────────────────┐
  LA-CAL coupled plugin                C++ CAL coupled plugin
  (owp WebSocket + OMS JSON,           (provider SDK + generated
   spec-aligned, runnable today         accessor mapping; one source,
   against Sleet)                       per-platform binaries)
      └───────────── the platform ASB ──────┘
```

Rungs, in order:

| # | Rung | Contents | Readiness |
|---|------|----------|-----------|
| 1 | **LA-CAL coupled plugin** | `owp` transport + OMS-JSON codec, one `.so`; prove against public Sleet + `la-cal-harness`; drive the starter-kit MS-leg demo (signal reports → `StateUpdate` → WorldModel) through it | Everything public; websocketpp in-tree; supersedes starter-kit review action #2's "spike" |
| 2 | **Capability row + manifests** | Declare `PUBSUB`-only caps; routing manifests realize Request ports as pub/sub over the CAL transport (the `exclusive` machinery already enforces one realization per leg) | Mechanism fully landed |
| 3 | **Profile tooling grows two emitters** | The planned `xsd2proto` profile manifest additionally feeds (a) the OMS-JSON codec tables, (b) the `pyramid_<T>_c` ↔ accessor mapping (consumed by rung 4) | Extends already-planned Phase-1 work |
| 4 | **C++ CAL coupled plugin** | Same vtable mapping as rung 1 with the provider SDK underneath; per-platform builds | Blocked on access to any C++ CAL implementation — spec-complete, implementation-starved |
| 5 | **Fallback: dual-homed gateway component** | An ordinary PYRAMID component linking the CAL natively, if a provider CAL's lifecycle/threading/licensing can't live inside the plugin ABI | Keep as documented fallback, don't build speculatively |

And one standing rule: **the CAL join carries no semantics.** Everything
A-GRA-meaningful (Action→goal mapping, approval gates, plan bookkeeping,
RBAC, heartbeats) stays in `agra_c2_bridge` per the a-gra review §5.2/§6.8;
the plugin moves already-shaped `agra.*` messages. If a proposed plugin
feature needs to know what a message *means*, it belongs in the bridge.

---

## 7. Risks and open questions

- **Verify against official text.** This review read the *unofficial
  Markdown* conversions in `open-arsenal/oms`; quote-level claims (CERT
  wording, `owp` field details, OMS JSON rules) must be re-checked against
  the official `.docx` set before any normative or compliance-facing use.
- **LA-CAL server availability on real platforms** (§5) — per-program PDD
  question; the fallback is rung 4/5.
- **No C++ CAL implementation to integrate against** (§4) — rung 4 cannot
  be *proven* until a provider SDK (or a deliberate toy CAL over an
  existing PCL transport, which would also be a good conformance harness)
  exists.
- **Schema-version alignment.** A-GRA is UCI 2.3-based with `MA_*`
  extensions and pending OACWG submission; OMS 2.5's message set and the
  starter kit's dialect are siblings, not the same drop. The profile
  manifest must pin exact schema versions per artefact (already the
  `xsd2proto` plan's discipline), and the LA-CAL `INIT.schema` field makes
  the negotiation explicit — surface a mismatch, don't coerce.
- **OMS JSON fidelity.** Hand-writing the codec invites drift from the
  XSD; generation from the same parse as `xsd2proto` is the mitigation
  (§5). Test against `la-cal-harness` fixtures, which are public.
- **QoS declaration honesty for LA-CAL** (§3.1 footnote) — confirm what
  reliability the plugin may declare from OMSC-SPC-013's buffering/QoS
  text rather than assuming WebSocket ⇒ RELIABLE.
- **Licensing.** The starter-kit repos are Apache-2.0 and the OMS repo is
  public; still check the OMS document set's distribution statement and
  the `uci` repo license before vendoring schemas or generating derived
  artifacts into this repo (same residual as a-gra review §7 Phase 0).

---

## 8. References

- OMS Standard v2.5 (public): https://github.com/open-arsenal/oms —
  `docs_official/` (`OMSC-STD-001` RevM, `OMSC-SPC-001` RevL CAL,
  `OMSC-SPC-008` RevK C++ CAL, `OMSC-SPC-007` RevK Java CAL,
  `OMSC-SPC-013` RevB LA-CAL, templates/checklists) and
  `docs_markdown_unofficial/` (read here)
- UCI schemas (public): https://github.com/open-arsenal/uci
- AMS GRA volumes: https://github.com/open-arsenal/ams-gra
  (Architecture_Volumes/, Compliance_Artifacts/,
  Interface_Description_Documents/ = RF/IR MEL IDDs)
- Starter-kit CAL (Java/JMS, binaries): https://github.com/open-arsenal/oms-sk-cal
- Sleet LA-CAL server (Rust, Apache-2.0):
  https://github.com/open-arsenal/ams-gra-hello-world-sk-infra-sleet;
  LA-CAL test harness (Python):
  https://github.com/open-arsenal/ams-gra-hello-world-sk-test-la-cal-harness
- Companions: [`a_gra_standard_review.md`](a_gra_standard_review.md),
  [`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md),
  [`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md)
- In-repo:
  [`pubsub_interaction_guide.md`](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md),
  [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md),
  [`agra_example/README.md`](../../../subprojects/PYRAMID/pim/agra_example/README.md),
  `subprojects/PCL/include/pcl/pcl_plugin.h` (transport ABI);
  websocketpp + asio are CMake-fetched at the repo root (`CMakeLists.txt`,
  currently consumed only by `ame_foxglove`)
