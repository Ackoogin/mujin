# PYRAMID Port-Grammar `Requirement` to `Entity` Rename Plan

**Status:** implemented (commit `57eb50f`); retained as the design record for
the rename. The status line was corrected on 2026-07-21, when a tracker review
found both this plan and the WS-H entry still describing the work as proposed.  
**Created:** 2026-07-17  
**Tracker:** [PYRAMID consolidated TODO](../../todo/PYRAMID/TODO.md)

## 1. Summary

PYRAMID Request ports currently call the value returned by `Read` and accepted
by `Update` a `Requirement`. The same word also names the provider-to-consumer
pub/sub leg:

```proto
message Example_Service_Request { ... }
message Example_Service_Requirement { ... }

service Example_Service {
  rpc Create(Example_Service_Request) returns (Ack);
  rpc Read(Query) returns (stream Example_Service_Requirement);
  rpc Update(Example_Service_Requirement) returns (Ack);
  rpc Cancel(Identifier) returns (Ack);
}
```

The new grammar will call that value and leg `Entity`:

```proto
message Example_Service_Request { ... }
message Example_Service_Entity { ... }

service Example_Service {
  rpc Create(Example_Service_Request) returns (Ack);
  rpc Read(Query) returns (stream Example_Service_Entity);
  rpc Update(Example_Service_Entity) returns (Ack);
  rpc Cancel(Identifier) returns (Ack);
}
```

The change is a coordinated contract, wire-name, generated API, deployment
configuration, and documentation rename. It is not a literal replacement of
every occurrence of the English word "requirement".

## 2. Decision and scope boundary

This plan renames the **port-grammar role** from `Requirement` to `Entity`.
It does not rename requirements that are part of the mission domain,
requirements engineering, or deployment capability checking.

The scope boundary is necessary because the common data model already defines
both `Requirement` and `Entity`:

- `Requirement` represents a requested outcome and its achievement status.
- `Entity` is the common identified base type. `Request`, `Information`,
  `Achievement`, and `Requirement` inherit from it.

Changing every data-model `Requirement` to `Entity` would create a type-name
collision. Changing the MBSE generator to classify every type derived from
`Entity` as the result/update payload would also make Request, Information,
and result payloads indistinguishable.

The MBSE generator may continue to recognize the existing domain
`Requirement` inheritance when it selects a result payload. Its emitted
wrapper, topic role, manifest leg, and generated API must use `Entity`.
A separate data-model redesign would be needed if the underlying domain
`Requirement` hierarchy is also to be removed.

### 2.1 Required rename

| Current grammar term | New grammar term |
|----------------------|------------------|
| `X_Service_Requirement` | `X_Service_Entity` |
| `Read(...) returns (stream X_Service_Requirement)` | `Read(...) returns (stream X_Service_Entity)` |
| `Update(X_Service_Requirement)` | `Update(X_Service_Entity)` |
| derived topic `<project>.<interface>.requirement` | `<project>.<interface>.entity` |
| interaction leg name `requirement` | `entity` |
| exclusive group suffix `requirement_leg` | `entity_leg` |
| configuration key `requirement_leg` | `entity_leg` |
| generated identifiers containing the grammar-role `Requirement` | corresponding identifiers containing `Entity` |
| "request/requirement pair" | "request/entity pair" |

The RPC method names `Create`, `Read`, `Update`, and `Cancel` do not change.
The service remains a Request port.

### 2.2 Terms that remain

The following examples are outside this rename unless a separate domain
change is approved:

- the common data-model messages `Requirement`, `RequirementSolution`, and
  `DependencyRequirement`;
- domain types such as `ObjectEvidenceRequirement`,
  `PlanningRequirement`, and `ExecutionRequirement`;
- topic names such as `standard.evidence_requirements`;
- fields such as `requirement_id` when they identify a domain requirement;
- `EndpointRequirement`, which describes what a routeable endpoint needs;
- HLR, LLR, requirement traceability, and other requirements-engineering
  documents and tools;
- legacy RPCs such as `CreateRequirement` or `UpdateRequirement` whose name
  describes a domain operation rather than the new four-method port grammar;
- A-GRA and UCI schema element names supplied by an external standard.

The implementation must use a reviewed allowlist for these retained uses.
A repository-wide search-and-replace is not acceptable.

## 3. Current-state inventory

The impact analysis on 2026-07-17 found:

- 259 `_Service_Requirement` message definitions across 26 tracked proto
  contract files;
- 863 `_Service_Requirement` references across 47 tracked files;
- 54 `requirement_leg` references across 10 tracked files;
- direct grammar knowledge in seven generator modules;
- checked-in PIM test, UCI P1, A-GRA P2, A-GRA P3, example, and interaction
  facade contracts;
- generated C++ and Ada facades, component skeletons, codecs, manifests,
  examples, and test harnesses;
- LA-CAL topic allowlists and routes that use `.requirement`;
- architecture, guide, plan, research, and TODO documents outside the
  PYRAMID source directory.

No existing message named `X_Service_Entity` was found, so the wrapper rename
does not currently collide with another service wrapper.

The authoritative grammar description is
[`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md).
The main implementation sources are:

| Area | Source of truth |
|------|-----------------|
| Service-shape recognition and derived topics | `subprojects/PYRAMID/pim/proto_parser.py`, `binding_contract.py` |
| MBSE contract generation | `subprojects/PYRAMID/pim/mbse/proto_generator.py` |
| Large A-GRA seam generation | `subprojects/PYRAMID/pim/gen_interaction_seam.py` |
| C++ interaction facade | `subprojects/PYRAMID/pim/cpp/interaction_facade_gen.py` |
| Ada interaction facade | `subprojects/PYRAMID/pim/ada/interaction_facade_gen.py` |
| OMS JSON wrapper recognition | `subprojects/PYRAMID/pim/cpp/oms_json_codec_gen.py`, `pim/ada/oms_json_codec_gen.py` |
| Generated binding manifest | `subprojects/PYRAMID/pim/generate_bindings.py` and `binding_contract.py` |
| Deployment manifest example | `subprojects/PYRAMID/pim/test_harness/contract_routing_manifest.py` |

## 4. Compatibility impact

### 4.1 Source and ABI compatibility

The generated C++ and Ada APIs will change. This includes wrapper types,
callback parameters, publisher and subscriber helpers, topic constants,
component skeleton signatures, and interaction binding configuration.
Existing source must be regenerated and updated. Existing binaries must not
be mixed with the new generated libraries.

The offline SDK therefore needs a contract or SDK version boundary for this
change.

### 4.2 Topic and routing compatibility

Changing `.requirement` to `.entity` changes the exact topic name used by
transports, broker allowlists, and PCL routes. An old subscriber will not
receive frames from a new publisher.

PCL interaction manifests also expose the leg name and use it to form
exclusive group names. The following values change together:

- topic key and wire name;
- interaction leg name;
- exclusive group name;
- `--realize` leg argument;
- JSON configuration key;
- generated route summaries.

All components in one deployment must use the same contract generation.

Explicit A-GRA or UCI topic names do not necessarily end in `.requirement`.
When a contract uses an authoritative external element name, that wire name
must remain unchanged. Only PYRAMID's internal interaction leg metadata is
renamed to `entity`.

### 4.3 Serialization and schema identity

If fields, field numbers, and oneof membership remain unchanged, protobuf
payload bytes can remain structurally compatible. The protobuf full message
name and descriptors will still change.

PYRAMID codec registration uses generated schema identifiers derived from
type names. Renaming the wrapper therefore changes its schema identity even
when the payload bytes have the same shape. FlatBuffers, Protobuf, ROS2, JSON,
and OMS JSON generation must all be regenerated and tested together.

gRPC service and method paths can remain unchanged because the service and RPC
method names do not change. Generated clients still need rebuilding, and gRPC
reflection will report the new input and output message types.

### 4.4 Checked-in generated contracts

The A-GRA P3 seam is generated by `gen_interaction_seam.py` and guarded by a
byte-stability test. It must be regenerated from the updated generator; its
large generated service files must not be edited by hand.

The A-GRA P3 seam retains selected P2 services. Update the P2 source contract
first, then regenerate P3 so the retained copy and the generated P3 services
use one grammar.

The PIM test tree and component skeleton baselines are also checked in. Their
expected byte changes must be reviewed as one deliberate grammar migration.

## 5. Migration policy

The preferred repository migration is one coordinated breaking change:

1. update the generators and source contracts;
2. regenerate every checked-in contract and binding baseline;
3. update every in-repository consumer and deployment configuration;
4. rebuild and publish the SDK under a new contract version;
5. reject mixed old and new artifacts.

This avoids keeping two grammar terms indefinitely.

If an external deployment requires a rolling upgrade, use a boundary adapter
for one release:

- accept `requirement_leg` as a deprecated configuration alias for
  `entity_leg`, and reject configurations that set both to different values;
- translate the old `.requirement` topic to `.entity` at a single deployment
  boundary;
- record and deduplicate by the existing correlation identifier;
- remove the adapter after every participant has moved to the new SDK.

Do not make every provider publish both topics. That can deliver the same
transition twice and would complicate snapshot and correlation behavior.
Proto does not provide a source-level message alias, so retaining both wrapper
names would also require explicit conversion code and duplicate schema
registration.

## 6. Implementation plan

### Phase 0 — Pin the new grammar contract

1. Add a small grammar fixture whose expected roles are exactly `request`,
   `entity`, and `information`.
2. Add tests for the exact wrapper names, Read and Update signatures, derived
   topic names, interaction leg names, group names, and configuration keys.
3. Add a retained-term allowlist covering the legitimate domain and
   requirements-engineering uses described in section 2.2.
4. Record the contract and SDK version that first uses the new grammar.

**Exit criteria:** the intended rename is executable as tests, and the tests
fail against the current implementation for only the expected grammar terms.

### Phase 1 — Change generator sources of truth

1. Update `mbse/proto_generator.py`:
   - store the result/update payload in an internal `entity` slot;
   - emit `X_Service_Entity`;
   - emit the `entity` topic role;
   - keep the current domain inheritance discriminator until a separate model
     change replaces it.
2. Update `gen_interaction_seam.py` to emit `_Entity` wrappers and use them in
   `Read` and `Update`.
3. Update `binding_contract.py`:
   - derive `.entity`;
   - find explicit or derived entity topics;
   - build an `entity` interaction leg;
   - form `.entity_leg` exclusive group names;
   - preserve the fallback from explicit topic names to the owning `Read`
     method.
4. Update the C++ and Ada interaction facade generators:
   - rename internal variables and public configuration parameters;
   - read `entity_leg`;
   - generate entity-topic publisher and subscriber helpers;
   - keep behavior such as snapshots, correlation filtering, and transition
     fan-out unchanged.
5. Update the C++ and Ada OMS JSON generators to recognize
   `_Service_Entity` wrappers when they select and unwrap service payloads.
6. Update comments, diagnostics, and manifest help text emitted by the
   generators.

**Exit criteria:** a small contract generates only the new grammar and all
generator unit tests pass.

### Phase 2 — Migrate source contracts and regenerate checked-in trees

Update these groups in dependency order:

1. hand-authored A-GRA and UCI example contracts;
2. A-GRA P2 and UCI P1 seam contracts;
3. interaction-facade test fixtures;
4. the MBSE-generated PIM test tree;
5. the A-GRA P3 seam, by rerunning `gen_interaction_seam.py`;
6. component skeleton baselines and OMS JSON golden output affected by wrapper
   recognition.

Preserve all proto field numbers, oneof alternatives, correlation identifiers,
QoS settings, and external-standard wire names.

**Exit criteria:** all checked-in contract trees regenerate deterministically,
and their diffs contain terminology and derived-name changes rather than
payload-shape changes.

### Phase 3 — Migrate generated API consumers and deployment files

1. Regenerate C++ and Ada bindings for each contract profile.
2. Update C++ and Ada examples and test harnesses to use `_Service_Entity`
   types and generated entity-topic helpers.
3. Update component skeleton implementations and handlers.
4. Update interaction binding configuration from `requirement_leg` to
   `entity_leg`.
5. Update PCL routing-manifest generation, `--realize` examples, and route
   assertions.
6. Update LA-CAL allowlists and service configuration from `.requirement` to
   `.entity` where the topic is derived from the PYRAMID grammar.
7. Rebuild SDK templates and packaged-SDK smoke consumers.

Do not rename a legacy `CreateRequirement` or `UpdateRequirement` API merely
because it contains the same word. First prove that it is a projection of the
new Request-port result role. Legacy domain operations remain outside this
workstream.

**Exit criteria:** every in-repository consumer builds against freshly
generated bindings and no deployment file refers to an obsolete grammar
topic or leg.

### Phase 4 — Update documentation

Update:

- the architecture description of the port grammar;
- the pub/sub and interaction facade guide;
- generated-binding and component-authoring documentation;
- A-GRA, UCI, LA-CAL, and SDK examples;
- active plans, research documents, and TODO text that describe the grammar;
- the glossary entry for `port`.

Each document must use "request/entity pair" for the grammar while retaining
"requirement" where it refers to a domain requirement or a formal software
requirement.

**Exit criteria:** a reader who has not followed this work sees one consistent
grammar and an explicit explanation of the retained domain terminology.

### Phase 5 — Validate and release

Run the validation matrix in section 7. Review generated diffs before running
the full build because accidental domain renames are easier to see in the
contract files than in generated C++ or Ada.

Publish the new SDK and contract version only after all profiles pass. Record
the last version that used the old grammar and whether a boundary adapter is
available.

**Exit criteria:** the completion criteria in section 9 are met and mixed
contract generations fail clearly rather than communicating partially.

## 7. Validation matrix

### 7.1 Contract and generator tests

- `subprojects/PYRAMID/pim/test_binding_contract.py`
- `subprojects/PYRAMID/pim/test_component_skeleton_gen.py`
- `subprojects/PYRAMID/pim/test_agra_p3_seam.py`
- `subprojects/PYRAMID/pim/test_oms_json_gen.py`
- `subprojects/PYRAMID/tests/test_binding_manifest.py`
- generic C++ and Ada binding tests
- FlatBuffers and Protobuf generation tests
- gRPC generation and smoke tests
- ROS2 IDL, codec, and transport-semantic tests

Assertions must cover:

- exactly two Request-port legs named `request` and `entity`;
- Read and Update use the same `_Service_Entity` type;
- derived topics end in `.request` and `.entity`;
- explicit external topic names remain exact;
- entity leg directions are inverse on provided and consumed services;
- schema IDs and manifests consistently name the new wrapper;
- OMS JSON continues to unwrap the same domain payload;
- correlation and snapshot behavior is unchanged.

### 7.2 Generated facade and route tests

- build and run `test_pcl_generated_interaction_facade`;
- run the A-GRA interaction facade example with RPC, pub/sub, and mixed
  realizations;
- run the contract-routing validation harness;
- run the shared-memory interchange harness;
- run the LA-CAL seam test with the updated topic allowlists;
- object-compile the generated Ada bindings and facade proofs.

### 7.3 Packaging and full regression

1. Regenerate the ordinary, P1, P2, and P3 binding outputs.
2. Build the affected generated libraries and codec plugins.
3. Package the offline SDK and run its import, plugin-load, and representative
   payload smoke tests.
4. Run the PYRAMID Python test suites.
5. Run the full Release CTest suite:

   ```bat
   ctest --test-dir build --output-on-failure -C Release
   ```

6. Run an allowlist-based terminology audit:
   - no `_Service_Requirement` in grammar service packages;
   - no derived `.requirement` topic;
   - no interaction leg or configuration key named `requirement_leg`;
   - all remaining occurrences have an reviewed domain, external-standard,
     traceability, or historical reason.

## 8. Risks and controls

| Risk | Control |
|------|---------|
| A literal replacement corrupts domain types or formal requirement documents | Use structural searches and a reviewed retained-term allowlist |
| `Entity` is used as the MBSE discriminator and matches too many payloads | Keep the existing result-payload discriminator separate from the emitted grammar role |
| Old and new topics silently partition a deployment | Version the contract and validate that every routed endpoint uses one generation |
| Dual publication produces duplicate transitions | Prefer a single boundary adapter with correlation-based deduplication |
| P3 hand edits drift from its generator | Change `gen_interaction_seam.py` and rerun the byte-stability test |
| External A-GRA/UCI wire names are accidentally normalized | Preserve explicit topic options and test them byte for byte |
| Wrapper rename changes schema identity but tests only compare payload bytes | Assert schema IDs, manifests, codec registration, and runtime dispatch |
| Generated C++ passes while Ada or a transport backend remains stale | Run the cross-language and backend matrix before release |
| Active A-GRA work creates large overlapping generated diffs | Complete or checkpoint that work, then perform the rename as a separate reviewed change |

## 9. Completion criteria

The workstream is complete when:

1. The documented grammar roles are `request`, `entity`, and `information`.
2. Every Request port uses `_Service_Request` for Create and
   `_Service_Entity` for Read and Update.
3. Every derived correlated pair uses `.request` and `.entity`.
4. Binding manifests and deployment configuration use `entity` and
   `entity_leg`.
5. C++ and Ada generated APIs, examples, and component skeletons use the new
   grammar term.
6. PIM, P1, P2, and P3 contracts regenerate deterministically.
7. External-standard topic names and legitimate domain requirements are
   unchanged.
8. Codec, transport, route, SDK, and full CTest validation pass.
9. The terminology audit has no unexplained old grammar-role occurrences.
10. The SDK and contract version clearly identify the breaking change.

