# A-GRA-vocabulary port-grammar fixture

> **Not an OMS wire contract.** This hand-authored tree tests PYRAMID's port
> grammar, classifier, routing metadata, and generated JSON/FlatBuffers
> bindings using A-GRA-like names. It was not derived from the formal A-GRA
> XSD, has no authoritative `wire_names.json`, and must not be used to claim
> OMS JSON or A-GRA wire compatibility. Use the proven UCI 2.5 P1 seam for
> current OMS/CAL work; see
> [`doc/architecture/oms_agra_compatibility.md`](../../doc/architecture/oms_agra_compatibility.md).

This fixture originated in Phase B of the SHM/UDP proving plan (the retired
`agra_pubsub_shm_udp_proving_plan.md` — design intent summarised in
[`doc/plans/PYRAMID/README.md`](../../../../doc/plans/PYRAMID/README.md),
full text in git history):
a hand-authored, grammar-conforming, options-stamped proto tree carrying an
A-GRA-vocabulary example contract, so the pubsub plan's §7 "mostly renaming,
not re-plumbing" bridge argument has a concrete artefact behind it.

## Why hand-authored

`mbse/test.json` is exported from the SysML model; hand-inserting an A-GRA
package there would fake a model round-trip this repo doesn't own. A
hand-written tree is a supported pipeline input (the legacy `proto/` tree is
one), and it faithfully mirrors the real situation an `agra_c2_bridge` drop
would be in: an external-standard-shaped contract entering the toolchain
from outside the model. If/when the SysML model grows an A-GRA package,
this tree regenerates from it via the normal MBSE path and this directory is
superseded.

## Vendored files

`pyramid/options/pyramid.options.proto` and
`pyramid/data_model/pyramid.data_model.{base,common}.proto` are byte-identical
copies of the `pim/test/` originals, guarded by
[`test_agra_example.py`](../test_agra_example.py)'s checksum test so they
cannot drift silently. If that test fires, re-vendor — never fork.

## Vocabulary and shape

One provided component (`agra.mission_autonomy`, the MA side) and one
consumed mirror (`agra.c2_station`), C2 <-> MA per
[`a_gra_e2e_worked_example.md`](../../../../doc/research/AME/a_gra_e2e_worked_example.md):

| A-GRA concept | This contract | Notes |
|---|---|---|
| `MA_Action` (Command-2 tasking) | `agra.data_model.MA_Action` | `action_type` (FIND_SEARCH, MONITOR_OBSERVE), `target_object`, `action_constraints` (EMCON/keep-out strings) |
| `ActionCancelCommand` | `MAAction_Service_Request.cancel` (`Identifier`) | the request wrapper's cancel variant |
| `MA_ActionStatus` / `ActionProcessingState` | `MAAction_Service_Requirement.ma_action_status` (`pyramid.data_model.common.Requirement`, carrying `Achievement`'s acceptance layer: RECEIVED/REJECTED + reason) | an ordinary requirement transition, per the pubsub plan's §3 mapping |
| `MA_ActionPlan` (Data-1 publication) | `agra.data_model.MA_ActionPlan` | plan id, correlated action id, summary -- what C2 reads before approving |
| Correlated request/requirement pair (Command-2/ActionRequest-2) | `MAAction_Service` (Create/Read/Update/Cancel) | topics `agra.ma_action.request` / `agra.ma_action.requirement`, RELIABLE (intended transport: SHM) |
| Information port (Data-1) | `MAActionPlan_Service` (Read) | topic `agra.ma_action_plan.information`, **BEST_EFFORT** (intended transport: UDP) |

The BEST_EFFORT stamp on the information topic is deliberate: it exercises
the transport-codec-plugin-system's §5.4 reconciliation rule (contract floor
<= transport ceiling) in the passing direction against a real BEST_EFFORT
transport, while the RELIABLE pair over the same transport is meant to be
the negative test that fails closed (Phase D).

## Topic naming vs A-GRA's own convention

A-GRA's own topic discipline (MA-L1-013: topic name == wrapped message type
name) differs from this repo's `<project>.<interface>.<role>` scheme used
throughout `pim/test/` and here. The `pyramid.options.pyramid_op` option
carries the topic string verbatim, so an A-GRA-conformant name (e.g. just
`MA_Action`) is *expressible* today by any generator or hand-authored
contract that wants it -- this example keeps the repo's own scheme because
it is our contract, not an OTA DMS artefact, and doing otherwise would mix
concerns that belong at the deploy-time bridge boundary instead. Renaming to
match MA-L1-013 exactly is a deploy-time bridge concern for a future
`agra_c2_bridge`, not a mechanism this example tree needs to build.

## Classifier fallback

Both services conform to the port grammar (4-rpc Request shape for
`MAAction_Service`, 1-rpc Information shape for `MAActionPlan_Service`) and
carry explicit `pyramid.options.pyramid_op` options, exercising the
authoritative Layer-2 path. [`test_agra_example.py`](../test_agra_example.py)
additionally strips the options from a copy of the provided-side proto and
confirms the same two services still classify correctly via the Layer-1
classifier fallback on RPC signatures alone.

## Generating bindings

```sh
python3 pim/generate_bindings.py pim/agra_example <out-dir> --languages cpp --backends json
```

produces a `binding_manifest.json` whose `endpoint_requirements`/`topics`
carry the three topics above with their distinct reliability floors
(`reliable`, `reliable`, `best_effort`), and (`--languages ada`) generates
GNAT-project-shaped Ada bindings; object-compiling those requires
GNAT/gprbuild, which this environment does not have installed -- a carried
note, same as the rest of this plan's Ada/Windows parity story.
