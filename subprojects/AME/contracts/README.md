# AME contract tree

This directory holds the PYRAMID contract files (`.proto`) that AME needs in
order to generate its bindings. PYRAMID turns a contract tree into the C++
types, codecs and service facades that AME's code includes, and it generates
from exactly one tree at a time, named by the CMake variable
`PYRAMID_PROTO_DIR`.

## Why AME has its own copy

PYRAMID's reusable machinery does not define a product contract of its own.
When PYRAMID's proof layer is built, it points `PYRAMID_PROTO_DIR` at the proof
contracts, and AME used to inherit that default without asking for it. The
effect was that AME could not be built at all unless PYRAMID's proof layer —
its example components, integration proofs and their tests — was built too,
even though AME uses almost none of it.

Keeping AME's own contracts here removes that. AME now names the tree it
depends on, which is the arrangement PYRAMID's own
[generated bindings guide](../../PYRAMID/doc/architecture/generated_bindings.md)
describes for consumers: "Select a versioned contract tree with
`PYRAMID_PROTO_DIR`".

## What is here

| File | Why AME needs it |
|------|------------------|
| `pyramid/options/pyramid.options.proto` | Field and message options the other contracts annotate themselves with |
| `pyramid/data_model/pyramid.data_model.base.proto` | Base identifier and timestamp types every other data model imports |
| `pyramid/data_model/pyramid.data_model.common.proto` | Shared enumerations and value types |
| `pyramid/data_model/pyramid.data_model.autonomy.proto` | The autonomy data model: plans, actions, execution state |
| `pyramid/components/pyramid.components.autonomy_backend.services.provided.proto` | The service AME implements as an autonomy backend |
| `pyramid/components/pyramid.components.agra_ma_grounding.services.provided.proto` | Needed only when `AME_BUILD_AGRA_MA_BRIDGE=ON` |

The set is closed: every non-`google/protobuf` import in these files resolves to
another file in this directory. `AmeContractTree.EveryImportResolvesInsideAmesOwnTree`
checks that, so adding a contract with an unmet import fails the test suite
rather than the build.

## Which tree a build actually uses

| Configuration | Tree used |
|---------------|-----------|
| `PYRAMID_BUILD_PROOFS=ON` (the default) | PYRAMID's proof contracts, which are a superset of this directory |
| `PYRAMID_BUILD_PROOFS=OFF` with AME enabled | This directory |

The selection is made in the workspace's top-level `CMakeLists.txt`.

## Keeping the two copies in step

Four of the files here also exist inside PYRAMID's proof contracts. That is the
accepted cost of AME being able to build without the proof layer, but it means a
change made in one place has to be made in the other.

`AmeContractTree.SharedFilesAreIdenticalToPyramidProofContracts` compares the two
copies byte for byte and fails if they have diverged, so the two configurations
above can never generate different bindings without a test noticing. If that test
fails, copy the changed file from `subprojects/PYRAMID/proofs/contracts/proto`
into the matching place here, and check that AME still builds against it.

When adding a file to this directory, add its path to `sharedContractFiles()` in
[`subprojects/AME/tests/test_contract_tree.cpp`](../tests/test_contract_tree.cpp)
so that it is covered by all three checks.
