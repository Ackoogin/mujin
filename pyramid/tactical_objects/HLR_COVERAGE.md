# HLR Requirements Coverage

Traceability of HLR requirements (TOBJ.001–TOBJ.053, RESP.001–RESP.017) to tests against the TacticalObjectsComponent (PCL integration layer).

## PCL Component Test Coverage

Tests against the PCL component exercise the full stack via services (`create_object`, `update_object`, `delete_object`, `query`, `get_object`, `upsert_zone`, `remove_zone`) and subscriber (`observation_ingress`).

| TOBJ | Description | PCL Component Test(s) |
|------|-------------|----------------------|
| TOBJ.001 | Object types | DirectCreateViaService, AllObjectTypesViaCreate |
| TOBJ.002 | Object properties | DirectCreateViaService, DirectUpdateViaService, FullMilitaryClassificationViaCreate |
| TOBJ.003 | Stable internal UUID | DirectCreateViaService, DirectDeleteViaService |
| TOBJ.004 | External ID association | ExternalIdAndLineageFromCorrelation |
| TOBJ.007 | Criteria query | QueryViaService, RegionQueryViaService, QueryBySourceRef |
| TOBJ.008 | Single object query | GetObjectViaService, QueryBySourceRef |
| TOBJ.009 | Region query | RegionQueryViaService |
| TOBJ.010 | Temporal query | TemporalQueryViaService |
| TOBJ.015 | Behavior estimation | BehaviorAndOperationalStateViaUpdate |
| TOBJ.016 | Operational state | BehaviorAndOperationalStateViaUpdate |
| TOBJ.017 | State freshness | TemporalQueryViaService |
| TOBJ.018 | Multi-source ingest | EvidenceIngressViaSubscriber, BurstObservationIngress, MixedEvidenceAndDirectUnderExecutor |
| TOBJ.019 | Confidence tracking | ConfidenceFromCorrelation |
| TOBJ.020 | Evidence lineage | ExternalIdAndLineageFromCorrelation |
| TOBJ.021 | Entity correlation | EvidenceIngressViaSubscriber |
| TOBJ.022 | Entity integration | ExternalIdAndLineageFromCorrelation |
| TOBJ.025 | Battle dimension | DirectCreateViaService |
| TOBJ.026 | Affiliation | DirectCreateViaService, DirectUpdateViaService |
| TOBJ.027 | Role/function | DirectCreateViaService |
| TOBJ.028 | Status | FullMilitaryClassificationViaCreate |
| TOBJ.029 | Echelon | FullMilitaryClassificationViaCreate |
| TOBJ.030 | Indicator flags | FullMilitaryClassificationViaCreate |
| TOBJ.031 | Mobility | FullMilitaryClassificationViaCreate |
| TOBJ.032 | Source SIDC preservation | SourceSidcPreserved |
| TOBJ.033 | Geometry types | ZoneUpsertViaService |
| TOBJ.034 | Zone semantics | ZoneUpsertViaService, ZoneRemoveViaService |
| TOBJ.036 | Zone relationship queries | RegionQueryViaService |
| TOBJ.037 | Temporal zones | ZoneUpsertViaService |
| TOBJ.038 | Thousands of entities | RapidServiceCalls |
| TOBJ.039 | Sparse component storage | SparseObjectCreation |
| TOBJ.040 | Spatial indexing | RegionQueryViaService |
| TOBJ.041 | Incremental updates | RapidServiceCalls |
| TOBJ.042 | Bulk ingest | BurstObservationIngress |
| TOBJ.045 | Snapshot and event access | ConfigureCreatesAllPorts, QueryViaService, EvidenceIngressViaSubscriber |

## ActiveFind Scenario Coverage (PCL + E2E)

The following entries cover the ActiveFind (solution dependency) scenario end-to-end, including the subscribe_interest service, evidence requirement publication, correlation, and entity streaming.

| TOBJ | Description | Test(s) | LLR(s) |
|------|-------------|---------|--------|
| TOBJ.005 | Interest requirement (ActiveFind mode) | ActiveFindModeDispatch (HLR), ActiveFindSolutionDrivesEvidenceProvider (E2E) | 066, 068, 070 |
| TOBJ.025 | Battle dimension (interest filter) | BattleDimensionFilterInInterestMatching (Matching) | 069 |
| TOBJ.045 | Event access (streaming to interest) | AdaClientZoneInterestReceivesEntityEvidence (E2E) | 068 |
| TOBJ.047 | Tactical object solution | ActiveFindModeDispatch (HLR), ActiveFindSolutionDrivesEvidenceProvider (E2E), SolutionResponseEncoding (HLR) | 066, 070, 071 |
| TOBJ.048 | Derived evidence requirement | EvidenceRequirementPublication (HLR), DerivedEvidenceCarriesCriteria (InterestManager) | 067, 072 |
| TOBJ.049 | Requirement/evidence traceability | ActiveFindSolutionDrivesEvidenceProvider (E2E) | 070 |

## Requirements Covered by Unit Tests (Not Yet in PCL Runtime)

The following TOBJs are implemented in engines (InterestManager, TacticalHistory, RelationshipIndex) that are not yet wired into TacticalObjectsRuntime. They are covered by dedicated unit tests:

| TOBJ | Description | Unit Test File |
|------|-------------|----------------|
| TOBJ.005 | Interest requirement | Test_InterestManager, Test_InterestManager_Matching |
| TOBJ.006 | Interest cancellation | Test_InterestManager |
| TOBJ.011 | Object relationships | Test_RelationshipIndex |
| TOBJ.012 | Hierarchical relationships | Test_RelationshipIndex |
| TOBJ.013 | Tactical relationships | Test_RelationshipIndex |
| TOBJ.014 | Relationship confidence | Test_RelationshipIndex |
| TOBJ.023 | Merge and split | Test_CorrelationEngine |
| TOBJ.024 | Constraints | Test_CorrelationEngine / Test_Runtime |
| TOBJ.035 | Geodesic reasoning | Test_ZoneEngine |
| TOBJ.043 | Capability reporting | Test_TacticalObjectsRuntime |
| TOBJ.044 | Missing information | Test_TacticalObjectsRuntime |
| TOBJ.046 | Requirement achievability | Test_TacticalObjectsRuntime |
| TOBJ.047 | Tactical object solution | Test_TacticalObjectsRuntime, Test_TacticalObjectsComponent_HLR, Test_TacticalObjects_E2E |
| TOBJ.048 | Derived evidence requirement | Test_InterestManager, Test_TacticalObjectsComponent_HLR |
| TOBJ.049 | Requirement/evidence traceability | Test_TacticalObjectsRuntime, Test_TacticalObjects_E2E |
| TOBJ.050 | Measurement criteria | Test_InterestManager |
| TOBJ.051 | Progress reporting | Test_InterestManager |
| TOBJ.052 | Object probability densities | Test_TacticalObjectsRuntime |
| TOBJ.053 | Capability progression | Test_TacticalObjectsRuntime |

## Test Files

| File | Purpose |
|------|---------|
| `Test_TacticalObjectsComponent.cpp` | Core PCL component tests |
| `Test_TacticalObjectsComponent_Robustness.cpp` | Stress and robustness |
| `Test_TacticalObjectsComponent_HLR.cpp` | Explicit HLR trace tags, coverage gaps, ActiveFind service-level tests |
| `Test_TacticalObjects_E2E.cpp` | End-to-end: interest → evidence → correlation → streaming |
| `Test_InterestManager.cpp` | Interest lifecycle, evidence derivation, measurement, progress |
| `Test_InterestManager_Matching.cpp` | Interest-entity matching including battle dimension filter |

## Trace Tag Format

Tests use requirement tags in comments:

```
///< TOBJ.0XX: Brief description.
///< RESP.0XX: Responsibility description.
```

Run tests with filter to verify a specific requirement:

```bat
build\tests\Release\test_tobj_component_hlr.exe --gtest_filter=*HLR*
```
