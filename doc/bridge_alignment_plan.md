# Bridge Alignment Plan — Original ↔ PYRAMID Proto Formats

> **Status**: Draft — created during service generator pipeline update.
> **Date**: 2026-03-25

---

## 1. Context

Two proto schema worlds co-exist in the codebase:

| Schema | Location | Owner | Consumers |
|--------|----------|-------|-----------|
| **Original** (reverse-engineered) | `proto/example/tactical_objects.proto` | C++ implementation | `tests/tactical_objects/`, `standalone_bridge.cpp`, `TacticalObjectsCodec` |
| **PYRAMID standard** (intended) | `proto/pyramid/components/tactical_objects/services/{provided,consumed}.proto` + `proto/pyramid/data_model/` | PIM generators | Ada generated bindings, `test_generated_bindings`, Ada E2E clients |

### Original proto types (C++ implementation)

- `ObjectType`, `Affiliation`, `BattleDimension` (domain-specific enums)
- `Position` (lat/lon/alt degrees), `Velocity`, `BoundingBox`
- `TacticalObject`, `Observation`, `EntityUpdateFrame`
- Services: `TacticalObjectService` (CRUD), `ZoneService` (CRUD), `ObservationIngressService`

### PYRAMID data model types (new)

- `StandardIdentity`, `BattleDimension`, `DataPolicy` (standard enums, different naming)
- `GeodeticPosition` (lat/lon radians), `Entity`, `Achievement`
- `ObjectDetail`, `ObjectInterestRequirement`, `ObjectEvidenceRequirement`, `ObjectMatch`
- Services: `Object_Of_Interest_Service`, `Matching_Objects_Service`, `Object_Evidence_Service`, etc.

---

## 2. Current State

### What works (510/511 CTest tests pass)

- **Ada generated-bindings unit test** (`test_generated_bindings`) — codec round-trips, wire-name constants, dispatch ✓
- **Ada socket E2E** (`tobj_ada_socket_e2e`) — Ada client connects to C++ `tobj_socket_server` over TCP, sends typed `create_requirement` ✓
- **All 509 C++ gtest tests** — unit, integration, component, codec, streaming, E2E ✓

### What fails

- **Ada active-find E2E** (`tobj_ada_active_find_e2e`) — 3-process test: C++ server → `standalone_bridge` → Ada client

**Root cause**: JSON format mismatch at the bridge.

The Ada `Invoke_Create_Requirement` now serializes `Object_Interest_Requirement` via the
data model codec (`Pyramid_Data_Model_Tactical_Types_Codec.To_Json`), producing
proto-derived nested JSON:

```json
{"base":{"update_time":0.0,"id":"","source":""},"status":{...},"source":"Source_Unspecified","policy":"Policy_Obtain","dimension":null,...}
```

But `standalone_bridge.cpp` expects the flat `json_schema.py` format:

```json
{"policy":"DATA_POLICY_OBTAIN","identity":"STANDARD_IDENTITY_HOSTILE","dimension":"BATTLE_DIMENSION_SEA_SURFACE","min_lat_rad":0.87,"max_lat_rad":0.91,...}
```

---

## 3. Wire Format Layers

```
Ada Client (PYRAMID types)
    │
    │  Object_Interest_Requirement → To_Json (data model codec)
    ▼
[GAP] ──── standalone_bridge.cpp expects json_schema.py flat format
    │
    │  Internal JSON (TacticalObjectsCodec format)
    ▼
TacticalObjectsComponent (original types)
```

The `standalone_bridge` currently performs TWO translations:

1. **Frontend** (Ada-facing): `json_schema.py` flat JSON ↔ internal `TacticalObjectsCodec` JSON
2. **Backend** (server-facing): Internal JSON + binary `StreamingCodec` frames ↔ topics

The gap is that the Ada client no longer speaks `json_schema.py` flat JSON — it speaks
proto-derived data model JSON.

---

## 4. Options

### Option A: Update the bridge to accept both formats (short-term fix)

Add a format-detection layer in `standalone_bridge.cpp` that recognises the
proto-derived JSON structure and translates it alongside the existing flat format.

**Pros**: Minimal Ada-side changes; both old and new clients work.
**Cons**: Bridge becomes more complex; two JSON dialects to maintain.

```cpp
// In frontend_create_requirement:
if (req_j.contains("base")) {
    // PYRAMID data-model format
    policy = req_j.value("policy", "Policy_Unspecified");
    // translate Policy_Obtain → DATA_POLICY_OBTAIN → active_find, etc.
} else {
    // Legacy json_schema.py flat format (existing code)
    policy = req_j.value("policy", "DATA_POLICY_OBTAIN");
}
```

### Option B: Ada client uses Json_Codec for bridge-facing calls (short-term)

Keep the `Json_Codec` (json_schema.py format) for `Send_Create_Requirement` in
`tobj_interest_client.adb` — use the flat `Codec.Create_Requirement_Request` record
and `Codec.To_Json` instead of the typed `Object_Interest_Requirement`.

The `Invoke_Create_Requirement` binding would need an overload that accepts a raw
`String` payload (or the client builds the string via `Codec.To_Json` and calls
the low-level PCL invoke directly).

**Pros**: No bridge changes; preserves existing bridge protocol.
**Cons**: Ada client carries two representations — typed data model types for
internal logic and flat Json_Codec types for bridge communication.

### Option C: Align C++ implementation to PYRAMID types (long-term)

Migrate `tests/tactical_objects/` C++ implementation to use the PYRAMID data model
types generated from `proto/pyramid/data_model/`. This would involve:

1. Replacing `Affiliation` → `StandardIdentity`, `Position` → `GeodeticPosition`, etc.
2. Updating `TacticalObjectsCodec` to serialize/deserialize PYRAMID types
3. Updating `standalone_bridge.cpp` to speak the PYRAMID JSON format natively
4. Updating all C++ tests to use the new types

**Pros**: Single source of truth; no bridge translation needed.
**Cons**: Large migration; touches 23 test files + component implementation.

### Option D: Generate a PYRAMID-aware bridge from proto (long-term)

Use the PIM generators to auto-generate a bridge that translates between
PYRAMID data model JSON and internal format, driven by both proto schemas.

**Pros**: Bridge stays in sync with proto changes automatically.
**Cons**: Requires new generator capability.

---

## 5. Recommended Path

| Phase | Action | Effort |
|-------|--------|--------|
| **Now** | **Option B** — Restore `Json_Codec` usage in `tobj_interest_client.adb` for bridge-facing calls, keeping typed internal logic. This unblocks the active-find E2E immediately. | Small |
| **Next** | **Option A** — Add format detection in `standalone_bridge.cpp` so it accepts both JSON dialects. This allows pure-typed Ada clients to work through the bridge. | Medium |
| **Later** | **Option C** — Incrementally align C++ implementation to PYRAMID types, starting with shared enums and working toward full type alignment. Use `proto/example/tactical_objects.proto` as the mapping reference. | Large |

---

## 6. Files Affected

### Bridge (C++)
- `tests/tactical_objects/standalone_bridge.cpp` — format detection / dual-dialect
- `tests/tactical_objects/tobj_socket_server.cpp` — if service handler parsing changes

### Ada clients
- `examples/ada/tobj_interest_client.adb` — bridge-facing serialization
- `examples/ada/tobj_evidence_provider.adb` — observation publishing format
- `examples/ada/ada_active_find_e2e.adb` — E2E test assertions

### Generators
- `pim/json_schema.py` — enum naming conventions (fixed)
- `pim/ada_service_generator.py` — JsonCodecGenerator GNATCOLL compatibility (fixed)

### Proto references
- `proto/example/tactical_objects.proto` — original format reference
- `proto/pyramid/data_model/{common,tactical}.proto` — PYRAMID standard types
- `proto/pyramid/components/tactical_objects/services/{provided,consumed}.proto` — PYRAMID services

---

## 7. Enum Mapping Reference

| PYRAMID Standard | Original (C++) | Notes |
|------------------|-----------------|-------|
| `StandardIdentity.STANDARD_IDENTITY_HOSTILE` | `Affiliation::Hostile` | Same concept, different naming |
| `StandardIdentity.STANDARD_IDENTITY_FRIENDLY` | `Affiliation::Friendly` | |
| `StandardIdentity.STANDARD_IDENTITY_ASSUMED_FRIENDLY` | `Affiliation::AssumedFriend` | |
| `BattleDimension.BATTLE_DIMENSION_GROUND` | `BattleDimension::Ground` | Same enum name, different prefix |
| `BattleDimension.BATTLE_DIMENSION_SEA_SURFACE` | `BattleDimension::SeaSurface` | |
| `DataPolicy.DATA_POLICY_OBTAIN` | N/A (query_mode="active_find") | Bridge translates |
| `DataPolicy.DATA_POLICY_QUERY` | N/A (query_mode="read_current") | Bridge translates |

---

## 8. Test Impact

| Test | Status | Action needed |
|------|--------|---------------|
| `ada_generated_bindings_roundtrip` (509) | ✅ Pass | None |
| `tobj_ada_socket_e2e` (510) | ✅ Pass | None |
| `tobj_ada_active_find_e2e` (511) | ❌ Fail | Bridge JSON format mismatch — apply Option B or A |
| All C++ tests (1–508) | ✅ Pass | None until Option C alignment |
