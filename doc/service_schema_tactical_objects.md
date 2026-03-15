# Service Schema Options for PCL / Tactical Objects

## 1. Problem Statement

The Tactical Objects subsystem currently defines its service contract implicitly through:

- C++ structs in `TacticalObjectsTypes.h` (ObjectDefinition, ObjectUpdate, QueryRequest, etc.)
- A binary streaming codec (`StreamingCodec.h`) with hand-rolled field masks
- A JSON codec (`TacticalObjectsCodec.h`) using nlohmann::json
- PCL service handlers in `TacticalObjectsComponent.h` that accept opaque `pcl_msg_t` blobs

There is no single, canonical **Interface Definition Language (IDL)** artefact that a downstream consumer (ROS2 node, gRPC client, web dashboard, test harness) can use to generate bindings. Every new transport requires hand-writing a codec.

### Prior Art: SysML-to-Code Pipeline

An existing code generation pipeline transforms SysML/UML models (XMI from MagicDraw/Cameo) into multiple target languages. This pipeline already establishes key patterns that any service schema option must accommodate:

- **EntityActions CRUD semantics** — a standard `create`/`read`/`update`/`delete` contract for all Entity-derived types
- **Entity vs. Value distinction** — properties typed with Entity-derived classes become service operations, not message fields
- **Service inheritance** — child services inherit parent entity properties and RPCs
- **Flow direction** — `inout` (full CRUD), `out` (read-only), `in` (write-only) controls which operations are generated
- **Multi-target generation** — `.proto`, C++14 headers, Python dataclasses, Ada specs/stubs from a single JSON intermediate representation

The pipeline defines the following canonical CRUD signatures:

| Operation | Request | Response | Notes |
|-----------|---------|----------|-------|
| `Create` | `Entity` | `Identifier` | Returns UUID of created entity |
| `Read` | `Query` | `stream Entity` | Filtered read; continuous or one-shot |
| `Update` | `Entity` | `Ack` | Success/failure acknowledgment |
| `Delete` | `Identifier` | `Ack` | Success/failure acknowledgment |

With supporting types:

```
Query     { id: Identifier[*], oneShot: Boolean }
Ack       { success: Boolean }
```

### Goals

| # | Goal |
|---|------|
| G1 | **Protocol-agnostic** — one schema drives all transports |
| G2 | **ROS2 compatible** — maps cleanly to `.msg`/`.srv` files |
| G3 | **gRPC compatible** — maps cleanly to `.proto` files |
| G4 | **User-friendly** — easy to read, version, and extend |
| G5 | **Backward-compatible** — existing C++ structs and PCL glue remain usable |
| G6 | **Consistent with EntityActions** — uses the established CRUD semantics from the SysML pipeline |
| G7 | **Ada/safety-critical compatible** — schema maps to Ada package specs for defence targets |

---

## 2. Current Service Inventory

Services exposed by `TacticalObjectsComponent` today:

| Service | Request | Response | Pattern |
|---------|---------|----------|---------|
| `createObject` | `ObjectDefinition` | `UUIDKey` | Request-Reply |
| `updateObject` | `UUIDKey` + `ObjectUpdate` | `bool success` | Request-Reply |
| `deleteObject` | `UUIDKey` | `bool success` | Request-Reply |
| `getObject` | `UUIDKey` | `EntityRecord` | Request-Reply |
| `query` | `QueryRequest` | `QueryResponse` | Request-Reply |
| `upsertZone` | `ZoneDefinition` | `UUIDKey` | Request-Reply |
| `removeZone` | `UUIDKey` | `bool success` | Request-Reply |
| `subscribeInterest` | `InterestCriteria` | `UUIDKey interest_id` | Request-Reply (subscription setup) |
| `resync` | `UUIDKey interest_id` | snapshot | Request-Reply |
| `entity_updates` | — | `EntityUpdateFrame[]` | Publish (streaming) |
| `observation_ingress` | `ObservationBatch` | — | Subscribe (ingest) |

### Mapping to EntityActions

The current services **almost** follow the EntityActions pattern but diverge in key ways:

| EntityActions | Current Tactical Objects | Gap |
|---------------|--------------------------|-----|
| `Create(Entity) → Identifier` | `createObject(ObjectDefinition) → UUIDKey` | Aligned |
| `Read(Query) → stream Entity` | `getObject(UUIDKey)` + `query(QueryRequest)` | Split into two; no streaming read |
| `Update(Entity) → Ack` | `updateObject(UUIDKey, ObjectUpdate) → bool` | Close, but separate id + delta vs whole entity |
| `Delete(Identifier) → Ack` | `deleteObject(UUIDKey) → bool` | Aligned (bool ≈ Ack) |

Zones follow a parallel pattern (`upsertZone`/`removeZone`) but are not yet unified under EntityActions.

---

## 3. Option A — Protobuf-First with Generated Adapters

### Overview

Define all messages and services in `.proto` files. Use protobuf as the canonical schema and generate:

- C++ structs (via `protoc`)
- gRPC service stubs (via `protoc` + gRPC plugin)
- ROS2 `.msg`/`.srv` files (via a lightweight code-generator script)
- JSON schema (via `protoc-gen-jsonschema` or similar)

### Schema sketch (aligned to EntityActions)

```protobuf
syntax = "proto3";
package pyramid.data_model.base;

// ── EntityActions Base Types ────────────────────────────

message Identifier {
  string uuid = 1;
}

message Query {
  repeated Identifier id = 1;   // filter by specific IDs (empty = all)
  bool one_shot = 2;            // true = single response; false = continuous stream
}

message Ack {
  bool success = 1;
}
```

```protobuf
syntax = "proto3";
package pyramid.components.tactical_objects;

import "pyramid/data_model/base.proto";

// ── Primitives ──────────────────────────────────────────

message Position {
  double latitude  = 1;  // WGS-84
  double longitude = 2;
  double altitude  = 3;
}

message Velocity {
  double north = 1;
  double east  = 2;
  double down  = 3;
}

enum ObjectType {
  OBJECT_TYPE_UNSPECIFIED = 0;
  PLATFORM     = 1;
  PERSON       = 2;
  EQUIPMENT    = 3;
  UNIT         = 4;
  FORMATION    = 5;
  INSTALLATION = 6;
  FEATURE      = 7;
  ROUTE        = 8;
  POINT        = 9;
  AREA         = 10;
  ZONE         = 11;
}

enum Affiliation {
  AFFILIATION_UNSPECIFIED = 0;
  FRIENDLY      = 1;
  HOSTILE       = 2;
  NEUTRAL       = 3;
  UNKNOWN       = 4;
  ASSUMED_FRIEND = 5;
  SUSPECT       = 6;
  JOKER         = 7;
  FAKER         = 8;
  PENDING       = 9;
}

// ... (BattleDimension, Echelon, Mobility, etc.)

message MilClassProfile {
  BattleDimension battle_dim = 1;
  Affiliation     affiliation = 2;
  string          role        = 3;
  MilStatus       status      = 4;
  Echelon         echelon     = 5;
  Mobility        mobility    = 6;
  bool            hq          = 7;
  bool            task_force  = 8;
  bool            feint_dummy = 9;
  bool            installation = 10;
  string          source_sidc  = 11;
}

// ── Entity: TacticalObject ──────────────────────────────
// Entity-derived type → generates CRUD service via EntityActions

message TacticalObject {
  Identifier      id          = 1;   // assigned on Create
  ObjectType      type        = 2;
  Position        position    = 3;
  Velocity        velocity    = 4;
  Affiliation     affiliation = 5;
  MilClassProfile mil_class   = 6;
  string          identity_name = 7;
  repeated string source_refs   = 8;
  string          operational_state = 9;
  string          behavior_pattern  = 10;
}

// ── Entity: Zone ────────────────────────────────────────

message Zone {
  Identifier      id         = 1;
  ZoneType        zone_type  = 2;
  ZoneGeometry    geometry   = 3;
  uint64          active_from  = 4;
  uint64          active_until = 5;
  uint32          priority   = 6;
  string          owner      = 7;
  string          semantics  = 8;
}

// ── Extended Query (superset of base Query) ─────────────

message TacticalObjectQuery {
  pyramid.data_model.base.Query base    = 1;  // ID filter + oneShot flag
  optional ObjectType  by_type          = 2;
  optional Affiliation by_affiliation   = 3;
  optional BoundingBox by_region        = 4;
  optional double      max_age_seconds  = 5;
  optional string      by_source_system = 6;
}

// ── gRPC Services (EntityActions pattern) ───────────────

service TacticalObjectService {
  // EntityActions CRUD — flow: inout
  rpc CreateTacticalObject (TacticalObject)      returns (Identifier);
  rpc ReadTacticalObject   (TacticalObjectQuery)  returns (stream TacticalObject);
  rpc UpdateTacticalObject (TacticalObject)      returns (Ack);
  rpc DeleteTacticalObject (Identifier)          returns (Ack);

  // Streaming (non-CRUD, component-specific)
  rpc SubscribeUpdates     (InterestCriteria) returns (stream EntityUpdateFrame);
  rpc IngestObservations   (stream Observation) returns (CorrelationResult);
}

service ZoneService {
  // EntityActions CRUD — flow: inout
  rpc CreateZone (Zone)       returns (Identifier);
  rpc ReadZone   (Query)      returns (stream Zone);
  rpc UpdateZone (Zone)       returns (Ack);
  rpc DeleteZone (Identifier) returns (Ack);
}
```

### Package namespace convention

Follows the SysML pipeline's established structure:

```
pyramid/
  data_model/
    base.proto              # Identifier, Query, Ack
    common.proto            # Position, Velocity, shared enums
  components/
    tactical_objects.proto  # TacticalObject, Zone, services
    sensors/
      radar_control.proto   # future component
```

### ROS2 mapping

A thin code-generation script converts `.proto` → `.msg`/`.srv`:

| Proto | ROS2 |
|-------|------|
| `message Position` | `Position.msg` |
| `rpc CreateTacticalObject` | `CreateTacticalObject.srv` (request: TacticalObject, response: Identifier) |
| `rpc ReadTacticalObject` | topic `tactical_objects` with `TacticalObject.msg` + `ReadTacticalObject.srv` for one-shot |
| `stream EntityUpdateFrame` | topic `entity_updates` with `EntityUpdateFrame.msg` |

### Pros

- Single source of truth; all codecs generated
- Native gRPC support (streaming, deadlines, interceptors)
- Excellent cross-language support (Python, Java, Go, C#, TypeScript)
- Strong versioning story (field numbers, `reserved`, `oneof`)
- Binary wire format is compact — close to the existing `StreamingCodec`
- EntityActions semantics map directly to gRPC `rpc` definitions
- Existing SysML pipeline `proto_generator.py` can target this schema

### Cons

- Protobuf `optional` semantics differ from C++ `tl::optional` — needs thin mapping layer
- ROS2 `.msg` generation is custom tooling (no off-the-shelf `proto→msg` compiler)
- Adds a build dependency on `protoc`
- Team must learn proto3 idioms (no required fields, default zero values)
- Streaming `Read` maps awkwardly to ROS2 services (must split into topic + one-shot srv)

### User-friendliness rating: **Medium-High**

Proto files are widely known, well-documented, and have IDE support. The EntityActions pattern makes CRUD operations predictable. The main friction is the ROS2 bridge generator.

---

## 4. Option B — ROS2 IDL-First with gRPC Adapter

### Overview

Define all messages in ROS2 `.msg`/`.srv`/`.action` files. Use the ROS2 IDL as the canonical schema and generate:

- C++ structs (via `rosidl`)
- gRPC `.proto` files (via a code-generator script, or manual mirroring)
- PCL codec (hand-written or generated from `.msg` metadata)

### Schema sketch

```
# TacticalObject.msg
string    object_id          # UUID
uint8     object_type        # see ObjectType constants below
uint8     affiliation
string    identity_name
Position  position
Velocity  velocity
MilClassProfile mil_class
string[]  source_refs

uint8 TYPE_PLATFORM=0
uint8 TYPE_PERSON=1
uint8 TYPE_EQUIPMENT=2
# ...
```

```
# CreateTacticalObject.srv (EntityActions: Create)
TacticalObject entity
---
string identifier       # UUID
```

```
# ReadTacticalObject.srv (EntityActions: Read — one-shot only)
string[] ids             # empty = all
uint8 by_type            # 0xFF = unfiltered
uint8 by_affiliation     # 0xFF = unfiltered
---
TacticalObject[] entities
uint32 total
```

```
# UpdateTacticalObject.srv (EntityActions: Update)
TacticalObject entity
---
bool success
```

```
# DeleteTacticalObject.srv (EntityActions: Delete)
string identifier
---
bool success
```

### gRPC mapping

A converter script or hand-maintained `.proto` mirrors the `.msg` types. The gRPC server wraps the PCL service handlers and converts between protobuf and ROS2 C struct representations.

### Pros

- Native ROS2 integration — zero friction for ROS2 consumers
- Familiar to robotics teams; rich ROS2 tooling (rviz, rosbag, ros2 topic echo)
- Existing `.msg` files in `ros2/msg/` and `ros2/srv/` already follow this pattern
- `rosidl` generates C, C++, and Python bindings out of the box

### Cons

- ROS2 IDL is limited compared to protobuf (no `oneof`, no maps, no streaming RPCs)
- **Cannot express streaming `Read`** — ROS2 services are request-reply only; continuous reads must use a separate topic subscription pattern
- gRPC adapter is manual work; no standard `.msg → .proto` tool
- Enum support is weak — constants are `uint8` fields, not first-class enums
- ROS2 dependency required even for non-ROS2 builds
- Versioning story is weaker (no field numbers, no `reserved`)
- Harder for non-ROS2 consumers (web frontends, standalone C++ apps)
- No natural Ada target — ROS2 has no Ada code generator

### User-friendliness rating: **High for ROS2 users, Low for others**

If the primary consumers are ROS2 nodes, this is the most natural fit. But it creates friction for gRPC clients, web dashboards, Ada targets, and standalone tools.

---

## 5. Option C — FlatBuffers Schema with Multi-Target Generation

### Overview

Use FlatBuffers (`.fbs`) as the canonical IDL. FlatBuffers supports:

- Zero-copy deserialization (important for high-frequency streaming)
- gRPC integration (official FlatBuffers gRPC plugin)
- JSON interop (built-in JSON parser/printer)
- Code generation for C++, Python, Java, Go, C#, TypeScript, Rust

A thin script generates ROS2 `.msg` from the `.fbs` schema.

### Schema sketch

```fbs
namespace pyramid.tactical;

enum ObjectType : byte {
  Platform = 0,
  Person,
  Equipment,
  Unit,
  Formation,
  Installation,
  Feature,
  Route,
  Point,
  Area,
  Zone
}

enum Affiliation : byte {
  Friendly = 0,
  Hostile,
  Neutral,
  Unknown,
  AssumedFriend,
  Suspect,
  Joker,
  Faker,
  Pending
}

struct Position {
  latitude:  double;
  longitude: double;
  altitude:  double;
}

struct Velocity {
  north: double;
  east:  double;
  down:  double;
}

table TacticalObject {
  id:             string;       // UUID, assigned on create
  type:           ObjectType;
  position:       Position;
  velocity:       Velocity;
  affiliation:    Affiliation;
  mil_class:      MilClassProfile;
  identity_name:  string;
  source_refs:    [string];
}

table Query {
  id:       [string];           // UUID filter list
  one_shot: bool = true;
}

table Ack {
  success: bool;
}

table Identifier {
  uuid: string;
}

rpc_service TacticalObjectService {
  CreateTacticalObject(TacticalObject): Identifier;
  ReadTacticalObject(Query): TacticalObject;   // no native streaming support
  UpdateTacticalObject(TacticalObject): Ack;
  DeleteTacticalObject(Identifier): Ack;
}
```

### Pros

- Zero-copy reads — ideal for the high-frequency `entity_updates` stream
- Binary format is even more compact than protobuf
- Replaces the hand-rolled `StreamingCodec` with a generated, type-safe codec
- `struct` types (Position, Velocity) are inline — no heap allocation
- gRPC-compatible via official plugin
- Built-in JSON support removes the need for a separate `TacticalObjectsCodec`

### Cons

- Smaller ecosystem and community than protobuf
- FlatBuffers API is less intuitive than protobuf (builders, verifiers)
- ROS2 `.msg` generation is custom (same as Option A)
- `rpc_service` support is less mature than protobuf gRPC
- **No streaming support in `rpc_service`** — streaming `Read` requires a separate mechanism
- No Ada code generator
- Team learning curve is steeper

### User-friendliness rating: **Medium**

Great performance characteristics, but the API ergonomics are rougher than protobuf. Best suited if streaming throughput is the primary concern.

---

## 6. Option D — Hybrid: Canonical C++ Structs + Schema Annotations

### Overview

Keep the existing C++ structs as the source of truth, but add compile-time reflection metadata (via macros or a lightweight DSL header) that enables automatic codec generation.

This is closest to the current architecture and minimises disruption.

### Schema sketch

```cpp
// TacticalObjectsSchema.h — annotated structs

AME_SCHEMA_BEGIN(TacticalObject, "pyramid.components.tactical_objects.TacticalObject")
  AME_FIELD(1, id,            Identifier,      REQUIRED)
  AME_FIELD(2, type,          ObjectType,      REQUIRED)
  AME_FIELD(3, position,      Position,        REQUIRED)
  AME_FIELD(4, velocity,      Velocity,        OPTIONAL)
  AME_FIELD(5, affiliation,   Affiliation,     REQUIRED)
  AME_FIELD(6, mil_class,     MilClassProfile, OPTIONAL)
  AME_FIELD(7, identity_name, std::string,     OPTIONAL)
  AME_FIELD(8, source_refs,   std::vector<std::string>, OPTIONAL)
AME_SCHEMA_END()

// EntityActions pattern: marks TacticalObject as an Entity
// Generates Create/Read/Update/Delete RPCs automatically
AME_ENTITY_SERVICE(TacticalObjectService, TacticalObject, INOUT)
  // Additional non-CRUD RPCs
  AME_RPC(SubscribeUpdates,   InterestCriteria,  EntityUpdateFrame, STREAM_RESPONSE)
  AME_RPC(IngestObservations, Observation,        CorrelationResult, STREAM_REQUEST)
AME_SERVICE_END()
```

A build-time code generator reads these macros (or a parallel YAML/TOML schema file) and emits:

- Binary codec (replaces hand-rolled `StreamingCodec`)
- JSON codec (replaces hand-written `TacticalObjectsCodec`)
- `.proto` file (for gRPC consumers)
- `.msg`/`.srv` files (for ROS2 consumers)

### Pros

- Minimal disruption — existing C++ structs and PCL glue stay as-is
- No new build dependency (protoc, flatc); generator is a simple Python script
- Field numbers enable binary compatibility and delta encoding
- Team continues working in C++ — no context switching to a separate IDL
- Full control over the codec; can preserve the existing `StreamingCodec` wire format
- `AME_ENTITY_SERVICE` macro enforces EntityActions consistency at definition time

### Cons

- Custom tooling — more maintenance burden than using an industry-standard IDL
- Cross-language support requires writing generators per target language
- Less ecosystem support (no `buf lint`, no `grpcurl`, etc.)
- Risk of the annotation macros diverging from the actual struct definitions
- No path to Ada without a custom Ada generator

### User-friendliness rating: **High for C++ developers, Low for polyglot teams**

If the team is primarily C++ and wants to avoid adding new IDL files, this is the least disruptive. But it trades ecosystem leverage for familiarity.

---

## 7. Option E — SysML-First: Model-Driven with Existing Pipeline

### Overview

Use the existing SysML-to-code generation pipeline as the canonical source of truth. The service schema lives in the SysML model (MagicDraw/Cameo), is exported as XMI, parsed into a JSON intermediate representation, and then code generators produce all targets.

This option leverages the pipeline that already exists and has proven patterns for EntityActions, service inheritance, flow direction, and multi-language output.

### How it works

```
┌──────────────┐     ┌────────────────┐     ┌──────────────────────────────────┐
│ SysML Model  │────▶│ sysml_parser.py│────▶│ JSON Intermediate Representation │
│ (MagicDraw)  │     │  (XMI → JSON)  │     │  (datamodel.json)                │
└──────────────┘     └────────────────┘     └────────┬─────────────────────────┘
                                                     │
                     ┌───────────────────────────────┬┼───────────────────────────┐
                     │                               ││                           │
                     ▼                               ▼│                           ▼
              ┌──────────────┐              ┌──────────────┐             ┌──────────────┐
              │proto_generator│              │cpp_model_gen │             │ada_model_gen │
              │    .py        │              │    .py       │             │ada_service_gen│
              └──────┬───────┘              └──────┬───────┘             └──────┬───────┘
                     │                             │                            │
                     ▼                             ▼                            ▼
              .proto + gRPC            C++14 headers               Ada specs + stubs
              service defs             (unique_ptr,                (event-driven,
                                        topo-sorted)               middleware binding)
                     │
                     ▼
              ┌──────────────┐
              │ proto→msg    │  (new script)
              │  converter   │
              └──────┬───────┘
                     ▼
              ROS2 .msg/.srv
```

### Schema as it appears in the SysML model

The model defines Entity-derived types as classes with properties. The pipeline distinguishes:

- **Entity-derived property** → generates a gRPC service with CRUD operations
- **Non-entity property** → becomes a message field

```
┌─────────────────────────────────────────────┐
│ «Component» Tactical_Objects                │
│─────────────────────────────────────────────│
│ + matching_object : TacticalObject {inout}  │  ← Entity → CRUD service
│ + zone            : Zone           {inout}  │  ← Entity → CRUD service
│ + interest        : Interest       {out}    │  ← Entity → Read-only service
│ + observation     : Observation    {in}     │  ← Entity → Write-only service
│ + config          : Configuration           │  ← non-Entity → message field
└─────────────────────────────────────────────┘
```

### Generated proto output (from proto_generator.py)

```protobuf
syntax = "proto3";
package pyramid.components.tactical_objects;

import "pyramid/data_model/base.proto";

// ── EntityActions CRUD for TacticalObject (flow: inout) ─

service Matching_Objects_Service {
  rpc CreateMatchingObject (TacticalObject) returns (pyramid.data_model.base.Identifier);
  rpc ReadMatchingObject   (pyramid.data_model.base.Query) returns (stream TacticalObject);
  rpc UpdateMatchingObject (TacticalObject) returns (pyramid.data_model.base.Ack);
  rpc DeleteMatchingObject (pyramid.data_model.base.Identifier) returns (pyramid.data_model.base.Ack);
}

// ── EntityActions CRUD for Zone (flow: inout) ───────────

service Zone_Service {
  rpc CreateZone (Zone) returns (pyramid.data_model.base.Identifier);
  rpc ReadZone   (pyramid.data_model.base.Query) returns (stream Zone);
  rpc UpdateZone (Zone) returns (pyramid.data_model.base.Ack);
  rpc DeleteZone (pyramid.data_model.base.Identifier) returns (pyramid.data_model.base.Ack);
}

// ── Interest (flow: out → Read only) ────────────────────

service Interest_Service {
  rpc ReadInterest (pyramid.data_model.base.Query) returns (stream Interest);
}

// ── Observation (flow: in → Write only) ─────────────────

service Observation_Ingress_Service {
  rpc CreateObservation (Observation) returns (pyramid.data_model.base.Identifier);
  rpc UpdateObservation (Observation) returns (pyramid.data_model.base.Ack);
  rpc DeleteObservation (pyramid.data_model.base.Identifier) returns (pyramid.data_model.base.Ack);
}
```

### Generated Ada output (from ada_service_generator.py)

```ada
-- Pyramid.Services.Components.Tactical_Objects.Provided
package Pyramid.Services.Components.Tactical_Objects.Provided is

   -- EntityActions for TacticalObject
   procedure Handle_Create_Matching_Object
     (Entity : in  TacticalObject;
      Result : out Identifier);

   procedure Handle_Read_Matching_Object
     (Scope  : in  Query;
      Result : out TacticalObject_Array);

   procedure Handle_Update_Matching_Object
     (Entity : in  TacticalObject;
      Result : out Ack);

   procedure Handle_Delete_Matching_Object
     (Id     : in  Identifier;
      Result : out Ack);

end Pyramid.Services.Components.Tactical_Objects.Provided;
```

### Service inheritance example

```
Base_Service  (has capability: Capability {inout})
  └── Sensor_Requirement  (inherits + has sensor_config: SensorConfig {inout})
```

Sensor_Requirement inherits all Capability CRUD RPCs from Base_Service plus its own SensorConfig RPCs — **without importing the parent message or embedding a base field**.

### Pros

- **True single source of truth** — the SysML model is the authoritative schema; all code is derived
- **Already built** — the pipeline (`sysml_parser.py`, `proto_generator.py`, `cpp_model_generator.py`, `ada_model_generator.py`, `ada_service_generator.py`) exists and has been validated
- **EntityActions consistency is enforced** — the generators apply CRUD semantics automatically based on Entity inheritance
- **Flow direction is modelled** — `inout`/`out`/`in` controls which operations are generated, avoiding unnecessary RPCs
- **Service inheritance works** — child services get parent RPCs without message embedding
- **Multi-language from day one** — proto, C++14, Ada, Python all generated from the same JSON IR
- **Defence/safety alignment** — SysML is standard in MoD/DoD programmes; the model is the assurance artefact
- **Ada support** — event-driven service stubs with middleware abstraction already generated

### Cons

- **MagicDraw/Cameo dependency** — editing the schema requires the SysML tool; not everyone has access or training
- **User-friendliness is lower for day-to-day development** — modifying a field requires: edit model → export XMI → run parser → run generators. Compared to editing a `.proto` directly, this is slower
- **JSON IR is an intermediate format** — not human-editable as a primary schema; changes must go through the model
- **ROS2 `.msg` generation not yet implemented** — needs a new `ros2_msg_generator.py` (but straightforward given the JSON IR)
- **Pipeline maintenance** — the generators are custom Python scripts; must be maintained alongside the model
- **Proto generator has known issues** — EntityActions semantics need fixing (`Read` takes `Query`, returns `stream`; `Update`/`Delete` return `Ack` not `Empty`; `List`/`Stream` RPCs need removal)
- **Empty parent message problem** — classes with only Entity-derived properties still generate empty messages and unwanted imports

### User-friendliness rating: **High for systems engineers, Medium-Low for developers**

The SysML model provides an excellent birds-eye view and is the natural home for the schema in a defence programme. But the edit-export-generate cycle is heavier than directly editing `.proto` or `.msg` files. Best when schema changes are infrequent and controlled by a configuration management process.

---

## 8. Comparison Matrix

| Criterion | A: Protobuf | B: ROS2 IDL | C: FlatBuffers | D: C++ Hybrid | E: SysML-First |
|-----------|:-----------:|:-----------:|:--------------:|:-------------:|:--------------:|
| Single source of truth | ++ | + | ++ | + | ++ |
| gRPC native | ++ | -- | + | - | ++ |
| ROS2 native | - | ++ | - | - | - |
| PCL integration | + | + | + | ++ | + |
| Binary performance | + | 0 | ++ | + | + |
| Cross-language | ++ | + | ++ | - | ++ |
| Versioning / compat | ++ | - | ++ | + | + |
| Enum support | ++ | -- | ++ | ++ | ++ |
| Streaming RPCs | ++ | + | + | + | ++ |
| Team familiarity | + | ++ | - | ++ | + |
| Tooling ecosystem | ++ | + | + | - | - |
| Build complexity | - | 0 | - | + | -- |
| Migration effort | Medium | Low | Medium | Low | Low |
| EntityActions alignment | + | - | - | + | ++ |
| Ada target support | - | -- | -- | - | ++ |
| Service inheritance | - | - | - | + | ++ |
| Flow direction control | - | - | - | + | ++ |
| Defence/MoD alignment | + | - | - | - | ++ |

Legend: `++` strong, `+` good, `0` neutral, `-` weak, `--` poor

---

## 9. Recommendation

### Recommended: **Option E+A Hybrid — SysML-authoritative, Protobuf-operational**

Neither Option A nor Option E alone is ideal. The strongest approach combines both:

```
                        ┌──────────────┐
                        │ SysML Model  │  ← Authoritative schema (design-time)
                        │ (MagicDraw)  │
                        └──────┬───────┘
                               │ export XMI
                               ▼
                        ┌──────────────┐
                        │sysml_parser  │
                        └──────┬───────┘
                               │ JSON IR
                               ▼
                        ┌──────────────┐
                        │proto_generator│  ← Generates .proto from model
                        └──────┬───────┘
                               │
                     ┌─────────┼─────────┐
                     ▼         ▼         ▼
               .proto files   C++14    Ada specs
              (checked in)   headers   + stubs
                     │
            ┌────────┼────────┐
            ▼        ▼        ▼
         gRPC     ROS2     PCL codec
         stubs   .msg/.srv  (protobuf
        (protoc) (script)   payload)
```

**The key insight:** `.proto` files are **generated from the SysML model** but **checked into the repository** as the operational interface contract. Day-to-day consumers work with `.proto` files directly. Schema changes flow through the model when architectural changes occur, but the `.proto` files are the practical source of truth for code generation.

### Why this hybrid works

| Concern | How it's addressed |
|---------|-------------------|
| **G1: Protocol-agnostic** | Protobuf is the operational IDL; all transports derive from it |
| **G2: ROS2 compatible** | `proto→msg` script generates `.msg`/`.srv` (same as Option A) |
| **G3: gRPC compatible** | Native — `.proto` files produce gRPC stubs directly |
| **G4: User-friendly** | Developers work with `.proto` files day-to-day; SysML is for architects |
| **G5: Backward-compatible** | `to_proto()`/`from_proto()` converters keep C++ structs working |
| **G6: EntityActions** | `proto_generator.py` enforces CRUD semantics from the model |
| **G7: Ada compatible** | `ada_service_generator.py` produces Ada specs from the same JSON IR |

### Migration path

```
Phase 1 — Fix proto_generator.py EntityActions issues:
          • Read takes Query, returns stream Entity
          • Update/Delete return Ack (not Empty)
          • Remove List/Stream RPCs (Read subsumes them)
          • Add Query and Ack to BASE_TYPE_DEFINITIONS
          • Fix empty parent message generation

Phase 2 — Generate .proto files for Tactical Objects component
          Check generated .proto into repo under proto/pyramid/
          Validate against existing TacticalObjectsTypes.h

Phase 3 — Write proto→ROS2 converter (ros2_msg_generator.py)
          Generate .msg/.srv from checked-in .proto files

Phase 4 — Integrate protobuf into PCL:
          • to_proto()/from_proto() converters for C++ structs
          • TacticalObjectsComponent handlers accept protobuf payload
          • Retire hand-rolled StreamingCodec + TacticalObjectsCodec

Phase 5 — Stand up gRPC server alongside PCL (for external clients)
          Ada services generated in parallel from same JSON IR

Phase 6 — Validate end-to-end:
          • SysML model change → XMI export → parse → generate
          • .proto, C++, Ada, ROS2 all consistent
          • PCL and gRPC both serve same data
```

### Fallback positions

**If the SysML pipeline is not yet ready for production use**, start with **Option A (Protobuf-First)** — hand-write the `.proto` files using EntityActions conventions, and retrofit the SysML model later. The `.proto` files remain compatible either way.

**If streaming throughput is critical**, consider **FlatBuffers (Option C)** for the `entity_updates` hot path only, while using protobuf for CRUD services. The two can coexist.

**If the team strongly prefers minimal tooling changes**, **Option D (C++ Hybrid)** is the lowest-friction starting point. The `AME_ENTITY_SERVICE` macro can enforce EntityActions patterns at the C++ level and be graduated to full protobuf later.

---

## Appendix A: EntityActions Quick Reference

The abstract `EntityActions` contract from the SysML pipeline:

```
┌──────────────────────────────────────────────────────────────────┐
│ EntityActions<T>                                                 │
│──────────────────────────────────────────────────────────────────│
│ create(data: T)           → Identifier                          │
│ read(scope: Query)        → stream T                            │
│ update(data: T)           → Ack                                 │
│ delete(id: Identifier)    → Ack                                 │
│──────────────────────────────────────────────────────────────────│
│ Query = { id: Identifier[*], oneShot: Boolean }                 │
│ Ack   = { success: Boolean }                                    │
│ Identifier = { uuid: string }                                   │
│──────────────────────────────────────────────────────────────────│
│ Flow Direction:                                                  │
│   inout → Create + Read + Update + Delete                       │
│   out   → Read only                                             │
│   in    → Create + Update + Delete                              │
└──────────────────────────────────────────────────────────────────┘
```

### Flow direction applied to Tactical Objects

| Entity Property | Flow | Generated RPCs |
|----------------|------|----------------|
| `matching_object : TacticalObject` | `inout` | Create, Read, Update, Delete |
| `zone : Zone` | `inout` | Create, Read, Update, Delete |
| `interest : Interest` | `out` | Read |
| `observation : Observation` | `in` | Create, Update, Delete |

## Appendix B: Known Proto Generator Issues

Issues in `proto_generator.py` that must be resolved before Phase 1 completion:

| # | Issue | Location | Fix |
|---|-------|----------|-----|
| 1 | `Read` takes `Identifier` instead of `Query` | `_write_grpc_service` (~L669) | Change request type to `Query` |
| 2 | `Read` returns single entity, not `stream` | `_write_grpc_service` (~L669) | Add `stream` to response |
| 3 | `Update` returns `google.protobuf.Empty` | `_write_grpc_service` (~L669) | Change to `Ack` |
| 4 | `Delete` returns `google.protobuf.Empty` | `_write_grpc_service` (~L669) | Change to `Ack` |
| 5 | Generates `List` and `Stream` RPCs | `_write_grpc_service` | Remove — `Read` with `Query` subsumes both |
| 6 | `Query` and `Ack` missing from `BASE_TYPE_DEFINITIONS` | `_write_base_types` | Add with field_rules for `repeated`/`optional` |
| 7 | Empty parent messages generated | message generation | Skip message body for classes with only entity-derived properties |
| 8 | Child services import parent `.proto` unnecessarily | import resolution | Inherit RPCs directly, don't embed parent message |
