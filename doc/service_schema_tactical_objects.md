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

// -- EntityActions Base Types ----------------------------

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

// -- Primitives ------------------------------------------

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

// -- Entity: TacticalObject ------------------------------
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

// -- Entity: Zone ----------------------------------------

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

// -- Extended Query (superset of base Query) -------------

message TacticalObjectQuery {
  pyramid.data_model.base.Query base    = 1;  // ID filter + oneShot flag
  optional ObjectType  by_type          = 2;
  optional Affiliation by_affiliation   = 3;
  optional BoundingBox by_region        = 4;
  optional double      max_age_seconds  = 5;
  optional string      by_source_system = 6;
}

// -- gRPC Services (EntityActions pattern) ---------------

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
┌--------------┐     ┌----------------┐     ┌----------------------------------┐
│ SysML Model  │----▶│ sysml_parser.py│----▶│ JSON Intermediate Representation │
│ (MagicDraw)  │     │  (XMI → JSON)  │     │  (datamodel.json)                │
└--------------┘     └----------------┘     └--------┬-------------------------┘
                                                     │
                     ┌-------------------------------┬┼---------------------------┐
                     │                               ││                           │
                     ▼                               ▼│                           ▼
              ┌--------------┐              ┌--------------┐             ┌--------------┐
              │proto_generator│              │cpp_model_gen │             │ada_model_gen │
              │    .py        │              │    .py       │             │ada_service_gen│
              └------┬-------┘              └------┬-------┘             └------┬-------┘
                     │                             │                            │
                     ▼                             ▼                            ▼
              .proto + gRPC            C++14 headers               Ada specs + stubs
              service defs             (unique_ptr,                (event-driven,
                                        topo-sorted)               middleware binding)
                     │
                     ▼
              ┌--------------┐
              │ proto→msg    │  (new script)
              │  converter   │
              └------┬-------┘
                     ▼
              ROS2 .msg/.srv
```

### Schema as it appears in the SysML model

The model defines Entity-derived types as classes with properties. The pipeline distinguishes:

- **Entity-derived property** → generates a gRPC service with CRUD operations
- **Non-entity property** → becomes a message field

```
┌---------------------------------------------┐
│ «Component» Tactical_Objects                │
│---------------------------------------------│
│ + matching_object : TacticalObject {inout}  │  ← Entity → CRUD service
│ + zone            : Zone           {inout}  │  ← Entity → CRUD service
│ + interest        : Interest       {out}    │  ← Entity → Read-only service
│ + observation     : Observation    {in}     │  ← Entity → Write-only service
│ + config          : Configuration           │  ← non-Entity → message field
└---------------------------------------------┘
```

### Generated proto output (from proto_generator.py)

```protobuf
syntax = "proto3";
package pyramid.components.tactical_objects;

import "pyramid/data_model/base.proto";

// -- EntityActions CRUD for TacticalObject (flow: inout) -

service Matching_Objects_Service {
  rpc CreateMatchingObject (TacticalObject) returns (pyramid.data_model.base.Identifier);
  rpc ReadMatchingObject   (pyramid.data_model.base.Query) returns (stream TacticalObject);
  rpc UpdateMatchingObject (TacticalObject) returns (pyramid.data_model.base.Ack);
  rpc DeleteMatchingObject (pyramid.data_model.base.Identifier) returns (pyramid.data_model.base.Ack);
}

// -- EntityActions CRUD for Zone (flow: inout) -----------

service Zone_Service {
  rpc CreateZone (Zone) returns (pyramid.data_model.base.Identifier);
  rpc ReadZone   (pyramid.data_model.base.Query) returns (stream Zone);
  rpc UpdateZone (Zone) returns (pyramid.data_model.base.Ack);
  rpc DeleteZone (pyramid.data_model.base.Identifier) returns (pyramid.data_model.base.Ack);
}

// -- Interest (flow: out → Read only) --------------------

service Interest_Service {
  rpc ReadInterest (pyramid.data_model.base.Query) returns (stream Interest);
}

// -- Observation (flow: in → Write only) -----------------

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
  └-- Sensor_Requirement  (inherits + has sensor_config: SensorConfig {inout})
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

## 9. Decision: Option A/E Hybrid — SysML-Authoritative, Protobuf as IDL

**Status:** DECIDED (2026-03-15)

### Selected approach

Option E+A hybrid: SysML model is the authoritative schema; protobuf (`.proto` files) is the operational IDL used by code generators. The existing SysML-to-code pipeline in `pim/` is the codex that drives generation for all target languages.

### Key constraints

| Constraint | Rationale |
|------------|-----------|
| **No gRPC runtime dependency** | This project must not take a dependency on the gRPC framework (libraries, `grpc++`, gRPC codegen plugins). gRPC adds significant build complexity and is not needed for our transports (PCL, ROS2, socket). |
| **Protobuf accepted as IDL** | Protobuf's `.proto` files are a reasonable, widely-understood baseline IDL for defining messages and service contracts. We use protobuf as a **schema language**, not as a wire protocol or RPC framework. |
| **Protobuf must align with SysML CRUD semantics** | The `service` and `rpc` definitions in `.proto` files express EntityActions CRUD operations (Create/Read/Update/Delete) with correct signatures. These are **semantic contracts**, not gRPC endpoint definitions. |
| **No backward compatibility requirement** | Tactical Objects schema can change freely to match the codex semantics. We are not constrained by existing wire formats or client expectations. |

### What "protobuf as IDL" means in practice

The `.proto` files define **what** the service contract is (messages, operations, types). They do **not** imply:

- A gRPC server or client
- The `grpc++` or `grpc_cpp_plugin` build dependency
- Protobuf binary wire format on any transport
- Any gRPC-specific concepts (channels, interceptors, deadlines)

The `service` / `rpc` blocks in `.proto` files are read by our codex generators (`ada_service_generator.py`, `ros2_msg_generator.py`) to produce target-language service stubs with EntityActions CRUD semantics. The generated code uses whatever transport the target platform requires (PCL, ROS2, Ada middleware, etc.).

### Pipeline: SysML model to generated code

The pipeline tooling lives in `pim/` and is checked into this repository:

| File | Role |
|------|------|
| `pim/sysml_parser.py` | Parses Cameo/MagicDraw XMI export into JSON IR (`datamodel.json`) |
| `pim/proto_generator.py` | Generates `.proto` files from JSON IR with EntityActions CRUD services |
| `pim/cpp_model_generator.py` | Generates C++14 headers (unique_ptr, topo-sorted) from JSON IR |
| `pim/ada_model_generator.py` | Generates Ada type packages from JSON IR *(scaffolding: generated once, then manually owned)* |
| `pim/ada_service_generator.py` | Generates Ada service specs/stubs from `.proto` IDL (transport target — not a JSON IR consumer) |
| `pim/ada_type_generator.py` | Generates Ada type definitions |
| `pim/python_model_generator.py` | Generates Python dataclasses from JSON IR |
| `pim/contract_resolver.py` | Resolves provided/consumed service contracts between components |
| `pim/activity_parser.py` | Parses SysML activity diagrams |
| `pim/mock_generator.py` | Generates mock implementations for testing |
| `pim/test_generator.py` | Generates test harnesses |

```
┌--------------┐     ┌--------------┐     ┌----------------------┐
│ SysML Model  │----▶│sysml_parser  │----▶│ JSON IR              │
│ (MagicDraw)  │     │    .py       │     │ (datamodel.json)     │
└--------------┘     └--------------┘     └----------┬-----------┘
                                                     │
                     ┌---------------┬---------------┼---------------┐
                     │               │               │               │
                     ▼               ▼               ▼               ▼
              ┌-------------┐ ┌-------------┐ ┌-------------┐ ┌-------------┐
              │proto_gen    │ │cpp_model_gen│ │ada_model_gen│ │python_gen   │
              │  .py        │ │  .py        │ │  .py        │ │  .py        │
              └------┬------┘ └------┬------┘ └------┬------┘ └------┬------┘
                     │               │               │               │
                     ▼               ▼               ▼               ▼
              .proto files     C++14 headers  ╔═════════════╗  Python
              (IDL only —      (unique_ptr,   ║  Ada types  ║  dataclasses
               no gRPC)        topo-sorted)   ║[scaffolding]║
                     │                        ╚═════════════╝
                     │
            ┌--------┼--------------┐
            ▼        ▼              ▼
         PCL codec  ROS2      Ada middleware
         (custom)  .msg/.srv  stubs
                   (ros2_msg_ (ada_service_
                   generator) generator.py)
```

**The key insight:** `.proto` files are **generated from the SysML model** and **checked into the repository** as the operational interface contract. The `service`/`rpc` blocks describe EntityActions CRUD semantics that codex generators consume — they are **not** gRPC endpoint definitions. Day-to-day consumers work with `.proto` files directly. Schema changes flow through the model when architectural changes occur.

### Why this hybrid works

| Concern | How it's addressed |
|---------|-------------------|
| **G1: Protocol-agnostic** | Protobuf is the IDL; transport adapters are generated per-target |
| **G2: ROS2 compatible** | `proto→msg` script generates `.msg`/`.srv` (same as Option A) |
| **G3: gRPC compatible** | `.proto` files *could* produce gRPC stubs if needed later, but this is not a current goal |
| **G4: User-friendly** | Developers work with `.proto` files day-to-day; SysML is for architects |
| **G5: Backward-compatible** | Not a constraint — tactical objects schema can change freely |
| **G6: EntityActions** | `proto_generator.py` enforces CRUD semantics from the model |
| **G7: Ada compatible** | `ada_service_generator.py` reads `.proto`; `ada_model_generator.py` bootstraps Ada types (scaffolding) |

### Migration path

```
Phase 1 — Fix proto_generator.py EntityActions semantics:
          • Read takes Query, returns stream Entity
          • Update/Delete return Ack (not google.protobuf.Empty)
          • Remove List/Stream RPCs (Read subsumes them)
          • Add Query and Ack to BASE_TYPE_DEFINITIONS
          • Fix empty parent message generation
          • Remove google.protobuf.Empty dependency entirely

Phase 2 — Generate .proto files for Tactical Objects component
          Check generated .proto into repo under proto/pyramid/
          Validate alignment with EntityActions CRUD semantics
          (No need to preserve existing TacticalObjectsTypes.h layout)

Phase 3 — Write proto→ROS2 converter (ros2_msg_generator.py)
          Generate .msg/.srv from checked-in .proto files

Phase 4 — Integrate protobuf IDL into PCL:
          • Transport-specific codec generated from .proto definitions
          • TacticalObjectsComponent handlers use generated types
          • Retire hand-rolled StreamingCodec + TacticalObjectsCodec

Phase 5 — Ada services generated from .proto (ada_service_generator.py reads .proto, not JSON IR)
          (gRPC server is NOT a goal — transport remains PCL/ROS2/socket)

Phase 6 — Validate end-to-end:
          • SysML model change → XMI export → parse → generate
          • .proto, C++, Ada, ROS2 all consistent
          • All targets use EntityActions CRUD semantics
```

### What we are NOT doing

- **No gRPC server or client.** If external clients need access, we will evaluate transport options separately. gRPC remains an option but is not assumed.
- **No protobuf binary wire format requirement.** Transports may use protobuf binary encoding if convenient, but this is a transport-layer decision, not an IDL-layer one.
- **No backward compatibility with existing tactical objects wire format.** The schema is free to change to match codex semantics.

---

## Appendix A: EntityActions Quick Reference

The abstract `EntityActions` contract from the SysML pipeline:

```
┌------------------------------------------------------------------┐
│ EntityActions<T>                                                 │
│------------------------------------------------------------------│
│ create(data: T)           → Identifier                          │
│ read(scope: Query)        → stream T                            │
│ update(data: T)           → Ack                                 │
│ delete(id: Identifier)    → Ack                                 │
│------------------------------------------------------------------│
│ Query = { id: Identifier[*], oneShot: Boolean }                 │
│ Ack   = { success: Boolean }                                    │
│ Identifier = { uuid: string }                                   │
│------------------------------------------------------------------│
│ Flow Direction:                                                  │
│   inout → Create + Read + Update + Delete                       │
│   out   → Read only                                             │
│   in    → Create + Update + Delete                              │
└------------------------------------------------------------------┘
```

### Flow direction applied to Tactical Objects

| Entity Property | Flow | Generated RPCs |
|----------------|------|----------------|
| `matching_object : TacticalObject` | `inout` | Create, Read, Update, Delete |
| `zone : Zone` | `inout` | Create, Read, Update, Delete |
| `interest : Interest` | `out` | Read |
| `observation : Observation` | `in` | Create, Update, Delete |

## Appendix B: Proto Generator Issues

Issues in `pim/proto_generator.py`. Line references are to the current source as of 2026-03-15.

### Resolved

| # | Issue | Resolution |
|---|-------|------------|
| 7 | Empty parent messages generated | `_write_message_from_class` L617: `skip_message` logic correctly skips message body for classes with only entity-derived properties |
| 8 | Child services import parent `.proto` unnecessarily | `_collect_imports` L346: checks `_type_has_non_entity_properties` before adding parent import |

### Open — must fix in Phase 1

| # | Issue | Location | Fix |
|---|-------|----------|-----|
| 1 | `Read` takes `Identifier` instead of `Query` | `_write_grpc_service` L768 | Change request type to `Query` |
| 2 | `Read` returns single entity, not `stream` | `_write_grpc_service` L768 | Add `stream` to response |
| 3 | `Update` returns `google.protobuf.Empty` | `_write_grpc_service` L771 | Change to `Ack` |
| 4 | `Delete` returns `google.protobuf.Empty` | `_write_grpc_service` L774 | Change to `Ack` |
| 5 | Generates `List` and `Stream` RPCs | `_write_grpc_service` L776–L780 | Remove — `Read` with `Query` subsumes both |
| 6 | `Query` and `Ack` missing from `BASE_TYPE_DEFINITIONS` | L34–L75 | Add with field rules for `repeated`/`optional` |
| 9 | `google.protobuf.Empty` import generated | `_needs_google_imports` L379 | Remove entirely — use `Ack` instead; no google proto dependencies needed |
| 10 | Service/RPC blocks imply gRPC | `_write_grpc_service` L741 | Rename method and comments to `_write_entity_service`; these are EntityActions CRUD contracts, not gRPC endpoints |

Issues 3, 4, 5, 9 are all consequences of the same root cause: the `_write_grpc_service` method was written assuming gRPC conventions (`google.protobuf.Empty`, `List`/`Stream` RPCs). Fixing L762–L800 resolves all four. Similarly, issues 1 and 2 in the `out` flow path (L781–L790) need the same Read signature fix.

## Appendix C: SysML Pipeline Files

The code generation pipeline is checked into `pim/`. These are the codex generators that consume the JSON IR and produce target-language output.

| File | Input | Output | Notes |
|------|-------|--------|-------|
| `pim/sysml_parser.py` | XMI (Cameo export) | `datamodel.json` | Extracts DataTypes, Enumerations, Classes with properties, generalizations, flow direction |
| `pim/proto_generator.py` | `datamodel.json` | `.proto` files | EntityActions CRUD services; protobuf as IDL only (no gRPC runtime) |
| `pim/cpp_model_generator.py` | `datamodel.json` | C++14 headers | `unique_ptr`, topologically sorted, includes from generalizations |
| `pim/ada_model_generator.py` | `datamodel.json` | Ada type packages | Ada record types with discriminants |
| `pim/ada_type_generator.py` | `datamodel.json` | Ada type definitions | Low-level Ada type generation |
| `pim/ada_service_generator.py` | `.proto` file or directory | Ada service specs/stubs | Reads proto `service`/`rpc` blocks; transport target alongside ROS2/PCL |
| `pim/python_model_generator.py` | `datamodel.json` | Python dataclasses | Python model types |
| `pim/contract_resolver.py` | `datamodel.json` | Contract mappings | Resolves provided/consumed service contracts between components |
| `pim/activity_parser.py` | XMI | Activity models | Parses SysML activity diagrams for behavioural generation |
| `pim/mock_generator.py` | `datamodel.json` | Mock implementations | Test doubles for service interfaces |
| `pim/test_generator.py` | `datamodel.json` | Test harnesses | Generated test scaffolding |

### JSON IR structure (from `sysml_parser.py`)

The intermediate representation has three top-level arrays:

```json
{
  "dataTypes": [
    {
      "name": "TacticalObject",
      "namespace": ["Components", "Tactical Objects", ...],
      "isAbstract": false,
      "properties": [
        {
          "name": "position",
          "typeName": "Position",
          "flow_direction": "inout",
          "multiplicity": { "lower": "1", "upper": "1" }
        }
      ],
      "generalizes": ["Entity"]
    }
  ],
  "enumerations": [ ... ],
  "classes": [ ... ]
}
```

Key semantic markers consumed by generators:
- **`generalizes: ["Entity"]`** — marks a type as Entity-derived; triggers CRUD service generation
- **`flow_direction`** — `inout` (full CRUD), `out` (read-only), `in` (write-only)
- **`isAbstract`** — abstract types generate `oneof` blocks in protobuf, not standalone messages
- **`multiplicity.upper: "*"`** — generates `repeated` fields in protobuf

---

## Appendix D: Migration Plan — PCL / Tactical Objects

### Context

The Ada service generator (`pim/ada_service_generator.py`) currently produces service stubs that bind to a `Pyramid.Middleware` abstraction — a low-level `Send`/`Receive`/`Receive_Any` interface over raw `System.Address` and integer channel IDs. This is a placeholder transport layer that assumes an unspecified middleware (DDS, ZeroMQ, shared memory, etc.).

The C++ side of this project already has a working component framework (PCL) with `TacticalObjectsComponent` providing typed service handlers over `pcl_msg_t`. The goal of this migration is to:

1. Align the Ada codex output with PCL service semantics (not the raw middleware abstraction)
2. Fix the proto generator so `.proto` IDL matches EntityActions CRUD
3. Make all generated code (C++, Ada, Python, proto) consistent with one service contract

### What changes

| Current (Ada middleware) | Target (PCL-aligned) |
|--------------------------|----------------------|
| `Pyramid.Middleware.Send(Channel, Address, Size)` | Typed service handlers per EntityActions operation |
| Integer channel IDs | Named service endpoints matching CRUD operations |
| `System.Address` raw buffers | Typed message parameters (Entity, Query, Identifier, Ack) |
| `Receive_Any` event loop | Dispatcher that routes to typed `Handle_*` procedures |
| Hand-written channel allocation | Channels derived from service contract (codex-generated) |

### Phase 1: Fix proto_generator.py (codex alignment)

Fix the 8 open issues in Appendix B so that generated `.proto` files express correct EntityActions CRUD semantics:

```
Before (current):
  rpc ReadMatchingObject(Identifier) returns (TacticalObject);
  rpc UpdateMatchingObject(TacticalObject) returns (google.protobuf.Empty);
  rpc DeleteMatchingObject(Identifier) returns (google.protobuf.Empty);
  rpc ListMatchingObject(google.protobuf.Empty) returns (stream Identifier);
  rpc StreamMatchingObject(Identifier) returns (stream TacticalObject);

After (target):
  rpc CreateMatchingObject(TacticalObject) returns (Identifier);
  rpc ReadMatchingObject(Query) returns (stream TacticalObject);
  rpc UpdateMatchingObject(TacticalObject) returns (Ack);
  rpc DeleteMatchingObject(Identifier) returns (Ack);
```

Specific code changes in `pim/proto_generator.py`:

1. **Add `Query` and `Ack` to `BASE_TYPE_DEFINITIONS`** (~L34):
   ```python
   'Query': {
       'comment': 'EntityActions query scope',
       'fields': [
           ('repeated Identifier', 'id', 1, 'Filter by specific IDs (empty = all)'),
           ('bool', 'one_shot', 2, 'true = single response; false = continuous stream'),
       ]
   },
   'Ack': {
       'comment': 'EntityActions acknowledgment',
       'fields': [
           ('bool', 'success', 1, 'Operation success/failure'),
       ]
   },
   ```

2. **Rewrite `_write_grpc_service`** (rename to `_write_entity_service`, ~L739):
   - `inout`: Create(Entity)→Identifier, Read(Query)→stream Entity, Update(Entity)→Ack, Delete(Identifier)→Ack
   - `out`: Read(Query)→stream Entity only
   - `in`: Create(Entity)→Identifier, Update(Entity)→Ack, Delete(Identifier)→Ack
   - Remove all `List` and `Stream` RPCs
   - Replace all `google.protobuf.Empty` with `Ack` or `Query`
   - Get qualified `Query` and `Ack` types the same way as `Identifier`

3. **Remove `google.protobuf.Empty` from `_needs_google_imports`** (~L379):
   - Remove `needs['empty']` entirely
   - Remove the `if google_imports['empty']` import line (~L259)

4. **Update comments**: `// gRPC Service for` → `// EntityActions Service for`

### Phase 2: Update ada_service_generator.py (replace middleware abstraction)

Replace the `Pyramid.Middleware` Send/Receive pattern with typed EntityActions handlers that match the PCL component pattern.

**Current output** (`ada_service_generator.py`):
```ada
with Pyramid.Middleware;

package Pyramid.Services.Components.Tactical_Objects.Provided is
   Ch_Matching_Objects_matching_object_Req : constant := 100;
   Ch_Matching_Objects_matching_object_Rsp : constant := 101;

   procedure On_Matching_Objects_matching_object
     (Req : TacticalObject; Rsp : out TacticalObject);

   procedure Run_Loop;
end;
```

**Target output** (PCL-aligned EntityActions):
```ada
package Pyramid.Services.Components.Tactical_Objects.Provided is

   --  EntityActions CRUD for TacticalObject (flow: inout)
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

   --  EntityActions CRUD for Zone (flow: inout)
   procedure Handle_Create_Zone
     (Entity : in  Zone;
      Result : out Identifier);

   -- ... same pattern for each entity property

   --  Service dispatcher (replaces Run_Loop + Receive_Any)
   procedure Dispatch
     (Operation : in  Operation_Kind;
      Channel   : in  Service_Channel;
      Request   : in  System.Address;
      Req_Size  : in  Natural;
      Response  : out System.Address;
      Rsp_Size  : out Natural);

end Pyramid.Services.Components.Tactical_Objects.Provided;
```

Key changes to `pim/ada_service_generator.py`:
- Generate one `Handle_<Op>_<Entity>` procedure per CRUD operation, respecting flow direction
- Parameter types match EntityActions signatures (Entity, Query, Identifier, Ack)
- Remove `Pyramid.Middleware` dependency — the `Dispatch` procedure is the integration point for any transport
- Generate an `Operation_Kind` enumeration: `(Op_Create, Op_Read, Op_Update, Op_Delete)`
- Generate a `Service_Channel` enumeration from entity property names
- The body (`.adb`) generates stub implementations with `null;` and a dispatch table

### Phase 3: Align C++ tactical objects with EntityActions

Update the existing C++ `TacticalObjectsComponent` service handlers to match EntityActions CRUD signatures. This is a rename/restructure — the logic already exists, just under different names.

| Current handler | EntityActions equivalent | Change |
|----------------|------------------------|--------|
| `createObject(ObjectDefinition) → UUIDKey` | `Create(TacticalObject) → Identifier` | Rename types |
| `getObject(UUIDKey) → EntityRecord` | `Read(Query) → stream TacticalObject` | Merge with `query`; use Query type |
| `updateObject(UUIDKey, ObjectUpdate) → bool` | `Update(TacticalObject) → Ack` | Whole-entity update, return Ack |
| `deleteObject(UUIDKey) → bool` | `Delete(Identifier) → Ack` | Return Ack type |
| `query(QueryRequest) → QueryResponse` | (subsumed by `Read`) | Remove; Read with Query handles this |
| `upsertZone(ZoneDefinition) → UUIDKey` | `Create(Zone) / Update(Zone)` | Split into Create + Update |
| `removeZone(UUIDKey) → bool` | `Delete(Identifier) → Ack` | Align |

No backward compatibility requirement — the existing handlers can be replaced directly.

### Phase 4: Hand-write .proto and validate

The SysML XMI export is not available at this time. Instead of running the codex pipeline (sysml_parser → proto_generator), hand-write the `.proto` files to be consistent with the existing C++ tactical objects content, using EntityActions CRUD semantics.

1. Hand-write `.proto` files under `proto/pyramid/` based on:
   - Existing C++ types in `TacticalObjectsTypes.h` (the types that actually exist)
   - EntityActions CRUD signatures (Create/Read/Update/Delete with correct request/response types)
   - The base types from Appendix A (Identifier, Query, Ack)
2. Validate `.proto` matches C++ content — every entity type, enum, and field in the C++ code has a corresponding protobuf definition
3. Run `ada_service_generator.py` on hand-written `.proto` (or equivalent JSON IR) → Ada specs/stubs
4. Verify cross-language consistency:
   - C++ handlers match `.proto` `rpc` signatures
   - Ada `Handle_*` procedures match `.proto` `rpc` signatures

When XMI becomes available later, the codex pipeline can regenerate the `.proto` and the hand-written version becomes the validation baseline.

### Phase 5: Align hand-rolled codecs to EntityActions semantics

Align existing codecs and middleware stubs to use EntityActions CRUD semantics. These are **not** being removed — they remain the working implementation. The goal is semantic consistency, not replacement.

| Artefact | Current state | Alignment work |
|----------|--------------|----------------|
| `StreamingCodec.h` | Hand-rolled binary codec with custom field masks | Align field layout and operation encoding with EntityActions types (Identifier, Query, Ack). Codec continues to own the wire format. |
| `TacticalObjectsCodec.h` | JSON codec using nlohmann::json | Align JSON field names and operation structure with `.proto` message/field names. Codec continues to work as-is. |
| `pyramid-middleware.ads/.adb` | Raw `Send`/`Receive`/`Receive_Any` over `System.Address` + integer channels | Align channel semantics with EntityActions operation kinds. Middleware remains the transport layer; typed `Handle_*` procedures sit above it. |

The codecs and middleware are the transport layer. The `.proto` IDL and generated `Handle_*` stubs are the semantic layer. Both layers continue to exist; this phase ensures they agree on types and operations.

### Non-goals

- **gRPC server/client** — not building one; `.proto` is IDL only
- **Protobuf wire format** — transport layer chooses encoding; not mandated by this migration
- **ROS2 `.msg` generation** — separate future work (`ros2_msg_generator.py`); not blocking this migration
- **Backward compatibility** — tactical objects schema changes freely to match codex semantics
- **Removing existing codecs** — `StreamingCodec`, `TacticalObjectsCodec`, and `Pyramid.Middleware` stay; they get aligned, not replaced
