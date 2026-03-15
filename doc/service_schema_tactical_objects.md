# Service Schema Options for PCL / Tactical Objects

## 1. Problem Statement

The Tactical Objects subsystem currently defines its service contract implicitly through:

- C++ structs in `TacticalObjectsTypes.h` (ObjectDefinition, ObjectUpdate, QueryRequest, etc.)
- A binary streaming codec (`StreamingCodec.h`) with hand-rolled field masks
- A JSON codec (`TacticalObjectsCodec.h`) using nlohmann::json
- PCL service handlers in `TacticalObjectsComponent.h` that accept opaque `pcl_msg_t` blobs

There is no single, canonical **Interface Definition Language (IDL)** artefact that a downstream consumer (ROS2 node, gRPC client, web dashboard, test harness) can use to generate bindings. Every new transport requires hand-writing a codec.

### Goals

| # | Goal |
|---|------|
| G1 | **Protocol-agnostic** — one schema drives all transports |
| G2 | **ROS2 compatible** — maps cleanly to `.msg`/`.srv` files |
| G3 | **gRPC compatible** — maps cleanly to `.proto` files |
| G4 | **User-friendly** — easy to read, version, and extend |
| G5 | **Backward-compatible** — existing C++ structs and PCL glue remain usable |

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

---

## 3. Option A — Protobuf-First with Generated Adapters

### Overview

Define all messages and services in `.proto` files. Use protobuf as the canonical schema and generate:

- C++ structs (via `protoc`)
- gRPC service stubs (via `protoc` + gRPC plugin)
- ROS2 `.msg`/`.srv` files (via a lightweight code-generator script)
- JSON schema (via `protoc-gen-jsonschema` or similar)

### Schema sketch

```protobuf
syntax = "proto3";
package pyramid.tactical;

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

// ── Service Messages ────────────────────────────────────

message ObjectDefinition {
  ObjectType      type        = 1;
  Position        position    = 2;
  Velocity        velocity    = 3;
  Affiliation     affiliation = 4;
  MilClassProfile mil_class   = 5;
  string          identity_name = 6;
  repeated string source_refs   = 7;
}

message ObjectUpdate {
  optional Position        position     = 1;
  optional Velocity        velocity     = 2;
  optional Affiliation     affiliation  = 3;
  optional MilClassProfile mil_class    = 4;
  optional string          identity_name = 5;
  optional string          operational_state = 6;
  optional string          behavior_pattern  = 7;
}

message CreateObjectRequest  { ObjectDefinition definition = 1; }
message CreateObjectResponse { string object_id = 1; }  // UUID

message UpdateObjectRequest  { string object_id = 1; ObjectUpdate update = 2; }
message UpdateObjectResponse { bool success = 1; }

message DeleteObjectRequest  { string object_id = 1; }
message DeleteObjectResponse { bool success = 1; }

message GetObjectRequest     { string object_id = 1; }
message GetObjectResponse    { EntityRecord record = 1; }

message QueryRequest {
  optional string      by_uuid           = 1;
  optional string      by_source_system  = 2;
  optional ObjectType  by_type           = 3;
  optional Affiliation by_affiliation    = 4;
  optional BoundingBox by_region         = 5;
  optional double      max_age_seconds   = 6;
}

message QueryResponse {
  repeated QueryResultEntry entries = 1;
  uint32 total = 2;
}

// ── gRPC Service ────────────────────────────────────────

service TacticalObjects {
  rpc CreateObject (CreateObjectRequest) returns (CreateObjectResponse);
  rpc UpdateObject (UpdateObjectRequest) returns (UpdateObjectResponse);
  rpc DeleteObject (DeleteObjectRequest) returns (DeleteObjectResponse);
  rpc GetObject    (GetObjectRequest)    returns (GetObjectResponse);
  rpc Query        (QueryRequest)        returns (QueryResponse);

  // Streaming
  rpc SubscribeUpdates (InterestCriteria) returns (stream EntityUpdateFrame);
  rpc IngestObservations (stream Observation) returns (CorrelationResult);
}
```

### ROS2 mapping

A thin code-generation script converts `.proto` → `.msg`/`.srv`:

| Proto | ROS2 |
|-------|------|
| `message Position` | `Position.msg` |
| `CreateObjectRequest` / `CreateObjectResponse` | `CreateObject.srv` |
| `service TacticalObjects.Query` | `Query.srv` |
| `stream EntityUpdateFrame` | topic `entity_updates` with `EntityUpdateFrame.msg` |

### Pros

- Single source of truth; all codecs generated
- Native gRPC support (streaming, deadlines, interceptors)
- Excellent cross-language support (Python, Java, Go, C#, TypeScript)
- Strong versioning story (field numbers, `reserved`, `oneof`)
- Binary wire format is compact — close to the existing `StreamingCodec`

### Cons

- Protobuf `optional` semantics differ from C++ `tl::optional` — needs thin mapping layer
- ROS2 `.msg` generation is custom tooling (no off-the-shelf `proto→msg` compiler)
- Adds a build dependency on `protoc`
- Team must learn proto3 idioms (no required fields, default zero values)

### User-friendliness rating: **Medium-High**

Proto files are widely known, well-documented, and have IDE support. The main friction is the ROS2 bridge generator.

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
# CreateObject.srv
ObjectDefinition definition
---
string object_id
```

```
# EntityUpdates.msg  (topic message)
builtin_interfaces/Time stamp
EntityUpdateFrame[] frames
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
- gRPC adapter is manual work; no standard `.msg → .proto` tool
- Enum support is weak — constants are `uint8` fields, not first-class enums
- ROS2 dependency required even for non-ROS2 builds
- Versioning story is weaker (no field numbers, no `reserved`)
- Harder for non-ROS2 consumers (web frontends, standalone C++ apps)

### User-friendliness rating: **High for ROS2 users, Low for others**

If the primary consumers are ROS2 nodes, this is the most natural fit. But it creates friction for gRPC clients, web dashboards, and standalone tools.

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

table ObjectDefinition {
  type:          ObjectType;
  position:      Position;
  velocity:      Velocity;
  affiliation:   Affiliation;
  mil_class:     MilClassProfile;
  identity_name: string;
  source_refs:   [string];
}

table CreateObjectRequest  { definition: ObjectDefinition; }
table CreateObjectResponse { object_id: string; }

// ... other request/response tables ...

rpc_service TacticalObjects {
  CreateObject(CreateObjectRequest): CreateObjectResponse;
  UpdateObject(UpdateObjectRequest): UpdateObjectResponse;
  Query(QueryRequest): QueryResponse;
  // streaming handled separately via FlatBuffers + raw transport
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

AME_SCHEMA_BEGIN(ObjectDefinition, "tactical.ObjectDefinition")
  AME_FIELD(1, type,          ObjectType,      REQUIRED)
  AME_FIELD(2, position,      Position,        REQUIRED)
  AME_FIELD(3, velocity,      Velocity,        OPTIONAL)
  AME_FIELD(4, affiliation,   Affiliation,     REQUIRED)
  AME_FIELD(5, mil_class,     MilClassProfile, OPTIONAL)
  AME_FIELD(6, identity_name, std::string,     OPTIONAL)
  AME_FIELD(7, source_refs,   std::vector<std::string>, OPTIONAL)
AME_SCHEMA_END()

AME_SERVICE(TacticalObjects)
  AME_RPC(CreateObject,  CreateObjectRequest,  CreateObjectResponse)
  AME_RPC(UpdateObject,  UpdateObjectRequest,  UpdateObjectResponse)
  AME_RPC(DeleteObject,  DeleteObjectRequest,  DeleteObjectResponse)
  AME_RPC(GetObject,     GetObjectRequest,     GetObjectResponse)
  AME_RPC(Query,         QueryRequest,         QueryResponse)
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

### Cons

- Custom tooling — more maintenance burden than using an industry-standard IDL
- Cross-language support requires writing generators per target language
- Less ecosystem support (no `buf lint`, no `grpcurl`, etc.)
- Risk of the annotation macros diverging from the actual struct definitions

### User-friendliness rating: **High for C++ developers, Low for polyglot teams**

If the team is primarily C++ and wants to avoid adding new IDL files, this is the least disruptive. But it trades ecosystem leverage for familiarity.

---

## 7. Comparison Matrix

| Criterion | A: Protobuf | B: ROS2 IDL | C: FlatBuffers | D: C++ Hybrid |
|-----------|:-----------:|:-----------:|:--------------:|:-------------:|
| Single source of truth | ++ | + | ++ | + |
| gRPC native | ++ | -- | + | - |
| ROS2 native | - | ++ | - | - |
| PCL integration | + | + | + | ++ |
| Binary performance | + | 0 | ++ | + |
| Cross-language | ++ | + | ++ | - |
| Versioning / compat | ++ | - | ++ | + |
| Enum support | ++ | -- | ++ | ++ |
| Streaming RPCs | ++ | + | + | + |
| Team familiarity | + | ++ | - | ++ |
| Tooling ecosystem | ++ | + | + | - |
| Build complexity | - | 0 | - | + |
| Migration effort | Medium | Low | Medium | Low |

Legend: `++` strong, `+` good, `0` neutral, `-` weak, `--` poor

---

## 8. Recommendation

### Primary: **Option A (Protobuf-First)** with a thin PCL adapter layer

**Rationale:**

1. **Protocol-agnostic by design** — protobuf is the most widely adopted IDL across defence, robotics, and cloud-native systems. It satisfies G1.

2. **gRPC is native** — no adapter needed. The `service TacticalObjects` definition produces client/server stubs directly. Satisfies G3.

3. **ROS2 bridge is tractable** — a ~200-line Python script can generate `.msg`/`.srv` from `.proto` using the `google.protobuf.descriptor` API. Several open-source examples exist. Satisfies G2.

4. **User-friendly** — `.proto` files are self-documenting, have first-class enum and `optional` support, and work with standard tools (`buf`, `grpcurl`, `Postman gRPC`). Satisfies G4.

5. **Backward-compatible** — the existing C++ structs remain as the in-process representation. A thin conversion layer (`to_proto()` / `from_proto()`) maps between the native structs and protobuf messages. The PCL service handlers continue to accept `pcl_msg_t`; the only change is that the payload is protobuf-encoded instead of ad-hoc binary. Satisfies G5.

### Migration path

```
Phase 1 — Define .proto files for all tactical object types and services
          (mirror existing TacticalObjectsTypes.h)

Phase 2 — Generate C++ bindings; write to_proto()/from_proto() converters
          Wire into TacticalObjectsComponent service handlers

Phase 3 — Generate ROS2 .msg/.srv from .proto (script)
          Update ros2/ message definitions

Phase 4 — Stand up gRPC server alongside PCL (optional, for external clients)

Phase 5 — Retire hand-rolled StreamingCodec and TacticalObjectsCodec
          (once all consumers use protobuf)
```

### If streaming throughput is critical

Consider **Option C (FlatBuffers)** for the `entity_updates` streaming path only, while using protobuf for request-reply services. FlatBuffers' zero-copy reads are a measurable win for high-frequency position updates at scale. The two can coexist — protobuf for services, FlatBuffers for the hot data path.

### If the team strongly prefers minimal tooling changes

**Option D (C++ Hybrid)** is the lowest-friction starting point. It can be introduced incrementally and later graduated to Option A once the schema stabilises.
