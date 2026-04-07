# Service Binding Code Generation — How It Works

> **Scope**: `TacticalObjects` component and its `StandardBridge`.
> This document covers the current state of the generation pipeline, the
> binary/language compatibility model, and the options available for wiring
> generated bindings into real components.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Generation Pipeline](#2-generation-pipeline)
3. [JSON Wire Format (Canonical Schema)](#3-json-wire-format-canonical-schema)
4. [Generated Artefacts — Ada](#4-generated-artefacts--ada)
5. [Generated Artefacts — C++](#5-generated-artefacts--c)
6. [Language and Binary Compatibility](#6-language-and-binary-compatibility)
7. [Integration Options](#7-integration-options)
8. [Adding a New Component / Service](#8-adding-a-new-component--service)
9. [Multi-Codec Architecture](#9-multi-codec-architecture)
10. [Quick-Reference Command Summary](#10-quick-reference-command-summary)

---

## 1. System Overview

```
  Proto IDL  ──►  subprojects/PYRAMID/pim/ada_service_generator.py  ──►  Ada generated/
             └──►  subprojects/PYRAMID/pim/cpp_service_generator.py  ──►  C++ generated/
                         ▲
             subprojects/PYRAMID/pim/json_schema.py  (canonical wire format & enum specs)
                         │
             subprojects/PYRAMID/proto/pyramid/data_model/common.proto  (enum definitions)
```

The **StandardBridge** (`pyramid/tactical_objects/include/StandardBridge.h`)
is the server-side component that accepts the standard protocol.  External
clients (Ada, C++, or any language) talk to it through **PCL** (Process
Component Language), a C-ABI transport layer.

```
  Ada/C++ client
      │
      │  JSON over PCL socket / in-process topic
      ▼
  StandardBridge   ◄──►  TacticalObjectsRuntime (internal binary format)
```

**Three wire-format layers** exist at the same time:

| Layer | Format | Owner |
|-------|--------|-------|
| Standard bridge | JSON (PROTO-string enum values) | `json_schema.py` |
| Internal service API | JSON (different field names) | `TacticalObjectsCodec` |
| Streaming entity updates | Binary batch frames | `StreamingCodec` |

The generated bindings and this document cover only the **standard bridge
layer**.

---

## 2. Generation Pipeline

### Source files

| File | Role |
|------|------|
| `subprojects/PYRAMID/pim/json_schema.py` | **Canonical authority** — wire field names, message shapes, enum naming rules |
| `subprojects/PYRAMID/pim/ada_service_generator.py` | Reads `.proto` + `json_schema`, emits Ada packages |
| `subprojects/PYRAMID/pim/cpp_service_generator.py` | Reads `.proto` + `json_schema`, emits C++ headers/sources |
| `subprojects/PYRAMID/pim/proto_parser.py` | Proto IDL parser — extracts messages, enums, services into `ProtoFile` / `ProtoTypeIndex` |
| `subprojects/PYRAMID/pim/generate_bindings.py` | Batch entry point — generates all bindings for a component |
| `subprojects/PYRAMID/pim/codec_backends.py` | Registry of binary codec backends (JSON, FlatBuffers, Protobuf) |
| `subprojects/PYRAMID/proto/pyramid/data_model/common.proto` | Source of enum values (`StandardIdentity`, `BattleDimension`, `DataPolicy`) |
| `subprojects/PYRAMID/proto/pyramid/data_model/tactical.proto` | Tactical data model types (`ObjectInterestRequirement`, `ObjectMatch`, etc.) |
| `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto` | Service contract for client-facing RPCs |
| `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/consumed.proto` | Service contract for evidence/capability RPCs |

### `json_schema.py` — what it does

`json_schema.py` is **the only file that knows the wire format**.  It:

- Defines `EnumSpec` objects that name the proto file and enum, plus the
  Ada prefix rule and C++ CamelCase rule for each enum type.
- Parses actual enum values from the `.proto` file at generation time —
  **no enum strings are hardcoded** in `json_schema.py` itself.
- Defines `MessageSchema` objects (field names, kinds, required flags) for
  each wire message.
- Exports `SUBSCRIBE_TOPICS` and `PUBLISH_TOPICS` dicts used by both
  generators to produce topic constants.

```
proto file  ──►  EnumSpec.table()  ──►  [(proto_string, ada_literal, cpp_literal, ordinal), ...]
```

Enum naming rules are configured once per enum type:

| Enum | Proto prefix stripped | Ada prefix | Ada special case | C++ |
|------|-----------------------|-----------|-----------------|-----|
| `StandardIdentity` | `STANDARD_IDENTITY_` | `Identity_` (all values) | — | `StandardIdentity::Hostile` |
| `BattleDimension` | `BATTLE_DIMENSION_` | `Dimension_` (all values) | — | `BattleDimension::SeaSurface` |
| `DataPolicy` | `DATA_POLICY_` | `Policy_` (all values) | — | `DataPolicy::Obtain` |

To add a new enum to the codec, add one `EnumSpec` entry to `ENUM_SPECS` in
`json_schema.py`.  No other file needs changing.

### Running the generators

```bash
cd <repo_root>

# Ada: data model types from all protos in data_model/
python subprojects/PYRAMID/pim/ada_service_generator.py --types \
    subprojects/PYRAMID/proto/pyramid/data_model subprojects/PYRAMID/examples/ada/generated

# Ada: data model codecs (To_Json / From_Json per type)
python subprojects/PYRAMID/pim/ada_service_generator.py --codec \
    subprojects/PYRAMID/proto/pyramid/data_model subprojects/PYRAMID/examples/ada/generated

# Ada: bridge-level Json_Codec (json_schema.py format) for one service proto
python subprojects/PYRAMID/pim/ada_service_generator.py --codec \
    subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    subprojects/PYRAMID/examples/ada/generated

# Ada: service binding for one proto
python subprojects/PYRAMID/pim/ada_service_generator.py \
    subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    subprojects/PYRAMID/examples/ada/generated

# C++: codec package only
python subprojects/PYRAMID/pim/cpp_service_generator.py --codec \
    subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    subprojects/PYRAMID/examples/cpp/generated

# C++: service binding for one proto
python subprojects/PYRAMID/pim/cpp_service_generator.py \
    subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    subprojects/PYRAMID/examples/cpp/generated
```

---

## 3. JSON Wire Format (Canonical Schema)

All enum values on the wire are the **proto enum string** (e.g.
`"STANDARD_IDENTITY_HOSTILE"`, `"BATTLE_DIMENSION_SEA_SURFACE"`).

### Messages

#### `create_requirement` request  →  `ObjectInterestRequirement`

Topic / service: `object_of_interest.create_requirement`

```json
{
  "policy":      "DATA_POLICY_OBTAIN",
  "identity":    "STANDARD_IDENTITY_HOSTILE",
  "dimension":   "BATTLE_DIMENSION_SEA_SURFACE",
  "min_lat_rad": 0.8726646,
  "max_lat_rad": 0.9075712,
  "min_lon_rad": -0.0174533,
  "max_lon_rad": 0.0174533
}
```

Fields `dimension`, `min_lat_rad`, `max_lat_rad`, `min_lon_rad`,
`max_lon_rad` are optional (omitted when zero / unspecified).

#### `create_requirement` response  →  `Identifier`

```json
{ "interest_id": "550e8400-e29b-41d4-a716-446655440000" }
```

#### `standard.entity_matches` publish  →  `ObjectMatch[]`

Topic: `standard.entity_matches`  (JSON array)

```json
[
  {
    "object_id":     "3f4e5d6c-…",
    "identity":      "STANDARD_IDENTITY_HOSTILE",
    "dimension":     "BATTLE_DIMENSION_SEA_SURFACE",
    "latitude_rad":  0.8901,
    "longitude_rad": 0.0012,
    "confidence":    0.92
  }
]
```

#### `standard.object_evidence` publish  →  `ObjectDetail`

Topic: `standard.object_evidence`

```json
{
  "identity":      "STANDARD_IDENTITY_HOSTILE",
  "dimension":     "BATTLE_DIMENSION_SEA_SURFACE",
  "latitude_rad":  0.8901,
  "longitude_rad": 0.0012,
  "confidence":    0.9,
  "observed_at":   0.5
}
```

#### `standard.evidence_requirements` publish  →  `ObjectEvidenceRequirement`

Topic: `standard.evidence_requirements`

```json
{
  "id":          "ae1b2c3d-…",
  "policy":      "DATA_POLICY_OBTAIN",
  "dimension":   "BATTLE_DIMENSION_SEA_SURFACE",
  "min_lat_rad": 0.8726646,
  "max_lat_rad": 0.9075712,
  "min_lon_rad": -0.0174533,
  "max_lon_rad":  0.0174533
}
```

---

## 4. Generated Artefacts — Ada

All files live in `subprojects/PYRAMID/examples/ada/generated/`.

### Package structure

```
Pyramid_Data_Model_Base_Types          (pyramid_data_model_base_types.ads)
Pyramid_Data_Model_Common_Types        (pyramid_data_model_common_types.ads)
Pyramid_Data_Model_Tactical_Types      (pyramid_data_model_tactical_types.ads)
Pyramid_Data_Model_Common_Types_Codec  (pyramid_data_model_common_types_codec.ads/.adb)
Pyramid_Data_Model_Tactical_Types_Codec(pyramid_data_model_tactical_types_codec.ads/.adb)
Pyramid_Data_Model_Base_Types_Codec    (pyramid_data_model_base_types_codec.ads/.adb)

Pyramid.Services.Tactical_Objects
├── Provided              (pyramid-services-tactical_objects-provided.ads/.adb)
├── Consumed              (pyramid-services-tactical_objects-consumed.ads/.adb)
└── Json_Codec            (pyramid-services-tactical_objects-json_codec.ads/.adb)
```

The data model types packages (`Pyramid_Data_Model_*_Types`) are generated from
`subprojects/PYRAMID/proto/pyramid/data_model/` via `--types`.  The per-type codecs
(`*_Types_Codec`) are generated via `--codec <data_model_dir>`.  The service
packages and bridge-level `Json_Codec` are generated from the service protos.

The packages depend on one hand-authored file that is **not** generated:

- `Pcl_Bindings` (`subprojects/PYRAMID/examples/ada/pcl_bindings.ads`) — thin Ada spec over the
  PCL C ABI.

### `Provided` package

| Symbol | Kind | Purpose |
|--------|------|---------|
| `Topic_Entity_Matches` | `constant String` | `"standard.entity_matches"` |
| `Topic_Evidence_Requirements` | `constant String` | `"standard.evidence_requirements"` |
| `Svc_Create_Requirement` | `constant String` | `"object_of_interest.create_requirement"` |
| `Subscribe_Entity_Matches` | procedure | Calls `pcl_container_add_subscriber` |
| `Subscribe_Evidence_Requirements` | procedure | Calls `pcl_container_add_subscriber` |
| `Invoke_Create_Requirement` | procedure | Calls `pcl_socket_transport_invoke_remote_async` |
| `Invoke_Read_Match` | procedure | ditto |
| `Handle_*` | procedure stubs | Override in component body |
| `Dispatch` | procedure | Routes raw PCL buffer to typed handler |
| `Msg_To_String` | function | `(Address, unsigned) → String` |

### `Consumed` package

| Symbol | Kind | Purpose |
|--------|------|---------|
| `Topic_Object_Evidence` | `constant String` | `"standard.object_evidence"` |
| `Publish_Object_Evidence` | procedure | Calls `pcl_executor_dispatch_incoming` |

### `Json_Codec` package

| Symbol | Kind | Notes |
|--------|------|-------|
| `Create_Requirement_Request` | record type | Policy, Identity, Dimension, bounding-box |
| `Create_Requirement_Response` | record type | Interest_Id |
| `Entity_Match` | record type | Object_Id, Identity, Dimension, lat/lon/confidence |
| `Object_Evidence` | record type | Identity, Dimension, lat/lon/confidence/observed_at |
| `Evidence_Requirement` | record type | Id, Policy, Dimension, bounding-box |
| `Entity_Match_Array` | array type | `array (Positive range <>)` |
| `To_Json(Msg : T) return String` | overloaded function | One per message type |
| `From_Json(S : String) return T` | overloaded function | One per message type |
| `Entity_Matches_From_Json` | function | Deserialises JSON array |
| `Standard_Identity_To_String` | function | Enum → proto string |
| `Standard_Identity_From_String` | function | Proto string → enum |
| `Battle_Dimension_To_String/From_String` | functions | ditto |
| `Data_Policy_To_String/From_String` | functions | ditto |

### Ada PCL callback conventions

All callbacks passed to PCL must have `pragma Convention (C, ...)` applied.
The generated `Provided` package wraps the raw C callbacks — component logic
never touches raw pointers directly.

```ada
--  Component registers a subscription in On_Configure:
Provided.Subscribe_Entity_Matches
  (Container => Container,
   Callback  => On_Entity_Matches'Unrestricted_Access);

--  Callback decodes using the codec:
Matches : constant Codec.Entity_Match_Array :=
  Codec.Entity_Matches_From_Json (Body_Str);
```

---

## 5. Generated Artefacts — C++

All files live in `subprojects/PYRAMID/examples/cpp/generated/`.

### Namespace structure

```
pyramid::services::tactical_objects
├── provided          (pyramid_services_tactical_objects_provided.hpp/.cpp)
├── consumed          (pyramid_services_tactical_objects_consumed.hpp/.cpp)
└── json_codec        (pyramid_services_tactical_objects_json_codec.hpp/.cpp)

pyramid::services::tactical_objects   (types only)
    (pyramid_services_tactical_objects_types.hpp)  — hand-authored, not generated
```

### `provided` namespace

| Symbol | Kind | Purpose |
|--------|------|---------|
| `kTopicEntityMatches` | `constexpr const char*` | `"standard.entity_matches"` |
| `kTopicEvidenceRequirements` | `constexpr const char*` | `"standard.evidence_requirements"` |
| `kSvcCreateRequirement` | `constexpr const char*` | `"object_of_interest.create_requirement"` |
| `subscribeEntityMatches` | free function | `pcl_container_add_subscriber` wrapper |
| `subscribeEvidenceRequirements` | free function | ditto |
| `invokeCreateRequirement` | free function | `pcl_socket_transport_invoke_remote_async` wrapper |
| `invokeReadMatch` etc. | free functions | ditto |
| `ServiceHandler` | abstract class | Override `handle*` methods |
| `dispatch` | free function | Routes raw PCL buffer to `ServiceHandler` |
| `msgToString` | free function | `(const void*, unsigned) → std::string` |

### `consumed` namespace

| Symbol | Kind | Purpose |
|--------|------|---------|
| `kTopicObjectEvidence` | `constexpr const char*` | `"standard.object_evidence"` |
| `subscribeObjectEvidence` | free function | Subscribe to server-published evidence |
| `publishObjectEvidence` | free function | `pcl_port_publish` wrapper |

### `json_codec` namespace

| Symbol | Kind | Notes |
|--------|------|-------|
| `CreateRequirementRequest` | struct | `DataPolicy`, `StandardIdentity`, `BattleDimension`, doubles |
| `CreateRequirementResponse` | struct | `std::string interest_id` |
| `EntityMatch` | struct | `object_id`, `identity`, `dimension`, lat/lon/confidence |
| `ObjectEvidence` | struct | identity/dimension/lat/lon/confidence/observed_at |
| `EvidenceRequirement` | struct | id, policy, dimension, bounding-box |
| `EntityMatchArray` | `using` alias | `std::vector<EntityMatch>` |
| `toJson(const T&)` | overloaded function | One per message type |
| `createRequirementRequestFromJson(const string&)` | function | Returns typed struct |
| `entityMatchesFromJson(const string&)` | function | Returns `EntityMatchArray` |
| `toString(StandardIdentity)` | overloaded | Enum → proto string |
| `standardIdentityFromString(const string&)` | function | Proto string → enum |
| `toString(BattleDimension)` / `battleDimensionFromString` | functions | ditto |
| `toString(DataPolicy)` / `dataPolicyFromString` | functions | ditto |

---

## 6. Language and Binary Compatibility

### Transport boundary — PCL C ABI

PCL exposes a **pure C interface** (`pcl_*.h`).  All cross-language
interoperability is achieved at this boundary, not at any higher level.

```
Ada component  ──►  pcl_bindings.ads (Ada thin spec)
                         │ pragma Import(C, ...)
                         ▼
                    pcl_*.h / libpcl.a   (C ABI)
                         │
C++ component  ──────────┘  (includes pcl headers directly)
```

Key implications:

- **No C++ exceptions** cross the boundary (PCL uses `pcl_status_t int`).
- **No Ada exceptions** cross the boundary (Ada `pragma Convention(C)` on
  callbacks suppresses exception propagation; do not let them escape).
- **No C++ RTTI** or vtables are exposed across the boundary.
- **No STL types** (`std::string`, `std::vector`) cross the boundary; only
  `const void*` + `size_t`.
- `pcl_msg_t.data` is a raw buffer; JSON serialisation happens before
  handing off to PCL and after receiving from PCL — always as `std::string`
  or Ada `String`.

### JSON as the lingua franca

The JSON wire format is the compatibility contract between languages.
Because all field names and enum strings are defined in `json_schema.py` and
validated against the `.proto` IDL, any language that can serialise to the
same JSON shape can participate.

The enum ordinals in `Pyramid_Data_Model_Common_Types` (Ada) and
`pyramid_services_tactical_objects_types.hpp` (C++) deliberately match the
proto ordinals, but the **wire format always uses the proto string name**
(e.g. `"BATTLE_DIMENSION_SEA_SURFACE"`), never the numeric ordinal.

### Binary compatibility risks

| Risk | Mitigation |
|------|-----------|
| Ada `pragma Convention(C)` callbacks may raise exceptions | Never raise in C-convention subprograms; log and return `PCL_ERR_CALLBACK` |
| `pcl_msg_t.data` lifetime — buffer owned by PCL, freed after callback returns | Copy to `String` / `std::string` at the start of every callback (via `Msg_To_String` / `msgToString`) |
| `pcl_port_t*` publisher used after PCL has destroyed it | Obtain during `on_configure`; do not cache across lifecycle transitions |
| Ada `Unrestricted_Access` on nested subprograms | Only take `Unrestricted_Access` of library-level subprograms; never of local ones |

---

## 7. Integration Options

The generated bindings support several integration patterns, in increasing
order of coupling to the standard bridge.

### Option A — In-process, same executor (simplest)

Both the client component and the `TacticalObjectsComponent` (which owns
`StandardBridge`) share a single `pcl_executor_t`.  Messages travel via
`pcl_executor_dispatch_incoming` — no sockets, no threads other than the
executor thread.

```
pcl::Executor exec;
exec.add(tobj);              // contains StandardBridge
exec.add(client_container);  // your Ada/C++ component

// Send a request (service call goes via in-process routing):
Provided::invokeCreateRequirement(transport, req_json, cb, user_data);
```

**Suitable for**: unit/integration tests, single-process deployments,
CI pipelines.

**Limitation**: both components must be in the same process and linked
against the same PCL shared library.

### Option B — Socket transport (separate processes or machines)

Client connects via `pcl_socket_transport_create_client`; server runs
`pcl_socket_transport_create_server` on a known port.  The PCL socket
transport marshals PCL messages over TCP; JSON payloads travel unchanged.

```
// Server side (TacticalObjectsComponent owns this):
pcl_socket_transport_t* srv =
    pcl_socket_transport_create_server(port, exec);

// Client side (Ada or C++):
pcl_socket_transport_t* transport =
    pcl_socket_transport_create_client(host, port, exec);
```

The `subprojects/PYRAMID/examples/ada/ada_active_find_e2e.adb` and
`subprojects/PYRAMID/examples/cpp/cpp_active_find_e2e.cpp` demonstrate this pattern end-to-end.

**Suitable for**: separate processes on the same machine, distributed
deployment, Ada/C++ client against a running server.

### Option C — Shared-memory transport (same host, low latency)

PCL also supports shared-memory transports.  The generated bindings are
transport-agnostic: `subscribe*` and `invoke*` wrappers only take a
`pcl_container_t*` or `pcl_socket_transport_t*`; swapping the transport
is purely a configuration decision at the executor level.

**Suitable for**: latency-sensitive, same-machine deployments.

### Option D — Server-side handler subclassing (C++ only)

The generated `ServiceHandler` base class can be subclassed to implement
the server side of the contract.  The `dispatch()` function routes raw PCL
buffers to the correct virtual method.

```cpp
class MyHandler : public provided::ServiceHandler {
    Identifier handleCreateRequirement(
        const ObjectInterestRequirement& req) override {
        // business logic here
        return Identifier{"some-uuid"};
    }
};
```

`TacticalObjectsComponent` uses this pattern internally via `StandardBridge`.
External components wanting to implement their own bridge can subclass
`ServiceHandler` and wire up `dispatch` as the PCL service handler.

**Ada** does not use class-based polymorphism for this; instead, override
the `Handle_*` procedure stubs in the generated package body.

### Option E — Bare PCL with manual JSON (no generated bindings)

For languages without generated bindings (Python, Rust, C, …) or for
integration testing, any client that can:

1. Open a PCL socket connection
2. Send a `pcl_msg_t` with `type_name = "application/json"` to the correct
   service wire name
3. Parse the JSON response

…will interoperate correctly.  The `subprojects/PYRAMID/examples/external_io_bridge_example.c`
shows this from plain C.

The canonical JSON field names and enum strings are the complete
specification; see §3 above.

---

## 8. Adding a New Component / Service

### Step 1 — Write the proto IDL

Add a new `.proto` file under `subprojects/PYRAMID/proto/pyramid/components/<component>/services/`.
Follow the EntityActions CRUD pattern:

```proto
service My_Thing_Service {
  rpc CreateThing(pyramid.data_model.tactical.ObjectInterestRequirement)
      returns (pyramid.data_model.base.Identifier);
  rpc ReadThing(pyramid.data_model.common.Query)
      returns (stream pyramid.data_model.tactical.ObjectInterestRequirement);
  // ...
}
```

### Step 2 — Extend the JSON schema (if new wire messages are needed)

In `subprojects/PYRAMID/pim/json_schema.py`:

1. If there are new enum types: add an `EnumSpec` entry to `ENUM_SPECS`.
2. Add `MessageSchema` entries to `ALL_SCHEMAS` for each new wire message.
3. Add topic names to `SUBSCRIBE_TOPICS` or `PUBLISH_TOPICS` if needed.

Enum values are read from the proto automatically — only the naming rule
needs to be configured.

### Step 3 — Regenerate

```bash
cd <repo_root>

# Ada: types + codecs + service bindings
python subprojects/PYRAMID/pim/ada_service_generator.py --types subprojects/PYRAMID/proto/pyramid/data_model subprojects/PYRAMID/examples/ada/generated
python subprojects/PYRAMID/pim/ada_service_generator.py --codec subprojects/PYRAMID/proto/pyramid/data_model subprojects/PYRAMID/examples/ada/generated
python subprojects/PYRAMID/pim/ada_service_generator.py --codec \
    subprojects/PYRAMID/proto/pyramid/components/<component>/services/provided.proto subprojects/PYRAMID/examples/ada/generated
python subprojects/PYRAMID/pim/ada_service_generator.py \
    subprojects/PYRAMID/proto/pyramid/components/<component>/services/provided.proto subprojects/PYRAMID/examples/ada/generated

# C++: codec + service bindings
python subprojects/PYRAMID/pim/cpp_service_generator.py --codec \
    subprojects/PYRAMID/proto/pyramid/components/<component>/services/provided.proto subprojects/PYRAMID/examples/cpp/generated
python subprojects/PYRAMID/pim/cpp_service_generator.py \
    subprojects/PYRAMID/proto/pyramid/components/<component>/services/provided.proto subprojects/PYRAMID/examples/cpp/generated
```

### Step 4 — Implement the component

- **C++**: Subclass `ServiceHandler`, implement `handle*` methods, pass
  `dispatch` as the PCL service callback.
- **Ada**: Implement the `Handle_*` procedure stubs in the generated package
  body; register `Subscribe_*` calls in `On_Configure`.

---

## 9. Multi-Codec Architecture

The binding system supports multiple wire codecs beyond JSON.  Codec
selection is **per-port** — each publisher, subscriber, or service endpoint
chooses its codec at configuration time via the `content_type` string passed
to `pcl_container_add_publisher` / `pcl_container_add_subscriber`.

### Supported codecs

| Content type | Backend | Notes |
|---|---|---|
| `application/json` | nlohmann/json (C++), GNATCOLL.JSON (Ada) | Default, always available |
| `application/flatbuffers` | FlatBuffers via flatc | Requires `CODEC_FLATBUFFERS` compile flag |
| `application/protobuf` | protoc-generated | Requires `CODEC_PROTOBUF` compile flag |
| gRPC | Full transport replacement | Separate backend, not dispatched via PCL |

### How it works

```
component logic
       │
       ▼
codec_dispatch::serialize(typed_struct, content_type)
       │
       ├── "application/json"        → json_codec::toJson()
       ├── "application/flatbuffers"  → flatbuffers_codec::toBinary()
       └── "application/protobuf"    → protobuf_codec::toBinary()
       │
       ▼
   pcl_msg_t { .data, .size, .type_name = content_type }
       │
       ▼
   PCL transport (in-process / socket / shared-memory)
       │
       ▼
subscriber callback receives pcl_msg_t with type_name preserved
       │
       ▼
codec_dispatch::deserialize*(data, size, content_type)
       │
       ▼
   typed struct returned to component business logic
```

### Proto as single source of truth

All codec backends are generated from `.proto` files by
`subprojects/PYRAMID/pim/generate_bindings.py`.  No codec generator has hardcoded knowledge of
specific data models — everything is derived from the proto IDL:

```bash
# Generate all backends for all languages
cd pim && python generate_bindings.py ../subprojects/PYRAMID/proto/pyramid/ output/

# Generate specific backends only
python generate_bindings.py ../subprojects/PYRAMID/proto/pyramid/ output/ --backends json,flatbuffers

# Generate for specific languages only
python generate_bindings.py ../subprojects/PYRAMID/proto/pyramid/ output/ --languages cpp
```

### Generation pipeline

```
subprojects/PYRAMID/proto/*.proto
      │
      ▼
  proto_parser.py          — parses proto IDL, builds ProtoTypeIndex
      │
      ├── backends/json_backend.py         → json_codec.{hpp,cpp,ads,adb}
      ├── backends/flatbuffers_backend.py  → .fbs + flatbuffers_codec.{hpp,ads}
      ├── backends/protobuf_backend.py     → protobuf_codec.{hpp,ads}
      ├── backends/grpc_backend.py         → grpc transport .{hpp,ads}
      │
      └── backends/codec_dispatch_generator.py
              → codec_dispatch.{hpp,cpp,ads,adb}
```

### Port-level codec configuration

The generated service binding wrappers (`subscribe*`, `publish*`, `invoke*`)
accept an optional `content_type` parameter that defaults to
`"application/json"`:

```cpp
// Default — backwards compatible, uses JSON
prov::subscribeEntityMatches(container, callback, user_data);

// Explicit — use FlatBuffers for this port
prov::subscribeEntityMatches(container, callback, user_data,
                              "application/flatbuffers");
```

```cpp
// Publish with explicit codec
cons::publishObjectEvidence(port, payload, "application/protobuf");
```

The content_type string flows through `pcl_msg_t.type_name` and is
preserved end-to-end: publisher → PCL transport → subscriber callback.
Components can read `msg->type_name` to determine which codec was used.

### Wire protocol

No changes to the PCL wire protocol are required.  `pcl_msg_t.type_name`
already existed and was previously hardcoded to `"application/json"`.  The
only change is that it is now configurable per-port.

PUBLISH frames carry `type_name` on the wire.  SERVICE frames do not — but
since codec selection is per-port (not per-message), both sides of a
service call agree on the codec at configuration time.

### Codec dispatch layer

The generated `codec_dispatch.hpp` provides `serialize()` and
`deserialize*()` functions that route on content_type:

```cpp
#include "pyramid_data_model_tactical_codec_dispatch.hpp"

namespace cd = pyramid::data_model::tactical::codec_dispatch;

// Serialize — routes to JSON, FlatBuffers, or Protobuf
std::string payload = cd::serialize(my_object_detail, content_type);

// Deserialize — routes to the correct codec
auto detail = cd::deserializeObjectDetail(msg->data, msg->size, content_type);

// Build pcl_msg_t from serialized payload
pcl_msg_t pcl_msg = cd::makeMsg(payload, content_type);
```

Binary codecs (FlatBuffers, Protobuf) are guarded by compile-time feature
flags (`CODEC_FLATBUFFERS`, `CODEC_PROTOBUF`).  If the flag is not defined,
attempting to use that codec throws `std::runtime_error`.

### E2E tests

`tests/test_codec_dispatch_e2e.cpp` validates multi-codec communication:

| Test | What it verifies |
|------|------------------|
| `JsonPubSubRoundTrip` | Publisher/subscriber with JSON codec, field-level round-trip |
| `ContentTypePropagation` | Non-JSON content type survives through pcl_msg_t |
| `PerPortCodecSelection` | Two publishers with different codecs, subscribers receive correct type |
| `DefaultCodecIsJson` | Generated subscribe wrappers default to "application/json" |
| `ExplicitCodecProtobuf` | Explicit "application/protobuf" content type |
| `JsonCodecSerDeRoundTrip` | JSON serialisation/deserialisation of all field types |
| `JsonCodecEnumRoundTrip` | All StandardIdentity enum values survive toString/fromString |

---

## 10. Quick-Reference Command Summary

```bash
# Regenerate Ada codec only
cd pim && python ada_service_generator.py --codec \
    ../subprojects/PYRAMID/examples/ada/generated

# Regenerate Ada service binding from one proto
cd pim && python ada_service_generator.py \
    ../subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    ../subprojects/PYRAMID/examples/ada/generated

# Regenerate C++ codec only
cd pim && python cpp_service_generator.py --codec \
    ../subprojects/PYRAMID/examples/cpp/generated

# Regenerate C++ service binding from one proto
cd pim && python cpp_service_generator.py \
    ../subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto \
    ../subprojects/PYRAMID/examples/cpp/generated

# Regenerate all four packages at once
cd pim
for proto in provided consumed; do
  python ada_service_generator.py \
      ../subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/${proto}.proto \
      ../subprojects/PYRAMID/examples/ada/generated
  python cpp_service_generator.py \
      ../subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/${proto}.proto \
      ../subprojects/PYRAMID/examples/cpp/generated
done
python ada_service_generator.py --codec ../subprojects/PYRAMID/examples/ada/generated
python cpp_service_generator.py --codec ../subprojects/PYRAMID/examples/cpp/generated
```

### File locations at a glance

```
subprojects/PYRAMID/pim/
├── json_schema.py               canonical wire format & enum specs
├── ada_service_generator.py     Ada generator
├── cpp_service_generator.py     C++ generator
├── proto_parser.py              proto IDL parser (single source of truth)
├── codec_backends.py            abstract CodecBackend + registry
├── generate_bindings.py         unified multi-codec generator CLI
└── backends/
    ├── json_backend.py          JSON codec backend
    ├── flatbuffers_backend.py   FlatBuffers codec backend
    ├── protobuf_backend.py      Protobuf codec backend
    ├── grpc_backend.py          gRPC transport backend
    └── codec_dispatch_generator.py  codec dispatch layer generator

subprojects/PYRAMID/proto/pyramid/
├── data_model/common.proto      enum definitions (StandardIdentity etc.)
├── data_model/tactical.proto    message definitions
└── components/tactical_objects/services/
    ├── provided.proto            client-facing RPC contract
    └── consumed.proto            evidence/capability RPC contract

subprojects/PYRAMID/examples/
├── ada/
│   ├── tactical_objects_types.ads   hand-authored Ada type mirror
│   ├── pcl_bindings.ads             Ada thin spec over PCL C ABI
│   ├── tobj_interest_client.{ads,adb}
│   ├── tobj_evidence_provider.{ads,adb}
│   ├── ada_active_find_e2e.adb      e2e driver (business logic only)
│   └── generated/
│       ├── pyramid-services-tactical_objects-provided.{ads,adb}
│       ├── pyramid-services-tactical_objects-consumed.{ads,adb}
│       └── pyramid-services-tactical_objects-json_codec.{ads,adb}
└── cpp/
    ├── tobj_interest_client.{hpp,cpp}
    ├── tobj_evidence_provider.{hpp,cpp}
    ├── cpp_active_find_e2e.cpp       e2e driver (business logic only)
    └── generated/
        ├── pyramid_services_tactical_objects_types.hpp   hand-authored
        ├── pyramid_services_tactical_objects_provided.{hpp,cpp}
        ├── pyramid_services_tactical_objects_consumed.{hpp,cpp}
        └── pyramid_services_tactical_objects_json_codec.{hpp,cpp}

tests/
└── test_codec_dispatch_e2e.cpp  multi-codec E2E tests
```

