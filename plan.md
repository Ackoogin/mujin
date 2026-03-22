# Multi-Codec PCL Service Binding Architecture

## Problem

The PCL service binding generators (`cpp_service_generator.py`, `ada_service_generator.py`)
currently produce only a JSON codec, with message schemas hardcoded in `json_schema.py`.
We need multiple codec backends generated from proto as the sole source of truth, with no
hardcoded knowledge of specific data models.

## Design Principles

1. **Proto is the single source of truth** — all codec generation derives from `.proto` files
2. **Generators are data-model agnostic** — no hardcoded message names, field names, or enum values
3. **All backends generated together** in a single invocation
4. **Same PCL service wrapping** pattern across all codec backends

## Codec Backends

| Backend | Serialisation | Transport | Dependencies |
|---------|--------------|-----------|-------------|
| JSON | nlohmann/json (C++), GNATCOLL.JSON (Ada) | PCL | Existing libs |
| FlatBuffers | flatc-generated code | PCL | flatbuffers runtime (header-only C++) |
| Protobuf | protoc-generated code | PCL | libprotobuf |
| gRPC | protoc + grpc plugin | gRPC (replaces PCL transport) | grpc++ / grpc-ada |

## Implementation Plan

### Step 1: Enhanced Proto Parser (`pim/proto_parser.py`)

Extract the proto parsing logic that currently lives duplicated across
`cpp_service_generator.py` and `ada_service_generator.py` into a standalone module that
parses **full** message definitions — not just service/RPC signatures.

```python
# proto_parser.py — full proto IDL parser

class ProtoEnum:
    name: str                          # e.g. "StandardIdentity"
    values: List[(str, int)]           # e.g. [("STANDARD_IDENTITY_UNSPECIFIED", 0), ...]

class ProtoField:
    name: str                          # e.g. "latitude"
    type: str                          # e.g. "double", "StandardIdentity", "Position"
    number: int                        # proto field number
    label: str                         # "optional" | "repeated" | "required" | ""
    is_map: bool
    oneof_group: Optional[str]

class ProtoMessage:
    name: str                          # e.g. "ObjectDetail"
    fields: List[ProtoField]
    nested_enums: List[ProtoEnum]
    nested_messages: List[ProtoMessage]

class ProtoRpc:
    name: str
    request_type: str
    response_type: str
    server_streaming: bool

class ProtoService:
    name: str
    rpcs: List[ProtoRpc]

class ProtoFile:
    path: Path
    package: str
    imports: List[str]
    enums: List[ProtoEnum]
    messages: List[ProtoMessage]
    services: List[ProtoService]

def parse_proto(path: Path) -> ProtoFile: ...
def parse_proto_tree(root: Path) -> List[ProtoFile]: ...
```

This replaces the duplicated `parse_proto()` in both generators and the regex-based
enum parser in `json_schema.py`.

### Step 2: Codec Backend Interface (`pim/codec_backends.py`)

Abstract interface that each backend implements, plus a registry.

```python
class CodecBackend(ABC):
    """Abstract base for a serialisation codec backend."""

    @property
    @abstractmethod
    def name(self) -> str: ...          # "json", "flatbuffers", "protobuf", "grpc"

    @abstractmethod
    def generate_cpp(self, proto_files: List[ProtoFile], output_dir: Path) -> None: ...

    @abstractmethod
    def generate_ada(self, proto_files: List[ProtoFile], output_dir: Path) -> None: ...

BACKENDS: Dict[str, CodecBackend] = {}

def register_backend(backend: CodecBackend): ...
def generate_all(proto_files, output_dir): ...
```

### Step 3: JSON Backend (`pim/backends/json_backend.py`)

Refactor the existing `CppJsonCodecGenerator` and `JsonCodecGenerator` (Ada) into this
backend. The logic stays the same but now receives parsed `ProtoMessage` / `ProtoEnum`
objects instead of hardcoded `json_schema.ALL_SCHEMAS`.

Key change: enum string representations, field names, and type mappings are all derived
from the proto parse tree. The `json_schema.py` file becomes unnecessary — its naming
conventions (snake_case JSON keys, SCREAMING_SNAKE enum strings) become configurable
policies on the JSON backend.

### Step 4: FlatBuffers Backend (`pim/backends/flatbuffers_backend.py`)

**Phase A — Schema generation:** Proto → `.fbs` schema file.

FlatBuffers schemas map almost 1:1 from proto:
- `message` → `table`
- `enum` → `enum` (with explicit values)
- `repeated` → `[Type]` (vector)
- `oneof` → `union`
- Scalar types map directly (double, bool, int32, etc.)
- `string` stays `string`

The generator emits `.fbs` files from the parsed proto tree.

**Phase B — PCL wrapper generation:** Generate C++ / Ada code that:
1. Calls `flatbuffers::GetRoot<T>()` / `CreateT()` for ser/de
2. Wraps the result into `pcl_msg_t` with `type_name = "application/flatbuffers"`
3. Provides the same `toBinary()` / `fromBinary()` API surface as other codecs

**Dependencies:**
- `flatc` compiler (build-time only, for verifier generation)
- `flatbuffers/flatbuffers.h` (header-only runtime, ~50KB)
- Ada: use the C API via thin binding or generate manual pack/unpack

### Step 5: Protobuf Codec Backend (`pim/backends/protobuf_backend.py`)

This backend uses `protoc`-generated C++/Ada code for serialisation but still routes
through PCL transport (not gRPC).

**Generated code:**
- C++: include protoc-generated `.pb.h`, wrap `SerializeToString()` / `ParseFromString()`
  into the same codec API surface, set `type_name = "application/protobuf"`
- Ada: call C++ protobuf via thin C binding (protobuf has no native Ada support)

This gives maximum compatibility with existing proto tooling at the cost of the
libprotobuf dependency.

### Step 6: gRPC Transport Backend (`pim/backends/grpc_backend.py`)

This is a **transport** backend (not just a codec) — it replaces PCL entirely for
deployments that want standard gRPC.

**Generated code:**
- C++: `grpc::Service` implementation that delegates to the same `ServiceHandler`
  base class the PCL binding uses. One-to-one mapping of proto `service` blocks to
  gRPC service implementations.
- Ada: gRPC Ada binding (or C interop to grpc_core)

The service handler interface stays the same — component business logic is transport-agnostic:
```
component logic → ServiceHandler → {PCL binding | gRPC service}
```

### Step 7: Unified CLI

```bash
# Generate everything from proto (all backends, all languages)
python generate_bindings.py proto/pyramid/ output/

# Or selectively:
python generate_bindings.py proto/pyramid/ output/ --backends json,flatbuffers
python generate_bindings.py proto/pyramid/ output/ --languages cpp,ada
```

The existing `--codec` flag on `cpp_service_generator.py` / `ada_service_generator.py`
continues to work for backwards compatibility but delegates to the new backend system.

## File Structure

```
pim/
├── proto_parser.py              # Full proto IDL parser (Step 1)
├── codec_backends.py            # Backend interface + registry (Step 2)
├── generate_bindings.py         # Unified CLI entry point (Step 7)
├── backends/
│   ├── __init__.py
│   ├── json_backend.py          # JSON codec (refactored from existing) (Step 3)
│   ├── flatbuffers_backend.py   # FlatBuffers codec (Step 4)
│   ├── protobuf_backend.py      # Protobuf-over-PCL codec (Step 5)
│   └── grpc_backend.py          # Full gRPC transport (Step 6)
├── cpp_service_generator.py     # Existing — service stubs (unchanged API)
├── ada_service_generator.py     # Existing — service stubs (unchanged API)
└── json_schema.py               # Deprecated — kept for backwards compat only
```

## Migration

- `json_schema.py` becomes a thin shim that imports from the new proto parser + JSON backend
  for any existing code that references `ALL_SCHEMAS`, `ENUM_SPECS`, etc.
- Existing generator CLI (`--codec`) continues to work unchanged.
- Service binding generators (`CppServiceGenerator`, `AdaServiceGenerator`) are updated to
  use `proto_parser.py` instead of their inline parsers.

## Implementation Order

1. `proto_parser.py` — foundational, unblocks everything else
2. `codec_backends.py` — interface definition
3. JSON backend — refactor existing code, prove the interface works
4. FlatBuffers backend — the new capability the user asked for
5. Protobuf backend — straightforward wrapper
6. gRPC backend — transport-level, most complex
7. Unified CLI + backwards compat shims
