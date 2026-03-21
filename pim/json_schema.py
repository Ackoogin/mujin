"""json_schema.py — Canonical JSON wire format for the pyramid standard bridge.

This module is the single source of truth for:
  - Which JSON field names map to which proto message fields
  - How enum values are represented as strings on the wire
  - Ada and C++ type/literal names for each message

Enum values are parsed from the proto IDL files at generation time — no
enum literals are hardcoded here beyond the per-enum naming conventions
(which prefix to add, which proto file to read).

Wire format sources:
  - StandardBridge.h documentation comments
  - proto/pyramid/data_model/common.proto   (StandardIdentity, BattleDimension,
                                             DataPolicy, Entity, Ack, Query)
  - proto/pyramid/data_model/tactical.proto (ObjectDetail,
      ObjectEvidenceRequirement, ObjectInterestRequirement, ObjectMatch)
  - proto/pyramid/components/tactical_objects/services/provided.proto
  - proto/pyramid/components/tactical_objects/services/consumed.proto
"""

import re
from pathlib import Path
from typing import List, Optional, Set, Tuple

# Root of the repository (parent of this script's directory).
_PROTO_ROOT = Path(__file__).resolve().parent.parent


# ── Utility: minimal proto enum parser ────────────────────────────────────────

def _camel_to_screaming_snake_prefix(name: str) -> str:
    """CamelCase enum name → SCREAMING_SNAKE_CASE prefix with trailing _.

    StandardIdentity  →  STANDARD_IDENTITY_
    BattleDimension   →  BATTLE_DIMENSION_
    DataPolicy        →  DATA_POLICY_
    """
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s.upper() + '_'


def _parse_proto_enum(proto_path: Path, enum_name: str) -> List[Tuple[str, int]]:
    """Return [(value_name, ordinal), ...] for the named enum in a .proto file."""
    text = proto_path.read_text(encoding='utf-8')
    # Strip comments before matching, so option lines don't confuse the regex.
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    m = re.search(
        rf'\benum\s+{re.escape(enum_name)}\s*\{{([^}}]*)\}}',
        text, re.DOTALL,
    )
    if not m:
        raise ValueError(f'Enum {enum_name!r} not found in {proto_path}')
    result: List[Tuple[str, int]] = []
    for line in m.group(1).splitlines():
        vm = re.match(r'\s*(\w+)\s*=\s*(\d+)\s*;', line)
        if vm:
            result.append((vm.group(1), int(vm.group(2))))
    return result


def _to_ada_literal(suffix: str, ada_prefix: str,
                    prefixed_suffixes: Optional[Set[str]]) -> str:
    """Build Ada enum literal from a SCREAMING_SNAKE suffix.

    suffix             – e.g. 'HOSTILE', 'SEA_SURFACE', 'ASSUMED_FRIENDLY'
    ada_prefix         – e.g. 'Identity_'
    prefixed_suffixes  – if None, prefix is always added;
                         if a set, only those suffixes get the prefix.
    """
    title = '_'.join(w.capitalize() for w in suffix.split('_'))
    use_prefix = (prefixed_suffixes is None) or (suffix in prefixed_suffixes)
    return (ada_prefix + title) if use_prefix else title


def _to_cpp_literal(suffix: str) -> str:
    """SCREAMING_SNAKE suffix → CamelCase C++ enum value (no underscores).

    SEA_SURFACE       → SeaSurface
    ASSUMED_FRIENDLY  → AssumedFriendly
    """
    return ''.join(w.capitalize() for w in suffix.split('_'))


# ── Field kinds ───────────────────────────────────────────────────────────────

class FieldKind:
    STRING            = 'string'
    DOUBLE            = 'double'
    BOOL              = 'bool'
    IDENTIFIER        = 'Identifier'
    # Enum kinds — values match proto enum names and are used as ENUM_SPECS keys.
    STANDARD_IDENTITY = 'StandardIdentity'
    BATTLE_DIMENSION  = 'BattleDimension'
    DATA_POLICY       = 'DataPolicy'


# ── EnumSpec ──────────────────────────────────────────────────────────────────

class EnumSpec:
    """Configuration for one proto enum plus its Ada/C++ naming conventions.

    The enum VALUES themselves are parsed from the proto file lazily; only
    the naming rules (prefix stripping, literal prefix insertion) are
    configured here.
    """

    def __init__(
        self,
        proto_file: str,
        proto_enum: str,
        field_kind: str,
        ada_type: str,
        ada_prefix: str,
        cpp_type: str,
        ada_prefixed_suffixes: Optional[Set[str]] = None,
    ):
        """
        proto_file            – path relative to repo root
        proto_enum            – CamelCase enum name in the proto file
        field_kind            – FieldKind constant (used as dict key)
        ada_type              – Ada type name, e.g. 'Standard_Identity'
        ada_prefix            – prefix for Ada literals, e.g. 'Identity_'
        cpp_type              – C++ type name, e.g. 'StandardIdentity'
        ada_prefixed_suffixes – None → all values get ada_prefix;
                                set  → only these SCREAMING suffixes get it.
        """
        self.proto_file = proto_file
        self.proto_enum = proto_enum
        self.field_kind = field_kind
        self.ada_type = ada_type
        self.ada_prefix = ada_prefix
        self.cpp_type = cpp_type
        self.ada_prefixed_suffixes = ada_prefixed_suffixes
        self._prefix = _camel_to_screaming_snake_prefix(proto_enum)
        self._table: Optional[List[Tuple[str, str, str, int]]] = None

    # -- Ada helper names -------------------------------------------------------

    @property
    def ada_to_str_fn(self) -> str:
        return f'{self.ada_type}_To_String'

    @property
    def ada_from_str_fn(self) -> str:
        return f'{self.ada_type}_From_String'

    # -- Default (zero-ordinal / Unspecified) literals --------------------------

    @property
    def default_ada(self) -> str:
        return _to_ada_literal('UNSPECIFIED', self.ada_prefix,
                               self.ada_prefixed_suffixes)

    @property
    def default_cpp(self) -> str:
        return 'Unspecified'

    # -- Table loading ----------------------------------------------------------

    def table(self, root: Path = _PROTO_ROOT) -> List[Tuple[str, str, str, int]]:
        """Load and return [(proto_name, ada_literal, cpp_literal, ordinal), ...].

        The result is cached after the first call.
        """
        if self._table is None:
            raw = _parse_proto_enum(root / self.proto_file, self.proto_enum)
            entries: List[Tuple[str, str, str, int]] = []
            for name, ordinal in raw:
                if not name.startswith(self._prefix):
                    continue  # skip reserved or unrecognised lines
                suffix = name[len(self._prefix):]
                ada_lit = _to_ada_literal(suffix, self.ada_prefix,
                                          self.ada_prefixed_suffixes)
                cpp_lit = _to_cpp_literal(suffix)
                entries.append((name, ada_lit, cpp_lit, ordinal))
            self._table = entries
        return self._table


# ── Enum specifications ───────────────────────────────────────────────────────
# Each entry configures how one proto enum maps to Ada/C++ literals.
# No enum value strings are hardcoded; they are read from proto at
# generation time via EnumSpec.table().

STANDARD_IDENTITY_SPEC = EnumSpec(
    proto_file='proto/pyramid/data_model/common.proto',
    proto_enum='StandardIdentity',
    field_kind=FieldKind.STANDARD_IDENTITY,
    ada_type='Standard_Identity',
    ada_prefix='Identity_',
    cpp_type='StandardIdentity',
    ada_prefixed_suffixes=None,          # all values get Identity_ prefix
)

BATTLE_DIMENSION_SPEC = EnumSpec(
    proto_file='proto/pyramid/data_model/common.proto',
    proto_enum='BattleDimension',
    field_kind=FieldKind.BATTLE_DIMENSION,
    ada_type='Battle_Dimension',
    ada_prefix='Dimension_',
    cpp_type='BattleDimension',
    ada_prefixed_suffixes={'UNSPECIFIED', 'UNKNOWN'},  # generic/ambiguous values only
)

DATA_POLICY_SPEC = EnumSpec(
    proto_file='proto/pyramid/data_model/common.proto',
    proto_enum='DataPolicy',
    field_kind=FieldKind.DATA_POLICY,
    ada_type='Data_Policy',
    ada_prefix='Policy_',
    cpp_type='DataPolicy',
    ada_prefixed_suffixes=None,          # all values get Policy_ prefix
)

# Lookup by FieldKind string — add new enum specs here to extend the codec.
ENUM_SPECS = {
    FieldKind.STANDARD_IDENTITY: STANDARD_IDENTITY_SPEC,
    FieldKind.BATTLE_DIMENSION:  BATTLE_DIMENSION_SPEC,
    FieldKind.DATA_POLICY:       DATA_POLICY_SPEC,
}


# ── Field descriptor ──────────────────────────────────────────────────────────

class Field:
    """Describes one JSON key/value pair in a message schema."""

    def __init__(self, name: str, kind: str, required: bool = True,
                 description: str = ''):
        self.name = name        # JSON key (snake_case)
        self.kind = kind        # one of FieldKind.*
        self.required = required
        self.description = description

    # -- General helpers -------------------------------------------------------

    @property
    def is_enum(self) -> bool:
        # Resolved at call time so ENUM_SPECS forward reference is fine.
        return self.kind in ENUM_SPECS

    # -- Ada name/type helpers -------------------------------------------------

    @property
    def ada_field_name(self) -> str:
        """Ada record component name (Title_Case)."""
        return '_'.join(w.capitalize() for w in self.name.split('_'))

    @property
    def ada_type(self) -> str:
        if self.kind in ENUM_SPECS:
            return ENUM_SPECS[self.kind].ada_type
        return {
            FieldKind.STRING:     'Unbounded_String',
            FieldKind.DOUBLE:     'Long_Float',
            FieldKind.BOOL:       'Boolean',
            FieldKind.IDENTIFIER: 'Unbounded_String',
        }[self.kind]

    @property
    def ada_default(self) -> str:
        if self.kind in ENUM_SPECS:
            return ENUM_SPECS[self.kind].default_ada
        return {
            FieldKind.STRING:     'Null_Unbounded_String',
            FieldKind.DOUBLE:     '0.0',
            FieldKind.BOOL:       'False',
            FieldKind.IDENTIFIER: 'Null_Unbounded_String',
        }[self.kind]

    # -- C++ name/type helpers -------------------------------------------------

    @property
    def cpp_type(self) -> str:
        if self.kind in ENUM_SPECS:
            return ENUM_SPECS[self.kind].cpp_type
        return {
            FieldKind.STRING:     'std::string',
            FieldKind.DOUBLE:     'double',
            FieldKind.BOOL:       'bool',
            FieldKind.IDENTIFIER: 'std::string',
        }[self.kind]

    @property
    def cpp_default(self) -> str:
        if self.kind in ENUM_SPECS:
            spec = ENUM_SPECS[self.kind]
            return f'{spec.cpp_type}::{spec.default_cpp}'
        return {
            FieldKind.STRING:     '{}',
            FieldKind.DOUBLE:     '0.0',
            FieldKind.BOOL:       'false',
            FieldKind.IDENTIFIER: '{}',
        }[self.kind]


# ── Message schemas ───────────────────────────────────────────────────────────

class MessageSchema:
    """Describes one wire-format message type."""

    def __init__(self, ada_name: str, cpp_name: str, fields: List[Field],
                 wire_description: str = '', is_array_response: bool = False):
        self.ada_name = ada_name            # Ada record type name
        self.cpp_name = cpp_name            # C++ struct name
        self.fields = fields
        self.wire_description = wire_description
        self.is_array_response = is_array_response


# create_requirement request → proto: ObjectInterestRequirement
# Wire: {"policy": "DATA_POLICY_OBTAIN", "identity": "STANDARD_IDENTITY_HOSTILE",
#         "dimension": "BATTLE_DIMENSION_SEA_SURFACE",
#         "min_lat_rad": .., "max_lat_rad": .., "min_lon_rad": .., "max_lon_rad": ..}
CREATE_REQUIREMENT_REQUEST = MessageSchema(
    ada_name='Create_Requirement_Request',
    cpp_name='CreateRequirementRequest',
    wire_description='object_of_interest.create_requirement request '
                     '(proto: ObjectInterestRequirement)',
    fields=[
        Field('policy',      FieldKind.DATA_POLICY,       required=True),
        Field('identity',    FieldKind.STANDARD_IDENTITY, required=True),
        Field('dimension',   FieldKind.BATTLE_DIMENSION,  required=False,
              description='Optional battle-dimension filter'),
        Field('min_lat_rad', FieldKind.DOUBLE, required=False,
              description='Bounding-box south edge, radians'),
        Field('max_lat_rad', FieldKind.DOUBLE, required=False,
              description='Bounding-box north edge, radians'),
        Field('min_lon_rad', FieldKind.DOUBLE, required=False,
              description='Bounding-box west edge, radians'),
        Field('max_lon_rad', FieldKind.DOUBLE, required=False,
              description='Bounding-box east edge, radians'),
    ],
)

# create_requirement response → proto: Identifier
# Wire: {"interest_id": "<uuid>"}
CREATE_REQUIREMENT_RESPONSE = MessageSchema(
    ada_name='Create_Requirement_Response',
    cpp_name='CreateRequirementResponse',
    wire_description='object_of_interest.create_requirement response '
                     '(proto: Identifier)',
    fields=[
        Field('interest_id', FieldKind.IDENTIFIER, required=False,
              description='Assigned interest ID; absent when creation failed'),
    ],
)

# entity_matches element → proto: ObjectMatch (enriched with position/confidence)
# Wire: [{"object_id": "..", "identity": "..", "dimension": "..",
#          "latitude_rad": .., "longitude_rad": .., "confidence": ..}, ...]
ENTITY_MATCH = MessageSchema(
    ada_name='Entity_Match',
    cpp_name='EntityMatch',
    wire_description='element of standard.entity_matches array '
                     '(proto: ObjectMatch + position/confidence)',
    is_array_response=True,
    fields=[
        Field('object_id',     FieldKind.IDENTIFIER,        required=True),
        Field('identity',      FieldKind.STANDARD_IDENTITY, required=True),
        Field('dimension',     FieldKind.BATTLE_DIMENSION,  required=False),
        Field('latitude_rad',  FieldKind.DOUBLE,            required=False),
        Field('longitude_rad', FieldKind.DOUBLE,            required=False),
        Field('confidence',    FieldKind.DOUBLE,            required=False),
    ],
)

# object_evidence publish → proto: ObjectDetail
# Wire: {"identity": "..", "dimension": "..",
#         "latitude_rad": .., "longitude_rad": .., "confidence": ..,
#         "observed_at": ..}
OBJECT_EVIDENCE = MessageSchema(
    ada_name='Object_Evidence',
    cpp_name='ObjectEvidence',
    wire_description='standard.object_evidence publish payload '
                     '(proto: ObjectDetail)',
    fields=[
        Field('identity',      FieldKind.STANDARD_IDENTITY, required=True),
        Field('dimension',     FieldKind.BATTLE_DIMENSION,  required=True),
        Field('latitude_rad',  FieldKind.DOUBLE,            required=True),
        Field('longitude_rad', FieldKind.DOUBLE,            required=True),
        Field('confidence',    FieldKind.DOUBLE,            required=True),
        Field('observed_at',   FieldKind.DOUBLE,            required=False,
              description='Observation timestamp, seconds'),
    ],
)

# evidence_requirements subscribe → proto: ObjectEvidenceRequirement
# Wire: {"id": "..", "policy": "..", "dimension": "..",
#         "min_lat_rad": .., "max_lat_rad": ..,
#         "min_lon_rad": .., "max_lon_rad": ..}
EVIDENCE_REQUIREMENT = MessageSchema(
    ada_name='Evidence_Requirement',
    cpp_name='EvidenceRequirement',
    wire_description='standard.evidence_requirements subscribe payload '
                     '(proto: ObjectEvidenceRequirement)',
    fields=[
        Field('id',          FieldKind.IDENTIFIER,       required=False),
        Field('policy',      FieldKind.DATA_POLICY,      required=False),
        Field('dimension',   FieldKind.BATTLE_DIMENSION, required=False),
        Field('min_lat_rad', FieldKind.DOUBLE,           required=False),
        Field('max_lat_rad', FieldKind.DOUBLE,           required=False),
        Field('min_lon_rad', FieldKind.DOUBLE,           required=False),
        Field('max_lon_rad', FieldKind.DOUBLE,           required=False),
    ],
)

# All schemas in generation order
ALL_SCHEMAS: List[MessageSchema] = [
    CREATE_REQUIREMENT_REQUEST,
    CREATE_REQUIREMENT_RESPONSE,
    ENTITY_MATCH,
    OBJECT_EVIDENCE,
    EVIDENCE_REQUIREMENT,
]

# ── Standard topic names ──────────────────────────────────────────────────────
# Topics the Ada/C++ client SUBSCRIBES to (server publishes these):
SUBSCRIBE_TOPICS = {
    'entity_matches':        'standard.entity_matches',
    'evidence_requirements': 'standard.evidence_requirements',
}

# Topics the Ada/C++ client PUBLISHES to (server subscribes to these):
PUBLISH_TOPICS = {
    'object_evidence': 'standard.object_evidence',
}
