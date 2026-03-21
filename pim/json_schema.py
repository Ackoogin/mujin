"""json_schema.py — Canonical JSON wire format for the pyramid standard bridge.

This module is the single source of truth for:
  - Which JSON field names map to which proto message fields
  - How enum values are represented as strings on the wire
  - The Ada record type names for each message

The Ada JSON codec package (pyramid-services-tactical_objects-json_codec)
is generated from this schema by ada_service_generator.py.

Wire format sources:
  - StandardBridge.h documentation comments
  - proto/pyramid/data_model/common.proto   (enums, Entity, Query, Ack)
  - proto/pyramid/data_model/tactical.proto (ObjectDetail,
      ObjectEvidenceRequirement, ObjectInterestRequirement, ObjectMatch)
  - proto/pyramid/components/tactical_objects/services/provided.proto
  - proto/pyramid/components/tactical_objects/services/consumed.proto
"""

from typing import List


# ── Field kinds ───────────────────────────────────────────────────────────────

class FieldKind:
    STRING = 'string'
    DOUBLE = 'double'
    BOOL = 'bool'
    STANDARD_IDENTITY = 'StandardIdentity'
    BATTLE_DIMENSION = 'BattleDimension'
    DATA_POLICY = 'DataPolicy'
    IDENTIFIER = 'Identifier'


# ── Field descriptor ──────────────────────────────────────────────────────────

class Field:
    """Describes one JSON key/value pair in a message schema."""

    def __init__(self, name: str, kind: str, required: bool = True,
                 description: str = ''):
        self.name = name            # JSON key (snake_case)
        self.kind = kind            # one of FieldKind.*
        self.required = required
        self.description = description

    # -- Ada name helpers -------------------------------------------------

    @property
    def ada_field_name(self) -> str:
        """Ada record component name (Title_Case)."""
        return '_'.join(w.capitalize() for w in self.name.split('_'))

    @property
    def ada_type(self) -> str:
        return {
            FieldKind.STRING:            'Unbounded_String',
            FieldKind.DOUBLE:            'Long_Float',
            FieldKind.BOOL:              'Boolean',
            FieldKind.STANDARD_IDENTITY: 'Standard_Identity',
            FieldKind.BATTLE_DIMENSION:  'Battle_Dimension',
            FieldKind.DATA_POLICY:       'Data_Policy',
            FieldKind.IDENTIFIER:        'Unbounded_String',
        }[self.kind]

    @property
    def ada_default(self) -> str:
        return {
            FieldKind.STRING:            'Null_Unbounded_String',
            FieldKind.DOUBLE:            '0.0',
            FieldKind.BOOL:              'False',
            FieldKind.STANDARD_IDENTITY: 'Identity_Unspecified',
            FieldKind.BATTLE_DIMENSION:  'Dimension_Unspecified',
            FieldKind.DATA_POLICY:       'Policy_Unspecified',
            FieldKind.IDENTIFIER:        'Null_Unbounded_String',
        }[self.kind]

    @property
    def is_enum(self) -> bool:
        return self.kind in (FieldKind.STANDARD_IDENTITY,
                             FieldKind.BATTLE_DIMENSION,
                             FieldKind.DATA_POLICY)


# ── Enum string tables ────────────────────────────────────────────────────────
# Each entry: (proto enum name string, Ada enum literal).
# The first entry is the default/unspecified value.

STANDARD_IDENTITY_TABLE = [
    ('STANDARD_IDENTITY_UNSPECIFIED',     'Identity_Unspecified'),
    ('STANDARD_IDENTITY_UNKNOWN',         'Identity_Unknown'),
    ('STANDARD_IDENTITY_FRIENDLY',        'Identity_Friendly'),
    ('STANDARD_IDENTITY_HOSTILE',         'Identity_Hostile'),
    ('STANDARD_IDENTITY_SUSPECT',         'Identity_Suspect'),
    ('STANDARD_IDENTITY_NEUTRAL',         'Identity_Neutral'),
    ('STANDARD_IDENTITY_PENDING',         'Identity_Pending'),
    ('STANDARD_IDENTITY_JOKER',           'Identity_Joker'),
    ('STANDARD_IDENTITY_FAKER',           'Identity_Faker'),
    ('STANDARD_IDENTITY_ASSUMED_FRIENDLY','Identity_Assumed_Friendly'),
]

BATTLE_DIMENSION_TABLE = [
    ('BATTLE_DIMENSION_UNSPECIFIED', 'Dimension_Unspecified'),
    ('BATTLE_DIMENSION_GROUND',      'Ground'),
    ('BATTLE_DIMENSION_SUBSURFACE',  'Subsurface'),
    ('BATTLE_DIMENSION_SEA_SURFACE', 'Sea_Surface'),
    ('BATTLE_DIMENSION_AIR',         'Air'),
    ('BATTLE_DIMENSION_UNKNOWN',     'Dimension_Unknown'),
]

DATA_POLICY_TABLE = [
    ('DATA_POLICY_UNSPECIFIED', 'Policy_Unspecified'),
    ('DATA_POLICY_QUERY',       'Policy_Query'),
    ('DATA_POLICY_OBTAIN',      'Policy_Obtain'),
]

ENUM_TABLES = {
    FieldKind.STANDARD_IDENTITY: {
        'table':    STANDARD_IDENTITY_TABLE,
        'ada_type': 'Standard_Identity',
        'default':  'Identity_Unspecified',
        'to_str_fn':   'Standard_Identity_To_String',
        'from_str_fn': 'Standard_Identity_From_String',
    },
    FieldKind.BATTLE_DIMENSION: {
        'table':    BATTLE_DIMENSION_TABLE,
        'ada_type': 'Battle_Dimension',
        'default':  'Dimension_Unspecified',
        'to_str_fn':   'Battle_Dimension_To_String',
        'from_str_fn': 'Battle_Dimension_From_String',
    },
    FieldKind.DATA_POLICY: {
        'table':    DATA_POLICY_TABLE,
        'ada_type': 'Data_Policy',
        'default':  'Policy_Unspecified',
        'to_str_fn':   'Data_Policy_To_String',
        'from_str_fn': 'Data_Policy_From_String',
    },
}


# ── Message schemas ───────────────────────────────────────────────────────────

class MessageSchema:
    """Describes one wire-format message type."""

    def __init__(self, ada_name: str, fields: List[Field],
                 wire_description: str = '', is_array_response: bool = False):
        self.ada_name = ada_name            # Ada record type name
        self.fields = fields
        self.wire_description = wire_description
        self.is_array_response = is_array_response  # True if wire format is a JSON array


# create_requirement request → proto: ObjectInterestRequirement
# Wire: {"policy": "DATA_POLICY_OBTAIN", "identity": "STANDARD_IDENTITY_HOSTILE",
#         "dimension": "BATTLE_DIMENSION_SEA_SURFACE",
#         "min_lat_rad": .., "max_lat_rad": .., "min_lon_rad": .., "max_lon_rad": ..}
CREATE_REQUIREMENT_REQUEST = MessageSchema(
    ada_name='Create_Requirement_Request',
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
    wire_description='element of standard.entity_matches array '
                     '(proto: ObjectMatch + position/confidence)',
    is_array_response=True,
    fields=[
        Field('object_id',     FieldKind.IDENTIFIER,       required=True),
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
# Topics the Ada client SUBSCRIBES to (server publishes these):
SUBSCRIBE_TOPICS = {
    'entity_matches':        'standard.entity_matches',
    'evidence_requirements': 'standard.evidence_requirements',
}

# Topics the Ada client PUBLISHES to (server subscribes to these):
PUBLISH_TOPICS = {
    'object_evidence': 'standard.object_evidence',
}
