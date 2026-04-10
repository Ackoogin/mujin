from dataclasses import dataclass
from typing import Dict, Tuple


def _camel_to_snake(name: str) -> str:
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0 and (not name[i - 1].isupper()):
            out.append('_')
        out.append(ch.lower())
    return ''.join(out)


@dataclass(frozen=True)
class TopicSpec:
    key: str
    wire_name: str
    full_type: str
    is_array: bool = False

    @property
    def short_type(self) -> str:
        return self.full_type.split('.')[-1]

    @property
    def cpp_payload_type(self) -> str:
        if self.is_array:
            return f'std::vector<{self.short_type}>'
        return self.short_type

    @property
    def ada_payload_type(self) -> str:
        base = _camel_to_snake(self.short_type).title().replace('_', '_')
        if self.is_array:
            return f'{base}_Array'
        return base

    @property
    def flatbuffers_suffix(self) -> str:
        if self.is_array:
            return f'{_camel_to_snake(self.short_type)}_array'
        return _camel_to_snake(self.short_type)


TACTICAL_OBJECTS_TOPIC_SPECS: Dict[str, TopicSpec] = {
    'entity_matches': TopicSpec(
        key='entity_matches',
        wire_name='standard.entity_matches',
        full_type='pyramid.data_model.tactical.ObjectMatch',
        is_array=True,
    ),
    'evidence_requirements': TopicSpec(
        key='evidence_requirements',
        wire_name='standard.evidence_requirements',
        full_type='pyramid.data_model.tactical.ObjectEvidenceRequirement',
    ),
    'object_evidence': TopicSpec(
        key='object_evidence',
        wire_name='standard.object_evidence',
        full_type='pyramid.data_model.tactical.ObjectDetail',
    ),
}


TACTICAL_OBJECTS_SUBSCRIBE_TOPICS: Dict[str, str] = {
    key: TACTICAL_OBJECTS_TOPIC_SPECS[key].wire_name
    for key in ('entity_matches', 'evidence_requirements')
}


TACTICAL_OBJECTS_PUBLISH_TOPICS: Dict[str, str] = {
    key: TACTICAL_OBJECTS_TOPIC_SPECS[key].wire_name
    for key in ('object_evidence',)
}


def topic_spec(key: str) -> TopicSpec:
    return TACTICAL_OBJECTS_TOPIC_SPECS[key]


def topics_for_service(package: str, is_provided: bool) -> Tuple[Dict[str, str], Dict[str, str]]:
    pkg_lower = package.lower()
    if 'tactical_objects' not in pkg_lower:
        return {}, {}
    if is_provided:
        return dict(TACTICAL_OBJECTS_SUBSCRIBE_TOPICS), {}
    return {}, dict(TACTICAL_OBJECTS_PUBLISH_TOPICS)
