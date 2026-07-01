"""Data-driven standard-topic metadata.

Topic wire names, payload types, and service bindings are NOT hard-coded here.
They are loaded from an explicit JSON metadata file (see ``topic_metadata/``).
The reusable logic in this module contains no domain knowledge: it only
resolves topics from whatever ``TopicMetadata`` it is given.

For backward compatibility the module-level default metadata is the PYRAMID
compatibility set (Tactical Objects standard topics). Callers that want no
topics -- e.g. arbitrary/generic proto layouts -- use ``EMPTY_METADATA`` or an
explicit ``TopicMetadata`` of their own.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple


_METADATA_DIR = Path(__file__).resolve().parent / 'topic_metadata'
_COMPAT_METADATA_PATH = _METADATA_DIR / 'tactical_objects_topics.json'


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


@dataclass(frozen=True)
class _Binding:
    package_match: str
    provided_subscribe: Tuple[str, ...] = ()
    provided_publish: Tuple[str, ...] = ()
    consumed_subscribe: Tuple[str, ...] = ()
    consumed_publish: Tuple[str, ...] = ()


@dataclass
class TopicMetadata:
    """Explicit topic metadata: specs keyed by topic key + service bindings.

    Holds no domain assumptions of its own -- everything comes from the data it
    is constructed with. An empty ``TopicMetadata`` yields no topics for any
    service, which is the correct default for arbitrary proto contracts.
    """

    specs: Dict[str, TopicSpec] = field(default_factory=dict)
    bindings: List[_Binding] = field(default_factory=list)

    def spec(self, key: str) -> TopicSpec:
        return self.specs[key]

    def topics_for_service(
        self, package: str, is_provided: bool
    ) -> Tuple[Dict[str, str], Dict[str, str]]:
        """Return (subscribe, publish) wire-name maps for a service package.

        A binding matches when ``binding.package_match`` is a substring of the
        (lower-cased) package. Selection is entirely metadata-driven; there is
        no built-in domain branching.
        """
        pkg_lower = package.lower()
        subscribe: Dict[str, str] = {}
        publish: Dict[str, str] = {}
        for binding in self.bindings:
            if binding.package_match.lower() not in pkg_lower:
                continue
            sub_keys = (binding.provided_subscribe if is_provided
                        else binding.consumed_subscribe)
            pub_keys = (binding.provided_publish if is_provided
                        else binding.consumed_publish)
            for key in sub_keys:
                subscribe[key] = self.specs[key].wire_name
            for key in pub_keys:
                publish[key] = self.specs[key].wire_name
        return subscribe, publish


def _binding_from_json(entry: dict) -> _Binding:
    provided = entry.get('provided', {})
    consumed = entry.get('consumed', {})
    return _Binding(
        package_match=entry['package_match'],
        provided_subscribe=tuple(provided.get('subscribe', ())),
        provided_publish=tuple(provided.get('publish', ())),
        consumed_subscribe=tuple(consumed.get('subscribe', ())),
        consumed_publish=tuple(consumed.get('publish', ())),
    )


def load_topic_metadata(path) -> TopicMetadata:
    """Load topic metadata from a JSON file."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    specs = {
        t['key']: TopicSpec(
            key=t['key'],
            wire_name=t['wire_name'],
            full_type=t['full_type'],
            is_array=bool(t.get('is_array', False)),
        )
        for t in data.get('topics', [])
    }
    bindings = [_binding_from_json(b) for b in data.get('bindings', [])]
    return TopicMetadata(specs=specs, bindings=bindings)


# Metadata with no topics -- the correct default for arbitrary/generic protos.
EMPTY_METADATA = TopicMetadata()

# PYRAMID compatibility default: the Tactical Objects standard topics, loaded
# from data. Keeps every existing call site producing identical output.
_DEFAULT_METADATA = load_topic_metadata(_COMPAT_METADATA_PATH)


def set_default_metadata(metadata: TopicMetadata) -> None:
    """Override the module-level default metadata (e.g. for generic layouts)."""
    global _DEFAULT_METADATA
    _DEFAULT_METADATA = metadata


def default_metadata() -> TopicMetadata:
    return _DEFAULT_METADATA


def topic_spec(key: str, metadata: TopicMetadata = None) -> TopicSpec:
    return (metadata or _DEFAULT_METADATA).spec(key)


def topics_for_service(
    package: str, is_provided: bool, metadata: TopicMetadata = None
) -> Tuple[Dict[str, str], Dict[str, str]]:
    return (metadata or _DEFAULT_METADATA).topics_for_service(
        package, is_provided)
