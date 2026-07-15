#!/usr/bin/env python3
"""
Codec Backend Registry

Defines the abstract CodecBackend interface that all serialisation backends
implement, plus a registry for discovering and invoking them.

Each backend generates language-specific (C++, Ada) code for serialising and
deserialising proto messages using a particular wire format, or for projecting
the service contract onto a transport such as gRPC or ROS2. The generated code
plugs into the existing PCL service binding layer.

No backend has hardcoded knowledge of specific proto message types; everything
is driven from the ProtoFile / ProtoTypeIndex parsed by proto_parser.py.
"""

from abc import ABC, abstractmethod
import inspect
from pathlib import Path
from typing import Dict, List, Optional

from proto_parser import ProtoTypeIndex


class CodecBackend(ABC):
    """Abstract interface for one serialisation/transport backend."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Short identifier: 'json', 'flatbuffers', 'protobuf', 'grpc', 'ros2'."""
        ...

    @property
    @abstractmethod
    def content_type(self) -> str:
        """MIME-style type for pcl_msg_t.type_name: 'application/json', etc."""
        ...

    @abstractmethod
    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        """Generate C++ codec files. Returns list of generated file paths."""
        ...

    @abstractmethod
    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        """Generate Ada codec files. Returns list of generated file paths."""
        ...


# -- Backend registry ---------------------------------------------------------

_REGISTRY: Dict[str, CodecBackend] = {}


def register(backend: CodecBackend) -> None:
    """Register a backend instance by its name."""
    _REGISTRY[backend.name] = backend


def get(name: str) -> Optional[CodecBackend]:
    """Look up a registered backend by name."""
    return _REGISTRY.get(name)


def all_backends() -> Dict[str, CodecBackend]:
    """Return all registered backends."""
    return dict(_REGISTRY)


def generate_all(index: ProtoTypeIndex, output_dir: Path,
                 languages: Optional[List[str]] = None,
                 backends: Optional[List[str]] = None,
                 naming_policy=None,
                 proto_import_root: Optional[Path] = None,
                 contract=None,
                 metadata=None) -> Dict[str, List[Path]]:
    """Generate codec files for all (or selected) backends and languages.

    Args:
        index: Parsed proto type index.
        output_dir: Root output directory.
        languages: List of language keys ('cpp', 'ada'). None = all.
        backends: List of backend names. None = all registered.
        naming_policy: Optional layout naming policy for policy-aware backends.
        proto_import_root: Optional protoc import root for path-aware backends.
        contract: Optional neutral binding contract for contract-aware backends.
        metadata: Optional contract identity (binding_metadata.json) for
            backends that stamp it into the artifacts they generate.

    Returns:
        Dict mapping backend name to list of generated file paths.
    """
    langs = set(languages or ['cpp', 'ada'])
    selected = backends or list(_REGISTRY.keys())
    result: Dict[str, List[Path]] = {}

    for name in selected:
        backend = _REGISTRY.get(name)
        if backend is None:
            raise ValueError(f'Unknown codec backend: {name!r}')

        generated: List[Path] = []
        backend_dir = output_dir / name

        if 'cpp' in langs:
            generated.extend(_call_generate(
                backend.generate_cpp,
                index,
                backend_dir / 'cpp',
                naming_policy=naming_policy,
                proto_import_root=proto_import_root,
                contract=contract,
                metadata=metadata,
            ))

        if 'ada' in langs:
            generated.extend(backend.generate_ada(index, backend_dir / 'ada'))

        result[name] = generated

    return result


def _call_generate(method, index: ProtoTypeIndex, output_dir: Path, **kwargs):
    """Call a backend generator with only the optional knobs it declares."""
    accepted = inspect.signature(method).parameters
    filtered = {
        key: value
        for key, value in kwargs.items()
        if key in accepted
    }
    return method(index, output_dir, **filtered)
