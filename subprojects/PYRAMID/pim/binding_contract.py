#!/usr/bin/env python3
"""Neutral binding contract model and naming policies."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Protocol, Set, Tuple

from proto_parser import (
    ProtoEnum,
    ProtoFile,
    ProtoMessage,
    ProtoRpc,
    ProtoService,
    ProtoTypeIndex,
    _PROTO_SCALARS,
    camel_to_lower_snake,
)
import standard_topics


@dataclass(frozen=True)
class ServiceTypeClosure:
    """Request/response type closure for one proto service."""

    service_name: str
    package: str
    root_types: Tuple[str, ...] = ()
    types: Tuple[str, ...] = ()


@dataclass(frozen=True)
class BindingTopic:
    """One resolved pub/sub topic from parsed contract options or port grammar."""

    key: str
    wire_name: str
    full_type: str
    direction: str
    pattern: str
    service_name: str
    rpc_name: str
    qos: Dict[str, object] = field(default_factory=dict)
    is_array: bool = False

    # Contract-derived topics carry QoS constraints; the legacy
    # standard_topics.TopicSpec side table does not (has_qos = False there).
    has_qos = True

    @property
    def short_type(self) -> str:
        return self.full_type.split(".")[-1]

    @property
    def cpp_payload_type(self) -> str:
        if self.is_array:
            return f"std::vector<{self.short_type}>"
        return self.short_type

    @property
    def ada_payload_type(self) -> str:
        base = camel_to_lower_snake(self.short_type).title().replace("_", "_")
        if self.is_array:
            return f"{base}_Array"
        return base

    @property
    def flatbuffers_suffix(self) -> str:
        suffix = camel_to_lower_snake(self.short_type)
        return f"{suffix}_array" if self.is_array else suffix

    @property
    def reliability_floor(self) -> str:
        return _qos_reliability_floor(self.qos)


@dataclass(frozen=True)
class CommandProjectability:
    """D2's per-command pub/sub projectability classification.

    For one `Create`/`Update`/`Cancel` rpc of a Request-shape service: can
    the rpc's request type be carried as the wire payload of the service's
    `.request` topic? Two ways to say yes (D2):

    - the request type *is* the service's `_Request` wrapper message
      (``reason="is_wrapper"``, e.g. `Create` in every contract today), or
    - the request type matches exactly one of the wrapper's `oneof` variant
      field types (``reason="matches_variant"``, `variant_field` names it,
      e.g. `Cancel` -> the `cancel` variant).

    Anything else is not projectable: no wrapper message resolves
    (``reason="no_wrapper"``), the request type appears in more than one
    variant field -- an ill-formed wrapper (``reason="ambiguous_variant"``)
    -- or it appears in none (``reason="no_match"``, e.g. `Update` against
    every wrapper in the tree before this plan's Phase 0 A-GRA fix).

    This is computed independently of `topics_for_proto_service`'s
    first-rpc-wins `.request` topic derivation -- an additive record layered
    on top, not a replacement for it. `topics_for_proto_service` keeps
    projecting whichever rpc it processes first onto the topic regardless of
    this classification; Phase 1 is where the manifest/facade start
    consuming `projectable` to change behaviour.
    """

    service_name: str
    rpc_name: str
    request_type: str
    wrapper_type: str
    projectable: bool
    reason: str
    variant_field: str = ""


@dataclass(frozen=True)
class InteractionEndpoint:
    """One member of an interaction leg's side (Phase 1, D5/§2.3 of the plan).

    A thin projection of `EndpointRequirement`/`BindingTopic` identity onto
    the PCL routing grammar's `<endpoint_name> <kind>` shape -- not a new
    source of truth. `rpc_name` is set for service-side (rpc) members and
    empty for topic members. `projectable`/`reason` carry Phase 0's
    per-command classification (`CommandProjectability`) for the request
    leg's `Create`/`Update`/`Cancel` members; both are `None`/`""` where
    projectability does not apply (`Read` members, topic members).
    """

    endpoint_name: str
    kind: str
    rpc_name: str = ""
    projectable: Optional[bool] = None
    reason: str = ""


@dataclass(frozen=True)
class InteractionLeg:
    """One directed leg of an `Interaction`: two mutually-exclusive realizations.

    `group_name` is the deterministic name of the PCL `exclusive` group this
    leg compiles to (see D5): `f"{service_key}.{leg_name}_leg"`, where
    `service_key` is the same string `_service_key` produces elsewhere in
    this module (e.g. `pyramid.components.agra.mission_autonomy.services.provided.MAAction_Service`)
    and `leg_name` is `"request"` / `"requirement"` / `"information"`. This
    convention must be reproducible from `binding_manifest.json` data alone
    -- `contract_routing_manifest.py` (and any future manifest generator)
    recomputes it rather than reading a stored group name, so the naming
    rule is documented here as the single source of truth for it.

    `side_a` is the rpc/service realization (one or more service endpoints
    -- multiple for the request leg's `Create`/`Update`/`Cancel`, exactly one
    for the requirement/information leg's `Read`); `side_b` is the single
    pub/sub topic realization.
    """

    name: str  # "request" | "requirement" | "information"
    group_name: str
    side_a: Tuple[InteractionEndpoint, ...]
    side_b: Tuple[InteractionEndpoint, ...]


@dataclass(frozen=True)
class Interaction:
    """One Request or Information port's contract element (plan §2.1).

    A Request-shape service (`port_kind == "request"`) yields one
    `Interaction` with two legs (`request`, `requirement`); an
    Information-shape service (`port_kind == "information"`) yields one
    `Interaction` with a single `information` leg. Grouping only -- every
    endpoint referenced here also appears in `BindingContract.endpoint_requirements`;
    this is a view over that data keyed by leg, not a new source of truth
    (see D5, Phase 1 of `doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md`).
    """

    service_key: str
    service_name: str
    port_kind: str  # "request" | "information"
    legs: Tuple[InteractionLeg, ...]


@dataclass(frozen=True)
class EndpointRequirement:
    """One routeable endpoint requirement derived from the proto contract."""

    endpoint_name: str
    kind: str
    capability: str
    reliability: str
    source: str
    service_name: str = ""
    rpc_name: str = ""
    topic_key: str = ""
    topic_wire_name: str = ""
    qos: Dict[str, object] = field(default_factory=dict)


@dataclass
class BindingContract:
    """Parsed proto files classified by content, independent of package names."""

    proto_files: List[ProtoFile]
    type_modules: List[ProtoFile]
    service_modules: List[ProtoFile]
    wrapper_modules: List[ProtoFile]
    service_type_closures: Dict[str, ServiceTypeClosure] = field(default_factory=dict)
    service_topics: Dict[str, Tuple[BindingTopic, ...]] = field(default_factory=dict)
    endpoint_requirements: Tuple[EndpointRequirement, ...] = ()
    schema_ids: Dict[str, str] = field(default_factory=dict)
    naming_policy: "NamingPolicy" = field(default=None)
    # Phase 0 (D2): per-command pub/sub projectability, keyed the same way as
    # service_topics (see _service_key). Additive -- Phase 1 is the first
    # consumer; nothing in this contract's existing fields or behaviour
    # depends on it.
    command_projectability: Dict[str, Tuple[CommandProjectability, ...]] = field(
        default_factory=dict
    )
    # Phase 1 (D5): declared interactions (Request/Information ports grouped
    # into legs with two sides each), keyed the same way as service_topics /
    # command_projectability (see _service_key). Additive -- nothing in this
    # contract's existing fields or behaviour depends on it.
    interactions: Dict[str, "Interaction"] = field(default_factory=dict)


class NamingPolicy(Protocol):
    """Policy for mapping proto identities to generated C++ names."""

    layout: str

    def cpp_namespace_for_package(self, package: str) -> str:
        ...

    def cpp_type_namespace_for_package(self, package: str) -> str:
        ...

    def type_header_for_package(self, package: str) -> str:
        ...

    def codec_header_for_package(self, package: str) -> str:
        ...

    def codec_source_for_package(self, package: str) -> str:
        ...

    def umbrella_types_header(self, proto_files: Iterable[ProtoFile]) -> Optional[str]:
        ...

    def service_namespace(self, package: str) -> str:
        ...

    def service_file_prefix(self, package: str) -> str:
        ...

    def service_transport_file_prefix(self, package: str) -> str:
        ...

    def service_role(self, package: str) -> str:
        ...

    def ros2_support_file_prefix(self, package: str) -> str:
        ...

    def ros2_support_namespace(self, package: str) -> str:
        ...

    def ros2_message_package(self, package: str) -> str:
        ...

    def ros2_codec_header(self, package: str) -> str:
        ...

    def ros2_codec_namespace(self, package: str) -> str:
        ...

    def ros2_data_model_namespace(self, package: str) -> str:
        ...

    def ros2_envelope_type(self, package: str) -> str:
        ...

    def ros2_route_prefix(self, package: str) -> str:
        ...

    def grpc_plugin_aggregator_file_prefix(self, package: str) -> str:
        ...

    def grpc_plugin_symbol_prefix(self, package: str) -> str:
        ...

    def schema_id_for_type(self, full_type: str) -> str:
        ...

    def protobuf_codec_namespace(self, package: str) -> str:
        ...

    def flatbuffers_service_group_key(self, package: str) -> Optional[str]:
        ...

    def flatbuffers_service_namespace(self, package: str) -> str:
        ...

    def flatbuffers_service_file_prefix(self, package: str) -> str:
        ...


class PyramidCompatNamingPolicy:
    """Naming policy that preserves the existing PYRAMID generated surface."""

    layout = "pyramid"
    data_model_proto_root = "pyramid.data_model"
    data_model_types_namespace = "pyramid::domain_model"
    data_model_types_header = "pyramid_data_model_types.hpp"

    # Canonical base/common short-name mapping for the PYRAMID compat layout.
    # Single source of truth for the pyramid.data_model.base.* / common.* domain
    # literals so the language naming modules (cpp/ada) and the gRPC backend do
    # not each hardcode them. Each value equals the type's own last segment, so
    # the mapping is a semantic anchor rather than a rename.
    base_type_map = {
        "pyramid.data_model.base.Identifier": "Identifier",
        "pyramid.data_model.base.Query": "Query",
        "pyramid.data_model.base.Ack": "Ack",
        "pyramid.data_model.common.Query": "Query",
        "pyramid.data_model.common.Ack": "Ack",
    }

    def cpp_namespace_for_package(self, package: str) -> str:
        if package == self.data_model_proto_root:
            return self.data_model_types_namespace
        if package.startswith(self.data_model_proto_root + "."):
            suffix = package[len(self.data_model_proto_root) + 1:]
            return self.data_model_types_namespace + "::" + suffix.replace(".", "::")
        return package.replace(".", "::")

    def cpp_type_namespace_for_package(self, package: str) -> str:
        return self.cpp_namespace_for_package(package)

    def type_header_for_package(self, package: str) -> str:
        return package.replace(".", "_") + "_types.hpp"

    def codec_header_for_package(self, package: str) -> str:
        return package.replace(".", "_") + "_codec.hpp"

    def codec_source_for_package(self, package: str) -> str:
        return package.replace(".", "_") + "_codec.cpp"

    def umbrella_types_header(self, proto_files: Iterable[ProtoFile]) -> Optional[str]:
        del proto_files
        return self.data_model_types_header

    def legacy_service_namespace(self, package: str) -> Tuple[str, str]:
        parts = package.split(".")

        last = parts[-1].lower()
        suffix = None
        if last in ("provided", "consumed"):
            suffix = last
            parts = parts[:-1]

        skip = {"pyramid", "components", "data_model", "base", "services"}
        meaningful = [p for p in parts if p.lower() not in skip]

        ns_parts = ["pyramid", "services"] + [p.lower() for p in meaningful]
        suffix = suffix or "provided"
        return "::".join(ns_parts), "_".join(ns_parts + [suffix])

    def service_namespace(self, package: str) -> str:
        parts = package.split(".")
        last = parts[-1].lower()
        suffix = None
        if last in ("provided", "consumed"):
            suffix = last
            parts = parts[:-1]
        suffix = suffix or "provided"
        return "::".join(parts + [suffix])

    def service_file_prefix(self, package: str) -> str:
        return self.legacy_service_namespace(package)[1]

    def service_transport_file_prefix(self, package: str) -> str:
        return package.replace(".", "_")

    def service_role(self, package: str) -> str:
        return "provided" if "provided" in package.lower() else "consumed"

    def ros2_support_file_prefix(self, package: str) -> str:
        del package
        return "pyramid_ros2_transport_support"

    def ros2_support_namespace(self, package: str) -> str:
        del package
        return "pyramid::transport::ros2"

    def ros2_message_package(self, package: str) -> str:
        del package
        return "pyramid_msgs"

    def ros2_codec_header(self, package: str) -> str:
        del package
        return "pyramid_ros2_codec.hpp"

    def ros2_codec_namespace(self, package: str) -> str:
        del package
        return "pyramid::ros2_codec"

    def ros2_data_model_namespace(self, package: str) -> str:
        del package
        return self.data_model_types_namespace

    def ros2_envelope_type(self, package: str) -> str:
        del package
        return "pyramid_ros2/PclEnvelope"

    def ros2_route_prefix(self, package: str) -> str:
        del package
        return "pyramid"

    def grpc_plugin_aggregator_file_prefix(self, package: str) -> str:
        del package
        return "pyramid_grpc_plugin_aggregator"

    def grpc_plugin_symbol_prefix(self, package: str) -> str:
        del package
        return "pyramid_grpc_plugin"

    def schema_id_for_type(self, full_type: str) -> str:
        return full_type.split(".")[-1]

    def protobuf_codec_namespace(self, package: str) -> str:
        return package.replace(".", "::") + "::protobuf_codec"

    def flatbuffers_service_group_key(self, package: str) -> Optional[str]:
        if ".services." not in f".{package}.":
            return None
        parts = [p for p in package.split(".") if p]
        if parts and parts[-1].lower() in ("provided", "consumed"):
            parts = parts[:-1]
        return ".".join(parts)

    def flatbuffers_service_namespace(self, package: str) -> str:
        parts = [p for p in package.split(".") if p]
        skip = {"pyramid", "components", "services", "data_model", "base"}
        meaningful = [p for p in parts if p.lower() not in skip]
        return ".".join(["pyramid", "services"] + [p.lower() for p in meaningful])

    def flatbuffers_service_file_prefix(self, package: str) -> str:
        return self.flatbuffers_service_namespace(package).replace(".", "_")


class GenericNamingPolicy:
    """Naming policy that derives names directly from proto package identity."""

    layout = "generic"

    # No domain base/common short-name mapping on the generic layout: names come
    # straight from proto package identity, so there are no compat literals.
    base_type_map: dict = {}

    def cpp_namespace_for_package(self, package: str) -> str:
        return package.replace(".", "::") if package else "proto"

    def cpp_type_namespace_for_package(self, package: str) -> str:
        return self.cpp_namespace_for_package(package)

    def _package_prefix(self, package: str) -> str:
        return package.replace(".", "_") if package else "proto"

    def type_header_for_package(self, package: str) -> str:
        return self._package_prefix(package) + "_types.hpp"

    def codec_header_for_package(self, package: str) -> str:
        return self._package_prefix(package) + "_codec.hpp"

    def codec_source_for_package(self, package: str) -> str:
        return self._package_prefix(package) + "_codec.cpp"

    def umbrella_types_header(self, proto_files: Iterable[ProtoFile]) -> Optional[str]:
        del proto_files
        return None

    def service_namespace(self, package: str) -> str:
        return self.cpp_namespace_for_package(package)

    def service_file_prefix(self, package: str) -> str:
        return self._package_prefix(package) + "_services"

    def service_transport_file_prefix(self, package: str) -> str:
        return self.service_file_prefix(package)

    def service_role(self, package: str) -> str:
        del package
        return "neutral"

    def ros2_support_file_prefix(self, package: str) -> str:
        return self._package_prefix(package) + "_ros2_transport_support"

    def ros2_support_namespace(self, package: str) -> str:
        return self.cpp_namespace_for_package(package) + "::transport::ros2"

    def ros2_message_package(self, package: str) -> str:
        return self._package_prefix(package) + "_msgs"

    def ros2_codec_header(self, package: str) -> str:
        return self._package_prefix(package) + "_ros2_codec.hpp"

    def ros2_codec_namespace(self, package: str) -> str:
        return self.cpp_namespace_for_package(package) + "::ros2_codec"

    def ros2_data_model_namespace(self, package: str) -> str:
        return self.cpp_type_namespace_for_package(package)

    def ros2_envelope_type(self, package: str) -> str:
        return self._package_prefix(package) + "_ros2/PclEnvelope"

    def ros2_route_prefix(self, package: str) -> str:
        return self._package_prefix(package)

    def grpc_plugin_aggregator_file_prefix(self, package: str) -> str:
        return self._package_prefix(package) + "_grpc_plugin_aggregator"

    def grpc_plugin_symbol_prefix(self, package: str) -> str:
        return self._package_prefix(package) + "_grpc_plugin"

    def schema_id_for_type(self, full_type: str) -> str:
        digest = hashlib.sha256(full_type.encode("utf-8")).hexdigest()[:16]
        return f"{full_type}@{digest}"

    def protobuf_codec_namespace(self, package: str) -> str:
        return self.cpp_namespace_for_package(package) + "::protobuf_codec"

    def flatbuffers_service_group_key(self, package: str) -> Optional[str]:
        return package if package else "proto"

    def flatbuffers_service_namespace(self, package: str) -> str:
        return package if package else "proto"

    def flatbuffers_service_file_prefix(self, package: str) -> str:
        return self.service_file_prefix(package)


def _fully_qualified_type(
    index: ProtoTypeIndex,
    type_name: str,
    current_package: str,
) -> str:
    if not type_name or type_name in _PROTO_SCALARS or type_name.startswith("google."):
        return ""
    if "." in type_name:
        return type_name
    current_fqn = f"{current_package}.{type_name}" if current_package else type_name
    if index.is_message_type(current_fqn) or index.is_enum_type(current_fqn):
        return current_fqn
    if index.is_message_type(type_name) or index.is_enum_type(type_name):
        return type_name
    return ""


def _message_by_fqn(index: ProtoTypeIndex, full_type: str) -> Optional[ProtoMessage]:
    return index.resolve_message(full_type)


def _field_type_closure(
    index: ProtoTypeIndex,
    root_type: str,
    current_package: str,
    seen: Set[str],
) -> None:
    full_type = _fully_qualified_type(index, root_type, current_package)
    if not full_type or full_type in seen:
        return
    seen.add(full_type)

    msg = _message_by_fqn(index, full_type)
    if msg is None:
        return
    msg_package = full_type.rsplit(".", 1)[0] if "." in full_type else ""
    for field in msg.all_fields():
        _field_type_closure(index, field.type, msg_package, seen)


def _service_key(package: str, service: ProtoService) -> str:
    return f"{package}.{service.name}" if package else service.name


def _service_role(package: str) -> str:
    return "provided" if package.lower().endswith(".provided") else "consumed"


def _project_from_service_package(package: str) -> str:
    parts = package.split(".")
    try:
        idx = parts.index("components")
    except ValueError:
        return "model"
    if idx + 1 < len(parts):
        return parts[idx + 1]
    return "model"


def _service_interface_snake(service_name: str) -> str:
    name = service_name
    if name.endswith("_Service"):
        name = name[:-len("_Service")]
    return camel_to_lower_snake(name)


def _service_wire_prefix(service_name: str) -> str:
    return _service_interface_snake(service_name)


def _rpc_wire_suffix(rpc_name: str) -> str:
    return camel_to_lower_snake(rpc_name)


def _topic_for_role(package: str, service_name: str, role: str) -> str:
    return ".".join((
        _project_from_service_package(package),
        _service_interface_snake(service_name),
        role,
    ))


def _topic_key(wire_name: str) -> str:
    return wire_name.replace(".", "_").replace("-", "_")


def _qos_reliability_floor(qos: Dict[str, object]) -> str:
    value = str(qos.get("reliability", "")).upper()
    if value == "RELIABLE":
        return "reliable"
    if value == "BEST_EFFORT":
        return "best_effort"
    return "unspecified"


def _fqn_for_rpc_type(type_name: str, package: str) -> str:
    if (not type_name or type_name in _PROTO_SCALARS
            or type_name.startswith("google.")):
        return type_name
    if "." in type_name:
        return type_name
    return f"{package}.{type_name}" if package else type_name


def _topic_from_rpc(
    pf: ProtoFile,
    service: ProtoService,
    rpc: ProtoRpc,
    wire_name: str,
    pattern: str,
) -> BindingTopic:
    payload_type = rpc.response_type if rpc.server_streaming else rpc.request_type
    direction = "publish" if pattern == "PUBLISH" else "subscribe"
    return BindingTopic(
        key=_topic_key(wire_name),
        wire_name=wire_name,
        full_type=_fqn_for_rpc_type(payload_type, pf.package),
        direction=direction,
        pattern=pattern,
        service_name=service.name,
        rpc_name=rpc.name,
        qos=dict(rpc.qos),
    )


def topics_for_proto_service(
    pf: ProtoFile,
    service: ProtoService,
) -> Tuple[Dict[str, BindingTopic], Dict[str, BindingTopic]]:
    """Return resolved (subscribe, publish) topics for one service."""

    subscribe: Dict[str, BindingTopic] = {}
    publish: Dict[str, BindingTopic] = {}

    def add(topic: BindingTopic) -> None:
        target = publish if topic.direction == "publish" else subscribe
        target.setdefault(topic.key, topic)

    option_rpcs = [
        rpc for rpc in service.rpcs
        if rpc.topic and rpc.pattern in ("PUBLISH", "SUBSCRIBE")
    ]
    if option_rpcs:
        for rpc in option_rpcs:
            add(_topic_from_rpc(pf, service, rpc, rpc.topic or "", rpc.pattern or ""))
        return subscribe, publish

    role = _service_role(pf.package)
    if service.port_kind == "information":
        rpc = service.rpcs[0]
        pattern = "PUBLISH" if role == "provided" else "SUBSCRIBE"
        add(_topic_from_rpc(
            pf, service, rpc,
            _topic_for_role(pf.package, service.name, "information"),
            pattern,
        ))
        return subscribe, publish

    if service.port_kind != "request":
        return subscribe, publish

    request_pattern = "SUBSCRIBE" if role == "provided" else "PUBLISH"
    requirement_pattern = "PUBLISH" if role == "provided" else "SUBSCRIBE"
    for rpc in service.rpcs:
        if rpc.name in ("Create", "Update", "Cancel"):
            add(_topic_from_rpc(
                pf, service, rpc,
                _topic_for_role(pf.package, service.name, "request"),
                request_pattern,
            ))
        elif rpc.name == "Read":
            add(_topic_from_rpc(
                pf, service, rpc,
                _topic_for_role(pf.package, service.name, "requirement"),
                requirement_pattern,
            ))
    return subscribe, publish


def _request_wrapper_type_name(service: ProtoService) -> str:
    """The `_Request` wrapper message name for a Request-shape service.

    Grammar-wide naming convention, not a guess from any one rpc: every
    Request-shape service in the tree (A-GRA example and all of `pim/test/`)
    names its wrapper `<ServiceName>_Request` (`MAAction_Service` ->
    `MAAction_Service_Request`, `SPRRequirement_Service` ->
    `SPRRequirement_Service_Request`, ...).
    """
    return f"{service.name}_Request"


def _oneof_variant_types(
    index: ProtoTypeIndex,
    wrapper_msg: ProtoMessage,
    wrapper_package: str,
) -> Dict[str, List[str]]:
    """Fully-qualified oneof variant field type -> field name(s) using it.

    More than one field name against a type means that type is an
    *ambiguous* variant match (an ill-formed wrapper -- two variants of the
    same payload type); callers must treat that as non-projectable rather
    than pick one of the field names arbitrarily.
    """
    variants: Dict[str, List[str]] = {}
    for oo in wrapper_msg.oneofs:
        for f in oo.fields:
            fqn = _fully_qualified_type(index, f.type, wrapper_package)
            if not fqn:
                continue
            variants.setdefault(fqn, []).append(f.name)
    return variants


def command_projectability_for_service(
    index: ProtoTypeIndex,
    pf: ProtoFile,
    service: ProtoService,
) -> Tuple[CommandProjectability, ...]:
    """Classify each Create/Update/Cancel rpc's D2 pub/sub projectability.

    Only meaningful for Request-shape services (`service.port_kind ==
    "request"`); other services yield an empty tuple. This is an additive
    classification computed alongside `topics_for_proto_service`'s topic
    derivation, not a replacement for it -- `topics_for_proto_service`'s
    first-rpc-wins `.request` topic mapping is untouched by this function's
    result.
    """
    if service.port_kind != "request":
        return ()

    wrapper_fqn = _fully_qualified_type(
        index, _request_wrapper_type_name(service), pf.package,
    )
    wrapper_msg = index.resolve_message(wrapper_fqn) if wrapper_fqn else None
    variants: Dict[str, List[str]] = {}
    if wrapper_msg is not None:
        wrapper_package = wrapper_fqn.rsplit(".", 1)[0] if "." in wrapper_fqn else ""
        variants = _oneof_variant_types(index, wrapper_msg, wrapper_package)

    results: List[CommandProjectability] = []
    for rpc in service.rpcs:
        if rpc.name not in ("Create", "Update", "Cancel"):
            continue

        request_fqn = (
            _fully_qualified_type(index, rpc.request_type, pf.package)
            or rpc.request_type
        )

        if wrapper_msg is None:
            results.append(CommandProjectability(
                service_name=service.name,
                rpc_name=rpc.name,
                request_type=request_fqn,
                wrapper_type=wrapper_fqn,
                projectable=False,
                reason="no_wrapper",
            ))
            continue

        if request_fqn == wrapper_fqn:
            results.append(CommandProjectability(
                service_name=service.name,
                rpc_name=rpc.name,
                request_type=request_fqn,
                wrapper_type=wrapper_fqn,
                projectable=True,
                reason="is_wrapper",
            ))
            continue

        matches = variants.get(request_fqn, [])
        if len(matches) == 1:
            results.append(CommandProjectability(
                service_name=service.name,
                rpc_name=rpc.name,
                request_type=request_fqn,
                wrapper_type=wrapper_fqn,
                projectable=True,
                reason="matches_variant",
                variant_field=matches[0],
            ))
        elif len(matches) > 1:
            results.append(CommandProjectability(
                service_name=service.name,
                rpc_name=rpc.name,
                request_type=request_fqn,
                wrapper_type=wrapper_fqn,
                projectable=False,
                reason="ambiguous_variant",
            ))
        else:
            results.append(CommandProjectability(
                service_name=service.name,
                rpc_name=rpc.name,
                request_type=request_fqn,
                wrapper_type=wrapper_fqn,
                projectable=False,
                reason="no_match",
            ))

    return tuple(results)


def command_projectability_for_file(
    index: ProtoTypeIndex,
    pf: ProtoFile,
) -> Tuple[CommandProjectability, ...]:
    """Aggregate `command_projectability_for_service` over one file's services."""
    result: List[CommandProjectability] = []
    for service in pf.services:
        result.extend(command_projectability_for_service(index, pf, service))
    return tuple(result)


def _leg_group_name(service_key: str, leg_name: str) -> str:
    """Deterministic PCL `exclusive` group name for one interaction leg.

    `f"{service_key}.{leg_name}_leg"` -- this must stay reproducible from
    `binding_manifest.json`'s `interactions` section alone (see
    `InteractionLeg.group_name`'s docstring): `contract_routing_manifest.py`
    and any future manifest generator recompute it from `service_key` +
    `leg["name"]` rather than trusting a stored value, so the convention
    lives here as the single source of truth.
    """
    return f"{service_key}.{leg_name}_leg"


def _find_topic_by_role_suffix(
    sub_topics: Dict[str, BindingTopic],
    pub_topics: Dict[str, BindingTopic],
    wire_name_suffix: str,
) -> Optional[BindingTopic]:
    """The one topic (subscribe or publish side) whose wire name ends with
    `wire_name_suffix` (`.request` / `.requirement` / `.information`) -- the
    same suffix convention `_topic_for_role` uses to derive these wire names
    and `contract_routing_manifest.py` already matches against today."""
    for topic in (*sub_topics.values(), *pub_topics.values()):
        if topic.wire_name.endswith(wire_name_suffix):
            return topic
    return None


def interaction_for_service(
    index: ProtoTypeIndex,
    pf: ProtoFile,
    service: ProtoService,
) -> Optional[Interaction]:
    """Build the Phase 1 `Interaction` for one service (D5), or `None` if the
    service is not a grammar-conforming Request/Information port, or is
    missing an rpc/topic a leg needs (e.g. a Request-shape service with no
    command rpcs and no `Read` rpc yields no legs at all).

    Purely a grouping over `topics_for_proto_service`'s topic derivation and
    `command_projectability_for_service`'s Phase 0 classification -- neither
    is recomputed nor overridden here.
    """
    if service.port_kind not in ("request", "information"):
        return None

    service_key = _service_key(pf.package, service)
    sub_topics, pub_topics = topics_for_proto_service(pf, service)
    projectability = {
        cp.rpc_name: cp
        for cp in command_projectability_for_service(index, pf, service)
    }

    def topic_endpoint(topic: BindingTopic) -> InteractionEndpoint:
        kind = "publisher" if topic.direction == "publish" else "subscriber"
        return InteractionEndpoint(endpoint_name=topic.wire_name, kind=kind)

    def rpc_endpoint(rpc: ProtoRpc) -> InteractionEndpoint:
        cp = projectability.get(rpc.name)
        return InteractionEndpoint(
            endpoint_name=_rpc_wire_name(service, rpc),
            kind=_service_endpoint_kind(pf.package, rpc),
            rpc_name=rpc.name,
            projectable=cp.projectable if cp else None,
            reason=cp.reason if cp else "",
        )

    if service.port_kind == "information":
        read_rpc = next((r for r in service.rpcs if r.name == "Read"), None)
        info_topic = _find_topic_by_role_suffix(sub_topics, pub_topics, ".information")
        if read_rpc is None or info_topic is None:
            return None
        leg = InteractionLeg(
            name="information",
            group_name=_leg_group_name(service_key, "information"),
            side_a=(rpc_endpoint(read_rpc),),
            side_b=(topic_endpoint(info_topic),),
        )
        return Interaction(
            service_key=service_key,
            service_name=service.name,
            port_kind="information",
            legs=(leg,),
        )

    # Request-shape: up to two independent legs (request, requirement).
    legs: List[InteractionLeg] = []

    command_rpcs = [
        rpc for rpc in service.rpcs if rpc.name in ("Create", "Update", "Cancel")
    ]
    request_topic = _find_topic_by_role_suffix(sub_topics, pub_topics, ".request")
    if command_rpcs and request_topic is not None:
        legs.append(InteractionLeg(
            name="request",
            group_name=_leg_group_name(service_key, "request"),
            side_a=tuple(rpc_endpoint(rpc) for rpc in command_rpcs),
            side_b=(topic_endpoint(request_topic),),
        ))

    read_rpc = next((r for r in service.rpcs if r.name == "Read"), None)
    requirement_topic = _find_topic_by_role_suffix(sub_topics, pub_topics, ".requirement")
    if read_rpc is not None and requirement_topic is not None:
        legs.append(InteractionLeg(
            name="requirement",
            group_name=_leg_group_name(service_key, "requirement"),
            side_a=(rpc_endpoint(read_rpc),),
            side_b=(topic_endpoint(requirement_topic),),
        ))

    if not legs:
        return None
    return Interaction(
        service_key=service_key,
        service_name=service.name,
        port_kind="request",
        legs=tuple(legs),
    )


def topics_for_proto_file(
    pf: ProtoFile,
) -> Tuple[Dict[str, BindingTopic], Dict[str, BindingTopic]]:
    subscribe: Dict[str, BindingTopic] = {}
    publish: Dict[str, BindingTopic] = {}
    for service in pf.services:
        svc_sub, svc_pub = topics_for_proto_service(pf, service)
        subscribe.update(svc_sub)
        publish.update(svc_pub)
    return subscribe, publish


def _schema_types(proto_files: Iterable[ProtoFile]) -> List[str]:
    result: List[str] = []
    for pf in proto_files:
        for msg in pf.messages:
            result.append(f"{pf.package}.{msg.name}" if pf.package else msg.name)
        for enum in pf.enums:
            result.append(f"{pf.package}.{enum.name}" if pf.package else enum.name)
    return result


def _rpc_wire_name(service: ProtoService, rpc: ProtoRpc) -> str:
    return f"{_service_wire_prefix(service.name)}.{_rpc_wire_suffix(rpc.name)}"


def _service_endpoint_kind(package: str, rpc: ProtoRpc) -> str:
    role = _service_role(package)
    if rpc.server_streaming:
        if role == "provided":
            return "stream_provided"
        if role == "consumed":
            return "stream_consumed"
    return role


def _service_endpoint_capability(kind: str) -> str:
    if kind in ("publisher", "subscriber"):
        return "PUBSUB"
    if kind in ("stream_provided", "stream_consumed"):
        return "RPC_STREAM"
    return "RPC_UNARY"


def _build_endpoint_requirements(
    service_modules: Iterable[ProtoFile],
) -> Tuple[EndpointRequirement, ...]:
    requirements: List[EndpointRequirement] = []
    seen: Set[Tuple[str, str, str]] = set()

    def add(req: EndpointRequirement) -> None:
        key = (req.endpoint_name, req.kind, req.source)
        if key in seen:
            return
        seen.add(key)
        requirements.append(req)

    for pf in service_modules:
        for service in pf.services:
            for rpc in service.rpcs:
                kind = _service_endpoint_kind(pf.package, rpc)
                add(EndpointRequirement(
                    endpoint_name=_rpc_wire_name(service, rpc),
                    kind=kind,
                    capability=_service_endpoint_capability(kind),
                    reliability=_qos_reliability_floor(rpc.qos),
                    source="service",
                    service_name=service.name,
                    rpc_name=rpc.name,
                    qos=dict(rpc.qos),
                ))

            sub_topics, pub_topics = topics_for_proto_service(pf, service)
            for topic in sub_topics.values():
                add(EndpointRequirement(
                    endpoint_name=topic.wire_name,
                    kind="subscriber",
                    capability="PUBSUB",
                    reliability=topic.reliability_floor,
                    source="topic",
                    service_name=service.name,
                    rpc_name=topic.rpc_name,
                    topic_key=topic.key,
                    topic_wire_name=topic.wire_name,
                    qos=dict(topic.qos),
                ))
            for topic in pub_topics.values():
                add(EndpointRequirement(
                    endpoint_name=topic.wire_name,
                    kind="publisher",
                    capability="PUBSUB",
                    reliability=topic.reliability_floor,
                    source="topic",
                    service_name=service.name,
                    rpc_name=topic.rpc_name,
                    topic_key=topic.key,
                    topic_wire_name=topic.wire_name,
                    qos=dict(topic.qos),
                ))

    return tuple(sorted(
        requirements,
        key=lambda r: (r.source, r.endpoint_name, r.kind, r.service_name, r.rpc_name),
    ))


def _build_service_closures(
    proto_files: List[ProtoFile],
) -> Dict[str, ServiceTypeClosure]:
    index = ProtoTypeIndex(proto_files)
    closures: Dict[str, ServiceTypeClosure] = {}

    for pf in proto_files:
        for service in pf.services:
            roots: List[str] = []
            closure: Set[str] = set()
            for rpc in service.rpcs:
                for type_name in (rpc.request_type, rpc.response_type):
                    fqn = _fully_qualified_type(index, type_name, pf.package)
                    if not fqn:
                        continue
                    roots.append(fqn)
                    _field_type_closure(index, type_name, pf.package, closure)
            closures[_service_key(pf.package, service)] = ServiceTypeClosure(
                service_name=service.name,
                package=pf.package,
                root_types=tuple(sorted(set(roots))),
                types=tuple(sorted(closure)),
            )

    return closures


def _build_command_projectability(
    service_modules: List[ProtoFile],
    proto_files: List[ProtoFile],
) -> Dict[str, Tuple[CommandProjectability, ...]]:
    index = ProtoTypeIndex(proto_files)
    result: Dict[str, Tuple[CommandProjectability, ...]] = {}
    for pf in service_modules:
        for service in pf.services:
            projectability = command_projectability_for_service(index, pf, service)
            if projectability:
                result[_service_key(pf.package, service)] = projectability
    return result


def _build_interactions(
    service_modules: List[ProtoFile],
    proto_files: List[ProtoFile],
) -> Dict[str, Interaction]:
    index = ProtoTypeIndex(proto_files)
    result: Dict[str, Interaction] = {}
    for pf in service_modules:
        for service in pf.services:
            interaction = interaction_for_service(index, pf, service)
            if interaction is not None:
                result[_service_key(pf.package, service)] = interaction
    return result


def naming_policy_for_layout(layout: str) -> NamingPolicy:
    if layout == "pyramid":
        return PyramidCompatNamingPolicy()
    if layout == "generic":
        return GenericNamingPolicy()
    raise ValueError(f"unsupported contract layout: {layout}")


def build_contract(proto_files: List[ProtoFile], layout: str) -> BindingContract:
    """Build a neutral contract for parsed proto files."""

    policy = naming_policy_for_layout(layout)
    type_modules = [pf for pf in proto_files if pf.messages or pf.enums]
    service_modules = [pf for pf in proto_files if pf.services]
    wrapper_modules = [
        pf for pf in service_modules
        if pf.messages or pf.enums
    ]
    schema_ids = {
        full_type: policy.schema_id_for_type(full_type)
        for full_type in _schema_types(proto_files)
    }
    service_topics = {}
    for pf in service_modules:
        for service in pf.services:
            sub_topics, pub_topics = topics_for_proto_service(pf, service)
            topics = tuple(sorted(
                [*sub_topics.values(), *pub_topics.values()],
                key=lambda topic: (topic.wire_name, topic.direction),
            ))
            service_topics[_service_key(pf.package, service)] = topics
    return BindingContract(
        proto_files=list(proto_files),
        type_modules=type_modules,
        service_modules=service_modules,
        wrapper_modules=wrapper_modules,
        service_type_closures=_build_service_closures(list(proto_files)),
        service_topics=service_topics,
        endpoint_requirements=_build_endpoint_requirements(service_modules),
        schema_ids=schema_ids,
        naming_policy=policy,
        command_projectability=_build_command_projectability(
            service_modules, list(proto_files),
        ),
        interactions=_build_interactions(service_modules, list(proto_files)),
    )


# -- Per-run topic-spec resolution ---------------------------------------------

def is_pyramid_compat_service_package(package: str) -> bool:
    """pyramid.components.<name>.services.{provided,consumed} package shape."""
    parts = [p for p in package.split('.') if p]
    return (
        len(parts) == 5
        and parts[0] == 'pyramid'
        and parts[1] == 'components'
        and parts[3] == 'services'
        and parts[4] in ('provided', 'consumed')
    )


class TopicSpecResolver:
    """Topic-key -> spec resolution for one generation run.

    Owns the contract-derived topic specs recorded while proto files are
    scanned (previously a module-global dict in each codegen), falling back
    to the legacy standard-topics side table for catalog topics that predate
    contract-derived ones.  Construct one per generation run and pass it to
    the generator classes so no state leaks between runs.
    """

    def __init__(self, metadata=None):
        # metadata: optional standard_topics.TopicMetadata overriding the
        # module default for the legacy fallback.
        self._contract_specs: Dict[str, BindingTopic] = {}
        self._metadata = metadata

    def spec(self, key: str):
        """Resolve a topic key: contract-derived first, then legacy catalog."""
        found = self._contract_specs.get(key)
        if found is not None:
            return found
        return standard_topics.topic_spec(key, self._metadata)

    def topics_for_proto(self, parsed: ProtoFile,
                         is_provided: bool) -> Tuple[Dict[str, str], Dict[str, str]]:
        """Return (sub_topics, pub_topics) wire-name maps for a service proto.

        Prefers contract-derived topics (recording their specs for later
        spec() lookups); falls back to the legacy standard-topics catalog for
        PYRAMID-compat service packages.
        """
        sub_specs, pub_specs = topics_for_proto_file(parsed)
        if sub_specs or pub_specs:
            self._contract_specs.update(sub_specs)
            self._contract_specs.update(pub_specs)
            return (
                {key: spec.wire_name for key, spec in sub_specs.items()},
                {key: spec.wire_name for key, spec in pub_specs.items()},
            )
        if not is_pyramid_compat_service_package(parsed.package):
            return {}, {}
        return standard_topics.topics_for_service(
            parsed.package, is_provided, self._metadata)
