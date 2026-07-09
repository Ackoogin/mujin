#!/usr/bin/env python3
"""Interaction facade emitter for CppServiceGenerator (Phase 2, consumer side).

Emits `RequestPortClient` / `InformationPortSink` classes into the same
generated `_components.hpp` file `components_gen.py` writes, composed from
the already-emitted `ConsumedService` primitives (its `<op>Async()`/
`<op>Streaming()`/`subscribe<Topic>()`/`publish<Topic>()` methods) rather
than re-emitting the `UnaryState<T>`/`StreamPushState<T>` machinery.

See doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md, Phase 2.
Strictly additive: nothing here changes a byte of ProvidedHandler/
ProvidedService/ConsumedService/StreamWriter/StreamHandle's already-emitted
text -- it is appended after it, in the same file, gated by
`interaction_for_service` (Phase 1, binding_contract.py).
"""

from typing import Dict, List, Optional, Tuple

from proto_parser import (
    ProtoFile,
    ProtoRpc,
    ProtoService,
    ProtoTypeIndex,
    lc_first as _lc_first,
    snake_to_pascal as _snake_to_pascal,
)
from binding_contract import (
    CommandProjectability,
    Interaction,
    command_projectability_for_service,
    interaction_for_service,
)
from .naming import (
    _SEP,
    _cpp_req_type,
    _cpp_rsp_type,
    _rpc_enum_value,
    _rpc_handler_name,
    _rpc_send_stream_frame_func,
    _rpc_service_const,
    _rpc_stream_handler_name,
    _rpc_symbol_base,
    _service_cpp_prefix,
)


_COMMAND_RPC_NAMES = ('Create', 'Update', 'Cancel')


def _fqn(index: ProtoTypeIndex, type_name: str, current_package: str) -> str:
    """Minimal local re-implementation of binding_contract._fully_qualified_type
    (kept local rather than importing a private helper across modules)."""
    if not type_name or type_name.startswith('google.'):
        return ''
    if '.' in type_name:
        return type_name
    fqn = f'{current_package}.{type_name}' if current_package else type_name
    if index.resolve_message(fqn) is not None or index.resolve_enum(fqn) is not None:
        return fqn
    if index.resolve_message(type_name) is not None or index.resolve_enum(type_name) is not None:
        return type_name
    return ''


def _requirement_correlation_field(
    index: ProtoTypeIndex,
    pf: ProtoFile,
    read_rpc: ProtoRpc,
) -> Optional[str]:
    """Best-effort discovery of a top-level 'id' field on the Read stream's
    frame type (D4's client-side query.id filter under pub/sub).

    Only handles the single-oneof-single-variant wrapper shape (every
    Requirement wrapper seen in the tree at Phase 2 authoring time -- the
    A-GRA example's MAAction_Service_Requirement, one 'ma_action_status'
    variant of type Requirement): the variant payload type must expose an
    'id' field, directly or one level of Entity-style inlining via a 'base'
    field (matching types_gen.py's own _inline_base_fields, which is why the
    generated field is always literally named '.id' either way -- no nested
    '.base.id' access is ever needed in the emitted C++).

    Returns the oneof variant's field name (the accessor on the frame
    struct) when a correlation id is resolvable, else None -- callers must
    then treat query.id as accept-all under pub/sub and say so in a
    generated comment (D4: a documented gap, not a silently-wrong filter).
    """
    frame_fqn = _fqn(index, read_rpc.response_type, pf.package)
    frame_msg = index.resolve_message(frame_fqn) if frame_fqn else None
    if frame_msg is None:
        return None
    if len(frame_msg.oneofs) != 1 or len(frame_msg.oneofs[0].fields) != 1:
        return None
    variant_field = frame_msg.oneofs[0].fields[0]
    variant_fqn = _fqn(index, variant_field.type, pf.package)
    variant_msg = index.resolve_message(variant_fqn) if variant_fqn else None
    if variant_msg is None:
        return None
    if any(fld.name == 'id' for fld in variant_msg.fields):
        return variant_field.name
    variant_pkg = variant_fqn.rsplit('.', 1)[0] if '.' in variant_fqn else ''
    for fld in variant_msg.fields:
        if fld.name == 'base' and not fld.is_repeated:
            base_fqn = _fqn(index, fld.type, variant_pkg)
            base_msg = index.resolve_message(base_fqn) if base_fqn else None
            if base_msg is not None and any(bf.name == 'id' for bf in base_msg.fields):
                return variant_field.name
    return None


def _direct_id_accessor(index: ProtoTypeIndex, type_fqn: str) -> Optional[str]:
    """The C++ accessor -- `''` (the value itself) or `'.id'` -- for a
    correlation id directly available on a non-optional value of `type_fqn`,
    with no oneof-unwrap: the `Identifier` scalar alias (generated as a
    plain `std::string`, per naming.py's `_FORCED_ALIASES`/
    `_UNIT_FIELD_NAMES` convention), a direct `id` field, or one level of
    `Entity`-style `base` inlining (types_gen.py flattens this, so no
    literal '.base.id' path exists in the generated C++ -- see
    `_requirement_correlation_field`'s docstring for the same fact). `None`
    if none of these shapes match.
    """
    msg = index.resolve_message(type_fqn) if type_fqn else None
    if msg is None:
        return None
    if msg.name == 'Identifier':
        return ''
    if any(f.name == 'id' for f in msg.fields):
        return '.id'
    msg_pkg = type_fqn.rsplit('.', 1)[0] if '.' in type_fqn else ''
    for f in msg.fields:
        if f.name == 'base' and not f.is_repeated:
            base_fqn = _fqn(index, f.type, msg_pkg)
            base_msg = index.resolve_message(base_fqn) if base_fqn else None
            if base_msg is not None and any(bf.name == 'id' for bf in base_msg.fields):
                return '.id'
    return None


def _command_correlation_accessor(
    index: ProtoTypeIndex,
    pf: ProtoFile,
    type_fqn: str,
) -> Optional[Tuple[Optional[str], str]]:
    """D6/D4 (Phase 3): how to extract a correlation id from an
    already-dereferenced command variant value of `type_fqn`, or `None` if
    unresolvable (documented gap, not a silently-wrong lookup -- same
    "structural discovery with a skip fallback" philosophy as
    `_requirement_correlation_field`).

    Returns `(needs_check, accessor)`:
      - `needs_check is None`: `accessor` is directly appendable to the
        dereferenced value (`_direct_id_accessor`'s two shapes -- Create's
        `MA_Action` via `base.id` inlining, Cancel's `Identifier` alias).
      - `needs_check` names a `tl::optional`-wrapped field the caller must
        `.has_value()`-check before dereferencing with `->` and applying
        `accessor` (one level of single-oneof/single-variant unwrap, e.g.
        Update's `MAAction_Service_Requirement` wrapping a `Requirement` via
        its `ma_action_status` field -- structurally the same shape
        `_requirement_correlation_field` already handles for the Read frame
        type, just returned here as an explicit two-step accessor instead
        of leaving the intermediate optional-check to a caller convention).
    """
    direct = _direct_id_accessor(index, type_fqn)
    if direct is not None:
        return None, direct
    msg = index.resolve_message(type_fqn) if type_fqn else None
    if msg is None:
        return None
    msg_pkg = type_fqn.rsplit('.', 1)[0] if '.' in type_fqn else ''
    if len(msg.oneofs) == 1 and len(msg.oneofs[0].fields) == 1:
        variant = msg.oneofs[0].fields[0]
        variant_fqn = _fqn(index, variant.type, msg_pkg)
        inner = _direct_id_accessor(index, variant_fqn)
        if inner is not None:
            return variant.name, inner
    return None


def _topic_key_for_wire_name(all_topics: Dict[str, str], wire_name: str) -> Optional[str]:
    """`all_topics` here is the {key: wire_name} map `_write_components_header`
    already computed via `TopicSpecResolver.topics_for_proto` -- not a map of
    `BindingTopic` objects (those live behind `self._topics.spec(key)`)."""
    for key, mapped_wire_name in all_topics.items():
        if mapped_wire_name == wire_name:
            return key
    return None


class InteractionFacadeEmitterMixin:
    """RequestPortClient / InformationPortSink facade emission (Phase 2)."""

    def _interaction_facade_type_index(self, parsed: ProtoFile) -> ProtoTypeIndex:
        """A ProtoTypeIndex wide enough to resolve cross-file requirement
        payload shapes (e.g. pyramid.data_model.common.Requirement's
        Entity-inlined 'id'), not just this file's own wrapper messages
        (Phase 1's interaction_for_service only ever needed the latter).
        Cached per generator instance -- proto_input is fixed for its
        lifetime, so the discovered tree does not change across files.
        """
        cached = getattr(self, '_interaction_facade_index_cache', None)
        if cached is not None:
            return cached
        all_files = self._discover_all_proto_files()
        index = ProtoTypeIndex(all_files) if all_files else ProtoTypeIndex([parsed])
        self._interaction_facade_index_cache = index
        return index

    def _write_interaction_facade(self, f, full_ns: str, parsed: ProtoFile,
                                   all_topics: Dict[str, str],
                                   is_provided: bool,
                                   duplicate_rpc_names: set) -> None:
        del full_ns, is_provided
        # No role gating needed: ComponentsFacadeEmitterMixin's ConsumedService
        # topic emission (subscribe<Topic>()/publish<Topic>()/add<Topic>Publisher())
        # already iterates the *merged* sub+pub topics dict unconditionally for
        # every topic key, in both 'provided' and 'consumed' role files (see
        # _write_components_header's two `for key in all_topics:` loops) --
        # ConsumedService in a 'provided'-role file already has both
        # subscribeAgraMaActionRequest() and publishAgraMaActionRequest(), not
        # just the direction its own pyramid_op pattern stamps. RequestPortClient/
        # InformationPortSink compose only those already-symmetric primitives, so
        # they compile and behave identically regardless of which role file they
        # are emitted into -- consistent with ProvidedHandler/ProvidedService/
        # ConsumedService's own existing unconditional per-role emission.
        index = self._interaction_facade_type_index(parsed)
        plans: List[Tuple[ProtoService, Interaction]] = []
        for service in parsed.services:
            if service.port_kind not in ('request', 'information'):
                continue
            interaction = interaction_for_service(index, parsed, service)
            if interaction is None:
                continue
            leg_names = {leg.name for leg in interaction.legs}
            if service.port_kind == 'request':
                if not {'request', 'requirement'} <= leg_names:
                    continue
            else:
                if 'information' not in leg_names:
                    continue
            plans.append((service, interaction))

        if not plans:
            return

        self._write_interaction_facade_preamble(f)
        for service, interaction in plans:
            if service.port_kind == 'request':
                self._write_request_port_client(
                    f, parsed, service, interaction, all_topics, index,
                    duplicate_rpc_names)
                self._write_request_port_provider(
                    f, parsed, service, interaction, all_topics, index,
                    duplicate_rpc_names)
            else:
                self._write_information_port_sink(
                    f, service, interaction, all_topics, duplicate_rpc_names)
                self._write_information_port_source(
                    f, service, interaction, all_topics, duplicate_rpc_names)

    # -- Shared preamble: InteractionBinding + SubscriptionHandle -----------

    def _write_interaction_facade_preamble(self, f) -> None:
        f.write(_SEP + '\n')
        f.write('// Interaction facade (rpc_pubsub_interchangeability_plan.md, Phase 2).\n')
        f.write('//\n')
        f.write('// RequestPortClient / InformationPortSink below let component code call\n')
        f.write('// submit()/transitions()/subscribe() without knowing whether the leg is\n')
        f.write('// realized as RPC or pub/sub -- that choice is\n')
        f.write('// configureInteractionBinding()\'s job (plan D1). Composed from the\n')
        f.write('// ConsumedService primitives above, not a reimplementation of them.\n')
        f.write(_SEP + '\n\n')

        f.write('enum class InteractionBinding {\n')
        f.write('    kRpc,\n')
        f.write('    kPubsub,\n')
        f.write('};\n\n')

        f.write('inline tl::optional<InteractionBinding> parseInteractionBindingValue(\n')
        f.write('        std::string_view value) {\n')
        f.write('    if (value.empty() || value == "rpc") return InteractionBinding::kRpc;\n')
        f.write('    if (value == "pubsub") return InteractionBinding::kPubsub;\n')
        f.write('    return tl::nullopt;\n')
        f.write('}\n\n')

        f.write('/// \\brief Move-only cancel handle unifying an RPC StreamHandle and a\n')
        f.write('///        pub/sub topic subscription behind one type -- transitions()/\n')
        f.write('///        subscribe() below must return the same shape under either\n')
        f.write('///        realization (plan D1) even though only ConsumedService can\n')
        f.write('///        construct a live StreamHandle (its cancel_fn_ constructor is\n')
        f.write('///        private to it, by design -- see the Phase 2 ledger entry for why\n')
        f.write('///        this wrapper exists instead of returning StreamHandle itself).\n')
        f.write('///\n')
        f.write('/// Under the RPC realization, cancel() *is* StreamHandle::cancel(),\n')
        f.write('/// unmodified -- this class only composes it. Under pub/sub, cancel()\n')
        f.write('/// suppresses further delivery to the caller\'s callback; PCL has no\n')
        f.write('/// per-subscriber unregister primitive today, so the underlying topic\n')
        f.write('/// port itself is not torn down -- documented, not papered over (D4).\n')
        f.write('class SubscriptionHandle {\n')
        f.write('public:\n')
        f.write('    SubscriptionHandle() = default;\n')
        f.write('    explicit SubscriptionHandle(StreamHandle rpc_handle)\n')
        f.write('        : rpc_handle_(std::move(rpc_handle)) {}\n')
        f.write('    explicit SubscriptionHandle(std::shared_ptr<bool> pubsub_active)\n')
        f.write('        : pubsub_active_(std::move(pubsub_active)) {}\n\n')
        f.write('    SubscriptionHandle(SubscriptionHandle&&) noexcept = default;\n')
        f.write('    SubscriptionHandle& operator=(SubscriptionHandle&&) noexcept = default;\n')
        f.write('    SubscriptionHandle(const SubscriptionHandle&) = delete;\n')
        f.write('    SubscriptionHandle& operator=(const SubscriptionHandle&) = delete;\n\n')
        f.write('    bool valid() const {\n')
        f.write('        return rpc_handle_.valid() || (pubsub_active_ && *pubsub_active_);\n')
        f.write('    }\n')
        f.write('    explicit operator bool() const { return valid(); }\n\n')
        f.write('    /// \\brief Cancel. Idempotent; subsequent calls no-op.\n')
        f.write('    void cancel() {\n')
        f.write('        if (rpc_handle_.valid()) {\n')
        f.write('            rpc_handle_.cancel();\n')
        f.write('            return;\n')
        f.write('        }\n')
        f.write('        if (pubsub_active_) *pubsub_active_ = false;\n')
        f.write('    }\n\n')
        f.write('private:\n')
        f.write('    StreamHandle          rpc_handle_;\n')
        f.write('    std::shared_ptr<bool> pubsub_active_;\n')
        f.write('};\n\n')

    # -- Small helper shared by both generated classes' bodies --------------

    def _write_config_value_helper(self, f) -> None:
        f.write('    static std::string configValue(std::string_view json,\n')
        f.write('                                   std::string_view key) {\n')
        f.write('        const std::string quoted_key = "\\"" + std::string(key) + "\\"";\n')
        f.write('        const auto key_pos = json.find(quoted_key);\n')
        f.write('        if (key_pos == std::string_view::npos) return {};\n')
        f.write("        const auto colon = json.find(':', key_pos + quoted_key.size());\n")
        f.write('        if (colon == std::string_view::npos) return {};\n')
        f.write("        const auto first_quote = json.find('\"', colon + 1u);\n")
        f.write('        if (first_quote == std::string_view::npos) return {};\n')
        f.write("        const auto second_quote = json.find('\"', first_quote + 1u);\n")
        f.write('        if (second_quote == std::string_view::npos) return {};\n')
        f.write('        return std::string(json.substr(first_quote + 1u,\n')
        f.write('                                       second_quote - first_quote - 1u));\n')
        f.write('    }\n\n')

    # -- RequestPortClient ----------------------------------------------------

    def _write_request_port_client(self, f, parsed: ProtoFile, service: ProtoService,
                                    interaction: Interaction,
                                    all_topics: Dict[str, str],
                                    index: ProtoTypeIndex,
                                    duplicate_rpc_names: set) -> None:
        prefix = _service_cpp_prefix(service.name)
        class_name = f'{prefix}RequestPortClient'

        command_rpcs = [rpc for rpc in service.rpcs if rpc.name in _COMMAND_RPC_NAMES]
        read_rpc = next(r for r in service.rpcs if r.name == 'Read')
        projectability: Dict[str, CommandProjectability] = {
            cp.rpc_name: cp
            for cp in command_projectability_for_service(index, parsed, service)
        }
        wrapper_cpp_type = ''
        if projectability:
            wrapper_cpp_type = next(iter(projectability.values())).wrapper_type.split('.')[-1]

        request_leg = next(leg for leg in interaction.legs if leg.name == 'request')
        requirement_leg = next(leg for leg in interaction.legs if leg.name == 'requirement')
        request_topic_key = _topic_key_for_wire_name(
            all_topics, request_leg.side_b[0].endpoint_name)
        requirement_topic_key = _topic_key_for_wire_name(
            all_topics, requirement_leg.side_b[0].endpoint_name)
        request_pascal = _snake_to_pascal(request_topic_key) if request_topic_key else ''
        requirement_pascal = _snake_to_pascal(requirement_topic_key) if requirement_topic_key else ''

        read_req_t = _cpp_req_type(read_rpc)
        read_frame_t = _cpp_rsp_type(read_rpc)[len('std::vector<'):-1]
        read_async_base = _rpc_symbol_base(service.name, read_rpc, duplicate_rpc_names)
        read_streaming_name = _lc_first(read_async_base) + 'Streaming'

        correlation_field = _requirement_correlation_field(index, parsed, read_rpc)

        ack_t = _cpp_rsp_type(command_rpcs[0]) if command_rpcs else 'Ack'

        f.write(_SEP + '\n')
        f.write(f'// {class_name} -- consumer facade for the \'{service.name}\' request port.\n')
        f.write('// One submit() overload per command; the leg realization (rpc/pubsub) is\n')
        f.write('// selected at runtime by configureInteractionBinding(), independently for\n')
        f.write('// the request leg (submit) and the requirement leg (transitions).\n')
        f.write(_SEP + '\n\n')

        f.write(f'class {class_name} {{\n')
        f.write('public:\n')
        f.write(f'    {class_name}(pcl::Component& host,\n')
        f.write('                    pcl::Executor& executor,\n')
        f.write('                    std::string content_type = kJsonContentType)\n')
        f.write('        : consumed_(host, executor, std::move(content_type)) {}\n\n')

        f.write(f'    {class_name}(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}& operator=(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}({class_name}&&) = delete;\n')
        f.write(f'    {class_name}& operator=({class_name}&&) = delete;\n\n')

        f.write('    /// \\brief Access the underlying ConsumedService for transport routing\n')
        f.write('    ///        (routeAllLocal()/routeAllRemote()/configureTransport()/\n')
        f.write('    ///        configurePubSubTransport() -- configureInteractionBinding()\n')
        f.write('    ///        below only selects the *mechanism*, not the peer).\n')
        f.write('    ConsumedService& consumedService() { return consumed_; }\n\n')

        f.write('    /// \\brief Bind the pub/sub-realization ports. Call once from the\n')
        f.write('    ///        owning component\'s on_configure() -- PCL only allows adding\n')
        f.write('    ///        publisher/subscriber ports while a container is configuring\n')
        f.write('    ///        (pcl_container_add_publisher()/add_subscriber()), so this\n')
        f.write('    ///        cannot happen lazily inside submit()/transitions() the way\n')
        f.write('    ///        the RPC realization\'s executor-level invoke can. Harmless to\n')
        f.write('    ///        call even if configureInteractionBinding() later selects\n')
        f.write('    ///        "rpc" for both legs -- the bound ports then simply sit idle.\n')
        f.write('    pcl_status_t bind() {\n')
        f.write(f'        if (auto rc = consumed_.add{request_pascal}Publisher(); rc != PCL_OK) {{\n')
        f.write('            return rc;\n')
        f.write('        }\n')
        f.write(f'        auto* port = consumed_.subscribe{requirement_pascal}(\n')
        f.write(f'            [this](const {read_frame_t}& frame) {{ dispatchPubsubTransition(frame); }});\n')
        f.write('        return port ? PCL_OK : PCL_ERR_NOMEM;\n')
        f.write('    }\n\n')

        f.write('    /// \\brief Select the realization per leg (plan D1). Config shapes:\n')
        f.write('    ///        {"binding":"rpc"}, {"binding":"pubsub"}, or the per-leg\n')
        f.write('    ///        override {"request_leg":"rpc","requirement_leg":"pubsub"}.\n')
        f.write('    ///        An unset leg falls back to "binding"; an unset "binding" too\n')
        f.write('    ///        defaults to "rpc" for both legs (the conservative default\n')
        f.write('    ///        this plan\'s Phase 1 manifest tooling also uses -- see §7).\n')
        f.write('    pcl_status_t configureInteractionBinding(std::string_view config_json) {\n')
        f.write('        const auto binding_value = configValue(config_json, "binding");\n')
        f.write('        auto request_value = configValue(config_json, "request_leg");\n')
        f.write('        auto requirement_value = configValue(config_json, "requirement_leg");\n')
        f.write('        if (request_value.empty()) request_value = binding_value;\n')
        f.write('        if (requirement_value.empty()) requirement_value = binding_value;\n')
        f.write('        const auto parsed_request = parseInteractionBindingValue(request_value);\n')
        f.write('        const auto parsed_requirement =\n')
        f.write('            parseInteractionBindingValue(requirement_value);\n')
        f.write('        if (!parsed_request || !parsed_requirement) return PCL_ERR_INVALID;\n')
        f.write('        request_binding_ = *parsed_request;\n')
        f.write('        requirement_binding_ = *parsed_requirement;\n')
        f.write('        return PCL_OK;\n')
        f.write('    }\n\n')

        f.write('    /// \\brief D3: transfer-accepted result. accepted is true when the\n')
        f.write('    ///        transfer succeeded under whichever realization is active --\n')
        f.write('    ///        RPC: the remote call completed without a transport error;\n')
        f.write('    ///        pub/sub: publish() returned PCL_OK. This is *not* an\n')
        f.write('    ///        acceptance/processing outcome -- read the correlated\n')
        f.write('    ///        requirement transition (Achievement.acceptance) for that.\n')
        f.write('    struct SubmitResult {\n')
        f.write('        bool accepted = false;\n')
        f.write('        /// \\brief PCL_OK on success under either realization; the\n')
        f.write('        ///        underlying transport/publish status otherwise, or\n')
        f.write('        ///        PCL_ERR_STATE when the command is not projectable to\n')
        f.write('        ///        pub/sub and the request leg is pubsub-realized (D2).\n')
        f.write('        pcl_status_t status = PCL_ERR_STATE;\n')
        f.write('        /// \\brief Populated only when the RPC realization ran; empty\n')
        f.write('        ///        under pub/sub (D3) -- never synthesized either way.\n')
        f.write(f'        tl::optional<{ack_t}> ack;\n\n')
        f.write(f'        const tl::optional<{ack_t}>& remoteAck() const {{ return ack; }}\n')
        f.write('    };\n\n')

        for rpc in command_rpcs:
            self._write_submit_overload(
                f, service, rpc, projectability.get(rpc.name), wrapper_cpp_type,
                request_pascal, duplicate_rpc_names)

        f.write('    /// \\brief D4: RPC realization is a server-streaming Read; pub/sub\n')
        f.write(f'    ///        realization subscribes {requirement_leg.side_b[0].endpoint_name}\n')
        f.write('    ///        and filters client-side by query.id (empty = accept all).\n')
        f.write('    ///        one_shot is honoured best-effort under pub/sub (ends after\n')
        f.write('    ///        the first matching frame and fires on_end(PCL_OK), same as a\n')
        f.write('    ///        one_shot RPC Read stream the provider ends after its first\n')
        f.write('    ///        match -- "what is observable now", not a guaranteed single\n')
        f.write('    ///        delivery). Late-join is out of scope for\n')
        f.write('    ///        Phase 2 (plan D4): this only observes transitions published\n')
        f.write('    ///        after the subscription starts, it does not replay current\n')
        f.write('    ///        state for ids already in flight -- that needs the Phase 3\n')
        f.write('    ///        provider-side snapshot store.\n')
        f.write('    SubscriptionHandle\n')
        f.write(f'    transitions(const {read_req_t}& query,\n')
        f.write(f'                std::function<void(const {read_frame_t}&)> on_transition,\n')
        f.write('                std::function<void(pcl_status_t)> on_end = {}) {\n')
        f.write('        if (requirement_binding_ == InteractionBinding::kRpc) {\n')
        f.write(f'            return SubscriptionHandle(consumed_.{read_streaming_name}(\n')
        f.write('                query, std::move(on_transition), std::move(on_end)));\n')
        f.write('        }\n')
        f.write('        // Pub/sub realization: register into the dispatch list bind()\n')
        f.write('        // populated a single shared subscription for -- see bind()\'s\n')
        f.write('        // doc comment for why this cannot subscribe a fresh port here.\n')
        f.write('        auto active = std::make_shared<bool>(true);\n')
        f.write('        auto want_ids = std::make_shared<std::vector<std::string>>(\n')
        f.write('            query.id.begin(), query.id.end());\n')
        f.write('        const bool one_shot = query.one_shot.has_value() && *query.one_shot;\n')
        f.write(f'        auto on_frame = std::make_shared<std::function<void(const {read_frame_t}&)>>(\n')
        f.write('            std::move(on_transition));\n')
        f.write('        auto on_end_shared = std::make_shared<std::function<void(pcl_status_t)>>(\n')
        f.write('            std::move(on_end));\n')
        f.write('        pubsub_transitions_.push_back(\n')
        f.write('            PubsubTransition{active, want_ids, one_shot, on_frame, on_end_shared});\n')
        f.write('        return SubscriptionHandle(active);\n')
        f.write('    }\n\n')

        f.write('private:\n')
        self._write_config_value_helper(f)
        f.write('    struct PubsubTransition {\n')
        f.write('        std::shared_ptr<bool>                     active;\n')
        f.write('        std::shared_ptr<std::vector<std::string>> want_ids;\n')
        f.write('        bool                                      one_shot = false;\n')
        f.write(f'        std::shared_ptr<std::function<void(const {read_frame_t}&)>> on_frame;\n')
        f.write('        std::shared_ptr<std::function<void(pcl_status_t)>>          on_end;\n')
        f.write('    };\n\n')

        f.write(f'    void dispatchPubsubTransition(const {read_frame_t}& frame) {{\n')
        f.write('        for (auto it = pubsub_transitions_.begin();\n')
        f.write('             it != pubsub_transitions_.end();) {\n')
        f.write('            if (!*it->active) { it = pubsub_transitions_.erase(it); continue; }\n')
        f.write('            bool matched = it->want_ids->empty();\n')
        if correlation_field is not None:
            f.write('            if (!matched')
            f.write(f' && frame.{correlation_field}.has_value()) {{\n')
            f.write('                for (const auto& wanted : *it->want_ids) {\n')
            f.write(f'                    if (wanted == frame.{correlation_field}->id) {{ matched = true; break; }}\n')
            f.write('                }\n')
            f.write('            }\n')
        else:
            f.write('            // This port\'s requirement frame type does not expose a\n')
            f.write('            // structurally-resolvable correlation id at generation time\n')
            f.write('            // (D4): query.id is accepted but not applied as a filter\n')
            f.write('            // under the pub/sub realization here (matched stays\n')
            f.write('            // "accept all"). Use the RPC realization (Read) if per-id\n')
            f.write('            // filtering is required for this port.\n')
            f.write('            matched = true;\n')
        f.write('            if (matched) {\n')
        f.write('                if (*it->on_frame) (*it->on_frame)(frame);\n')
        f.write('                // D4: mirror what the RPC realization\'s fanOutRpc() does for\n')
        f.write('                // Query.one_shot -- stop future delivery *and* fire on_end,\n')
        f.write('                // the same completion signal a one_shot RPC Read stream gives\n')
        f.write('                // when the provider ends it after the first match, so a\n')
        f.write('                // caller waiting on on_end is not left hanging just because\n')
        f.write('                // pub/sub was selected instead of RPC.\n')
        f.write('                if (it->one_shot) {\n')
        f.write('                    *it->active = false;\n')
        f.write('                    if (*it->on_end) (*it->on_end)(PCL_OK);\n')
        f.write('                }\n')
        f.write('            }\n')
        f.write('            ++it;\n')
        f.write('        }\n')
        f.write('    }\n\n')

        f.write('    ConsumedService              consumed_;\n')
        f.write('    InteractionBinding            request_binding_ = InteractionBinding::kRpc;\n')
        f.write('    InteractionBinding            requirement_binding_ = InteractionBinding::kRpc;\n')
        f.write('    std::list<PubsubTransition>   pubsub_transitions_;\n')
        f.write('};\n\n')

    def _write_submit_overload(self, f, service: ProtoService, rpc: ProtoRpc,
                                cp: Optional[CommandProjectability],
                                wrapper_cpp_type: str, request_pascal: str,
                                duplicate_rpc_names: set) -> None:
        req_t = _cpp_req_type(rpc)
        base = _rpc_symbol_base(service.name, rpc, duplicate_rpc_names)
        async_name = _lc_first(base) + 'Async'
        publish_name = f'publish{request_pascal}'

        f.write(f'    /// \\brief submit() for {rpc.name} (plan §2.1). RPC realization:\n')
        f.write(f'    ///        {async_name}(); remoteAck() is populated on return.\n')
        f.write('    std::future<SubmitResult>\n')
        f.write(f'    submit(const {req_t}& command) {{\n')
        f.write('        if (request_binding_ == InteractionBinding::kRpc) {\n')
        f.write(f'            auto pending = consumed_.{async_name}(command);\n')
        f.write('            return std::async(std::launch::deferred,\n')
        f.write('                [pending = std::move(pending)]() mutable -> SubmitResult {\n')
        f.write('                    auto result = pending.get();\n')
        f.write('                    SubmitResult out;\n')
        f.write('                    out.status = result.status;\n')
        f.write('                    out.accepted = result.ok();\n')
        f.write('                    if (result.ok()) out.ack = result.value;\n')
        f.write('                    return out;\n')
        f.write('                });\n')
        f.write('        }\n')

        projectable = bool(cp and cp.projectable)
        if not projectable:
            reason = cp.reason if cp else 'no_wrapper'
            f.write(f'        // {rpc.name} is not projectable to pub/sub for this contract\n')
            f.write(f'        // (D2: reason="{reason}") -- no wrapper variant carries this\n')
            f.write('        // command\'s payload type. submit() under a pub/sub-realized\n')
            f.write('        // request leg always fails closed here; only the RPC\n')
            f.write('        // realization can carry this command.\n')
            f.write('        SubmitResult out;\n')
            f.write('        out.status = PCL_ERR_STATE;\n')
            f.write('        out.accepted = false;\n')
            f.write('        std::promise<SubmitResult> promise;\n')
            f.write('        promise.set_value(out);\n')
            f.write('        return promise.get_future();\n')
            f.write('    }\n\n')
            return

        if cp.reason == 'is_wrapper':
            f.write(f'        // {rpc.name}\'s request type is the wrapper itself (D2:\n')
            f.write('        // reason="is_wrapper") -- publish it as-is.\n')
            f.write('        const pcl_status_t rc = consumed_.'
                    f'{publish_name}(command);\n')
        else:
            f.write(f'        // {rpc.name} matches the \'{cp.variant_field}\' variant of\n')
            f.write(f'        // {wrapper_cpp_type} (D2: reason="matches_variant").\n')
            f.write(f'        {wrapper_cpp_type} wrapper{{}};\n')
            f.write(f'        wrapper.{cp.variant_field} = command;\n')
            f.write('        const pcl_status_t rc = consumed_.'
                    f'{publish_name}(wrapper);\n')
        f.write('        SubmitResult out;\n')
        f.write('        out.status = rc;\n')
        f.write('        out.accepted = (rc == PCL_OK);\n')
        f.write('        std::promise<SubmitResult> promise;\n')
        f.write('        promise.set_value(out);\n')
        f.write('        return promise.get_future();\n')
        f.write('    }\n\n')

    # -- RequestPortProvider (Phase 3, D6) -------------------------------------

    def _write_request_port_provider(self, f, parsed: ProtoFile, service: ProtoService,
                                      interaction: Interaction,
                                      all_topics: Dict[str, str],
                                      index: ProtoTypeIndex,
                                      duplicate_rpc_names: set) -> None:
        prefix = _service_cpp_prefix(service.name)
        handler_class = f'{prefix}RequestPortHandler'
        class_name = f'{prefix}RequestPortProvider'

        command_rpcs = [rpc for rpc in service.rpcs if rpc.name in _COMMAND_RPC_NAMES]
        read_rpc = next(r for r in service.rpcs if r.name == 'Read')
        projectability: Dict[str, CommandProjectability] = {
            cp.rpc_name: cp
            for cp in command_projectability_for_service(index, parsed, service)
        }
        wrapper_cpp_type = ''
        if projectability:
            wrapper_cpp_type = next(iter(projectability.values())).wrapper_type.split('.')[-1]

        request_leg = next(leg for leg in interaction.legs if leg.name == 'request')
        requirement_leg = next(leg for leg in interaction.legs if leg.name == 'requirement')
        request_topic_key = _topic_key_for_wire_name(
            all_topics, request_leg.side_b[0].endpoint_name)
        requirement_topic_key = _topic_key_for_wire_name(
            all_topics, requirement_leg.side_b[0].endpoint_name)
        request_pascal = _snake_to_pascal(request_topic_key) if request_topic_key else ''
        requirement_pascal = _snake_to_pascal(requirement_topic_key) if requirement_topic_key else ''

        req_t = _cpp_req_type(read_rpc)  # Query
        frame_t = _cpp_rsp_type(read_rpc)[len('std::vector<'):-1]  # Requirement type
        ack_t = _cpp_rsp_type(command_rpcs[0]) if command_rpcs else 'Ack'
        correlation_field = _requirement_correlation_field(index, parsed, read_rpc)

        f.write(_SEP + '\n')
        f.write(f'// {handler_class} / {class_name} -- provider facade for the\n')
        f.write(f'// \'{service.name}\' request port (plan Phase 3, D6). Component code\n')
        f.write(f'// implements {handler_class} (command callbacks only -- Read is\n')
        f.write(f'// facade-internal); {class_name} owns RPC dispatch, a pub/sub probe,\n')
        f.write('// the open-Read-stream registry, and the bounded per-id snapshot store\n')
        f.write('// that backs both realizations of the requirement leg.\n')
        f.write(_SEP + '\n\n')

        # ---- Handler interface --------------------------------------------
        f.write(f'class {handler_class} {{\n')
        f.write('public:\n')
        f.write(f'    virtual ~{handler_class}() = default;\n\n')
        for rpc in command_rpcs:
            on_name = 'on' + rpc.name
            f.write(f'    virtual {ack_t} {on_name}(const {_cpp_req_type(rpc)}& /*request*/) {{\n')
            f.write(f'        return {{}};\n')
            f.write('    }\n')
        f.write('};\n\n')

        # ---- Provider ------------------------------------------------------
        f.write(f'class {class_name} {{\n')
        f.write('public:\n')
        f.write(f'    {class_name}(pcl::Component& host,\n')
        f.write('                    pcl::Executor& executor,\n')
        f.write(f'                    {handler_class}& handler,\n')
        f.write('                    std::string content_type = kJsonContentType)\n')
        f.write('        : host_(&host),\n')
        f.write('          executor_(&executor),\n')
        f.write('          handler_(&handler),\n')
        f.write('          content_type_(std::move(content_type)),\n')
        f.write('          bridge_(*this),\n')
        f.write('          probe_(host, executor, content_type_) {}\n\n')

        f.write('    /// \\brief Owning constructor. \\p handler must not be null;\n')
        f.write('    ///        std::invalid_argument is thrown otherwise.\n')
        f.write(f'    {class_name}(pcl::Component& host,\n')
        f.write('                    pcl::Executor& executor,\n')
        f.write(f'                    std::unique_ptr<{handler_class}> handler,\n')
        f.write('                    std::string content_type = kJsonContentType)\n')
        f.write(f'        : {class_name}(host, executor,\n')
        f.write('                          requireHandler(handler.get()),\n')
        f.write('                          std::move(content_type)) {\n')
        f.write('        owned_handler_ = std::move(handler);\n')
        f.write('    }\n\n')

        f.write(f'    {class_name}(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}& operator=(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}({class_name}&&) = delete;\n')
        f.write(f'    {class_name}& operator=({class_name}&&) = delete;\n\n')

        f.write('    /// \\brief Access the pub/sub probe\'s ConsumedService for transport\n')
        f.write('    ///        routing (routeAllLocal()/configurePubSubTransport() etc.).\n')
        f.write('    ConsumedService& consumedService() { return probe_; }\n\n')

        f.write('    /// \\brief Bind the RPC service ports (this service\'s Create/Update/\n')
        f.write('    ///        Cancel/Read only -- independent of any file-wide\n')
        f.write('    ///        ProvidedService the component may separately construct for\n')
        f.write('    ///        other services in this file) and the pub/sub probe\'s ports.\n')
        f.write('    ///        Both realizations of the request leg are always bound and\n')
        f.write('    ///        listening: which one a deployment actually delivers through\n')
        f.write('    ///        is a routing-manifest decision (D5\'s exclusive groups), not\n')
        f.write('    ///        an application-level choice here -- there is no per-leg\n')
        f.write('    ///        "request_leg" binding to configure on the provider side.\n')
        f.write('    ///        Call once from the owning component\'s on_configure().\n')
        f.write('    pcl_status_t bind() {\n')
        f.write('        if (!supportsContentType(content_type_.c_str())) {\n')
        f.write('            return PCL_ERR_INVALID;\n')
        f.write('        }\n')
        for rpc in command_rpcs:
            svc_const = _rpc_service_const(service.name, rpc, duplicate_rpc_names)
            enum_val = _rpc_enum_value(service.name, rpc, duplicate_rpc_names)
            f.write(f'        if (!addUnaryBinding({svc_const}, ServiceChannel::{enum_val})) return PCL_ERR_NOMEM;\n')
        read_svc_const = _rpc_service_const(service.name, read_rpc, duplicate_rpc_names)
        read_enum_val = _rpc_enum_value(service.name, read_rpc, duplicate_rpc_names)
        f.write(f'        if (!addStreamBinding({read_svc_const}, ServiceChannel::{read_enum_val})) return PCL_ERR_NOMEM;\n')
        f.write(f'        if (auto rc = probe_.add{requirement_pascal}Publisher(); rc != PCL_OK) {{\n')
        f.write('            return rc;\n')
        f.write('        }\n')
        f.write(f'        auto* port = probe_.subscribe{request_pascal}(\n')
        f.write(f'            [this](const {wrapper_cpp_type}& wrapper) {{ dispatchPubsubCommand(wrapper); }});\n')
        f.write('        return port ? PCL_OK : PCL_ERR_NOMEM;\n')
        f.write('    }\n\n')

        f.write('    /// \\brief Select the requirement leg\'s emission realization (D1).\n')
        f.write('    ///        Config shape: {"requirement_leg":"rpc"|"pubsub"} or\n')
        f.write('    ///        {"binding":"rpc"|"pubsub"} (both keys accepted, "binding" as\n')
        f.write('    ///        a fallback). Defaults to "rpc". "request_leg" is accepted and\n')
        f.write('    ///        ignored -- see bind()\'s doc comment for why.\n')
        f.write('    pcl_status_t configureInteractionBinding(std::string_view config_json) {\n')
        f.write('        auto value = configValue(config_json, "requirement_leg");\n')
        f.write('        if (value.empty()) value = configValue(config_json, "binding");\n')
        f.write('        const auto parsed_value = parseInteractionBindingValue(value);\n')
        f.write('        if (!parsed_value) return PCL_ERR_INVALID;\n')
        f.write('        requirement_binding_ = *parsed_value;\n')
        f.write('        return PCL_OK;\n')
        f.write('    }\n\n')

        f.write('    /// \\brief D6: the provider\'s single way to emit a transition. Fans\n')
        f.write('    ///        out to every open Read stream matching the transition\'s\n')
        f.write('    ///        correlation id (RPC realization) or publishes on the\n')
        f.write('    ///        requirement topic (pub/sub realization), per\n')
        f.write('    ///        configureInteractionBinding(). Either way, updates the\n')
        f.write('    ///        bounded per-id snapshot this transition\'s id maps to\n')
        f.write('    ///        (serves RPC Read initial-state callers indirectly via\n')
        f.write('    ///        late re-publication -- see dispatchPubsubCommand()).\n')
        f.write('    class TransitionWriter {\n')
        f.write('    public:\n')
        f.write(f'        explicit TransitionWriter({class_name}& owner) : owner_(&owner) {{}}\n')
        f.write(f'        pcl_status_t send(const {frame_t}& transition) {{\n')
        f.write('            return owner_->sendTransition(transition);\n')
        f.write('        }\n')
        f.write('    private:\n')
        f.write(f'        {class_name}* owner_;\n')
        f.write('    };\n')
        f.write('    TransitionWriter transitionWriter() { return TransitionWriter(*this); }\n\n')

        f.write('private:\n')
        f.write(f'    static {handler_class}& requireHandler({handler_class}* h) {{\n')
        f.write(f'        if (!h) throw std::invalid_argument("{handler_class} must not be null");\n')
        f.write('        return *h;\n')
        f.write('    }\n\n')
        self._write_config_value_helper(f)

        # ---- Bridge: ServiceHandler override scoped to this service --------
        f.write('    // Implements ServiceHandler for just this service\'s rpcs; every\n')
        f.write('    // other rpc in this file (if any) keeps ServiceHandler\'s own default\n')
        f.write('    // stub body, unreferenced by this class\'s dispatch trampolines.\n')
        f.write('    class Bridge final : public ServiceHandler {\n')
        f.write('    public:\n')
        f.write(f'        explicit Bridge({class_name}& owner) : owner_(&owner) {{}}\n\n')
        for rpc in command_rpcs:
            handler_name = _rpc_handler_name(service.name, rpc, duplicate_rpc_names)
            on_name = 'on' + rpc.name
            f.write(f'        {ack_t}\n')
            f.write(f'        {handler_name}(const {_cpp_req_type(rpc)}& request) override {{\n')
            f.write(f'            return owner_->handler_->{on_name}(request);\n')
            f.write('        }\n')
        stream_name = _rpc_stream_handler_name(service.name, read_rpc, duplicate_rpc_names)
        send_frame = _rpc_send_stream_frame_func(service.name, read_rpc, duplicate_rpc_names)
        f.write('        pcl_status_t\n')
        f.write(f'        {stream_name}(const {req_t}& request,\n')
        f.write('        ' + ' ' * len(stream_name) + ' pcl_stream_context_t* stream_context,\n')
        f.write('        ' + ' ' * len(stream_name) + ' const char* content_type) override {\n')
        f.write(f'            StreamWriter<{frame_t}> writer{{\n')
        f.write('                stream_context,\n')
        f.write('                content_type ? content_type : kJsonContentType,\n')
        f.write(f'                &{send_frame}\n')
        f.write('            };\n')
        f.write('            owner_->registerOpenStream(request, std::move(writer));\n')
        f.write('            return PCL_STREAMING;\n')
        f.write('        }\n')
        f.write('    private:\n')
        f.write(f'        {class_name}* owner_;\n')
        f.write('    };\n\n')

        f.write('    struct UnaryBinding {\n')
        f.write('        Bridge*         bridge = nullptr;\n')
        f.write('        ServiceChannel  channel = ServiceChannel{};\n')
        f.write('        std::string     content_type;\n')
        f.write('        std::string     response_storage;\n')
        f.write('    };\n\n')
        f.write('    struct StreamBinding {\n')
        f.write('        Bridge*         bridge = nullptr;\n')
        f.write('        ServiceChannel  channel = ServiceChannel{};\n')
        f.write('        std::string     content_type;\n')
        f.write('    };\n\n')

        f.write('    static pcl_status_t unaryDispatch(pcl_container_t* /*self*/,\n')
        f.write('                                       const pcl_msg_t*  request,\n')
        f.write('                                       pcl_msg_t*        response,\n')
        f.write('                                       pcl_svc_context_t* /*ctx*/,\n')
        f.write('                                       void*             user_data) {\n')
        f.write('        auto* b = static_cast<UnaryBinding*>(user_data);\n')
        f.write('        if (!b || !b->bridge || !response) return PCL_ERR_INVALID;\n')
        f.write('        void*  buf = nullptr;\n')
        f.write('        size_t sz = 0;\n')
        f.write('        const char* ct = (request && request->type_name)\n')
        f.write('                             ? request->type_name\n')
        f.write('                             : b->content_type.c_str();\n')
        f.write('        dispatch(*b->bridge, b->channel,\n')
        f.write('                 request ? request->data : nullptr,\n')
        f.write('                 request ? request->size : 0u,\n')
        f.write('                 ct, &buf, &sz);\n')
        f.write('        b->response_storage.clear();\n')
        f.write('        if (buf && sz > 0u) {\n')
        f.write('            b->response_storage.assign(static_cast<const char*>(buf), sz);\n')
        f.write('            std::free(buf);\n')
        f.write('        }\n')
        f.write('        response->data = b->response_storage.empty() ? nullptr\n')
        f.write('                                                     : b->response_storage.data();\n')
        f.write('        response->size = static_cast<uint32_t>(b->response_storage.size());\n')
        f.write('        response->type_name = ct;\n')
        f.write('        return PCL_OK;\n')
        f.write('    }\n\n')

        f.write('    static pcl_status_t streamDispatch(pcl_container_t* /*self*/,\n')
        f.write('                                        const pcl_msg_t*       request,\n')
        f.write('                                        pcl_stream_context_t*  stream_context,\n')
        f.write('                                        void*                  user_data) {\n')
        f.write('        auto* b = static_cast<StreamBinding*>(user_data);\n')
        f.write('        if (!b || !b->bridge) return PCL_ERR_INVALID;\n')
        f.write('        const char* ct = (request && request->type_name)\n')
        f.write('                             ? request->type_name\n')
        f.write('                             : b->content_type.c_str();\n')
        f.write('        return dispatchStream(*b->bridge, b->channel,\n')
        f.write('                              request ? request->data : nullptr,\n')
        f.write('                              request ? request->size : 0u,\n')
        f.write('                              ct, stream_context);\n')
        f.write('    }\n\n')

        f.write('    bool addUnaryBinding(const char* service_name, ServiceChannel channel) {\n')
        f.write('        unary_bindings_.push_back(UnaryBinding{&bridge_, channel, content_type_, {}});\n')
        f.write('        UnaryBinding& binding = unary_bindings_.back();\n')
        f.write('        pcl::Port port = host_->addService(service_name, content_type_.c_str(),\n')
        f.write(f'                                           &{class_name}::unaryDispatch, &binding);\n')
        f.write('        if (!port) { unary_bindings_.pop_back(); return false; }\n')
        f.write('        ports_.push_back(port);\n')
        f.write('        return true;\n')
        f.write('    }\n\n')

        f.write('    bool addStreamBinding(const char* service_name, ServiceChannel channel) {\n')
        f.write('        stream_bindings_.push_back(StreamBinding{&bridge_, channel, content_type_});\n')
        f.write('        StreamBinding& binding = stream_bindings_.back();\n')
        f.write('        pcl::Port port = host_->addStreamService(service_name, content_type_.c_str(),\n')
        f.write(f'                                                 &{class_name}::streamDispatch, &binding);\n')
        f.write('        if (!port) { stream_bindings_.pop_back(); return false; }\n')
        f.write('        ports_.push_back(port);\n')
        f.write('        return true;\n')
        f.write('    }\n\n')

        # ---- Open-stream registry + fan-out ---------------------------------
        f.write('    struct OpenStream {\n')
        f.write(f'        StreamWriter<{frame_t}> writer;\n')
        f.write(f'        {req_t}                 query;\n')
        f.write('    };\n\n')

        f.write(f'    void registerOpenStream(const {req_t}& query, StreamWriter<{frame_t}> writer) {{\n')
        f.write('        open_streams_.push_back(OpenStream{std::move(writer), query});\n')
        f.write('        // D6: an RPC Read stream opened after a transition was already\n')
        f.write('        // recorded (no further transition may ever follow, e.g. a\n')
        f.write('        // COMPLETED/CANCELLED terminal state) must not wait forever for\n')
        f.write('        // one -- replay the matching stored snapshot(s) immediately, the\n')
        f.write('        // same late-join mitigation dispatchPubsubCommand() already gives\n')
        f.write('        // pub/sub-realized readers via republishSnapshotFor().\n')
        f.write('        replayStoredSnapshots(open_streams_.back());\n')
        f.write('    }\n\n')

        f.write(f'    pcl_status_t fanOutRpc(const {frame_t}& transition) {{\n')
        f.write('        pcl_status_t last_rc = PCL_OK;\n')
        f.write('        for (auto it = open_streams_.begin(); it != open_streams_.end();) {\n')
        f.write('            if (!it->writer.live()) { it = open_streams_.erase(it); continue; }\n')
        f.write('            // A client-cancelled stream stays live() (pcl_stream_cancel()\n')
        f.write('            // only sets the cancelled flag, not ctx_) -- without this check\n')
        f.write('            // it would remain in open_streams_ forever, so every future\n')
        f.write('            // send() keeps hitting a stream nobody is reading from and can\n')
        f.write('            // report PCL_ERR_CANCELLED for otherwise-valid transitions.\n')
        f.write('            if (it->writer.cancelled()) {\n')
        f.write('                it->writer.end();\n')
        f.write('                it = open_streams_.erase(it);\n')
        f.write('                continue;\n')
        f.write('            }\n')
        f.write('            bool matched = it->query.id.empty();\n')
        if correlation_field is not None:
            f.write('            if (!matched')
            f.write(f' && transition.{correlation_field}.has_value()) {{\n')
            f.write('                for (const auto& wanted : it->query.id) {\n')
            f.write(f'                    if (wanted == transition.{correlation_field}->id) {{ matched = true; break; }}\n')
            f.write('                }\n')
            f.write('            }\n')
        else:
            f.write('            // No structurally-resolvable correlation id (see the client\n')
            f.write('            // facade\'s matching comment) -- every open stream matches.\n')
            f.write('            matched = true;\n')
        f.write('            if (matched) last_rc = it->writer.send(transition);\n')
        f.write('            // D4: RPC realization must honour Query.one_shot the same way\n')
        f.write('            // the pub/sub realization\'s dispatchPubsubTransition() does --\n')
        f.write('            // end the stream after its first matching delivery rather than\n')
        f.write('            // continuing to fan out future transitions to it.\n')
        f.write('            if (matched && it->query.one_shot.has_value() && *it->query.one_shot) {\n')
        f.write('                it->writer.end(last_rc);\n')
        f.write('                it = open_streams_.erase(it);\n')
        f.write('                continue;\n')
        f.write('            }\n')
        f.write('            ++it;\n')
        f.write('        }\n')
        f.write('        return last_rc;\n')
        f.write('    }\n\n')

        # ---- Snapshot store --------------------------------------------------
        f.write('    // D6: bounded per-id snapshot of the latest transition, updated on\n')
        f.write('    // every sendTransition() regardless of realization. Deliberately\n')
        f.write('    // minimal -- not the A-GRA retention/history policy (a recorded\n')
        f.write('    // follow-up) -- insertion-ordered eviction of the oldest entry once\n')
        f.write('    // over capacity, not true access-order LRU.\n')
        f.write('    static constexpr size_t kSnapshotCapacity = 64;\n\n')

        f.write(f'    void recordSnapshot(const {frame_t}& transition) {{\n')
        if correlation_field is not None:
            f.write(f'        if (!transition.{correlation_field}.has_value()) return;\n')
            f.write(f'        const std::string& id = transition.{correlation_field}->id;\n')
            f.write('        auto existing = snapshot_index_.find(id);\n')
            f.write('        if (existing != snapshot_index_.end()) {\n')
            f.write('            snapshot_order_.erase(existing->second);\n')
            f.write('            snapshot_index_.erase(existing);\n')
            f.write('        }\n')
            f.write('        snapshot_order_.push_back({id, transition});\n')
            f.write('        snapshot_index_[id] = std::prev(snapshot_order_.end());\n')
            f.write('        while (snapshot_order_.size() > kSnapshotCapacity) {\n')
            f.write('            snapshot_index_.erase(snapshot_order_.front().first);\n')
            f.write('            snapshot_order_.pop_front();\n')
            f.write('        }\n')
        else:
            f.write('        (void)transition;  // no resolvable correlation id -- see fanOutRpc()\n')
        f.write('    }\n\n')

        f.write('    void republishSnapshotFor(const std::string& id) {\n')
        f.write('        auto it = snapshot_index_.find(id);\n')
        f.write('        if (it == snapshot_index_.end()) return;\n')
        f.write('        // Copy out before calling sendTransition(): it re-enters\n')
        f.write('        // recordSnapshot(), which erases-then-reinserts the existing\n')
        f.write('        // entry for this same id -- passing a reference straight into\n')
        f.write('        // that call would alias the list node recordSnapshot erases out\n')
        f.write('        // from under it (use-after-free).\n')
        f.write(f'        const {frame_t} snapshot_copy = it->second->second;\n')
        f.write('        sendTransition(snapshot_copy);\n')
        f.write('    }\n\n')

        f.write('    // Replays every currently-stored snapshot the given stream\'s query\n')
        f.write('    // filter accepts, directly to that stream\'s own writer -- not\n')
        f.write('    // through sendTransition()/fanOutRpc(), which would also re-deliver\n')
        f.write('    // to every other already-open matching stream. Called once, right\n')
        f.write('    // after a new RPC Read stream is registered (see\n')
        f.write('    // registerOpenStream()\'s comment) -- an id\'s current state may\n')
        f.write('    // already be terminal (e.g. COMPLETED/CANCELLED) with no further\n')
        f.write('    // transition ever coming, so a late reader must not wait forever.\n')
        f.write('    void replayStoredSnapshots(OpenStream& stream) {\n')
        f.write('        if (stream.query.id.empty()) {\n')
        f.write('            for (const auto& entry : snapshot_order_) {\n')
        f.write('                if (!stream.writer.live()) return;\n')
        f.write('                stream.writer.send(entry.second);\n')
        f.write('            }\n')
        f.write('            return;\n')
        f.write('        }\n')
        f.write('        for (const auto& wanted : stream.query.id) {\n')
        f.write('            if (!stream.writer.live()) return;\n')
        f.write('            auto it = snapshot_index_.find(wanted);\n')
        f.write('            if (it != snapshot_index_.end()) stream.writer.send(it->second->second);\n')
        f.write('        }\n')
        f.write('    }\n\n')

        f.write(f'    pcl_status_t sendTransition(const {frame_t}& transition) {{\n')
        f.write('        recordSnapshot(transition);\n')
        f.write('        if (requirement_binding_ == InteractionBinding::kRpc) {\n')
        f.write('            return fanOutRpc(transition);\n')
        f.write('        }\n')
        f.write(f'        return probe_.publish{requirement_pascal}(transition);\n')
        f.write('    }\n\n')
        f.write('    friend class TransitionWriter;\n\n')

        # ---- pub/sub command reception + D4 re-publication -------------------
        f.write('    /// \\brief D4: pub/sub-realized command reception -- unwraps the\n')
        f.write(f'    ///        {wrapper_cpp_type} oneof and dispatches to the same handler\n')
        f.write('    ///        methods the RPC realization uses (the returned Ack is\n')
        f.write('    ///        discarded, per D3: pub/sub has no synchronous ack), then\n')
        f.write('    ///        re-publishes the current snapshot for the command\'s id if\n')
        f.write('    ///        one exists -- the late-join mitigation D4 calls for.\n')
        f.write(f'    void dispatchPubsubCommand(const {wrapper_cpp_type}& wrapper) {{\n')
        # matches_variant commands (Update/Cancel in every contract seen at
        # authoring time) get a specific `if (wrapper.<variant>.has_value())`
        # check, tried first, since each publishes under a *different*
        # populated oneof field on the shared wrapper type (Phase 2's
        # `_write_submit_overload` `matches_variant` branch). The is_wrapper
        # command (Create) has no variant field of its own -- its rpc
        # parameter type *is* the whole wrapper -- so it is the unconditional
        # `else` fallback, tried last: any wire message not recognised as one
        # of the specific variants is a Create. At most one is_wrapper
        # command is expected per service (D2); a second would indicate an
        # ill-formed contract, not something to silently disambiguate here.
        variant_commands = [
            (rpc, cp) for rpc in command_rpcs
            for cp in [projectability.get(rpc.name)]
            if cp and cp.projectable and cp.reason == 'matches_variant'
        ]
        wrapper_commands = [
            (rpc, cp) for rpc in command_rpcs
            for cp in [projectability.get(rpc.name)]
            if cp and cp.projectable and cp.reason == 'is_wrapper'
        ]
        for i, (rpc, cp) in enumerate(variant_commands):
            on_name = 'on' + rpc.name
            branch = 'if' if i == 0 else 'else if'
            f.write(f'        {branch} (wrapper.{cp.variant_field}.has_value()) {{\n')
            f.write(f'            handler_->{on_name}(*wrapper.{cp.variant_field});\n')
            resolved = _command_correlation_accessor(index, parsed, cp.request_type)
            self._write_republish_call(f, resolved, f'(*wrapper.{cp.variant_field})')
            f.write('        }\n')
        if wrapper_commands:
            rpc, cp = wrapper_commands[0]
            on_name = 'on' + rpc.name
            keyword = 'else' if variant_commands else 'if (true)'
            f.write(f'        {keyword} {{\n')
            f.write(f'            handler_->{on_name}(wrapper);\n')
            # is_wrapper's own request type *is* the wrapper (a multi-variant
            # oneof), so _command_correlation_accessor can't resolve it
            # directly the way it resolves matches_variant's single-field
            # type. Instead: find the wrapper's oneof field not already
            # claimed by a matches_variant command -- by construction (D2)
            # that leftover field is the one is_wrapper's payload actually
            # populates (e.g. Create's `ma_action`) -- and resolve
            # correlation on *that* field's type. The leftover field itself
            # is a further tl::optional on the wrapper, so it needs its own
            # has_value() check before `_write_republish_call`'s (possible)
            # second-level one.
            claimed = {c.variant_field for _, c in variant_commands}
            leftover_field, leftover_resolved = self._leftover_wrapper_variant(
                index, parsed, cp.wrapper_type, claimed)
            if leftover_field is not None and leftover_resolved is not None:
                f.write(f'            if (wrapper.{leftover_field}.has_value()) {{\n')
                self._write_republish_call(
                    f, leftover_resolved, f'(*wrapper.{leftover_field})')
                f.write('            }\n')
            f.write('        }\n')
        # Non-projectable commands (D2) never arrive via pub/sub by
        # construction (submit() fails closed before publishing), so there is
        # deliberately no branch for them here.
        f.write('    }\n\n')

        f.write(f'    pcl::Component*                  host_     = nullptr;\n')
        f.write(f'    pcl::Executor*                   executor_ = nullptr;\n')
        f.write(f'    {handler_class}*                 handler_  = nullptr;\n')
        f.write(f'    std::unique_ptr<{handler_class}> owned_handler_;\n')
        f.write('    std::string                      content_type_;\n')
        f.write('    Bridge                            bridge_;\n')
        f.write('    ConsumedService                   probe_;\n')
        f.write('    InteractionBinding                 requirement_binding_ = InteractionBinding::kRpc;\n')
        f.write('    std::list<UnaryBinding>           unary_bindings_;\n')
        f.write('    std::list<StreamBinding>          stream_bindings_;\n')
        f.write('    std::vector<pcl::Port>            ports_;\n')
        f.write('    std::list<OpenStream>             open_streams_;\n')
        f.write('    std::list<std::pair<std::string, ' + frame_t + '>> snapshot_order_;\n')
        f.write('    std::unordered_map<std::string, decltype(snapshot_order_)::iterator> snapshot_index_;\n')
        f.write('};\n\n')

    def _write_republish_call(
        self, f, resolved: Optional[Tuple[Optional[str], str]], value_expr: str,
    ) -> None:
        """Emit republishSnapshotFor(id) for a command's already-dereferenced
        `value_expr`, using a `_command_correlation_accessor`-shaped
        `(needs_check, accessor)` result; emits nothing (documented gap, not
        a silent no-op typo -- see the surrounding comment in the caller)
        when `resolved` is `None`."""
        if resolved is None:
            return
        needs_check, accessor = resolved
        if needs_check is None:
            f.write(f'            republishSnapshotFor({value_expr}{accessor});\n')
            return
        f.write(f'            if ({value_expr}.{needs_check}.has_value()) {{\n')
        f.write(f'                republishSnapshotFor((*{value_expr}.{needs_check}){accessor});\n')
        f.write('            }\n')

    def _leftover_wrapper_variant(
        self, index: ProtoTypeIndex, pf: ProtoFile, wrapper_type_fqn: str,
        claimed_fields: set,
    ) -> Tuple[Optional[str], Optional[Tuple[Optional[str], str]]]:
        """The is_wrapper command's payload variant, found by elimination:
        the wrapper message's one oneof field not already claimed by a
        matches_variant command (D2) -- see the caller's comment. Returns
        `(field_name, resolved)` where `resolved` is a
        `_command_correlation_accessor`-shaped result for that field's type;
        either may be `None` if the wrapper has no oneof, more than one
        unclaimed field (an ambiguous/ill-formed contract this function does
        not try to disambiguate further), or the unclaimed field's type
        doesn't resolve a correlation id (same fallback philosophy as
        `_command_correlation_accessor`).
        """
        wrapper_msg = index.resolve_message(wrapper_type_fqn) if wrapper_type_fqn else None
        if wrapper_msg is None or len(wrapper_msg.oneofs) != 1:
            return None, None
        unclaimed = [fld for fld in wrapper_msg.oneofs[0].fields
                     if fld.name not in claimed_fields]
        if len(unclaimed) != 1:
            return None, None
        field = unclaimed[0]
        wrapper_pkg = wrapper_type_fqn.rsplit('.', 1)[0] if '.' in wrapper_type_fqn else ''
        field_fqn = _fqn(index, field.type, wrapper_pkg)
        resolved = _command_correlation_accessor(index, pf, field_fqn)
        return field.name, resolved

    # -- InformationPortSink ---------------------------------------------------

    def _write_information_port_sink(self, f, service: ProtoService,
                                      interaction: Interaction,
                                      all_topics: Dict[str, str],
                                      duplicate_rpc_names: set) -> None:
        prefix = _service_cpp_prefix(service.name)
        class_name = f'{prefix}InformationPortSink'

        read_rpc = next(r for r in service.rpcs if r.name == 'Read')
        info_leg = next(leg for leg in interaction.legs if leg.name == 'information')
        info_topic_key = _topic_key_for_wire_name(
            all_topics, info_leg.side_b[0].endpoint_name)
        info_pascal = _snake_to_pascal(info_topic_key) if info_topic_key else ''

        req_t = _cpp_req_type(read_rpc)
        frame_t = _cpp_rsp_type(read_rpc)[len('std::vector<'):-1]
        base = _rpc_symbol_base(service.name, read_rpc, duplicate_rpc_names)
        streaming_name = _lc_first(base) + 'Streaming'

        f.write(_SEP + '\n')
        f.write(f'// {class_name} -- consumer facade for the \'{service.name}\' information\n')
        f.write('// port: reads the Data-1 publication via whichever realization\n')
        f.write('// configureInteractionBinding() selects.\n')
        f.write(_SEP + '\n\n')

        f.write(f'class {class_name} {{\n')
        f.write('public:\n')
        f.write(f'    {class_name}(pcl::Component& host,\n')
        f.write('                    pcl::Executor& executor,\n')
        f.write('                    std::string content_type = kJsonContentType)\n')
        f.write('        : consumed_(host, executor, std::move(content_type)) {}\n\n')

        f.write(f'    {class_name}(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}& operator=(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}({class_name}&&) = delete;\n')
        f.write(f'    {class_name}& operator=({class_name}&&) = delete;\n\n')

        f.write('    ConsumedService& consumedService() { return consumed_; }\n\n')

        f.write('    /// \\brief Bind the pub/sub-realization subscriber port. Call once\n')
        f.write('    ///        from the owning component\'s on_configure() -- see\n')
        f.write('    ///        RequestPortClient::bind()\'s doc comment; the same PCL\n')
        f.write('    ///        configuring-phase-only constraint applies here.\n')
        f.write('    pcl_status_t bind() {\n')
        f.write(f'        auto* port = consumed_.subscribe{info_pascal}(\n')
        f.write(f'            [this](const {frame_t}& msg) {{ dispatchPubsubInformation(msg); }});\n')
        f.write('        return port ? PCL_OK : PCL_ERR_NOMEM;\n')
        f.write('    }\n\n')

        f.write('    /// \\brief Select the realization (plan D1): {"binding":"rpc"} or\n')
        f.write('    ///        {"binding":"pubsub"}. Defaults to "rpc" when unset.\n')
        f.write('    pcl_status_t configureInteractionBinding(std::string_view config_json) {\n')
        f.write('        const auto value = configValue(config_json, "binding");\n')
        f.write('        const auto parsed_value = parseInteractionBindingValue(value);\n')
        f.write('        if (!parsed_value) return PCL_ERR_INVALID;\n')
        f.write('        binding_ = *parsed_value;\n')
        f.write('        return PCL_OK;\n')
        f.write('    }\n\n')

        f.write('    SubscriptionHandle\n')
        f.write(f'    subscribe(std::function<void(const {frame_t}&)> on_msg) {{\n')
        f.write('        if (binding_ == InteractionBinding::kRpc) {\n')
        f.write(f'            return SubscriptionHandle(consumed_.{streaming_name}(\n')
        f.write(f'                {req_t}{{}}, std::move(on_msg)));\n')
        f.write('        }\n')
        f.write('        // Pub/sub realization: register into the dispatch list bind()\n')
        f.write('        // populated a single shared subscription for.\n')
        f.write('        auto active = std::make_shared<bool>(true);\n')
        f.write(f'        auto on_msg_shared = std::make_shared<std::function<void(const {frame_t}&)>>(\n')
        f.write('            std::move(on_msg));\n')
        f.write('        pubsub_subscribers_.push_back({active, on_msg_shared});\n')
        f.write('        return SubscriptionHandle(active);\n')
        f.write('    }\n\n')

        f.write('private:\n')
        self._write_config_value_helper(f)
        f.write('    struct PubsubSubscriber {\n')
        f.write('        std::shared_ptr<bool> active;\n')
        f.write(f'        std::shared_ptr<std::function<void(const {frame_t}&)>> on_msg;\n')
        f.write('    };\n\n')

        f.write(f'    void dispatchPubsubInformation(const {frame_t}& msg) {{\n')
        f.write('        for (auto it = pubsub_subscribers_.begin();\n')
        f.write('             it != pubsub_subscribers_.end();) {\n')
        f.write('            if (!*it->active) { it = pubsub_subscribers_.erase(it); continue; }\n')
        f.write('            if (*it->on_msg) (*it->on_msg)(msg);\n')
        f.write('            ++it;\n')
        f.write('        }\n')
        f.write('    }\n\n')

        f.write('    ConsumedService                 consumed_;\n')
        f.write('    InteractionBinding               binding_ = InteractionBinding::kRpc;\n')
        f.write('    std::list<PubsubSubscriber>      pubsub_subscribers_;\n')
        f.write('};\n\n')

    # -- InformationPortSource (Phase 3) ---------------------------------------

    def _write_information_port_source(self, f, service: ProtoService,
                                        interaction: Interaction,
                                        all_topics: Dict[str, str],
                                        duplicate_rpc_names: set) -> None:
        prefix = _service_cpp_prefix(service.name)
        class_name = f'{prefix}InformationPortSource'

        read_rpc = next(r for r in service.rpcs if r.name == 'Read')
        info_leg = next(leg for leg in interaction.legs if leg.name == 'information')
        info_topic_key = _topic_key_for_wire_name(
            all_topics, info_leg.side_b[0].endpoint_name)
        info_pascal = _snake_to_pascal(info_topic_key) if info_topic_key else ''

        req_t = _cpp_req_type(read_rpc)  # google.protobuf.Empty
        frame_t = _cpp_rsp_type(read_rpc)[len('std::vector<'):-1]
        svc_const = _rpc_service_const(service.name, read_rpc, duplicate_rpc_names)
        enum_val = _rpc_enum_value(service.name, read_rpc, duplicate_rpc_names)
        handler_name = _rpc_handler_name(service.name, read_rpc, duplicate_rpc_names)
        stream_name = _rpc_stream_handler_name(service.name, read_rpc, duplicate_rpc_names)
        send_frame = _rpc_send_stream_frame_func(service.name, read_rpc, duplicate_rpc_names)

        f.write(_SEP + '\n')
        f.write(f'// {class_name} -- provider facade for the \'{service.name}\'\n')
        f.write('// information port: publish() fans out to every open Read stream\n')
        f.write('// (RPC realization) or publishes the information topic (pub/sub\n')
        f.write('// realization), per configureInteractionBinding(). No per-message\n')
        f.write('// correlation/snapshot store -- Data-1 publications are a one-way\n')
        f.write('// broadcast, not a correlated request/requirement pair (D6 is scoped\n')
        f.write('// to Request-shape ports only).\n')
        f.write(_SEP + '\n\n')

        f.write(f'class {class_name} {{\n')
        f.write('public:\n')
        f.write(f'    {class_name}(pcl::Component& host,\n')
        f.write('                    pcl::Executor& executor,\n')
        f.write('                    std::string content_type = kJsonContentType)\n')
        f.write('        : host_(&host),\n')
        f.write('          executor_(&executor),\n')
        f.write('          content_type_(std::move(content_type)),\n')
        f.write('          bridge_(*this),\n')
        f.write('          probe_(host, executor, content_type_) {}\n\n')

        f.write(f'    {class_name}(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}& operator=(const {class_name}&) = delete;\n')
        f.write(f'    {class_name}({class_name}&&) = delete;\n')
        f.write(f'    {class_name}& operator=({class_name}&&) = delete;\n\n')

        f.write('    ConsumedService& consumedService() { return probe_; }\n\n')

        f.write('    /// \\brief Bind the RPC stream port (this service\'s Read only) and\n')
        f.write('    ///        the pub/sub probe\'s publisher port. Both realizations are\n')
        f.write('    ///        always bound; see the sibling RequestPortProvider::bind()\'s\n')
        f.write('    ///        doc comment for why. Call once from on_configure().\n')
        f.write('    pcl_status_t bind() {\n')
        f.write('        if (!supportsContentType(content_type_.c_str())) {\n')
        f.write('            return PCL_ERR_INVALID;\n')
        f.write('        }\n')
        f.write(f'        if (!addStreamBinding({svc_const}, ServiceChannel::{enum_val})) {{\n')
        f.write('            return PCL_ERR_NOMEM;\n')
        f.write('        }\n')
        f.write(f'        return probe_.add{info_pascal}Publisher();\n')
        f.write('    }\n\n')

        f.write('    /// \\brief Select the emission realization (D1): {"binding":"rpc"}\n')
        f.write('    ///        or {"binding":"pubsub"}. Defaults to "rpc".\n')
        f.write('    pcl_status_t configureInteractionBinding(std::string_view config_json) {\n')
        f.write('        const auto value = configValue(config_json, "binding");\n')
        f.write('        const auto parsed_value = parseInteractionBindingValue(value);\n')
        f.write('        if (!parsed_value) return PCL_ERR_INVALID;\n')
        f.write('        binding_ = *parsed_value;\n')
        f.write('        return PCL_OK;\n')
        f.write('    }\n\n')

        f.write(f'    pcl_status_t publish(const {frame_t}& msg) {{\n')
        f.write('        if (binding_ == InteractionBinding::kRpc) {\n')
        f.write('            pcl_status_t last_rc = PCL_OK;\n')
        f.write('            for (auto it = open_streams_.begin(); it != open_streams_.end();) {\n')
        f.write('                if (!it->live()) { it = open_streams_.erase(it); continue; }\n')
        f.write('                // See RequestPortProvider::fanOutRpc()\'s identical comment:\n')
        f.write('                // a cancelled sink stays live() until explicitly ended, so\n')
        f.write('                // this must be checked separately or a dropped/cancelled\n')
        f.write('                // sink\'s writer lingers forever, taking every future\n')
        f.write('                // publish() down the PCL_ERR_CANCELLED path with it.\n')
        f.write('                if (it->cancelled()) {\n')
        f.write('                    it->end();\n')
        f.write('                    it = open_streams_.erase(it);\n')
        f.write('                    continue;\n')
        f.write('                }\n')
        f.write('                last_rc = it->send(msg);\n')
        f.write('                ++it;\n')
        f.write('            }\n')
        f.write('            return last_rc;\n')
        f.write('        }\n')
        f.write(f'        return probe_.publish{info_pascal}(msg);\n')
        f.write('    }\n\n')

        f.write('private:\n')
        self._write_config_value_helper(f)

        f.write('    // ServiceHandler override scoped to this service\'s single Read rpc\n')
        f.write('    // (mirrors RequestPortProvider::Bridge\'s narrowing, see its comment).\n')
        f.write('    class Bridge final : public ServiceHandler {\n')
        f.write('    public:\n')
        f.write(f'        explicit Bridge({class_name}& owner) : owner_(&owner) {{}}\n\n')
        f.write('        pcl_status_t\n')
        f.write(f'        {stream_name}(const {req_t}& /*request*/,\n')
        f.write('        ' + ' ' * len(stream_name) + ' pcl_stream_context_t* stream_context,\n')
        f.write('        ' + ' ' * len(stream_name) + ' const char* content_type) override {\n')
        f.write(f'            owner_->open_streams_.push_back(StreamWriter<{frame_t}>{{\n')
        f.write('                stream_context,\n')
        f.write('                content_type ? content_type : kJsonContentType,\n')
        f.write(f'                &{send_frame}\n')
        f.write('            });\n')
        f.write('            return PCL_STREAMING;\n')
        f.write('        }\n')
        f.write('    private:\n')
        f.write(f'        {class_name}* owner_;\n')
        f.write('    };\n\n')

        f.write('    struct StreamBinding {\n')
        f.write('        Bridge*         bridge = nullptr;\n')
        f.write('        ServiceChannel  channel = ServiceChannel{};\n')
        f.write('        std::string     content_type;\n')
        f.write('    };\n\n')

        f.write('    static pcl_status_t streamDispatch(pcl_container_t* /*self*/,\n')
        f.write('                                        const pcl_msg_t*       request,\n')
        f.write('                                        pcl_stream_context_t*  stream_context,\n')
        f.write('                                        void*                  user_data) {\n')
        f.write('        auto* b = static_cast<StreamBinding*>(user_data);\n')
        f.write('        if (!b || !b->bridge) return PCL_ERR_INVALID;\n')
        f.write('        const char* ct = (request && request->type_name)\n')
        f.write('                             ? request->type_name\n')
        f.write('                             : b->content_type.c_str();\n')
        f.write('        return dispatchStream(*b->bridge, b->channel,\n')
        f.write('                              request ? request->data : nullptr,\n')
        f.write('                              request ? request->size : 0u,\n')
        f.write('                              ct, stream_context);\n')
        f.write('    }\n\n')

        f.write('    bool addStreamBinding(const char* service_name, ServiceChannel channel) {\n')
        f.write('        stream_bindings_.push_back(StreamBinding{&bridge_, channel, content_type_});\n')
        f.write('        StreamBinding& binding = stream_bindings_.back();\n')
        f.write('        pcl::Port port = host_->addStreamService(service_name, content_type_.c_str(),\n')
        f.write(f'                                                 &{class_name}::streamDispatch, &binding);\n')
        f.write('        if (!port) { stream_bindings_.pop_back(); return false; }\n')
        f.write('        ports_.push_back(port);\n')
        f.write('        return true;\n')
        f.write('    }\n\n')

        f.write('    pcl::Component*           host_     = nullptr;\n')
        f.write('    pcl::Executor*            executor_ = nullptr;\n')
        f.write('    std::string               content_type_;\n')
        f.write('    Bridge                    bridge_;\n')
        f.write('    ConsumedService           probe_;\n')
        f.write('    InteractionBinding        binding_ = InteractionBinding::kRpc;\n')
        f.write('    std::list<StreamBinding>  stream_bindings_;\n')
        f.write('    std::vector<pcl::Port>    ports_;\n')
        f.write(f'    std::list<StreamWriter<{frame_t}>> open_streams_;\n')
        f.write('};\n\n')
