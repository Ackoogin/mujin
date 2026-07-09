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
            else:
                self._write_information_port_sink(
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
        f.write('    ///        the first matching frame -- "what is observable now", not a\n')
        f.write('    ///        guaranteed single delivery). Late-join is out of scope for\n')
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
        f.write('        pubsub_transitions_.push_back(\n')
        f.write('            PubsubTransition{active, want_ids, one_shot, on_frame});\n')
        f.write('        return SubscriptionHandle(active);\n')
        f.write('    }\n\n')

        f.write('private:\n')
        self._write_config_value_helper(f)
        f.write('    struct PubsubTransition {\n')
        f.write('        std::shared_ptr<bool>                     active;\n')
        f.write('        std::shared_ptr<std::vector<std::string>> want_ids;\n')
        f.write('        bool                                      one_shot = false;\n')
        f.write(f'        std::shared_ptr<std::function<void(const {read_frame_t}&)>> on_frame;\n')
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
        f.write('                if (it->one_shot) *it->active = false;\n')
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
