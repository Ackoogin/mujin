#!/usr/bin/env python3
"""Component-shaped facade header emitter for CppServiceGenerator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import List, Tuple

from proto_parser import (
    ProtoFile,
    ProtoRpc,
    snake_to_pascal as _snake_to_pascal,
    lc_first as _lc_first,
)
from .naming import (
    _SEP,
    _duplicate_rpc_names,
    _rpc_symbol_base,
    _rpc_handler_name,
    _rpc_enum_value,
    _rpc_service_const,
    _rpc_invoke_func,
    _rpc_decode_response_func,
    _rpc_stream_handler_name,
    _rpc_decode_stream_frame_func,
    _rpc_send_stream_frame_func,
    _rpc_invoke_stream_func,
    _cpp_req_type,
    _cpp_rsp_type,
    _is_provided,
)


class ComponentsFacadeEmitterMixin:
    """ProvidedHandler / ProvidedService / ConsumedService facade
    header emission."""

    # -- Component-shaped facade header (.hpp, header-only) --------------------
    #
    # Emits ProvidedHandler / ProvidedService / ConsumedService, layered on
    # top of the existing typed invoke/dispatch/encode/decode primitives. The
    # generated classes are *service bindings*, not pcl::Component subclasses:
    # users compose them as members of their own component and call bind() on
    # the provided side from on_configure().

    def _write_components_header(self, path: Path, file_prefix: str,
                                  full_ns: str, parsed: ProtoFile,
                                  all_rpcs: List[Tuple[str, ProtoRpc]]):
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)
        hpp_name = file_prefix + '.hpp'
        is_provided = _is_provided(parsed)
        sub_topics, pub_topics = self._topics.topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated component facade for the service binding.\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('//\n')
            f.write('// Layers on top of the low-level invoke/dispatch primitives in\n')
            f.write(f'// {hpp_name}. Components written against this header do not need to\n')
            f.write('// see pcl_msg_t, stream contexts, or response buffer ownership.\n')
            f.write('#pragma once\n\n')

            f.write(f'#include "{hpp_name}"\n\n')
            f.write('#include <pcl/component.hpp>\n')
            f.write('#include <pcl/executor.hpp>\n\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <functional>\n')
            f.write('#include <future>\n')
            f.write('#include <list>\n')
            f.write('#include <memory>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <string>\n')
            f.write('#include <string_view>\n')
            f.write('#include <unordered_map>\n')
            f.write('#include <utility>\n')
            f.write('#include <vector>\n\n')

            f.write(f'namespace {full_ns} {{\n\n')

            # ---- Result<T> -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Result<T> -- typed result + pcl_status_t for async API.\n')
            f.write(_SEP + '\n\n')
            f.write('template <class T>\n')
            f.write('struct Result {\n')
            f.write('    pcl_status_t status = PCL_OK;\n')
            f.write('    T            value{};\n\n')
            f.write('    bool ok() const { return status == PCL_OK; }\n')
            f.write('    explicit operator bool() const { return ok(); }\n')
            f.write('};\n\n')

            # ---- StreamWriter<T> -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// StreamWriter<T> -- typed write handle the server-streaming handler\n')
            f.write('// receives. The handler may keep it alive across ticks and call send()\n')
            f.write('// when new frames are available, then end(). Dropping the writer without\n')
            f.write('// calling end() aborts the stream with PCL_ERR_STATE.\n')
            f.write(_SEP + '\n\n')

            f.write('template <class T>\n')
            f.write('class StreamWriter {\n')
            f.write('public:\n')
            f.write('    using SendFn = pcl_status_t (*)(pcl_stream_context_t*,\n')
            f.write('                                    const T&,\n')
            f.write('                                    const char*);\n\n')
            f.write('    /// \\brief Live writer: send() goes on the wire via pcl_stream_send.\n')
            f.write('    StreamWriter(pcl_stream_context_t* ctx,\n')
            f.write('                 std::string content_type,\n')
            f.write('                 SendFn send_fn)\n')
            f.write('        : ctx_(ctx),\n')
            f.write('          content_type_(std::move(content_type)),\n')
            f.write('          send_fn_(send_fn) {}\n\n')
            f.write('    /// \\brief Collecting writer: send() appends to \\p sink. Used when the\n')
            f.write('    ///        binding bridges a streaming-RPC handler to a unary caller.\n')
            f.write('    static StreamWriter collecting(std::vector<T>* sink) {\n')
            f.write('        StreamWriter w;\n')
            f.write('        w.sink_ = sink;\n')
            f.write('        return w;\n')
            f.write('    }\n\n')
            f.write('    StreamWriter() = default;\n')
            f.write('    StreamWriter(const StreamWriter&) = delete;\n')
            f.write('    StreamWriter& operator=(const StreamWriter&) = delete;\n')
            f.write('    StreamWriter(StreamWriter&& o) noexcept { *this = std::move(o); }\n')
            f.write('    StreamWriter& operator=(StreamWriter&& o) noexcept {\n')
            f.write('        if (this != &o) {\n')
            f.write('            abortIfLive();\n')
            f.write('            ctx_          = o.ctx_;          o.ctx_      = nullptr;\n')
            f.write('            content_type_ = std::move(o.content_type_);\n')
            f.write('            send_fn_      = o.send_fn_;      o.send_fn_  = nullptr;\n')
            f.write('            sink_         = o.sink_;         o.sink_     = nullptr;\n')
            f.write('        }\n')
            f.write('        return *this;\n')
            f.write('    }\n')
            f.write('    ~StreamWriter() { abortIfLive(); }\n\n')

            f.write('    /// \\brief Emit one typed frame. Returns PCL_OK on success.\n')
            f.write('    pcl_status_t send(const T& frame) {\n')
            f.write('        if (sink_)   { sink_->push_back(frame); return PCL_OK; }\n')
            f.write('        if (!ctx_ || !send_fn_) return PCL_ERR_STATE;\n')
            f.write('        return send_fn_(ctx_, frame, content_type_.c_str());\n')
            f.write('    }\n\n')

            f.write('    /// \\brief End the stream. \\p status == PCL_OK clean-ends, otherwise\n')
            f.write('    ///        aborts with the given status. Idempotent.\n')
            f.write('    pcl_status_t end(pcl_status_t status = PCL_OK) {\n')
            f.write('        if (sink_) { sink_ = nullptr; return PCL_OK; }\n')
            f.write('        if (!ctx_) return PCL_OK;\n')
            f.write('        pcl_stream_context_t* c = ctx_;\n')
            f.write('        ctx_ = nullptr;\n')
            f.write('        send_fn_ = nullptr;\n')
            f.write('        if (status == PCL_OK) return pcl_stream_end(c);\n')
            f.write('        return pcl_stream_abort(c, status);\n')
            f.write('    }\n\n')

            f.write('    /// \\brief True if the client has asked to cancel the stream.\n')
            f.write('    ///        Servers emitting long streams should poll this and stop.\n')
            f.write('    bool cancelled() const {\n')
            f.write('        return ctx_ && pcl_stream_is_cancelled(ctx_);\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Live state: an open wire stream the handler must end().\n')
            f.write('    ///        False for collecting writers and after end() has run.\n')
            f.write('    bool live() const { return ctx_ != nullptr; }\n\n')

            f.write('private:\n')
            f.write('    void abortIfLive() {\n')
            f.write('        if (!ctx_) return;\n')
            f.write('        pcl_stream_context_t* c = ctx_;\n')
            f.write('        ctx_ = nullptr;\n')
            f.write('        send_fn_ = nullptr;\n')
            f.write('        (void)pcl_stream_abort(c, PCL_ERR_STATE);\n')
            f.write('    }\n\n')

            f.write('    pcl_stream_context_t* ctx_ = nullptr;\n')
            f.write('    std::string           content_type_;\n')
            f.write('    SendFn                send_fn_ = nullptr;\n')
            f.write('    std::vector<T>*       sink_ = nullptr;\n')
            f.write('};\n\n')

            # ---- ProvidedHandler -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ProvidedHandler -- typed callbacks the component author overrides.\n')
            f.write('//\n')
            f.write('// One on<Name> method per RPC. Unary RPCs return a typed value. Server-\n')
            f.write('// streaming RPCs take a StreamWriter<Frame> the handler keeps and pumps\n')
            f.write('// frames to over time; it must call writer.end() when the stream is\n')
            f.write('// complete (the writer auto-aborts if dropped without end()).\n')
            f.write(_SEP + '\n\n')

            f.write('class ProvidedHandler {\n')
            f.write('public:\n')
            f.write('    virtual ~ProvidedHandler() = default;\n')

            current_svc = None
            for svc_name, rpc in all_rpcs:
                if svc_name != current_svc:
                    f.write(f'\n    // {svc_name}\n')
                    current_svc = svc_name
                on_name = 'on' + _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                if rpc.server_streaming:
                    frame_t = _cpp_rsp_type(rpc)[len('std::vector<'):-1]
                    f.write('    virtual void\n')
                    f.write(f'    {on_name}(const {_cpp_req_type(rpc)}& /*request*/,\n')
                    f.write(f'        StreamWriter<{frame_t}> writer) {{\n')
                    f.write('        // Default: empty stream. Override to emit frames + end.\n')
                    f.write('        writer.end();\n')
                    f.write('    }\n')
                else:
                    f.write(f'    virtual {_cpp_rsp_type(rpc)}\n')
                    f.write(f'    {on_name}(const {_cpp_req_type(rpc)}& /*request*/) {{\n')
                    f.write('        return {};\n')
                    f.write('    }\n')
            f.write('};\n\n')

            # ---- ProvidedService -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ProvidedService -- attach to your pcl::Component to host this package\'s\n')
            f.write('// RPCs. Owns the ServiceHandler-bridge adapter and the per-channel binding\n')
            f.write('// storage (response buffer, content type). Lifecycle of any open streams is\n')
            f.write('// owned by the user via StreamWriter::end().\n')
            f.write('//\n')
            f.write('// Usage: construct as a member of your component, call bind() from\n')
            f.write('// on_configure(); optionally restrict callers with routeAllRemote().\n')
            f.write(_SEP + '\n\n')

            f.write('class ProvidedService {\n')
            f.write('public:\n')
            f.write('    ProvidedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    ProvidedHandler& handler,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : host_(&host),\n')
            f.write('          executor_(&executor),\n')
            f.write('          handler_(&handler),\n')
            f.write('          content_type_(std::move(content_type)),\n')
            f.write('          bridge_(*this) {}\n\n')

            f.write('    /// \\brief Owning constructor. \\p handler must not be null;\n')
            f.write('    ///        std::invalid_argument is thrown otherwise.\n')
            f.write('    ProvidedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    std::unique_ptr<ProvidedHandler> handler,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : ProvidedService(host, executor,\n')
            f.write('                          requireHandler(handler.get()),\n')
            f.write('                          std::move(content_type)) {\n')
            f.write('        owned_handler_ = std::move(handler);\n')
            f.write('    }\n\n')

            f.write('    ProvidedService(const ProvidedService&) = delete;\n')
            f.write('    ProvidedService& operator=(const ProvidedService&) = delete;\n')
            f.write('    ProvidedService(ProvidedService&&) = delete;\n')
            f.write('    ProvidedService& operator=(ProvidedService&&) = delete;\n\n')

            f.write('    /// \\brief Install the service ports on the host component. Call from\n')
            f.write('    ///        the host\'s on_configure().\n')
            f.write('    pcl_status_t bind() {\n')
            f.write('        if (!supportsContentType(content_type_.c_str())) {\n')
            f.write('            return PCL_ERR_INVALID;\n')
            f.write('        }\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                enum_val = _rpc_enum_value(
                    svc_name, rpc, duplicate_rpc_names)
                if rpc.server_streaming:
                    f.write(f'        if (!addStreamBinding({svc_const}, ServiceChannel::{enum_val})) return PCL_ERR_NOMEM;\n')
                else:
                    f.write(f'        if (!addUnaryBinding({svc_const}, ServiceChannel::{enum_val})) return PCL_ERR_NOMEM;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Restrict every advertised service to local callers only.\n')
            f.write('    pcl_status_t routeAllLocal() {\n')
            f.write('        for (const auto& port : ports_) {\n')
            f.write('            const pcl_status_t rc = port.routeLocal();\n')
            f.write('            if (rc != PCL_OK) return rc;\n')
            f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Restrict every advertised service to a single peer.\n')
            f.write('    pcl_status_t routeAllRemote(std::string_view peer_id) {\n')
            f.write('        for (const auto& port : ports_) {\n')
            f.write('            const pcl_status_t rc = port.routeRemote(peer_id);\n')
            f.write('            if (rc != PCL_OK) return rc;\n')
            f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Configure service exposure from an opaque JSON route config.\n')
            f.write('    ///\n')
            f.write('    /// Supported shapes are {"transport":"local"} and\n')
            f.write('    /// {"transport":"remote","peer":"peer_id"}. "route" may be used\n')
            f.write('    /// instead of "transport"; "peer_id" may be used instead of "peer".\n')
            f.write('    pcl_status_t configureTransport(std::string_view config_json) {\n')
            f.write('        const auto transport = configValue(config_json, "transport");\n')
            f.write('        const auto route = transport.empty()\n')
            f.write('            ? configValue(config_json, "route")\n')
            f.write('            : transport;\n')
            f.write('        if (route.empty() || route == "local") {\n')
            f.write('            return routeAllLocal();\n')
            f.write('        }\n')
            f.write('        if (route == "remote") {\n')
            f.write('            auto peer = configValue(config_json, "peer");\n')
            f.write('            if (peer.empty()) peer = configValue(config_json, "peer_id");\n')
            f.write('            if (peer.empty()) return PCL_ERR_INVALID;\n')
            f.write('            return routeAllRemote(peer);\n')
            f.write('        }\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n\n')

            f.write('private:\n')
            f.write('    static ProvidedHandler& requireHandler(ProvidedHandler* h) {\n')
            f.write('        if (!h) throw std::invalid_argument("ProvidedHandler must not be null");\n')
            f.write('        return *h;\n')
            f.write('    }\n\n')

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
            f.write('    // Adapter that satisfies the existing ServiceHandler ABI by\n')
            f.write('    // forwarding typed requests to the user-supplied ProvidedHandler.\n')
            f.write('    class Bridge final : public ServiceHandler {\n')
            f.write('    public:\n')
            f.write('        explicit Bridge(ProvidedService& owner) : owner_(&owner) {}\n\n')

            for svc_name, rpc in all_rpcs:
                on_name = 'on' + _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                if rpc.server_streaming:
                    frame_t = _cpp_rsp_type(rpc)[len('std::vector<'):-1]
                    send_frame = _rpc_send_stream_frame_func(
                        svc_name, rpc, duplicate_rpc_names)
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)

                    # Unary path on a streaming RPC: forward to the user's
                    # streaming handler with a collecting writer and return
                    # the accumulated vector. The codec dispatcher still
                    # exercises this when the wire-level invoke arrives via
                    # the unary path (e.g. legacy clients).
                    f.write(f'        {_cpp_rsp_type(rpc)}\n')
                    f.write(f'        {handler_name}(const {_cpp_req_type(rpc)}& request) override {{\n')
                    f.write(f'            std::vector<{frame_t}> collected;\n')
                    f.write(f'            auto writer = StreamWriter<{frame_t}>::collecting(&collected);\n')
                    f.write(f'            owner_->handler_->{on_name}(request, std::move(writer));\n')
                    f.write('            return collected;\n')
                    f.write('        }\n')

                    # Streaming path: hand the user a live writer over the
                    # PCL stream context. The user owns end(); if they drop
                    # the writer without ending, the destructor aborts.
                    f.write('        pcl_status_t\n')
                    f.write(f'        {stream_name}(const {_cpp_req_type(rpc)}& request,\n')
                    f.write('                                    pcl_stream_context_t* stream_context,\n')
                    f.write('                                    const char* content_type) override {\n')
                    f.write(f'            StreamWriter<{frame_t}> writer{{\n')
                    f.write('                stream_context,\n')
                    f.write('                content_type ? content_type : kJsonContentType,\n')
                    f.write(f'                &{send_frame}\n')
                    f.write('            };\n')
                    f.write(f'            owner_->handler_->{on_name}(request, std::move(writer));\n')
                    f.write('            return PCL_STREAMING;\n')
                    f.write('        }\n')
                else:
                    f.write(f'        {_cpp_rsp_type(rpc)}\n')
                    f.write(f'        {handler_name}(const {_cpp_req_type(rpc)}& request) override {{\n')
                    f.write(f'            return owner_->handler_->{on_name}(request);\n')
                    f.write('        }\n')
            f.write('    private:\n')
            f.write('        ProvidedService* owner_;\n')
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
            f.write('                                           &ProvidedService::unaryDispatch, &binding);\n')
            f.write('        if (!port) { unary_bindings_.pop_back(); return false; }\n')
            f.write('        ports_.push_back(port);\n')
            f.write('        return true;\n')
            f.write('    }\n\n')

            f.write('    bool addStreamBinding(const char* service_name, ServiceChannel channel) {\n')
            f.write('        stream_bindings_.push_back(StreamBinding{&bridge_, channel, content_type_});\n')
            f.write('        StreamBinding& binding = stream_bindings_.back();\n')
            f.write('        pcl::Port port = host_->addStreamService(service_name, content_type_.c_str(),\n')
            f.write('                                                 &ProvidedService::streamDispatch, &binding);\n')
            f.write('        if (!port) { stream_bindings_.pop_back(); return false; }\n')
            f.write('        ports_.push_back(port);\n')
            f.write('        return true;\n')
            f.write('    }\n\n')

            f.write('    pcl::Component*                   host_     = nullptr;\n')
            f.write('    pcl::Executor*                    executor_ = nullptr;\n')
            f.write('    ProvidedHandler*                  handler_  = nullptr;\n')
            f.write('    std::unique_ptr<ProvidedHandler>  owned_handler_;\n')
            f.write('    std::string                       content_type_;\n')
            f.write('    Bridge                            bridge_;\n')
            f.write('    std::list<UnaryBinding>           unary_bindings_;\n')
            f.write('    std::list<StreamBinding>          stream_bindings_;\n')
            f.write('    std::vector<pcl::Port>            ports_;\n')
            f.write('};\n\n')

            # ---- ConsumedService -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// ConsumedService -- attach to your pcl::Component to call this package\'s\n')
            f.write('// RPCs. Per-RPC entry points are async-shaped:\n')
            f.write('//   * Unary RPCs return std::future<Result<T>>.\n')
            f.write('//   * Streaming RPCs return a StreamHandle and deliver frames via the\n')
            f.write('//     supplied on_frame/on_end callbacks (both fire on the executor\n')
            f.write('//     thread; together they cover the stream lifetime).\n')
            f.write('//\n')
            f.write('// Usage: construct as a member of your component; call routeAllRemote()\n')
            f.write('// (or routeAllLocal()) once the executor transport is up.\n')
            f.write(_SEP + '\n\n')

            f.write('/// \\brief Move-only handle for an in-flight server-streaming RPC.\n')
            f.write('///\n')
            f.write('/// Returned by every <op>Streaming() entry point. Call cancel() to stop\n')
            f.write('/// receiving frames -- the on_frame callback is suppressed from then on,\n')
            f.write('/// the underlying executor stream context is cancelled (the transport may\n')
            f.write('/// notify the server depending on its cancel support), and on_end fires\n')
            f.write('/// with PCL_ERR_CANCELLED once the framework finalises the stream.\n')
            f.write('/// Destroying or overwriting the handle also cancels, but suppresses\n')
            f.write('/// later user callbacks because their captures may be going away.\n')
            f.write('class StreamHandle {\n')
            f.write('public:\n')
            f.write('    StreamHandle() = default;\n')
            f.write('    StreamHandle(StreamHandle&& o) noexcept\n')
            f.write('        : cancel_fn_(std::move(o.cancel_fn_)) {\n')
            f.write('        o.cancel_fn_ = {};\n')
            f.write('    }\n')
            f.write('    StreamHandle& operator=(StreamHandle&& o) noexcept {\n')
            f.write('        if (this != &o) {\n')
            f.write('            cancelImpl(false);\n')
            f.write('            cancel_fn_ = std::move(o.cancel_fn_);\n')
            f.write('            o.cancel_fn_ = {};\n')
            f.write('        }\n')
            f.write('        return *this;\n')
            f.write('    }\n')
            f.write('    StreamHandle(const StreamHandle&) = delete;\n')
            f.write('    StreamHandle& operator=(const StreamHandle&) = delete;\n\n')
            f.write('    ~StreamHandle() { cancelImpl(false); }\n\n')
            f.write('    bool valid() const { return static_cast<bool>(cancel_fn_); }\n')
            f.write('    explicit operator bool() const { return valid(); }\n\n')
            f.write('    /// \\brief Cancel the stream. Idempotent; subsequent calls no-op.\n')
            f.write('    void cancel() {\n')
            f.write('        cancelImpl(true);\n')
            f.write('    }\n\n')
            f.write('private:\n')
            f.write('    void cancelImpl(bool notify_end) {\n')
            f.write('        if (cancel_fn_) {\n')
            f.write('            cancel_fn_(notify_end);\n')
            f.write('            cancel_fn_ = {};\n')
            f.write('        }\n')
            f.write('    }\n\n')
            f.write('    friend class ConsumedService;\n')
            f.write('    explicit StreamHandle(std::function<void(bool)> cancel_fn)\n')
            f.write('        : cancel_fn_(std::move(cancel_fn)) {}\n')
            f.write('    std::function<void(bool)> cancel_fn_;\n')
            f.write('};\n\n')

            f.write('class ConsumedService {\n')
            f.write('public:\n')
            f.write('    ConsumedService(pcl::Component& host,\n')
            f.write('                    pcl::Executor& executor,\n')
            f.write('                    std::string content_type = kJsonContentType)\n')
            f.write('        : host_(&host),\n')
            f.write('          executor_(&executor),\n')
            f.write('          content_type_(std::move(content_type)) {}\n\n')

            f.write('    ConsumedService(const ConsumedService&) = delete;\n')
            f.write('    ConsumedService& operator=(const ConsumedService&) = delete;\n')
            f.write('    ConsumedService(ConsumedService&&) = delete;\n')
            f.write('    ConsumedService& operator=(ConsumedService&&) = delete;\n\n')

            f.write('    /// \\brief Route every consumed endpoint to the executor\'s\n')
            f.write('    ///        default transport. The transport itself picks the peer\n')
            f.write('    ///        (e.g. shared-memory bus discovers the unique provider).\n')
            f.write('    pcl_status_t routeAllRemote() {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                kind = 'PCL_ENDPOINT_STREAM_CONSUMED' if rpc.server_streaming else 'PCL_ENDPOINT_CONSUMED'
                f.write('        if (auto rc = executor_->setEndpointRoute(\n')
                f.write(f'                {svc_const}, {kind},\n')
                f.write('                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Route every consumed endpoint to a named peer transport\n')
            f.write('    ///        previously registered via executor.registerTransport().\n')
            f.write('    pcl_status_t routeAllRemote(std::string_view peer_id) {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                kind = 'PCL_ENDPOINT_STREAM_CONSUMED' if rpc.server_streaming else 'PCL_ENDPOINT_CONSUMED'
                f.write('        if (auto rc = executor_->routeRemote(\n')
                f.write(f'                {svc_const}, peer_id, {kind}); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Route every consumed endpoint to local providers only.\n')
            f.write('    pcl_status_t routeAllLocal() {\n')
            for svc_name, rpc in all_rpcs:
                svc_const = _rpc_service_const(
                    svc_name, rpc, duplicate_rpc_names)
                kind = 'PCL_ENDPOINT_STREAM_CONSUMED' if rpc.server_streaming else 'PCL_ENDPOINT_CONSUMED'
                f.write('        if (auto rc = executor_->routeLocal(\n')
                f.write(f'                {svc_const}, {kind}); rc != PCL_OK) return rc;\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Configure consumed service routes from opaque JSON.\n')
            f.write('    ///\n')
            f.write('    /// Supported shapes are {"transport":"local"},\n')
            f.write('    /// {"transport":"remote"}, and\n')
            f.write('    /// {"transport":"remote","peer":"peer_id"}. "route" may be used\n')
            f.write('    /// instead of "transport"; "peer_id" may be used instead of "peer".\n')
            f.write('    pcl_status_t configureTransport(std::string_view config_json) {\n')
            f.write('        const auto transport = configValue(config_json, "transport");\n')
            f.write('        const auto route = transport.empty()\n')
            f.write('            ? configValue(config_json, "route")\n')
            f.write('            : transport;\n')
            f.write('        if (route.empty() || route == "local") {\n')
            f.write('            return routeAllLocal();\n')
            f.write('        }\n')
            f.write('        if (route == "remote") {\n')
            f.write('            auto peer = configValue(config_json, "peer");\n')
            f.write('            if (peer.empty()) peer = configValue(config_json, "peer_id");\n')
            f.write('            return peer.empty() ? routeAllRemote() : routeAllRemote(peer);\n')
            f.write('        }\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n\n')

            for key in all_topics:
                pascal = _snake_to_pascal(key)
                fname = f'subscribe{pascal}'
                trampoline = f'trampoline{pascal}'
                decode_name = f'decode{pascal}'
                spec = self._topics.spec(key)
                payload_t = spec.cpp_payload_type
                f.write(f'    pcl_port_t* {fname}(\n')
                f.write(f'        std::function<void(const {payload_t}&)> on_message) {{\n')
                f.write('        auto callback =\n')
                f.write(f'            std::make_shared<std::function<void(const {payload_t}&)>>(\n')
                f.write('                std::move(on_message));\n')
                f.write('        topic_callbacks_.push_back(callback);\n')
                f.write('        pcl_port_t* port =\n')
                f.write(f'            ::{full_ns}::{fname}(\n')
                f.write(f'                host_->handle(), &ConsumedService::{trampoline},\n')
                # A port's declared type is its wire schema identity.  The
                # encoded pcl_msg_t remains tagged with content_type_ for
                # codec dispatch, but remote PUBSUB transports need this name
                # when subscribing to a typed bus such as OMS/LA-CAL.
                f.write(f'                callback.get(), "{spec.short_type}");\n')
                f.write('        if (!port) {\n')
                f.write('            topic_callbacks_.pop_back();\n')
                f.write('        } else {\n')
                f.write('            topic_subscriptions_.push_back(\n')
                f.write(f'                pcl::Port{{port, "{spec.short_type}"}});\n')
                f.write('        }\n')
                f.write('        return port;\n')
                f.write('    }\n\n')

            for key in all_topics:
                pascal = _snake_to_pascal(key)
                add_name = f'add{pascal}Publisher'
                publish_name = f'publish{pascal}'
                helper_name = f'::{full_ns}::{publish_name}'
                topic_const = f'kTopic{pascal}'
                port_member = f'pub_{key}_'
                spec = self._topics.spec(key)
                f.write(f'    pcl_status_t {add_name}() {{\n')
                f.write(f'        {port_member} = host_->addPublisher(\n')
                f.write(f'            {topic_const}, "{spec.short_type}");\n')
                f.write(f'        return {port_member} ? PCL_OK : PCL_ERR_NOMEM;\n')
                f.write('    }\n\n')
                f.write(f'    pcl_status_t {publish_name}(\n')
                f.write(f'        const {spec.cpp_payload_type}& payload) {{\n')
                f.write(f'        return {helper_name}(\n')
                f.write(f'            {port_member}.handle(), payload, content_type_.c_str());\n')
                f.write('    }\n\n')
                f.write(f'    pcl_status_t {publish_name}(\n')
                f.write('        const std::string& payload) {\n')
                f.write(f'        return {helper_name}(\n')
                f.write(f'            {port_member}.handle(), payload, content_type_.c_str());\n')
                f.write('    }\n\n')

            f.write('    pcl_status_t routeAllPublishersLocal() {\n')
            for key in all_topics:
                f.write(f'        if ({f"pub_{key}_"}) {{\n')
                f.write(f'            if (auto rc = {f"pub_{key}_"}.routeLocal(); rc != PCL_OK) return rc;\n')
                f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    pcl_status_t routeAllPublishersRemote(std::string_view peer_id) {\n')
            for key in all_topics:
                f.write(f'        if ({f"pub_{key}_"}) {{\n')
                f.write(f'            if (auto rc = {f"pub_{key}_"}.routeRemote(peer_id); rc != PCL_OK) return rc;\n')
                f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    pcl_status_t routeAllSubscribersLocal() {\n')
            f.write('        for (const auto& port : topic_subscriptions_) {\n')
            f.write('            if (auto rc = port.routeLocal(); rc != PCL_OK) return rc;\n')
            f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    pcl_status_t routeAllSubscribersRemote(std::string_view peer_id) {\n')
            f.write('        for (const auto& port : topic_subscriptions_) {\n')
            f.write('            if (auto rc = port.routeRemote(peer_id); rc != PCL_OK) return rc;\n')
            f.write('        }\n')
            f.write('        return PCL_OK;\n')
            f.write('    }\n\n')

            f.write('    /// \\brief Configure generated topic ports from opaque JSON.\n')
            f.write('    ///\n')
            f.write('    /// Supported shapes match configureTransport(). Publishers and\n')
            f.write('    /// subscribers that have not been created yet are ignored.\n')
            f.write('    pcl_status_t configurePubSubTransport(std::string_view config_json) {\n')
            f.write('        const auto transport = configValue(config_json, "transport");\n')
            f.write('        const auto route = transport.empty()\n')
            f.write('            ? configValue(config_json, "route")\n')
            f.write('            : transport;\n')
            f.write('        if (route.empty() || route == "local") {\n')
            f.write('            if (auto rc = routeAllPublishersLocal(); rc != PCL_OK) return rc;\n')
            f.write('            return routeAllSubscribersLocal();\n')
            f.write('        }\n')
            f.write('        if (route == "remote") {\n')
            f.write('            auto peer = configValue(config_json, "peer");\n')
            f.write('            if (peer.empty()) peer = configValue(config_json, "peer_id");\n')
            f.write('            if (peer.empty()) return PCL_ERR_INVALID;\n')
            f.write('            if (auto rc = routeAllPublishersRemote(peer); rc != PCL_OK) return rc;\n')
            f.write('            return routeAllSubscribersRemote(peer);\n')
            f.write('        }\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n\n')

            for svc_name, rpc in all_rpcs:
                base = _rpc_symbol_base(svc_name, rpc, duplicate_rpc_names)
                async_name = _lc_first(base) + 'Async'
                invoke_fn = _rpc_invoke_func(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = _cpp_req_type(rpc)
                if rpc.server_streaming:
                    frame_t = _cpp_rsp_type(rpc)[len('std::vector<'):-1]
                    invoke_stream = _rpc_invoke_stream_func(
                        svc_name, rpc, duplicate_rpc_names)
                    decode_frame = _rpc_decode_stream_frame_func(
                        svc_name, rpc, duplicate_rpc_names)
                    streaming_name = _lc_first(base) + 'Streaming'

                    # Push-mode streaming variant -- callbacks fire on the
                    # executor thread and together cover the stream lifetime.
                    f.write('    StreamHandle\n')
                    f.write(f'    {streaming_name}(const {req_t}& request,\n')
                    f.write(f'                std::function<void(const {frame_t}&)> on_frame,\n')
                    f.write('                std::function<void(pcl_status_t)> on_end = {}) {\n')
                    f.write(f'        auto push = std::make_shared<StreamPushState<{frame_t}>>();\n')
                    f.write('        push->on_frame  = std::move(on_frame);\n')
                    f.write('        push->on_end    = std::move(on_end);\n')
                    f.write(f'        push->decoder   = [](const pcl_msg_t* msg, {frame_t}* out) {{\n')
                    f.write(f'            return {decode_frame}(msg, out);\n')
                    f.write('        };\n')
                    f.write(f'        auto holder = std::make_unique<StreamPushHolder<{frame_t}>>(\n')
                    f.write(f'            StreamPushHolder<{frame_t}>{{push}});\n')
                    f.write('        pcl_stream_context_t* ctx_handle = nullptr;\n')
                    f.write(f'        const pcl_status_t rc = {invoke_stream}(\n')
                    f.write('            executor_->handle(), request,\n')
                    f.write(f'            &StreamPushState<{frame_t}>::trampoline, holder.get(),\n')
                    f.write('            &ctx_handle, nullptr, content_type_.c_str());\n')
                    f.write('        if (rc != PCL_OK && rc != PCL_STREAMING) {\n')
                    f.write('            return StreamHandle{};\n')
                    f.write('        }\n')
                    f.write('        (void)holder.release();\n')
                    f.write('        return StreamHandle{[push, ctx_handle](bool notify_end) {\n')
                    f.write('            push->cancelled = true;\n')
                    f.write('            if (!notify_end) {\n')
                    f.write('                push->on_frame = {};\n')
                    f.write('                push->on_end = {};\n')
                    f.write('            }\n')
                    f.write('            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);\n')
                    f.write('        }};\n')
                    f.write('    }\n\n')
                else:
                    rsp_t = _cpp_rsp_type(rpc)
                    decode_resp = _rpc_decode_response_func(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write(f'    std::future<Result<{rsp_t}>>\n')
                    f.write(f'    {async_name}(const {req_t}& request) {{\n')
                    f.write(f'        auto state = std::make_shared<UnaryState<{rsp_t}>>();\n')
                    f.write('        auto future = state->promise.get_future();\n')
                    f.write('        state->decoder = [](const pcl_msg_t* msg, '
                            f'{rsp_t}* out) {{\n')
                    f.write(f'            return {decode_resp}(msg, out);\n')
                    f.write('        };\n')
                    f.write(f'        auto holder = std::make_unique<UnaryHolder<{rsp_t}>>(UnaryHolder<{rsp_t}>{{state}});\n')
                    f.write(f'        const pcl_status_t rc = {invoke_fn}(\n')
                    f.write('            executor_->handle(), request,\n')
                    f.write(f'            &UnaryState<{rsp_t}>::trampoline,\n')
                    f.write('            holder.get(),\n')
                    f.write('            nullptr, content_type_.c_str());\n')
                    f.write('        if (rc == PCL_OK) {\n')
                    f.write('            (void)holder.release();\n')
                    f.write('        } else {\n')
                    f.write('            state->promise.set_value({rc, {}});\n')
                    f.write('        }\n')
                    f.write('        return future;\n')
                    f.write('    }\n\n')

            # State templates -------------------------------------------------
            f.write('private:\n')
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

            f.write('    template <class T>\n')
            f.write('    struct UnaryState {\n')
            f.write('        std::promise<Result<T>>                       promise;\n')
            f.write('        std::function<bool(const pcl_msg_t*, T*)>     decoder;\n\n')
            f.write('        static void trampoline(const pcl_msg_t* msg, void* user_data);\n')
            f.write('    };\n\n')
            f.write('    template <class T>\n')
            f.write('    struct UnaryHolderT { std::shared_ptr<UnaryState<T>> state; };\n')
            f.write('    template <class T> using UnaryHolder = UnaryHolderT<T>;\n\n')

            f.write('    template <class T>\n')
            f.write('    struct StreamPushState {\n')
            f.write('        std::function<void(const T&)>                 on_frame;\n')
            f.write('        std::function<void(pcl_status_t)>             on_end;\n')
            f.write('        std::function<bool(const pcl_msg_t*, T*)>     decoder;\n')
            f.write('        bool                                          cancelled = false;\n\n')
            f.write('        bool                                          closed = false;\n\n')
            f.write('        static void trampoline(const pcl_msg_t* msg,\n')
            f.write('                                bool             end,\n')
            f.write('                                pcl_status_t     status,\n')
            f.write('                                void*            user_data);\n')
            f.write('    };\n\n')
            f.write('    template <class T>\n')
            f.write('    struct StreamPushHolderT { std::shared_ptr<StreamPushState<T>> state; };\n')
            f.write('    template <class T> using StreamPushHolder = StreamPushHolderT<T>;\n\n')

            for key in all_topics:
                pascal = _snake_to_pascal(key)
                trampoline = f'trampoline{pascal}'
                decode_name = f'decode{pascal}'
                payload_t = self._topics.spec(key).cpp_payload_type
                f.write(f'    static void {trampoline}(pcl_container_t*,\n')
                f.write('                           const pcl_msg_t* msg,\n')
                f.write('                           void* user_data) {\n')
                f.write('        auto* callback =\n')
                f.write(f'            static_cast<std::function<void(const {payload_t}&)>*>(\n')
                f.write('                user_data);\n')
                f.write('        if (!callback || !*callback) return;\n')
                f.write(f'        {payload_t} payload{{}};\n')
                f.write(f'        if ({decode_name}(msg, &payload)) {{\n')
                f.write('            (*callback)(payload);\n')
                f.write('        }\n')
                f.write('    }\n\n')

            f.write('    pcl::Component* host_     = nullptr;\n')
            f.write('    pcl::Executor*  executor_ = nullptr;\n')
            f.write('    std::string     content_type_;\n')
            if all_topics:
                f.write('    std::vector<std::shared_ptr<void>> topic_callbacks_;\n')
            f.write('    std::vector<pcl::Port> topic_subscriptions_;\n')
            for key in all_topics:
                f.write(f'    pcl::Port pub_{key}_;\n')
            f.write('};\n\n')

            # Template member-fn definitions outside the class --------------
            f.write('template <class T>\n')
            f.write('inline void ConsumedService::UnaryState<T>::trampoline(\n')
            f.write('        const pcl_msg_t* msg, void* user_data) {\n')
            f.write('    std::unique_ptr<ConsumedService::UnaryHolder<T>> holder(\n')
            f.write('        static_cast<ConsumedService::UnaryHolder<T>*>(user_data));\n')
            f.write('    if (!holder || !holder->state) return;\n')
            f.write('    auto& state = *holder->state;\n')
            f.write('    Result<T> result{};\n')
            f.write('    if (state.decoder && state.decoder(msg, &result.value)) {\n')
            f.write('        result.status = PCL_OK;\n')
            f.write('    } else {\n')
            f.write('        result.status = PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    state.promise.set_value(std::move(result));\n')
            f.write('}\n\n')

            f.write('template <class T>\n')
            f.write('inline void ConsumedService::StreamPushState<T>::trampoline(\n')
            f.write('        const pcl_msg_t* msg, bool end, pcl_status_t status,\n')
            f.write('        void* user_data) {\n')
            f.write('    auto* holder = static_cast<ConsumedService::StreamPushHolder<T>*>(\n')
            f.write('        user_data);\n')
            f.write('    if (!holder || !holder->state) {\n')
            f.write('        if (end) delete holder;\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    auto& state = *holder->state;\n')
            f.write('    if (end) {\n')
            f.write('        state.closed = true;\n')
            f.write('        const pcl_status_t end_status = state.cancelled\n')
            f.write('            ? PCL_ERR_CANCELLED\n')
            f.write('            : status;\n')
            f.write('        if (state.on_end) state.on_end(end_status);\n')
            f.write('        delete holder;\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    if (state.cancelled || status != PCL_OK) return;\n')
            f.write('    T frame{};\n')
            f.write('    if (state.decoder && state.decoder(msg, &frame) && state.on_frame) {\n')
            f.write('        state.on_frame(frame);\n')
            f.write('    }\n')
            f.write('}\n\n')

            self._write_interaction_facade(
                f, full_ns, parsed, all_topics, is_provided, duplicate_rpc_names)

            f.write(f'}} // namespace {full_ns}\n')

