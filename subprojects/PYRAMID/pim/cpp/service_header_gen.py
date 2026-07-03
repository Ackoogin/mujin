#!/usr/bin/env python3
"""Service binding header (.hpp) emitter for CppServiceGenerator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import Dict, List, Tuple

from proto_parser import (
    ProtoFile,
    ProtoRpc,
    snake_to_pascal as _snake_to_pascal,
)
from proto_resolve import (
    _DATA_MODEL_PROTO_ROOT,
    _qualified_package_for_type,
)
from .naming import (
    _SEP,
    _DEFAULT_CONTENT_TYPE,
    _topic_key_to_phrase,
    _cpp_qos_reliability_expr,
    _short_type,
    _service_wire_prefix,
    _duplicate_rpc_names,
    _rpc_handler_name,
    _rpc_enum_value,
    _rpc_service_const,
    _rpc_invoke_func,
    _rpc_decode_response_func,
    _rpc_stream_handler_name,
    _rpc_encode_stream_frame_func,
    _rpc_decode_stream_frame_func,
    _rpc_send_stream_frame_func,
    _rpc_invoke_stream_func,
    _crud_rpcs,
    _rpc_wire_name,
    _cpp_req_type,
    _cpp_rsp_type,
    _cpp_ns_for_proto_package,
    _legacy_service_namespace,
    _is_provided,
)


class ServiceHeaderEmitterMixin:
    """Service constants/topics, handler surface, client invoke and
    ROS2 bind declarations of the generated binding header."""

    # -- Header (.hpp) ---------------------------------------------------------

    def _write_header(self, path: Path, full_ns: str, types_ns: str,
                      types_header: str, parsed: ProtoFile,
                      all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        has_grpc = self._has_backend('grpc')
        has_ros2 = self._has_backend('ros2')
        legacy_svc_base_ns, _legacy_prefix = _legacy_service_namespace(parsed.package)
        legacy_role = parsed.package.split('.')[-1] if parsed.package else 'provided'
        legacy_full_ns = legacy_svc_base_ns + '::' + legacy_role
        sub_topics, pub_topics = self._topics.topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)

        # Collect raw type names (no BASE_TYPE_MAP) for using declarations
        # Map each referenced short type name to its proto FQN so the using
        # declarations can be qualified by the type's real (sub-)namespace
        # instead of relying on a flat re-export.
        raw_type_fqn: Dict[str, str] = {}
        for _, rpc in all_rpcs:
            raw_type_fqn.setdefault(_short_type(rpc.request_type), rpc.request_type)
            raw_type_fqn.setdefault(_short_type(rpc.response_type), rpc.response_type)
        for key in topic_set:
            spec = self._topics.spec(key)
            raw_type_fqn.setdefault(spec.short_type, spec.full_type)
        raw_types = sorted(raw_type_fqn)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            # File-level comment block
            f.write('// Auto-generated service binding header\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('//\n')
            f.write('// Architecture: component logic > service binding (this) > PCL\n')
            f.write('//\n')
            f.write('// This header provides:\n')
            f.write('//   1. Wire-name constants and topic constants\n')
            f.write('//   2. EntityActions handler base class'
                    ' (ServiceHandler \u2014 override Handle*)\n')
            f.write('//   3. PCL binding functions (subscribe*, publish*, invoke*)\n')
            f.write('//   4. Content-type support metadata and msgToString utility\n')
            f.write('#pragma once\n\n')

            # Includes
            f.write(f'#include "{types_header}"\n')
            # Service-local wrapper messages (and Empty) live in this contract's
            # own component types header.
            has_local_types = any(
                _qualified_package_for_type(raw_type_fqn.get(t, '')) == parsed.package
                or raw_type_fqn.get(t, '') == 'google.protobuf.Empty'
                for t in raw_types
            )
            if has_local_types:
                f.write(f'#include "{parsed.package.replace(".", "_")}_types.hpp"\n')
            f.write('\n')
            f.write('#include <pcl/pcl_container.h>\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_transport.h>\n')
            f.write('#include <pcl/pcl_types.h>\n\n')
            if has_ros2:
                f.write('#include "ros2/cpp/pyramid_ros2_transport_support.hpp"\n\n')
            if has_grpc:
                f.write('#include <memory>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

            # ---- Content-type constants --------------------------------------
            f.write(_SEP + '\n')
            f.write('// Content-type constants and support metadata\n')
            f.write(_SEP + '\n\n')
            f.write('constexpr const char* kJsonContentType = "application/json";\n')
            if self._has_backend('flatbuffers'):
                f.write('constexpr const char* kFlatBuffersContentType = "application/flatbuffers";\n')
            if self._has_backend('protobuf') or has_grpc:
                f.write('constexpr const char* kProtobufContentType = "application/protobuf";\n')
            f.write('\n')
            f.write('bool supportsContentType(const char* content_type);\n')
            f.write('std::vector<const char*> supportedContentTypes();\n\n')

            # ---- Service wire-name constants ----------------------------------
            f.write(_SEP + '\n')
            f.write('// Service wire-name constants (generated from proto)\n')
            f.write(_SEP + '\n\n')

            # Align '=' across all constant declarations
            const_names = [
                _rpc_service_const(svc.name, rpc, duplicate_rpc_names)
                for svc in parsed.services
                for rpc in _crud_rpcs(svc)
            ]
            max_const = max((len(n) for n in const_names), default=0)

            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    wire = f'{_service_wire_prefix(svc.name)}.{_rpc_wire_name(rpc)}'
                    service_const = _rpc_service_const(
                        svc.name, rpc, duplicate_rpc_names)
                    pad = max_const - len(service_const)
                    f.write(f'constexpr const char* {service_const}'
                            f'{" " * pad}  = "{wire}";\n')
            f.write('\n')

            # ---- Topic constants ---------------------------------------------
            if topic_set:
                f.write(_SEP + '\n')
                f.write('// Standard topic name constants\n')
                f.write(_SEP + '\n\n')
                topic_consts = [f'kTopic{_snake_to_pascal(k)}' for k in topic_set]
                max_topic = max((len(n) for n in topic_consts), default=0)
                # Only pad when there are multiple constants to align
                multi = len(topic_set) > 1
                for key, wire in topic_set.items():
                    cname = f'kTopic{_snake_to_pascal(key)}'
                    if multi:
                        pad = max_topic - len(cname)
                        f.write(f'constexpr const char* {cname}'
                                f'{" " * pad}  = "{wire}";\n')
                    else:
                        f.write(f'constexpr const char* {cname} = "{wire}";\n')
                f.write('\n')
                qos_keys = [
                    key for key in topic_set
                    if self._topics.spec(key).has_qos
                ]
                if qos_keys:
                    for key in qos_keys:
                        pascal = _snake_to_pascal(key)
                        spec = self._topics.spec(key)
                        f.write(f'inline constexpr pcl_qos_t kTopic{pascal}Qos = '
                                f'{{{_cpp_qos_reliability_expr(spec)}}};\n')
                    f.write('\n')

            # ---- ServiceChannel enum -----------------------------------------
            f.write(_SEP + '\n')
            f.write('// Service channel discriminant\n')
            f.write(_SEP + '\n\n')
            f.write('enum class ServiceChannel {\n')
            for svc_name, rpc in all_rpcs:
                f.write(
                    f'    {_rpc_enum_value(svc_name, rpc, duplicate_rpc_names)},\n')
            f.write('};\n\n')

            # ---- msgToString -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL message utility\n')
            f.write(_SEP + '\n\n')
            f.write('/// \\brief Convert a raw PCL message buffer to a std::string.\n')
            f.write('std::string msgToString(const void* data, unsigned size);\n\n')

            # ---- ServiceHandler base class -----------------------------------
            f.write(_SEP + '\n')
            f.write('// EntityActions handler base class\n')
            f.write('//\n')
            f.write('// Subclass and override the handle* methods to implement'
                    ' business logic.\n')
            f.write('// Default implementations return empty / null values'
                    ' (stub behaviour).\n')
            f.write(_SEP + '\n\n')

            # Data-model RPC/topic types are imported from their package
            # sub-namespace. Service-local wrapper messages and Empty are
            # defined in this contract's own namespace (== full_ns, supplied by
            # the included component types header) and need no using-declaration.
            for t in raw_types:
                fqn = raw_type_fqn.get(t, '')
                pkg = _qualified_package_for_type(fqn)
                if (pkg == _DATA_MODEL_PROTO_ROOT
                        or pkg.startswith(_DATA_MODEL_PROTO_ROOT + '.')):
                    f.write(f'using {_cpp_ns_for_proto_package(pkg)}::{t};\n')
            f.write('\n')

            f.write('class ServiceHandler {\n')
            f.write('public:\n')
            f.write('    virtual ~ServiceHandler() = default;\n')

            current_svc = None
            for i, (svc_name, rpc) in enumerate(all_rpcs):
                if svc_name != current_svc:
                    f.write(f'\n    // {svc_name}\n')
                    current_svc = svc_name
                f.write(f'    virtual {_cpp_rsp_type(rpc)}\n')
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                f.write(
                    f'    {handler_name}(const {_cpp_req_type(rpc)}& request);\n')
                if rpc.server_streaming:
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write('\n')
                    f.write('    /// \\brief Begin an asynchronous stream for this RPC.\n')
                    f.write('    ///\n')
                    f.write('    /// Override this for true server streaming. Store stream_context,\n')
                    f.write('    /// return PCL_STREAMING, then emit frames with send*StreamFrame().\n')
                    f.write('    virtual pcl_status_t\n')
                    f.write(f'    {stream_name}(const {_cpp_req_type(rpc)}& request,\n')
                    f.write('    ' + ' ' * len(stream_name)
                            + ' pcl_stream_context_t* stream_context,\n')
                    f.write('    ' + ' ' * len(stream_name)
                            + ' const char* content_type);\n')
                # Blank line after method unless: last method, or next method
                # is a new service section (its leading \n already provides spacing)
                is_last = i == len(all_rpcs) - 1
                next_svc_change = (not is_last
                                   and all_rpcs[i + 1][0] != svc_name)
                if not is_last and not next_svc_change:
                    f.write('\n')

            f.write('};\n\n')

            # ---- PCL binding functions ---------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL binding functions \u2014'
                    ' Subscribe / Publish / Invoke (typed)\n')
            f.write(_SEP + '\n\n')

            # Subscribe helpers (unchanged -- PCL registration is content_type
            # aware but not typed, since the callback receives raw pcl_msg_t)
            for key, _wire in topic_set.items():
                pascal = _snake_to_pascal(key)
                fname = f'subscribe{pascal}'
                cname = f'kTopic{pascal}'
                col = len(f'pcl_port_t* {fname}(')
                sp = ' ' * col
                brief = (f'/// \\brief Subscribe to'
                         f' {_topic_key_to_phrase(key)} publications on'
                         f' {cname}.')
                if len(brief) > 80:
                    split = brief.rfind(f' {cname}.')
                    f.write(brief[:split] + '\n')
                    f.write(f'///        {cname}.\n')
                else:
                    f.write(brief + '\n')
                f.write(f'pcl_port_t* {fname}(pcl_container_t*  container,\n')
                f.write(f'{sp}pcl_sub_callback_t callback,\n')
                f.write(f'{sp}void*             user_data = nullptr,\n')
                f.write(f'{sp}const char*       content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

            # Typed publish helpers.  These are generated for every topic in
            # both provider and consumer namespaces so components on either side
            # can use the generated binding as their only codec facade.
            for key, _wire in topic_set.items():
                pascal = _snake_to_pascal(key)
                fname = f'publish{pascal}'
                cname = f'kTopic{pascal}'
                col = 13 + len(fname) + 1
                sp = ' ' * col
                spec = self._topics.spec(key)
                wire_decl_t = spec.cpp_payload_type
                f.write(f'/// \\brief Publish a typed message on'
                        f' {cname}.\n')
                f.write('///\n')
                f.write(f'/// \\p publisher must be the pcl_port_t*'
                        f' returned by addPublisher for\n')
                f.write(f'/// {cname}, obtained during on_configure.\n')
                f.write(f'pcl_status_t {fname}'
                        f'(pcl_port_t*        publisher,\n')
                f.write(f'{sp}const {wire_decl_t}& payload,\n')
                f.write(f'{sp}const char*        content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n')
                f.write('\n')
                f.write(f'pcl_status_t {fname}'
                        f'(pcl_port_t*        publisher,\n')
                f.write(f'{sp}const std::string& payload,\n')
                f.write(f'{sp}const char*'
                        f'        content_type'
                        f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

                encode_name = f'encode{pascal}'
                col = len(f'bool {encode_name}(')
                sp = ' ' * col
                f.write(f'/// \\brief Encode a typed message for {cname}.\n')
                f.write(f'bool {encode_name}(const {wire_decl_t}& payload,\n')
                f.write(f'{sp}const char*        content_type,\n')
                f.write(f'{sp}std::string*       out);\n\n')

                decode_name = f'decode{pascal}'
                col = len(f'bool {decode_name}(')
                sp = ' ' * col
                f.write(f'/// \\brief Decode a PCL message from {cname}.\n')
                f.write(f'bool {decode_name}(const pcl_msg_t* msg,\n')
                f.write(f'{sp}{wire_decl_t}* out);\n\n')

            # Typed invoke helpers
            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    wire_full = f'{_service_wire_prefix(svc.name)}.{_rpc_wire_name(rpc)}'
                    rsp_decl_t = _cpp_rsp_type(rpc)
                    decode_func = _rpc_decode_response_func(
                        svc.name, rpc, duplicate_rpc_names)
                    invoke_func = _rpc_invoke_func(
                        svc.name, rpc, duplicate_rpc_names)
                    decode_col = len(f'bool {decode_func}(')
                    decode_sp = ' ' * decode_col
                    f.write(f'/// \\brief Decode a response from {wire_full}.\n')
                    f.write(f'bool {decode_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{decode_sp}{rsp_decl_t}* out);\n\n')
                    if rpc.server_streaming:
                        frame_t = _cpp_rsp_type(rpc)[len('std::vector<'):-1]
                        encode_frame = _rpc_encode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        decode_frame = _rpc_decode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        send_frame = _rpc_send_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        f.write(f'/// \\brief Encode one stream frame for {wire_full}.\n')
                        col = len(f'bool {encode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {encode_frame}(const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type,\n')
                        f.write(f'{sp}std::string*       out);\n\n')
                        f.write(f'/// \\brief Decode one stream frame from {wire_full}.\n')
                        col = len(f'bool {decode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {decode_frame}(const pcl_msg_t* msg,\n')
                        f.write(f'{sp}{frame_t}* out);\n\n')
                        f.write(f'/// \\brief Send one typed stream frame for {wire_full}.\n')
                        col = 13 + len(send_frame) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {send_frame}'
                                f'(pcl_stream_context_t* stream_context,\n')
                        f.write(f'{sp}const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type'
                                f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')
                    req_decl_t = _cpp_req_type(rpc)
                    col = 13 + len(invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'/// \\brief Invoke {wire_full}'
                            f' (typed, serialisation handled internally).\n')
                    f.write(f'///\n')
                    f.write(f'/// Uses the configured endpoint route, or the legacy\n')
                    f.write(f'/// executor transport fallback when no route is supplied.\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}pcl_resp_cb_fn_t'
                            f'        callback,\n')
                    f.write(f'{sp}void*'
                            f'                   user_data'
                            f' = nullptr,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route'
                            f' = nullptr,\n')
                    f.write(f'{sp}const char*       content_type'
                            f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')
                    f.write(f'/// \\brief Invoke {wire_full} and ignore the async response.\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type'
                            f' = "{_DEFAULT_CONTENT_TYPE}",\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route'
                            f' = nullptr);\n\n')
                    if rpc.server_streaming:
                        invoke_stream = _rpc_invoke_stream_func(
                            svc.name, rpc, duplicate_rpc_names)
                        col = 13 + len(invoke_stream) + 1
                        sp = ' ' * col
                        f.write(f'/// \\brief Invoke {wire_full} as an asynchronous stream.\n')
                        f.write(f'pcl_status_t {invoke_stream}'
                                f'(pcl_executor_t* executor,\n')
                        f.write(f'{sp}const {req_decl_t}&'
                                f'{" " * max(1, 22 - len(req_decl_t))}'
                                f'request,\n')
                        f.write(f'{sp}pcl_stream_msg_fn_t'
                                f'   callback,\n')
                        f.write(f'{sp}void*'
                                f'                   user_data'
                                f' = nullptr,\n')
                        f.write(f'{sp}pcl_stream_context_t** out_context'
                                f' = nullptr,\n')
                        f.write(f'{sp}const pcl_endpoint_route_t* route'
                                f' = nullptr,\n')
                        f.write(f'{sp}const char*       content_type'
                                f' = "{_DEFAULT_CONTENT_TYPE}");\n\n')

            if has_grpc:
                f.write(_SEP + '\n')
                f.write('// gRPC binding startup hook\n')
                f.write(_SEP + '\n\n')
                f.write('class GrpcServer {\n')
                f.write('public:\n')
                f.write('    GrpcServer();\n')
                f.write('    GrpcServer(GrpcServer&&) noexcept;\n')
                f.write('    GrpcServer& operator=(GrpcServer&&) noexcept;\n')
                f.write('    GrpcServer(const GrpcServer&) = delete;\n')
                f.write('    GrpcServer& operator=(const GrpcServer&) = delete;\n')
                f.write('    ~GrpcServer();\n\n')
                f.write('    bool started() const;\n')
                f.write('    explicit operator bool() const { return started(); }\n')
                f.write('    void wait();\n')
                f.write('    void shutdown();\n\n')
                f.write('private:\n')
                f.write('    struct Impl;\n')
                f.write('    explicit GrpcServer(std::unique_ptr<Impl> impl);\n')
                f.write('    std::unique_ptr<Impl> impl_;\n')
                f.write('    friend GrpcServer buildGrpcServer(const std::string& listen_address,\n')
                f.write('                                      pcl_executor_t* executor);\n')
                f.write('};\n\n')
                f.write('/// \\brief Start generated gRPC ingress endpoints on a PCL executor.\n')
                f.write('GrpcServer buildGrpcServer(const std::string& listen_address,\n')
                f.write('                           pcl_executor_t* executor);\n\n')

            if has_ros2:
                f.write(_SEP + '\n')
                f.write('// ROS2 binding startup hook\n')
                f.write(_SEP + '\n\n')
                f.write('/// \\brief Bind generated ROS2 ingress endpoints to the executor.\n')
                f.write('inline void bindRos2(pyramid::transport::ros2::Adapter& adapter,\n')
                f.write('                     pcl_executor_t* executor)\n')
                f.write('{\n')
                if sub_topics:
                    for key in sub_topics:
                        cname = f'kTopic{_snake_to_pascal(key)}'
                        if self._topics.spec(key).has_qos:
                            qname = f'kTopic{_snake_to_pascal(key)}Qos'
                            f.write('    pyramid::transport::ros2::bindTopicIngress'
                                    f'(adapter, executor, {cname}, {qname});\n')
                        else:
                            f.write('    pyramid::transport::ros2::bindTopicIngress'
                                    f'(adapter, executor, {cname});\n')
                for svc in parsed.services:
                    for rpc in _crud_rpcs(svc):
                        bind_func = ('bindStreamServiceIngress'
                                     if rpc.server_streaming
                                     else 'bindUnaryServiceIngress')
                        service_const = _rpc_service_const(
                            svc.name, rpc, duplicate_rpc_names)
                        f.write(f'    pyramid::transport::ros2::{bind_func}'
                                f'(adapter, executor, {service_const});\n')
                f.write('}\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Dispatch -- deserialises request, calls handler,'
                    ' serialises response.\n')
            f.write('//\n')
            f.write('// Response buffer is heap-allocated via std::malloc;'
                    ' caller frees with std::free.\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceHandler& handler,\n')
            f.write('              ServiceChannel  channel,\n')
            f.write('              const void*     request_buf,\n')
            f.write('              size_t          request_size,\n')
            f.write('              const char*     content_type,\n')
            f.write('              void**          response_buf,\n')
            f.write('              size_t*         response_size);\n\n')
            f.write('inline void dispatch(ServiceHandler& handler,\n')
            f.write('                     ServiceChannel  channel,\n')
            f.write('                     const void*     request_buf,\n')
            f.write('                     size_t          request_size,\n')
            f.write('                     void**          response_buf,\n')
            f.write('                     size_t*         response_size)\n')
            f.write('{\n')
            f.write(f'    dispatch(handler, channel, request_buf, request_size, "{_DEFAULT_CONTENT_TYPE}", response_buf, response_size);\n')
            f.write('}\n\n')
            f.write('/// \\brief Dispatch a server-streaming service request.\n')
            f.write('pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                            ServiceChannel  channel,\n')
            f.write('                            const void*     request_buf,\n')
            f.write('                            size_t          request_size,\n')
            f.write('                            const char*     content_type,\n')
            f.write('                            pcl_stream_context_t* stream_context);\n\n')
            f.write('inline pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                                   ServiceChannel  channel,\n')
            f.write('                                   const void*     request_buf,\n')
            f.write('                                   size_t          request_size,\n')
            f.write('                                   pcl_stream_context_t* stream_context)\n')
            f.write('{\n')
            f.write(f'    return dispatchStream(handler, channel, request_buf, request_size, "{_DEFAULT_CONTENT_TYPE}", stream_context);\n')
            f.write('}\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')
            if legacy_full_ns != full_ns:
                legacy_parts = legacy_full_ns.split('::')
                alias_name = legacy_parts[-1]
                alias_parent = '::'.join(legacy_parts[:-1])
                f.write('\n')
                f.write('// Legacy service namespace alias for existing callers.\n')
                f.write(f'namespace {alias_parent} {{\n')
                f.write(f'namespace {alias_name} = ::{full_ns};\n')
                f.write(f'}} // namespace {alias_parent}\n')

