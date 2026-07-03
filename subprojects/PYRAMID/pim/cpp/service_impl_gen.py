#!/usr/bin/env python3
"""Service binding implementation (.cpp) emitter for CppServiceGenerator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import List, Tuple

from proto_parser import (
    ProtoFile,
    ProtoRpc,
    snake_to_pascal as _snake_to_pascal,
)
from .naming import (
    _SEP,
    _short_type,
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
    _cpp_req_type,
    _cpp_rsp_type,
    _DATA_MODEL_TYPES_NS,
    _namespace_from_proto,
    _is_provided,
    _c_struct_for_type,
    _native_namespace_for_type,
    _service_codec_imports,
)


class ServiceImplEmitterMixin:
    """Dispatch/handler, codec, client invoke and ROS2 bind emission
    of the generated binding implementation."""

    # -- Implementation (.cpp) -------------------------------------------------

    def _write_impl(self, path: Path, file_prefix: str, full_ns: str,
                    types_ns: str, parsed: ProtoFile,
                    all_rpcs: List[Tuple[str, ProtoRpc]]):
        is_provided = _is_provided(parsed)
        svc_base_ns = _namespace_from_proto(parsed)[2]
        has_flatbuffers = self._has_backend('flatbuffers')
        has_protobuf = (
            (self._has_backend('protobuf') or self._has_backend('grpc'))
            and self._service_protobuf_codec_available(svc_base_ns)
        )
        flatbuffers_codec_ns = svc_base_ns + '::flatbuffers_codec'
        flatbuffers_codec_header = 'flatbuffers/cpp/' + '_'.join(svc_base_ns.split('::')) + '_flatbuffers_codec.hpp'
        protobuf_codec_ns = svc_base_ns + '::protobuf_codec'
        protobuf_codec_header = '_'.join(svc_base_ns.split('::')) + '_protobuf_codec.hpp'
        sub_topics, pub_topics = self._topics.topics_for_proto(parsed, is_provided)
        all_topics = dict(sub_topics)
        all_topics.update(pub_topics)
        topic_set = all_topics
        hpp_name = file_prefix + '.hpp'
        duplicate_rpc_names = _duplicate_rpc_names(all_rpcs)

        data_model_files = self._discover_data_model_proto_files()
        codec_imports = _service_codec_imports(
            data_model_files,
            all_rpcs,
            list(topic_set.keys()),
            self._topics,
        )
        dm_codec_nss = [ns for ns, _header in codec_imports]
        dm_codec_headers = [header for _ns, header in codec_imports]

        # C-ABI typed-value boundary: schema types this contract marshals
        # through the runtime codec registry, and the cabi marshalling headers
        # for the data-model modules in their dependency closure.
        _cc_base, _cc_ns, cabi_codec_types = \
            self._collect_contract_codec_types(parsed)
        cabi_marshal_headers = sorted({
            '.'.join(full_type.split('.')[:-1]).replace('.', '_')
            + '_cabi_marshal.hpp'
            for _short, full_type, is_alias, _alias_cpp, _is_array in cabi_codec_types
            if not is_alias and '.' in full_type
        })

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            # File-level comment block
            f.write('// Auto-generated service binding implementation\n')
            f.write(f'// Generated from: {self._proto_input.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n\n')

            # Includes
            f.write(f'#include "{hpp_name}"\n\n')
            # C-ABI marshalling for the typed-value plugin boundary
            for ch in cabi_marshal_headers:
                f.write(f'#include "{ch}"\n')
            f.write('\n')
            f.write('extern "C" {\n')
            f.write('#include <pcl/pcl_codec.h>\n')
            f.write('#include <pcl/pcl_codec_registry.h>\n')
            f.write('#include <pcl/pcl_container.h>\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_transport.h>\n')
            f.write('#include <pcl/pcl_alloc.h>\n')
            f.write('#include "pyramid_datamodel_cabi.h"\n')
            f.write('}\n')
            f.write('\n#include <cstdlib>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <cstring>\n')
            f.write('#include <limits>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')

            # Namespace open
            f.write(f'namespace {full_ns} {{\n\n')

            f.write('\n')

            # ---- msgToString -------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// PCL message utility\n')
            f.write(_SEP + '\n\n')
            f.write('std::string msgToString(const void* data, unsigned size) {\n')
            f.write('    return std::string(static_cast<const char*>(data), size);\n')
            f.write('}\n\n')

            # ---- ServiceHandler stubs ----------------------------------------
            f.write(_SEP + '\n')
            f.write('// ServiceHandler \u2014 default stub implementations\n')
            f.write(_SEP + '\n\n')

            for svc_name, rpc in all_rpcs:
                rsp_t = _cpp_rsp_type(rpc)
                req_t = _cpp_req_type(rpc)
                handler_name = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)

                f.write(f'{rsp_t}\n')
                f.write(f'ServiceHandler::{handler_name}'
                        f'(const {req_t}& /*request*/) {{\n')

                if rsp_t == 'Ack':
                    f.write(f'    return {types_ns}::kAckOk;\n')
                else:
                    f.write('    return {};\n')

                f.write('}\n\n')

                if rpc.server_streaming:
                    stream_name = _rpc_stream_handler_name(
                        svc_name, rpc, duplicate_rpc_names)
                    f.write('pcl_status_t\n')
                    f.write(f'ServiceHandler::{stream_name}'
                            f'(const {req_t}& /*request*/,\n')
                    f.write('    pcl_stream_context_t* /*stream_context*/,\n')
                    f.write('    const char* /*content_type*/) {\n')
                    f.write('    return PCL_ERR_INVALID;\n')
                    f.write('}\n\n')

            # ---- Internal PCL helpers (anonymous namespace) ------------------
            f.write(_SEP + '\n')
            f.write('// Internal PCL helpers\n')
            f.write(_SEP + '\n\n')
            f.write('namespace {\n\n')

            # C-ABI typed-value boundary: marshal the native value to its frozen
            # C struct (to_c), hand the C struct to the registry codec, then free.
            # Decode is symmetric: the plugin fills the C representation, we
            # move it into the native value and free any C-owned storage.
            non_alias_cabi = [
                (short, full_type)
                for short, full_type, is_alias, _alias_cpp, is_array in cabi_codec_types
                if not is_alias and not is_array
            ]
            alias_cabi = [
                (short, _alias_cpp)
                for short, _full_type, is_alias, _alias_cpp, _is_array in cabi_codec_types
                if is_alias
            ]
            array_cabi = [
                (short, full_type)
                for short, full_type, is_alias, _alias_cpp, is_array in cabi_codec_types
                if is_array
            ]

            f.write('static int pyramid_cabi_encode(const pcl_codec_t* c,\n')
            f.write('                               const char* schema_id,\n')
            f.write('                               const void* value,\n')
            f.write('                               pcl_msg_t* out_msg)\n')
            f.write('{\n')
            for short, alias_cpp in alias_cabi:
                native = f'{_DATA_MODEL_TYPES_NS}::{short}'
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                if alias_cpp == 'std::string':
                    f.write(f'        const auto& native = *static_cast<const {native}*>(value);\n')
                    f.write('        if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                    f.write('        pyramid_str_t cs;\n')
                    f.write('        cs.ptr = native.data();\n')
                    f.write('        cs.len = static_cast<uint32_t>(native.size());\n')
                    f.write('        const pcl_status_t rc =\n')
                    f.write('            c->encode(c->codec_ctx, schema_id, &cs, out_msg);\n')
                else:
                    f.write('        const pcl_status_t rc =\n')
                    f.write('            c->encode(c->codec_ctx, schema_id, value, out_msg);\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            for short, _full in non_alias_cabi:
                native = f'{_native_namespace_for_type(_full)}::{short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        {c_struct} cs;\n')
                f.write(f'        pyramid::cabi::to_c(\n')
                f.write(f'            *static_cast<const {native}*>(value), &cs);\n')
                f.write('        const pcl_status_t rc =\n')
                f.write('            c->encode(c->codec_ctx, schema_id, &cs, out_msg);\n')
                f.write(f'        {c_struct}_free(&cs);\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            for short, _full in array_cabi:
                elem_short = short[:-len('Array')]
                native = f'{_native_namespace_for_type(_full)}::{elem_short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        const auto& native = *static_cast<const std::vector<{native}>*>(value);\n')
                f.write('        if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        std::vector<{c_struct}> values(native.size());\n')
                f.write('        if (!values.empty()) {\n')
                f.write('            std::memset(values.data(), 0, values.size() * sizeof(values[0]));\n')
                f.write('        }\n')
                f.write('        for (std::size_t i = 0; i < native.size(); ++i) {\n')
                f.write('            pyramid::cabi::to_c(native[i], &values[i]);\n')
                f.write('        }\n')
                f.write('        pyramid_slice_t slice;\n')
                f.write('        slice.ptr = values.empty() ? nullptr : values.data();\n')
                f.write('        slice.len = static_cast<uint32_t>(values.size());\n')
                f.write('        const pcl_status_t rc =\n')
                f.write('            c->encode(c->codec_ctx, schema_id, &slice, out_msg);\n')
                f.write('        for (auto& item : values) {\n')
                f.write(f'            {c_struct}_free(&item);\n')
                f.write('        }\n')
                f.write('        return rc == PCL_OK ? 1 : -1;\n')
                f.write('    }\n')
            f.write('    (void)c; (void)value; (void)out_msg;\n')
            f.write('    return 0;\n')
            f.write('}\n\n')

            f.write('static int pyramid_cabi_decode(const pcl_codec_t* c,\n')
            f.write('                               const char* schema_id,\n')
            f.write('                               const pcl_msg_t* msg,\n')
            f.write('                               void* out_value)\n')
            f.write('{\n')
            for short, alias_cpp in alias_cabi:
                native = f'{_DATA_MODEL_TYPES_NS}::{short}'
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                if alias_cpp == 'std::string':
                    f.write('        pyramid_str_t cs;\n')
                    f.write('        std::memset(&cs, 0, sizeof(cs));\n')
                    f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &cs)\n')
                    f.write('                != PCL_OK) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                    f.write(f'        auto* native = static_cast<{native}*>(out_value);\n')
                    f.write('        native->assign(cs.ptr ? cs.ptr : "", cs.len);\n')
                    f.write('        pcl_free(const_cast<char*>(cs.ptr));\n')
                else:
                    f.write('        if (c->decode(c->codec_ctx, schema_id, msg, out_value)\n')
                    f.write('                != PCL_OK) {\n')
                    f.write('            return -1;\n')
                    f.write('        }\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            for short, _full in non_alias_cabi:
                native = f'{_native_namespace_for_type(_full)}::{short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write(f'        {c_struct} cs;\n')
                f.write('        std::memset(&cs, 0, sizeof(cs));\n')
                f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &cs)\n')
                f.write('                != PCL_OK) {\n')
                f.write(f'            {c_struct}_free(&cs);\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        pyramid::cabi::from_c(\n')
                f.write(f'            &cs, *static_cast<{native}*>(out_value));\n')
                f.write(f'        {c_struct}_free(&cs);\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            for short, _full in array_cabi:
                elem_short = short[:-len('Array')]
                native = f'{_native_namespace_for_type(_full)}::{elem_short}'
                c_struct = _c_struct_for_type(_full)
                f.write(f'    if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                f.write('        pyramid_slice_t slice;\n')
                f.write('        std::memset(&slice, 0, sizeof(slice));\n')
                f.write('        if (c->decode(c->codec_ctx, schema_id, msg, &slice)\n')
                f.write('                != PCL_OK) {\n')
                f.write('            return -1;\n')
                f.write('        }\n')
                f.write(f'        auto* native = static_cast<std::vector<{native}>*>(out_value);\n')
                f.write('        native->clear();\n')
                f.write('        native->reserve(slice.len);\n')
                f.write(f'        auto* values = static_cast<{c_struct}*>(const_cast<void*>(slice.ptr));\n')
                f.write('        for (uint32_t i = 0; i < slice.len; ++i) {\n')
                f.write(f'            {native} item;\n')
                f.write('            pyramid::cabi::from_c(&values[i], item);\n')
                f.write('            native->push_back(std::move(item));\n')
                f.write(f'            {c_struct}_free(&values[i]);\n')
                f.write('        }\n')
                f.write('        pcl_free(values);\n')
                f.write('        return 1;\n')
                f.write('    }\n')
            f.write('    (void)c; (void)msg; (void)out_value;\n')
            f.write('    return 0;\n')
            f.write('}\n\n')

            # A single process (e.g. a PYRAMID bridge) may load codec plugins for
            # several components, all registered under the same content_type. We
            # iterate the codecs registered for the content_type and try each by
            # schema_id until one handles it; if none do (or the schema is a
            # scalar alias), we return -1 so the caller uses the static codec.
            f.write('static int pyramid_try_registry_encode(const char* content_type,\n')
            f.write('                                       const char* schema_id,\n')
            f.write('                                       const void* value,\n')
            f.write('                                       std::string* out)\n')
            f.write('{\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    for (uint32_t i = 0; ; ++i) {\n')
            f.write('        const pcl_codec_t* c =\n')
            f.write('            pcl_codec_registry_get_at(reg, content_type, i);\n')
            f.write('        if (!c) {\n')
            f.write('            break;\n')
            f.write('        }\n')
            f.write('        if (!c->encode) {\n')
            f.write('            continue;\n')
            f.write('        }\n')
            f.write('        pcl_msg_t m;\n')
            f.write('        m.data = nullptr;\n')
            f.write('        m.size = 0;\n')
            f.write('        m.type_name = nullptr;\n')
            f.write('        const int r = pyramid_cabi_encode(c, schema_id, value, &m);\n')
            f.write('        if (r == 1) {\n')
            f.write('            if (m.data && m.size != 0) {\n')
            f.write('                out->assign(static_cast<const char*>(m.data), m.size);\n')
            f.write('            } else {\n')
            f.write('                out->clear();\n')
            f.write('            }\n')
            f.write('            if (c->free_msg) {\n')
            f.write('                c->free_msg(c->codec_ctx, &m);\n')
            f.write('            }\n')
            f.write('            return 1;\n')
            f.write('        }\n')
            f.write('        if (r == 0) {\n')
            f.write('            return -1;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return -1;\n')
            f.write('}\n\n')

            f.write('static int pyramid_try_registry_decode(const pcl_msg_t* msg,\n')
            f.write('                                       const char* schema_id,\n')
            f.write('                                       void* out_value)\n')
            f.write('{\n')
            f.write('    if (!msg) {\n')
            f.write('        return -1;\n')
            f.write('    }\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    for (uint32_t i = 0; ; ++i) {\n')
            f.write('        const pcl_codec_t* c =\n')
            f.write('            pcl_codec_registry_get_at(reg, msg->type_name, i);\n')
            f.write('        if (!c) {\n')
            f.write('            break;\n')
            f.write('        }\n')
            f.write('        if (!c->decode) {\n')
            f.write('            continue;\n')
            f.write('        }\n')
            f.write('        const int r =\n')
            f.write('            pyramid_cabi_decode(c, schema_id, msg, out_value);\n')
            f.write('        if (r == 1) {\n')
            f.write('            return 1;\n')
            f.write('        }\n')
            f.write('        if (r == 0) {\n')
            f.write('            return -1;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return -1;\n')
            f.write('}\n\n')

            f.write('void ignore_async_response(const pcl_msg_t*, void*) {}\n\n')

            f.write('pcl_status_t invoke_async('
                    'pcl_executor_t* executor,\n')
            f.write('                           const char*'
                    '             service_name,\n')
            f.write('                           const std::string&'
                    '      payload,\n')
            f.write('                           pcl_resp_cb_fn_t'
                    '        callback,\n')
            f.write('                           void*'
                    '                   user_data,\n')
            f.write('                           const pcl_endpoint_route_t*'
                    ' route,\n')
            f.write('                           const char*             content_type)\n')
            f.write('{\n')
            f.write('    pcl_msg_t msg{};\n')
            f.write('    msg.data      = payload.data();\n')
            f.write('    msg.size      = static_cast<uint32_t>'
                    '(payload.size());\n')
            f.write('    msg.type_name = content_type;\n')
            f.write('    if (route) {\n')
            f.write('        const pcl_status_t route_rc ='
                    ' pcl_executor_set_endpoint_route(executor, route);\n')
            f.write('        if (route_rc != PCL_OK) {\n')
            f.write('            return route_rc;\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return pcl_executor_invoke_async(\n')
            f.write('        executor, service_name, &msg,'
                    ' callback, user_data);\n')
            f.write('}\n\n')

            f.write('} // namespace\n\n')

            # ---- Content-type metadata ---------------------------------------
            f.write(_SEP + '\n')
            f.write('// Content-type support metadata\n')
            f.write(_SEP + '\n\n')
            f.write('std::vector<const char*> supportedContentTypes()\n')
            f.write('{\n')
            f.write('    std::vector<const char*> result;\n')
            f.write('    pcl_codec_registry_t* reg = pcl_codec_registry_default();\n')
            f.write('    if (pcl_codec_registry_get(reg, kJsonContentType) != nullptr) {\n')
            f.write('        result.push_back(kJsonContentType);\n')
            f.write('    }\n')
            if has_flatbuffers:
                f.write('    if (pcl_codec_registry_get(reg, kFlatBuffersContentType) != nullptr) {\n')
                f.write('        result.push_back(kFlatBuffersContentType);\n')
                f.write('    }\n')
            if has_protobuf:
                f.write('    if (pcl_codec_registry_get(reg, kProtobufContentType) != nullptr) {\n')
                f.write('        result.push_back(kProtobufContentType);\n')
                f.write('    }\n')
            f.write('    return result;\n')
            f.write('}\n\n')
            f.write('bool supportsContentType(const char* content_type)\n')
            f.write('{\n')
            f.write('    return content_type\n')
            f.write('        && pcl_codec_registry_get(pcl_codec_registry_default(), content_type)\n')
            f.write('            != nullptr;\n')
            f.write('}\n\n')

            # Subscribe wrappers (unchanged)
            n_topics = len(topic_set)
            if n_topics:
                f.write(_SEP + '\n')
                label = 'PCL subscribe wrapper' + ('s' if n_topics > 1 else '')
                f.write(f'// {label}\n')
                f.write(_SEP + '\n\n')

                for key, _wire in topic_set.items():
                    pascal = _snake_to_pascal(key)
                    fname = f'subscribe{pascal}'
                    cname = f'kTopic{pascal}'
                    col = len(f'pcl_port_t* {fname}(')
                    sp = ' ' * col
                    f.write(f'pcl_port_t* {fname}(pcl_container_t*   container,\n')
                    f.write(f'{sp}pcl_sub_callback_t  callback,\n')
                    f.write(f'{sp}void*              user_data,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    return pcl_container_add_subscriber(container,\n')
                    f.write(f'                                 {cname},\n')
                    f.write('                                 content_type,\n')
                    f.write('                                 callback,\n')
                    f.write('                                 user_data);\n')
                    f.write('}\n\n')

            # Typed response decoders -- hide content-type details from
            # component response callbacks.
            f.write(_SEP + '\n')
            f.write('// Typed response decode wrappers\n')
            f.write(_SEP + '\n\n')
            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    rsp_decl_t = _cpp_rsp_type(rpc)
                    rsp_raw_t = _short_type(rpc.response_type)
                    decode_func = _rpc_decode_response_func(
                        svc.name, rpc, duplicate_rpc_names)
                    col = len(f'bool {decode_func}(')
                    sp = ' ' * col
                    f.write(f'bool {decode_func}'
                            f'(const pcl_msg_t* msg,\n')
                    f.write(f'{sp}{rsp_decl_t}* out)\n')
                    f.write('{\n')
                    f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    if rpc.server_streaming:
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{rsp_raw_t}Array", out); if (r == 1) return true; }}\n')
                    else:
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{rsp_raw_t}", out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

                    if rpc.server_streaming:
                        frame_t = _cpp_rsp_type(rpc)[len('std::vector<'):-1]
                        encode_frame = _rpc_encode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        decode_frame = _rpc_decode_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)
                        send_frame = _rpc_send_stream_frame_func(
                            svc.name, rpc, duplicate_rpc_names)

                        col = len(f'bool {encode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {encode_frame}(const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type,\n')
                        f.write(f'{sp}std::string*       out)\n')
                        f.write('{\n')
                        f.write('    if (!out) {\n')
                        f.write('        return false;\n')
                        f.write('    }\n')
                        f.write(f'    {{ int r = pyramid_try_registry_encode(content_type, "{_short_type(rpc.response_type)}", &payload, out); if (r == 1) return true; }}\n')
                        f.write('    return false;\n')
                        f.write('}\n\n')

                        col = len(f'bool {decode_frame}(')
                        sp = ' ' * col
                        f.write(f'bool {decode_frame}(const pcl_msg_t* msg,\n')
                        f.write(f'{sp}{frame_t}* out)\n')
                        f.write('{\n')
                        f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                        f.write('        return false;\n')
                        f.write('    }\n')
                        f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{_short_type(rpc.response_type)}", out); if (r == 1) return true; }}\n')
                        f.write('    return false;\n')
                        f.write('}\n\n')

                        col = 13 + len(send_frame) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {send_frame}'
                                f'(pcl_stream_context_t* stream_context,\n')
                        f.write(f'{sp}const {frame_t}& payload,\n')
                        f.write(f'{sp}const char*        content_type)\n')
                        f.write('{\n')
                        f.write('    std::string wire_payload;\n')
                        f.write(f'    if (!{encode_frame}(payload, content_type, &wire_payload)) {{\n')
                        f.write('        return PCL_ERR_INVALID;\n')
                        f.write('    }\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data = wire_payload.data();\n')
                        f.write('    msg.size = static_cast<uint32_t>(wire_payload.size());\n')
                        f.write('    msg.type_name = content_type;\n')
                        f.write('    return pcl_stream_send(stream_context, &msg);\n')
                        f.write('}\n\n')

            # Typed invoke wrappers -- serialize request, then PCL
            f.write(_SEP + '\n')
            f.write('// Typed invoke wrappers \u2014 serialise and dispatch via executor transport\n')
            f.write(_SEP + '\n\n')
            for svc in parsed.services:
                for rpc in _crud_rpcs(svc):
                    req_decl_t = _cpp_req_type(rpc)
                    invoke_func = _rpc_invoke_func(
                        svc.name, rpc, duplicate_rpc_names)
                    service_const = _rpc_service_const(
                        svc.name, rpc, duplicate_rpc_names)
                    col = 13 + len(invoke_func) + 1
                    sp = ' ' * col
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}pcl_resp_cb_fn_t'
                            f'        callback,\n')
                    f.write(f'{sp}void*'
                            f'                   user_data,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route,\n')
                    f.write(f'{sp}const char*       content_type)\n')
                    f.write('{\n')
                    f.write('    std::string payload;\n')
                    f.write(f'    if (pyramid_try_registry_encode(content_type, "{_short_type(rpc.request_type)}", &request, &payload) != 1) {{\n')
                    f.write('        return PCL_ERR_NOT_FOUND;\n')
                    f.write('    }\n')
                    f.write('    return invoke_async(executor,'
                            f' {service_const},'
                            f' payload, callback, user_data, route, content_type);\n')
                    f.write('}\n\n')
                    f.write(f'pcl_status_t {invoke_func}'
                            f'(pcl_executor_t* executor,\n')
                    f.write(f'{sp}const {req_decl_t}&'
                            f'{" " * max(1, 22 - len(req_decl_t))}'
                            f'request,\n')
                    f.write(f'{sp}const char*       content_type,\n')
                    f.write(f'{sp}const pcl_endpoint_route_t* route)\n')
                    f.write('{\n')
                    f.write(f'    return {invoke_func}'
                            '(executor, request, ignore_async_response,\n')
                    f.write('                         nullptr, route, content_type);\n')
                    f.write('}\n\n')
                    if rpc.server_streaming:
                        invoke_stream = _rpc_invoke_stream_func(
                            svc.name, rpc, duplicate_rpc_names)
                        col = 13 + len(invoke_stream) + 1
                        sp = ' ' * col
                        f.write(f'pcl_status_t {invoke_stream}'
                                f'(pcl_executor_t* executor,\n')
                        f.write(f'{sp}const {req_decl_t}&'
                                f'{" " * max(1, 22 - len(req_decl_t))}'
                                f'request,\n')
                        f.write(f'{sp}pcl_stream_msg_fn_t'
                                f'   callback,\n')
                        f.write(f'{sp}void*'
                                f'                   user_data,\n')
                        f.write(f'{sp}pcl_stream_context_t** out_context,\n')
                        f.write(f'{sp}const pcl_endpoint_route_t* route,\n')
                        f.write(f'{sp}const char*       content_type)\n')
                        f.write('{\n')
                        f.write('    std::string payload;\n')
                        f.write(f'    if (pyramid_try_registry_encode(content_type, "{_short_type(rpc.request_type)}", &request, &payload) != 1) {{\n')
                        f.write('        return PCL_ERR_NOT_FOUND;\n')
                        f.write('    }\n')
                        f.write('    if (route) {\n')
                        f.write('        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);\n')
                        f.write('        if (route_rc != PCL_OK) {\n')
                        f.write('            return route_rc;\n')
                        f.write('        }\n')
                        f.write('    }\n')
                        f.write('    pcl_msg_t msg{};\n')
                        f.write('    msg.data = payload.data();\n')
                        f.write('    msg.size = static_cast<uint32_t>(payload.size());\n')
                        f.write('    msg.type_name = content_type;\n')
                        f.write(f'    return pcl_executor_invoke_stream(executor, {service_const},\n')
                        f.write('                                      &msg, callback, user_data,\n')
                        f.write('                                      out_context);\n')
                        f.write('}\n\n')
            # Publish and topic decode wrappers
            n_pubs = len(topic_set)
            if n_pubs:
                f.write(_SEP + '\n')
                label = 'PCL topic wrapper' + ('s' if n_pubs > 1 else '')
                f.write(f'// {label}\n')
                f.write(_SEP + '\n\n')
                for key, _wire in topic_set.items():
                    pascal = _snake_to_pascal(key)
                    fname = f'publish{pascal}'
                    decode_name = f'decode{pascal}'
                    encode_name = f'encode{pascal}'
                    spec = self._topics.spec(key)
                    col = len(f'bool {encode_name}(')
                    sp = ' ' * col
                    f.write(f'bool {encode_name}(const {spec.cpp_payload_type}& payload,\n')
                    f.write(f'{sp}const char*        content_type,\n')
                    f.write(f'{sp}std::string*       out)\n')
                    f.write('{\n')
                    f.write('    if (!out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    topic_schema = spec.short_type + ('Array' if spec.is_array else '')
                    f.write(f'    {{ int r = pyramid_try_registry_encode(content_type, "{topic_schema}", &payload, out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

                    col = 13 + len(fname) + 1
                    sp = ' ' * col
                    f.write(f'pcl_status_t {fname}'
                            f'(pcl_port_t*        publisher,\n')
                    f.write(f'{sp}const {spec.cpp_payload_type}& payload,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    std::string wire_payload;\n')
                    f.write(f'    if (!{encode_name}(payload, content_type, &wire_payload)) {{\n')
                    f.write('        return PCL_ERR_INVALID;\n')
                    f.write('    }\n')
                    f.write(f'    return {fname}(publisher, wire_payload, content_type);\n')
                    f.write('}\n\n')
                    f.write(f'pcl_status_t {fname}'
                            f'(pcl_port_t*        publisher,\n')
                    f.write(f'{sp}const std::string& payload,\n')
                    f.write(f'{sp}const char*        content_type)\n')
                    f.write('{\n')
                    f.write('    pcl_msg_t msg{};\n')
                    f.write('    msg.data      = payload.data();\n')
                    f.write('    msg.size      = static_cast<uint32_t>'
                            '(payload.size());\n')
                    f.write('    msg.type_name = content_type;\n')
                    f.write('    return pcl_port_publish(publisher,'
                            ' &msg);\n')
                    f.write('}\n\n')

                    col = len(f'bool {decode_name}(')
                    sp = ' ' * col
                    f.write(f'bool {decode_name}(const pcl_msg_t* msg,\n')
                    f.write(f'{sp}{spec.cpp_payload_type}* out)\n')
                    f.write('{\n')
                    f.write('    if (!msg || !msg->data || msg->size == 0 || !out) {\n')
                    f.write('        return false;\n')
                    f.write('    }\n')
                    f.write(f'    {{ int r = pyramid_try_registry_decode(msg, "{topic_schema}", out); if (r == 1) return true; }}\n')
                    f.write('    return false;\n')
                    f.write('}\n\n')

            # ---- dispatch() --------------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Dispatch \u2014 deserialise request, call handler,'
                    ' serialise response\n')
            f.write(_SEP + '\n\n')
            f.write('void dispatch(ServiceHandler& handler,\n')
            f.write('              ServiceChannel  channel,\n')
            f.write('              const void*     request_buf,\n')
            f.write('              size_t          request_size,\n')
            f.write('              const char*     content_type,\n')
            f.write('              void**          response_buf,\n')
            f.write('              size_t*         response_size)\n')
            f.write('{\n')
            f.write('    if (!response_buf || !response_size) {\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    *response_buf = nullptr;\n')
            f.write('    *response_size = 0;\n')
            f.write('    pcl_msg_t req_msg{};\n')
            f.write('    req_msg.data = request_buf;\n')
            f.write('    req_msg.size = static_cast<uint32_t>(request_size);\n')
            f.write('    req_msg.type_name = content_type;\n')
            f.write('    std::string rsp_payload;\n\n')
            f.write('    try {\n')
            f.write('    switch (channel) {\n')

            for svc_name, rpc in all_rpcs:
                enum_val = _rpc_enum_value(svc_name, rpc, duplicate_rpc_names)
                handler_fn = _rpc_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = _cpp_req_type(rpc)
                rsp_t = _cpp_rsp_type(rpc)

                f.write(f'    case ServiceChannel::{enum_val}: {{\n')
                f.write(f'        {req_t} req;\n')
                f.write(f'        if (pyramid_try_registry_decode(&req_msg, "{_short_type(rpc.request_type)}", &req) != 1) {{\n')
                f.write('            break;\n')
                f.write('        }\n')
                f.write(f'        auto rsp = handler.{handler_fn}(req);\n')
                if rsp_t.startswith('std::vector<'):
                    f.write(f'        if (pyramid_try_registry_encode(content_type, "{_short_type(rpc.response_type)}Array", &rsp, &rsp_payload) != 1) {{\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                else:
                    f.write(f'        if (pyramid_try_registry_encode(content_type, "{_short_type(rpc.response_type)}", &rsp, &rsp_payload) != 1) {{\n')
                    f.write('            break;\n')
                    f.write('        }\n')
                f.write('        break;\n')
                f.write('    }\n')

            f.write('    }\n')
            f.write('    } catch (...) {\n')
            f.write('        *response_buf = nullptr;\n')
            f.write('        *response_size = 0;\n')
            f.write('        return;\n')
            f.write('    }\n\n')

            # Copy to heap buffer
            f.write('    if (!rsp_payload.empty()) {\n')
            f.write('        *response_size = rsp_payload.size();\n')
            f.write('        *response_buf = std::malloc(rsp_payload.size());\n')
            f.write('        std::memcpy(*response_buf, rsp_payload.data(),'
                    ' rsp_payload.size());\n')
            f.write('    } else {\n')
            f.write('        *response_buf = nullptr;\n')
            f.write('        *response_size = 0;\n')
            f.write('    }\n')
            f.write('}\n\n')

            # ---- dispatchStream() -------------------------------------------
            f.write(_SEP + '\n')
            f.write('// Stream dispatch \u2014 deserialise request and open stream\n')
            f.write(_SEP + '\n\n')
            f.write('pcl_status_t dispatchStream(ServiceHandler& handler,\n')
            f.write('                            ServiceChannel  channel,\n')
            f.write('                            const void*     request_buf,\n')
            f.write('                            size_t          request_size,\n')
            f.write('                            const char*     content_type,\n')
            f.write('                            pcl_stream_context_t* stream_context)\n')
            f.write('{\n')
            f.write('    pcl_msg_t req_msg{};\n')
            f.write('    req_msg.data = request_buf;\n')
            f.write('    req_msg.size = static_cast<uint32_t>(request_size);\n')
            f.write('    req_msg.type_name = content_type;\n\n')
            f.write('    try {\n')
            f.write('    switch (channel) {\n')

            for svc_name, rpc in all_rpcs:
                if not rpc.server_streaming:
                    continue
                enum_val = _rpc_enum_value(svc_name, rpc, duplicate_rpc_names)
                stream_fn = _rpc_stream_handler_name(
                    svc_name, rpc, duplicate_rpc_names)
                req_t = _cpp_req_type(rpc)
                f.write(f'    case ServiceChannel::{enum_val}: {{\n')
                f.write(f'        {req_t} req;\n')
                f.write(f'        if (pyramid_try_registry_decode(&req_msg, "{_short_type(rpc.request_type)}", &req) != 1) {{\n')
                f.write('            return PCL_ERR_NOT_FOUND;\n')
                f.write('        }\n')
                f.write(f'        return handler.{stream_fn}(req, stream_context, content_type);\n')
                f.write('    }\n')

            f.write('    }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_INVALID;\n')
            f.write('}\n\n')

            # Namespace close
            f.write(f'}} // namespace {full_ns}\n')

