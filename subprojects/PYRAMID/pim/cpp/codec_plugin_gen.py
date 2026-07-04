#!/usr/bin/env python3
"""Contract codec plugin emitters for CppServiceGenerator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

from pathlib import Path
from typing import List, Tuple

from proto_parser import (ProtoTypeIndex, ProtoFile)
from proto_resolve import (_DATA_MODEL_PROTO_ROOT, _resolve_message)
from .naming import (
    _c_struct_for_type,
    _native_namespace_for_type,
    _service_contract_names,
    _json_codec_namespace_for_type,
    _json_codec_header_for_type,
    _alias_cpp_types,
    _service_group_key,
)


class CodecPluginEmitterMixin:
    """Per-contract codec plugin source emission."""

    def _collect_contract_codec_types(
            self, parsed: ProtoFile,
    ) -> Tuple[str, str, List[Tuple[str, str, bool, str]]]:
        """Return contract file base, namespace, and root schema types.

        Type tuples are (short_name, full_type, is_alias, alias_cpp_type).
        """
        base_package, service_files = self._contract_service_files(parsed.package)
        if not base_package:
            return '', '', []
        if not service_files:
            return '', '', []

        dm_files = [
            pf for pf in self._discover_all_proto_files()
            if pf.package == _DATA_MODEL_PROTO_ROOT
            or pf.package.startswith(_DATA_MODEL_PROTO_ROOT + '.')
        ]
        indexed_files = dm_files + [
            pf for pf in service_files
            if pf not in dm_files
        ]
        index = ProtoTypeIndex(indexed_files)
        aliases = _alias_cpp_types(index)
        file_base, cpp_base_ns = _service_contract_names(base_package)
        root_types: List[Tuple[str, str]] = []

        for pf in service_files:
            for svc in pf.services:
                for rpc in svc.rpcs:
                    root_types.append((rpc.request_type, pf.package))
                    root_types.append((rpc.response_type, pf.package))
            is_provided = 'provided' in pf.package.lower()
            sub_topics, pub_topics = self._topics.topics_for_proto(pf, is_provided)
            topic_keys = list(sub_topics.keys()) + list(pub_topics.keys())
            for key in topic_keys:
                root_types.append((self._topics.spec(key).full_type, ''))

        result: List[Tuple[str, str, bool, str, bool]] = []
        seen_schema_ids = set()
        for type_name, current_pkg in root_types:
            short = type_name.split('.')[-1]
            if short in seen_schema_ids:
                continue

            msg, msg_pkg = _resolve_message(index, type_name, current_pkg)
            if msg is None and short not in aliases:
                continue

            full_type = type_name
            if msg is not None and '.' not in type_name:
                full_type = f'{msg_pkg}.{msg.name}' if msg_pkg else type_name

            result.append((short, full_type, short in aliases,
                           aliases.get(short, ''), False))
            seen_schema_ids.add(short)

        for pf in service_files:
            for svc in pf.services:
                for rpc in svc.rpcs:
                    if not rpc.server_streaming:
                        continue
                    elem_short = rpc.response_type.split('.')[-1]
                    array_schema = elem_short + 'Array'
                    if array_schema in seen_schema_ids:
                        continue
                    msg, msg_pkg = _resolve_message(
                        index, rpc.response_type, pf.package)
                    if msg is None:
                        continue
                    full_type = rpc.response_type
                    if '.' not in full_type:
                        full_type = f'{msg_pkg}.{msg.name}' if msg_pkg else full_type
                    result.append((array_schema, full_type, False, '', True))
                    seen_schema_ids.add(array_schema)

        return file_base, cpp_base_ns, result

    def _write_codec_plugins(self, output_path: Path, parsed: ProtoFile) -> None:
        file_base, cpp_base_ns, codec_types = self._collect_contract_codec_types(parsed)
        if not file_base or not codec_types:
            return

        json_path = output_path / (file_base + '_json_codec_plugin.cpp')
        self._write_codec_plugin_impl(
            json_path,
            file_base,
            cpp_base_ns,
            'json',
            'application/json',
            codec_types,
        )

        if self._has_backend('flatbuffers'):
            flatbuffers_path = output_path / (
                file_base + '_flatbuffers_codec_plugin.cpp')
            self._write_codec_plugin_impl(
                flatbuffers_path,
                file_base,
                cpp_base_ns,
                'flatbuffers',
                'application/flatbuffers',
                codec_types,
            )

        # Only emit a protobuf codec plugin for service packages that have the
        # local-struct <-> protobuf bridge (the checked-in service protobuf
        # codec). This mirrors the facade's protobuf gating in
        # _service_protobuf_codec_available; today only tactical_objects has it.
        if (self._has_backend('protobuf')
                and self._service_protobuf_codec_available(cpp_base_ns)):
            protobuf_path = output_path / (
                file_base + '_protobuf_codec_plugin.cpp')
            self._write_codec_plugin_impl(
                protobuf_path,
                file_base,
                cpp_base_ns,
                'protobuf',
                'application/protobuf',
                codec_types,
            )

        if self._has_backend('ros2'):
            codec_package = _service_group_key(parsed.package) or parsed.package
            # The typed ROS2 codec (pyramid_ros2_codec.hpp) only marshals emitted
            # message types; scalar-wrapper aliases (e.g. Identifier -> std::string)
            # have no pyramid_msgs .msg and are excluded from its toBinary/
            # fromBinary surface. Drop alias schema types here so the generated
            # plugin does not reference non-existent overloads (those payloads
            # fall back to the envelope wire). is_alias is tuple element [2].
            ros2_codec_types = [t for t in codec_types if not t[2]]
            if ros2_codec_types:
                ros2_path = output_path / 'ros2' / 'cpp' / (
                    file_base + '_ros2_codec_plugin.cpp')
                self._write_codec_plugin_impl(
                    ros2_path,
                    file_base,
                    cpp_base_ns,
                    'ros2',
                    'application/ros2',
                    ros2_codec_types,
                    codec_package=codec_package,
                )

    def _write_codec_plugin_impl(
            self,
            path: Path,
            file_base: str,
            cpp_base_ns: str,
            backend: str,
            content_type: str,
            codec_types: List[Tuple[str, str, bool, str]],
            codec_package: str = '',
    ) -> None:
        is_json = backend == 'json'
        json_headers = sorted({
            _json_codec_header_for_type(full_type)
            for _short, full_type, is_alias, _alias_cpp, _is_array in codec_types
            if not is_alias and _json_codec_header_for_type(full_type)
        })
        cabi_marshal_headers = sorted({
            '.'.join(full_type.split('.')[:-1]).replace('.', '_')
            + '_cabi_marshal.hpp'
            for _short, full_type, is_alias, _alias_cpp, _is_array in codec_types
            if not is_alias and '.' in full_type
        })
        # flatbuffers, protobuf, and ros2 are binary backends with the same
        # native<->wire surface (wire_codec::toBinary(native) /
        # fromBinary<Short>(data, size)); only the namespace and header differ.
        if backend == 'protobuf':
            wire_codec_ns = cpp_base_ns + '::protobuf_codec'
            # The protobuf service bridge header ships as source support, which
            # pyramid_protobuf_support exposes as an include dir, so it is
            # included bare (unlike the flatbuffers codec header, which is
            # generated into the build bindings dir under flatbuffers/cpp).
            wire_codec_header = file_base + '_protobuf_codec.hpp'
        elif backend == 'ros2':
            wire_codec_ns = self._naming.ros2_codec_namespace(codec_package)
            wire_codec_header = self._naming.ros2_codec_header(codec_package)
        else:
            wire_codec_ns = cpp_base_ns + '::flatbuffers_codec'
            wire_codec_header = (
                'flatbuffers/cpp/' + file_base + '_flatbuffers_codec.hpp')

        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated PCL codec plugin\n')
            f.write(f'// Backend: {backend} | Content-Type: {content_type}\n\n')
            f.write('#include "pyramid_data_model_types.hpp"\n')
            if is_json:
                for header in json_headers:
                    f.write(f'#include "{header}"\n')
                f.write('#include <nlohmann/json.hpp>\n')
            else:
                f.write(f'#include "{wire_codec_header}"\n')
            # C-ABI marshalling: the plugin receives/returns frozen C structs and
            # marshals to/from the native type before invoking the wire codec.
            for header in cabi_marshal_headers:
                f.write(f'#include "{header}"\n')
            f.write('\n')
            f.write('extern "C" {\n')
            f.write('#include <pcl/pcl_codec.h>\n')
            f.write('#include <pcl/pcl_alloc.h>\n')
            f.write('#include "pyramid_datamodel_cabi.h"\n')
            f.write('}\n\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstring>\n')
            f.write('#include <limits>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write('#if defined(_WIN32)\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT __declspec(dllexport)\n')
            f.write('#elif defined(__GNUC__) || defined(__clang__)\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT __attribute__((visibility("default")))\n')
            f.write('#else\n')
            f.write('#  define PCL_CODEC_PLUGIN_EXPORT\n')
            f.write('#endif\n\n')
            f.write('namespace {\n\n')
            f.write('namespace data_model = pyramid::domain_model;\n')
            if not is_json:
                f.write(f'namespace wire_codec = {wire_codec_ns};\n')
            f.write('\n')
            f.write('pcl_status_t assign_payload(const std::string& payload,\n')
            f.write('                            const char* content_type,\n')
            f.write('                            pcl_msg_t* out_msg)\n')
            f.write('{\n')
            f.write('    if (!out_msg) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    if (payload.size() > std::numeric_limits<uint32_t>::max()) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    void* copy = nullptr;\n')
            f.write('    if (!payload.empty()) {\n')
            f.write('        copy = std::malloc(payload.size());\n')
            f.write('        if (!copy) {\n')
            f.write('            return PCL_ERR_NOMEM;\n')
            f.write('        }\n')
            f.write('        std::memcpy(copy, payload.data(), payload.size());\n')
            f.write('    }\n')
            f.write('    out_msg->data = copy;\n')
            f.write('    out_msg->size = static_cast<uint32_t>(payload.size());\n')
            f.write('    out_msg->type_name = content_type;\n')
            f.write('    return PCL_OK;\n')
            f.write('}\n\n')
            if is_json:
                f.write('template <class T>\n')
                f.write('std::string scalar_to_json(const T& value)\n')
                f.write('{\n')
                f.write('    return nlohmann::json(value).dump();\n')
                f.write('}\n\n')
                f.write('template <class T>\n')
                f.write('T scalar_from_json(const std::string& payload)\n')
                f.write('{\n')
                f.write('    return nlohmann::json::parse(payload).get<T>();\n')
                f.write('}\n\n')
            f.write('} // namespace\n\n')

            f.write('extern "C" {\n\n')
            f.write('static pcl_status_t plugin_encode(void*       codec_ctx,\n')
            f.write('                                  const char* schema_id,\n')
            f.write('                                  const void* value,\n')
            f.write('                                  pcl_msg_t*  out_msg)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!schema_id || !value || !out_msg) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    try {\n')
            for short, full_type, is_alias, _alias_cpp, is_array in codec_types:
                if is_alias:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    if _alias_cpp == 'std::string':
                        f.write('            const auto* cs = static_cast<const pyramid_str_t*>(value);\n')
                        f.write('            const char* data = cs && cs->ptr ? cs->ptr : "";\n')
                        f.write(f'            {cpp_type} native(data, cs ? cs->len : 0u);\n')
                    else:
                        f.write(f'            const auto* native = static_cast<const {cpp_type}*>(value);\n')
                        f.write('            if (!native) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                    if is_json:
                        if _alias_cpp == 'std::string':
                            f.write('            return assign_payload(scalar_to_json(native),\n')
                        else:
                            f.write('            return assign_payload(scalar_to_json(*native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        if _alias_cpp == 'std::string':
                            f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        else:
                            f.write('            return assign_payload(wire_codec::toBinary(*native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
                elif is_array:
                    elem_short = short[:-len('Array')]
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{elem_short}'
                    c_struct = _c_struct_for_type(full_type)
                    codec_ns = _json_codec_namespace_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write('            const auto* slice = static_cast<const pyramid_slice_t*>(value);\n')
                    f.write('            if (!slice || (!slice->ptr && slice->len != 0u)) {\n')
                    f.write('                return PCL_ERR_INVALID;\n')
                    f.write('            }\n')
                    f.write(f'            const auto* values = static_cast<const {c_struct}*>(slice->ptr);\n')
                    f.write(f'            std::vector<{cpp_type}> native;\n')
                    f.write('            native.reserve(slice->len);\n')
                    f.write('            for (uint32_t i = 0; i < slice->len; ++i) {\n')
                    f.write(f'                {cpp_type} item;\n')
                    f.write('                pyramid::cabi::from_c(&values[i], item);\n')
                    f.write('                native.push_back(std::move(item));\n')
                    f.write('            }\n')
                    if is_json:
                        f.write('            nlohmann::json arr = nlohmann::json::array();\n')
                        f.write('            for (const auto& item : native) {\n')
                        f.write(f'                arr.push_back(nlohmann::json::parse({codec_ns}::toJson(item)));\n')
                        f.write('            }\n')
                        f.write('            return assign_payload(arr.dump(),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
                else:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    c_struct = _c_struct_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            const auto* cs = static_cast<const {c_struct}*>(value);\n')
                    f.write(f'            {cpp_type} native;\n')
                    f.write('            pyramid::cabi::from_c(cs, native);\n')
                    if is_json:
                        codec_ns = _json_codec_namespace_for_type(full_type)
                        f.write(f'            return assign_payload({codec_ns}::toJson(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    else:
                        f.write('            return assign_payload(wire_codec::toBinary(native),\n')
                        f.write(f'                                  "{content_type}", out_msg);\n')
                    f.write('        }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_CALLBACK;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_NOT_FOUND;\n')
            f.write('}\n\n')

            f.write('static pcl_status_t plugin_decode(void*            codec_ctx,\n')
            f.write('                                  const char*      schema_id,\n')
            f.write('                                  const pcl_msg_t* msg,\n')
            f.write('                                  void*            out_value)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!schema_id || !msg || (!msg->data && msg->size != 0) || !out_value) {\n')
            f.write('        return PCL_ERR_INVALID;\n')
            f.write('    }\n')
            f.write('    try {\n')
            f.write('        const std::string payload = msg->data\n')
            f.write('            ? std::string(static_cast<const char*>(msg->data), msg->size)\n')
            f.write('            : std::string();\n')
            for short, full_type, is_alias, _alias_cpp, is_array in codec_types:
                if is_alias:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            {cpp_type} native;\n')
                    if is_json:
                        f.write(f'            native = scalar_from_json<{cpp_type}>(payload);\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    if _alias_cpp == 'std::string':
                        f.write('            auto* cs = static_cast<pyramid_str_t*>(out_value);\n')
                        f.write('            cs->ptr = nullptr;\n')
                        f.write('            cs->len = 0u;\n')
                        f.write('            if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                        f.write('            if (!native.empty()) {\n')
                        f.write('                void* copy = pcl_alloc(native.size());\n')
                        f.write('                if (!copy) {\n')
                        f.write('                    return PCL_ERR_NOMEM;\n')
                        f.write('                }\n')
                        f.write('                std::memcpy(copy, native.data(), native.size());\n')
                        f.write('                cs->ptr = static_cast<const char*>(copy);\n')
                        f.write('                cs->len = static_cast<uint32_t>(native.size());\n')
                        f.write('            }\n')
                    else:
                        f.write(f'            *static_cast<{cpp_type}*>(out_value) = native;\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
                elif is_array:
                    elem_short = short[:-len('Array')]
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{elem_short}'
                    c_struct = _c_struct_for_type(full_type)
                    codec_ns = _json_codec_namespace_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            std::vector<{cpp_type}> native;\n')
                    if is_json:
                        f.write('            const auto arr = nlohmann::json::parse(payload);\n')
                        f.write('            if (!arr.is_array()) {\n')
                        f.write('                return PCL_ERR_INVALID;\n')
                        f.write('            }\n')
                        f.write('            native.reserve(arr.size());\n')
                        f.write('            for (const auto& item : arr) {\n')
                        f.write(f'                native.push_back({codec_ns}::fromJson(\n')
                        f.write(f'                    item.dump(), static_cast<{cpp_type}*>(nullptr)));\n')
                        f.write('            }\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    f.write('            if (native.size() > std::numeric_limits<uint32_t>::max()) {\n')
                    f.write('                return PCL_ERR_INVALID;\n')
                    f.write('            }\n')
                    f.write('            auto* slice = static_cast<pyramid_slice_t*>(out_value);\n')
                    f.write('            slice->ptr = nullptr;\n')
                    f.write('            slice->len = 0u;\n')
                    f.write('            if (!native.empty()) {\n')
                    f.write(f'                auto* values = static_cast<{c_struct}*>(\n')
                    f.write(f'                    pcl_alloc(native.size() * sizeof({c_struct})));\n')
                    f.write('                if (!values) {\n')
                    f.write('                    return PCL_ERR_NOMEM;\n')
                    f.write('                }\n')
                    f.write('                for (std::size_t i = 0; i < native.size(); ++i) {\n')
                    f.write('                    pyramid::cabi::to_c(native[i], &values[i]);\n')
                    f.write('                }\n')
                    f.write('                slice->ptr = values;\n')
                    f.write('                slice->len = static_cast<uint32_t>(native.size());\n')
                    f.write('            }\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
                else:
                    cpp_type = f'{_native_namespace_for_type(full_type)}::{short}'
                    c_struct = _c_struct_for_type(full_type)
                    f.write(f'        if (std::strcmp(schema_id, "{short}") == 0) {{\n')
                    f.write(f'            {cpp_type} native;\n')
                    if is_json:
                        codec_ns = _json_codec_namespace_for_type(full_type)
                        f.write(f'            native = {codec_ns}::fromJson(\n')
                        f.write(f'                payload, static_cast<{cpp_type}*>(nullptr));\n')
                    else:
                        f.write(f'            native = wire_codec::fromBinary{short}(\n')
                        f.write('                msg->data, msg->size);\n')
                    f.write(f'            auto* cs = static_cast<{c_struct}*>(out_value);\n')
                    f.write('            pyramid::cabi::to_c(native, cs);\n')
                    f.write('            return PCL_OK;\n')
                    f.write('        }\n')
            f.write('    } catch (...) {\n')
            f.write('        return PCL_ERR_CALLBACK;\n')
            f.write('    }\n')
            f.write('    return PCL_ERR_NOT_FOUND;\n')
            f.write('}\n\n')

            f.write('static void plugin_free_msg(void* codec_ctx, pcl_msg_t* msg)\n')
            f.write('{\n')
            f.write('    (void)codec_ctx;\n')
            f.write('    if (!msg) {\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    std::free(const_cast<void*>(msg->data));\n')
            f.write('    msg->data = nullptr;\n')
            f.write('    msg->size = 0u;\n')
            f.write('    msg->type_name = nullptr;\n')
            f.write('}\n\n')

            f.write('static pcl_codec_t k_codec = {\n')
            f.write('    PCL_CODEC_ABI_VERSION,\n')
            f.write(f'    "{content_type}",\n')
            f.write('    plugin_encode,\n')
            f.write('    plugin_decode,\n')
            f.write('    plugin_free_msg,\n')
            f.write('    nullptr\n')
            f.write('};\n\n')
            f.write('// Opaque, plugin-specific configuration threaded through the loader.\n')
            f.write('// Stored here and exposed via codec_ctx so encode/decode can honor it.\n')
            f.write('static std::string k_config_json;\n\n')
            f.write('PCL_CODEC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(\n')
            f.write('    const char* config_json)\n')
            f.write('{\n')
            f.write('    k_config_json = config_json ? config_json : "";\n')
            f.write('    k_codec.codec_ctx = k_config_json.empty() ? nullptr : &k_config_json;\n')
            f.write('    return &k_codec;\n')
            f.write('}\n\n')
            f.write('} // extern "C"\n')
