#!/usr/bin/env python3
"""
gRPC Transport Backend

Generates C++ and Ada code that exposes proto services as full gRPC services,
bridging incoming gRPC calls onto the generated service facade and PCL
executor.

The C++ generated service facade owns the component-facing startup hook:

    pyramid::services::<component>::provided::buildGrpcServer(...)

The transport-specific files remain implementation glue and do not define a
parallel ServiceHandler interface.

Prerequisites:
  - protoc + grpc_cpp_plugin (build-time)
  - grpc++ (link-time)
"""

from pathlib import Path
from typing import List

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoService, ProtoRpc,
    camel_to_snake, camel_to_lower_snake, lc_first,
)
import codec_backends


# -- EntityActions operation detection ----------------------------------------

_OP_PREFIXES = ['Create', 'Read', 'Update', 'Delete']

_BASE_TYPE_MAP = {
    'pyramid.data_model.base.Identifier': 'Identifier',
    'pyramid.data_model.common.Query': 'Query',
    'pyramid.data_model.common.Ack': 'Ack',
    'pyramid.data_model.base.Ack': 'Ack',
}


def _ada_pkg_segment(segment: str) -> str:
    return '_'.join(part.capitalize() for part in segment.split('_'))


def _ada_type_name(type_name: str) -> str:
    return camel_to_snake(_short_type(type_name))


def _ada_types_pkg(type_name: str) -> str:
    parts = type_name.split('.')
    if len(parts) >= 4 and parts[0] == 'pyramid' and parts[1] == 'data_model':
        domain = _ada_pkg_segment(parts[-2])
        return f'Pyramid.Data_Model.{domain}.Types'
    return ''


def _ada_codec_pkg(type_name: str) -> str:
    parts = type_name.split('.')
    if len(parts) >= 4 and parts[0] == 'pyramid' and parts[1] == 'data_model':
        if parts[-2] == 'base':
            return ''
        domain = _ada_pkg_segment(parts[-2])
        return f'Pyramid.Data_Model.{domain}.Types_Codec'
    return ''


def _ada_type(type_name: str) -> str:
    pkg = _ada_types_pkg(type_name)
    if pkg:
        return f'{pkg}.{_ada_type_name(type_name)}'
    return _ada_type_name(type_name)


def _ada_array_type(type_name: str) -> str:
    return f'{_ada_type_name(type_name)}_Array'


def _short_type(fqn: str) -> str:
    if fqn in _BASE_TYPE_MAP:
        return _BASE_TYPE_MAP[fqn]
    return fqn.split('.')[-1]


def _rpc_op(rpc: ProtoRpc):
    """Detect CRUD operation prefix."""
    for prefix in _OP_PREFIXES:
        if rpc.name.startswith(prefix):
            return prefix, rpc.name[len(prefix):]
    return None, rpc.name


def _facade_namespace(package: str) -> str:
    parts = [p for p in package.split('.') if p]
    if 'components' in parts and 'services' in parts:
        comp_i = parts.index('components')
        svc_i = parts.index('services')
        if comp_i + 1 < svc_i and svc_i + 1 < len(parts):
            root = '::'.join(parts[:comp_i])
            component = parts[comp_i + 1]
            role = parts[svc_i + 1]
            return f'{root}::services::{component}::{role}'
    return '::'.join(parts)


def _facade_header(package: str) -> str:
    parts = [p for p in package.split('.') if p]
    if 'components' in parts and 'services' in parts:
        comp_i = parts.index('components')
        svc_i = parts.index('services')
        if comp_i + 1 < svc_i and svc_i + 1 < len(parts):
            return f'pyramid_services_{parts[comp_i + 1]}_{parts[svc_i + 1]}.hpp'
    return package.replace('.', '_') + '.hpp'


class GrpcBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'grpc'

    @property
    def content_type(self) -> str:
        return 'application/grpc'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.services:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)
            facade_ns = _facade_namespace(pf.package)
            ns = facade_ns + '::grpc_transport'

            # protoc+grpc_cpp_plugin generated header
            grpc_header = '/'.join(pf.package.split('.')) + '.grpc.pb.h'

            hpp_path = output_dir / (file_base + '_grpc_transport.hpp')
            cpp_path = output_dir / (file_base + '_grpc_transport.cpp')

            self._write_cpp_header(hpp_path, ns, facade_ns, grpc_header,
                                   _facade_header(pf.package), pf, index)
            self._write_cpp_impl(cpp_path, ns, facade_ns, file_base, pf, index)
            generated.extend([hpp_path, cpp_path])

        return generated

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.services:
                continue

            pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
            file_base = '_'.join(p.lower() for p in pkg_parts)

            spec_path = output_dir / (file_base + '-grpc_transport.ads')
            body_path = output_dir / (file_base + '-grpc_transport.adb')
            self._write_ada_spec(spec_path, pf, index)
            self._write_ada_body(body_path, pf, index)
            generated.extend([spec_path, body_path])

        return generated

    # -- C++ header ------------------------------------------------------------

    def _write_cpp_header(self, path: Path, ns: str, facade_ns: str,
                          grpc_header: str, facade_header: str,
                          pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w', newline='\n') as f:
            f.write(f'// Auto-generated gRPC transport — do not edit\n')
            f.write(f'// Backend: grpc | Namespace: {ns}\n')
            f.write(f'//\n')
            f.write(f'// Exposes proto services as gRPC services, delegating to\n')
            f.write(f'// the standard generated service facade and PCL executor.\n')
            f.write(f'//\n')
            f.write(f'// Requires: protoc + grpc_cpp_plugin generated code\n')
            f.write(f'// Link with: grpc++ protobuf\n')
            f.write(f'#pragma once\n\n')
            f.write(f'#include "{facade_header}"\n')
            f.write(f'#include "{grpc_header}"\n\n')
            f.write(f'#include <grpcpp/grpcpp.h>\n')
            f.write(f'#include <pcl/pcl_executor.h>\n')
            f.write(f'#include <memory>\n')
            f.write(f'#include <string>\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # gRPC service implementations
            for svc in pf.services:
                f.write(f'// ---------------------------------------------------------------------------\n')
                f.write(f'// {svc.name} — gRPC service implementation\n')
                f.write(f'// ---------------------------------------------------------------------------\n\n')
                f.write(f'class {svc.name}Impl final : public {proto_ns}::{svc.name}::Service {{\n')
                f.write(f'public:\n')
                f.write(f'    explicit {svc.name}Impl(pcl_executor_t* executor)\n')
                f.write(f'        : executor_(executor) {{}}\n\n')

                for rpc in svc.rpcs:
                    req_t = rpc.request_type
                    rsp_t = rpc.response_type
                    # Use fully qualified proto types in the grpc method signature
                    req_cpp = '::'.join(req_t.split('.'))
                    rsp_cpp = '::'.join(rsp_t.split('.'))

                    if rpc.server_streaming:
                        f.write(f'    grpc::Status {rpc.name}(\n')
                        f.write(f'        grpc::ServerContext* context,\n')
                        f.write(f'        const ::{req_cpp}* request,\n')
                        f.write(f'        grpc::ServerWriter<::{rsp_cpp}>* writer) override;\n\n')
                    else:
                        f.write(f'    grpc::Status {rpc.name}(\n')
                        f.write(f'        grpc::ServerContext* context,\n')
                        f.write(f'        const ::{req_cpp}* request,\n')
                        f.write(f'        ::{rsp_cpp}* response) override;\n\n')

                f.write(f'private:\n')
                f.write(f'    pcl_executor_t* executor_;\n')
                f.write(f'}};\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- C++ implementation ----------------------------------------------------

    def _write_cpp_impl(self, path: Path, ns: str, facade_ns: str, file_base: str,
                        pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w', newline='\n') as f:
            f.write(f'// Auto-generated gRPC transport — do not edit\n\n')
            f.write(f'#include "{file_base}_grpc_transport.hpp"\n\n')
            f.write(f'#include <pcl/pcl_types.h>\n\n')
            f.write(f'#include <cstdint>\n')
            f.write(f'#include <future>\n')
            f.write(f'#include <memory>\n')
            f.write(f'#include <stdexcept>\n')
            f.write(f'#include <string>\n')
            f.write(f'#include <vector>\n\n')

            f.write('namespace {\n\n')
            f.write('struct ExecutorResponse {\n')
            f.write('    pcl_status_t status = PCL_OK;\n')
            f.write('    std::string content_type;\n')
            f.write('    std::string payload;\n')
            f.write('};\n\n')
            f.write('struct UnaryState {\n')
            f.write('    std::promise<ExecutorResponse> promise;\n')
            f.write('};\n\n')
            f.write('void executor_response_callback(const pcl_msg_t* response, void* user_data)\n')
            f.write('{\n')
            f.write('    auto* state = static_cast<UnaryState*>(user_data);\n')
            f.write('    if (!state) {\n')
            f.write('        return;\n')
            f.write('    }\n')
            f.write('    ExecutorResponse out;\n')
            f.write('    if (response && response->type_name) {\n')
            f.write('        out.content_type = response->type_name;\n')
            f.write('    }\n')
            f.write('    if (response && response->data && response->size > 0U) {\n')
            f.write('        const auto* bytes = static_cast<const char*>(response->data);\n')
            f.write('        out.payload.assign(bytes, bytes + response->size);\n')
            f.write('    }\n')
            f.write('    state->promise.set_value(std::move(out));\n')
            f.write('}\n\n')
            f.write('ExecutorResponse invoke_executor(pcl_executor_t* executor,\n')
            f.write('                                 const char* service_name,\n')
            f.write('                                 const std::string& request_payload)\n')
            f.write('{\n')
            f.write('    ExecutorResponse error;\n')
            f.write('    if (!executor || !service_name || !service_name[0]) {\n')
            f.write('        error.status = PCL_ERR_INVALID;\n')
            f.write('        return error;\n')
            f.write('    }\n')
            f.write('    UnaryState state;\n')
            f.write('    pcl_msg_t request{};\n')
            f.write('    request.data = request_payload.empty() ? nullptr : const_cast<char*>(request_payload.data());\n')
            f.write('    request.size = static_cast<uint32_t>(request_payload.size());\n')
            f.write(f'    request.type_name = {facade_ns}::kProtobufContentType;\n')
            f.write('    const auto rc = pcl_executor_post_service_request(\n')
            f.write('        executor, service_name, &request, executor_response_callback, &state);\n')
            f.write('    if (rc != PCL_OK) {\n')
            f.write('        error.status = rc;\n')
            f.write('        return error;\n')
            f.write('    }\n')
            f.write('    return state.promise.get_future().get();\n')
            f.write('}\n\n')
            f.write('std::string serialize_grpc_request(const google::protobuf::MessageLite& request)\n')
            f.write('{\n')
            f.write('    std::string payload;\n')
            f.write('    if (!request.SerializeToString(&payload)) {\n')
            f.write('        throw std::runtime_error("could not serialize gRPC request");\n')
            f.write('    }\n')
            f.write('    return payload;\n')
            f.write('}\n\n')
            f.write('template <typename ResponsePb>\n')
            f.write('grpc::Status parse_unary_response(const ExecutorResponse& response,\n')
            f.write('                                  ResponsePb* out,\n')
            f.write('                                  const char* rpc_name)\n')
            f.write('{\n')
            f.write('    if (response.status != PCL_OK) {\n')
            f.write('        return grpc::Status(grpc::StatusCode::INTERNAL, "executor service dispatch failed");\n')
            f.write('    }\n')
            f.write('    if (!out || !out->ParseFromArray(response.payload.data(), static_cast<int>(response.payload.size()))) {\n')
            f.write('        return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse response for ") + rpc_name);\n')
            f.write('    }\n')
            f.write('    return grpc::Status::OK;\n')
            f.write('}\n\n')
            f.write('bool read_varint32(const char*& cursor, const char* end, uint32_t& value)\n')
            f.write('{\n')
            f.write('    value = 0U;\n')
            f.write('    int shift = 0;\n')
            f.write('    while (cursor < end && shift <= 28) {\n')
            f.write('        const auto byte = static_cast<unsigned char>(*cursor++);\n')
            f.write('        value |= static_cast<uint32_t>(byte & 0x7FU) << shift;\n')
            f.write('        if ((byte & 0x80U) == 0U) {\n')
            f.write('            return true;\n')
            f.write('        }\n')
            f.write('        shift += 7;\n')
            f.write('    }\n')
            f.write('    return false;\n')
            f.write('}\n\n')
            f.write('template <typename ResponsePb, typename WriterT>\n')
            f.write('grpc::Status write_stream_response(const ExecutorResponse& response,\n')
            f.write('                                  WriterT* writer,\n')
            f.write('                                  const char* rpc_name)\n')
            f.write('{\n')
            f.write('    if (response.status != PCL_OK) {\n')
            f.write('        return grpc::Status(grpc::StatusCode::INTERNAL, "executor stream dispatch failed");\n')
            f.write('    }\n')
            f.write('    const char* cursor = response.payload.data();\n')
            f.write('    const char* end = cursor + response.payload.size();\n')
            f.write('    while (cursor < end) {\n')
            f.write('        uint32_t frame_size = 0U;\n')
            f.write('        if (!read_varint32(cursor, end, frame_size) || static_cast<size_t>(end - cursor) < frame_size) {\n')
            f.write('            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("invalid stream frame for ") + rpc_name);\n')
            f.write('        }\n')
            f.write('        ResponsePb item;\n')
            f.write('        if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {\n')
            f.write('            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse stream frame for ") + rpc_name);\n')
            f.write('        }\n')
            f.write('        cursor += frame_size;\n')
            f.write('        if (!writer->Write(item)) {\n')
            f.write('            return grpc::Status(grpc::StatusCode::CANCELLED, "client cancelled gRPC stream");\n')
            f.write('        }\n')
            f.write('    }\n')
            f.write('    return grpc::Status::OK;\n')
            f.write('}\n\n')
            f.write('} // namespace\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # gRPC method implementations
            for svc in pf.services:
                for rpc in svc.rpcs:
                    req_cpp = '::'.join(rpc.request_type.split('.'))
                    rsp_cpp = '::'.join(rpc.response_type.split('.'))

                    if rpc.server_streaming:
                        f.write(f'grpc::Status {svc.name}Impl::{rpc.name}(\n')
                        f.write(f'    grpc::ServerContext* /*context*/,\n')
                        f.write(f'    const ::{req_cpp}* request,\n')
                        f.write(f'    grpc::ServerWriter<::{rsp_cpp}>* writer)\n')
                        f.write(f'{{\n')
                        f.write(f'    // Delegate to handler, write each result to stream\n')
                        f.write(f'    auto results = handler_.handle{rpc.name}(*request);\n')
                        f.write(f'    for (const auto& item : results) {{\n')
                        f.write(f'        writer->Write(item);\n')
                        f.write(f'    }}\n')
                        f.write(f'    return grpc::Status::OK;\n')
                        f.write(f'}}\n\n')
                    else:
                        f.write(f'grpc::Status {svc.name}Impl::{rpc.name}(\n')
                        f.write(f'    grpc::ServerContext* /*context*/,\n')
                        f.write(f'    const ::{req_cpp}* request,\n')
                        f.write(f'    ::{rsp_cpp}* response)\n')
                        f.write(f'{{\n')
                        f.write(f'    *response = handler_.handle{rpc.name}(*request);\n')
                        f.write(f'    return grpc::Status::OK;\n')
                        f.write(f'}}\n\n')

            # buildServer
            f.write(f'std::unique_ptr<grpc::Server> buildServer(\n')
            f.write(f'    const std::string& listen_address,\n')
            f.write(f'    ServiceHandler& handler)\n')
            f.write(f'{{\n')
            f.write(f'    grpc::ServerBuilder builder;\n')
            f.write(f'    builder.AddListeningPort(listen_address,'
                    f' grpc::InsecureServerCredentials());\n\n')

            for svc in pf.services:
                f.write(f'    auto {camel_to_lower_snake(svc.name)} ='
                        f' std::make_unique<{svc.name}Impl>(handler);\n')
                f.write(f'    builder.RegisterService({camel_to_lower_snake(svc.name)}.get());\n\n')

            f.write(f'    return builder.BuildAndStart();\n')
            f.write(f'}}\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- Ada spec --------------------------------------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.GRPC_Transport'
        package_role = pf.package.split('.')[-1]
        type_packages = sorted({
            _ada_types_pkg(t)
            for svc in pf.services
            for rpc in svc.rpcs
            for t in (rpc.request_type, rpc.response_type)
            if _ada_types_pkg(t)
        })
        streaming_responses = []
        seen_streaming = set()
        for svc in pf.services:
            for rpc in svc.rpcs:
                if rpc.server_streaming:
                    name = _ada_array_type(rpc.response_type)
                    if name not in seen_streaming:
                        seen_streaming.add(name)
                        streaming_responses.append((name, _ada_type(rpc.response_type)))

        with open(path, 'w', newline='\n') as f:
            f.write(f'--  Auto-generated gRPC transport spec — do not edit\n')
            f.write(f'--  Backend: grpc | Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  Component-facing calls are typed; the JSON/C ABI shim is private.\n\n')
            for pkg in type_packages:
                f.write(f'with {pkg};\n')
            f.write('\n')
            f.write(f'package {pkg_name} is\n\n')
            f.write(f'   Content_Type : constant String := "application/grpc";\n\n')
            f.write(f'   procedure Configure_Library (Path : String);\n\n')

            for array_name, element_type in streaming_responses:
                f.write(f'   type {array_name} is array (Positive range <>) of {element_type};\n')
            if streaming_responses:
                f.write('\n')

            for svc in pf.services:
                f.write(f'   --  {svc.name}\n\n')
                for rpc in svc.rpcs:
                    ada_rpc = camel_to_snake(rpc.name)
                    req_t = _ada_type(rpc.request_type)
                    rsp_t = (_ada_array_type(rpc.response_type)
                             if rpc.server_streaming
                             else _ada_type(rpc.response_type))
                    f.write(f'   function Invoke_{ada_rpc}\n')
                    f.write(f'     (Channel : String;\n')
                    f.write(f'      Request : {req_t})\n')
                    f.write(f'      return {rsp_t};\n\n')

            f.write(f'end {pkg_name};\n')

    def _write_ada_body(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.GRPC_Transport'
        package_role = pf.package.split('.')[-1]
        type_packages = sorted({
            _ada_types_pkg(t)
            for svc in pf.services
            for rpc in svc.rpcs
            for t in (rpc.request_type, rpc.response_type)
            if _ada_types_pkg(t)
        })
        codec_packages = sorted({
            _ada_codec_pkg(t)
            for svc in pf.services
            for rpc in svc.rpcs
            for t in (rpc.request_type, rpc.response_type)
            if _ada_codec_pkg(t)
        })

        def encode_expr(type_name: str, value: str) -> str:
            if _short_type(type_name) == 'Identifier':
                return f'Encode_Identifier ({value})'
            codec_pkg = _ada_codec_pkg(type_name)
            return f'{codec_pkg}.To_Json ({value})'

        def decode_expr(type_name: str, response: str, streaming: bool) -> str:
            if streaming:
                return f'Decode_{_ada_array_type(type_name)} ({response})'
            if _short_type(type_name) == 'Identifier':
                return f'Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String ({response}))'
            codec_pkg = _ada_codec_pkg(type_name)
            return f'{codec_pkg}.From_Json ({response}, null)'

        with open(path, 'w', newline='\n') as f:
            f.write(f'--  Auto-generated gRPC transport body — do not edit\n\n')
            f.write(f'with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;\n')
            f.write(f'with Ada.Unchecked_Conversion;\n')
            f.write(f'with Interfaces.C.Strings;\n')
            f.write(f'with GNATCOLL.JSON; use GNATCOLL.JSON;\n')
            f.write(f'with System;\n')
            for pkg in type_packages:
                f.write(f'with {pkg};\n')
            for pkg in codec_packages:
                f.write(f'with {pkg};\n')
            f.write('\n')
            f.write(f'package body {pkg_name} is\n\n')
            f.write('   use type Interfaces.C.Strings.chars_ptr;\n\n')
            f.write('   use type System.Address;\n\n')

            f.write('   type Invoke_Json_Access is access function\n')
            f.write('     (Channel : Interfaces.C.Strings.chars_ptr;\n')
            f.write('      Request : Interfaces.C.Strings.chars_ptr)\n')
            f.write('      return Interfaces.C.Strings.chars_ptr;\n')
            f.write('   pragma Convention (C, Invoke_Json_Access);\n\n')

            f.write('   type Free_String_Access is access procedure\n')
            f.write('     (Value : Interfaces.C.Strings.chars_ptr);\n')
            f.write('   pragma Convention (C, Free_String_Access);\n\n')

            f.write('   function To_Invoke_Json is new Ada.Unchecked_Conversion\n')
            f.write('     (System.Address, Invoke_Json_Access);\n')
            f.write('   function To_Free_String is new Ada.Unchecked_Conversion\n')
            f.write('     (System.Address, Free_String_Access);\n\n')

            f.write('   function Load_Library_A (Name : Interfaces.C.Strings.chars_ptr)\n')
            f.write('     return System.Address\n')
            f.write('     with Import, Convention => C, External_Name => "LoadLibraryA";\n\n')

            f.write('   function Get_Proc_Address\n')
            f.write('     (Module : System.Address; Name : Interfaces.C.Strings.chars_ptr)\n')
            f.write('      return System.Address\n')
            f.write('     with Import, Convention => C, External_Name => "GetProcAddress";\n\n')

            f.write('   Library_Path : Unbounded_String :=\n')
            f.write('     To_Unbounded_String ("pyramid_grpc_c_shim.dll");\n')
            f.write('   Library_Module : System.Address := System.Null_Address;\n\n')

            f.write('   procedure Configure_Library (Path : String) is\n')
            f.write('   begin\n')
            f.write('      Library_Path := To_Unbounded_String (Path);\n')
            f.write('      Library_Module := System.Null_Address;\n')
            f.write('   end Configure_Library;\n\n')

            f.write('   function Ensure_Library return System.Address is\n')
            f.write('      Path_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.New_String (To_String (Library_Path));\n')
            f.write('   begin\n')
            f.write('      if Library_Module = System.Null_Address then\n')
            f.write('         Library_Module := Load_Library_A (Path_C);\n')
            f.write('      end if;\n')
            f.write('      Interfaces.C.Strings.Free (Path_C);\n')
            f.write('      if Library_Module = System.Null_Address then\n')
            f.write('         raise Program_Error with "could not load gRPC C shim: " & To_String (Library_Path);\n')
            f.write('      end if;\n')
            f.write('      return Library_Module;\n')
            f.write('   end Ensure_Library;\n\n')

            f.write('   function Symbol_Address (Name : String) return System.Address is\n')
            f.write('      Name_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.New_String (Name);\n')
            f.write('      Result : constant System.Address :=\n')
            f.write('        Get_Proc_Address (Ensure_Library, Name_C);\n')
            f.write('   begin\n')
            f.write('      Interfaces.C.Strings.Free (Name_C);\n')
            f.write('      if Result = System.Null_Address then\n')
            f.write('         raise Program_Error with "gRPC C shim symbol not found: " & Name;\n')
            f.write('      end if;\n')
            f.write('      return Result;\n')
            f.write('   end Symbol_Address;\n\n')

            f.write('   function Normalise_Json_String (Value : String) return String is\n')
            f.write('   begin\n')
            f.write('      if Value\'Length >= 2\n')
            f.write("        and then Value (Value'First) = '\"'\n")
            f.write("        and then Value (Value'Last) = '\"'\n")
            f.write('      then\n')
            f.write('         return Value (Value\'First + 1 .. Value\'Last - 1);\n')
            f.write('      end if;\n')
            f.write('      return Value;\n')
            f.write('   end Normalise_Json_String;\n\n')

            f.write('   function Encode_Identifier\n')
            f.write('     (Value : Pyramid.Data_Model.Base.Types.Identifier) return String is\n')
            f.write('   begin\n')
            f.write('      return """\" & Ada.Strings.Unbounded.To_String (Value) & """\";\n')
            f.write('   end Encode_Identifier;\n\n')

            f.write('   function Call_Json\n')
            f.write('     (Channel : String;\n')
            f.write('      Request_Json : String;\n')
            f.write('      Invoke : Invoke_Json_Access)\n')
            f.write('      return String\n')
            f.write('   is\n')
            f.write('      Channel_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.New_String (Channel);\n')
            f.write('      Request_C : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.New_String (Request_Json);\n')
            f.write('      Response_Ptr : Interfaces.C.Strings.chars_ptr :=\n')
            f.write('        Interfaces.C.Strings.Null_Ptr;\n')
            f.write('      Response_Text : Unbounded_String := Null_Unbounded_String;\n')
            f.write('   begin\n')
            f.write('      Response_Ptr := Invoke (Channel_C, Request_C);\n')
            f.write('      Interfaces.C.Strings.Free (Channel_C);\n')
            f.write('      Interfaces.C.Strings.Free (Request_C);\n')
            f.write('      if Response_Ptr /= Interfaces.C.Strings.Null_Ptr then\n')
            f.write('         Response_Text := To_Unbounded_String\n')
            f.write('           (Interfaces.C.Strings.Value (Response_Ptr));\n')
            f.write('         To_Free_String\n')
            f.write('           (Symbol_Address ("pyramid_services_tactical_objects_grpc_free_string"))\n')
            f.write('           (Response_Ptr);\n')
            f.write('      end if;\n')
            f.write('      return To_String (Response_Text);\n')
            f.write('   end Call_Json;\n\n')

            streaming_done = set()
            for svc in pf.services:
                for rpc in svc.rpcs:
                    if not rpc.server_streaming:
                        continue
                    array_t = _ada_array_type(rpc.response_type)
                    if array_t in streaming_done:
                        continue
                    streaming_done.add(array_t)
                    elem_t = _ada_type(rpc.response_type)
                    codec_pkg = _ada_codec_pkg(rpc.response_type)
                    f.write(f'   function Decode_{array_t} (Response_Json : String) return {array_t} is\n')
                    f.write('      J : constant JSON_Value := Read (Response_Json);\n')
                    f.write('      Arr : constant JSON_Array := Get (J);\n')
                    f.write('      Len : constant Natural := Length (Arr);\n')
                    f.write(f'      Result : {array_t} (1 .. Len);\n')
                    f.write('   begin\n')
                    f.write('      for I in Result\'Range loop\n')
                    f.write(f'         Result (I) := {codec_pkg}.From_Json (Write (Get (Arr, I)), null);\n')
                    f.write('      end loop;\n')
                    f.write('      return Result;\n')
                    f.write(f'   end Decode_{array_t};\n\n')

            for svc in pf.services:
                for rpc in svc.rpcs:
                    ada_rpc = camel_to_snake(rpc.name)
                    req_t = _ada_type(rpc.request_type)
                    rsp_t = (_ada_array_type(rpc.response_type)
                             if rpc.server_streaming
                             else _ada_type(rpc.response_type))
                    f.write(f'   function Invoke_{ada_rpc}\n')
                    f.write(f'     (Channel : String;\n')
                    f.write(f'      Request : {req_t})\n')
                    f.write(f'      return {rsp_t}\n')
                    f.write('   is\n')
                    f.write('      Response_Json : constant String :=\n')
                    svc_name = camel_to_lower_snake(svc.name)
                    rpc_name = camel_to_lower_snake(rpc.name)
                    symbol_name = f'grpc_{package_role}_{svc_name}_{rpc_name}_json'
                    f.write(f'        Call_Json (Channel, {encode_expr(rpc.request_type, "Request")},\n')
                    f.write(f'                   To_Invoke_Json (Symbol_Address ("{symbol_name}")));\n')
                    f.write('   begin\n')
                    f.write(f'      return {decode_expr(rpc.response_type, "Response_Json", rpc.server_streaming)};\n')
                    f.write(f'   end Invoke_{ada_rpc};\n\n')

            f.write(f'end {pkg_name};\n')


# -- Register ------------------------------------------------------------------

codec_backends.register(GrpcBackend())
