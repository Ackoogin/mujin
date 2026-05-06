#!/usr/bin/env python3
"""
gRPC Transport Backend

Generates C++ and Ada code that exposes proto services as full gRPC services,
bridging incoming gRPC calls onto the generated service facade and PCL
executor.

The C++ generated service facade owns the component-facing startup hook:

    pyramid::components::<component>::services::provided::buildGrpcServer(...)

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


_PYRAMID_ROOT_DIR = Path(__file__).resolve().parents[2]


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
    return '::'.join(p for p in package.split('.') if p)


def _facade_header(package: str) -> str:
    parts = [p for p in package.split('.') if p]
    if 'components' in parts and 'services' in parts:
        comp_i = parts.index('components')
        svc_i = parts.index('services')
        if comp_i + 1 < svc_i and svc_i + 1 < len(parts):
                return f'pyramid_services_{parts[comp_i + 1]}_{parts[svc_i + 1]}.hpp'
    return package.replace('.', '_') + '.hpp'


def _service_wire_prefix(service_name: str) -> str:
    name = service_name
    if name.endswith('_Service'):
        name = name[:-len('_Service')]
    return camel_to_lower_snake(name)


def _component_name(package: str) -> str:
    parts = [p for p in package.split('.') if p]
    if 'components' in parts and 'services' in parts:
        comp_i = parts.index('components')
        svc_i = parts.index('services')
        if comp_i + 1 < svc_i:
            return parts[comp_i + 1]
    return ''


def _package_role(package: str) -> str:
    parts = [p for p in package.split('.') if p]
    return parts[-1] if parts else ''


def _protobuf_pb_header(type_name: str) -> str:
    parts = [p for p in type_name.split('.') if p]
    if len(parts) < 2:
        return ''
    return '/'.join(parts[:-1]) + '.pb.h'


def _protobuf_shim_header(component: str) -> str:
    return f'pyramid_services_{component}_protobuf_shim.h'


def _protobuf_shim_prefix(component: str) -> str:
    return f'pyramid_services_{component}'


def _grpc_c_api_support_base(component: str) -> str:
    return f'pyramid_services_{component}_grpc_c_api_support'


def _has_protobuf_json_shim(component: str) -> bool:
    if not component:
        return False
    shim_header = (_PYRAMID_ROOT_DIR / 'bindings' / 'protobuf' / 'cpp' /
                   _protobuf_shim_header(component))
    return shim_header.exists()


def _ada_grpc_rpc_name(service_name: str, rpc_name: str) -> str:
    return f'{_service_wire_prefix(service_name)}_{camel_to_snake(rpc_name)}'


def _grpc_pb_header_path(pf: ProtoFile) -> str:
    proto_path = Path(pf.path)
    proto_root = _PYRAMID_ROOT_DIR / 'proto'
    try:
        proto_path = proto_path.relative_to(proto_root)
    except ValueError:
        pass
    return str(proto_path.with_suffix('.grpc.pb.h')).replace('\\', '/')


def _ensure_parent_packages(output_dir: Path, pkg_names: List[str]) -> None:
    """Generate empty parent package specs required by Ada child packages.

    For a package like Pyramid.Components.Foo.Services.Provided.GRPC_Transport,
    Ada requires that every ancestor (Pyramid.Components, Pyramid.Components.Foo,
    etc.) has a corresponding .ads file.  This function collects all such
    ancestors from *pkg_names* and writes trivial specs for any that don't
    already exist in *output_dir*.
    """
    needed: dict[str, None] = {}          # ordered set via dict
    for pkg in pkg_names:
        parts = pkg.split('.')
        for i in range(1, len(parts)):    # skip full name, keep ancestors
            ancestor = '.'.join(parts[:i])
            needed[ancestor] = None

    for ancestor in needed:
        file_base = ancestor.lower().replace('.', '-')
        ads = output_dir / (file_base + '.ads')
        if ads.exists():
            continue
        with open(ads, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated parent package spec (empty)\n')
            f.write(f'package {ancestor} is\n')
            f.write(f'end {ancestor};\n')
        print(f'  Generated parent spec {ancestor}')


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
        generated_c_api_support = set()

        for pf in index.files:
            if not pf.services:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)
            facade_ns = _facade_namespace(pf.package)
            ns = facade_ns + '::grpc_transport'

            # protoc+grpc_cpp_plugin generated header
            grpc_header = _grpc_pb_header_path(pf)

            hpp_path = output_dir / (file_base + '_grpc_transport.hpp')
            cpp_path = output_dir / (file_base + '_grpc_transport.cpp')

            self._write_cpp_header(hpp_path, ns, facade_ns, grpc_header,
                                   _facade_header(pf.package), pf, index)
            self._write_cpp_impl(cpp_path, ns, facade_ns, file_base, pf, index)
            generated.extend([hpp_path, cpp_path])

            component = _component_name(pf.package)
            if _has_protobuf_json_shim(component):
                support_base = _grpc_c_api_support_base(component)
                if component not in generated_c_api_support:
                    support_hpp = output_dir / f'{support_base}.hpp'
                    support_cpp = output_dir / f'{support_base}.cpp'
                    self._write_cpp_c_api_support_header(support_hpp, component)
                    self._write_cpp_c_api_support_impl(support_cpp, component)
                    generated.extend([support_hpp, support_cpp])
                    generated_c_api_support.add(component)

                c_api_hpp = output_dir / (file_base + '_grpc_c_api.hpp')
                c_api_cpp = output_dir / (file_base + '_grpc_c_api.cpp')
                self._write_cpp_c_api_header(c_api_hpp, pf, component)
                self._write_cpp_c_api_impl(c_api_cpp, pf, component)
                generated.extend([c_api_hpp, c_api_cpp])

        return generated

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []
        generated_pkgs = []

        for pf in index.files:
            if not pf.services:
                continue

            pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
            pkg_name = '.'.join(pkg_parts) + '.GRPC_Transport'
            # Use dashes for Ada file naming convention (Pyramid.Foo.Bar -> pyramid-foo-bar)
            file_base = '-'.join(p.lower() for p in pkg_parts)

            spec_path = output_dir / (file_base + '-grpc_transport.ads')
            body_path = output_dir / (file_base + '-grpc_transport.adb')
            self._write_ada_spec(spec_path, pf, index)
            self._write_ada_body(body_path, pf, index)
            generated.extend([spec_path, body_path])
            generated_pkgs.append(pkg_name)

        # Generate empty parent package stubs required by Ada child packages
        _ensure_parent_packages(output_dir, generated_pkgs)

        return generated

    # -- C++ header ------------------------------------------------------------

    def _write_cpp_header(self, path: Path, ns: str, facade_ns: str,
                          grpc_header: str, facade_header: str,
                          pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'// Auto-generated gRPC transport -- do not edit\n')
            f.write(f'// Backend: grpc | Namespace: {ns}\n')
            f.write(f'//\n')
            f.write(f'// Exposes proto services as gRPC services, delegating to\n')
            f.write(f'// the standard generated service facade and PCL executor.\n')
            f.write(f'//\n')
            f.write(f'// Requires: protoc + grpc_cpp_plugin generated code\n')
            f.write(f'// Link with: grpc++ protobuf\n')
            f.write(f'#pragma once\n\n')
            f.write(f'#include "{grpc_header}"\n\n')
            f.write(f'#include <grpcpp/grpcpp.h>\n')
            f.write(f'#include <pcl/pcl_executor.h>\n')
            f.write(f'#include <memory>\n')
            f.write(f'#include <string>\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # gRPC service implementations
            for svc in pf.services:
                f.write(f'// ---------------------------------------------------------------------------\n')
                f.write(f'// {svc.name} -- gRPC service implementation\n')
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

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'// Auto-generated gRPC transport -- do not edit\n\n')
            f.write(f'#include "{file_base}_grpc_transport.hpp"\n\n')
            f.write(f'#include <google/protobuf/message_lite.h>\n')
            f.write(f'#include <pcl/pcl_types.h>\n\n')
            f.write(f'#include <cstdint>\n')
            f.write(f'#include <future>\n')
            f.write(f'#include <memory>\n')
            f.write(f'#include <stdexcept>\n')
            f.write(f'#include <string>\n')
            f.write(f'#include <utility>\n')
            f.write(f'#include <vector>\n\n')

            f.write('namespace {\n\n')
            f.write('constexpr const char* kProtobufContentType = "application/protobuf";\n\n')
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
            f.write('    request.data = request_payload.empty() ? nullptr : request_payload.data();\n')
            f.write('    request.size = static_cast<uint32_t>(request_payload.size());\n')
            f.write('    request.type_name = kProtobufContentType;\n')
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
            f.write('    if (!writer) {\n')
            f.write('        return grpc::Status(grpc::StatusCode::INTERNAL, "missing gRPC stream writer");\n')
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
                    service_name = f'{_service_wire_prefix(svc.name)}.{camel_to_lower_snake(rpc.name)}'

                    if rpc.server_streaming:
                        f.write(f'grpc::Status {svc.name}Impl::{rpc.name}(\n')
                        f.write(f'    grpc::ServerContext* /*context*/,\n')
                        f.write(f'    const ::{req_cpp}* request,\n')
                        f.write(f'    grpc::ServerWriter<::{rsp_cpp}>* writer)\n')
                        f.write(f'{{\n')
                        f.write(f'    try {{\n')
                        f.write(f'        if (!request) {{\n')
                        f.write(f'            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");\n')
                        f.write(f'        }}\n')
                        f.write(f'        const auto payload = serialize_grpc_request(*request);\n')
                        f.write(f'        const auto executor_response = invoke_executor(\n')
                        f.write(f'            executor_, "{service_name}", payload);\n')
                        f.write(f'        return write_stream_response<::{rsp_cpp}>(\n')
                        f.write(f'            executor_response, writer, "{rpc.name}");\n')
                        f.write(f'    }} catch (const std::exception& ex) {{\n')
                        f.write(f'        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());\n')
                        f.write(f'    }}\n')
                        f.write(f'}}\n\n')
                    else:
                        f.write(f'grpc::Status {svc.name}Impl::{rpc.name}(\n')
                        f.write(f'    grpc::ServerContext* /*context*/,\n')
                        f.write(f'    const ::{req_cpp}* request,\n')
                        f.write(f'    ::{rsp_cpp}* response)\n')
                        f.write(f'{{\n')
                        f.write(f'    try {{\n')
                        f.write(f'        if (!request) {{\n')
                        f.write(f'            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");\n')
                        f.write(f'        }}\n')
                        f.write(f'        const auto payload = serialize_grpc_request(*request);\n')
                        f.write(f'        const auto executor_response = invoke_executor(\n')
                        f.write(f'            executor_, "{service_name}", payload);\n')
                        f.write(f'        return parse_unary_response(\n')
                        f.write(f'            executor_response, response, "{rpc.name}");\n')
                        f.write(f'    }} catch (const std::exception& ex) {{\n')
                        f.write(f'        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());\n')
                        f.write(f'    }}\n')
                        f.write(f'}}\n\n')

            f.write(f'}} // namespace {ns}\n')
            f.write(f'\nnamespace {facade_ns} {{\n\n')
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
            f.write('struct GrpcServer::Impl {\n')
            for svc in pf.services:
                member = camel_to_lower_snake(svc.name)
                f.write(f'    std::unique_ptr<grpc_transport::{svc.name}Impl> {member};\n')
            f.write('    std::unique_ptr<grpc::Server> server;\n')
            f.write('};\n\n')
            f.write('GrpcServer::GrpcServer() = default;\n\n')
            f.write('GrpcServer::GrpcServer(std::unique_ptr<Impl> impl)\n')
            f.write('    : impl_(std::move(impl))\n')
            f.write('{}\n\n')
            f.write('GrpcServer::GrpcServer(GrpcServer&&) noexcept = default;\n\n')
            f.write('GrpcServer& GrpcServer::operator=(GrpcServer&&) noexcept = default;\n\n')
            f.write('GrpcServer::~GrpcServer() = default;\n\n')
            f.write('bool GrpcServer::started() const\n')
            f.write('{\n')
            f.write('    return impl_ && impl_->server != nullptr;\n')
            f.write('}\n\n')
            f.write('void GrpcServer::wait()\n')
            f.write('{\n')
            f.write('    if (started()) {\n')
            f.write('        impl_->server->Wait();\n')
            f.write('    }\n')
            f.write('}\n\n')
            f.write('void GrpcServer::shutdown()\n')
            f.write('{\n')
            f.write('    if (started()) {\n')
            f.write('        impl_->server->Shutdown();\n')
            f.write('    }\n')
            f.write('}\n\n')
            f.write('GrpcServer buildGrpcServer(const std::string& listen_address,\n')
            f.write('                           pcl_executor_t* executor)\n')
            f.write('{\n')
            f.write('    if (!executor) {\n')
            f.write('        return {};\n')
            f.write('    }\n\n')
            f.write('    auto impl = std::make_unique<GrpcServer::Impl>();\n')
            f.write('    grpc::ServerBuilder builder;\n')
            f.write('    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());\n\n')
            for svc in pf.services:
                member = camel_to_lower_snake(svc.name)
                f.write(f'    impl->{member} = std::make_unique<grpc_transport::{svc.name}Impl>(executor);\n')
                f.write(f'    builder.RegisterService(impl->{member}.get());\n\n')
            f.write('    impl->server = builder.BuildAndStart();\n')
            f.write('    if (!impl->server) {\n')
            f.write('        return {};\n')
            f.write('    }\n')
            f.write('    return GrpcServer(std::move(impl));\n')
            f.write('}\n\n')
            f.write(f'}} // namespace {facade_ns}\n')

    def _write_cpp_c_api_support_header(self, path: Path, component: str):
        support_ns = f'pyramid::services::{component}::grpc_transport::c_api'
        free_string_name = f'pyramid_services_{component}_grpc_free_string'

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#pragma once\n\n')
            f.write('#include <grpcpp/grpcpp.h>\n')
            f.write('#include <nlohmann/json.hpp>\n\n')
            f.write('#include <cstdlib>\n')
            f.write('#include <cstring>\n')
            f.write('#include <memory>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <string>\n')
            f.write('#include <utility>\n\n')
            f.write('#if defined(_WIN32)\n')
            f.write('#define PYRAMID_GRPC_C_SHIM_EXPORT __declspec(dllexport)\n')
            f.write('#else\n')
            f.write('#define PYRAMID_GRPC_C_SHIM_EXPORT\n')
            f.write('#endif\n\n')
            f.write(f'namespace {support_ns} {{\n\n')
            f.write('using EncodeJsonFn = void* (*)(const char*, size_t*);\n')
            f.write('using DecodeJsonFn = char* (*)(const void*, size_t);\n\n')
            f.write('inline char* copyCString(const std::string& value) {\n')
            f.write('  char* out = static_cast<char*>(std::malloc(value.size() + 1));\n')
            f.write('  if (out == nullptr) {\n')
            f.write('    return nullptr;\n')
            f.write('  }\n')
            f.write('  std::memcpy(out, value.c_str(), value.size() + 1);\n')
            f.write('  return out;\n')
            f.write('}\n\n')
            f.write('inline char* errorCString(const std::string& message) {\n')
            f.write('  return copyCString("ERROR:" + message);\n')
            f.write('}\n\n')
            f.write('template <typename RequestPb>\n')
            f.write('bool parseRequest(const char* request_json, EncodeJsonFn encode_json,\n')
            f.write('                  RequestPb* request, std::string* error_message) {\n')
            f.write('  if (!request_json || !request || !error_message) {\n')
            f.write('    return false;\n')
            f.write('  }\n\n')
            f.write('  size_t request_size = 0;\n')
            f.write('  std::unique_ptr<void, decltype(&std::free)> request_bytes(\n')
            f.write('      encode_json(request_json, &request_size), &std::free);\n')
            f.write('  if (!request_bytes || request_size == 0) {\n')
            f.write('    *error_message = "encode-request";\n')
            f.write('    return false;\n')
            f.write('  }\n')
            f.write('  if (!request->ParseFromArray(request_bytes.get(),\n')
            f.write('                               static_cast<int>(request_size))) {\n')
            f.write('    *error_message = "parse-request";\n')
            f.write('    return false;\n')
            f.write('  }\n')
            f.write('  return true;\n')
            f.write('}\n\n')
            f.write('template <typename ResponsePb>\n')
            f.write('char* renderUnaryResponse(const ResponsePb& response, DecodeJsonFn decode_json) {\n')
            f.write('  std::string response_bytes;\n')
            f.write('  if (!response.SerializeToString(&response_bytes)) {\n')
            f.write('    return errorCString("serialize-response");\n')
            f.write('  }\n')
            f.write('  char* response_json = decode_json(response_bytes.data(), response_bytes.size());\n')
            f.write('  if (!response_json) {\n')
            f.write('    return errorCString("decode-response");\n')
            f.write('  }\n')
            f.write('  return response_json;\n')
            f.write('}\n\n')
            f.write('template <typename RequestPb, typename ResponsePb, typename StubFactory,\n')
            f.write('          typename RpcInvoker>\n')
            f.write('char* invokeUnaryJsonRpc(const char* endpoint, const char* request_json,\n')
            f.write('                         EncodeJsonFn encode_json, DecodeJsonFn decode_json,\n')
            f.write('                         StubFactory stub_factory, RpcInvoker rpc_invoker) {\n')
            f.write('  if (!endpoint || !request_json) {\n')
            f.write('    return errorCString("null-argument");\n')
            f.write('  }\n\n')
            f.write('  RequestPb request;\n')
            f.write('  std::string request_error;\n')
            f.write('  if (!parseRequest(request_json, encode_json, &request, &request_error)) {\n')
            f.write('    return errorCString(request_error);\n')
            f.write('  }\n\n')
            f.write('  auto channel =\n')
            f.write('      grpc::CreateChannel(endpoint, grpc::InsecureChannelCredentials());\n')
            f.write('  auto stub = stub_factory(channel);\n')
            f.write('  if (!stub) {\n')
            f.write('    return errorCString("create-stub");\n')
            f.write('  }\n\n')
            f.write('  ResponsePb response;\n')
            f.write('  grpc::ClientContext context;\n')
            f.write('  const grpc::Status status =\n')
            f.write('      rpc_invoker(stub.get(), &context, request, &response);\n')
            f.write('  if (!status.ok()) {\n')
            f.write('    return errorCString(status.error_message());\n')
            f.write('  }\n\n')
            f.write('  return renderUnaryResponse(response, decode_json);\n')
            f.write('}\n\n')
            f.write('template <typename RequestPb, typename ItemPb, typename StubFactory,\n')
            f.write('          typename RpcInvoker>\n')
            f.write('char* invokeServerStreamJsonRpc(const char* endpoint, const char* request_json,\n')
            f.write('                                EncodeJsonFn encode_json,\n')
            f.write('                                DecodeJsonFn decode_json,\n')
            f.write('                                StubFactory stub_factory,\n')
            f.write('                                RpcInvoker rpc_invoker) {\n')
            f.write('  if (!endpoint || !request_json) {\n')
            f.write('    return errorCString("null-argument");\n')
            f.write('  }\n\n')
            f.write('  RequestPb request;\n')
            f.write('  std::string request_error;\n')
            f.write('  if (!parseRequest(request_json, encode_json, &request, &request_error)) {\n')
            f.write('    return errorCString(request_error);\n')
            f.write('  }\n\n')
            f.write('  auto channel =\n')
            f.write('      grpc::CreateChannel(endpoint, grpc::InsecureChannelCredentials());\n')
            f.write('  auto stub = stub_factory(channel);\n')
            f.write('  if (!stub) {\n')
            f.write('    return errorCString("create-stub");\n')
            f.write('  }\n\n')
            f.write('  grpc::ClientContext context;\n')
            f.write('  auto reader = rpc_invoker(stub.get(), &context, request);\n')
            f.write('  if (!reader) {\n')
            f.write('    return errorCString("create-reader");\n')
            f.write('  }\n\n')
            f.write('  nlohmann::json items = nlohmann::json::array();\n')
            f.write('  ItemPb item;\n')
            f.write('  while (reader->Read(&item)) {\n')
            f.write('    std::string item_bytes;\n')
            f.write('    if (!item.SerializeToString(&item_bytes)) {\n')
            f.write('      return errorCString("serialize-stream-item");\n')
            f.write('    }\n\n')
            f.write('    std::unique_ptr<char, decltype(&std::free)> item_json(\n')
            f.write('        decode_json(item_bytes.data(), item_bytes.size()), &std::free);\n')
            f.write('    if (!item_json) {\n')
            f.write('      return errorCString("decode-stream-item");\n')
            f.write('    }\n\n')
            f.write('    auto parsed = nlohmann::json::parse(item_json.get(), nullptr, false);\n')
            f.write('    if (parsed.is_discarded()) {\n')
            f.write('      return errorCString("parse-stream-item-json");\n')
            f.write('    }\n')
            f.write('    items.push_back(std::move(parsed));\n')
            f.write('  }\n\n')
            f.write('  const grpc::Status status = reader->Finish();\n')
            f.write('  if (!status.ok()) {\n')
            f.write('    return errorCString(status.error_message());\n')
            f.write('  }\n\n')
            f.write('  return copyCString(items.dump());\n')
            f.write('}\n\n')
            f.write(f'}}  // namespace {support_ns}\n\n')
            f.write('extern "C" {\n')
            f.write(f'PYRAMID_GRPC_C_SHIM_EXPORT void {free_string_name}(void* value);\n')
            f.write('}\n')

    def _write_cpp_c_api_support_impl(self, path: Path, component: str):
        support_base = _grpc_c_api_support_base(component)
        free_string_name = f'pyramid_services_{component}_grpc_free_string'

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'#include "{support_base}.hpp"\n\n')
            f.write('extern "C" {\n\n')
            f.write(f'void {free_string_name}(void* value) {{\n')
            f.write('  std::free(value);\n')
            f.write('}\n\n')
            f.write('}  // extern "C"\n')

    def _write_cpp_c_api_header(self, path: Path, pf: ProtoFile, component: str):
        pkg_parts = [p for p in pf.package.split('.') if p]
        pkg_prefix = '_'.join(pkg_parts)
        package_role = _package_role(pf.package)
        support_base = _grpc_c_api_support_base(component)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#pragma once\n\n')
            f.write(f'#include "{support_base}.hpp"\n\n')
            f.write('extern "C" {\n\n')

            for svc in pf.services:
                svc_name = _service_wire_prefix(svc.name)
                for rpc in svc.rpcs:
                    rpc_name = camel_to_lower_snake(rpc.name)
                    long_name = (
                        f'{pkg_prefix}_{svc_name}_{rpc_name}_json'
                    )
                    short_name = f'grpc_{package_role}_{svc_name}_{rpc_name}_json'
                    f.write('PYRAMID_GRPC_C_SHIM_EXPORT char*\n')
                    f.write(f'{long_name}(\n')
                    f.write('    const char* endpoint, const char* request_json);\n')
                    f.write(f'PYRAMID_GRPC_C_SHIM_EXPORT char* {short_name}(\n')
                    f.write('    const char* endpoint, const char* request_json);\n\n')

            f.write('}\n')

    def _write_cpp_c_api_impl(self, path: Path, pf: ProtoFile, component: str):
        pkg_parts = [p for p in pf.package.split('.') if p]
        file_base = '_'.join(pkg_parts)
        package_role = _package_role(pf.package)
        shim_prefix = _protobuf_shim_prefix(component)
        grpc_header = _grpc_pb_header_path(pf)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'#include "{file_base}_grpc_c_api.hpp"\n\n')
            f.write(f'#include "{_protobuf_shim_header(component)}"\n\n')
            f.write(f'#include "{grpc_header}"\n\n')
            f.write(f'namespace grpc_c_api = pyramid::services::{component}::grpc_transport::c_api;\n')
            f.write(f'namespace proto_services = {"::".join(pkg_parts)};\n\n')
            f.write('extern "C" {\n\n')

            for svc in pf.services:
                svc_name = _service_wire_prefix(svc.name)
                for rpc in svc.rpcs:
                    rpc_name = camel_to_lower_snake(rpc.name)
                    long_name = (
                        f'{"_".join(pkg_parts)}_{svc_name}_{rpc_name}_json'
                    )
                    short_name = f'grpc_{package_role}_{svc_name}_{rpc_name}_json'
                    req_cpp = '::' + '::'.join(rpc.request_type.split('.'))
                    rsp_cpp = '::' + '::'.join(rpc.response_type.split('.'))
                    req_encode = f'{shim_prefix}_{_short_type(rpc.request_type)}_to_protobuf_json'
                    rsp_decode = f'{shim_prefix}_{_short_type(rpc.response_type)}_from_protobuf_json'

                    f.write('char*\n')
                    f.write(f'{long_name}(\n')
                    f.write('    const char* endpoint, const char* request_json) {\n')
                    if rpc.server_streaming:
                        f.write(f'  return grpc_c_api::invokeServerStreamJsonRpc<\n')
                        f.write(f'      {req_cpp}, {rsp_cpp}>(\n')
                    else:
                        f.write(f'  return grpc_c_api::invokeUnaryJsonRpc<\n')
                        f.write(f'      {req_cpp}, {rsp_cpp}>(\n')
                    f.write('      endpoint, request_json,\n')
                    f.write(f'      {req_encode},\n')
                    f.write(f'      {rsp_decode},\n')
                    f.write('      [](const std::shared_ptr<grpc::Channel>& channel) {\n')
                    f.write(f'        return proto_services::{svc.name}::NewStub(channel);\n')
                    f.write('      },\n')
                    if rpc.server_streaming:
                        f.write('      [](auto* stub, grpc::ClientContext* context,\n')
                        f.write(f'         const {req_cpp}& request) {{\n')
                        f.write(f'        return stub->{rpc.name}(context, request);\n')
                        f.write('      });\n')
                    else:
                        f.write('      [](auto* stub, grpc::ClientContext* context,\n')
                        f.write(f'         const {req_cpp}& request,\n')
                        f.write(f'         {rsp_cpp}* response) {{\n')
                        f.write(f'        return stub->{rpc.name}(context, request, response);\n')
                        f.write('      });\n')
                    f.write('}\n\n')
                    f.write(f'char* {short_name}(\n')
                    f.write('    const char* endpoint, const char* request_json) {\n')
                    f.write(f'  return {long_name}(endpoint, request_json);\n')
                    f.write('}\n\n')

            f.write('}  // extern "C"\n')

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

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated gRPC transport spec -- do not edit\n')
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
                    ada_rpc = _ada_grpc_rpc_name(svc.name, rpc.name)
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

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated gRPC transport body -- do not edit\n\n')
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
                    ada_rpc = _ada_grpc_rpc_name(svc.name, rpc.name)
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
                    svc_name = _service_wire_prefix(svc.name)
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
