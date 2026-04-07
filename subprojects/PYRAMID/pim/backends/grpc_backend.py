#!/usr/bin/env python3
"""
gRPC Transport Backend

Generates C++ and Ada code that exposes proto services as full gRPC services,
replacing the PCL transport layer entirely.

The generated service implementations delegate to the same ServiceHandler
interface used by the PCL bindings, so component business logic is
transport-agnostic:

    component logic → ServiceHandler → { PCL binding | gRPC service }

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
            ns = '::'.join(pkg_parts) + '::grpc_transport'

            # protoc+grpc_cpp_plugin generated header
            grpc_header = '/'.join(pf.package.split('.')) + '.grpc.pb.h'

            hpp_path = output_dir / (file_base + '_grpc_transport.hpp')
            cpp_path = output_dir / (file_base + '_grpc_transport.cpp')

            self._write_cpp_header(hpp_path, ns, grpc_header, pf, index)
            self._write_cpp_impl(cpp_path, ns, file_base, pf, index)
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
            self._write_ada_spec(spec_path, pf, index)
            generated.append(spec_path)

        return generated

    # -- C++ header ------------------------------------------------------------

    def _write_cpp_header(self, path: Path, ns: str, grpc_header: str,
                          pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w') as f:
            f.write(f'// Auto-generated gRPC transport — do not edit\n')
            f.write(f'// Backend: grpc | Namespace: {ns}\n')
            f.write(f'//\n')
            f.write(f'// Exposes proto services as gRPC services, delegating to\n')
            f.write(f'// the same ServiceHandler interface used by PCL bindings.\n')
            f.write(f'//\n')
            f.write(f'// Requires: protoc + grpc_cpp_plugin generated code\n')
            f.write(f'// Link with: grpc++ protobuf\n')
            f.write(f'#pragma once\n\n')
            f.write(f'#include "{grpc_header}"\n\n')
            f.write(f'#include <grpcpp/grpcpp.h>\n')
            f.write(f'#include <memory>\n')
            f.write(f'#include <string>\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # ServiceHandler base class (same interface as PCL binding)
            f.write(f'// ---------------------------------------------------------------------------\n')
            f.write(f'// ServiceHandler — transport-agnostic business logic interface\n')
            f.write(f'//\n')
            f.write(f'// Implement this interface once; it works with both PCL and gRPC transport.\n')
            f.write(f'// ---------------------------------------------------------------------------\n\n')
            f.write(f'class ServiceHandler {{\n')
            f.write(f'public:\n')
            f.write(f'    virtual ~ServiceHandler() = default;\n')

            for svc in pf.services:
                f.write(f'\n    // {svc.name}\n')
                for rpc in svc.rpcs:
                    op, entity = _rpc_op(rpc)
                    req_t = _short_type(rpc.request_type)
                    rsp_t = _short_type(rpc.response_type)
                    if rpc.server_streaming:
                        rsp_t = f'std::vector<{rsp_t}>'
                    f.write(f'    virtual {rsp_t} handle{rpc.name}(const {req_t}& request);\n')

            f.write(f'}};\n\n')

            # gRPC service implementations
            for svc in pf.services:
                f.write(f'// ---------------------------------------------------------------------------\n')
                f.write(f'// {svc.name} — gRPC service implementation\n')
                f.write(f'// ---------------------------------------------------------------------------\n\n')
                f.write(f'class {svc.name}Impl final : public {proto_ns}::{svc.name}::Service {{\n')
                f.write(f'public:\n')
                f.write(f'    explicit {svc.name}Impl(ServiceHandler& handler)\n')
                f.write(f'        : handler_(handler) {{}}\n\n')

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
                f.write(f'    ServiceHandler& handler_;\n')
                f.write(f'}};\n\n')

            # Server builder utility
            f.write(f'// ---------------------------------------------------------------------------\n')
            f.write(f'// Server builder — registers all services on one gRPC server\n')
            f.write(f'// ---------------------------------------------------------------------------\n\n')
            f.write(f'std::unique_ptr<grpc::Server> buildServer(\n')
            f.write(f'    const std::string& listen_address,\n')
            f.write(f'    ServiceHandler& handler);\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- C++ implementation ----------------------------------------------------

    def _write_cpp_impl(self, path: Path, ns: str, file_base: str,
                        pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w') as f:
            f.write(f'// Auto-generated gRPC transport — do not edit\n\n')
            f.write(f'#include "{file_base}_grpc_transport.hpp"\n\n')
            f.write(f'namespace {ns} {{\n\n')

            # ServiceHandler default stubs
            for svc in pf.services:
                for rpc in svc.rpcs:
                    req_t = _short_type(rpc.request_type)
                    rsp_t = _short_type(rpc.response_type)
                    if rpc.server_streaming:
                        rsp_t = f'std::vector<{rsp_t}>'
                    f.write(f'{rsp_t}\n')
                    f.write(f'ServiceHandler::handle{rpc.name}(const {req_t}& /*request*/) {{\n')
                    f.write(f'    return {{}};\n')
                    f.write(f'}}\n\n')

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

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated gRPC transport spec — do not edit\n')
            f.write(f'--  Backend: grpc | Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  gRPC transport for Ada requires C++ interop via grpc_core.\n')
            f.write(f'--  This package defines the Ada-side interface; actual gRPC\n')
            f.write(f'--  communication is delegated to C++ via pragma Import.\n\n')
            f.write(f'with Interfaces.C; use Interfaces.C;\n')
            f.write(f'with System;\n\n')
            f.write(f'package {pkg_name} is\n\n')

            for svc in pf.services:
                ada_svc = camel_to_snake(svc.name)
                f.write(f'   --  {svc.name}\n\n')
                for rpc in svc.rpcs:
                    ada_rpc = camel_to_snake(rpc.name)
                    f.write(f'   procedure Invoke_{ada_rpc}\n')
                    f.write(f'     (Channel  : System.Address;\n')
                    f.write(f'      Request  : System.Address;\n')
                    f.write(f'      Response : System.Address)\n')
                    f.write(f'     with Import, Convention => C,\n')
                    f.write(f'          External_Name => "grpc_{camel_to_lower_snake(svc.name)}_{camel_to_lower_snake(rpc.name)}";\n\n')

            f.write(f'   --  Server lifecycle\n\n')
            f.write(f'   procedure Start_Server (Address : Interfaces.C.Strings.chars_ptr)\n')
            f.write(f'     with Import, Convention => C,\n')
            f.write(f'          External_Name => "grpc_server_start";\n\n')
            f.write(f'   procedure Stop_Server\n')
            f.write(f'     with Import, Convention => C,\n')
            f.write(f'          External_Name => "grpc_server_stop";\n\n')

            f.write(f'end {pkg_name};\n')


# -- Register ------------------------------------------------------------------

codec_backends.register(GrpcBackend())
