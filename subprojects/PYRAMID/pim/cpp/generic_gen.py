#!/usr/bin/env python3
"""Generic-layout service emitters for CppServiceGenerator.

Split verbatim from cpp_codegen.py (generator refactor plan, phase 3).
"""

import sys
from pathlib import Path
from typing import List

from proto_parser import (parse_proto, ProtoTypeIndex)
from proto_resolve import _proto_type_fqn
from .naming import (_CPP_SCALAR_MAP, _service_cpp_prefix)


class GenericServiceEmitterMixin:
    """Generic (role-neutral) service header/impl emission."""

    def _generic_proto_files(self) -> List[Path]:
        if self._proto_input.is_dir():
            return list(self._proto_input.rglob('*.proto'))
        if self._proto_input.is_file():
            return [self._proto_input]
        print(f'ERROR: {self._proto_input} is not a file or directory',
              file=sys.stderr)
        sys.exit(1)

    def _generic_cpp_type(self, type_name: str, current_pkg: str,
                          index: ProtoTypeIndex) -> str:
        short = type_name.split('.')[-1]
        if type_name in _CPP_SCALAR_MAP:
            return _CPP_SCALAR_MAP[type_name]
        if type_name == 'google.protobuf.Empty':
            return 'Empty'
        if '.' in type_name and not type_name.startswith('google.'):
            pkg = type_name.rsplit('.', 1)[0]
            if pkg != current_pkg:
                return self._naming.cpp_type_namespace_for_package(pkg) + '::' + short
            return short
        fqn = _proto_type_fqn(index, type_name, current_pkg)
        if fqn and '.' in fqn:
            pkg = fqn.rsplit('.', 1)[0]
            resolved_short = fqn.split('.')[-1]
            if pkg != current_pkg:
                return (self._naming.cpp_type_namespace_for_package(pkg)
                        + '::' + resolved_short)
            return resolved_short
        return short

    def _generic_rpc_symbol(self, service_name: str, rpc_name: str) -> str:
        return _service_cpp_prefix(service_name) + rpc_name

    def _generate_generic(self, output_dir: str) -> None:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        all_files = self._discover_all_proto_files()
        index = ProtoTypeIndex(all_files)

        for proto_path in self._generic_proto_files():
            parsed = parse_proto(proto_path)
            if not parsed.services:
                continue
            file_prefix = self._naming.service_file_prefix(parsed.package)
            full_ns = self._naming.service_namespace(parsed.package)
            hpp_path = output_path / (file_prefix + '.hpp')
            cpp_path = output_path / (file_prefix + '.cpp')
            self._write_generic_service_header(
                hpp_path, file_prefix, full_ns, parsed, index)
            self._write_generic_service_impl(
                cpp_path, file_prefix, full_ns, parsed, index)
            print(f'  Generated {full_ns}')

    def _write_generic_service_header(self, path: Path, file_prefix: str,
                                      full_ns: str, parsed,
                                      index: ProtoTypeIndex) -> None:
        del file_prefix
        codec_header = self._naming.codec_header_for_package(parsed.package)
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated generic JSON service binding header\n')
            f.write(f'// Generated from: {parsed.path.name}'
                    f' by generate_bindings.py\n')
            f.write(f'// Namespace: {full_ns}\n')
            f.write('#pragma once\n\n')
            f.write(f'#include "{codec_header}"\n\n')
            f.write('#include <string>\n\n')
            f.write(f'namespace {full_ns} {{\n\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    const_name = 'kSvc' + self._generic_rpc_symbol(
                        svc.name, rpc.name)
                    wire = f'{svc.name}.{rpc.name}'
                    f.write(f'constexpr const char* {const_name} = "{wire}";\n')
            f.write('\n')
            f.write('class ServiceHandler {\n')
            f.write('public:\n')
            f.write('    virtual ~ServiceHandler() = default;\n')
            for svc in parsed.services:
                f.write(f'\n    // {svc.name}\n')
                for rpc in svc.rpcs:
                    req_t = self._generic_cpp_type(
                        rpc.request_type, parsed.package, index)
                    rsp_t = self._generic_cpp_type(
                        rpc.response_type, parsed.package, index)
                    handler = 'handle' + self._generic_rpc_symbol(
                        svc.name, rpc.name)
                    f.write(f'    virtual {rsp_t}\n')
                    f.write(f'    {handler}(const {req_t}& request);\n')
            f.write('};\n\n')
            f.write('std::string dispatch(ServiceHandler& handler,\n')
            f.write('                     const std::string& service_name,\n')
            f.write('                     const std::string& request_json);\n\n')
            f.write(f'}} // namespace {full_ns}\n')

    def _write_generic_service_impl(self, path: Path, file_prefix: str,
                                    full_ns: str, parsed,
                                    index: ProtoTypeIndex) -> None:
        hpp_name = file_prefix + '.hpp'
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated generic JSON service binding implementation\n')
            f.write(f'// Namespace: {full_ns}\n\n')
            f.write(f'#include "{hpp_name}"\n\n')
            f.write(f'namespace {full_ns} {{\n\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    req_t = self._generic_cpp_type(
                        rpc.request_type, parsed.package, index)
                    rsp_t = self._generic_cpp_type(
                        rpc.response_type, parsed.package, index)
                    handler = 'handle' + self._generic_rpc_symbol(
                        svc.name, rpc.name)
                    f.write(f'{rsp_t}\n')
                    f.write(f'ServiceHandler::{handler}'
                            f'(const {req_t}& /*request*/) {{\n')
                    f.write('    return {};\n')
                    f.write('}\n\n')
            f.write('std::string dispatch(ServiceHandler& handler,\n')
            f.write('                     const std::string& service_name,\n')
            f.write('                     const std::string& request_json)\n')
            f.write('{\n')
            for svc in parsed.services:
                for rpc in svc.rpcs:
                    req_t = self._generic_cpp_type(
                        rpc.request_type, parsed.package, index)
                    rsp_t = self._generic_cpp_type(
                        rpc.response_type, parsed.package, index)
                    handler = 'handle' + self._generic_rpc_symbol(
                        svc.name, rpc.name)
                    const_name = 'kSvc' + self._generic_rpc_symbol(
                        svc.name, rpc.name)
                    f.write(f'    if (service_name == {const_name}) {{\n')
                    f.write(f'        {req_t} request = fromJson(\n')
                    f.write(f'            request_json, static_cast<{req_t}*>(nullptr));\n')
                    f.write(f'        {rsp_t} response = handler.{handler}(request);\n')
                    f.write('        return toJson(response);\n')
                    f.write('    }\n')
            f.write('    return {};\n')
            f.write('}\n\n')
            f.write(f'}} // namespace {full_ns}\n')

