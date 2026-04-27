#!/usr/bin/env python3
"""
ROS2 Messaging Backend

Generates transport projection code that maps PCL/PYRAMID service semantics to
ROS2 naming and envelope conventions. The generated C++ transport code binds
ROS2 ingress onto the PCL executor so business logic still runs only on the
executor thread.

Unlike gRPC, ROS2 does not provide a native server-streaming service shape, so
streaming RPCs are mapped to:

  - an "open" ROS2 service endpoint
  - a correlated "frames" ROS2 topic
  - a correlated "cancel" ROS2 topic

The selected service bindings own the service-specific ROS2 surface. C++ emits
top-level bindRos2(...) startup hooks from cpp_codegen.py, and Ada emits
top-level endpoint constants from ada_codegen.py. This backend remains
registered so --backends ros2 selects that facade behavior; the generic runtime
support layer lives under bindings/cpp/generated/ros2/cpp.
"""

from pathlib import Path
from typing import List

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import ProtoFile, ProtoTypeIndex, camel_to_lower_snake, camel_to_snake
import codec_backends


def _strip_service_suffix(name: str) -> str:
    if name.endswith('_Service'):
        return name[:-len('_Service')]
    if name.endswith('Service'):
        return name[:-len('Service')]
    return name


def _wire_service_name(svc_name: str) -> str:
    return camel_to_lower_snake(_strip_service_suffix(svc_name))


def _is_service_package(pf: ProtoFile) -> bool:
    return '.services.' in f'.{pf.package}.'


class Ros2Backend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'ros2'

    @property
    def content_type(self) -> str:
        return 'application/ros2'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        del index, output_dir
        return []

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        del index, output_dir
        return []

    def _write_cpp_header(self, path: Path, ns: str, pf: ProtoFile):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated ROS2 transport projection -- do not edit\n')
            f.write('// Backend: ros2\n')
            f.write('#pragma once\n\n')
            f.write('#include "pyramid_ros2_transport_support.hpp"\n\n')
            f.write('#include <pcl/pcl_types.h>\n\n')
            f.write('namespace ')
            f.write(ns)
            f.write(' {\n\n')
            f.write('class ServiceBinder {\n')
            f.write(' public:\n')
            f.write('  ServiceBinder(pyramid::transport::ros2::Adapter& adapter,\n')
            f.write('                pcl_executor_t* executor);\n')
            f.write('  void bind();\n\n')
            f.write(' private:\n')
            f.write('  pyramid::transport::ros2::Adapter& adapter_;\n')
            f.write('  pcl_executor_t* executor_;\n')
            f.write('};\n\n')
            f.write('}  // namespace ')
            f.write(ns)
            f.write('\n')

    def _write_cpp_impl(self, path: Path, ns: str, file_base: str, pf: ProtoFile):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('// Auto-generated ROS2 transport projection -- do not edit\n\n')
            f.write(f'#include "{file_base}_ros2_transport.hpp"\n\n')
            f.write('namespace {\n\n')

            for svc in pf.services:
                svc_wire = _wire_service_name(svc.name)
                for rpc in svc.rpcs:
                    full_name = f'{svc_wire}.{camel_to_lower_snake(rpc.name)}'
                    const_name = f'kSvc{svc_wire}_{rpc.name}'.replace('.', '_')
                    f.write(f'constexpr const char* {const_name} = "{full_name}";\n')
            f.write('\n')
            f.write('}  // namespace\n\n')
            f.write('namespace ')
            f.write(ns)
            f.write(' {\n\n')
            f.write('ServiceBinder::ServiceBinder(\n')
            f.write('    pyramid::transport::ros2::Adapter& adapter,\n')
            f.write('    pcl_executor_t* executor)\n')
            f.write('    : adapter_(adapter), executor_(executor) {}\n\n')
            f.write('void ServiceBinder::bind() {\n')

            for svc in pf.services:
                svc_wire = _wire_service_name(svc.name)
                for rpc in svc.rpcs:
                    const_name = f'kSvc{svc_wire}_{rpc.name}'.replace('.', '_')
                    if rpc.server_streaming:
                        f.write(f'  pyramid::transport::ros2::bindStreamServiceIngress('
                                f'adapter_, executor_, {const_name});\n')
                    else:
                        f.write(f'  pyramid::transport::ros2::bindUnaryServiceIngress('
                                f'adapter_, executor_, {const_name});\n')

            f.write('}\n\n')
            f.write('}  // namespace ')
            f.write(ns)
            f.write('\n')

    def _write_ada_spec(self, path: Path, pf: ProtoFile):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.ROS2_Transport'

        with open(path, 'w', encoding='utf-8') as f:
            f.write('--  Auto-generated ROS2 transport constants -- do not edit\n')
            f.write(f'--  Backend: ros2 | Package: {pkg_name}\n\n')
            f.write(f'package {pkg_name} is\n\n')
            f.write('   Content_Type : constant String := "application/ros2";\n\n')

            for svc in pf.services:
                svc_wire = _wire_service_name(svc.name)
                for rpc in svc.rpcs:
                    rpc_wire = camel_to_lower_snake(rpc.name)
                    ident_prefix = f'{camel_to_snake(_strip_service_suffix(svc.name))}_{camel_to_snake(rpc.name)}'
                    ros2_base = f'/pyramid/{svc_wire}/{rpc_wire}'
                    if rpc.server_streaming:
                        f.write(f'   {ident_prefix}_Open_Service : constant String := '
                                f'"/pyramid/stream/{svc_wire}/{rpc_wire}/open";\n')
                        f.write(f'   {ident_prefix}_Frame_Topic : constant String := '
                                f'"/pyramid/stream/{svc_wire}/{rpc_wire}/frames";\n')
                        f.write(f'   {ident_prefix}_Cancel_Topic : constant String := '
                                f'"/pyramid/stream/{svc_wire}/{rpc_wire}/cancel";\n\n')
                    else:
                        f.write(f'   {ident_prefix}_Service : constant String := '
                                f'"/pyramid/service/{svc_wire}/{rpc_wire}";\n\n')

            f.write(f'end {pkg_name};\n')


codec_backends.register(Ros2Backend())
