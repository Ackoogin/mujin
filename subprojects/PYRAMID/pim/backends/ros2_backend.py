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
        del index
        output_dir.mkdir(parents=True, exist_ok=True)
        hpp_path = output_dir / 'pyramid_ros2_transport_support.hpp'
        cpp_path = output_dir / 'pyramid_ros2_transport_support.cpp'
        self._write_cpp_support_header(hpp_path)
        self._write_cpp_support_impl(cpp_path)
        return [hpp_path, cpp_path]

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        del index, output_dir
        return []

    def _write_cpp_support_header(self, path: Path):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#pragma once\n\n')
            f.write('#include <pcl/pcl_executor.h>\n')
            f.write('#include <pcl/pcl_types.h>\n\n')
            f.write('#include <functional>\n')
            f.write('#include <string>\n')
            f.write('#include <string_view>\n')
            f.write('#include <vector>\n\n')
            f.write('namespace pyramid::transport::ros2 {\n\n')
            f.write('inline constexpr const char* kTransportContentType = "application/ros2";\n')
            f.write('inline constexpr const char* kEnvelopeType = "pyramid_ros2/PclEnvelope";\n\n')
            f.write('struct Envelope {\n')
            f.write('  std::string content_type;\n')
            f.write('  std::string correlation_id;\n')
            f.write('  std::vector<unsigned char> payload;\n')
            f.write('  bool end_of_stream = false;\n')
            f.write('  pcl_status_t status = PCL_OK;\n')
            f.write('};\n\n')
            f.write('struct TopicBinding {\n')
            f.write('  std::string pcl_topic;\n')
            f.write('  std::string ros2_topic;\n')
            f.write('};\n\n')
            f.write('struct UnaryServiceBinding {\n')
            f.write('  std::string pcl_service;\n')
            f.write('  std::string ros2_service;\n')
            f.write('};\n\n')
            f.write('struct StreamServiceBinding {\n')
            f.write('  std::string pcl_service;\n')
            f.write('  std::string ros2_open_service;\n')
            f.write('  std::string ros2_frame_topic;\n')
            f.write('  std::string ros2_cancel_topic;\n')
            f.write('};\n\n')
            f.write('using TopicHandler = std::function<void(const Envelope&)>;\n')
            f.write('using UnaryHandler = std::function<Envelope(const Envelope&)>;\n')
            f.write('using StreamEmitter = std::function<bool(const Envelope&)>;\n')
            f.write('using StreamHandler = std::function<void(const Envelope&, const StreamEmitter&)>;\n\n')
            f.write('class Adapter {\n')
            f.write(' public:\n')
            f.write('  virtual ~Adapter() = default;\n\n')
            f.write('  virtual void subscribe(const TopicBinding& binding, TopicHandler handler) = 0;\n')
            f.write('  virtual void advertise(const UnaryServiceBinding& binding,\n')
            f.write('                         UnaryHandler handler) = 0;\n')
            f.write('  virtual void advertise(const StreamServiceBinding& binding,\n')
            f.write('                         StreamHandler handler) = 0;\n')
            f.write('  virtual void publish(const TopicBinding& binding, const Envelope& envelope) = 0;\n')
            f.write('};\n\n')
            f.write('TopicBinding makeTopicBinding(std::string_view pcl_topic);\n')
            f.write('UnaryServiceBinding makeUnaryServiceBinding(std::string_view pcl_service);\n')
            f.write('StreamServiceBinding makeStreamServiceBinding(std::string_view pcl_service);\n\n')
            f.write('Envelope envelopeFromMessage(const pcl_msg_t* msg);\n')
            f.write('pcl_status_t postIncomingEnvelope(pcl_executor_t* executor, const char* pcl_topic,\n')
            f.write('                                  const Envelope& envelope);\n')
            f.write('pcl_status_t publishOutboundEnvelope(Adapter& adapter, const char* pcl_topic,\n')
            f.write('                                     const pcl_msg_t* msg);\n')
            f.write('Envelope invokeUnaryOnExecutor(pcl_executor_t* executor, const char* pcl_service,\n')
            f.write('                               const Envelope& request);\n')
            f.write('void invokeStreamOnExecutor(pcl_executor_t* executor, const char* pcl_service,\n')
            f.write('                            const Envelope& request,\n')
            f.write('                            const StreamEmitter& emit_frame);\n\n')
            f.write('void bindTopicIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                      const char* pcl_topic);\n')
            f.write('void bindUnaryServiceIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                             const char* pcl_service);\n')
            f.write('void bindStreamServiceIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                              const char* pcl_service);\n\n')
            f.write('}  // namespace pyramid::transport::ros2\n')

    def _write_cpp_support_impl(self, path: Path):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('#include "pyramid_ros2_transport_support.hpp"\n\n')
            f.write('#include <cctype>\n')
            f.write('#include <future>\n')
            f.write('#include <stdexcept>\n\n')
            f.write('namespace pyramid::transport::ros2 {\n\n')
            f.write('namespace {\n\n')
            f.write('std::string sanitizeName(std::string_view value) {\n')
            f.write('  std::string out;\n')
            f.write('  out.reserve(value.size() + 8U);\n')
            f.write('  for (char ch : value) {\n')
            f.write('    const auto uch = static_cast<unsigned char>(ch);\n')
            f.write('    if (std::isalnum(uch) != 0) {\n')
            f.write('      out.push_back(static_cast<char>(std::tolower(uch)));\n')
            f.write("    } else if (ch == '.' || ch == '/') {\n")
            f.write("      if (out.empty() || out.back() != '/') {\n")
            f.write("        out.push_back('/');\n")
            f.write('      }\n')
            f.write("    } else if (ch == '_') {\n")
            f.write("      out.push_back('_');\n")
            f.write('    } else {\n')
            f.write("      out.push_back('_');\n")
            f.write('    }\n')
            f.write('  }\n')
            f.write("  while (!out.empty() && out.back() == '/') {\n")
            f.write('    out.pop_back();\n')
            f.write('  }\n')
            f.write('  return out;\n')
            f.write('}\n\n')
            f.write('bool readVarint32(const unsigned char*& cursor, const unsigned char* end,\n')
            f.write('                  uint32_t& value) {\n')
            f.write('  value = 0U;\n')
            f.write('  uint32_t shift = 0U;\n')
            f.write('  while (cursor < end && shift <= 28U) {\n')
            f.write('    const uint8_t byte = *cursor++;\n')
            f.write('    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;\n')
            f.write('    if ((byte & 0x80U) == 0U) {\n')
            f.write('      return true;\n')
            f.write('    }\n')
            f.write('    shift += 7U;\n')
            f.write('  }\n')
            f.write('  return false;\n')
            f.write('}\n\n')
            f.write('struct UnaryState {\n')
            f.write('  std::promise<Envelope> promise;\n')
            f.write('};\n\n')
            f.write('void unaryResponseCallback(const pcl_msg_t* response, void* user_data) {\n')
            f.write('  auto* state = static_cast<UnaryState*>(user_data);\n')
            f.write('  if (!state) {\n')
            f.write('    return;\n')
            f.write('  }\n')
            f.write('  Envelope envelope;\n')
            f.write('  if (response && response->type_name) {\n')
            f.write('    envelope.content_type = response->type_name;\n')
            f.write('  }\n')
            f.write('  if (response && response->data && response->size > 0U) {\n')
            f.write('    const auto* begin =\n')
            f.write('        static_cast<const unsigned char*>(response->data);\n')
            f.write('    envelope.payload.assign(begin, begin + response->size);\n')
            f.write('  }\n')
            f.write('  state->promise.set_value(std::move(envelope));\n')
            f.write('}\n\n')
            f.write('}  // namespace\n\n')
            f.write('TopicBinding makeTopicBinding(std::string_view pcl_topic) {\n')
            f.write('  return {std::string(pcl_topic),\n')
            f.write('          std::string("/pyramid/topic/") + sanitizeName(pcl_topic)};\n')
            f.write('}\n\n')
            f.write('UnaryServiceBinding makeUnaryServiceBinding(std::string_view pcl_service) {\n')
            f.write('  return {std::string(pcl_service),\n')
            f.write('          std::string("/pyramid/service/") + sanitizeName(pcl_service)};\n')
            f.write('}\n\n')
            f.write('StreamServiceBinding makeStreamServiceBinding(std::string_view pcl_service) {\n')
            f.write('  const auto base = std::string("/pyramid/stream/") + sanitizeName(pcl_service);\n')
            f.write('  return {\n')
            f.write('      std::string(pcl_service),\n')
            f.write('      base + "/open",\n')
            f.write('      base + "/frames",\n')
            f.write('      base + "/cancel",\n')
            f.write('  };\n')
            f.write('}\n\n')
            f.write('Envelope envelopeFromMessage(const pcl_msg_t* msg) {\n')
            f.write('  Envelope envelope;\n')
            f.write('  if (!msg) {\n')
            f.write('    envelope.status = PCL_ERR_INVALID;\n')
            f.write('    return envelope;\n')
            f.write('  }\n')
            f.write('  if (msg->type_name) {\n')
            f.write('    envelope.content_type = msg->type_name;\n')
            f.write('  }\n')
            f.write('  if (msg->data && msg->size > 0U) {\n')
            f.write('    const auto* begin = static_cast<const unsigned char*>(msg->data);\n')
            f.write('    envelope.payload.assign(begin, begin + msg->size);\n')
            f.write('  }\n')
            f.write('  return envelope;\n')
            f.write('}\n\n')
            f.write('pcl_status_t postIncomingEnvelope(pcl_executor_t* executor, const char* pcl_topic,\n')
            f.write('                                  const Envelope& envelope) {\n')
            f.write('  if (!executor || !pcl_topic || !pcl_topic[0]) {\n')
            f.write('    return PCL_ERR_INVALID;\n')
            f.write('  }\n')
            f.write('  pcl_msg_t msg{};\n')
            f.write('  msg.data = envelope.payload.empty() ? nullptr : envelope.payload.data();\n')
            f.write('  msg.size = static_cast<uint32_t>(envelope.payload.size());\n')
            f.write('  msg.type_name =\n')
            f.write('      envelope.content_type.empty() ? nullptr : envelope.content_type.c_str();\n')
            f.write('  return pcl_executor_post_incoming(executor, pcl_topic, &msg);\n')
            f.write('}\n\n')
            f.write('pcl_status_t publishOutboundEnvelope(Adapter& adapter, const char* pcl_topic,\n')
            f.write('                                     const pcl_msg_t* msg) {\n')
            f.write('  if (!pcl_topic || !msg) {\n')
            f.write('    return PCL_ERR_INVALID;\n')
            f.write('  }\n')
            f.write('  adapter.publish(makeTopicBinding(pcl_topic), envelopeFromMessage(msg));\n')
            f.write('  return PCL_OK;\n')
            f.write('}\n\n')
            f.write('Envelope invokeUnaryOnExecutor(pcl_executor_t* executor, const char* pcl_service,\n')
            f.write('                               const Envelope& request) {\n')
            f.write('  Envelope error;\n')
            f.write('  if (!executor || !pcl_service || !pcl_service[0]) {\n')
            f.write('    error.status = PCL_ERR_INVALID;\n')
            f.write('    return error;\n')
            f.write('  }\n\n')
            f.write('  UnaryState state;\n')
            f.write('  pcl_msg_t request_msg{};\n')
            f.write('  request_msg.data = request.payload.empty() ? nullptr : request.payload.data();\n')
            f.write('  request_msg.size = static_cast<uint32_t>(request.payload.size());\n')
            f.write('  request_msg.type_name =\n')
            f.write('      request.content_type.empty() ? nullptr : request.content_type.c_str();\n\n')
            f.write('  const auto rc = pcl_executor_post_service_request(\n')
            f.write('      executor, pcl_service, &request_msg, unaryResponseCallback, &state);\n')
            f.write('  if (rc != PCL_OK) {\n')
            f.write('    error.status = rc;\n')
            f.write('    return error;\n')
            f.write('  }\n\n')
            f.write('  auto result = state.promise.get_future().get();\n')
            f.write('  result.correlation_id = request.correlation_id;\n')
            f.write('  return result;\n')
            f.write('}\n\n')
            f.write('void invokeStreamOnExecutor(pcl_executor_t* executor, const char* pcl_service,\n')
            f.write('                            const Envelope& request,\n')
            f.write('                            const StreamEmitter& emit_frame) {\n')
            f.write('  if (!emit_frame) {\n')
            f.write('    return;\n')
            f.write('  }\n\n')
            f.write('  auto response = invokeUnaryOnExecutor(executor, pcl_service, request);\n')
            f.write('  if (response.status != PCL_OK) {\n')
            f.write('    response.end_of_stream = true;\n')
            f.write('    emit_frame(response);\n')
            f.write('    return;\n')
            f.write('  }\n\n')
            f.write('  const auto* cursor = response.payload.data();\n')
            f.write('  const auto* end = cursor + response.payload.size();\n')
            f.write('  while (cursor < end) {\n')
            f.write('    uint32_t frame_size = 0U;\n')
            f.write('    if (!readVarint32(cursor, end, frame_size) ||\n')
            f.write('        static_cast<size_t>(end - cursor) < frame_size) {\n')
            f.write('      Envelope terminal;\n')
            f.write('      terminal.content_type = response.content_type;\n')
            f.write('      terminal.correlation_id = request.correlation_id;\n')
            f.write('      terminal.status = PCL_ERR_INVALID;\n')
            f.write('      terminal.end_of_stream = true;\n')
            f.write('      emit_frame(terminal);\n')
            f.write('      return;\n')
            f.write('    }\n\n')
            f.write('    Envelope frame;\n')
            f.write('    frame.content_type = response.content_type;\n')
            f.write('    frame.correlation_id = request.correlation_id;\n')
            f.write('    frame.payload.assign(cursor, cursor + frame_size);\n')
            f.write('    cursor += frame_size;\n\n')
            f.write('    if (!emit_frame(frame)) {\n')
            f.write('      Envelope terminal;\n')
            f.write('      terminal.content_type = response.content_type;\n')
            f.write('      terminal.correlation_id = request.correlation_id;\n')
            f.write('      terminal.status = PCL_ERR_INVALID;\n')
            f.write('      terminal.end_of_stream = true;\n')
            f.write('      emit_frame(terminal);\n')
            f.write('      return;\n')
            f.write('    }\n')
            f.write('  }\n\n')
            f.write('  Envelope terminal;\n')
            f.write('  terminal.content_type = response.content_type;\n')
            f.write('  terminal.correlation_id = request.correlation_id;\n')
            f.write('  terminal.end_of_stream = true;\n')
            f.write('  emit_frame(terminal);\n')
            f.write('}\n\n')
            f.write('void bindTopicIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                      const char* pcl_topic) {\n')
            f.write('  const auto binding = makeTopicBinding(pcl_topic);\n')
            f.write('  adapter.subscribe(binding, [executor, topic = std::string(pcl_topic)](\n')
            f.write('                                  const Envelope& envelope) {\n')
            f.write('    postIncomingEnvelope(executor, topic.c_str(), envelope);\n')
            f.write('  });\n')
            f.write('}\n\n')
            f.write('void bindUnaryServiceIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                             const char* pcl_service) {\n')
            f.write('  const auto binding = makeUnaryServiceBinding(pcl_service);\n')
            f.write('  adapter.advertise(binding,\n')
            f.write('                    [executor, service = std::string(pcl_service)](\n')
            f.write('                        const Envelope& request) {\n')
            f.write('                      return invokeUnaryOnExecutor(executor, service.c_str(),\n')
            f.write('                                                   request);\n')
            f.write('                    });\n')
            f.write('}\n\n')
            f.write('void bindStreamServiceIngress(Adapter& adapter, pcl_executor_t* executor,\n')
            f.write('                              const char* pcl_service) {\n')
            f.write('  const auto binding = makeStreamServiceBinding(pcl_service);\n')
            f.write('  adapter.advertise(binding,\n')
            f.write('                    [executor, service = std::string(pcl_service)](\n')
            f.write('                        const Envelope& request, const StreamEmitter& emit) {\n')
            f.write('                      invokeStreamOnExecutor(executor, service.c_str(), request,\n')
            f.write('                                             emit);\n')
            f.write('                    });\n')
            f.write('}\n\n')
            f.write('}  // namespace pyramid::transport::ros2\n')

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
