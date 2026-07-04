#!/usr/bin/env python3
"""
ROS2 marshalling + codec generator.

Emits ``pyramid_ros2_codec.hpp`` -- a header-only ``pyramid::domain_model`` <->
``pyramid_msgs`` ROS2-message marshalling plus a ``rclcpp``-serialised wire codec.
This is the typed ROS2 wire that replaces the opaque ``PclEnvelope`` pass-through:
the bytes on the wire are native ROS2 messages (introspectable by any ROS2 node),
not codec-bytes in a generic envelope.

It is the ROS2 analogue of the protobuf/flatbuffers wire codecs and presents the
same surface the generated codec plugin expects:

* ``std::string toBinary(const domain::<pkg>::<T>&)`` -- build the ROS2 message
  via ``toRos`` then serialise with ``rclcpp::Serialization``.
* ``domain::<pkg>::<T> fromBinary<T>(const void*, size_t)`` -- deserialise then
  ``fromRos``.

The field model comes from :mod:`ros2_ir` (shared with the IDL generator), so the
converters line up with the generated ``.msg`` by construction.
"""

from pathlib import Path
from typing import List

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from proto_parser import ProtoMessage, ProtoTypeIndex  # noqa: E402
from binding_contract import PyramidCompatNamingPolicy  # noqa: E402
from ros2_ir import DomainField, DomainIR, pascal, ros_header_stem  # noqa: E402

_DEFAULT_NAMING_POLICY = PyramidCompatNamingPolicy()


def _infer_codec_package(index: ProtoTypeIndex) -> str:
    for pf in index.files:
        if pf.services:
            return pf.package
    for pf in index.files:
        if pf.messages or pf.enums:
            return pf.package
    return ''


class Ros2MarshalGenerator:
    def __init__(self, index: ProtoTypeIndex, naming_policy=None,
                 package: str = ''):
        self.index = index
        self.naming_policy = naming_policy or _DEFAULT_NAMING_POLICY
        self.package = package or _infer_codec_package(index)
        self.ir = DomainIR(index, self.naming_policy, self.package)

    def _domain_type_headers(self) -> List[str]:
        umbrella = self.naming_policy.umbrella_types_header(self.index.files)
        if umbrella:
            return [umbrella]
        packages = {
            pf.package
            for pf in self.index.files
            if pf.messages or pf.enums
        }
        return [
            self.naming_policy.type_header_for_package(package)
            for package in sorted(packages)
        ]

    # -- helpers --------------------------------------------------------------

    def _domain_elem_cpp(self, df: DomainField) -> str:
        if df.category in ('enum', 'message'):
            return self.ir.domain_fqn(df.msg_short)
        if df.category == 'string':
            return 'std::string'
        return df.cpp_scalar

    def _ros_elem_cpp(self, df: DomainField) -> str:
        return self.ir.ros_msg_type(df.msg_short)  # enum/message only

    # -- toRos (domain -> ros) ------------------------------------------------

    def _to_ros_field(self, df: DomainField) -> List[str]:
        d = f'd.{df.name}'
        r = f'r.{df.name}'
        cat = df.category
        if df.presence == 'repeated':
            if cat in ('scalar', 'string'):
                return [f'  {r}.assign({d}.begin(), {d}.end());']
            if cat == 'enum':
                return [f'  for (const auto& e : {d}) {{',
                        f'    {self._ros_elem_cpp(df)} m; m.value = static_cast<int32_t>(e);',
                        f'    {r}.push_back(std::move(m)); }}']
            return [f'  for (const auto& it : {d}) {{',
                    f'    {self._ros_elem_cpp(df)} m; toRos(it, m);',
                    f'    {r}.push_back(std::move(m)); }}']
        if df.presence == 'opt':
            inner = {
                'scalar': f'    {r} = *{d};',
                'string': f'    {r} = *{d};',
                'enum':   f'    {r}.value = static_cast<int32_t>(*{d});',
                'message': f'    toRos(*{d}, {r});',
            }[cat]
            return [f'  r.has_{df.name} = {d}.has_value();',
                    f'  if ({d}.has_value()) {{', inner, '  }']
        # plain
        if cat == 'scalar' or cat == 'string':
            return [f'  {r} = {d};']
        if cat == 'enum':
            return [f'  {r}.value = static_cast<int32_t>({d});']
        return [f'  toRos({d}, {r});']

    # -- fromRos (ros -> domain) ----------------------------------------------

    def _from_ros_field(self, df: DomainField) -> List[str]:
        d = f'd.{df.name}'
        r = f'r.{df.name}'
        cat = df.category
        if df.presence == 'repeated':
            if cat in ('scalar', 'string'):
                return [f'  {d}.assign({r}.begin(), {r}.end());']
            if cat == 'enum':
                ed = self._domain_elem_cpp(df)
                return [f'  for (const auto& v : {r}) '
                        f'{d}.push_back(static_cast<{ed}>(v.value));']
            ed = self._domain_elem_cpp(df)
            return [f'  for (const auto& it : {r}) {{',
                    f'    {ed} tmp; fromRos(it, tmp);',
                    f'    {d}.push_back(std::move(tmp)); }}']
        if df.presence == 'opt':
            ed = self._domain_elem_cpp(df)
            if cat == 'message':
                set_ = [f'    {ed} tmp; fromRos({r}, tmp); {d} = std::move(tmp);']
            elif cat == 'enum':
                set_ = [f'    {d} = static_cast<{ed}>({r}.value);']
            else:
                set_ = [f'    {d} = {r};']
            return [f'  if (r.has_{df.name}) {{'] + set_ + ['  } else {',
                    f'    {d} = tl::nullopt;', '  }']
        # plain
        if cat == 'scalar' or cat == 'string':
            return [f'  {d} = {r};']
        if cat == 'enum':
            ed = self._domain_elem_cpp(df)
            return [f'  {d} = static_cast<{ed}>({r}.value);']
        return [f'  fromRos({r}, {d});']

    # -- emit -----------------------------------------------------------------

    def generate(self, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / self.naming_policy.ros2_codec_header(self.package)
        msgs = self.ir.emitted_messages()
        msg_package = self.naming_policy.ros2_message_package(self.package)
        codec_ns = self.naming_policy.ros2_codec_namespace(self.package)
        data_model_ns = self.naming_policy.ros2_data_model_namespace(self.package)

        out: List[str] = []
        w = out.append
        w('// Auto-generated ROS2 marshalling + codec -- do not edit')
        w(f'// {data_model_ns} <-> {msg_package} ROS2 messages + rclcpp wire codec')
        w('#pragma once')
        w('')
        for header in self._domain_type_headers():
            w(f'#include "{header}"')
        w('')
        for enum in self.index.all_enums():
            w(f'#include "{msg_package}/msg/{ros_header_stem(pascal(enum.name))}.hpp"')
        for msg in msgs:
            w(f'#include "{msg_package}/msg/{ros_header_stem(pascal(msg.name))}.hpp"')
        w('')
        w('#include <rclcpp/serialization.hpp>')
        w('#include <rclcpp/serialized_message.hpp>')
        w('')
        w('#include <cstdint>')
        w('#include <cstring>')
        w('#include <limits>')
        w('#include <stdexcept>')
        w('#include <string>')
        w('#include <utility>')
        w('#include <vector>')
        w('')
        w(f'namespace {codec_ns} {{')
        w('')
        w(f'namespace domain = {data_model_ns};')
        w('')
        w('// -- rclcpp (de)serialisation helpers -----------------------------------')
        w('template <class MsgT>')
        w('inline std::string serializeRos(const MsgT& msg) {')
        w('  rclcpp::Serialization<MsgT> ser;')
        w('  rclcpp::SerializedMessage sm;')
        w('  ser.serialize_message(&msg, &sm);')
        w('  const auto& rcl = sm.get_rcl_serialized_message();')
        w('  return std::string(reinterpret_cast<const char*>(rcl.buffer),')
        w('                     rcl.buffer_length);')
        w('}')
        w('')
        w('template <class MsgT>')
        w('inline MsgT deserializeRos(const void* data, size_t size) {')
        w('  rclcpp::SerializedMessage sm(size);')
        w('  auto& rcl = sm.get_rcl_serialized_message();')
        w('  if (data != nullptr && size > 0) {')
        w('    std::memcpy(rcl.buffer, data, size);')
        w('  }')
        w('  rcl.buffer_length = size;')
        w('  rclcpp::Serialization<MsgT> ser;')
        w('  MsgT out;')
        w('  ser.deserialize_message(&sm, &out);')
        w('  return out;')
        w('}')
        w('')
        w('inline void appendVarint32(std::string& out, uint32_t value) {')
        w('  while (value >= 0x80U) {')
        w('    out.push_back(static_cast<char>((value & 0x7FU) | 0x80U));')
        w('    value >>= 7U;')
        w('  }')
        w('  out.push_back(static_cast<char>(value));')
        w('}')
        w('')
        w('inline bool readVarint32(const unsigned char*& cursor,')
        w('                         const unsigned char* end,')
        w('                         uint32_t& value) {')
        w('  value = 0U;')
        w('  uint32_t shift = 0U;')
        w('  while (cursor < end && shift <= 28U) {')
        w('    const uint8_t byte = *cursor++;')
        w('    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;')
        w('    if ((byte & 0x80U) == 0U) {')
        w('      return true;')
        w('    }')
        w('    shift += 7U;')
        w('  }')
        w('  return false;')
        w('}')
        w('')
        w('// -- forward declarations -----------------------------------------------')
        for msg in msgs:
            dfqn = self.ir.domain_fqn(msg.name)
            rfqn = self.ir.ros_msg_type(msg.name)
            w(f'inline void toRos(const {dfqn}& d, {rfqn}& r);')
            w(f'inline void fromRos(const {rfqn}& r, {dfqn}& d);')
        w('')
        w('// -- converters ---------------------------------------------------------')
        for msg in msgs:
            self._emit_converters(out, msg)
        w('// -- wire codec (toBinary / fromBinary<T>) ------------------------------')
        for msg in msgs:
            dfqn = self.ir.domain_fqn(msg.name)
            rfqn = self.ir.ros_msg_type(msg.name)
            w(f'inline std::string toBinary(const {dfqn}& v) {{')
            w(f'  {rfqn} m; toRos(v, m); return serializeRos(m);')
            w('}')
            w(f'inline {dfqn} fromBinary{msg.name}(const void* data, size_t size) {{')
            w(f'  auto m = deserializeRos<{rfqn}>(data, size);')
            w(f'  {dfqn} v; fromRos(m, v); return v;')
            w('}')
            w(f'inline {dfqn} fromBinary{msg.name}(const std::string& s) {{')
            w(f'  return fromBinary{msg.name}(s.data(), s.size());')
            w('}')
            w(f'inline std::string toBinary(const std::vector<{dfqn}>& values) {{')
            w('  std::string out;')
            w('  for (const auto& item : values) {')
            w('    const std::string frame = toBinary(item);')
            w('    if (frame.size() > std::numeric_limits<uint32_t>::max()) {')
            w('      throw std::length_error("ROS2 frame exceeds uint32_t");')
            w('    }')
            w('    appendVarint32(out, static_cast<uint32_t>(frame.size()));')
            w('    out.append(frame);')
            w('  }')
            w('  return out;')
            w('}')
            w(f'inline std::vector<{dfqn}> fromBinary{msg.name}Array(')
            w('    const void* data, size_t size) {')
            w(f'  std::vector<{dfqn}> values;')
            w('  if (data == nullptr && size != 0) {')
            w('    throw std::runtime_error("null ROS2 array payload");')
            w('  }')
            w('  if (size == 0) {')
            w('    return values;')
            w('  }')
            w('  const auto* cursor = static_cast<const unsigned char*>(data);')
            w('  const auto* end = cursor + size;')
            w('  while (cursor < end) {')
            w('    uint32_t frame_size = 0U;')
            w('    if (!readVarint32(cursor, end, frame_size) ||')
            w('        static_cast<size_t>(end - cursor) < frame_size) {')
            w('      throw std::runtime_error("invalid ROS2 array frame");')
            w('    }')
            w(f'    values.push_back(fromBinary{msg.name}(cursor, frame_size));')
            w('    cursor += frame_size;')
            w('  }')
            w('  return values;')
            w('}')
            w('')
        w(f'}}  // namespace {codec_ns}')

        path.write_text('\n'.join(out) + '\n', encoding='utf-8')
        return [path]

    def _emit_converters(self, out: List[str], msg: ProtoMessage) -> None:
        dfqn = self.ir.domain_fqn(msg.name)
        rfqn = self.ir.ros_msg_type(msg.name)
        fields = self.ir.fields(msg)

        out.append(f'inline void toRos(const {dfqn}& d, {rfqn}& r) {{')
        out.append('  (void)d; (void)r;')
        for df in fields:
            out.extend(self._to_ros_field(df))
        out.append('}')

        out.append(f'inline void fromRos(const {rfqn}& r, {dfqn}& d) {{')
        out.append('  (void)d; (void)r;')
        for df in fields:
            out.extend(self._from_ros_field(df))
        out.append('}')
        out.append('')


def generate_ros2_codec(index: ProtoTypeIndex, output_dir: Path,
                        naming_policy=None, package: str = '') -> List[Path]:
    return Ros2MarshalGenerator(index, naming_policy, package).generate(output_dir)


def _main(argv: List[str]) -> int:
    if len(argv) < 3:
        print('Usage: python ros2_marshal_codegen.py <proto_dir> <out_dir>',
              file=sys.stderr)
        return 2
    from proto_parser import parse_proto_tree  # noqa: E402
    index = ProtoTypeIndex(parse_proto_tree(Path(argv[1])))
    written = generate_ros2_codec(index, Path(argv[2]))
    print(f'Wrote {len(written)} ROS2 codec file(s) to {argv[2]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(_main(sys.argv))
