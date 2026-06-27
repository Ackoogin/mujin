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
from ros2_ir import DomainField, DomainIR, pascal, ros_header_stem  # noqa: E402

CODEC_NS = 'pyramid::ros2_codec'


class Ros2MarshalGenerator:
    def __init__(self, index: ProtoTypeIndex):
        self.index = index
        self.ir = DomainIR(index)

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
        path = output_dir / 'pyramid_ros2_codec.hpp'
        msgs = self.ir.emitted_messages()

        out: List[str] = []
        w = out.append
        w('// Auto-generated ROS2 marshalling + codec -- do not edit')
        w('// pyramid::domain_model <-> pyramid_msgs ROS2 messages + rclcpp wire codec')
        w('#pragma once')
        w('')
        w('#include "pyramid_data_model_types.hpp"')
        w('')
        for enum in self.index.all_enums():
            w(f'#include "pyramid_msgs/msg/{ros_header_stem(pascal(enum.name))}.hpp"')
        for msg in msgs:
            w(f'#include "pyramid_msgs/msg/{ros_header_stem(pascal(msg.name))}.hpp"')
        w('')
        w('#include <rclcpp/serialization.hpp>')
        w('#include <rclcpp/serialized_message.hpp>')
        w('')
        w('#include <cstdint>')
        w('#include <cstring>')
        w('#include <string>')
        w('#include <utility>')
        w('')
        w(f'namespace {CODEC_NS} {{')
        w('')
        w('namespace domain = pyramid::domain_model;')
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
            w('')
        w(f'}}  // namespace {CODEC_NS}')

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


def generate_ros2_codec(index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
    return Ros2MarshalGenerator(index).generate(output_dir)


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
