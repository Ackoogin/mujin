#!/usr/bin/env python3
"""
Protobuf Codec Backend

Generates C++ and Ada wrapper code that uses protoc-generated classes for
serialisation, but routes through the existing PCL transport layer (not gRPC).

This gives protobuf wire efficiency while keeping PCL as the transport.
The protoc-generated .pb.h/.pb.cc files are expected to exist (generated
by protoc as a separate build step); this backend only produces the thin
PCL integration wrapper.
"""

from pathlib import Path
from typing import Dict, List, Tuple

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoField, ProtoMessage,
    camel_to_snake,
)
from binding_contract import PyramidCompatNamingPolicy
import codec_backends
from backends.flatbuffers_backend import (
    _alias_info,
    _collect_service_group,
    _cpp_type_namespace_for_type,
    _full_type_for,
    _message_spec_from_proto,
    _service_proto_groups,
)
from proto_resolve import (
    _field_with_type,
    _proto_type_fqn,
    _resolve_enum,
    _resolve_message,
)


_DEFAULT_NAMING_POLICY = PyramidCompatNamingPolicy()


def _proto_pb_header(pf, proto_import_root: Path = None) -> str:
    """Return the protoc-generated ``.pb.h`` include path for a proto file.

    protoc derives the output filename from the proto file's path relative to
    the import root (not from its package), so a file at
    ``proto/pyramid/data_model/pyramid.data_model.base.proto`` yields
    ``pyramid/data_model/pyramid.data_model.base.pb.h``.

    The import root is the directory passed as ``protoc -I``. When that root is
    not supplied, keep the legacy PYRAMID anchor as a compatibility fallback for
    direct backend callers.
    """
    pb_path = Path(pf.path).with_suffix('.pb.h')
    if proto_import_root is not None:
        try:
            return pb_path.relative_to(proto_import_root).as_posix()
        except ValueError:
            try:
                return pb_path.resolve().relative_to(
                    Path(proto_import_root).resolve()).as_posix()
            except ValueError:
                pass

    parts = pb_path.parts
    if 'pyramid' in parts:
        parts = parts[parts.index('pyramid'):]
    return '/'.join(parts)


class ProtobufBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'protobuf'

    @property
    def content_type(self) -> str:
        return 'application/protobuf'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path,
                     naming_policy=None,
                     proto_import_root: Path = None,
                     contract=None) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []
        naming = naming_policy or _DEFAULT_NAMING_POLICY

        for pf in index.files:
            if not pf.messages:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)
            ns = naming.protobuf_codec_namespace(pf.package)

            # protoc names its output after the proto FILE path (relative to the
            # proto import root), not the package, e.g.
            #   proto/pyramid/data_model/pyramid.data_model.base.proto
            #   -> pyramid/data_model/pyramid.data_model.base.pb.h
            # Mirror that here so the generated include resolves against protoc.
            pb_header = _proto_pb_header(pf, proto_import_root)

            hpp_path = output_dir / (file_base + '_protobuf_codec.hpp')
            self._write_cpp_wrapper(hpp_path, ns, pb_header, pf, index)
            generated.append(hpp_path)

        for base_package, files in _service_proto_groups(
                index, contract=contract, naming_policy=naming):
            group = _collect_service_group(index, base_package, files, naming)
            service_hpp = output_dir / (
                group.file_base + '_protobuf_codec.hpp')
            service_cpp = output_dir / (
                group.file_base + '_protobuf_codec.cpp')
            self._write_service_cpp_header(service_hpp, group, naming)
            self._write_service_cpp_impl(
                service_cpp, group, index, naming, proto_import_root)
            generated.extend([service_hpp, service_cpp])

        return generated

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.messages:
                continue

            pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
            file_base = '_'.join(p.lower() for p in pkg_parts)

            spec_path = output_dir / (file_base + '-protobuf_codec.ads')
            self._write_ada_spec(spec_path, pf, index)
            generated.append(spec_path)

        return generated

    # -- C++ PCL wrapper -------------------------------------------------------

    def _write_cpp_wrapper(self, path: Path, ns: str, pb_header: str,
                           pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p for p in pf.package.split('.') if p]
        # Proto C++ namespace uses :: between package segments
        proto_ns = '::'.join(pkg_parts)

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'// Auto-generated Protobuf PCL codec -- do not edit\n')
            f.write(f'// Backend: protobuf | Namespace: {ns}\n')
            f.write(f'//\n')
            f.write(f'// Wraps protoc-generated SerializeToString / ParseFromString\n')
            f.write(f'// into the PCL codec API surface (toBinary / fromBinary).\n')
            f.write(f'//\n')
            f.write(f'// Requires: protoc-generated {pb_header}\n')
            f.write(f'// Link with: libprotobuf\n')
            f.write(f'#pragma once\n\n')
            f.write(f'#include "{pb_header}"\n\n')
            f.write(f'#include <string>\n')
            f.write(f'#include <cstdint>\n\n')
            f.write(f'namespace {ns} {{\n\n')
            f.write(f'static constexpr const char* kContentType = "application/protobuf";\n\n')

            for msg in pf.messages:
                fqn = f'{proto_ns}::{msg.name}'

                # toBinary
                f.write(f'/// Serialise {msg.name} via protobuf.\n')
                f.write(f'inline std::string toBinary(const {fqn}& msg) {{\n')
                f.write(f'    std::string out;\n')
                f.write(f'    msg.SerializeToString(&out);\n')
                f.write(f'    return out;\n')
                f.write(f'}}\n\n')

                # fromBinary
                f.write(f'/// Deserialise {msg.name} from protobuf wire format.\n')
                f.write(f'inline {fqn} fromBinary{msg.name}(const void* data, size_t size) {{\n')
                f.write(f'    {fqn} result;\n')
                f.write(f'    result.ParseFromArray(data, static_cast<int>(size));\n')
                f.write(f'    return result;\n')
                f.write(f'}}\n\n')

                f.write(f'inline {fqn} fromBinary{msg.name}(const std::string& s) {{\n')
                f.write(f'    return fromBinary{msg.name}(s.data(), s.size());\n')
                f.write(f'}}\n\n')

            f.write(f'}} // namespace {ns}\n')

    # -- C++ local-struct service protobuf bridge -----------------------------

    def _write_service_cpp_header(self, path: Path, group, naming_policy) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        headers = self._service_type_headers(group, naming_policy)
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated service Protobuf codec\n')
            f.write(f'// Backend: protobuf | Namespace: {group.cpp_base_ns}::protobuf_codec\n')
            f.write(f'// Generated from proto service closure for {group.base_package}\n')
            f.write('#pragma once\n\n')
            for header in headers:
                f.write(f'#include "{header}"\n')
            f.write('\n')
            f.write('#include <cstddef>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write(f'namespace {group.cpp_base_ns}::protobuf_codec {{\n\n')
            f.write('static constexpr const char* kContentType = "application/protobuf";\n\n')

            for msg in group.message_specs:
                short = msg.cpp_type.split('::')[-1]
                f.write(f'std::string toBinary(const {msg.cpp_type}& msg);\n')
                f.write(f'{msg.cpp_type} fromBinary{short}(\n')
                f.write('    const void* data, size_t size);\n')
                f.write(f'inline {msg.cpp_type} fromBinary{short}(\n')
                f.write('    const std::string& s) {\n')
                f.write(f'  return fromBinary{short}(s.data(), s.size());\n')
                f.write('}\n\n')

            for array_spec in group.array_specs:
                f.write(f'std::string toBinary(const std::vector<{array_spec.element_cpp_type}>& msg);\n')
                f.write(f'std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(\n')
                f.write('    const void* data, size_t size);\n')
                f.write(f'inline std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(\n')
                f.write('    const std::string& s) {\n')
                f.write(f'  return fromBinary{array_spec.name}(s.data(), s.size());\n')
                f.write('}\n\n')

            f.write(f'}}  // namespace {group.cpp_base_ns}::protobuf_codec\n')

    @staticmethod
    def _service_type_headers(group, naming_policy) -> List[str]:
        headers = set()
        umbrella = naming_policy.umbrella_types_header([])
        if umbrella:
            headers.add(umbrella)
        for msg in group.message_specs:
            if '.' in msg.full_type:
                headers.add(naming_policy.type_header_for_package(
                    msg.full_type.rsplit('.', 1)[0]))
        for array_spec in group.array_specs:
            if '.' in array_spec.element_full_type:
                headers.add(naming_policy.type_header_for_package(
                    array_spec.element_full_type.rsplit('.', 1)[0]))
        return sorted(headers)

    def _write_service_cpp_impl(self, path: Path, group, index: ProtoTypeIndex,
                                naming_policy, proto_import_root: Path) -> None:
        message_by_full, package_by_full = self._message_maps(index)
        aliases = _alias_info(index)
        pb_headers = self._service_pb_headers(
            group, index, package_by_full, proto_import_root)
        helper_specs = self._message_specs_with_base_closure(
            group, index, aliases, naming_policy)

        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated service Protobuf codec\n')
            f.write(f'#include "{group.file_base}_protobuf_codec.hpp"\n\n')
            for header in pb_headers:
                f.write(f'#include "{header}"\n')
            f.write('\n')
            f.write('#include <cmath>\n')
            f.write('#include <cstdint>\n')
            f.write('#include <stdexcept>\n')
            f.write('#include <string>\n')
            f.write('#include <vector>\n\n')
            f.write(f'namespace {group.cpp_base_ns}::protobuf_codec {{\n\n')
            f.write('namespace {\n\n')
            self._emit_service_common_helpers(f)
            # Forward-declare every to_proto/from_proto helper up front so the
            # definitions can call one another regardless of the order they are
            # emitted in. message_specs are not topologically sorted by
            # dependency (e.g. a message may reference its base type before that
            # base's helpers are defined), so relying on definition order breaks
            # overload resolution for arbitrary contracts.
            for msg in helper_specs:
                if not msg.is_alias_root and \
                        message_by_full.get(msg.full_type) is None:
                    continue
                proto_t = self._proto_cpp_type(msg.full_type)
                f.write(f'{proto_t} to_proto(const {msg.cpp_type}& msg);\n')
                f.write(f'{msg.cpp_type} from_proto(const {proto_t}& msg);\n')
            f.write('\n')
            for msg in helper_specs:
                if msg.is_alias_root:
                    self._emit_alias_root_helpers(f, msg, aliases)
                    continue
                proto_msg = message_by_full.get(msg.full_type)
                proto_pkg = package_by_full.get(msg.full_type, '')
                if proto_msg is None:
                    continue
                self._emit_message_helpers(
                    f, msg, proto_msg, proto_pkg, index, aliases,
                    naming_policy)
            f.write('}  // namespace\n\n')
            self._emit_service_exports(f, group)
            f.write(f'}}  // namespace {group.cpp_base_ns}::protobuf_codec\n')

    def _message_specs_with_base_closure(self, group, index: ProtoTypeIndex,
                                         aliases, naming):
        """group.message_specs plus the transitive closure of inlinable-base
        message types.

        The shared service closure inlines `base` fields (FlatBuffers
        semantics), so a base message reached ONLY via another message's `base`
        field never lands in message_specs. The protobuf backend, however,
        emits ``to_proto(base)`` / ``from_proto(msg.base())`` for such fields and
        therefore needs those base types' helpers defined. Pull them in so the
        codec compiles for arbitrary contracts, not just ones that happen to
        reference every base type directly elsewhere."""
        message_by_full, package_by_full = self._message_maps(index)
        specs = list(group.message_specs)
        present = {spec.full_type for spec in specs}
        queue = [spec for spec in specs if not spec.is_alias_root]
        while queue:
            spec = queue.pop()
            proto_msg = message_by_full.get(spec.full_type)
            if proto_msg is None:
                continue
            proto_pkg = package_by_full.get(spec.full_type, '')
            for field in proto_msg.fields:
                if not self._is_inlinable_base(field, proto_pkg, index, aliases):
                    continue
                base_full = _full_type_for(index, field.type, proto_pkg)
                if base_full in present:
                    continue
                base_msg, base_pkg = _resolve_message(
                    index, field.type, proto_pkg)
                if base_msg is None:
                    continue
                base_spec = _message_spec_from_proto(
                    base_msg, base_full, index, aliases, base_pkg, naming)
                specs.append(base_spec)
                present.add(base_full)
                queue.append(base_spec)
        return specs

    @staticmethod
    def _message_maps(index: ProtoTypeIndex):
        message_by_full = {}
        package_by_full = {}
        for pf in index.files:
            for msg in pf.messages:
                full = f'{pf.package}.{msg.name}' if pf.package else msg.name
                message_by_full[full] = msg
                package_by_full[full] = pf.package
        return message_by_full, package_by_full

    @staticmethod
    def _proto_cpp_type(full_type: str) -> str:
        return '::' + full_type.replace('.', '::')

    def _service_pb_headers(self, group, index: ProtoTypeIndex,
                            package_by_full: Dict[str, str],
                            proto_import_root: Path) -> List[str]:
        packages = set()
        for msg in group.message_specs:
            if msg.full_type in package_by_full:
                packages.add(package_by_full[msg.full_type])
        for array_spec in group.array_specs:
            if array_spec.element_full_type in package_by_full:
                packages.add(package_by_full[array_spec.element_full_type])

        headers = []
        for pf in index.files:
            if pf.package in packages:
                headers.append(_proto_pb_header(pf, proto_import_root))
        return sorted(set(headers))

    @staticmethod
    def _emit_service_common_helpers(f) -> None:
        f.write('void append_varint32(uint32_t value, std::string& out) {\n')
        f.write('  while (value >= 0x80U) {\n')
        f.write('    out.push_back(static_cast<char>((value & 0x7FU) | 0x80U));\n')
        f.write('    value >>= 7U;\n')
        f.write('  }\n')
        f.write('  out.push_back(static_cast<char>(value));\n')
        f.write('}\n\n')
        f.write('bool read_varint32(const char*& cursor, const char* end, uint32_t& value) {\n')
        f.write('  value = 0U;\n')
        f.write('  uint32_t shift = 0U;\n')
        f.write('  while (cursor < end && shift <= 28U) {\n')
        f.write('    const auto byte = static_cast<uint8_t>(*cursor++);\n')
        f.write('    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;\n')
        f.write('    if ((byte & 0x80U) == 0U) {\n')
        f.write('      return true;\n')
        f.write('    }\n')
        f.write('    shift += 7U;\n')
        f.write('  }\n')
        f.write('  return false;\n')
        f.write('}\n\n')
        f.write('template <typename MessageT>\n')
        f.write('std::string serialize_proto(const MessageT& message, const char* type_name) {\n')
        f.write('  std::string out;\n')
        f.write('  if (!message.SerializeToString(&out)) {\n')
        f.write('    throw std::runtime_error(std::string("protobuf serialization failed for ") + type_name);\n')
        f.write('  }\n')
        f.write('  return out;\n')
        f.write('}\n\n')
        f.write('template <typename MessageT>\n')
        f.write('MessageT parse_proto(const void* data, size_t size, const char* type_name) {\n')
        f.write('  MessageT message;\n')
        f.write('  if ((data == nullptr && size != 0) ||\n')
        f.write('      !message.ParseFromArray(data, static_cast<int>(size))) {\n')
        f.write('    throw std::runtime_error(std::string("protobuf decode failed for ") + type_name);\n')
        f.write('  }\n')
        f.write('  return message;\n')
        f.write('}\n\n')
        f.write('template <typename ValueT, typename ProtoT, typename ToProtoFn>\n')
        f.write('std::string serialize_proto_array(const std::vector<ValueT>& values,\n')
        f.write('                                  ToProtoFn to_proto_fn,\n')
        f.write('                                  const char* type_name) {\n')
        f.write('  std::string out;\n')
        f.write('  for (const auto& value : values) {\n')
        f.write('    const auto encoded = serialize_proto<ProtoT>(to_proto_fn(value), type_name);\n')
        f.write('    append_varint32(static_cast<uint32_t>(encoded.size()), out);\n')
        f.write('    out.append(encoded);\n')
        f.write('  }\n')
        f.write('  return out;\n')
        f.write('}\n\n')
        f.write('template <typename ValueT, typename ProtoT, typename FromProtoFn>\n')
        f.write('std::vector<ValueT> parse_proto_array(const void* data, size_t size,\n')
        f.write('                                      FromProtoFn from_proto_fn,\n')
        f.write('                                      const char* type_name) {\n')
        f.write('  const char* cursor = static_cast<const char*>(data);\n')
        f.write('  const char* end = cursor + size;\n')
        f.write('  std::vector<ValueT> out;\n')
        f.write('  while (cursor < end) {\n')
        f.write('    uint32_t frame_size = 0U;\n')
        f.write('    if (!read_varint32(cursor, end, frame_size) ||\n')
        f.write('        static_cast<size_t>(end - cursor) < frame_size) {\n')
        f.write('      throw std::runtime_error(\n')
        f.write('          std::string("invalid protobuf array framing for ") + type_name);\n')
        f.write('    }\n')
        f.write('    ProtoT item;\n')
        f.write('    if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {\n')
        f.write('      throw std::runtime_error(std::string("protobuf decode failed for ") + type_name);\n')
        f.write('    }\n')
        f.write('    cursor += frame_size;\n')
        f.write('    out.push_back(from_proto_fn(item));\n')
        f.write('  }\n')
        f.write('  return out;\n')
        f.write('}\n\n')
        f.write('double timestamp_to_seconds(const ::pyramid::data_model::base::Timestamp& ts) {\n')
        f.write('  if (!ts.has_value()) {\n')
        f.write('    return 0.0;\n')
        f.write('  }\n')
        f.write('  return static_cast<double>(ts.value().seconds()) +\n')
        f.write("         static_cast<double>(ts.value().nanos()) / 1'000'000'000.0;\n")
        f.write('}\n\n')
        f.write('void set_timestamp(double seconds, ::pyramid::data_model::base::Timestamp* out) {\n')
        f.write('  if (out == nullptr) {\n')
        f.write('    return;\n')
        f.write('  }\n')
        f.write('  auto whole_seconds = static_cast<int64_t>(std::floor(seconds));\n')
        f.write('  auto nanos = static_cast<int32_t>(\n')
        f.write("      std::llround((seconds - static_cast<double>(whole_seconds)) * 1'000'000'000.0));\n")
        f.write("  if (nanos >= 1'000'000'000) {\n")
        f.write('    whole_seconds += 1;\n')
        f.write("    nanos -= 1'000'000'000;\n")
        f.write('  } else if (nanos < 0) {\n')
        f.write('    whole_seconds -= 1;\n')
        f.write("    nanos += 1'000'000'000;\n")
        f.write('  }\n')
        f.write('  auto* value = out->mutable_value();\n')
        f.write('  value->set_seconds(whole_seconds);\n')
        f.write('  value->set_nanos(nanos);\n')
        f.write('}\n\n')

    def _emit_alias_root_helpers(self, f, msg, aliases) -> None:
        short = msg.cpp_type.split('::')[-1]
        proto_t = self._proto_cpp_type(msg.full_type)
        _scalar, field_name = aliases[short]
        f.write(f'{proto_t} to_proto(const {msg.cpp_type}& msg) {{\n')
        f.write(f'  {proto_t} out;\n')
        if short == 'Timestamp':
            f.write('  set_timestamp(msg, &out);\n')
        else:
            f.write(f'  out.set_{field_name}(msg);\n')
        f.write('  return out;\n')
        f.write('}\n\n')
        f.write(f'{msg.cpp_type} from_proto(const {proto_t}& msg) {{\n')
        if short == 'Timestamp':
            f.write('  return timestamp_to_seconds(msg);\n')
        else:
            f.write(f'  return msg.{field_name}();\n')
        f.write('}\n\n')

    def _emit_message_helpers(self, f, msg, proto_msg, proto_pkg: str,
                              index: ProtoTypeIndex, aliases, naming) -> None:
        proto_t = self._proto_cpp_type(msg.full_type)
        f.write(f'{proto_t} to_proto(\n')
        f.write(f'    const {msg.cpp_type}& msg) {{\n')
        f.write(f'  {proto_t} out;\n')

        for field in proto_msg.fields:
            if self._is_inlinable_base(field, proto_pkg, index, aliases):
                self._emit_base_to_proto(
                    f, field, proto_msg, proto_pkg, index, aliases, naming)
            else:
                self._emit_field_to_proto(
                    f, field, field.name, 'msg.' + field.name, 'out',
                    proto_pkg, index, aliases, naming)
        for oneof in proto_msg.oneofs:
            for i, field in enumerate(oneof.fields):
                self._emit_field_to_proto(
                    f, field, field.name, 'msg.' + field.name, 'out',
                    proto_pkg, index, aliases, naming, force_optional=True,
                    optional_prefix='if' if i == 0 else 'else if')
        f.write('  return out;\n')
        f.write('}\n\n')

        f.write(f'{msg.cpp_type} from_proto(\n')
        f.write(f'    const {proto_t}& msg) {{\n')
        f.write(f'  {msg.cpp_type} out;\n')
        for field in proto_msg.fields:
            if self._is_inlinable_base(field, proto_pkg, index, aliases):
                self._emit_base_from_proto(
                    f, field, proto_msg, proto_pkg, index, aliases, naming)
            else:
                self._emit_field_from_proto(
                    f, field, field.name, 'out.' + field.name, 'msg',
                    proto_pkg, index, aliases, naming)
        for oneof in proto_msg.oneofs:
            for i, field in enumerate(oneof.fields):
                self._emit_field_from_proto(
                    f, field, field.name, 'out.' + field.name, 'msg',
                    proto_pkg, index, aliases, naming, force_optional=True,
                    optional_prefix='if' if i == 0 else 'else if')
        f.write('  return out;\n')
        f.write('}\n\n')

    @staticmethod
    def _is_inlinable_base(field: ProtoField, current_pkg: str,
                           index: ProtoTypeIndex, aliases) -> bool:
        if field.name != 'base' or field.is_repeated:
            return False
        base_msg, _base_pkg = _resolve_message(index, field.type, current_pkg)
        return base_msg is not None and base_msg.name not in aliases

    def _emit_base_to_proto(self, f, field: ProtoField, parent_msg,
                            current_pkg: str, index: ProtoTypeIndex,
                            aliases, naming) -> None:
        base_msg, base_pkg = _resolve_message(index, field.type, current_pkg)
        base_full = _full_type_for(index, field.type, current_pkg)
        base_cpp = _cpp_type_namespace_for_type(
            base_full, naming) + '::' + base_msg.name
        f.write(f'  {base_cpp} base;\n')
        for bf, parent_member, base_member_expr in self._base_member_map(
                field, parent_msg, base_msg, base_pkg, current_pkg, index,
                aliases):
            f.write(f'  base.{base_member_expr} = msg.{parent_member};\n')
        f.write('  *out.mutable_base() = to_proto(base);\n')

    def _emit_base_from_proto(self, f, field: ProtoField, parent_msg,
                              current_pkg: str, index: ProtoTypeIndex,
                              aliases, naming) -> None:
        base_msg, base_pkg = _resolve_message(index, field.type, current_pkg)
        if base_msg is None:
            return
        f.write('  if (msg.has_base()) {\n')
        f.write('    const auto base = from_proto(msg.base());\n')
        for bf, parent_member, base_member_expr in self._base_member_map(
                field, parent_msg, base_msg, base_pkg, current_pkg, index,
                aliases):
            f.write(f'    out.{parent_member} = base.{base_member_expr};\n')
        f.write('  }\n')

    def _base_member_map(self, field: ProtoField, parent_msg, base_msg,
                         base_pkg: str, current_pkg: str,
                         index: ProtoTypeIndex, aliases):
        del field
        own_names = {f.name for f in parent_msg.fields if f.name != 'base'}
        for bf in base_msg.fields:
            parent_member = bf.name
            if parent_member in own_names:
                parent_member = base_msg.name.lower() + '_' + parent_member
            nested_msg, nested_pkg = _resolve_message(index, bf.type, base_pkg)
            if (bf.name == 'base' and not bf.is_repeated
                    and nested_msg is not None
                    and nested_msg.name not in aliases):
                for nested in nested_msg.fields:
                    nested_type = _proto_type_fqn(
                        index, nested.type, nested_pkg) or nested.type
                    nested_field = _field_with_type(nested, nested_type)
                    yield nested_field, f'{parent_member}.{nested.name}', nested.name
            else:
                yield bf, parent_member, bf.name

    def _emit_field_to_proto(self, f, field: ProtoField, proto_name: str,
                             member_expr: str, out_expr: str,
                             current_pkg: str, index: ProtoTypeIndex,
                             aliases, naming, force_optional: bool = False,
                             optional_prefix: str = 'if') -> None:
        short = field.type.split('.')[-1]
        is_alias = short in aliases
        alias_field = aliases[short][1] if is_alias else ''
        field_full = self._resolved_full_type(index, field.type, current_pkg)
        is_enum = index.resolve_enum(field_full) is not None
        is_msg = index.resolve_message(field_full) is not None
        optional = force_optional or field.is_optional

        if field.is_repeated:
            f.write(f'  for (const auto& item : {member_expr}) {{\n')
            if is_alias:
                if short == 'Timestamp':
                    f.write(f'    set_timestamp(item, {out_expr}.add_{proto_name}());\n')
                else:
                    f.write(f'    {out_expr}.add_{proto_name}()->set_{alias_field}(item);\n')
            elif is_enum:
                proto_t = self._proto_cpp_type(field_full)
                f.write(f'    {out_expr}.add_{proto_name}(static_cast<{proto_t}>(\n')
                f.write('        static_cast<int>(item)));\n')
            elif is_msg:
                f.write(f'    *{out_expr}.add_{proto_name}() = to_proto(item);\n')
            else:
                f.write(f'    {out_expr}.add_{proto_name}(item);\n')
            f.write('  }\n')
            return

        value_expr = '*' + member_expr if optional and not (
            is_alias and aliases[short][0] == 'string' and not force_optional
        ) else member_expr
        if optional:
            if is_alias and aliases[short][0] == 'string' and not force_optional:
                f.write(f'  {optional_prefix} (!{member_expr}.empty()) {{\n')
                value_expr = member_expr
            else:
                f.write(f'  {optional_prefix} ({member_expr}.has_value()) {{\n')
        elif is_alias and aliases[short][0] == 'string' and field.is_optional:
            f.write(f'  if (!{member_expr}.empty()) {{\n')

        indent = '    ' if optional or (
            is_alias and aliases[short][0] == 'string' and field.is_optional
        ) else '  '
        if is_alias:
            if short == 'Timestamp':
                f.write(f'{indent}set_timestamp({value_expr}, {out_expr}.mutable_{proto_name}());\n')
            else:
                f.write(f'{indent}{out_expr}.mutable_{proto_name}()->set_{alias_field}({value_expr});\n')
        elif is_enum:
            proto_t = self._proto_cpp_type(field_full)
            f.write(f'{indent}{out_expr}.set_{proto_name}(static_cast<{proto_t}>(\n')
            f.write(f'{indent}    static_cast<int>({value_expr})));\n')
        elif is_msg:
            f.write(f'{indent}*{out_expr}.mutable_{proto_name}() = to_proto({value_expr});\n')
        else:
            f.write(f'{indent}{out_expr}.set_{proto_name}({value_expr});\n')

        if optional or (is_alias and aliases[short][0] == 'string' and field.is_optional):
            f.write('  }\n')

    def _emit_field_from_proto(self, f, field: ProtoField, proto_name: str,
                               member_expr: str, msg_expr: str,
                               current_pkg: str, index: ProtoTypeIndex,
                               aliases, naming, force_optional: bool = False,
                               optional_prefix: str = 'if') -> None:
        short = field.type.split('.')[-1]
        is_alias = short in aliases
        alias_field = aliases[short][1] if is_alias else ''
        field_full = self._resolved_full_type(index, field.type, current_pkg)
        is_enum = index.resolve_enum(field_full) is not None
        is_msg = index.resolve_message(field_full) is not None
        optional = force_optional or field.is_optional

        if field.is_repeated:
            f.write(f'  {member_expr}.reserve(static_cast<size_t>({msg_expr}.{proto_name}_size()));\n')
            f.write(f'  for (const auto& item : {msg_expr}.{proto_name}()) {{\n')
            if is_alias:
                if short == 'Timestamp':
                    f.write(f'    {member_expr}.push_back(timestamp_to_seconds(item));\n')
                else:
                    f.write(f'    {member_expr}.push_back(item.{alias_field}());\n')
            elif is_enum:
                native_t = _cpp_type_namespace_for_type(
                    field_full, naming) + '::' + short
                f.write(f'    {member_expr}.push_back(static_cast<{native_t}>(\n')
                f.write('        static_cast<int>(item)));\n')
            elif is_msg:
                f.write(f'    {member_expr}.push_back(from_proto(item));\n')
            else:
                f.write(f'    {member_expr}.push_back(item);\n')
            f.write('  }\n')
            return

        if optional or is_msg or is_alias:
            f.write(f'  {optional_prefix} ({msg_expr}.has_{proto_name}()) {{\n')
            indent = '    '
        else:
            indent = '  '

        if is_alias:
            if short == 'Timestamp':
                f.write(f'{indent}{member_expr} = timestamp_to_seconds({msg_expr}.{proto_name}());\n')
            else:
                f.write(f'{indent}{member_expr} = {msg_expr}.{proto_name}().{alias_field}();\n')
        elif is_enum:
            native_t = _cpp_type_namespace_for_type(
                field_full, naming) + '::' + short
            f.write(f'{indent}{member_expr} = static_cast<{native_t}>(\n')
            f.write(f'{indent}    static_cast<int>({msg_expr}.{proto_name}()));\n')
        elif is_msg:
            f.write(f'{indent}{member_expr} = from_proto({msg_expr}.{proto_name}());\n')
        else:
            f.write(f'{indent}{member_expr} = {msg_expr}.{proto_name}();\n')

        if optional or is_msg or is_alias:
            f.write('  }\n')

    @staticmethod
    def _resolved_full_type(index: ProtoTypeIndex, type_name: str,
                            current_pkg: str) -> str:
        if '.' in type_name:
            return type_name
        enum, enum_pkg = _resolve_enum(index, type_name, current_pkg)
        if enum is not None and enum_pkg:
            return f'{enum_pkg}.{enum.name}'
        msg, msg_pkg = _resolve_message(index, type_name, current_pkg)
        if msg is not None and msg_pkg:
            return f'{msg_pkg}.{msg.name}'
        return _full_type_for(index, type_name, current_pkg)

    def _emit_service_exports(self, f, group) -> None:
        for msg in group.message_specs:
            short = msg.cpp_type.split('::')[-1]
            proto_t = self._proto_cpp_type(msg.full_type)
            f.write(f'std::string toBinary(const {msg.cpp_type}& msg) {{\n')
            f.write(f'  return serialize_proto(to_proto(msg), "{short}");\n')
            f.write('}\n\n')
            f.write(f'{msg.cpp_type} fromBinary{short}(const void* data,\n')
            f.write('                                         size_t size) {\n')
            f.write(f'  return from_proto(parse_proto<{proto_t}>(\n')
            f.write(f'      data, size, "{short}"));\n')
            f.write('}\n\n')

        for array_spec in group.array_specs:
            short = array_spec.name[:-len('Array')]
            proto_t = self._proto_cpp_type(array_spec.element_full_type)
            f.write(f'std::string toBinary(const std::vector<{array_spec.element_cpp_type}>& msg) {{\n')
            f.write(f'  return serialize_proto_array<{array_spec.element_cpp_type},\n')
            f.write(f'                               {proto_t}>(\n')
            f.write('      msg,\n')
            f.write(f'      [](const {array_spec.element_cpp_type}& value) {{\n')
            f.write('        return to_proto(value);\n')
            f.write('      },\n')
            f.write(f'      "{short}");\n')
            f.write('}\n\n')
            f.write(f'std::vector<{array_spec.element_cpp_type}> fromBinary{array_spec.name}(\n')
            f.write('    const void* data, size_t size) {\n')
            f.write(f'  return parse_proto_array<{array_spec.element_cpp_type},\n')
            f.write(f'                           {proto_t}>(\n')
            f.write('      data, size,\n')
            f.write(f'      [](const {proto_t}& value) {{\n')
            f.write('        return from_proto(value);\n')
            f.write('      },\n')
            f.write(f'      "{short}");\n')
            f.write('}\n\n')

    # -- Ada spec (C interop to protobuf C++ ) ---------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.Protobuf_Codec'

        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(f'--  Auto-generated Protobuf codec spec -- do not edit\n')
            f.write(f'--  Backend: protobuf | Package: {pkg_name}\n')
            f.write(f'--\n')
            f.write(f'--  Thin Ada binding to protobuf C++ codec via C interop.\n')
            f.write(f'--  Protobuf has no native Ada support; serialisation is\n')
            f.write(f'--  delegated to the C++ implementation via pragma Import.\n\n')
            f.write(f'with Interfaces.C; use Interfaces.C;\n')
            f.write(f'with Interfaces.C.Strings;\n')
            f.write(f'with System;\n\n')
            f.write(f'package {pkg_name} is\n\n')
            f.write(f'   Content_Type : constant String := "application/protobuf";\n\n')

            for msg in pf.messages:
                ada_name = camel_to_snake(msg.name)
                f.write(f'   --  {msg.name}: protobuf SerializeToString / ParseFromArray\n')
                f.write(f'   function To_Binary_{ada_name} (Msg : System.Address)\n')
                f.write(f'     return Interfaces.C.Strings.chars_ptr\n')
                f.write(f'     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_to_protobuf";\n\n')
                f.write(f'   function From_Binary_{ada_name}\n')
                f.write(f'     (Data : System.Address; Size : Interfaces.C.size_t)\n')
                f.write(f'     return System.Address\n')
                f.write(f'     with Import, Convention => C,\n')
                f.write(f'          External_Name => "{msg.name}_from_protobuf";\n\n')

            f.write(f'end {pkg_name};\n')


# -- Register ------------------------------------------------------------------

codec_backends.register(ProtobufBackend())
