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
from typing import List

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoMessage,
    camel_to_snake,
)
import codec_backends


class ProtobufBackend(codec_backends.CodecBackend):

    @property
    def name(self) -> str:
        return 'protobuf'

    @property
    def content_type(self) -> str:
        return 'application/protobuf'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        generated = []

        for pf in index.files:
            if not pf.messages:
                continue

            pkg_parts = [p for p in pf.package.split('.') if p]
            file_base = '_'.join(pkg_parts)
            ns = '::'.join(pkg_parts) + '::protobuf_codec'

            # The protoc-generated header (e.g. pyramid/data_model/tactical.pb.h)
            proto_rel = str(pf.path.with_suffix('.pb.h')).replace('\\', '/')
            # Simplify: use just the stem
            pb_header = '/'.join(pf.package.split('.')) + '.pb.h'

            hpp_path = output_dir / (file_base + '_protobuf_codec.hpp')
            self._write_cpp_wrapper(hpp_path, ns, pb_header, pf, index)
            generated.append(hpp_path)

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

        with open(path, 'w') as f:
            f.write(f'// Auto-generated Protobuf PCL codec — do not edit\n')
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

    # -- Ada spec (C interop to protobuf C++ ) ---------------------------------

    def _write_ada_spec(self, path: Path, pf: ProtoFile, index: ProtoTypeIndex):
        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        pkg_name = '.'.join(pkg_parts) + '.Protobuf_Codec'

        with open(path, 'w') as f:
            f.write(f'--  Auto-generated Protobuf codec spec — do not edit\n')
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
