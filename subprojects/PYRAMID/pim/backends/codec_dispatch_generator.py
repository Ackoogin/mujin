#!/usr/bin/env python3
"""
Codec Dispatch Generator

Generates the C++ and Ada codec dispatch layer that sits between component
business logic and the PCL transport.  Components work with typed structs;
the dispatch layer handles serialisation using whichever codec is configured
on the port.

Port-level codec selection:
  - Each port's type_name (set at pcl_container_add_publisher/subscriber/service
    time) determines which codec is used: "application/json",
    "application/flatbuffers", "application/protobuf".
  - The dispatch layer reads the port's type_name and routes to the correct
    codec's serialize/deserialize function.
  - Component business logic never touches raw bytes or codec-specific APIs.

Generated artefacts:
  - codec_dispatch.hpp / .cpp — C++ codec dispatcher
  - codec_dispatch.ads / .adb — Ada codec dispatcher

Architecture:
  component logic → typed structs → CodecDispatch → raw bytes → PCL port
"""

from pathlib import Path
from typing import List

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from proto_parser import (
    ProtoTypeIndex, ProtoFile, ProtoMessage, ProtoEnum,
    camel_to_snake, lc_first, screaming_to_pascal,
)

_SEP = '// ' + '-' * 75


def generate_cpp_dispatch(index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
    """Generate C++ codec dispatch header and implementation."""
    output_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    for pf in index.files:
        if not pf.messages:
            continue

        pkg_parts = [p for p in pf.package.split('.') if p]
        file_base = '_'.join(pkg_parts)
        ns = '::'.join(pkg_parts)

        hpp_path = output_dir / (file_base + '_codec_dispatch.hpp')
        cpp_path = output_dir / (file_base + '_codec_dispatch.cpp')

        _write_cpp_header(hpp_path, ns, file_base, pf, index)
        _write_cpp_impl(cpp_path, ns, file_base, pf, index)
        generated.extend([hpp_path, cpp_path])

    return generated


def generate_ada_dispatch(index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
    """Generate Ada codec dispatch spec and body."""
    output_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    for pf in index.files:
        if not pf.messages:
            continue

        pkg_parts = [p.capitalize() for p in pf.package.split('.') if p]
        file_base = '_'.join(p.lower() for p in pkg_parts)
        pkg_name = '.'.join(pkg_parts) + '.Codec_Dispatch'

        spec_path = output_dir / (file_base + '-codec_dispatch.ads')
        body_path = output_dir / (file_base + '-codec_dispatch.adb')

        _write_ada_spec(spec_path, pkg_name, pf, index)
        _write_ada_body(body_path, pkg_name, pf, index)
        generated.extend([spec_path, body_path])

    return generated


# -- C++ header ----------------------------------------------------------------

def _write_cpp_header(path: Path, ns: str, file_base: str,
                      pf: ProtoFile, index: ProtoTypeIndex):
    dispatch_ns = ns + '::codec_dispatch'

    with open(path, 'w') as f:
        f.write(f'// Auto-generated codec dispatch — do not edit\n')
        f.write(f'// Namespace: {dispatch_ns}\n')
        f.write(f'//\n')
        f.write(f'// Port-level codec routing: reads pcl_msg_t.type_name to select\n')
        f.write(f'// the correct codec for serialisation/deserialisation.\n')
        f.write(f'//\n')
        f.write(f'// Supported codecs (compile-time feature flags):\n')
        f.write(f'//   "application/json"        — always available\n')
        f.write(f'//   "application/flatbuffers"  — requires CODEC_FLATBUFFERS\n')
        f.write(f'//   "application/protobuf"     — requires CODEC_PROTOBUF\n')
        f.write(f'#pragma once\n\n')

        f.write(f'#include <pcl/pcl_types.h>\n')
        f.write(f'#include <string>\n')
        f.write(f'#include <stdexcept>\n')
        f.write(f'#include <cstring>\n\n')

        # Include codec headers conditionally
        f.write(f'// JSON codec (always available)\n')
        f.write(f'#include "{file_base}_json_codec.hpp"\n\n')
        f.write(f'#if defined(CODEC_FLATBUFFERS)\n')
        f.write(f'#include "{file_base}_flatbuffers_codec.hpp"\n')
        f.write(f'#endif\n\n')
        f.write(f'#if defined(CODEC_PROTOBUF)\n')
        f.write(f'#include "{file_base}_protobuf_codec.hpp"\n')
        f.write(f'#endif\n\n')

        f.write(f'namespace {dispatch_ns} {{\n\n')

        # Content type constants
        f.write(f'{_SEP}\n')
        f.write(f'// Content type constants\n')
        f.write(f'{_SEP}\n\n')
        f.write(f'constexpr const char* kJson        = "application/json";\n')
        f.write(f'constexpr const char* kFlatBuffers  = "application/flatbuffers";\n')
        f.write(f'constexpr const char* kProtobuf     = "application/protobuf";\n\n')

        # Per-message serialize/deserialize that routes on content_type
        f.write(f'{_SEP}\n')
        f.write(f'// Codec-dispatched serialisation\n')
        f.write(f'//\n')
        f.write(f'// Each function takes a content_type string (from port config) and\n')
        f.write(f'// routes to the appropriate codec.  Component logic calls these\n')
        f.write(f'// instead of codec-specific toJson/toBinary directly.\n')
        f.write(f'{_SEP}\n\n')

        json_ns = ns + '::json_codec'

        for msg in pf.messages:
            # serialize: typed struct → std::string (wire bytes)
            f.write(f'/// Serialize {msg.name} using the codec identified by content_type.\n')
            f.write(f'inline std::string serialize(const {json_ns}::{msg.name}& msg,\n')
            f.write(f'                             const char* content_type) {{\n')
            f.write(f'    if (std::strcmp(content_type, kJson) == 0)\n')
            f.write(f'        return {json_ns}::toJson(msg);\n')
            f.write(f'#if defined(CODEC_FLATBUFFERS)\n')
            f.write(f'    if (std::strcmp(content_type, kFlatBuffers) == 0)\n')
            f.write(f'        return {ns}::flatbuffers_codec::toBinary(msg);\n')
            f.write(f'#endif\n')
            f.write(f'#if defined(CODEC_PROTOBUF)\n')
            f.write(f'    if (std::strcmp(content_type, kProtobuf) == 0)\n')
            f.write(f'        return {ns}::protobuf_codec::toBinary(msg);\n')
            f.write(f'#endif\n')
            f.write(f'    throw std::runtime_error(\n')
            f.write(f'        std::string("unsupported codec: ") + content_type);\n')
            f.write(f'}}\n\n')

            # deserialize: raw bytes → typed struct
            f.write(f'/// Deserialize {msg.name} using the codec identified by content_type.\n')
            f.write(f'inline {json_ns}::{msg.name} deserialize{msg.name}(\n')
            f.write(f'    const void* data, size_t size, const char* content_type) {{\n')
            f.write(f'    if (std::strcmp(content_type, kJson) == 0)\n')
            f.write(f'        return {json_ns}::{lc_first(msg.name)}FromJson(\n')
            f.write(f'            std::string(static_cast<const char*>(data), size));\n')
            f.write(f'    // FlatBuffers and Protobuf deserialize to their own types;\n')
            f.write(f'    // conversion to the common type requires a mapping layer.\n')
            f.write(f'    throw std::runtime_error(\n')
            f.write(f'        std::string("unsupported codec for deserialization: ") + content_type);\n')
            f.write(f'}}\n\n')

        # Convenience: serialize + wrap into pcl_msg_t
        f.write(f'{_SEP}\n')
        f.write(f'// PCL message helpers\n')
        f.write(f'{_SEP}\n\n')

        f.write(f'/// Build a pcl_msg_t from a serialized payload and content type.\n')
        f.write(f'/// The caller must keep `payload` alive for the lifetime of the returned msg.\n')
        f.write(f'inline pcl_msg_t makeMsg(const std::string& payload,\n')
        f.write(f'                         const char* content_type) {{\n')
        f.write(f'    pcl_msg_t msg{{}};\n')
        f.write(f'    msg.data      = payload.data();\n')
        f.write(f'    msg.size      = static_cast<uint32_t>(payload.size());\n')
        f.write(f'    msg.type_name = content_type;\n')
        f.write(f'    return msg;\n')
        f.write(f'}}\n\n')

        f.write(f'}} // namespace {dispatch_ns}\n')


# -- C++ implementation --------------------------------------------------------

def _write_cpp_impl(path: Path, ns: str, file_base: str,
                    pf: ProtoFile, index: ProtoTypeIndex):
    dispatch_ns = ns + '::codec_dispatch'

    with open(path, 'w') as f:
        f.write(f'// Auto-generated codec dispatch — do not edit\n')
        f.write(f'// All dispatch logic is inline in the header.\n')
        f.write(f'// This file is reserved for future non-inline implementations.\n\n')
        f.write(f'#include "{file_base}_codec_dispatch.hpp"\n')


# -- Ada spec ------------------------------------------------------------------

def _write_ada_spec(path: Path, pkg_name: str, pf: ProtoFile,
                    index: ProtoTypeIndex):
    with open(path, 'w') as f:
        f.write(f'--  Auto-generated codec dispatch — do not edit\n')
        f.write(f'--  Package: {pkg_name}\n')
        f.write(f'--\n')
        f.write(f'--  Port-level codec routing for Ada components.\n')
        f.write(f'--  Routes to Json_Codec, Flatbuffers_Codec, or Protobuf_Codec\n')
        f.write(f'--  based on the content type string from port configuration.\n\n')
        f.write(f'with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;\n\n')
        f.write(f'package {pkg_name} is\n\n')

        f.write(f'   --  Content type constants\n')
        f.write(f'   Json_Content_Type        : constant String := "application/json";\n')
        f.write(f'   Flatbuffers_Content_Type  : constant String := "application/flatbuffers";\n')
        f.write(f'   Protobuf_Content_Type     : constant String := "application/protobuf";\n\n')

        for msg in pf.messages:
            ada_name = camel_to_snake(msg.name)
            f.write(f'   --  Serialize {msg.name} using the codec identified by Content_Type.\n')
            f.write(f'   function Serialize_{ada_name}\n')
            f.write(f'     (Msg          : {ada_name};\n')
            f.write(f'      Content_Type : String) return String;\n\n')

            f.write(f'   --  Deserialize {msg.name} from raw bytes using Content_Type.\n')
            f.write(f'   function Deserialize_{ada_name}\n')
            f.write(f'     (Data         : String;\n')
            f.write(f'      Content_Type : String) return {ada_name};\n\n')

        f.write(f'end {pkg_name};\n')


# -- Ada body ------------------------------------------------------------------

def _write_ada_body(path: Path, pkg_name: str, pf: ProtoFile,
                    index: ProtoTypeIndex):
    with open(path, 'w') as f:
        f.write(f'--  Auto-generated codec dispatch — do not edit\n\n')
        f.write(f'with Ada.Exceptions;\n\n')
        f.write(f'package body {pkg_name} is\n\n')

        for msg in pf.messages:
            ada_name = camel_to_snake(msg.name)

            # Serialize
            f.write(f'   function Serialize_{ada_name}\n')
            f.write(f'     (Msg          : {ada_name};\n')
            f.write(f'      Content_Type : String) return String\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Content_Type = Json_Content_Type then\n')
            f.write(f'         return Json_Codec.To_Json (Msg);\n')
            f.write(f'      end if;\n')
            f.write(f'      raise Program_Error with\n')
            f.write(f'        "Unsupported codec: " & Content_Type;\n')
            f.write(f'   end Serialize_{ada_name};\n\n')

            # Deserialize
            f.write(f'   function Deserialize_{ada_name}\n')
            f.write(f'     (Data         : String;\n')
            f.write(f'      Content_Type : String) return {ada_name}\n')
            f.write(f'   is\n')
            f.write(f'   begin\n')
            f.write(f'      if Content_Type = Json_Content_Type then\n')
            f.write(f'         return Json_Codec.From_Json (Data);\n')
            f.write(f'      end if;\n')
            f.write(f'      raise Program_Error with\n')
            f.write(f'        "Unsupported codec: " & Content_Type;\n')
            f.write(f'   end Deserialize_{ada_name};\n\n')

        f.write(f'end {pkg_name};\n')
