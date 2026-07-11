#!/usr/bin/env python3
"""Emitter for the OMS-JSON PCL codec plugin (UCI-schema-shaped packages).

Generalized in UCI MMS conversion plan Phase 2 from the seam-only emitter:
one plugin per UCI-shaped data-model package, driven by the package's
``wire_names.json`` sidecar when present (plan D3 -- wire names are data),
falling back to the snake->Pascal heuristic for hand-authored contracts
like ``pim/uci_seam_example`` (whose emitted wire behaviour is the frozen
D4(b) regression bar).

Wire rules, pinned from OMSC-SPC-013 via the verified hand codec and the
independently-authored ``la-cal-harness`` XSD-derived generator:

* global-element wrapper: one root key, the XSD *element* name;
* XSD extension (proto ``base`` composition) is invisible on the wire; the
  native struct layer already inlines one level (``types_gen``), and any
  deeper retained ``base`` member is flattened here (``json::update`` on
  encode, shared-object decode);
* ``xs:choice`` (proto oneof): the active member's element key appears
  directly in the parent object -- no wrapper, no discriminator;
* repeated choice *members* (synthesized ``*_List`` wrappers from
  xsd2proto) render as an array under the member's element key;
* enums: the XSD enumeration literal verbatim (sidecar-authoritative;
  enum-prefix-strip heuristic otherwise); ``*_UNSPECIFIED`` never appears
  on the wire -- encoding it is an error;
* NaN/Infinity as strings; UUID-named fields RFC-4122 validated.

Known unimplemented shape (loud, not silent): repeated ``xs:choice``
itself (synthesized ``*_Choice`` wrapper as a message field) -- absent
from profile P1; generation fails with a clear message if reached.
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple
import json as _json

from proto_parser import (ProtoField, ProtoFile, ProtoMessage,
                          ProtoTypeIndex, screaming_to_pascal)
from .naming import _DEFAULT_NAMING_POLICY
from .types_gen import find_scalar_wrappers

_SCALARS = {'double', 'float', 'int32', 'int64', 'uint32', 'uint64',
            'sint32', 'sint64', 'fixed32', 'fixed64', 'sfixed32', 'sfixed64',
            'bool', 'string', 'bytes'}
_NUMBERS = _SCALARS - {'bool', 'string', 'bytes'}
_ACRONYMS = {'id': 'ID', 'uuid': 'UUID'}


def _key(name: str) -> str:
    """Heuristic UCI element key: snake case with the small UCI acronym
    vocabulary.  Only trusted for hand-authored contracts; sidecar-backed
    packages carry authoritative names instead."""
    return ''.join(_ACRONYMS.get(word, word.capitalize())
                   for word in name.split('_'))


class OmsJsonShapeError(RuntimeError):
    """A package shape this emitter cannot put on the UCI wire."""


class _WireNames:
    """Plan-D3 wire-name source for one data-model package."""

    def __init__(self, pf: ProtoFile):
        self.sidecar: Optional[dict] = None
        for parent in list(pf.path.parents)[:4]:
            candidate = parent / 'wire_names.json'
            if candidate.is_file():
                doc = _json.loads(candidate.read_text(encoding='utf-8'))
                if doc.get('package') == pf.package:
                    self.sidecar = doc
                break

    def roots(self) -> Optional[Dict[str, str]]:
        """element -> message, or None when no sidecar (heuristic path)."""
        if self.sidecar is None:
            return None
        return dict(self.sidecar.get('roots', {}))

    def field_key(self, msg_name: str, field_name: str) -> Optional[str]:
        """The XSD element name for a field, or None when the field has no
        wire presence of its own (retained base members, choice carriers)."""
        if self.sidecar is None:
            return _key(field_name)
        entry = (self.sidecar.get('messages', {}).get(msg_name, {})
                 .get('fields', {}).get(field_name))
        return entry.get('element') if entry else None

    def is_synthesized(self, msg_name: str) -> bool:
        if self.sidecar is None:
            return False
        return bool(self.sidecar.get('messages', {}).get(msg_name, {})
                    .get('synthesized'))

    def enum_literal(self, enum_name: str, value_name: str,
                     suffix: Optional[str]) -> str:
        if self.sidecar is not None:
            lit = self.sidecar.get('enums', {}).get(enum_name, {}) \
                .get(value_name)
            if lit is not None:
                return lit
        return suffix if suffix else value_name


class CppOmsJsonCodecGenerator:
    """Generate one PCL codec plugin per UCI-shaped data-model package."""

    def __init__(self, index: ProtoTypeIndex, naming_policy=None):
        self.index = index
        self.naming = naming_policy or _DEFAULT_NAMING_POLICY
        self.aliases = find_scalar_wrappers(index)

    def generate(self, out: Path) -> List[Path]:
        out.mkdir(parents=True, exist_ok=True)
        paths: List[Path] = []
        for pf in self.index.files:
            if not pf.messages or pf.services:
                continue
            emitter = _PackageEmitter(self.index, pf, self.naming,
                                      self.aliases)
            if not emitter.roots:
                continue  # no wire entry points -- not a UCI payload package
            prefix = pf.package.replace('.', '_')
            path = out / f'{prefix}_oms_json_codec_plugin.cpp'
            emitter.write(path)
            paths.append(path)
        return paths


class _PackageEmitter:
    def __init__(self, index: ProtoTypeIndex, pf: ProtoFile, naming,
                 aliases: Dict[str, str]):
        self.index = index
        self.pf = pf
        self.naming = naming
        self.aliases = aliases
        self.wire = _WireNames(pf)
        self.prefix = pf.package.replace('.', '_')
        self.msgs: Dict[str, ProtoMessage] = {m.name: m for m in pf.messages}
        self.enums = {e.name: e for e in pf.enums}
        self.roots = self._resolve_roots()
        self.wrappers = self._resolve_wrappers()

    # -- shape resolution ---------------------------------------------------

    def _short(self, type_name: str) -> str:
        return type_name.split('.')[-1]

    def _msg(self, type_name: str) -> Optional[ProtoMessage]:
        return self.msgs.get(self._short(type_name))

    def _resolve_roots(self) -> List[Tuple[str, str]]:
        """[(element, message)] in deterministic order."""
        sidecar_roots = self.wire.roots()
        if sidecar_roots is not None:
            return [(el, msg) for el, msg in sidecar_roots.items()]
        # Heuristic path: data-model messages referenced as variants of any
        # *_Service_Request/_Requirement wrapper, in data-model doc order.
        referenced = set()
        for other in self.index.files:
            for msg in other.messages:
                if not msg.name.split('_Service_')[-1] in (
                        'Request', 'Requirement'):
                    continue
                for oo in msg.oneofs:
                    for fld in oo.fields:
                        if self._msg(fld.type) is not None:
                            referenced.add(self._short(fld.type))
        return [(m.name, m.name) for m in self.pf.messages
                if m.name in referenced]

    def _resolve_wrappers(self) -> List[dict]:
        """Request-port wrapper messages whose variants live in this
        package: [{name, wire_struct, variants:[(field, type_short)]}]."""
        wrappers = []
        seen = set()
        for other in self.index.files:
            for msg in other.messages:
                parts = msg.name.split('_Service_')
                if len(parts) != 2 or parts[1] not in ('Request',
                                                       'Requirement'):
                    continue
                if msg.name in seen:
                    # Provided/consumed mirrors declare identical wrappers;
                    # the codec needs exactly one Wire struct per name.
                    continue
                if msg.fields or len(msg.oneofs) != 1:
                    continue
                variants = [(f.name, self._short(f.type))
                            for f in msg.oneofs[0].fields]
                if not variants or any(v not in self.msgs
                                       for _, v in variants):
                    continue
                seen.add(msg.name)
                wrappers.append({
                    'name': msg.name,
                    'wire_struct': f'{parts[0]}{parts[1]}Wire',
                    'variants': variants,
                })
        return wrappers

    def _element_for_message(self, msg_name: str) -> str:
        for element, message in self.roots:
            if message == msg_name:
                return element
        return msg_name

    def _emittable_messages(self) -> List[ProtoMessage]:
        """Messages that get encode_/decode_ functions: everything except
        synthesized wrappers, which are handled inline at their use site."""
        return [m for m in self.pf.messages
                if not self.wire.is_synthesized(m.name)]

    def _is_list_wrapper(self, type_name: str) -> Optional[ProtoField]:
        """If type is a synthesized single-repeated-'items' wrapper, return
        its items field (repeated choice member rule)."""
        short = self._short(type_name)
        msg = self.msgs.get(short)
        if (msg is not None and self.wire.is_synthesized(short)
                and not msg.oneofs and len(msg.fields) == 1
                and msg.fields[0].name == 'items'
                and msg.fields[0].is_repeated):
            return msg.fields[0]
        return None

    def _walk(self, msg: ProtoMessage):
        """Mirror types_gen._inline_base_fields: yield the package-level
        fields as they exist on the generated native struct.

        Yields ('field', field, member_name, owner_msg_name) for ordinary
        members and ('flatten', member_name, base_type_short) for a retained
        deeper 'base' member (extension chains of depth 2)."""
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for field in msg.fields:
            if field.name == 'base' and not field.is_repeated:
                base = self._msg(field.type)
                if base is not None and base.name not in self.aliases:
                    if base.oneofs:
                        raise OmsJsonShapeError(
                            f'{msg.name}: extension base {base.name} carries '
                            'a oneof, which types_gen does not inline -- '
                            'unsupported on the OMS wire path')
                    for bf in base.fields:
                        name = bf.name
                        if name in own_names:
                            name = base.name.lower() + '_' + name
                        if bf.name == 'base' and not bf.is_repeated:
                            deeper = self._msg(bf.type)
                            if deeper is None:
                                raise OmsJsonShapeError(
                                    f'{base.name}.base: unresolved type '
                                    f'{bf.type}')
                            yield ('flatten', name, deeper.name)
                        else:
                            yield ('field', bf, name, base.name)
                    continue
            yield ('field', field, field.name, msg.name)

    def _field_kind(self, field: ProtoField) -> str:
        """'scalar' | 'enum' | 'message' for a package-local field."""
        if field.type in _SCALARS:
            return 'scalar'
        short = self._short(field.type)
        if short in self.enums:
            return 'enum'
        if short in self.aliases:
            raise OmsJsonShapeError(
                f'field type {short} is a scalar-wrapper alias '
                '(types_gen collapses it); its OMS wire shape is not '
                'defined -- extend the emitter before putting it on a '
                'wire path')
        if short in self.msgs:
            return 'message'
        raise OmsJsonShapeError(f'unresolved field type {field.type}')

    # -- emission -----------------------------------------------------------

    def write(self, path: Path) -> None:
        with path.open('w', encoding='utf-8', newline='\n') as f:
            self._write(f)

    def _write(self, f) -> None:
        ns = self.naming.cpp_namespace_for_package(self.pf.package)
        f.write('// Auto-generated UCI 2.5 OMS-JSON PCL codec plugin.\n')
        f.write('// Only the schema-shaped UCI seam contract is supported.\n\n')
        f.write(f'#include "{self.prefix}_types.hpp"\n')
        f.write(f'#include "{self.prefix}_cabi_marshal.hpp"\n')
        f.write('#include <pcl/pcl_alloc.h>\n#include <pcl/pcl_codec.h>\n')
        f.write('#include <nlohmann/json.hpp>\n')
        f.write('#include <cmath>\n#include <cstdint>\n#include <cstring>\n#include <limits>\n#include <string>\n\n')
        f.write('namespace {\nusing json = nlohmann::json;\n')
        f.write(f'namespace uci = {ns};\n')
        f.write('constexpr const char* kContentType = "application/oms-json";\n')
        f.write('bool uuid(const std::string& s) { if (s.size() != 36) return false; '
                'for (size_t i=0;i<s.size();++i) { if (i==8||i==13||i==18||i==23) { if(s[i]!=\'-\') return false; } '
                'else if (!((s[i]>=\'0\'&&s[i]<=\'9\')||(s[i]>=\'a\'&&s[i]<=\'f\')||(s[i]>=\'A\'&&s[i]<=\'F\'))) return false; } return true; }\n')
        f.write('json number(double v) { if (std::isnan(v)) return "NaN"; if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity"; return v; }\n')
        f.write('double number_in(const json& v) { if (v.is_number()) return v.get<double>(); '
                'auto s=v.get<std::string>(); if(s=="NaN") return std::numeric_limits<double>::quiet_NaN(); '
                'if(s=="Infinity") return std::numeric_limits<double>::infinity(); if(s=="-Infinity") return -std::numeric_limits<double>::infinity(); throw json::type_error::create(302,"invalid OMS number",&v); }\n\n')
        self._write_enum_helpers(f)
        emittable = self._emittable_messages()
        for msg in emittable:
            f.write(f'json encode_{msg.name}(const uci::{msg.name}& m);\n')
            f.write(f'uci::{msg.name} decode_{msg.name}(const json& j);\n')
        f.write('\n')
        for msg in emittable:
            self._encode(f, msg)
            self._decode(f, msg)
        self._write_wrapper_structs(f)
        f.write('pcl_status_t assign(const std::string& s, pcl_msg_t* out) {\n'
                '  if (!out || s.size()>std::numeric_limits<uint32_t>::max()) return PCL_ERR_INVALID;\n'
                '  void* p=s.empty()?nullptr:pcl_alloc(s.size()); if (!s.empty() && !p) return PCL_ERR_NOMEM;\n'
                '  if(p) std::memcpy(p,s.data(),s.size()); out->data=p; out->size=static_cast<uint32_t>(s.size()); out->type_name=kContentType; return PCL_OK; }\n')
        self._write_encode_dispatch(f)
        self._write_decode_dispatch(f)
        f.write('void free_msg(void*, pcl_msg_t* m) { if(!m)return; pcl_free(const_cast<void*>(m->data)); m->data=nullptr;m->size=0;m->type_name=nullptr; }\n'
                'pcl_codec_t codec={PCL_CODEC_ABI_VERSION,kContentType,encode,decode,free_msg,nullptr};\n} // namespace\n\n'
                'extern "C" __attribute__((visibility("default"))) const pcl_codec_t* pcl_codec_plugin_entry(const char*) { return &codec; }\n')

    def _write_enum_helpers(self, f) -> None:
        if not self.pf.enums:
            return
        for enum in self.pf.enums:
            cases = []
            parses = []
            for v in enum.values:
                suf = enum.suffix_of(v.name)
                if (suf or v.name).upper().endswith('UNSPECIFIED'):
                    continue  # never on the UCI wire
                cpp = screaming_to_pascal(suf) if suf else v.name
                lit = self.wire.enum_literal(enum.name, v.name, suf)
                cases.append(f'case uci::{enum.name}::{cpp}: return "{lit}";')
                parses.append(f'if (s=="{lit}") return uci::{enum.name}::{cpp};')
            f.write(f'const char* wire_{enum.name}(uci::{enum.name} v) {{ '
                    f'switch (v) {{ {" ".join(cases)} '
                    'default: throw json::type_error::create(302,"enum value not on the UCI wire",nullptr); } }\n')
            f.write(f'uci::{enum.name} parse_{enum.name}(const std::string& s) {{ '
                    f'{" ".join(parses)} '
                    'throw json::type_error::create(302,"unknown UCI enum literal",nullptr); }\n')
        f.write('\n')

    # -- per-message encode/decode -------------------------------------------

    def _encode(self, f, msg: ProtoMessage) -> None:
        # UCI's IdentifierType is represented as a string in an enclosing
        # UUID element, not as an extra JSON object level.
        if msg.name == 'Uuid' and len(msg.fields) == 1 \
                and msg.fields[0].name == 'uuid' and not msg.oneofs:
            f.write('json encode_Uuid(const uci::Uuid& m) { if (!uuid(m.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return m.uuid; }\n\n')
            return
        f.write(f'json encode_{msg.name}(const uci::{msg.name}& m) {{ json o=json::object();\n')
        for item in self._walk(msg):
            if item[0] == 'flatten':
                _, member, base_type = item
                f.write(f'  o.update(encode_{base_type}(m.{member}));\n')
                continue
            _, fld, member, owner = item
            key = self.wire.field_key(owner, fld.name)
            if key is None:
                raise OmsJsonShapeError(
                    f'{owner}.{fld.name}: no wire element name -- repeated '
                    'xs:choice carriers are not yet supported on the OMS '
                    'wire path')
            access = f'm.{member}'
            if fld.is_repeated:
                if self.wire.sidecar is not None:
                    # Sidecar path: an empty array means "zero occurrences";
                    # the XSD wire form for that is absence (the la-cal-harness
                    # generator omits min_occurs=0 members the same way).  The
                    # heuristic/seam path keeps its verified always-emit form.
                    f.write(f'  if (!{access}.empty()) o["{key}"] = {self._enc_expr(fld, access)};\n')
                else:
                    f.write(f'  o["{key}"] = {self._enc_expr(fld, access)};\n')
            elif fld.is_optional:
                if fld.type in ('string', 'bytes'):
                    f.write(f'  if (!{access}.empty()) o["{key}"] = {self._enc_expr(fld, access)};\n')
                else:
                    f.write(f'  if ({access}) o["{key}"] = {self._enc_one(fld, "*" + access)};\n')
            else:
                f.write(f'  o["{key}"] = {self._enc_expr(fld, access)};\n')
        for oo in msg.oneofs:
            for fld in oo.fields:
                key = self.wire.field_key(msg.name, fld.name)
                if key is None:
                    raise OmsJsonShapeError(
                        f'{msg.name}.{fld.name}: choice member without a '
                        'wire element name')
                access = f'm.{fld.name}'
                items = self._is_list_wrapper(fld.type)
                if items is not None:
                    inner = self._enc_one(items, 'x')
                    f.write(f'  if ({access}) o["{key}"] = ([&]{{ json a=json::array(); for(const auto& x:(*{access}).items) a.push_back({inner}); return a; }})();\n')
                else:
                    f.write(f'  if ({access}) o["{key}"] = {self._enc_one(fld, "*" + access)};\n')
        f.write('  return o; }\n\n')

    def _enc_expr(self, fld: ProtoField, access: str) -> str:
        if fld.is_repeated:
            return f'([&]{{ json a=json::array(); for(const auto& x:{access}) a.push_back({self._enc_one(fld, "x")}); return a; }})()'
        return self._enc_one(fld, access)

    def _enc_one(self, fld: ProtoField, access: str) -> str:
        if fld.type in ('double', 'float'):
            return f'number({access})'
        if fld.type in _SCALARS:
            if fld.name == 'uuid':
                return f'([&]{{ if(!uuid({access})) throw json::type_error::create(302,"invalid UUID",nullptr); return json({access}); }})()'
            return access
        kind = self._field_kind(fld)
        short = self._short(fld.type)
        if kind == 'enum':
            return f'wire_{short}({access})'
        return f'encode_{short}({access})'

    def _decode(self, f, msg: ProtoMessage) -> None:
        if msg.name == 'Uuid' and len(msg.fields) == 1 \
                and msg.fields[0].name == 'uuid' and not msg.oneofs:
            f.write('uci::Uuid decode_Uuid(const json& j) { uci::Uuid r{}; r.uuid=j.get<std::string>(); if (!uuid(r.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return r; }\n\n')
            return
        f.write(f'uci::{msg.name} decode_{msg.name}(const json& j) {{ uci::{msg.name} r{{}};\n')
        for item in self._walk(msg):
            if item[0] == 'flatten':
                f.write(f'  r.{item[1]} = decode_{item[2]}(j);\n')
                continue
            _, fld, member, owner = item
            key = self.wire.field_key(owner, fld.name)
            if key is None:
                raise OmsJsonShapeError(
                    f'{owner}.{fld.name}: no wire element name -- repeated '
                    'xs:choice carriers are not yet supported on the OMS '
                    'wire path')
            value = f'j.at("{key}")'
            if fld.is_optional and not fld.is_repeated:
                f.write(f'  if (j.contains("{key}")) r.{member} = {self._dec_expr(fld, value, member)};\n')
            elif fld.is_repeated and self.wire.sidecar is not None:
                # Mirror of the omit-empty encode rule: absence == empty.
                f.write(f'  if (j.contains("{key}")) r.{member} = {self._dec_expr(fld, value, member)};\n')
            else:
                f.write(f'  r.{member} = {self._dec_expr(fld, value, member)};\n')
        for oo in msg.oneofs:
            first = True
            for fld in oo.fields:
                key = self.wire.field_key(msg.name, fld.name)
                cond = 'if' if first else 'else if'
                first = False
                items = self._is_list_wrapper(fld.type)
                if items is not None:
                    wrapper = self._short(fld.type)
                    inner = self._dec_one(items, 'x')
                    f.write(f'  {cond} (j.contains("{key}")) {{ uci::{wrapper} w{{}}; for (const auto& x : j.at("{key}")) w.items.push_back({inner}); r.{fld.name} = w; }}\n')
                else:
                    value = 'j.at("' + str(key) + '")'
                    f.write(f'  {cond} (j.contains("{key}")) r.{fld.name} = {self._dec_one(fld, value)};\n')
        f.write('  return r; }\n\n')

    def _dec_expr(self, fld: ProtoField, value: str, member: str) -> str:
        if fld.is_repeated:
            inner = self._dec_one(fld, 'x')
            return f'([&]{{ decltype(r.{member}) a; for(const auto& x:{value}) a.push_back({inner}); return a; }})()'
        return self._dec_one(fld, value)

    def _dec_one(self, fld: ProtoField, value: str) -> str:
        if fld.type in ('double', 'float'):
            return f'static_cast<{("float" if fld.type=="float" else "double")}>(number_in({value}))'
        if fld.type in _SCALARS:
            base = {'string': 'std::string', 'bytes': 'std::string', 'bool': 'bool'}.get(fld.type)
            if base:
                out = f'{value}.get<{base}>()'
            else:
                cpp = {'int32':'int32_t','int64':'int64_t','uint32':'uint32_t','uint64':'uint64_t','sint32':'int32_t','sint64':'int64_t','fixed32':'uint32_t','fixed64':'uint64_t','sfixed32':'int32_t','sfixed64':'int64_t'}[fld.type]
                out = f'{value}.get<{cpp}>()'
            if fld.name == 'uuid':
                return f'([&]{{ auto s={out}; if(!uuid(s)) throw json::type_error::create(302,"invalid UUID",nullptr); return s; }})()'
            return out
        kind = self._field_kind(fld)
        short = self._short(fld.type)
        if kind == 'enum':
            return f'parse_{short}({value}.get<std::string>())'
        return f'decode_{short}({value})'

    # -- wrapper structs and dispatch ------------------------------------------

    def _write_wrapper_structs(self, f) -> None:
        if not self.wrappers:
            return
        # The generated interaction facade passes the C ABI representation
        # of its topic wrappers to the content codec.  OMS validates a UCI
        # global element, not that wrapper, so retain only the active root
        # variant at this boundary.  These deliberately mirror the stable
        # C ABI layout of the request-port wrappers; no port abstraction is
        # involved in the wire conversion.
        for w in self.wrappers:
            members = ' '.join(
                f'uint8_t has_{fname}; {self.prefix}_{tshort}_c {fname};'
                for fname, tshort in w['variants'])
            f.write(f'struct {w["wire_struct"]} {{ {members} }};\n')
        f.write('\n')

    def _write_encode_dispatch(self, f) -> None:
        f.write('pcl_status_t encode(void*, const char* id, const void* value, pcl_msg_t* out) {\n'
                '  if(!id||!value||!out) return PCL_ERR_INVALID; try {\n')
        for element, root in self.roots:
            ids = f'std::strcmp(id,"{element}")==0'
            if root != element:
                ids += f' || std::strcmp(id,"{root}")==0'
            f.write(f'    if({ids}) {{ uci::{root} n; pyramid::cabi::from_c(static_cast<const {self.prefix}_{root}_c*>(value),n); return assign(json({{{{"{element}",encode_{root}(n)}}}}).dump(),out); }}\n')
        for w in self.wrappers:
            flags = [f'w.has_{fname}' for fname, _ in w['variants']]
            if len(flags) == 1:
                guard = f'if(!{flags[0]}) return PCL_ERR_INVALID;'
            elif len(flags) == 2:
                guard = f'if({flags[0]} == {flags[1]}) return PCL_ERR_INVALID;'
            else:
                guard = (f'if(({" + ".join(flags)}) != 1) '
                         'return PCL_ERR_INVALID;')
            body = []
            for i, (fname, tshort) in enumerate(w['variants']):
                element = self._element_for_message(tshort)
                emit = (f'uci::{tshort} n; pyramid::cabi::from_c(&w.{fname},n); '
                        f'return assign(json({{{{"{element}",encode_{tshort}(n)}}}}).dump(),out);')
                if i < len(w['variants']) - 1:
                    body.append(f'if(w.has_{fname}) {{ {emit} }}')
                else:
                    body.append(emit)
            f.write(f'    if(std::strcmp(id,"{w["name"]}")==0) {{ const auto& w=*static_cast<const {w["wire_struct"]}*>(value); {guard} {" ".join(body)} }}\n')
        f.write('  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }\n')

    def _write_decode_dispatch(self, f) -> None:
        f.write('pcl_status_t decode(void*, const char* id, const pcl_msg_t* msg, void* value) {\n'
                '  if(!id||!msg||(!msg->data&&msg->size)||!value) return PCL_ERR_INVALID; try { json d=json::parse(msg->data?std::string(static_cast<const char*>(msg->data),msg->size):std::string());\n')
        for element, root in self.roots:
            ids = f'std::strcmp(id,"{element}")==0'
            if root != element:
                ids += f' || std::strcmp(id,"{root}")==0'
            f.write(f'    if({ids}) {{ auto n=decode_{root}(d.at("{element}")); pyramid::cabi::to_c(n,static_cast<{self.prefix}_{root}_c*>(value)); return PCL_OK; }}\n')
        for w in self.wrappers:
            if len(w['variants']) == 1:
                fname, tshort = w['variants'][0]
                element = self._element_for_message(tshort)
                f.write(f'    if(std::strcmp(id,"{w["name"]}")==0) {{ auto* w=static_cast<{w["wire_struct"]}*>(value); std::memset(w,0,sizeof(*w)); if(!d.contains("{element}")) return PCL_ERR_INVALID; auto n=decode_{tshort}(d.at("{element}")); pyramid::cabi::to_c(n,&w->{fname}); w->has_{fname}=1; return PCL_OK; }}\n')
            else:
                arms = []
                for fname, tshort in w['variants']:
                    element = self._element_for_message(tshort)
                    arms.append(f'if(d.contains("{element}")) {{ auto n=decode_{tshort}(d.at("{element}")); pyramid::cabi::to_c(n,&w->{fname}); w->has_{fname}=1; return PCL_OK; }}')
                f.write(f'    if(std::strcmp(id,"{w["name"]}")==0) {{ auto* w=static_cast<{w["wire_struct"]}*>(value); std::memset(w,0,sizeof(*w)); {" ".join(arms)} return PCL_ERR_INVALID; }}\n')
        f.write('  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }\n')
