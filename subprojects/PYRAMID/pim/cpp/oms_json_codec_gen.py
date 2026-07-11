#!/usr/bin/env python3
"""Emitter for the deliberately narrow, schema-shaped UCI OMS JSON codec."""

from pathlib import Path
from typing import List

from proto_parser import ProtoTypeIndex, snake_to_pascal

_SCALARS = {'double', 'float', 'int32', 'int64', 'uint32', 'uint64',
            'sint32', 'sint64', 'fixed32', 'fixed64', 'sfixed32', 'sfixed64',
            'bool', 'string', 'bytes'}
_NUMBERS = _SCALARS - {'bool', 'string', 'bytes'}
_ACRONYMS = {'id': 'ID', 'uuid': 'UUID'}


def _key(name: str) -> str:
    """UCI element key: snake case with the small UCI acronym vocabulary."""
    return ''.join(_ACRONYMS.get(word, word.capitalize())
                   for word in name.split('_'))


class CppOmsJsonCodecGenerator:
    """Generate one PCL plugin for the UCI seam data-model package."""

    def __init__(self, index: ProtoTypeIndex):
        self.index = index
        self.pf = next((p for p in index.files
                        if p.package == 'pyramid.data_model.uci'), None)

    def generate(self, out: Path) -> List[Path]:
        out.mkdir(parents=True, exist_ok=True)
        if self.pf is None:
            return []
        path = out / 'pyramid_data_model_uci_oms_json_codec_plugin.cpp'
        self._write(path)
        return [path]

    def _write(self, path: Path) -> None:
        messages = self.pf.messages
        with path.open('w', encoding='utf-8', newline='\n') as f:
            f.write('// Auto-generated UCI 2.5 OMS-JSON PCL codec plugin.\n')
            f.write('// Only the schema-shaped UCI seam contract is supported.\n\n')
            f.write('#include "pyramid_data_model_uci_types.hpp"\n')
            f.write('#include "pyramid_data_model_uci_cabi_marshal.hpp"\n')
            f.write('#include <pcl/pcl_alloc.h>\n#include <pcl/pcl_codec.h>\n')
            f.write('#include <nlohmann/json.hpp>\n')
            f.write('#include <cmath>\n#include <cstdint>\n#include <cstring>\n#include <limits>\n#include <string>\n\n')
            f.write('namespace {\nusing json = nlohmann::json;\n')
            f.write('namespace uci = pyramid::domain_model::uci;\n')
            f.write('constexpr const char* kContentType = "application/oms-json";\n')
            f.write('bool uuid(const std::string& s) { if (s.size() != 36) return false; '
                    'for (size_t i=0;i<s.size();++i) { if (i==8||i==13||i==18||i==23) { if(s[i]!=\'-\') return false; } '
                    'else if (!((s[i]>=\'0\'&&s[i]<=\'9\')||(s[i]>=\'a\'&&s[i]<=\'f\')||(s[i]>=\'A\'&&s[i]<=\'F\'))) return false; } return true; }\n')
            f.write('json number(double v) { if (std::isnan(v)) return "NaN"; if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity"; return v; }\n')
            f.write('double number_in(const json& v) { if (v.is_number()) return v.get<double>(); '
                    'auto s=v.get<std::string>(); if(s=="NaN") return std::numeric_limits<double>::quiet_NaN(); '
                    'if(s=="Infinity") return std::numeric_limits<double>::infinity(); if(s=="-Infinity") return -std::numeric_limits<double>::infinity(); throw json::type_error::create(302,"invalid OMS number",&v); }\n\n')
            for msg in messages:
                f.write(f'json encode_{msg.name}(const uci::{msg.name}& m);\n')
                f.write(f'uci::{msg.name} decode_{msg.name}(const json& j);\n')
            f.write('\n')
            for msg in messages:
                self._encode(f, msg)
                self._decode(f, msg)
            # The generated interaction facade passes the C ABI representation
            # of its topic wrappers to the content codec.  OMS validates a UCI
            # global element, not that wrapper, so retain only the active root
            # variant at this boundary.  These deliberately mirror the stable
            # C ABI layout of the two UCI seam wrappers; no port abstraction is
            # involved in the wire conversion.
            f.write('struct ActionCommandRequestWire { uint8_t has_action_command; pyramid_data_model_uci_ActionCommand_c action_command; uint8_t has_update; pyramid_data_model_uci_ActionCommandStatus_c update; };\n')
            f.write('struct ActionCommandRequirementWire { uint8_t has_action_command_status; pyramid_data_model_uci_ActionCommandStatus_c action_command_status; };\n\n')
            f.write('pcl_status_t assign(const std::string& s, pcl_msg_t* out) {\n'
                    '  if (!out || s.size()>std::numeric_limits<uint32_t>::max()) return PCL_ERR_INVALID;\n'
                    '  void* p=s.empty()?nullptr:pcl_alloc(s.size()); if (!s.empty() && !p) return PCL_ERR_NOMEM;\n'
                    '  if(p) std::memcpy(p,s.data(),s.size()); out->data=p; out->size=static_cast<uint32_t>(s.size()); out->type_name=kContentType; return PCL_OK; }\n')
            f.write('pcl_status_t encode(void*, const char* id, const void* value, pcl_msg_t* out) {\n'
                    '  if(!id||!value||!out) return PCL_ERR_INVALID; try {\n')
            for root in ('ActionCommand', 'ActionCommandStatus'):
                f.write(f'    if(std::strcmp(id,"{root}")==0) {{ uci::{root} n; pyramid::cabi::from_c(static_cast<const pyramid_data_model_uci_{root}_c*>(value),n); return assign(json({{{{"{root}",encode_{root}(n)}}}}).dump(),out); }}\n')
            f.write('    if(std::strcmp(id,"ActionCommand_Service_Request")==0) { const auto& w=*static_cast<const ActionCommandRequestWire*>(value); if(w.has_action_command == w.has_update) return PCL_ERR_INVALID; if(w.has_action_command) { uci::ActionCommand n; pyramid::cabi::from_c(&w.action_command,n); return assign(json({{"ActionCommand",encode_ActionCommand(n)}}).dump(),out); } uci::ActionCommandStatus n; pyramid::cabi::from_c(&w.update,n); return assign(json({{"ActionCommandStatus",encode_ActionCommandStatus(n)}}).dump(),out); }\n')
            f.write('    if(std::strcmp(id,"ActionCommand_Service_Requirement")==0) { const auto& w=*static_cast<const ActionCommandRequirementWire*>(value); if(!w.has_action_command_status) return PCL_ERR_INVALID; uci::ActionCommandStatus n; pyramid::cabi::from_c(&w.action_command_status,n); return assign(json({{"ActionCommandStatus",encode_ActionCommandStatus(n)}}).dump(),out); }\n')
            f.write('  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }\n')
            f.write('pcl_status_t decode(void*, const char* id, const pcl_msg_t* msg, void* value) {\n'
                    '  if(!id||!msg||(!msg->data&&msg->size)||!value) return PCL_ERR_INVALID; try { json d=json::parse(msg->data?std::string(static_cast<const char*>(msg->data),msg->size):std::string());\n')
            for root in ('ActionCommand', 'ActionCommandStatus'):
                f.write(f'    if(std::strcmp(id,"{root}")==0) {{ auto n=decode_{root}(d.at("{root}")); pyramid::cabi::to_c(n,static_cast<pyramid_data_model_uci_{root}_c*>(value)); return PCL_OK; }}\n')
            f.write('    if(std::strcmp(id,"ActionCommand_Service_Request")==0) { auto* w=static_cast<ActionCommandRequestWire*>(value); std::memset(w,0,sizeof(*w)); if(d.contains("ActionCommand")) { auto n=decode_ActionCommand(d.at("ActionCommand")); pyramid::cabi::to_c(n,&w->action_command); w->has_action_command=1; return PCL_OK; } if(d.contains("ActionCommandStatus")) { auto n=decode_ActionCommandStatus(d.at("ActionCommandStatus")); pyramid::cabi::to_c(n,&w->update); w->has_update=1; return PCL_OK; } return PCL_ERR_INVALID; }\n')
            f.write('    if(std::strcmp(id,"ActionCommand_Service_Requirement")==0) { auto* w=static_cast<ActionCommandRequirementWire*>(value); std::memset(w,0,sizeof(*w)); if(!d.contains("ActionCommandStatus")) return PCL_ERR_INVALID; auto n=decode_ActionCommandStatus(d.at("ActionCommandStatus")); pyramid::cabi::to_c(n,&w->action_command_status); w->has_action_command_status=1; return PCL_OK; }\n')
            f.write('  } catch (...) { return PCL_ERR_INVALID; } return PCL_ERR_NOT_FOUND; }\n')
            f.write('void free_msg(void*, pcl_msg_t* m) { if(!m)return; pcl_free(const_cast<void*>(m->data)); m->data=nullptr;m->size=0;m->type_name=nullptr; }\n'
                    'pcl_codec_t codec={PCL_CODEC_ABI_VERSION,kContentType,encode,decode,free_msg,nullptr};\n} // namespace\n\n'
                    'extern "C" __attribute__((visibility("default"))) const pcl_codec_t* pcl_codec_plugin_entry(const char*) { return &codec; }\n')

    def _encode(self, f, msg):
        # UCI's IdentifierType is represented as a string in an enclosing
        # UUID element, not as an extra JSON object level.
        if msg.name == 'Uuid':
            f.write('json encode_Uuid(const uci::Uuid& m) { if (!uuid(m.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return m.uuid; }\n\n')
            return
        f.write(f'json encode_{msg.name}(const uci::{msg.name}& m) {{ json o=json::object();\n')
        for fld in msg.fields:
            key = _key(fld.name)
            access = f'm.{fld.name}'
            if fld.is_optional:
                if fld.type in ('string', 'bytes'):
                    f.write(f'  if (!{access}.empty()) o["{key}"] = {self._enc_expr(fld, access)};\n')
                else:
                    f.write(f'  if ({access}) o["{key}"] = {self._enc_expr(fld, "*" + access)};\n')
            else:
                f.write(f'  o["{key}"] = {self._enc_expr(fld, access)};\n')
        f.write('  return o; }\n\n')

    def _enc_expr(self, fld, access):
        if fld.is_repeated:
            return f'([&]{{ json a=json::array(); for(const auto& x:{access}) a.push_back({self._enc_one(fld, "x")}); return a; }})()'
        return self._enc_one(fld, access)

    def _enc_one(self, fld, access):
        if fld.type in ('double', 'float'):
            return f'number({access})'
        if fld.type in _SCALARS:
            if fld.name == 'uuid':
                return f'([&]{{ if(!uuid({access})) throw json::type_error::create(302,"invalid UUID",nullptr); return json({access}); }})()'
            return access
        return f'encode_{fld.short_type}({access})'

    def _decode(self, f, msg):
        if msg.name == 'Uuid':
            f.write('uci::Uuid decode_Uuid(const json& j) { uci::Uuid r{}; r.uuid=j.get<std::string>(); if (!uuid(r.uuid)) throw json::type_error::create(302,"invalid UUID",nullptr); return r; }\n\n')
            return
        f.write(f'uci::{msg.name} decode_{msg.name}(const json& j) {{ uci::{msg.name} r{{}};\n')
        for fld in msg.fields:
            key = _key(fld.name)
            target = f'r.{fld.name}'
            value = f'j.at("{key}")'
            if fld.is_optional:
                f.write(f'  if (j.contains("{key}")) {target} = {self._dec_expr(fld, value)};\n')
            else:
                f.write(f'  {target} = {self._dec_expr(fld, value)};\n')
        f.write('  return r; }\n\n')

    def _dec_expr(self, fld, value):
        if fld.is_repeated:
            inner = self._dec_one(fld, 'x')
            return f'([&]{{ decltype(r.{fld.name}) a; for(const auto& x:{value}) a.push_back({inner}); return a; }})()'
        return self._dec_one(fld, value)

    def _dec_one(self, fld, value):
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
        return f'decode_{fld.short_type}({value})'
