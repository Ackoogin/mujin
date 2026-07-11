#!/usr/bin/env python3
"""Ada OMS-JSON encoder emitter.

The C++ plugin is the PCL wire implementation.  These packages expose the
same content type and wire shape to Ada clients without introducing a
GNATCOLL runtime dependency into generated bindings.

Two paths, mirroring the C++ emitter (UCI MMS conversion plan Phase 2):

* **Seam template** -- the hand-shaped body for the exact
  ``pim/uci_seam_example`` contract, byte-frozen (it references the seam's
  message shapes and components packages by name).
* **Generalized emitter** -- for wire_names.json-backed (xsd2proto)
  packages: ``To_Oms_Json`` encoders for every profile root, JSON keys
  emitted in sorted order (matching nlohmann's object ordering on the C++
  side), enums as XSD literals with ``*_Unspecified`` raising, oneofs as
  the active member's element key, codegen-time flattening of retained
  extension-base members, and omit-empty arrays.

Presence envelope (documented deviation from C++): the native Ada record
layer carries ``Has_`` flags only for optional scalars and oneof members
(``ada/types_gen.py``), so the encoder uses the established content-
sentinel conventions elsewhere -- optional strings omit when empty,
optional enums omit at ``*_Unspecified`` (the zero-sentinel added by the
conversion exists exactly for this), and optional message fields omit when
recursively default (``Is_Default_*`` probes).  Values outside that
envelope (e.g. a deliberately all-defaults present optional message)
cannot be expressed from Ada; the C ABI marshal has the same limit.
"""
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import json as _json

from proto_parser import (ProtoField, ProtoFile, ProtoMessage,
                          ProtoTypeIndex, screaming_to_pascal)
from .naming import (_ADA_SCALAR_MAP, _ada_field_name, _ada_name,
                     _ada_array_name_for_repeated, _ada_pkg_segment)


class AdaOmsShapeError(RuntimeError):
    """A package shape this emitter cannot put on the UCI wire."""


class AdaOmsJsonCodecGenerator:
    """Dispatch: seam template for the seam contract; generalized emitter
    for wire_names.json-backed packages; skip (with a printed note)
    otherwise."""

    _SEAM_COMPONENT_PACKAGES = (
        'pyramid.components.uci.mission_autonomy.services.provided',
        'pyramid.components.uci.c2_station.services.consumed',
    )

    def __init__(self, index: ProtoTypeIndex):
        self.index = index
        self.general: List[ProtoFile] = []
        self.pf = next((p for p in index.files if p.package == 'pyramid.data_model.uci'), None)
        if self.pf is not None:
            packages = {p.package for p in index.files}
            is_seam = (
                self.pf.find_message('ActionCommand') is not None
                and self.pf.find_message('ActionCommandStatus') is not None
                and all(pkg in packages
                        for pkg in self._SEAM_COMPONENT_PACKAGES))
            if not is_seam:
                self.pf = None
        for pf in index.files:
            if pf is self.pf or not pf.messages or pf.services:
                continue
            if _load_sidecar(pf) is not None:
                self.general.append(pf)
        if self.pf is None and not self.general:
            print('  oms_json (Ada): no seam contract and no '
                  'wire_names.json-backed package -- nothing to emit')

    def generate(self, out: Path) -> List[Path]:
        out.mkdir(parents=True, exist_ok=True)
        paths: List[Path] = []
        for pf in self.general:
            paths.extend(_AdaPackageEmitter(self.index, pf).write(out))
        if self.pf is None:
            return paths
        spec = out / 'pyramid-data_model-uci-oms_json_codec.ads'
        body = out / 'pyramid-data_model-uci-oms_json_codec.adb'
        spec.write_text('''with Pyramid.Data_Model.Uci.Types;
with Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types;
with Pyramid.Components.Uci.C2_Station.Services.Consumed.Types;
package Pyramid.Data_Model.Uci.Oms_Json_Codec is
   Content_Type : constant String := "application/oms-json";
   function To_Oms_Json (Msg : Pyramid.Data_Model.Uci.Types.Action_Command)
      return String;
   function To_Oms_Json (Msg : Pyramid.Data_Model.Uci.Types.Action_Command_Status)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Request)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Requirement)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Request)
      return String;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Requirement)
      return String;
end Pyramid.Data_Model.Uci.Oms_Json_Codec;
'''.replace('\\"', '""'), encoding='utf-8')
        # Keep this emitter dependency-free: generated Ada bindings do not
        # require GNATCOLL.JSON merely to use the same OMS wire shape.
        body.write_text('''with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types;
with Pyramid.Components.Uci.C2_Station.Services.Consumed.Types;
package body Pyramid.Data_Model.Uci.Oms_Json_Codec is
   package U renames Pyramid.Data_Model.Uci.Types;
   use type U.Government_Identifier_Array_Acc;
   use type U.Command_Array_Acc;
   function Q (S : Unbounded_String) return String is
   begin
      return "\\\"" & To_String (S) & "\\\"";
   end Q;
   function Security (S : U.Security_Information) return String is
   begin
      if S.Owner_Producer = null then
         raise Constraint_Error with "OwnerProducer is required";
      end if;
      return "{\\\"Classification\\\":" & Q (S.Classification) &
        ",\\\"OwnerProducer\\\":[{\\\"GovernmentIdentifier\\\":" &
        Q (S.Owner_Producer (S.Owner_Producer'First).Government_Identifier) & "}]}";
   end Security;
   function Header (H : U.Message_Header) return String is
      Prefix : Unbounded_String := Null_Unbounded_String;
      Service : Unbounded_String := Null_Unbounded_String;
   begin
      if Length (H.Val_Mission_Id.Val_Uuid.Uuid) /= 0 then
         Prefix := To_Unbounded_String ("\\\"MissionID\\\":{\\\"UUID\\\":" &
           Q (H.Val_Mission_Id.Val_Uuid.Uuid) & "},");
      end if;
      if Length (H.Val_Service_Id.Val_Uuid.Uuid) /= 0 then
         Service := To_Unbounded_String (",\\\"ServiceID\\\":{\\\"UUID\\\":" &
           Q (H.Val_Service_Id.Val_Uuid.Uuid) & "}");
      end if;
      return "{" & To_String (Prefix) & "\\\"SystemID\\\":{\\\"UUID\\\":" &
        Q (H.Val_System_Id.Val_Uuid.Uuid) & "}" & To_String (Service) & ",\\\"Timestamp\\\":" &
        Q (H.Timestamp) & ",\\\"SchemaVersion\\\":" &
        Q (H.Schema_Version) & ",\\\"Mode\\\":" & Q (H.Mode) & "}";
   end Header;
   function To_Oms_Json (Msg : U.Action_Command) return String is
      Result : Unbounded_String := To_Unbounded_String ("{\\\"ActionCommand\\\":{\\\"SecurityInformation\\\":" &
        Security (Msg.Val_Security_Information) & ",\\\"MessageHeader\\\":" & Header (Msg.Val_Message_Header) & ",\\\"MessageData\\\":{\\\"Command\\\":[");
   begin
      if Msg.Message_Data.Command = null then raise Constraint_Error with "Command is required"; end if;
      for I in Msg.Message_Data.Command'Range loop
         declare C : constant U.Capability := Msg.Message_Data.Command (I).Val_Capability; begin
            if I /= Msg.Message_Data.Command'First then Append (Result, ","); end if;
            Append (Result, "{\\\"Capability\\\":{\\\"CommandID\\\":{\\\"UUID\\\":" & Q (C.Val_Command_Id.Val_Uuid.Uuid) & "},\\\"CommandState\\\":" & Q (C.Command_State) & ",\\\"CapabilityID\\\":{\\\"UUID\\\":" & Q (C.Val_Capability_Id.Val_Uuid.Uuid) & "},\\\"Ranking\\\":{\\\"Rank\\\":{\\\"Priority\\\":" & Integer'Image (C.Val_Ranking.Val_Rank.Priority) & "}},\\\"ActionID\\\":{\\\"UUID\\\":" & Q (C.Val_Action_Id.Val_Uuid.Uuid) & "}}}");
         end;
      end loop;
      return To_String (Result) & "]}}}";
   end To_Oms_Json;
   function To_Oms_Json (Msg : U.Action_Command_Status) return String is
   begin
      return "{\\\"ActionCommandStatus\\\":{\\\"SecurityInformation\\\":" & Security (Msg.Val_Security_Information) &
        ",\\\"MessageHeader\\\":" & Header (Msg.Val_Message_Header) & ",\\\"MessageData\\\":{\\\"CommandID\\\":{\\\"UUID\\\":" &
        Q (Msg.Message_Data.Val_Command_Id.Val_Uuid.Uuid) & "},\\\"CommandProcessingState\\\":" &
        Q (Msg.Message_Data.Command_Processing_State) & "}}}";
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Request)
      return String is
   begin
      if Msg.Has_Action_Command = Msg.Has_Update then
         raise Constraint_Error with "exactly one UCI request variant is required";
      elsif Msg.Has_Action_Command then
         return To_Oms_Json (Msg.Action_Command);
      end if;
      return To_Oms_Json (Msg.Update);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.Mission_Autonomy.Services.Provided.Types.Action_Command_Service_Requirement)
      return String is
   begin
      if not Msg.Has_Action_Command_Status then
         raise Constraint_Error with "ActionCommandStatus is required";
      end if;
      return To_Oms_Json (Msg.Action_Command_Status);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Request)
      return String is
   begin
      if Msg.Has_Action_Command = Msg.Has_Update then
         raise Constraint_Error with "exactly one UCI request variant is required";
      elsif Msg.Has_Action_Command then
         return To_Oms_Json (Msg.Action_Command);
      end if;
      return To_Oms_Json (Msg.Update);
   end To_Oms_Json;
   function To_Oms_Json
      (Msg : Pyramid.Components.Uci.C2_Station.Services.Consumed.Types.Action_Command_Service_Requirement)
      return String is
   begin
      if not Msg.Has_Action_Command_Status then
         raise Constraint_Error with "ActionCommandStatus is required";
      end if;
      return To_Oms_Json (Msg.Action_Command_Status);
   end To_Oms_Json;
end Pyramid.Data_Model.Uci.Oms_Json_Codec;
'''.replace(chr(92) + '"', '""'), encoding='utf-8')
        return paths + [spec, body]


# -- generalized emitter (wire_names.json-backed packages) --------------------


def _load_sidecar(pf: ProtoFile) -> Optional[dict]:
    for parent in list(pf.path.parents)[:4]:
        candidate = parent / 'wire_names.json'
        if candidate.is_file():
            doc = _json.loads(candidate.read_text(encoding='utf-8'))
            if doc.get('package') == pf.package:
                return doc
        if candidate.is_file():
            break
    return None


class _AdaPackageEmitter:
    """Emit To_Oms_Json encoders for one sidecar-backed data-model package."""

    def __init__(self, index: ProtoTypeIndex, pf: ProtoFile):
        self.index = index
        self.pf = pf
        self.sidecar = _load_sidecar(pf)
        self.msgs: Dict[str, ProtoMessage] = {m.name: m for m in pf.messages}
        self.enums = {e.name: e for e in pf.enums}
        self.roots: List[Tuple[str, str]] = list(
            self.sidecar.get('roots', {}).items())
        self.types_pkg = '.'.join(
            _ada_pkg_segment(seg) for seg in pf.package.split('.')) + '.Types'
        self.codec_pkg = '.'.join(
            _ada_pkg_segment(seg) for seg in pf.package.split('.')
        ) + '.Oms_Json_Codec'
        self.is_default_needed: List[str] = []   # emission order
        self.int_img_types: set = set()

    # -- shape queries (mirroring cpp/oms_json_codec_gen semantics) ----------

    def _short(self, t: str) -> str:
        return t.split('.')[-1]

    def _field_key(self, msg_name: str, field_name: str) -> Optional[str]:
        entry = (self.sidecar.get('messages', {}).get(msg_name, {})
                 .get('fields', {}).get(field_name))
        return entry.get('element') if entry else None

    def _field_required(self, msg_name: str, field_name: str) -> bool:
        entry = (self.sidecar.get('messages', {}).get(msg_name, {})
                 .get('fields', {}).get(field_name))
        return bool(entry and entry.get('required'))

    def _is_synthesized(self, msg_name: str) -> bool:
        return bool(self.sidecar.get('messages', {}).get(msg_name, {})
                    .get('synthesized'))

    def _enum_literal(self, enum_name: str, value_name: str,
                      suffix: Optional[str]) -> str:
        lit = self.sidecar.get('enums', {}).get(enum_name, {}).get(value_name)
        if lit is not None:
            return lit
        return suffix if suffix else value_name

    def _is_list_wrapper(self, type_name: str) -> Optional[ProtoField]:
        short = self._short(type_name)
        msg = self.msgs.get(short)
        if (msg is not None and self._is_synthesized(short)
                and not msg.oneofs and len(msg.fields) == 1
                and msg.fields[0].name == 'items'
                and msg.fields[0].is_repeated):
            return msg.fields[0]
        return None

    def _kind(self, fld: ProtoField) -> str:
        if fld.type in _ADA_SCALAR_MAP:
            return 'scalar'
        short = self._short(fld.type)
        if short in self.enums:
            return 'enum'
        if short in self.msgs:
            return 'message'
        raise AdaOmsShapeError(f'unresolved field type {fld.type}')

    def _ada_component(self, fname: str, fld: ProtoField) -> str:
        """The generated record component name (types_gen naming rules)."""
        if fld.is_repeated:
            ada_type = _ada_array_name_for_repeated(fld.type, fname) + '_Acc'
        elif fld.type in _ADA_SCALAR_MAP:
            ada_type = _ADA_SCALAR_MAP[fld.type]
        else:
            ada_type = _ada_name(self._short(fld.type))
        return _ada_field_name(fname, ada_type)

    def _enum_prefix(self, enum_name: str) -> str:
        return _ada_name(enum_name).split('_')[-1] + '_'

    def _enum_ada_value(self, enum, value) -> str:
        suf = enum.suffix_of(value.name)
        return self._enum_prefix(enum.name) + (
            screaming_to_pascal(suf) if suf else value.name)

    def _walk(self, msg: ProtoMessage):
        """Mirror ada/types_gen._inline_base_fields: the record's actual
        components.  Yields ('field', fld, component_expr_suffix, owner) and
        ('flatten', component, base_type_short)."""
        own_names = {f.name for f in msg.fields if f.name != 'base'}
        for fld in msg.fields:
            if fld.name == 'base' and not fld.is_repeated:
                base = self.msgs.get(self._short(fld.type))
                if base is not None:
                    if base.oneofs:
                        raise AdaOmsShapeError(
                            f'{msg.name}: extension base {base.name} carries '
                            'a oneof (not inlined by types_gen)')
                    for bf in base.fields:
                        name = bf.name
                        if name in own_names:
                            name = base.name.lower() + '_' + name
                        if bf.name == 'base' and not bf.is_repeated:
                            yield ('flatten', self._ada_component(name, bf),
                                   self._short(bf.type))
                        else:
                            yield ('field', bf, self._ada_component(name, bf),
                                   base.name)
                    continue
            yield ('field', fld, self._ada_component(fld.name, fld),
                   msg.name)

    # -- member rendering ------------------------------------------------------

    def _scalar_value(self, fld: ProtoField, access: str) -> str:
        ada = _ADA_SCALAR_MAP[fld.type]
        if ada == 'Unbounded_String':
            return f'Esc (To_String ({access}))'
        if ada == 'Boolean':
            return f'(if {access} then "true" else "false")'
        # Natural is a subtype of Integer: one Img overload covers both.
        self.int_img_types.add('Integer' if ada == 'Natural' else ada)
        return f'Img ({access})'

    def _value_expr(self, fld: ProtoField, access: str) -> str:
        kind = self._kind(fld)
        if kind == 'scalar':
            return self._scalar_value(fld, access)
        short = self._short(fld.type)
        if kind == 'enum':
            return f'Wire_{_ada_name(short)} ({access})'
        return f'Encode_{_ada_name(short)} ({access})'

    def _array_render(self, fld: ProtoField, access: str) -> List[str]:
        """Statements rendering '[' .. ']' of a non-null array access."""
        inner = self._value_expr(fld, f'{access} (I)')
        return [
            'declare',
            '   A : Unbounded_String;',
            'begin',
            f'   for I in {access}\'Range loop',
            '      if Length (A) /= 0 then Append (A, ","); end if;',
            f'      Append (A, {inner});',
            '   end loop;',
            '   V := To_Unbounded_String ("[" & To_String (A) & "]");',
            'end;',
        ]

    def _member_stmts(self, key: str, fld: ProtoField, comp: str,
                      owner: str) -> List[str]:
        """Guarded statements adding one member to buffer B of record M."""
        access = f'M.{comp}'
        kind = self._kind(fld)
        if fld.is_repeated:
            body = ['declare', '   V : Unbounded_String;', 'begin']
            body += ['   ' + s for s in self._array_render(fld, access)]
            body += [f'   Add (B, "{key}", To_String (V));', 'end;']
            if self._field_required(owner, fld.name):
                # XSD minOccurs >= 1: an empty list has no valid wire form.
                return ([f'if {access} = null or else {access}\'Length = 0 then',
                         f'   raise Constraint_Error with '
                         f'"required repeated element {key} is empty";',
                         'end if;'] + body)
            return ([f'if {access} /= null and then {access}\'Length > 0 then']
                    + ['   ' + s for s in body] + ['end if;'])
        if kind == 'scalar':
            ada = _ADA_SCALAR_MAP[fld.type]
            stmt = f'Add (B, "{key}", {self._scalar_value(fld, access)});'
            if fld.is_optional and ada == 'Unbounded_String':
                return [f'if Length ({access}) /= 0 then', '   ' + stmt,
                        'end if;']
            if fld.is_optional:
                return [f'if M.Has_{comp} then', '   ' + stmt, 'end if;']
            return [stmt]
        if kind == 'enum':
            short = self._short(fld.type)
            enum = self.enums[short]
            unspecified = self._enum_ada_value(enum, enum.values[0])
            stmt = f'Add (B, "{key}", Wire_{_ada_name(short)} ({access}));'
            if fld.is_optional:
                return [f'if {access} /= {unspecified} then', '   ' + stmt,
                        'end if;']
            return [stmt]
        # message
        short = self._short(fld.type)
        stmt = f'Add (B, "{key}", Encode_{_ada_name(short)} ({access}));'
        if fld.is_optional:
            self._need_is_default(short)
            return [f'if not Is_Default_{_ada_name(short)} ({access}) then',
                    '   ' + stmt, 'end if;']
        return [stmt]

    def _oneof_member_stmts(self, msg: ProtoMessage,
                            fld: ProtoField) -> List[str]:
        key = self._field_key(msg.name, fld.name)
        if key is None:
            raise AdaOmsShapeError(
                f'{msg.name}.{fld.name}: choice member without a wire '
                'element name')
        comp = self._ada_component(fld.name, fld)
        items = self._is_list_wrapper(fld.type)
        if items is not None:
            inner_access = f'M.{comp}.Items'
            body = ['declare', '   V : Unbounded_String;', 'begin']
            body += ['   ' + s for s in self._array_render(items, inner_access)]
            body += [f'   Add (B, "{key}", To_String (V));', 'end;']
            return ([f'if M.Has_{comp} and then {inner_access} /= null '
                     f'and then {inner_access}\'Length > 0 then']
                    + ['   ' + s for s in body] + ['end if;'])
        stmt = f'Add (B, "{key}", {self._value_expr(fld, f"M.{comp}")});'
        return [f'if M.Has_{comp} then', '   ' + stmt, 'end if;']

    def _need_is_default(self, short: str) -> None:
        if short not in self.is_default_needed:
            self.is_default_needed.append(short)
            # transitively: message members of the probed type
            msg = self.msgs.get(short)
            if msg is None:
                return
            for item in self._walk(msg):
                if item[0] == 'flatten':
                    self._need_is_default(item[2])
                elif (not item[1].is_repeated
                      and item[1].type not in _ADA_SCALAR_MAP
                      and self._short(item[1].type) in self.msgs):
                    self._need_is_default(self._short(item[1].type))
            for oo in msg.oneofs:
                pass  # oneof presence uses Has_ flags; no probe needed

    def _message_members(self, msg: ProtoMessage) -> List[Tuple[str, List[str]]]:
        """[(sorted key, statements)] for one message's encode body."""
        members: List[Tuple[str, List[str]]] = []
        for item in self._walk(msg):
            if item[0] == 'flatten':
                _, comp, base_short = item
                base = self.msgs[base_short]
                for key, stmts in self._message_members_with_prefix(
                        base, f'M.{comp}.'):
                    members.append((key, stmts))
                continue
            _, fld, comp, owner = item
            key = self._field_key(owner, fld.name)
            if key is None:
                raise AdaOmsShapeError(
                    f'{owner}.{fld.name}: no wire element name -- repeated '
                    'xs:choice carriers are not supported on the OMS wire '
                    'path')
            members.append((key, self._member_stmts(key, fld, comp, owner)))
        for oo in msg.oneofs:
            if len(oo.fields) > 1:
                # An xs:choice has exactly one active arm; the record's
                # independent Has_ flags cannot enforce that, so the encoder
                # must (mirrors the C++ emitter's multi-arm rejection).
                arms = ' + '.join(
                    f'Boolean\'Pos (M.Has_{self._ada_component(f.name, f)})'
                    for f in oo.fields)
                members.append(('', [
                    f'if {arms} > 1 then',
                    f'   raise Constraint_Error with '
                    f'"multiple active choice arms in {msg.name}";',
                    'end if;']))
            for fld in oo.fields:
                key = self._field_key(msg.name, fld.name)
                members.append((key or '', self._oneof_member_stmts(msg, fld)))
        return sorted(members, key=lambda kv: kv[0])

    def _message_members_with_prefix(self, msg: ProtoMessage, prefix: str):
        """Members of a retained-base message, re-rooted at ``prefix``."""
        out = []
        for key, stmts in self._message_members(msg):
            out.append((key, [s.replace('M.', prefix) for s in stmts]))
        return out

    # -- emission ----------------------------------------------------------------

    def _emittable(self) -> List[ProtoMessage]:
        return [m for m in self.pf.messages
                if not self._is_synthesized(m.name)]

    def write(self, out: Path) -> List[Path]:
        base = self.codec_pkg.lower().replace('.', '-')
        spec = out / (base + '.ads')
        body = out / (base + '.adb')
        self._write_spec(spec)
        self._write_body(body)
        print(f'  Generated {self.codec_pkg} (generalized Ada OMS-JSON '
              'encoder)')
        return [spec, body]

    def _root_record(self, root_msg: str) -> str:
        return f'{self.types_pkg}.{_ada_name(root_msg)}'

    def _write_spec(self, path: Path) -> None:
        with path.open('w', encoding='utf-8', newline='\n') as f:
            f.write('--  Auto-generated Ada OMS-JSON encoder specification\n')
            f.write(f'--  Generated from: {self.pf.path.name} '
                    '(wire_names.json-backed)\n')
            f.write('--  Encode-only: the C++ plugin is the PCL wire '
                    'implementation; this\n')
            f.write('--  package gives Ada clients the same wire shape '
                    '(sorted keys, XSD\n')
            f.write('--  enum literals, extension flattening) without '
                    'GNATCOLL.\n\n')
            f.write(f'with {self.types_pkg};\n')
            f.write(f'\npackage {self.codec_pkg} is\n\n')
            f.write('   Content_Type : constant String := '
                    '"application/oms-json";\n\n')
            for element, root_msg in self.roots:
                f.write(f'   --  Root element "{element}"\n')
                f.write(f'   function To_Oms_Json '
                        f'(Msg : {self._root_record(root_msg)}) '
                        'return String;\n\n')
            f.write(f'end {self.codec_pkg};\n')

    def _write_body(self, path: Path) -> None:
        emittable = self._emittable()
        # Pre-compute all member statement lists first so Img/Is_Default
        # requirements are known before the support section is written.
        rendered: Dict[str, List[Tuple[str, List[str]]]] = {}
        for msg in emittable:
            rendered[msg.name] = self._message_members(msg)
        lines: List[str] = []
        w = lines.append
        w('--  Auto-generated Ada OMS-JSON encoder body')
        w('with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;')
        w(f'with {self.types_pkg};  use {self.types_pkg};')
        w('')
        w(f'package body {self.codec_pkg} is')
        w('')
        # -- support ------------------------------------------------------
        w('   procedure Add (B : in out Unbounded_String;')
        w('                  Key : String; Val : String) is')
        w('   begin')
        w('      if Length (B) /= 0 then Append (B, ","); end if;')
        w('      Append (B, \'"\' & Key & \'"\' & \':\' & Val);')
        w('   end Add;')
        w('')
        w('   function Esc (S : String) return String is')
        w('      Hex : constant String := "0123456789abcdef";')
        w('      R : Unbounded_String;')
        w('   begin')
        w('      Append (R, \'"\');')
        w('      for C of S loop')
        w('         if C = \'"\' then Append (R, "\\""");')
        w('         elsif C = \'\\\' then Append (R, "\\\\");')
        w('         elsif Character\'Pos (C) < 32 then')
        w('            Append (R, "\\u00"')
        w('              & Hex (Character\'Pos (C) / 16 + 1)')
        w('              & Hex (Character\'Pos (C) mod 16 + 1));')
        w('         else')
        w('            Append (R, C);')
        w('         end if;')
        w('      end loop;')
        w('      Append (R, \'"\');')
        w('      return To_String (R);')
        w('   end Esc;')
        w('')
        for ada_type in sorted(self.int_img_types):
            if ada_type in ('Long_Float', 'Float'):
                w(f'   function Img (V : {ada_type}) return String is')
                w('   begin')
                w('      if V /= V then return """NaN"""; end if;')
                w(f'      if V > {ada_type}\'Last then '
                  'return """Infinity"""; end if;')
                w(f'      if V < {ada_type}\'First then '
                  'return """-Infinity"""; end if;')
                w('      declare')
                w(f'         S : constant String := {ada_type}\'Image (V);')
                w('      begin')
                w('         if S (S\'First) = \' \' then')
                w('            return S (S\'First + 1 .. S\'Last);')
                w('         end if;')
                w('         return S;')
                w('      end;')
                w('   end Img;')
            else:
                w(f'   function Img (V : {ada_type}) return String is')
                w(f'      S : constant String := {ada_type}\'Image (V);')
                w('   begin')
                w('      if S (S\'First) = \' \' then')
                w('         return S (S\'First + 1 .. S\'Last);')
                w('      end if;')
                w('      return S;')
                w('   end Img;')
            w('')
        # -- enum wire helpers ---------------------------------------------
        for enum in self.pf.enums:
            ada_enum = _ada_name(enum.name)
            w(f'   function Wire_{ada_enum} (V : {ada_enum}) '
              'return String is')
            w('   begin')
            w('      case V is')
            unspecified = self._enum_ada_value(enum, enum.values[0])
            for v in enum.values:
                ada_val = self._enum_ada_value(enum, v)
                suf = enum.suffix_of(v.name)
                if (suf or v.name).upper().endswith('UNSPECIFIED'):
                    w(f'         when {ada_val} =>')
                    w('            raise Constraint_Error with '
                      f'"{enum.name} value not on the UCI wire";')
                    continue
                lit = self._enum_literal(enum.name, v.name, suf)
                w(f'         when {ada_val} => return """{lit}""";')
            w('      end case;')
            w(f'   end Wire_{ada_enum};')
            w('')
        # -- forward specs ---------------------------------------------------
        for msg in emittable:
            w(f'   function Encode_{_ada_name(msg.name)} '
              f'(M : {_ada_name(msg.name)}) return String;')
        for short in self.is_default_needed:
            w(f'   function Is_Default_{_ada_name(short)} '
              f'(M : {_ada_name(short)}) return Boolean;')
        w('')
        # -- Is_Default probes ----------------------------------------------
        for short in self.is_default_needed:
            self._emit_is_default(w, self.msgs[short])
        # -- encoders ---------------------------------------------------------
        for msg in emittable:
            ada_msg = _ada_name(msg.name)
            w(f'   function Encode_{ada_msg} (M : {ada_msg}) '
              'return String is')
            members = rendered[msg.name]
            if not members:
                w('      pragma Unreferenced (M);')
            w('      B : Unbounded_String;')
            w('   begin')
            for _key, stmts in members:
                for s in stmts:
                    w('      ' + s)
            w('      return "{" & To_String (B) & "}";')
            w(f'   end Encode_{ada_msg};')
            w('')
        # -- roots ------------------------------------------------------------
        for element, root_msg in self.roots:
            w(f'   function To_Oms_Json '
              f'(Msg : {self._root_record(root_msg)}) return String is')
            w('   begin')
            w(f'      return "{{""{element}"":" '
              f'& Encode_{_ada_name(root_msg)} (Msg) & "}}";')
            w('   end To_Oms_Json;')
            w('')
        w(f'end {self.codec_pkg};')
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8',
                        newline='\n')

    def _emit_is_default(self, w, msg: ProtoMessage) -> None:
        ada_msg = _ada_name(msg.name)
        conds: List[str] = []
        for item in self._walk(msg):
            if item[0] == 'flatten':
                _, comp, base_short = item
                conds.append(f'Is_Default_{_ada_name(base_short)} (M.{comp})')
                self._need_is_default(base_short)
                continue
            _, fld, comp, _owner = item
            access = f'M.{comp}'
            if fld.is_repeated:
                conds.append(f'({access} = null or else '
                             f'{access}\'Length = 0)')
            elif fld.type in _ADA_SCALAR_MAP:
                ada = _ADA_SCALAR_MAP[fld.type]
                if ada == 'Unbounded_String':
                    conds.append(f'Length ({access}) = 0')
                elif fld.is_optional:
                    conds.append(f'not M.Has_{comp}')
                elif ada == 'Boolean':
                    conds.append(f'{access} = False')
                else:
                    conds.append(f'{access} = 0'
                                 + ('.0' if ada in ('Long_Float', 'Float')
                                    else ''))
            elif self._short(fld.type) in self.enums:
                enum = self.enums[self._short(fld.type)]
                conds.append(f'{access} = '
                             f'{self._enum_ada_value(enum, enum.values[0])}')
            else:
                conds.append(
                    f'Is_Default_{_ada_name(self._short(fld.type))} '
                    f'({access})')
        for oo in msg.oneofs:
            for fld in oo.fields:
                conds.append(f'not M.Has_{self._ada_component(fld.name, fld)}')
        w(f'   function Is_Default_{ada_msg} (M : {ada_msg}) '
          'return Boolean is')
        if not conds:
            w('      pragma Unreferenced (M);')
        w('   begin')
        if not conds:
            w('      return True;')
        else:
            w('      return ' + ('\n        and then '.join(conds)) + ';')
        w(f'   end Is_Default_{ada_msg};')
        w('')
