#!/usr/bin/env python3
"""Deterministic Ada component skeleton generation."""

from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from binding_contract import (
    ComponentGroup,
    ComponentPort,
    NamingPolicy,
    command_projectability_for_service,
    interaction_for_service,
)
from proto_parser import ProtoFile, ProtoRpc, ProtoService, ProtoTypeIndex
from .naming import (
    _ada_pkg_from_proto_pkg,
    _ada_pkg_segment,
    _ada_req_type,
    _ada_rsp_type,
    _ensure_parent_packages,
    _pkg_name_from_proto,
    _proto_type_to_ada,
    _service_ada_prefix,
)


def _component_segment(name: str) -> str:
    return ".".join(_ada_pkg_segment(part) for part in name.split("."))


def _frame_type(rpc: ProtoRpc) -> str:
    value = _ada_rsp_type(rpc)
    if value.endswith("_Array"):
        return value[:-len("_Array")]
    return value


def _topic_suffix(wire_name: str) -> str:
    return "_".join(
        word[:1].upper() + word[1:]
        for word in wire_name.replace(".", "_").split("_")
        if word
    )


def _endpoint_kind(kind: str) -> str:
    return {
        "publisher": "PCL_ENDPOINT_PUBLISHER",
        "subscriber": "PCL_ENDPOINT_SUBSCRIBER",
        "provided": "PCL_ENDPOINT_PROVIDED",
        "consumed": "PCL_ENDPOINT_CONSUMED",
        "stream_provided": "PCL_ENDPOINT_STREAM_PROVIDED",
        "stream_consumed": "PCL_ENDPOINT_STREAM_CONSUMED",
    }[kind]


class AdaComponentSkeletonGenerator:
    """Generate one component-spanning Ada skeleton package."""

    def __init__(
        self,
        group: ComponentGroup,
        proto_files: Iterable[ProtoFile],
        index: ProtoTypeIndex,
        naming_policy: NamingPolicy,
    ):
        del naming_policy
        self.group = group
        self.index = index
        self._files: Dict[str, ProtoFile] = {
            proto.package: proto for proto in proto_files
        }
        self._services: Dict[Tuple[str, str], ProtoService] = {}
        for proto in self._files.values():
            for service in proto.services:
                self._services[(proto.package, service.name)] = service

    @property
    def stem(self) -> str:
        component = self.group.component.replace(".", "-")
        return f"pyramid-skeletons-{self.group.project}-{component}".lower()

    @property
    def package(self) -> str:
        return (
            f"Pyramid.Skeletons.{_ada_pkg_segment(self.group.project)}."
            f"{_component_segment(self.group.component)}"
        )

    def generate(self, output_dir: str) -> List[Path]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        _ensure_parent_packages(output, [self.package])
        spec = output / f"{self.stem}.ads"
        body = output / f"{self.stem}.adb"
        spec.write_text(self.render_spec(), encoding="utf-8", newline="\n")
        body.write_text(self.render_body(), encoding="utf-8", newline="\n")
        print(f"  Generated {spec}")
        print(f"  Generated {body}")
        return [spec, body]

    def generate_scaffold(self, scaffold_dir: str) -> List[Path]:
        """Write the Ada process scaffold without replacing user files."""
        root = (
            Path(scaffold_dir)
            / f"{self.group.project}_{self.group.component.replace('.', '_')}"
            / "ada"
        )
        source = root / "src"
        component = self.group.component.replace(".", "_").lower()
        paths = [
            self._write_if_absent(
                source / f"{self.stem}-impl.ads",
                self._scaffold_spec(),
            ),
            self._write_if_absent(
                source / f"{self.stem}-impl.adb",
                self._scaffold_body(),
            ),
            self._write_if_absent(
                source / f"{component}_main.adb",
                self._scaffold_main(),
            ),
            self._write_if_absent(
                root / f"{component}.gpr",
                self._scaffold_gpr(component),
            ),
        ]
        return paths

    def slot_names(self) -> Tuple[Tuple[str, str], ...]:
        slots = []
        for port in self._emitted_ports():
            if port.role == "provided" and port.port_kind == "request":
                slots.append((port.port_key, "handler"))
            elif port.role == "consumed" and port.port_kind == "information":
                slots.append((port.port_key, "sink_function"))
            elif port.role == "consumed" and port.port_kind == "request":
                slots.append(
                    (f"{port.port_key}_transitions", "transitions_function")
                )
        return tuple(slots)

    def port_names(self) -> Tuple[str, ...]:
        return tuple(port.port_key for port in self._emitted_ports())

    def render_spec(self) -> str:
        service_packages = self._service_packages()
        type_packages = self._type_packages()
        lines = [
            "--  Auto-generated component skeleton. Regenerated on every run.",
        ]
        for package in service_packages:
            lines.append(f"with {package};")
        for package in type_packages:
            lines.append(f"with {package}; use {package};")
        lines.extend([
            "with Pcl_Bindings;",
            "with Pcl_Process_Runtime;",
            "",
            f"package {self.package} is",
            "   type Handlers is record",
        ])
        slot_lines: List[str] = []
        for port in self._emitted_ports():
            slot_lines.extend(self._handler_slot_spec(port))
        lines.extend(slot_lines or ["      null;"])
        lines.extend([
            "   end record;",
            "",
            "   procedure Bind",
            "     (Container : Pcl_Bindings.Pcl_Container_Access;",
            "      Exec      : Pcl_Bindings.Pcl_Executor_Access;",
            "      Binding   : Handlers);",
            "",
        ])
        lines.extend([
            "   function Deployment_Ports return Pcl_Process_Runtime.Port_Array;",
            "",
            f"end {self.package};",
            "",
        ])
        return "\n".join(lines)

    def render_body(self) -> str:
        service_packages = self._service_packages()
        type_packages = self._type_packages()
        lines = []
        for package in service_packages:
            lines.append(f"with {package}; use {package};")
        for package in type_packages:
            lines.append(f"with {package}; use {package};")
        lines.extend([
            "with System;",
            "",
            f"package body {self.package} is",
            "   Bound : Boolean := False;",
            "   Binding_State : access Handlers := null;",
            "",
        ])
        for port in self._emitted_ports():
            lines.extend(self._trampolines(port))
        lines.extend([
            "   procedure Bind",
            "     (Container : Pcl_Bindings.Pcl_Container_Access;",
            "      Exec      : Pcl_Bindings.Pcl_Executor_Access;",
            "      Binding   : Handlers) is",
            "   begin",
            "      if Bound then",
            '         raise Program_Error with "component skeleton is already bound";',
            "      end if;",
        ])
        for port in self._emitted_ports():
            if port.role == "consumed" and port.port_kind == "information":
                field = self._slot_field(port)
                lines.extend([
                    f"      if Binding.{field} = null then",
                    f'         raise Program_Error with "required handler slot {field} is null";',
                    "      end if;",
                ])
        lines.extend([
            "      Binding_State := new Handlers'(Binding);",
            "      Bound := True;",
        ])
        for port in self._emitted_ports():
            lines.extend(self._bind_lines(port))
        lines.extend([
            "   end Bind;",
            "",
        ])
        lines.extend(self._deployment_body())
        lines.extend([
            f"end {self.package};",
            "",
        ])
        return "\n".join(lines)

    def _emitted_ports(self) -> Tuple[ComponentPort, ...]:
        return tuple(port for port in self.group.ports if port.has_interaction)

    def _service(self, port: ComponentPort) -> ProtoService:
        return self._services[(port.package, port.service_name)]

    def _proto(self, port: ComponentPort) -> ProtoFile:
        return self._files[port.package]

    def _interaction(self, port: ComponentPort):
        return interaction_for_service(
            self.index, self._proto(port), self._service(port)
        )

    def _command_rpcs(self, port: ComponentPort) -> Tuple[ProtoRpc, ...]:
        interaction = self._interaction(port)
        leg = next(item for item in interaction.legs if item.name == "request")
        names = {endpoint.rpc_name for endpoint in leg.side_a}
        return tuple(sorted(
            (rpc for rpc in self._service(port).rpcs if rpc.name in names),
            key=lambda rpc: rpc.name,
        ))

    def _read_rpc(self, port: ComponentPort) -> ProtoRpc:
        return next(
            rpc for rpc in self._service(port).rpcs if rpc.name == "Read"
        )

    def _service_package(self, port: ComponentPort) -> str:
        return _pkg_name_from_proto(self._proto(port))

    def _service_packages(self) -> Tuple[str, ...]:
        return tuple(sorted({
            self._service_package(port) for port in self._emitted_ports()
        }))

    def _type_packages(self) -> Tuple[str, ...]:
        packages = set()
        for port in self._emitted_ports():
            proto = self._proto(port)
            local_package = _ada_pkg_from_proto_pkg(proto.package)
            for rpc in self._service(port).rpcs:
                for type_name in (rpc.request_type, rpc.response_type):
                    if "." in type_name and not type_name.startswith("google."):
                        packages.add(
                            _ada_pkg_from_proto_pkg(type_name.rsplit(".", 1)[0])
                        )
                    else:
                        packages.add(local_package)
        return tuple(sorted(packages))

    @staticmethod
    def _slot_field(port: ComponentPort) -> str:
        return _ada_pkg_segment(port.port_key)

    def _handler_slot_spec(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        field = self._slot_field(port)
        if port.role == "provided" and port.port_kind == "request":
            return [
                f"      {field} :",
                f"        {package}.{prefix}_Interaction_Handlers;",
            ]
        if port.role == "consumed" and port.port_kind == "information":
            return [
                f"      {field} :",
                "        access procedure",
                f"          (Item : {_frame_type(self._read_rpc(port))});",
            ]
        if port.role == "consumed" and port.port_kind == "request":
            return [
                f"      {field}_Transitions :",
                f"        {package}.{prefix}_Transition_Callback := null;",
            ]
        return []

    def _trampolines(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        lines: List[str] = []
        if port.role == "consumed" and port.port_kind == "information":
            if not self._ada_info_topic_available(port):
                return lines
            interaction = self._interaction(port)
            wire = interaction.legs[0].side_b[0].endpoint_name
            suffix = _topic_suffix(wire)
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure {prefix}_Information_Trampoline",
                "     (Self      : Pcl_Bindings.Pcl_Container_Access;",
                "      Msg       : access constant Pcl_Bindings.Pcl_Msg;",
                "      User_Data : System.Address);",
                f"   pragma Convention(C, {prefix}_Information_Trampoline);",
                "",
                f"   procedure {prefix}_Information_Trampoline",
                "     (Self      : Pcl_Bindings.Pcl_Container_Access;",
                "      Msg       : access constant Pcl_Bindings.Pcl_Msg;",
                "      User_Data : System.Address) is",
                "      pragma Unreferenced(Self, User_Data);",
                f"      Item : constant {frame} := "
                f"{package}.Decode_{suffix}(Msg);",
                "   begin",
                f"      Binding_State.{self._slot_field(port)}.all(Item);",
                f"   end {prefix}_Information_Trampoline;",
                "",
            ])
        return lines

    def _bind_lines(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        if port.role == "provided" and port.port_kind == "request":
            return [
                f"      {package}.{prefix}_Provider_Bind",
                f"        (Container, Exec, Binding.{self._slot_field(port)});",
            ]
        if port.role == "consumed" and port.port_kind == "request":
            read = self._read_rpc(port)
            return [
                f"      {package}.{prefix}_Client_Bind(Container, Exec);",
                f"      if Binding.{self._slot_field(port)}_Transitions /= null then",
                f"         {package}.{prefix}_Transitions",
                f"           ({_ada_req_type(read)}'(others => <>),",
                f"            Binding.{self._slot_field(port)}_Transitions);",
                "      end if;",
            ]
        if port.role == "consumed" and port.port_kind == "information":
            if not self._ada_info_topic_available(port):
                return [
                    f"      --  {port.port_key}: fail closed because the current Ada",
                    "      --  facade does not expose a local-wrapper topic decoder.",
                    f'      raise Program_Error with "{port.port_key} has no local-wrapper topic decoder";'
                ]
            wire = self._interaction(port).legs[0].side_b[0].endpoint_name
            suffix = _topic_suffix(wire)
            return [
                f"      {package}.Subscribe_{suffix}",
                f"        (Container, {prefix}_Information_Trampoline'Access);",
            ]
        return [
            f"      --  {port.port_key} publishes through Exec; "
            "no subscriber binding is required."
        ]

    def _ada_info_topic_available(self, port: ComponentPort) -> bool:
        """Match the existing Ada service emitter's data-model topic filter."""
        response_type = self._read_rpc(port).response_type
        return "." in response_type and not response_type.startswith("google.")

    def _deployment_body(self) -> List[str]:
        ports = self._emitted_ports()
        lines = [
            "   function Deployment_Ports return Pcl_Process_Runtime.Port_Array is",
            "   begin",
            "      return Pcl_Process_Runtime.Port_Array'(",
        ]
        for port_index, port in enumerate(ports, start=1):
            interaction = self._interaction(port)
            rpc_endpoints = []
            pubsub_endpoints = []
            for leg in interaction.legs:
                rpc_endpoints.extend(leg.side_a)
                pubsub_endpoints.extend(leg.side_b)
            comma = "," if port_index < len(ports) else ""
            lines.extend([
                f"        {port_index} => Pcl_Process_Runtime.Make_Port",
                f'          ("{port.port_key}",',
                "           Pcl_Process_Runtime.Endpoint_Array'(",
            ])
            for index, endpoint in enumerate(rpc_endpoints, start=1):
                endpoint_comma = "," if index < len(rpc_endpoints) else ""
                lines.append(
                    f"             {index} => "
                    "Pcl_Process_Runtime.Make_Endpoint"
                    f'("{endpoint.endpoint_name}", '
                    f"Pcl_Bindings.{_endpoint_kind(endpoint.kind)})"
                    f"{endpoint_comma}"
                )
            lines.append("           ),")
            lines.append("           Pcl_Process_Runtime.Endpoint_Array'(")
            for index, endpoint in enumerate(pubsub_endpoints, start=1):
                endpoint_comma = "," if index < len(pubsub_endpoints) else ""
                lines.append(
                    f"             {index} => "
                    "Pcl_Process_Runtime.Make_Endpoint"
                    f'("{endpoint.endpoint_name}", '
                    f"Pcl_Bindings.{_endpoint_kind(endpoint.kind)})"
                    f"{endpoint_comma}"
                )
            lines.append(f"           )){comma}")
        lines.extend([
            "      );",
            "   end Deployment_Ports;",
            "",
        ])
        return lines

    def _scaffold_callbacks(self):
        callbacks = []
        for port in self._emitted_ports():
            prefix = _service_ada_prefix(port.service_name)
            if port.role == "provided" and port.port_kind == "request":
                projectability = {
                    item.rpc_name: item
                    for item in command_projectability_for_service(
                        self.index, self._proto(port), self._service(port)
                    )
                }
                for rpc in self._command_rpcs(port):
                    if not (
                        projectability.get(rpc.name)
                        and projectability[rpc.name].projectable
                    ):
                        continue
                    callbacks.append((
                        "function",
                        f"On_{prefix}_{rpc.name}",
                        _ada_req_type(rpc),
                        _ada_rsp_type(rpc),
                        port,
                    ))
            elif port.role == "consumed" and port.port_kind == "information":
                callbacks.append((
                    "procedure",
                    f"On_{prefix}",
                    _frame_type(self._read_rpc(port)),
                    "",
                    port,
                ))
            elif port.role == "consumed" and port.port_kind == "request":
                callbacks.append((
                    "procedure",
                    f"On_{prefix}_Transition",
                    _frame_type(self._read_rpc(port)),
                    "",
                    port,
                ))
        return tuple(callbacks)

    @property
    def _impl_package(self) -> str:
        return f"{self.package}.Impl"

    def _scaffold_spec(self) -> str:
        lines = [
            f"with {self.package}; use {self.package};",
        ]
        for package in self._type_packages():
            lines.append(f"with {package}; use {package};")
        lines.extend([
            "with Interfaces.C;",
            "",
            f"package {self._impl_package} is",
            "",
        ])
        for kind, name, request, response, _port in self._scaffold_callbacks():
            if kind == "function":
                lines.extend([
                    f"   function {name}",
                    f"     (Request : {request}) return {response};",
                    "",
                ])
            else:
                lines.extend([
                    f"   procedure {name} (Item : {request});",
                    "",
                ])
        lines.extend([
            "   procedure On_Tick (Dt_Seconds : Interfaces.C.double);",
            f"end {self._impl_package};",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_body(self) -> str:
        lines = [
            f"package body {self._impl_package} is",
        ]
        for kind, name, request, response, _port in self._scaffold_callbacks():
            if kind == "function":
                lines.extend([
                    f"   function {name}",
                    f"     (Request : {request}) return {response} is",
                    "      pragma Unreferenced(Request);",
                    "   begin",
                    "      --  TODO: implement component business logic.",
                    "      return (others => <>);",
                    f"   end {name};",
                    "",
                ])
            else:
                lines.extend([
                    f"   procedure {name} (Item : {request}) is",
                    "      pragma Unreferenced(Item);",
                    "   begin",
                    "      --  TODO: implement component business logic.",
                    "      null;",
                    f"   end {name};",
                    "",
                ])
        lines.extend([
            "   procedure On_Tick (Dt_Seconds : Interfaces.C.double) is",
            "      pragma Unreferenced(Dt_Seconds);",
            "   begin",
            "      --  TODO: implement periodic component work.",
            "      null;",
            "   end On_Tick;",
            f"end {self._impl_package};",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_main(self) -> str:
        main_name = (
            self.group.component.replace(".", "_").title().replace("_", "_")
            + "_Main"
        )
        logic_name = self.group.component.replace(".", "_")
        lines = [
            "with Ada.Command_Line;",
            "with Interfaces.C;",
            f"with {self._impl_package};",
            f"with {self.package};",
            "with Pcl_Bindings;",
            "with Pcl_Component;",
            "with Pcl_Process_Runtime;",
            "",
            f"procedure {main_name} is",
            "   use type Pcl_Bindings.Pcl_Status;",
            "",
            "   Runtime : Pcl_Process_Runtime.Runtime;",
            f"   Binding : constant {self.package}.Handlers :=",
            "     (",
        ]
        slots = [
            port for port in self._emitted_ports()
            if not (port.role == "provided" and port.port_kind == "information")
        ]
        for port_index, port in enumerate(slots):
            field = self._slot_field(port)
            comma = "," if port_index < len(slots) - 1 else ""
            if port.role == "provided" and port.port_kind == "request":
                prefix = _service_ada_prefix(port.service_name)
                callbacks = [
                    item for item in self._scaffold_callbacks()
                    if item[4] == port and item[0] == "function"
                ]
                lines.append(f"      {field} =>")
                lines.append("        (")
                for callback_index, (_kind, name, _req, _rsp, _p) in enumerate(callbacks):
                    lines.append(
                        f"         On_{name.removeprefix(f'On_{prefix}_')} => "
                        f"{self._impl_package}.{name}'Access,"
                    )
                lines.append(f"         others => <>){comma}")
            elif port.role == "consumed" and port.port_kind == "information":
                callback = next(
                    item for item in self._scaffold_callbacks()
                    if item[4] == port
                )
                lines.append(
                    f"      {field} => {self._impl_package}.{callback[1]}'Access{comma}"
                )
            elif port.role == "consumed" and port.port_kind == "request":
                callback = next(
                    item for item in self._scaffold_callbacks()
                    if item[4] == port
                )
                lines.append(
                    f"      {field}_Transitions => "
                    f"{self._impl_package}.{callback[1]}'Access{comma}"
                )
        lines.extend([
            "     );",
            "",
            "   type Runtime_Container is new Pcl_Component.Component with null record;",
            "   overriding procedure On_Configure",
            "     (Self : in out Runtime_Container);",
            "   overriding procedure On_Tick",
            "     (Self       : in out Runtime_Container;",
            "      Dt_Seconds : Interfaces.C.double);",
            "",
            "   overriding procedure On_Configure",
            "     (Self : in out Runtime_Container) is",
            "   begin",
            f"      {self.package}.Bind",
            "        (Pcl_Component.Handle(Self),",
            "         Pcl_Process_Runtime.Executor(Runtime), Binding);",
            "   end On_Configure;",
            "",
            "   overriding procedure On_Tick",
            "     (Self       : in out Runtime_Container;",
            "      Dt_Seconds : Interfaces.C.double) is",
            "   begin",
            "      pragma Unreferenced(Self);",
            f"      {self._impl_package}.On_Tick(Dt_Seconds);",
            "   end On_Tick;",
            "",
            "   Container : Runtime_Container;",
            "   Status    : Pcl_Bindings.Pcl_Status;",
            "begin",
            "   if Ada.Command_Line.Argument_Count < 2 then",
            "      raise Program_Error with",
            '        "usage: <process> <port-config> <codec-plugin>";',
            "   end if;",
            "   Pcl_Process_Runtime.Create(Runtime);",
            "   Pcl_Process_Runtime.Load_Codec",
            "     (Runtime, Ada.Command_Line.Argument(2));",
            "   Pcl_Process_Runtime.Load_Ports_File",
            "     (Runtime, Ada.Command_Line.Argument(1),",
            f"      {self.package}.Deployment_Ports);",
            f'   Pcl_Component.Create(Container, "{logic_name}");',
            "   Status := Pcl_Process_Runtime.Run",
            "     (Runtime, Pcl_Component.Handle(Container));",
            "   Pcl_Process_Runtime.Destroy(Runtime);",
            "   if Status /= Pcl_Bindings.PCL_OK then",
            "      raise Program_Error with \"component runtime failed\";",
            "   end if;",
            f"end {main_name};",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_gpr(self, component: str) -> str:
        project_name = component.title().replace("_", "_")
        return "\n".join([
            'with "pyramid_sdk_ada.gpr";',
            "",
            f"project {project_name} is",
            '   for Source_Dirs use ("src");',
            f'   for Main use ("{component}_main.adb");',
            '   for Object_Dir use "obj";',
            '   for Exec_Dir use "bin";',
            "   package Compiler is",
            '      for Default_Switches ("Ada") use ("-g");',
            "   end Compiler;",
            f"end {project_name};",
            "",
        ])

    @staticmethod
    def _write_if_absent(path: Path, content: str) -> Path:
        if path.exists():
            print(f"scaffold: kept {path}")
            return path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8", newline="\n")
        print(f"scaffold: created {path}")
        return path
