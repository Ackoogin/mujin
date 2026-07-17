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

    def hook_names(self) -> Tuple[str, ...]:
        hooks = []
        for port in self._emitted_ports():
            prefix = _service_cpp_style_prefix(port.service_name)
            if port.role == "provided" and port.port_kind == "request":
                for rpc in self._command_rpcs(port):
                    hooks.append(f"on{prefix}{rpc.name}")
            elif port.role == "consumed" and port.port_kind == "request":
                hooks.append(f"on{prefix}Transition")
            elif port.role == "consumed" and port.port_kind == "information":
                hooks.append(f"on{prefix}")
        return tuple(sorted(hooks))

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
            "with Interfaces.C;",
            "with Pcl_Bindings;",
            "with Pcl_Process_Runtime;",
            "",
            f"package {self.package} is",
            "   type Component_Skeleton is abstract tagged limited private;",
            "",
            "   procedure Bind",
            "     (Self      : access Component_Skeleton'Class;",
            "      Container : Pcl_Bindings.Pcl_Container_Access;",
            "      Exec      : Pcl_Bindings.Pcl_Executor_Access);",
            "",
        ])
        for port in self._emitted_ports():
            lines.extend(self._spec_for_port(port))
        lines.extend([
            "   procedure On_Tick",
            "     (Self       : in out Component_Skeleton;",
            "      Dt_Seconds : Interfaces.C.double) is null;",
            "",
            "   function Deployment_Ports return Pcl_Process_Runtime.Port_Array;",
            "",
            "private",
            "   type Component_Skeleton is abstract tagged limited null record;",
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
            "   Instance : access Component_Skeleton'Class := null;",
            "   Instance_Executor : Pcl_Bindings.Pcl_Executor_Access := null;",
            "",
        ])
        for port in self._emitted_ports():
            lines.extend(self._trampolines(port))
        lines.extend([
            "   procedure Bind",
            "     (Self      : access Component_Skeleton'Class;",
            "      Container : Pcl_Bindings.Pcl_Container_Access;",
            "      Exec      : Pcl_Bindings.Pcl_Executor_Access) is",
            "   begin",
            "      if Instance /= null then",
            '         raise Program_Error with "component skeleton is already bound";',
            "      end if;",
            "      Instance := Self;",
            "      Instance_Executor := Exec;",
        ])
        for port in self._emitted_ports():
            lines.extend(self._bind_lines(port))
        lines.extend([
            "   end Bind;",
            "",
        ])
        for port in self._emitted_ports():
            lines.extend(self._helper_bodies(port))
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

    def _spec_for_port(self, port: ComponentPort) -> List[str]:
        prefix = _service_ada_prefix(port.service_name)
        lines: List[str] = []
        if port.role == "provided" and port.port_kind == "request":
            for rpc in self._command_rpcs(port):
                lines.extend([
                    f"   function On_{prefix}_{rpc.name}",
                    "     (Self    : in out Component_Skeleton;",
                    f"      Request : {_ada_req_type(rpc)}) return "
                    f"{_ada_rsp_type(rpc)} is abstract;",
                    "",
                ])
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure Send_{prefix}_Transition",
                f"     (Self : in out Component_Skeleton; Item : {frame});",
                "",
            ])
        elif port.role == "consumed" and port.port_kind == "request":
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure On_{prefix}_Transition",
                f"     (Self : in out Component_Skeleton; Item : {frame}) is null;",
                "",
            ])
            for rpc in self._command_rpcs(port):
                lines.extend([
                    f"   procedure Submit_{prefix}_{rpc.name}",
                    "     (Self            : in out Component_Skeleton;",
                    f"      Request         : {_ada_req_type(rpc)};",
                    "      Result_Accepted : out Boolean;",
                    "      Result_Status    : out Interfaces.C.int;",
                    "      Result_Has_Ack   : out Boolean;",
                    f"      Result_Ack       : out {_ada_rsp_type(rpc)});",
                    "",
                ])
        elif port.role == "provided" and port.port_kind == "information":
            lines.extend([
                f"   procedure Publish_{prefix}",
                "     (Self : in out Component_Skeleton;",
                f"      Item : {_frame_type(self._read_rpc(port))});",
                "",
            ])
        elif port.role == "consumed" and port.port_kind == "information":
            lines.extend([
                f"   procedure On_{prefix}",
                "     (Self : in out Component_Skeleton;",
                f"      Item : {_frame_type(self._read_rpc(port))}) is abstract;",
                "",
            ])
        return lines

    def _trampolines(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        lines: List[str] = []
        if port.role == "provided" and port.port_kind == "request":
            for rpc in self._command_rpcs(port):
                lines.extend([
                    f"   function {prefix}_{rpc.name}_Trampoline",
                    f"     (Request : {_ada_req_type(rpc)}) return "
                    f"{_ada_rsp_type(rpc)} is",
                    "   begin",
                    f"      return Instance.On_{prefix}_{rpc.name}(Request);",
                    f"   end {prefix}_{rpc.name}_Trampoline;",
                    "",
                ])
        elif port.role == "consumed" and port.port_kind == "request":
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure {prefix}_Transition_Trampoline (Item : {frame}) is",
                "   begin",
                f"      Instance.On_{prefix}_Transition(Item);",
                f"   end {prefix}_Transition_Trampoline;",
                "",
            ])
        elif port.role == "consumed" and port.port_kind == "information":
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
                f"      Instance.On_{prefix}(Item);",
                f"   end {prefix}_Information_Trampoline;",
                "",
            ])
        return lines

    def _bind_lines(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        if port.role == "provided" and port.port_kind == "request":
            projectability = {
                item.rpc_name: item
                for item in command_projectability_for_service(
                    self.index, self._proto(port), self._service(port)
                )
            }
            supported = [
                rpc for rpc in self._command_rpcs(port)
                if projectability.get(rpc.name)
                and projectability[rpc.name].projectable
            ]
            lines = [
                f"      {package}.{prefix}_Provider_Bind",
                "        (Container, Exec,",
                f"         {package}.{prefix}_Interaction_Handlers'(",
            ]
            for index, rpc in enumerate(supported):
                comma = "," if index < len(supported) - 1 else ""
                lines.append(
                    f"           On_{rpc.name} => "
                    f"{prefix}_{rpc.name}_Trampoline'Access{comma}"
                )
            lines.append("         ));")
            return lines
        if port.role == "consumed" and port.port_kind == "request":
            read = self._read_rpc(port)
            return [
                f"      {package}.{prefix}_Client_Bind(Container, Exec);",
                f"      {package}.{prefix}_Transitions",
                f"        ({_ada_req_type(read)}'(others => <>),",
                f"         {prefix}_Transition_Trampoline'Access);",
            ]
        if port.role == "consumed" and port.port_kind == "information":
            if not self._ada_info_topic_available(port):
                return [
                    f"      --  {port.port_key}: the current Ada facade does "
                    "not expose a local-wrapper topic decoder."
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

    def _helper_bodies(self, port: ComponentPort) -> List[str]:
        package = self._service_package(port)
        prefix = _service_ada_prefix(port.service_name)
        lines: List[str] = []
        if port.role == "provided" and port.port_kind == "request":
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure Send_{prefix}_Transition",
                f"     (Self : in out Component_Skeleton; Item : {frame}) is",
                "      pragma Unreferenced(Self);",
                "   begin",
                f"      {package}.{prefix}_Send_Transition(Item);",
                f"   end Send_{prefix}_Transition;",
                "",
            ])
        elif port.role == "consumed" and port.port_kind == "request":
            for rpc in self._command_rpcs(port):
                lines.extend([
                    f"   procedure Submit_{prefix}_{rpc.name}",
                    "     (Self            : in out Component_Skeleton;",
                    f"      Request         : {_ada_req_type(rpc)};",
                    "      Result_Accepted : out Boolean;",
                    "      Result_Status    : out Interfaces.C.int;",
                    "      Result_Has_Ack   : out Boolean;",
                    f"      Result_Ack       : out {_ada_rsp_type(rpc)}) is",
                    "      pragma Unreferenced(Self);",
                    "   begin",
                    f"      {package}.{prefix}_Submit_{rpc.name}",
                    "        (Request, Result_Accepted, Result_Status,",
                    "         Result_Has_Ack, Result_Ack);",
                    f"   end Submit_{prefix}_{rpc.name};",
                    "",
                ])
        elif port.role == "provided" and port.port_kind == "information":
            frame = _frame_type(self._read_rpc(port))
            lines.extend([
                f"   procedure Publish_{prefix}",
                f"     (Self : in out Component_Skeleton; Item : {frame}) is",
                "      pragma Unreferenced(Self);",
                "   begin",
            ])
            if self._ada_info_topic_available(port):
                wire = self._interaction(port).legs[0].side_b[0].endpoint_name
                suffix = _topic_suffix(wire)
                lines.append(
                    f"      {package}.Publish_{suffix}"
                    "(Instance_Executor, Item);"
                )
            else:
                lines.extend([
                    "      pragma Unreferenced(Item);",
                    '      raise Program_Error with "the current Ada facade '
                    'does not expose this local-wrapper information topic";',
                ])
            lines.extend([
                f"   end Publish_{prefix};",
                "",
            ])
        return lines

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

    def _pure_hooks(self):
        hooks = []
        for port in self._emitted_ports():
            prefix = _service_ada_prefix(port.service_name)
            if port.role == "provided" and port.port_kind == "request":
                for rpc in self._command_rpcs(port):
                    hooks.append((
                        "function",
                        f"On_{prefix}_{rpc.name}",
                        _ada_req_type(rpc),
                        _ada_rsp_type(rpc),
                    ))
            elif port.role == "consumed" and port.port_kind == "information":
                hooks.append((
                    "procedure",
                    f"On_{prefix}",
                    _frame_type(self._read_rpc(port)),
                    "",
                ))
        return tuple(hooks)

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
            "   type Component is new Component_Skeleton with null record;",
            "",
        ])
        for kind, name, request, response in self._pure_hooks():
            if kind == "function":
                lines.extend([
                    f"   overriding function {name}",
                    "     (Self    : in out Component;",
                    f"      Request : {request}) return {response};",
                    "",
                ])
            else:
                lines.extend([
                    f"   overriding procedure {name}",
                    "     (Self : in out Component;",
                    f"      Item : {request});",
                    "",
                ])
        lines.extend([
            "   overriding procedure On_Tick",
            "     (Self       : in out Component;",
            "      Dt_Seconds : Interfaces.C.double);",
            f"end {self._impl_package};",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_body(self) -> str:
        lines = [
            f"package body {self._impl_package} is",
        ]
        for kind, name, request, response in self._pure_hooks():
            if kind == "function":
                lines.extend([
                    f"   overriding function {name}",
                    "     (Self    : in out Component;",
                    f"      Request : {request}) return {response} is",
                    "      pragma Unreferenced(Self, Request);",
                    "   begin",
                    "      --  TODO: implement component business logic.",
                    "      return (others => <>);",
                    f"   end {name};",
                    "",
                ])
            else:
                lines.extend([
                    f"   overriding procedure {name}",
                    "     (Self : in out Component;",
                    f"      Item : {request}) is",
                    "      pragma Unreferenced(Self, Item);",
                    "   begin",
                    "      --  TODO: implement component business logic.",
                    "      null;",
                    f"   end {name};",
                    "",
                ])
        lines.extend([
            "   overriding procedure On_Tick",
            "     (Self       : in out Component;",
            "      Dt_Seconds : Interfaces.C.double) is",
            "      pragma Unreferenced(Self, Dt_Seconds);",
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
        return "\n".join([
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
            f"   Logic : aliased {self._impl_package}.Component;",
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
            "        (Logic'Access, Pcl_Component.Handle(Self),",
            "         Pcl_Process_Runtime.Executor(Runtime));",
            "   end On_Configure;",
            "",
            "   overriding procedure On_Tick",
            "     (Self       : in out Runtime_Container;",
            "      Dt_Seconds : Interfaces.C.double) is",
            "   begin",
            "      pragma Unreferenced(Self);",
            f"      {self._impl_package}.On_Tick(Logic, Dt_Seconds);",
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


def _service_cpp_style_prefix(service_name: str) -> str:
    name = service_name
    if name.endswith("_Service"):
        name = name[:-len("_Service")]
    return "".join(part.capitalize() for part in name.split("_") if part)
