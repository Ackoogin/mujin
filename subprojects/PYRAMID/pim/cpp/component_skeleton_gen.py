#!/usr/bin/env python3
"""Deterministic C++ component skeleton generation."""

from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from binding_contract import (
    ComponentGroup,
    ComponentPort,
    NamingPolicy,
    interaction_for_service,
)
from proto_parser import (
    ProtoFile,
    ProtoRpc,
    ProtoService,
    ProtoTypeIndex,
    snake_to_pascal,
)
from .naming import _cpp_req_type, _cpp_rsp_type, _service_cpp_prefix


def _component_pascal(name: str) -> str:
    return "".join(snake_to_pascal(part) for part in name.split("."))


def _lower_camel(name: str) -> str:
    pascal = snake_to_pascal(name)
    return pascal[:1].lower() + pascal[1:]


def _frame_type(rpc: ProtoRpc) -> str:
    value = _cpp_rsp_type(rpc)
    prefix = "std::vector<"
    if value.startswith(prefix) and value.endswith(">"):
        return value[len(prefix):-1]
    return value


def _qualified(role: str, cpp_type: str) -> str:
    if (
        cpp_type.startswith("std::")
        or "::" in cpp_type
        or cpp_type in {
            "bool", "double", "float", "int32_t", "int64_t",
            "uint32_t", "uint64_t",
        }
    ):
        return cpp_type
    return f"services::{role}::{cpp_type}"


class CppComponentSkeletonGenerator:
    """Generate one component-spanning C++ skeleton pair."""

    def __init__(
        self,
        group: ComponentGroup,
        proto_files: Iterable[ProtoFile],
        index: ProtoTypeIndex,
        naming_policy: NamingPolicy,
    ):
        self.group = group
        self.index = index
        self.naming_policy = naming_policy
        self._files: Dict[str, ProtoFile] = {
            proto.package: proto for proto in proto_files
        }
        self._services: Dict[Tuple[str, str], ProtoService] = {}
        for proto in self._files.values():
            for service in proto.services:
                self._services[(proto.package, service.name)] = service

    @property
    def stem(self) -> str:
        component = self.group.component.replace(".", "_")
        return f"pyramid_component_{self.group.project}_{component}_skeleton"

    @property
    def class_name(self) -> str:
        return f"{_component_pascal(self.group.component)}Skeleton"

    @property
    def namespace(self) -> str:
        suffix = self.group.component.replace(".", "::")
        return f"pyramid::components::{self.group.project}::{suffix}"

    def generate(self, output_dir: str) -> List[Path]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        header = output / f"{self.stem}.hpp"
        source = output / f"{self.stem}.cpp"
        header.write_text(self.render_header(), encoding="utf-8", newline="\n")
        source.write_text(self.render_source(), encoding="utf-8", newline="\n")
        print(f"  Generated {header}")
        print(f"  Generated {source}")
        return [header, source]

    def generate_scaffold(self, scaffold_dir: str) -> List[Path]:
        """Write the C++ process scaffold without replacing user files."""
        root = (
            Path(scaffold_dir)
            / f"{self.group.project}_{self.group.component.replace('.', '_')}"
        )
        include_dir = (
            root / "include"
            / f"{self.group.project}_{self.group.component.replace('.', '_')}"
        )
        source_dir = root / "src"
        component_file = f"{self.group.component.replace('.', '_')}_component"
        paths = [
            self._write_if_absent(
                include_dir / f"{component_file}.hpp",
                self._scaffold_header(component_file),
            ),
            self._write_if_absent(
                source_dir / f"{component_file}.cpp",
                self._scaffold_source(component_file),
            ),
            self._write_if_absent(
                source_dir / f"{self.group.component.replace('.', '_')}_main.cpp",
                self._scaffold_main(component_file),
            ),
            self._write_if_absent(
                root / "CMakeLists.txt",
                self._scaffold_cmake(component_file),
            ),
        ]
        for platform, plugin_suffix in (
            ("linux", ".so"),
            ("windows", ".dll"),
        ):
            for transport in ("tcp", "shared_memory"):
                config = (
                    root / "configs" / platform / transport
                    / f"{self.group.component.replace('.', '_')}.ports"
                )
                paths.append(self._write_if_absent(
                    config,
                    self._ports_template(
                        platform, transport, plugin_suffix),
                ))
        return paths

    def hook_names(self) -> Tuple[str, ...]:
        hooks = []
        for port in self._emitted_ports():
            prefix = _service_cpp_prefix(port.service_name)
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

    def render_header(self) -> str:
        lines = [
            "// Auto-generated component skeleton. Regenerated on every run.",
            "#pragma once",
            "",
        ]
        packages = sorted({
            port.package for port in self._emitted_ports()
        })
        for package in packages:
            prefix = self.naming_policy.service_file_prefix(package)
            lines.append(f'#include "{prefix}_components.hpp"')
        lines.extend([
            "",
            "#include <pcl/component.hpp>",
            "#include <pcl/executor.hpp>",
            "#include <pcl/process_runtime.hpp>",
            "",
            "#include <future>",
            "#include <string>",
            "#include <vector>",
            "",
            f"namespace {self.namespace} {{",
            "",
            f"class {self.class_name} : public pcl::Component {{",
            "private:",
        ])
        adapters = [
            port for port in self._emitted_ports()
            if port.role == "provided" and port.port_kind == "request"
        ]
        if not adapters:
            lines.append("  // This component has no provided request adapters.")
        for port in adapters:
            lines.extend(self._adapter_declaration(port))
        lines.extend([
            "",
            "public:",
            f'  explicit {self.class_name}(',
            "      pcl::Executor& executor,",
            f'      std::string name = "{self.group.component}");',
            f"  ~{self.class_name}() override = default;",
            "",
            "  static std::vector<pcl::DeploymentPort> deploymentPorts();",
            "",
            f"  {self.class_name}(const {self.class_name}&) = delete;",
            f"  {self.class_name}& operator=(const {self.class_name}&) = delete;",
            "",
            "protected:",
            "  pcl_status_t on_configure() override;",
            "  virtual pcl_status_t on_user_configure() { return PCL_OK; }",
            "",
            "  // Provider callbacks are deliberately pure: the generated",
            "  // facade handlers have defaults, but a component skeleton does not.",
        ])
        for port in self._emitted_ports():
            lines.extend(self._hook_and_helper_declarations(port))
        lines.extend([
            "",
            "private:",
        ])
        for port in self._emitted_ports():
            if port.role == "provided" and port.port_kind == "request":
                lines.append(
                    f"  {_adapter_name(port)} {_adapter_member(port)};"
                )
        for port in self._emitted_ports():
            facade = self._facade_type(port)
            lines.append(f"  {facade} {port.port_key}_port_;")
            if port.role == "consumed":
                subscription_type = self._subscription_type(port)
                if subscription_type:
                    lines.append(
                        f"  {subscription_type} "
                        f"{port.port_key}_subscription_;"
                    )
        for port in self.group.ports:
            if not port.has_interaction:
                lines.append(
                    f"  // {port.port_key}: no complete interaction legs; "
                    "no facade member was generated."
                )
        lines.extend([
            "};",
            "",
            f"}}  // namespace {self.namespace}",
            "",
        ])
        return "\n".join(lines)

    def render_source(self) -> str:
        emitted = self._emitted_ports()
        initializers = ["pcl::Component(name)"]
        for port in emitted:
            if port.role == "provided" and port.port_kind == "request":
                initializers.append(f"{_adapter_member(port)}(*this)")
        for port in emitted:
            args = "*this, executor"
            if port.role == "provided" and port.port_kind == "request":
                args += f", {_adapter_member(port)}"
            initializers.append(f"{port.port_key}_port_({args})")
        lines = [
            f'#include "{self.stem}.hpp"',
            "",
            f"namespace {self.namespace} {{",
            "",
            f"{self.class_name}::{self.class_name}(",
            "    pcl::Executor& executor, std::string name)",
            "    : " + ",\n      ".join(initializers) + " {}",
            "",
            f"pcl_status_t {self.class_name}::on_configure() {{",
        ]
        for port in emitted:
            lines.extend([
                f"  if (const auto status = {port.port_key}_port_.bind();",
                "      status != PCL_OK) {",
                "    return status;",
                "  }",
            ])
            if port.role == "consumed" and port.port_kind == "information":
                frame = self._frame_type(port)
                prefix = _service_cpp_prefix(port.service_name)
                lines.extend([
                    f"  {port.port_key}_subscription_ =",
                    f"      {port.port_key}_port_.subscribe(",
                    f"          [this](const {frame}& item) {{ "
                    f"on{prefix}(item); }});",
                    f"  if (!{port.port_key}_subscription_) "
                    "return PCL_ERR_STATE;",
                ])
            elif port.role == "consumed" and port.port_kind == "request":
                service = self._service(port)
                read = next(rpc for rpc in service.rpcs if rpc.name == "Read")
                query = _qualified(port.role, _cpp_req_type(read))
                frame = self._frame_type(port)
                prefix = _service_cpp_prefix(port.service_name)
                lines.extend([
                    f"  {port.port_key}_subscription_ =",
                    f"      {port.port_key}_port_.transitions(",
                    f"          {query}{{}},",
                    f"          [this](const {frame}& item) {{ "
                    f"on{prefix}Transition(item); }});",
                    f"  if (!{port.port_key}_subscription_) "
                    "return PCL_ERR_STATE;",
                ])
        lines.extend([
            "  return on_user_configure();",
            "}",
            "",
            f"std::vector<pcl::DeploymentPort> "
            f"{self.class_name}::deploymentPorts() {{",
            "  return {",
        ])
        for port in emitted:
            lines.append(
                f'      {{"{port.port_key}", '
                f"{self._facade_type(port)}::deploymentDescriptor()}},"
            )
        lines.extend([
            "  };",
            "}",
            "",
            f"}}  // namespace {self.namespace}",
            "",
        ])
        return "\n".join(lines)

    def _emitted_ports(self) -> Tuple[ComponentPort, ...]:
        return tuple(port for port in self.group.ports if port.has_interaction)

    def _service(self, port: ComponentPort) -> ProtoService:
        return self._services[(port.package, port.service_name)]

    def _interaction(self, port: ComponentPort):
        proto = self._files[port.package]
        return interaction_for_service(
            self.index, proto, self._service(port)
        )

    def _command_rpcs(self, port: ComponentPort) -> Tuple[ProtoRpc, ...]:
        interaction = self._interaction(port)
        request_leg = next(
            leg for leg in interaction.legs if leg.name == "request"
        )
        names = {endpoint.rpc_name for endpoint in request_leg.side_a}
        return tuple(sorted(
            (
                rpc for rpc in self._service(port).rpcs
                if rpc.name in names
            ),
            key=lambda rpc: rpc.name,
        ))

    def _frame_type(self, port: ComponentPort) -> str:
        read = next(
            rpc for rpc in self._service(port).rpcs if rpc.name == "Read"
        )
        return _qualified(port.role, _frame_type(read))

    def _facade_type(self, port: ComponentPort) -> str:
        return f"services::{port.role}::{port.facade}"

    def _subscription_type(self, port: ComponentPort) -> Optional[str]:
        facade = self._facade_type(port)
        if port.port_kind in ("request", "information"):
            return f"services::{port.role}::SubscriptionHandle"
        return None

    def _adapter_declaration(self, port: ComponentPort) -> List[str]:
        role = port.role
        prefix = _service_cpp_prefix(port.service_name)
        handler = (
            f"services::{role}::{prefix}RequestPortHandler"
        )
        lines = [
            f"  struct {_adapter_name(port)} final : {handler} {{",
            f"    explicit {_adapter_name(port)}({self.class_name}& owner)",
            "        : owner_(owner) {}",
        ]
        for rpc in self._command_rpcs(port):
            request = _qualified(role, _cpp_req_type(rpc))
            response = _qualified(role, _cpp_rsp_type(rpc))
            lines.extend([
                f"    {response} on{rpc.name}("
                f"const {request}& request) override {{",
                f"      return owner_.on{prefix}{rpc.name}(request);",
                "    }",
            ])
        lines.extend([
            f"    {self.class_name}& owner_;",
            "  };",
        ])
        return lines

    def _pure_hooks(self):
        hooks = []
        for port in self._emitted_ports():
            prefix = _service_cpp_prefix(port.service_name)
            if port.role == "provided" and port.port_kind == "request":
                for rpc in self._command_rpcs(port):
                    hooks.append((
                        _qualified(port.role, _cpp_rsp_type(rpc)),
                        f"on{prefix}{rpc.name}",
                        _qualified(port.role, _cpp_req_type(rpc)),
                    ))
            elif port.role == "consumed" and port.port_kind == "information":
                hooks.append((
                    "void",
                    f"on{prefix}",
                    self._frame_type(port),
                ))
        return tuple(hooks)

    def _scaffold_namespace(self) -> str:
        return self.namespace

    def _scaffold_class(self) -> str:
        return f"{_component_pascal(self.group.component)}Component"

    def _scaffold_header(self, component_file: str) -> str:
        del component_file
        lines = [
            "#pragma once",
            "",
            f'#include "{self.stem}.hpp"',
            "",
            f"namespace {self._scaffold_namespace()} {{",
            "",
            f"class {self._scaffold_class()} final : public {self.class_name} {{",
            "public:",
            f"  using {self.class_name}::{self.class_name};",
            "",
            "protected:",
        ]
        for response, name, request in self._pure_hooks():
            lines.append(
                f"  {response} {name}(const {request}& request) override;"
            )
        lines.extend([
            "  pcl_status_t on_tick(double dt) override;",
            "};",
            "",
            f"}}  // namespace {self._scaffold_namespace()}",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_source(self, component_file: str) -> str:
        include_root = (
            f"{self.group.project}_"
            f"{self.group.component.replace('.', '_')}"
        )
        lines = [
            f'#include "{include_root}/{component_file}.hpp"',
            "",
            f"namespace {self._scaffold_namespace()} {{",
            "",
        ]
        for response, name, request in self._pure_hooks():
            lines.extend([
                f"{response} {self._scaffold_class()}::{name}(",
                f"    const {request}& request) {{",
                "  (void)request;",
                "  // TODO: implement component business logic.",
            ])
            if response != "void":
                lines.append("  return {};")
            lines.extend([
                "}",
                "",
            ])
        lines.extend([
            f"pcl_status_t {self._scaffold_class()}::on_tick(double dt) {{",
            "  (void)dt;",
            "  // TODO: implement periodic component work.",
            "  return PCL_OK;",
            "}",
            "",
            f"}}  // namespace {self._scaffold_namespace()}",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_main(self, component_file: str) -> str:
        include_root = (
            f"{self.group.project}_"
            f"{self.group.component.replace('.', '_')}"
        )
        label = self.group.component.replace(".", "_")
        return "\n".join([
            f'#include "{include_root}/{component_file}.hpp"',
            "",
            "#include <pcl/process_runtime.hpp>",
            "",
            "#include <exception>",
            "#include <iostream>",
            "",
            "#ifndef PYRAMID_COMPONENT_CODEC_PLUGIN_PATH",
            '#  define PYRAMID_COMPONENT_CODEC_PLUGIN_PATH ""',
            "#endif",
            "",
            "int main(int argc, char** argv) {",
            "  try {",
            "    pcl::ProcessRuntime runtime(",
            "        argc, argv, {PYRAMID_COMPONENT_CODEC_PLUGIN_PATH},",
            f"        {self.namespace}::{self.class_name}::deploymentPorts());",
            f"    {self.namespace}::{self._scaffold_class()} component(",
            "        runtime.executor());",
            "    return runtime.run(component);",
            "  } catch (const std::exception& error) {",
            f'    std::cerr << "{label}: " << error.what() << std::endl;',
            "    return 1;",
            "  }",
            "}",
            "",
        ])

    def _scaffold_cmake(self, component_file: str) -> str:
        del component_file
        component = self.group.component.replace(".", "_")
        target = f"{self.group.project}_{component}"
        bindings = []
        if self.group.provided_package:
            bindings.append(
                self.naming_policy.service_file_prefix(
                    self.group.provided_package
                ).removeprefix("pyramid_services_")
            )
        if self.group.consumed_package:
            bindings.append(
                self.naming_policy.service_file_prefix(
                    self.group.consumed_package
                ).removeprefix("pyramid_services_")
            )
        codec_target = f"pyramid_codec_json_{self.group.project}_{component}"
        return "\n".join([
            "cmake_minimum_required(VERSION 3.21)",
            f"project({target} LANGUAGES CXX)",
            "",
            'set(PYRAMID_SDK "" CACHE PATH "Path to the PYRAMID SDK")',
            "if(NOT PYRAMID_SDK)",
            '  message(FATAL_ERROR "Set PYRAMID_SDK to the SDK directory.")',
            "endif()",
            "",
            "set(PYRAMID_SDK_SERVICE_BINDINGS",
            f'    "{";".join(bindings)}" CACHE STRING "" FORCE)',
            "set(PYRAMID_SDK_COMPONENTS",
            f'    "{self.group.key}" CACHE STRING "" FORCE)',
            "add_subdirectory(${PYRAMID_SDK}/sdk_project",
            "                 ${CMAKE_CURRENT_BINARY_DIR}/pyramid_sdk)",
            "",
            f"add_executable({target}",
            f"    src/{component}_component.cpp",
            f"    src/{component}_main.cpp)",
            f"target_include_directories({target} PRIVATE include)",
            f"target_link_libraries({target} PRIVATE",
            "    pyramid_component_skeletons",
            "    pyramid_generated_services",
            "    pcl_core)",
            f"target_compile_definitions({target} PRIVATE",
            '    "PYRAMID_COMPONENT_CODEC_PLUGIN_PATH='
            f'\\"$<TARGET_FILE:{codec_target}>\\"")',
            f"add_dependencies({target} {codec_target})",
            "if(MSVC)",
            f"  target_compile_options({target} PRIVATE /bigobj)",
            "endif()",
            "",
        ])

    def _ports_template(
        self,
        platform: str,
        transport: str,
        plugin_suffix: str,
    ) -> str:
        del platform
        if transport == "tcp":
            plugin = f"plugins/pcl_transport_socket_plugin{plugin_suffix}"
            config = (
                '{"role":"<provided-or-consumed>",'
                '"host":"127.0.0.1","port":<port>}'
            )
        else:
            plugin = (
                "plugins/pcl_transport_shared_memory_plugin"
                f"{plugin_suffix}"
            )
            config = (
                '{"bus_name":"<shared-bus>",'
                f'"participant_id":"{self.group.component}"' + "}"
            )
        lines = [
            "# One line configures one generated logical port.",
            "# port NAME MODE PEER PLUGIN PLUGIN_CONFIG",
            "# EDIT: replace every <...> placeholder before running.",
        ]
        for port in self._emitted_ports():
            mode = "rpc" if port.port_kind == "request" else "pubsub"
            lines.append(
                f"port {port.port_key} {mode} <peer-process> "
                f"{plugin} {config}"
            )
        lines.append("")
        return "\n".join(lines)

    @staticmethod
    def _write_if_absent(path: Path, content: str) -> Path:
        if path.exists():
            print(f"scaffold: kept {path}")
            return path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8", newline="\n")
        print(f"scaffold: created {path}")
        return path

    def _hook_and_helper_declarations(
        self,
        port: ComponentPort,
    ) -> List[str]:
        prefix = _service_cpp_prefix(port.service_name)
        facade = self._facade_type(port)
        accessor = _lower_camel(port.port_key) + "Port"
        lines: List[str] = []
        if port.role == "provided" and port.port_kind == "request":
            for rpc in self._command_rpcs(port):
                request = _qualified(port.role, _cpp_req_type(rpc))
                response = _qualified(port.role, _cpp_rsp_type(rpc))
                lines.append(
                    f"  virtual {response} on{prefix}{rpc.name}("
                    f"const {request}& request) = 0;"
                )
            lines.append(
                f"  pcl_status_t send{prefix}Transition("
                f"const {self._frame_type(port)}& item) {{"
            )
            lines.append(
                f"    return {port.port_key}_port_.transitionWriter().send(item);"
            )
            lines.append("  }")
        elif port.role == "consumed" and port.port_kind == "request":
            lines.append(
                f"  virtual void on{prefix}Transition("
                f"const {self._frame_type(port)}& item) {{ (void)item; }}"
            )
            for rpc in self._command_rpcs(port):
                request = _qualified(port.role, _cpp_req_type(rpc))
                lines.append(
                    f"  std::future<typename {facade}::SubmitResult> "
                    f"submit{prefix}{rpc.name}("
                )
                lines.append(f"      const {request}& request) {{")
                lines.append(
                    f"    return {port.port_key}_port_.submit(request);"
                )
                lines.append("  }")
        elif port.role == "provided" and port.port_kind == "information":
            lines.append(
                f"  pcl_status_t publish{prefix}("
                f"const {self._frame_type(port)}& item) {{"
            )
            lines.append(
                f"    return {port.port_key}_port_.publish(item);"
            )
            lines.append("  }")
        elif port.role == "consumed" and port.port_kind == "information":
            lines.append(
                f"  virtual void on{prefix}("
                f"const {self._frame_type(port)}& item) = 0;"
            )
        lines.append(f"  {facade}& {accessor}() {{")
        lines.append(f"    return {port.port_key}_port_;")
        lines.append("  }")
        return lines


def _adapter_name(port: ComponentPort) -> str:
    return f"{_service_cpp_prefix(port.service_name)}HandlerAdapter"


def _adapter_member(port: ComponentPort) -> str:
    return f"{port.port_key}_handler_"
