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

    def generate(
        self,
        output_dir: str,
        emit_support: bool = True,
    ) -> List[Path]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        paths: List[Path] = []
        if emit_support:
            support = output / "pyramid_component_skeleton_support.hpp"
            support.write_text(
                self.render_support_header(), encoding="utf-8", newline="\n"
            )
            print(f"  Generated {support}")
            paths.append(support)
        header = output / f"{self.stem}.hpp"
        source = output / f"{self.stem}.cpp"
        header.write_text(self.render_header(), encoding="utf-8", newline="\n")
        source.write_text(self.render_source(), encoding="utf-8", newline="\n")
        print(f"  Generated {header}")
        print(f"  Generated {source}")
        return paths + [header, source]

    def generate_scaffold(self, scaffold_dir: str) -> List[Path]:
        """Write the C++ process scaffold without replacing user files."""
        component = self.group.component.replace(".", "_")
        root = Path(scaffold_dir) / f"{self.group.project}_{component}"
        include_dir = root / "include" / f"{self.group.project}_{component}"
        source_dir = root / "src"
        component_file = f"{component}_component"
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
                source_dir / f"{component}_main.cpp",
                self._scaffold_main(component_file),
            ),
            self._write_if_absent(
                root / "CMakeLists.txt", self._scaffold_cmake(component_file)
            ),
            self._write_if_absent(
                root / "README.md", self._scaffold_readme(component_file)
            ),
        ]
        for platform, plugin_suffix in (("linux", ".so"), ("windows", ".dll")):
            for transport in ("tcp", "shared_memory"):
                config = root / "configs" / platform / transport / f"{component}.ports"
                paths.append(self._write_if_absent(
                    config,
                    self._ports_template(platform, transport, plugin_suffix),
                ))
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

    @staticmethod
    def render_support_header() -> str:
        return "\n".join([
            "// Auto-generated component skeleton support. Regenerated on every run.",
            "#pragma once",
            "",
            "#include <cassert>",
            "#include <functional>",
            "#include <memory>",
            "#include <string>",
            "#include <utility>",
            "",
            "namespace pyramid::component_skeleton {",
            "",
            "// Resolves a port's wire content type by port name. The skeleton",
            "// calls it once per port when constructing the port facades, so a",
            "// process can encode each port with its own codec. An empty",
            "// resolver means \"application/json for every port\".",
            "using ContentTypeResolver =",
            "    std::function<std::string(const std::string&)>;",
            "",
            "inline std::string resolveContentType(",
            "    const ContentTypeResolver& codec_for, const char* port) {",
            "  return codec_for ? codec_for(port) : std::string(\"application/json\");",
            "}",
            "",
            "template <typename T>",
            "class HandlerSlot {",
            "public:",
            "  HandlerSlot(T& handler) : ptr_(&handler) {}",
            "",
            "  HandlerSlot(std::unique_ptr<T> handler)",
            "      : ptr_(handler.get()), owned_(std::move(handler)) {",
            "    assert(ptr_ != nullptr);",
            "  }",
            "",
            "  T& get() {",
            "    assert(ptr_ != nullptr);",
            "    return *ptr_;",
            "  }",
            "",
            "private:",
            "  T* ptr_;",
            "  std::unique_ptr<T> owned_;",
            "};",
            "",
            "}  // namespace pyramid::component_skeleton",
            "",
        ])

    def render_header(self) -> str:
        lines = [
            "// Auto-generated component skeleton. Regenerated on every run.",
            "#pragma once",
            "",
        ]
        packages = sorted({port.package for port in self._emitted_ports()})
        for package in packages:
            prefix = self.naming_policy.service_file_prefix(package)
            lines.append(f'#include "{prefix}_components.hpp"')
        lines.extend([
            '#include "pyramid_component_skeleton_support.hpp"',
            "",
            "#include <pcl/component.hpp>",
            "#include <pcl/executor.hpp>",
            "#include <pcl/process_runtime.hpp>",
            "",
            "#include <functional>",
            "#include <string>",
            "#include <utility>",
            "#include <vector>",
            "",
            f"namespace {self.namespace} {{",
            "",
            f"class {self.class_name} : public pcl::Component {{",
            "public:",
            "  struct Handlers {",
        ])
        slot_lines: List[str] = []
        for port in self._emitted_ports():
            slot_lines.extend(self._handler_slot_declaration(port))
        lines.extend(slot_lines or ["    // This component has no injectable handlers."])
        lines.extend([
            "  };",
            "",
            f"  explicit {self.class_name}(",
            "      pcl::Executor& executor,",
            "      Handlers handlers,",
            "      pyramid::component_skeleton::ContentTypeResolver codec_for = {},",
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
        ])
        for port in self._emitted_ports():
            lines.extend(self._port_accessor_declaration(port))
        lines.extend(["", "private:", "  Handlers handlers_;"])
        for port in self._emitted_ports():
            lines.append(f"  {self._facade_type(port)} {port.port_key}_port_;")
            if port.role == "consumed":
                subscription_type = self._subscription_type(port)
                if subscription_type:
                    lines.append(
                        f"  {subscription_type} {port.port_key}_subscription_;"
                    )
        for port in self.group.ports:
            if not port.has_interaction:
                lines.append(
                    f"  // {port.port_key}: no complete interaction legs; "
                    "no facade member was generated."
                )
        lines.extend(["};", "", f"}}  // namespace {self.namespace}", ""])
        return "\n".join(lines)

    def render_source(self) -> str:
        emitted = self._emitted_ports()
        initializers = ["pcl::Component(name)", "handlers_(std::move(handlers))"]
        for port in emitted:
            args = "*this, executor"
            if port.role == "provided" and port.port_kind == "request":
                args += f", handlers_.{port.port_key}.get()"
            args += (
                ", pyramid::component_skeleton::resolveContentType("
                f'codec_for, "{port.port_key}")'
            )
            initializers.append(f"{port.port_key}_port_({args})")
        lines = [
            f'#include "{self.stem}.hpp"',
            "",
            f"namespace {self.namespace} {{",
            "",
            f"{self.class_name}::{self.class_name}(",
            "    pcl::Executor& executor, Handlers handlers,",
            "    pyramid::component_skeleton::ContentTypeResolver codec_for,",
            "    std::string name)",
            "    : " + ",\n      ".join(initializers) + " {}",
            "",
            f"pcl_status_t {self.class_name}::on_configure() {{",
        ]
        for port in emitted:
            if port.role == "consumed" and port.port_kind == "information":
                lines.extend([
                    f"  if (!handlers_.{port.port_key}) {{",
                    f'    logError("required handler slot {port.port_key} is empty");',
                    "    return PCL_ERR_STATE;",
                    "  }",
                ])
        for port in emitted:
            lines.extend([
                f"  if (const auto status = {port.port_key}_port_.bind();",
                "      status != PCL_OK) {",
                "    return status;",
                "  }",
            ])
            if port.role == "consumed" and port.port_kind == "information":
                lines.extend([
                    f"  {port.port_key}_subscription_ =",
                    f"      {port.port_key}_port_.subscribe(",
                    f"          handlers_.{port.port_key});",
                    f"  if (!{port.port_key}_subscription_) return PCL_ERR_STATE;",
                ])
            elif port.role == "consumed" and port.port_kind == "request":
                read = next(rpc for rpc in self._service(port).rpcs if rpc.name == "Read")
                query = _qualified(port.role, _cpp_req_type(read))
                lines.extend([
                    f"  if (handlers_.{port.port_key}_transitions) {{",
                    f"    {port.port_key}_subscription_ =",
                    f"        {port.port_key}_port_.transitions(",
                    f"            {query}{{}},",
                    f"            handlers_.{port.port_key}_transitions);",
                    f"    if (!{port.port_key}_subscription_) return PCL_ERR_STATE;",
                    "  }",
                ])
        lines.extend([
            "  return on_user_configure();",
            "}",
            "",
            f"std::vector<pcl::DeploymentPort> {self.class_name}::deploymentPorts() {{",
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
        return interaction_for_service(
            self.index, self._files[port.package], self._service(port)
        )

    def _command_rpcs(self, port: ComponentPort) -> Tuple[ProtoRpc, ...]:
        request_leg = next(
            leg for leg in self._interaction(port).legs if leg.name == "request"
        )
        names = {endpoint.rpc_name for endpoint in request_leg.side_a}
        return tuple(sorted(
            (rpc for rpc in self._service(port).rpcs if rpc.name in names),
            key=lambda rpc: rpc.name,
        ))

    def _frame_type(self, port: ComponentPort) -> str:
        read = next(rpc for rpc in self._service(port).rpcs if rpc.name == "Read")
        return _qualified(port.role, _frame_type(read))

    def _facade_type(self, port: ComponentPort) -> str:
        return f"services::{port.role}::{port.facade}"

    def _subscription_type(self, port: ComponentPort) -> Optional[str]:
        if port.port_kind in ("request", "information"):
            return f"services::{port.role}::SubscriptionHandle"
        return None

    def _handler_slot_declaration(self, port: ComponentPort) -> List[str]:
        prefix = _service_cpp_prefix(port.service_name)
        if port.role == "provided" and port.port_kind == "request":
            return [
                "    pyramid::component_skeleton::HandlerSlot<",
                f"        services::provided::{prefix}RequestPortHandler>",
                f"        {port.port_key};",
            ]
        if port.role == "consumed" and port.port_kind == "information":
            return [
                f"    std::function<void(const {self._frame_type(port)}&)>",
                f"        {port.port_key};",
            ]
        if port.role == "consumed" and port.port_kind == "request":
            return [
                f"    std::function<void(const {self._frame_type(port)}&)>",
                f"        {port.port_key}_transitions{{}};",
            ]
        return []

    def _port_accessor_declaration(self, port: ComponentPort) -> List[str]:
        accessor = _lower_camel(port.port_key) + "Port"
        return [
            f"  {self._facade_type(port)}& {accessor}() {{",
            f"    return {port.port_key}_port_;",
            "  }",
        ]

    def _provider_ports(self) -> Tuple[ComponentPort, ...]:
        return tuple(
            port for port in self._emitted_ports()
            if port.role == "provided" and port.port_kind == "request"
        )

    def _scaffold_namespace(self) -> str:
        return self.namespace

    def _scaffold_class(self) -> str:
        return f"{_component_pascal(self.group.component)}Component"

    def _handler_class(self, port: ComponentPort) -> str:
        return f"{_service_cpp_prefix(port.service_name)}Handler"

    def _accessor_name(self, port: ComponentPort) -> str:
        """Name of the skeleton's protected accessor for this port facade.

        Must match the accessor emitted by _port_accessor_declaration so the
        scaffold can re-expose it with a using-declaration.
        """
        return _lower_camel(port.port_key) + "Port"

    def _sink_method(self, port: ComponentPort) -> str:
        """Component method that receives frames for a consumed sink port."""
        suffix = "Transitions" if port.port_kind == "request" else ""
        return "on" + snake_to_pascal(port.port_key) + suffix

    def _sink_ports(self) -> Tuple[ComponentPort, ...]:
        """Consumed ports the component receives frames on (sinks)."""
        return tuple(
            port for port in self._emitted_ports()
            if port.role == "consumed"
            and port.port_kind in ("information", "request")
        )

    def _scaffold_header(self, component_file: str) -> str:
        del component_file
        ns = self._scaffold_namespace()
        component_class = self._scaffold_class()
        provider_ports = self._provider_ports()
        lines = [
            "#pragma once",
            "",
            f'#include "{self.stem}.hpp"',
            "",
            "#include <string>",
            "",
            f"namespace {ns} {{",
            "",
            f"class {component_class};",
            "",
        ]
        if provider_ports:
            lines.extend([
                "// One handler object per provided request port. Each keeps a",
                f"// reference to the {component_class} so its business logic (defined",
                "// in the .cpp) can publish on output ports and call consumed",
                "// services through the component's port accessors below.",
            ])
        for port in provider_ports:
            prefix = _service_cpp_prefix(port.service_name)
            handler = self._handler_class(port)
            lines.extend([
                f"class {handler} final",
                f"    : public services::provided::{prefix}RequestPortHandler {{",
                "public:",
                f"  explicit {handler}({component_class}& component)",
                "      : component_(component) {}",
                "",
            ])
            for rpc in self._command_rpcs(port):
                request = _qualified(port.role, _cpp_req_type(rpc))
                response = _qualified(port.role, _cpp_rsp_type(rpc))
                lines.append(
                    f"  {response} on{rpc.name}(const {request}& request) override;"
                )
            lines.extend([
                "",
                "private:",
                f"  {component_class}& component_;",
                "};",
                "",
            ])
        lines.extend([
            f"class {component_class} final : public {self.class_name} {{",
            "public:",
            f"  explicit {component_class}(",
            "      pcl::Executor& executor,",
            "      pyramid::component_skeleton::ContentTypeResolver "
            "codec_for = {});",
            "",
        ])
        emitted = self._emitted_ports()
        if emitted:
            lines.extend([
                "  // Port instances exposed for use from on_tick and the port",
                "  // handlers. Publish on output ports, or call consumed services,",
                "  // through these accessors.",
            ])
            for port in emitted:
                lines.append(
                    f"  using {self.class_name}::{self._accessor_name(port)};"
                )
            lines.append("")
        lines.extend([
            "protected:",
            "  // Runs once after every port is bound. Cache writers or start",
            "  // periodic work here.",
            "  pcl_status_t on_user_configure() override;",
            "  // Periodic component work.",
            "  pcl_status_t on_tick(double dt) override;",
        ])
        for port in self._sink_ports():
            frame = self._frame_type(port)
            lines.append(
                f"  void {self._sink_method(port)}(const {frame}& frame);"
            )
        lines.extend([
            "",
            "private:",
            "  Handlers makeHandlers();",
            "};",
            "",
            f"}}  // namespace {ns}",
            "",
        ])
        return "\n".join(lines)

    def _scaffold_source(self, component_file: str) -> str:
        include_root = (
            f"{self.group.project}_{self.group.component.replace('.', '_')}"
        )
        ns = self._scaffold_namespace()
        component_class = self._scaffold_class()
        lines = [
            f'#include "{include_root}/{component_file}.hpp"',
            "",
            "#include <memory>",
            "#include <utility>",
            "",
            f"namespace {ns} {{",
            "",
            "// -----------------------------------------------------------------",
            "// Generated wiring. You normally do not edit this section: it binds",
            "// the port handlers to the component and builds the handler set the",
            "// skeleton base owns. Add your logic in the hooks further down.",
            "// -----------------------------------------------------------------",
            "",
            f"{component_class}::{component_class}(",
            "    pcl::Executor& executor,",
            "    pyramid::component_skeleton::ContentTypeResolver codec_for)",
            f"    : {self.class_name}(",
            "          executor, makeHandlers(), std::move(codec_for),",
            f'          "{self.group.component}") {{}}',
            "",
            f"{self.class_name}::Handlers {component_class}::makeHandlers() {{",
            f"  return {self.class_name}::Handlers{{",
        ]
        for port in self._emitted_ports():
            if port.role == "provided" and port.port_kind == "request":
                prefix = _service_cpp_prefix(port.service_name)
                handler = self._handler_class(port)
                lines.extend([
                    f"      /* {port.port_key} = */",
                    "      pyramid::component_skeleton::HandlerSlot<",
                    f"          services::provided::{prefix}RequestPortHandler>(",
                    f"          std::make_unique<{handler}>(*this)),",
                ])
            elif port.role == "consumed" and port.port_kind == "information":
                frame = self._frame_type(port)
                lines.extend([
                    f"      /* {port.port_key} = */",
                    f"      [this](const {frame}& frame) {{",
                    f"        {self._sink_method(port)}(frame);",
                    "      },",
                ])
            elif port.role == "consumed" and port.port_kind == "request":
                frame = self._frame_type(port)
                lines.extend([
                    f"      /* {port.port_key}_transitions = */",
                    f"      [this](const {frame}& frame) {{",
                    f"        {self._sink_method(port)}(frame);",
                    "      },",
                ])
        lines.extend([
            "  };",
            "}",
            "",
            "// -----------------------------------------------------------------",
            "// Your component logic. Everything below is a TODO to implement.",
            "// -----------------------------------------------------------------",
            "",
            f"pcl_status_t {component_class}::on_user_configure() {{",
            "  // TODO: one-time initialisation. Every port is bound at this",
            "  // point; cache writers or start timers here.",
            "  return PCL_OK;",
            "}",
            "",
            f"pcl_status_t {component_class}::on_tick(double dt) {{",
            "  (void)dt;",
            "  // TODO: implement periodic component work. Publish on output",
            "  // ports through the port accessors declared in the header.",
            "  return PCL_OK;",
            "}",
            "",
        ])
        for port in self._sink_ports():
            frame = self._frame_type(port)
            comment = (
                "handle request-port transitions."
                if port.port_kind == "request"
                else "handle incoming information."
            )
            lines.extend([
                f"void {component_class}::{self._sink_method(port)}(",
                f"    const {frame}& frame) {{",
                "  (void)frame;",
                f"  // TODO: {comment}",
                "}",
                "",
            ])
        for port in self._provider_ports():
            handler = self._handler_class(port)
            for rpc in self._command_rpcs(port):
                request = _qualified(port.role, _cpp_req_type(rpc))
                response = _qualified(port.role, _cpp_rsp_type(rpc))
                lines.extend([
                    f"{response} {handler}::on{rpc.name}(",
                    f"    const {request}& request) {{",
                    "  (void)request;",
                    "  // TODO: implement port business logic. Reach writers",
                    "  // through component_, for example",
                    "  // component_.<name>Port().publish(...).",
                    "  return {};",
                    "}",
                    "",
                ])
        lines.extend([f"}}  // namespace {ns}", ""])
        return "\n".join(lines)

    def _scaffold_main(self, component_file: str) -> str:
        include_root = (
            f"{self.group.project}_{self.group.component.replace('.', '_')}"
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
            "    // The port-config file selects the transport and the codec.",
            "    // PYRAMID_COMPONENT_CODEC_PLUGIN_PATH is a build-time fallback",
            "    // used when the config names no codec.",
            "    pcl::ProcessRuntime runtime(",
            "        argc, argv, {PYRAMID_COMPONENT_CODEC_PLUGIN_PATH},",
            f"        {self.namespace}::{self._scaffold_class()}::deploymentPorts());",
            "    // Each port encodes with the codec its `codec`/`port_codec`",
            "    // line selected; ports without an override use the default.",
            f"    {self.namespace}::{self._scaffold_class()} component(",
            "        runtime.executor(),",
            "        [&runtime](const std::string& port) {",
            "          return runtime.contentTypeFor(port);",
            "        });",
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
            bindings.append(self.naming_policy.service_file_prefix(
                self.group.provided_package).removeprefix("pyramid_services_"))
        if self.group.consumed_package:
            bindings.append(self.naming_policy.service_file_prefix(
                self.group.consumed_package).removeprefix("pyramid_services_"))
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
            f"target_include_directories({target} PRIVATE include src)",
            f"target_compile_features({target} PRIVATE cxx_std_17)",
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

    def _scaffold_readme(self, component_file: str) -> str:
        component = self.group.component.replace(".", "_")
        target = f"{self.group.project}_{component}"
        return "\n".join([
            f"# {self.group.key} component skeleton",
            "",
            "This is a one-time C++ application scaffold generated from the",
            f"`{self.group.key}` contract. The generated skeleton base under",
            "`sdk_project/generated/` owns and binds the ports; the files here",
            "are the application layer.",
            "",
            "## Where to write your code",
            "",
            f"Everything you implement lives in one file: `src/{component_file}.cpp`.",
            "It contains, below a short generated wiring section:",
            "",
            "- `on_user_configure()` -- one-time setup, run after every port is",
            "  bound.",
            "- `on_tick(double dt)` -- periodic work.",
            "- one method per provided request-port operation and per consumed",
            "  information sink -- your business logic.",
            "",
            f"The matching header `include/{target}/{component_file}.hpp` holds the",
            "generated wiring: the component builds and binds its own handler set,",
            "so `main.cpp` only constructs and runs it. The component re-exposes the",
            "skeleton's port accessors (the output-port writers and consumed-service",
            "handles) with `using`-declarations, so both `on_tick` and the port",
            "handlers can publish and call other ports.",
            "",
            "## Build",
            "",
            "From a Visual Studio 2022 x64 developer command prompt, from this",
            "directory:",
            "",
            "```bat",
            "cmake -S . -B build -DPYRAMID_SDK=<path-to-sdk>",
            "cmake --build build --config Release --parallel",
            "```",
            "",
            "## Run",
            "",
            "Copy one of the files under `configs/`, replace every `<...>`",
            "placeholder, and pass the edited `.ports` file as the first argument.",
            "The `.ports` file selects the transport per port (`port` lines) and",
            "the wire codec: a `codec` line sets the process default, and a",
            "`port_codec PORT CONTENT_TYPE` line overrides one port so a process",
            "can speak several codecs. Add `--duration-seconds=N` for a bounded run.",
            "",
        ])

    def _ports_template(
        self, platform: str, transport: str, plugin_suffix: str
    ) -> str:
        del platform
        if transport == "tcp":
            plugin = f"plugins/pcl_transport_socket_plugin{plugin_suffix}"
            config = '{"role":"<provided-or-consumed>","host":"127.0.0.1","port":<port>}'
        else:
            plugin = f"plugins/pcl_transport_shared_memory_plugin{plugin_suffix}"
            config = (
                '{"bus_name":"<shared-bus>",'
                f'"participant_id":"{self.group.component}"' + "}"
            )
        component = self.group.component.replace(".", "_")
        codec_plugin = (
            f"plugins/pyramid_codec_json_{self.group.project}_{component}"
            f"{plugin_suffix}"
        )
        emitted = self._emitted_ports()
        example_port = emitted[0].port_key if emitted else "<port>"
        lines = [
            "# One line configures one generated logical port.",
            "# codec CONTENT_TYPE PLUGIN loads a wire codec; the first codec",
            "#   line is the process-wide default.",
            "# port_codec PORT CONTENT_TYPE overrides the codec for one port,",
            "#   so a process can speak several codecs.",
            "# port NAME MODE PEER PLUGIN PLUGIN_CONFIG configures one port.",
            "# EDIT: replace every <...> placeholder before running.",
            f"codec application/json {codec_plugin}",
            "# To give one port its own codec, load it with another `codec`",
            "# line and then, for example:",
            f"# port_codec {example_port} application/x-your-codec",
        ]
        for port in emitted:
            mode = "rpc" if port.port_kind == "request" else "pubsub"
            lines.append(
                f"port {port.port_key} {mode} <peer-process> {plugin} {config}"
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
