/**
 * @file ame_py.cpp
 * @brief pybind11 Python bindings for ame_core library.
 *
 * Exposes WorldModel, Planner, PlanCompiler, and PDDL parsing to Python,
 * enabling the devenv to run without ROS2.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eval.h>

#include <ame/world_model.h>
#include <ame/planner.h>
#include <ame/plan_compiler.h>
#include <ame/action_registry.h>
#include <ame/pddl_parser.h>
#include <ame/goal_allocator.h>
#include <ame/executor_component.h>
#include <ame/autonomy_backend.h>
#include <ame/current_ame_backend_adapter.h>
#include <ame/execution_sink.h>

#if defined(AME_BUILD_AGRA_MA_BRIDGE)
#include <cctype>
#include <chrono>
#include <map>
#include <memory>
#include <unordered_map>
#include <ame/agra_ma_bridge.h>
#include <ame/agra_action_dispatch_component.hpp>
#include <ame/agra_ma_component.hpp>
#include <ame/agra_ma_grounding_component.hpp>
#include <pyramid_data_model_agra_codec.hpp>
#include <pyramid_data_model_agra_cabi_marshal.hpp>
#include <pyramid_components_agra_ma_grounding_services_provided_types.hpp>
#include <pyramid_services_agra_c2_consumed_components.hpp>
#include <pyramid_services_agra_c2_provided_components.hpp>
#include <pyramid_services_agra_ma_grounding_provided_components.hpp>
#include <ame/agra_native_json_uuid.h>
#include <ame/agra_oms_uuid.h>
#include <pcl/await.hpp>
#include <pcl/process_runtime.hpp>

extern "C" {
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
}
#endif

namespace py = pybind11;

#if defined(AME_BUILD_AGRA_MA_BRIDGE)
namespace {

namespace agra_codec = pyramid::domain_model::agra;
namespace c2 = pyramid::components::agra::c2::services::provided;
namespace c2_consumed =
    pyramid::components::agra::c2::services::consumed;
namespace grounding_service =
    pyramid::components::agra_ma_grounding::services::provided;

std::string protobufToNativeJson(
    const py::bytes& payload,
    const std::string& message_type) {
    const auto module = py::module_::import("_ame_py");
    return module.attr("_agra_protobuf_to_native_json")(
        payload, message_type).cast<std::string>();
}

py::bytes nativeJsonToProtobuf(
    const std::string& payload,
    const std::string& message_type) {
    const auto module = py::module_::import("_ame_py");
    return module.attr("_agra_native_json_to_protobuf")(
        payload, message_type).cast<py::bytes>();
}

void installAgraProtobufCodec(py::module_& module) {
    py::dict scope;
    scope["__name__"] = "_ame_py._agra_protobuf_codec";
    py::exec(R"PY(
import json as _agra_json
from google.protobuf.json_format import MessageToDict as _agra_to_dict
from google.protobuf.json_format import ParseDict as _agra_parse_dict


def _agra_message_class(message_type):
    from automtk.agra.generated import agra_pb2
    message_class = getattr(agra_pb2, message_type, None)
    if message_class is None:
        raise ValueError(
            "unknown A-GRA protobuf message type: " + message_type
        )
    return message_class


def _agra_field_to_native(value, field):
    if field.message_type is None:
        return value
    if field.is_repeated:
        return [
            _agra_message_to_native(item, field.message_type)
            for item in value
        ]
    return _agra_message_to_native(value, field.message_type)


def _agra_message_to_native(value, descriptor):
    result = {}
    fields = descriptor.fields_by_name
    base_field = fields.get("base")
    if (
        base_field is not None
        and base_field.number == 1
        and base_field.message_type is not None
        and "base" in value
    ):
        for name, item in value["base"].items():
            inherited_field = base_field.message_type.fields_by_name[name]
            result[name] = _agra_field_to_native(item, inherited_field)
    for name, item in value.items():
        if name == "base" and base_field is not None:
            continue
        result[name] = _agra_field_to_native(item, fields[name])
    return result


def _agra_field_to_protobuf(value, field):
    if field.message_type is None:
        return value
    if field.is_repeated:
        return [
            _agra_message_to_protobuf(item, field.message_type)
            for item in value
        ]
    return _agra_message_to_protobuf(value, field.message_type)


def _agra_message_to_protobuf(value, descriptor):
    result = {}
    fields = descriptor.fields_by_name
    base_field = fields.get("base")
    inherited_names = set()
    if (
        base_field is not None
        and base_field.number == 1
        and base_field.message_type is not None
    ):
        base_value = {}
        for inherited_field in base_field.message_type.fields:
            name = inherited_field.name
            inherited_names.add(name)
            if name in value:
                base_value[name] = _agra_field_to_protobuf(
                    value[name], inherited_field
                )
        if base_value:
            result["base"] = base_value
    for name, item in value.items():
        if name in inherited_names:
            continue
        result[name] = _agra_field_to_protobuf(item, fields[name])
    return result


def _agra_protobuf_to_native_json(payload, message_type):
    message_class = _agra_message_class(message_type)
    message = message_class()
    message.ParseFromString(payload)
    proto_value = _agra_to_dict(
        message,
        preserving_proto_field_name=True,
    )
    native_value = _agra_message_to_native(
        proto_value,
        message.DESCRIPTOR,
    )
    return _agra_json.dumps(
        native_value,
        separators=(",", ":"),
        sort_keys=True,
    )


def _agra_native_json_to_protobuf(payload, message_type):
    message_class = _agra_message_class(message_type)
    message = message_class()
    native_value = _agra_json.loads(payload)
    proto_value = _agra_message_to_protobuf(
        native_value,
        message.DESCRIPTOR,
    )
    _agra_parse_dict(proto_value, message)
    return message.SerializeToString(deterministic=True)
)PY", scope);
    module.attr("_agra_protobuf_to_native_json") =
        scope["_agra_protobuf_to_native_json"];
    module.attr("_agra_native_json_to_protobuf") =
        scope["_agra_native_json_to_protobuf"];
}

void decodeUuid(std::string& value) {
    value = ame::nativeUuidFromNativeJson(value);
}

void encodeUuid(std::string& value) {
    value = ame::nativeJsonUuidFromNative(value);
}

class PyGroundingProviderComponent final : public pcl::Component {
public:
    PyGroundingProviderComponent(
        pcl::Executor& executor,
        ame::AgraMaBridge& bridge)
        : pcl::Component("automtk_py_grounding_provider"),
          handler_(bridge),
          provided_(*this, executor, handler_) {}

    pcl_status_t configureTransport(const std::string& config_json) {
        return provided_.configureTransport(config_json);
    }

protected:
    pcl_status_t on_configure() override {
        return provided_.bind();
    }

private:
    ame::GroundingServiceHandler handler_;
    grounding_service::ProvidedService provided_;
};

class PyGroundingPortClient {
public:
    explicit PyGroundingPortClient(
        const std::string& ports_file,
        ame::AgraMaBridge* local_bridge)
        : runtime_(createRuntime(ports_file, local_bridge)),
          owned_executor_(
              runtime_ ? nullptr : std::make_unique<pcl::Executor>()),
          executor_(
              runtime_ ? &runtime_->executor() : owned_executor_.get()),
          host_("automtk_py_grounding_consumer"),
          consumed_(
              host_,
              *executor_,
              runtime_
                  ? runtime_->contentTypeFor("ma_grounding")
                  : grounding_service::kJsonContentType) {
        configured_ = runtime_ != nullptr;
        if (local_bridge != nullptr) {
            loadLocalCodec();
            provider_ = std::make_unique<PyGroundingProviderComponent>(
                *executor_, *local_bridge);
            requireStatus(
                provider_->configure(), "configure local grounding provider");
            requireStatus(
                provider_->activate(), "activate local grounding provider");
            requireStatus(
                executor_->add(*provider_), "add local grounding provider");
            provider_active_ = true;
            requireStatus(
                provider_->configureTransport(R"({"transport":"local"})"),
                "route local grounding provider");
        }
    }

    ~PyGroundingPortClient() {
        if (provider_active_) {
            executor_->remove(*provider_);
            provider_->deactivate();
            provider_->cleanup();
        }
        if (local_codec_handle_ != nullptr) {
            pcl_plugin_unload(local_codec_handle_);
        }
    }

    PyGroundingPortClient(const PyGroundingPortClient&) = delete;
    PyGroundingPortClient& operator=(const PyGroundingPortClient&) = delete;

    void configureTransport(const std::string& config_json) {
        requireStatus(
            consumed_.configureTransport(config_json),
            "configure grounding transport");
        configured_ = true;
    }

    std::string createTaskGrounding(
        const std::string& task_payload,
        const std::vector<std::string>& goal_fluents,
        unsigned timeout_ms) {
        requireConfigured();
        grounding_service::TaskGrounding request;
        request.task_payload = task_payload;
        request.goal_fluents = goal_fluents;
        return awaitResult(
            consumed_.groundingCreateTaskGroundingAsync(request),
            timeout_ms,
            "CreateTaskGrounding");
    }

    std::string createPlanGrounding(
        const std::string& mission_plan_command_uuid,
        const py::iterable& grounding_items,
        unsigned timeout_ms) {
        requireConfigured();
        grounding_service::PlanGrounding request;
        request.mission_plan_command_uuid = mission_plan_command_uuid;
        for (const auto item : grounding_items) {
            const auto data = py::reinterpret_borrow<py::dict>(item);
            grounding_service::ActionGrounding action;
            action.action_signature =
                data["action_signature"].cast<std::string>();
            action.action_payload =
                data["action_payload"].cast<std::string>();
            action.requires_kinematics =
                data["requires_kinematics"].cast<bool>();
            action.route_plan_id =
                data["route_plan_id"].cast<std::string>();
            for (const auto waypoint_item :
                 data["waypoints"].cast<py::iterable>()) {
                const auto waypoint =
                    py::reinterpret_borrow<py::dict>(waypoint_item);
                grounding_service::Waypoint value;
                value.latitude_deg =
                    waypoint["latitude_deg"].cast<double>();
                value.longitude_deg =
                    waypoint["longitude_deg"].cast<double>();
                value.altitude_m =
                    waypoint["altitude_m"].cast<double>();
                action.waypoints.push_back(std::move(value));
            }
            const auto parameters =
                data["command_parameters"].cast<
                    std::unordered_map<std::string, std::string>>();
            for (const auto& [key, value] : parameters) {
                grounding_service::CommandParameter parameter;
                parameter.key = key;
                parameter.value = value;
                action.command_parameters.push_back(std::move(parameter));
            }
            request.grounding.push_back(std::move(action));
        }
        return awaitResult(
            consumed_.groundingCreatePlanGroundingAsync(request),
            timeout_ms,
            "CreatePlanGrounding");
    }

private:
    static std::unique_ptr<pcl::ProcessRuntime> createRuntime(
        const std::string& ports_file,
        const ame::AgraMaBridge* local_bridge) {
        if (ports_file.empty()) {
            if (local_bridge == nullptr) {
                throw std::invalid_argument(
                    "GroundingPortClient requires a ports file for remote "
                    "transport or a local_bridge for local transport");
            }
            return nullptr;
        }
        if (local_bridge != nullptr) {
            throw std::invalid_argument(
                "GroundingPortClient cannot combine a ports file with "
                "local_bridge");
        }
        std::string program_name = "_ame_py.GroundingPortClient";
        std::string config_path = ports_file;
        char* arguments[] = {
            program_name.data(),
            config_path.data(),
        };
        return std::make_unique<pcl::ProcessRuntime>(
            2,
            arguments,
            std::initializer_list<const char*>{},
            std::initializer_list<pcl::DeploymentPort>{
                {"ma_grounding",
                 grounding_service::ConsumedService::
                     deploymentDescriptor()}});
    }

    void loadLocalCodec() {
        requireStatus(
            pcl_plugin_load_codec(
                AGRA_MA_GROUNDING_JSON_CODEC_PLUGIN_PATH,
                nullptr,
                pcl_codec_registry_default(),
                &local_codec_handle_),
            "load grounding JSON codec");
    }

    static void requireStatus(
        pcl_status_t status,
        const std::string& operation) {
        if (status == PCL_OK) {
            return;
        }
        throw std::runtime_error(
            operation + " failed with PCL status " +
            std::to_string(static_cast<int>(status)));
    }

    void requireConfigured() const {
        if (!configured_) {
            throw std::runtime_error(
                "GroundingPortClient transport is not configured");
        }
    }

    std::string awaitResult(
        std::future<grounding_service::Result<
            grounding_service::Identifier>> future,
        unsigned timeout_ms,
        const char* operation) {
        if (!pcl::await(
                *executor_,
                future,
                std::chrono::milliseconds(timeout_ms))) {
            throw std::runtime_error(
                std::string(operation) + " timed out after " +
                std::to_string(timeout_ms) + " ms");
        }
        auto result = future.get();
        requireStatus(result.status, operation);
        if (result.value.empty()) {
            throw std::runtime_error(
                std::string(operation) +
                " returned an empty grounding identifier");
        }
        return result.value;
    }

    std::unique_ptr<pcl::ProcessRuntime> runtime_;
    std::unique_ptr<pcl::Executor> owned_executor_;
    pcl::Executor* executor_;
    pcl::Component host_;
    grounding_service::ConsumedService consumed_;
    std::unique_ptr<PyGroundingProviderComponent> provider_;
    pcl_plugin_handle_t* local_codec_handle_ = nullptr;
    bool provider_active_ = false;
    bool configured_ = false;
};

// ---------------------------------------------------------------------------
// OMS-JSON bridge for Python hosts.
//
// The bridge process is Python and holds agra.* protobuf messages; the LA-CAL
// transport accepts only application/oms-json. Rather than write a second
// OMS-JSON encoder in Python -- the duplication P3.3 declined -- this exposes
// the generated C++ one, which is the same codec the C++ composite uses.
//
// The chain, and why each step exists:
//   protobuf bytes
//     -> protobufToNativeJson   (existing; emits protobuf-JSON, uuids base64)
//     -> agra_codec::fromJson   (generated json codec -> C++ type)
//     -> decodeUuid             (base64 text -> raw 16 bytes, AME's form)
//     -> omsUuidFromNative      (raw 16 bytes -> 32 hex, the OWP form)
//     -> pyramid::cabi::to_c    (C++ type -> C ABI struct)
//     -> codec->encode          (C ABI struct -> OMS-JSON)
//
// The two UUID steps are not redundant: three representations are in play, and
// conflating any pair of them is exactly the class of bug that produced
// "length 32 != 16" against a conformance server.
struct OmsJsonPlugin {
    pcl_codec_registry_t* registry = nullptr;
    pcl_plugin_handle_t* handle = nullptr;
    const pcl_codec_t* codec = nullptr;

    explicit OmsJsonPlugin(const std::string& path) {
        registry = pcl_codec_registry_create();
        if (registry == nullptr) {
            throw std::runtime_error("could not create a PCL codec registry");
        }
        if (pcl_plugin_load_codec(path.c_str(), nullptr, registry, &handle) !=
            PCL_OK) {
            pcl_codec_registry_destroy(registry);
            registry = nullptr;
            throw std::runtime_error(
                "could not load the OMS-JSON codec plugin: " + path);
        }
        codec = pcl_codec_registry_get(registry, "application/oms-json");
        if (codec == nullptr) {
            throw std::runtime_error(
                "plugin does not provide application/oms-json: " + path);
        }
    }

    ~OmsJsonPlugin() {
        if (handle != nullptr) pcl_plugin_unload(handle);
        if (registry != nullptr) pcl_codec_registry_destroy(registry);
    }

    OmsJsonPlugin(const OmsJsonPlugin&) = delete;
    OmsJsonPlugin& operator=(const OmsJsonPlugin&) = delete;
};

// Loading a plugin per call would be wasteful and would churn dlopen handles;
// one per path is enough, and the map outlives the interpreter's use of it.
OmsJsonPlugin& omsJsonPlugin(const std::string& path) {
    static std::map<std::string, std::unique_ptr<OmsJsonPlugin>> cache;
    auto found = cache.find(path);
    if (found == cache.end()) {
        found = cache.emplace(path, std::make_unique<OmsJsonPlugin>(path))
                    .first;
    }
    return *found->second;
}

void decodeHeader(ame::agra::HeaderType& header) {
    decodeUuid(header.system_id.uuid);
    if (header.mission_id.has_value()) {
        decodeUuid(header.mission_id->base.uuid);
    }
}

void encodeHeader(ame::agra::HeaderType& header) {
    encodeUuid(header.system_id.uuid);
    if (header.mission_id.has_value()) {
        encodeUuid(header.mission_id->base.uuid);
    }
}

// Walk the header and the message-specific ids from AME's raw-16-byte form
// into the 32-hex OWP form, or back. Kept explicit per root rather than
// generic: a missed field would silently ship an identifier the far side
// cannot correlate, which is worse than a compile error.
void headerToOms(ame::agra::HeaderType& header) {
    header.system_id.uuid = ame::omsUuidFromNative(header.system_id.uuid);
    if (header.mission_id.has_value()) {
        header.mission_id->base.uuid =
            ame::omsUuidFromNative(header.mission_id->base.uuid);
    }
}

void headerFromOms(ame::agra::HeaderType& header) {
    header.system_id.uuid = ame::nativeUuidFromOms(header.system_id.uuid);
    if (header.mission_id.has_value()) {
        header.mission_id->base.uuid =
            ame::nativeUuidFromOms(header.mission_id->base.uuid);
    }
}

template <typename Message, typename CValue>
py::bytes encodeOmsJson(const pcl_codec_t* codec, const char* root,
                        Message& message,
                        void (*free_value)(CValue*)) {
    CValue value{};
    pyramid::cabi::to_c(message, &value);
    pcl_msg_t encoded{};
    const pcl_status_t status =
        codec->encode(codec->codec_ctx, root, &value, &encoded);
    free_value(&value);
    if (status != PCL_OK) {
        throw std::runtime_error(
            std::string("OMS-JSON encode refused ") + root + " (status " +
            std::to_string(static_cast<int>(status)) +
            "); the message is not encodable as it stands");
    }
    py::bytes out(static_cast<const char*>(encoded.data), encoded.size);
    codec->free_msg(codec->codec_ctx, &encoded);
    return out;
}

ame::agra::MA_TaskMT decodeTask(const py::bytes& payload) {
    auto task = agra_codec::fromJson(
        protobufToNativeJson(payload, "MA_TaskMT"),
        static_cast<ame::agra::MA_TaskMT*>(nullptr));
    decodeHeader(task.message_header);
    decodeUuid(task.message_data.task_id.base.uuid);
    return task;
}

ame::agra::MA_ActionMT decodeAction(const py::bytes& payload) {
    auto action = agra_codec::fromJson(
        protobufToNativeJson(payload, "MA_ActionMT"),
        static_cast<ame::agra::MA_ActionMT*>(nullptr));
    decodeHeader(action.message_header);
    decodeUuid(action.message_data.action_id.base.uuid);
    return action;
}

ame::agra::MA_MissionPlanCommandMT decodeMissionPlanCommand(
    const py::bytes& payload) {
    auto command = agra_codec::fromJson(
        protobufToNativeJson(payload, "MA_MissionPlanCommandMT"),
        static_cast<ame::agra::MA_MissionPlanCommandMT*>(nullptr));
    decodeHeader(command.message_header);
    decodeUuid(command.message_data.command_id.uuid);
    decodeUuid(
        command.message_data.inputs.planning_process_id.uuid);
    if (command.message_data.inputs.proposed_requirements.has_value()) {
        for (auto& task :
             command.message_data.inputs.proposed_requirements
                 ->proposed_task) {
            decodeUuid(task.task_id.base.uuid);
        }
    }
    return command;
}

ame::agra::MA_TaskCommandMT decodeTaskCommand(
    const py::bytes& payload) {
    auto command = agra_codec::fromJson(
        protobufToNativeJson(payload, "MA_TaskCommandMT"),
        static_cast<ame::agra::MA_TaskCommandMT*>(nullptr));
    decodeHeader(command.message_header);
    for (auto& item : command.message_data.command) {
        decodeUuid(item.capability.base.command_id.uuid);
        decodeUuid(item.capability.capability_id.uuid);
        decodeUuid(item.capability.task_id.base.uuid);
    }
    return command;
}

ame::agra::MA_MissionPlanActivationCommandMT
decodeMissionPlanActivationCommand(const py::bytes& payload) {
    auto command = agra_codec::fromJson(
        protobufToNativeJson(
            payload, "MA_MissionPlanActivationCommandMT"),
        static_cast<
            ame::agra::MA_MissionPlanActivationCommandMT*>(nullptr));
    decodeHeader(command.message_header);
    decodeUuid(command.message_data.command_id.uuid);
    for (auto& item : command.message_data.command) {
        decodeUuid(item.mission_plan_id.base.uuid);
    }
    return command;
}

ame::agra::MA_ApprovalRequestStatusMT decodeApprovalStatus(
    const py::bytes& payload) {
    auto status = agra_codec::fromJson(
        protobufToNativeJson(payload, "MA_ApprovalRequestStatusMT"),
        static_cast<ame::agra::MA_ApprovalRequestStatusMT*>(nullptr));
    decodeHeader(status.message_header);
    decodeUuid(status.message_data.request_id.uuid);
    return status;
}

void encodeSystem(ame::agra::SystemID_Type& system) {
    encodeUuid(system.uuid);
}

void encodeMissionPlanId(ame::agra::MissionPlanID_Type& plan_id) {
    encodeUuid(plan_id.base.uuid);
}

void encodeTaskCommandStatus(
    ame::agra::MA_TaskCommandStatusMT& status) {
    encodeHeader(status.message_header);
    encodeUuid(status.message_data.base.command_id.uuid);
}

void encodeMissionPlanCommandStatus(
    ame::agra::MA_MissionPlanCommandStatusMT& status) {
    encodeHeader(status.message_header);
    encodeUuid(status.message_data.command_id.uuid);
    if (status.message_data.resulting_plan_identifier.has_value()) {
        for (auto& plan_id :
             status.message_data.resulting_plan_identifier
                 ->mission_plan_id) {
            encodeMissionPlanId(plan_id);
        }
    }
}

void encodeActivationCommandStatus(
    ame::agra::MA_MissionPlanActivationCommandStatusMT& status) {
    encodeHeader(status.message_header);
    encodeUuid(status.message_data.command_id.uuid);
}

void encodeMissionPlan(ame::agra::MA_MissionPlanMT& plan) {
    encodeHeader(plan.message_header);
    encodeMissionPlanId(plan.message_data.mission_plan_id);
    if (plan.message_data.mission_plan_command_id.has_value() &&
        plan.message_data.mission_plan_command_id
            ->mission_plan_command_id.has_value()) {
        encodeUuid(
            plan.message_data.mission_plan_command_id
                ->mission_plan_command_id->uuid);
    }
    if (plan.message_data.applicability.planned_for_id.has_value()) {
        encodeSystem(
            plan.message_data.applicability.planned_for_id.value());
    }
    if (plan.message_data.applicability.applicable_to_i_ds
            .system_id.has_value()) {
        encodeSystem(
            plan.message_data.applicability.applicable_to_i_ds
                .system_id.value());
    }
    if (plan.message_data.sub_plans.has_value()) {
        for (auto& action_plan_id :
             plan.message_data.sub_plans->action_plan_id) {
            encodeUuid(action_plan_id.base.uuid);
        }
        for (auto& route_plan_id :
             plan.message_data.sub_plans->route_plan_id) {
            encodeUuid(route_plan_id.base.uuid);
        }
    }
    if (!plan.message_data.execution_sequence.has_value()) {
        return;
    }
    auto& sequence = plan.message_data.execution_sequence.value();
    encodeUuid(sequence.initial_execution_plan_set_id.uuid);
    for (auto& plan_set : sequence.route_execution_plan_set) {
        encodeUuid(plan_set.execution_plan_set_id.uuid);
        encodeUuid(plan_set.route_plan_id.base.uuid);
        if (plan_set.next_execution_plan_set_id.has_value()) {
            encodeUuid(plan_set.next_execution_plan_set_id->uuid);
        }
    }
}

void encodeApprovalRequest(ame::agra::MA_ApprovalRequestMT& request) {
    encodeHeader(request.message_header);
    encodeUuid(request.message_data.request_id.uuid);
    if (request.message_data.approver.non_operator_identifier
            .has_value()) {
        encodeSystem(
            request.message_data.approver.non_operator_identifier
                ->system_id);
    }
    encodeUuid(
        request.message_data.approval_references
            .approval_policy_id.uuid);
    if (request.message_data.approval_references.approval_item
            .plan_approval.has_value() &&
        request.message_data.approval_references.approval_item
            .plan_approval->mission_plan_id.has_value()) {
        encodeMissionPlanId(
            request.message_data.approval_references.approval_item
                .plan_approval->mission_plan_id.value());
    }
}

void encodeExecutionStatus(
    ame::agra::MA_MissionPlanExecutionStatusMT& status) {
    encodeHeader(status.message_header);
    encodeSystem(status.message_data.system_id);
    for (auto& plan_status :
         status.message_data.plan_execution_status) {
        encodeMissionPlanId(plan_status.mission_plan_id);
        for (auto& plan_set :
             plan_status.execution_plan_set_status) {
            encodeUuid(plan_set.execution_plan_set_id.uuid);
        }
    }
}

void encodeContingencyAlert(
    ame::agra::MissionContingencyAlertMT& alert) {
    encodeHeader(alert.message_header);
    encodeUuid(
        alert.message_data.mission_contingency_alert_id.uuid);
    encodeSystem(alert.message_data.source_system_id);
    for (auto& condition :
         alert.message_data.contingency_condition) {
        encodeSystem(condition.conflicted_system_id);
    }
}

template <typename Information, typename Message, typename Encoder>
std::vector<py::bytes> encodeDrain(
    std::vector<Information> information,
    tl::optional<Message> Information::* member,
    const std::string& message_type,
    Encoder encoder) {
    std::vector<py::bytes> payloads;
    payloads.reserve(information.size());
    for (auto& item : information) {
        if (!(item.*member).has_value()) {
            throw std::runtime_error(
                message_type + " egress is missing its payload");
        }
        auto message = (item.*member).value();
        encoder(message);
        payloads.push_back(nativeJsonToProtobuf(
            agra_codec::toJson(message), message_type));
    }
    return payloads;
}

class PyLacalAgraMaCompositeClient final
    : public pcl::Component,
      private c2::MaActioncommandStatusPortHandler {
public:
    explicit PyLacalAgraMaCompositeClient(
        const std::string& ports_file)
        : pcl::Component("automtk_py_agra_composite_client"),
          runtime_(createRuntime(ports_file)),
          grounding_(
              *this,
              runtime_->executor(),
              runtime_->contentTypeFor("ma_grounding")),
          task_port_(
              *this, runtime_->executor(), runtime_->contentType()),
          mission_plan_command_port_(
              *this, runtime_->executor(), runtime_->contentType()),
          activation_port_(
              *this, runtime_->executor(), runtime_->contentType()),
          action_port_(
              *this, runtime_->executor(), *this,
              runtime_->contentType()),
          mission_plan_port_(
              *this, runtime_->executor(), runtime_->contentType()),
          execution_status_port_(
              *this, runtime_->executor(), runtime_->contentType()),
          contingency_alert_port_(
              *this, runtime_->executor(), runtime_->contentType()) {
        if (ports_file.empty()) {
            throw std::invalid_argument(
                "LacalAgraMaCompositeClient requires a ports file");
        }
        requireStatus(configure(), "configure lacal composite client");
        requireStatus(activate(), "activate lacal composite client");
        requireStatus(
            runtime_->executor().add(*this),
            "add lacal composite client");
        active_ = true;
    }

    ~PyLacalAgraMaCompositeClient() {
        if (active_) {
            runtime_->executor().remove(*this);
            deactivate();
            cleanup();
        }
    }

    PyLacalAgraMaCompositeClient(
        const PyLacalAgraMaCompositeClient&) = delete;
    PyLacalAgraMaCompositeClient& operator=(
        const PyLacalAgraMaCompositeClient&) = delete;

    std::string createTaskGrounding(
        const std::string& task_payload,
        const std::vector<std::string>& goal_fluents,
        const std::string& domain_pddl,
        const std::string& problem_pddl,
        const py::iterable& action_bindings,
        const std::vector<std::string>& confirmed_fluents,
        unsigned timeout_ms) {
        grounding_service::TaskGrounding request;
        request.task_payload = task_payload;
        request.goal_fluents = goal_fluents;
        request.domain_pddl = domain_pddl;
        request.problem_pddl = problem_pddl;
        request.confirmed_fluents = confirmed_fluents;
        for (const auto item : action_bindings) {
            const auto data =
                py::reinterpret_borrow<py::dict>(item);
            grounding_service::ActionBinding binding;
            binding.pddl_name =
                data["pddl_name"].cast<std::string>();
            binding.binding_kind =
                data["binding_kind"].cast<std::string>();
            binding.implementation =
                data["implementation"].cast<std::string>();
            if (data.contains("reactive")) {
                binding.reactive =
                    data["reactive"].cast<bool>();
            }
            request.action_bindings.push_back(
                std::move(binding));
        }
        return awaitGrounding(
            grounding_.groundingCreateTaskGroundingAsync(request),
            timeout_ms,
            "CreateTaskGrounding");
    }

    std::string createPlanGrounding(
        const std::string& mission_plan_command_uuid,
        const py::iterable& grounding_items,
        unsigned timeout_ms) {
        grounding_service::PlanGrounding request;
        request.mission_plan_command_uuid =
            mission_plan_command_uuid;
        for (const auto item : grounding_items) {
            const auto data =
                py::reinterpret_borrow<py::dict>(item);
            grounding_service::ActionGrounding action;
            action.action_signature =
                data["action_signature"].cast<std::string>();
            action.action_payload =
                data["action_payload"].cast<std::string>();
            action.requires_kinematics =
                data["requires_kinematics"].cast<bool>();
            action.route_plan_id =
                data["route_plan_id"].cast<std::string>();
            for (const auto waypoint_item :
                 data["waypoints"].cast<py::iterable>()) {
                const auto waypoint =
                    py::reinterpret_borrow<py::dict>(
                        waypoint_item);
                grounding_service::Waypoint value;
                value.latitude_deg =
                    waypoint["latitude_deg"].cast<double>();
                value.longitude_deg =
                    waypoint["longitude_deg"].cast<double>();
                value.altitude_m =
                    waypoint["altitude_m"].cast<double>();
                action.waypoints.push_back(std::move(value));
            }
            const auto parameters =
                data["command_parameters"].cast<
                    std::unordered_map<
                        std::string, std::string>>();
            for (const auto& [key, value] : parameters) {
                grounding_service::CommandParameter parameter;
                parameter.key = key;
                parameter.value = value;
                action.command_parameters.push_back(
                    std::move(parameter));
            }
            request.grounding.push_back(std::move(action));
        }
        return awaitGrounding(
            grounding_.groundingCreatePlanGroundingAsync(request),
            timeout_ms,
            "CreatePlanGrounding");
    }

    void submitTaskCommand(
        const py::bytes& payload, unsigned timeout_ms) {
        c2::MA_TaskCommand_Service_Information information;
        information.ma_task_command = decodeTaskCommand(payload);
        const auto previous = task_statuses_.size();
        requireStatus(
            task_port_.submit(
                ame::detail::agraTaskCommandNativeToWire(
                    std::move(information))),
            "publish MA_TaskCommand");
        waitFor(
            [this, previous]() {
                return task_statuses_.size() > previous;
            },
            timeout_ms,
            "MA_TaskCommandStatus");
    }

    void submitMissionPlanCommand(
        const py::bytes& payload, unsigned timeout_ms) {
        c2::MA_MissionPlanCommand_Service_Information information;
        information.ma_mission_plan_command =
            decodeMissionPlanCommand(payload);
        const auto previous =
            mission_plan_command_statuses_.size();
        const auto previous_plans = mission_plans_.size();
        requireStatus(
            mission_plan_command_port_.submit(
                ame::detail::
                    agraMissionPlanCommandNativeToWire(
                        std::move(information))),
            "publish MA_MissionPlanCommand");
        waitFor(
            [this, previous, previous_plans]() {
                if (mission_plan_command_statuses_.size() <=
                    previous) {
                    return false;
                }
                if (mission_plans_.size() > previous_plans) {
                    return true;
                }
                const auto& information =
                    mission_plan_command_statuses_.back();
                return information.ma_mission_plan_command_status
                           .has_value() &&
                    information.ma_mission_plan_command_status
                            ->message_data.planning_status
                            .command_processing_state ==
                        ame::agra::CommandProcessingStateEnum::
                            Rejected;
            },
            timeout_ms,
            "MA_MissionPlanCommandStatus");
    }

    void submitActivationCommand(
        const py::bytes& payload, unsigned timeout_ms) {
        c2::MA_MissionPlanActivationCommand_Service_Information
            information;
        information.ma_mission_plan_activation_command =
            decodeMissionPlanActivationCommand(payload);
        const auto previous = activation_statuses_.size();
        requireStatus(
            activation_port_.submit(
                ame::detail::
                    agraMissionPlanActivationCommandNativeToWire(
                        std::move(information))),
            "publish MA_MissionPlanActivationCommand");
        waitFor(
            [this, previous]() {
                return activation_statuses_.size() > previous;
            },
            timeout_ms,
            "MA_MissionPlanActivationCommandStatus");
    }

    void submitApprovalStatus(
        const py::bytes& payload, unsigned timeout_ms) {
        grounding_service::NativePayload request;
        request.payload = protobufToNativeJson(
            payload, "MA_ApprovalRequestStatusMT");
        awaitGrounding(
            grounding_.groundingCreateApprovalStatusAsync(request),
            timeout_ms,
            "CreateApprovalStatus");
    }

    std::vector<py::bytes> readTaskCommandStatuses() {
        spinOnce();
        auto values = drain(task_statuses_);
        for (auto& value : values) {
            value = ame::detail::
                agraTaskCommandStatusWireToNative(
                    std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::MA_TaskCommandStatus_Service_Information::
                ma_task_command_status,
            "MA_TaskCommandStatusMT",
            encodeTaskCommandStatus);
    }

    std::vector<py::bytes> readMissionPlanCommandStatuses() {
        spinOnce();
        auto values = drain(mission_plan_command_statuses_);
        for (auto& value : values) {
            value = ame::detail::
                agraMissionPlanCommandStatusWireToNative(
                    std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::
                MA_MissionPlanCommandStatus_Service_Information::
                    ma_mission_plan_command_status,
            "MA_MissionPlanCommandStatusMT",
            encodeMissionPlanCommandStatus);
    }

    std::vector<py::bytes> readActivationCommandStatuses() {
        spinOnce();
        auto values = drain(activation_statuses_);
        for (auto& value : values) {
            value = ame::detail::
                agraMissionPlanActivationCommandStatusWireToNative(
                    std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::
                MA_MissionPlanActivationCommandStatus_Service_Information::
                    ma_mission_plan_activation_command_status,
            "MA_MissionPlanActivationCommandStatusMT",
            encodeActivationCommandStatus);
    }

    std::vector<py::bytes> readMissionPlans() {
        spinOnce();
        auto values = drain(mission_plans_);
        for (auto& value : values) {
            value = ame::detail::agraMissionPlanWireToNative(
                std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::MA_MissionPlan_Service_Information::
                ma_mission_plan,
            "MA_MissionPlanMT",
            encodeMissionPlan);
    }

    std::vector<py::bytes> readApprovalRequests(
        unsigned timeout_ms) {
        auto future =
            grounding_.groundingReadApprovalRequestsAsync(
                grounding_service::Empty{});
        if (!pcl::await(
                runtime_->executor(), future,
                std::chrono::milliseconds(timeout_ms))) {
            throw std::runtime_error(
                "ReadApprovalRequests timed out after " +
                std::to_string(timeout_ms) + " ms");
        }
        auto result = future.get();
        requireStatus(result.status, "ReadApprovalRequests");
        std::vector<py::bytes> payloads;
        if (!result.value.payload.empty()) {
            payloads.push_back(nativeJsonToProtobuf(
                result.value.payload, "MA_ApprovalRequestMT"));
        }
        return payloads;
    }

    std::vector<py::bytes> readExecutionStatuses() {
        spinOnce();
        auto values = drain(execution_statuses_);
        for (auto& value : values) {
            value = ame::detail::agraExecutionStatusWireToNative(
                std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::
                MA_MissionPlanExecutionStatus_Service_Information::
                    ma_mission_plan_execution_status,
            "MA_MissionPlanExecutionStatusMT",
            encodeExecutionStatus);
    }

    std::vector<py::bytes> readContingencyAlerts() {
        spinOnce();
        auto values = drain(contingency_alerts_);
        for (auto& value : values) {
            value = ame::detail::agraContingencyAlertWireToNative(
                std::move(value));
        }
        return encodeDrain(
            std::move(values),
            &c2::MissionContingencyAlert_Service_Information::
                mission_contingency_alert,
            "MissionContingencyAlertMT",
            encodeContingencyAlert);
    }

    std::vector<py::dict> pullActionCommands() {
        spinOnce();
        std::vector<py::dict> result;
        for (auto& information : drain(action_commands_)) {
            if (!information.ma_action_command.has_value() ||
                information.ma_action_command->message_data.command
                        .size() != 1u ||
                !information.ma_action_command->message_data.command
                     .front()
                     .capability.has_value()) {
                throw std::runtime_error(
                    "MA_ActionCommand ingress is malformed");
            }
            auto message =
                information.ma_action_command.value();
            const auto& capability =
                message.message_data.command.front()
                    .capability.value();
            const std::string command_id =
                capability.base.command_id.descriptive_label;
            if (command_id.empty()) {
                throw std::runtime_error(
                    "MA_ActionCommand has no correlating "
                    "command_id label");
            }
            const std::string action_name =
                capability.capability_id.descriptive_label;
            const std::string signature =
                capability.action_id.base.descriptive_label;
            const auto open = signature.find('(');
            const std::string operation =
                open == std::string::npos
                    ? action_name
                    : signature.substr(0, open);
            py::dict request_fields;
            if (open != std::string::npos &&
                !signature.empty() &&
                signature.back() == ')') {
                const auto body = signature.substr(
                    open + 1u,
                    signature.size() - open - 2u);
                std::size_t start = 0;
                unsigned index = 0;
                while (start <= body.size()) {
                    const auto comma = body.find(',', start);
                    const auto end =
                        comma == std::string::npos
                            ? body.size()
                            : comma;
                    auto value = body.substr(start, end - start);
                    const auto first =
                        value.find_first_not_of(" \t");
                    const auto last =
                        value.find_last_not_of(" \t");
                    if (first != std::string::npos) {
                        value = value.substr(
                            first, last - first + 1u);
                        request_fields[
                            py::str("param" + std::to_string(index))] =
                            py::str(value);
                        if (index == 0u) {
                            request_fields["entity_id"] =
                                py::str(value);
                        }
                        ++index;
                    }
                    if (comma == std::string::npos) {
                        break;
                    }
                    start = comma + 1u;
                }
            }
            if (!pending_actions_.emplace(
                    command_id, std::move(message)).second) {
                throw std::runtime_error(
                    "duplicate MA_ActionCommand command_id " +
                    command_id);
            }
            py::dict command;
            command["command_id"] = command_id;
            command["action_name"] = action_name;
            command["operation"] = operation;
            command["signature"] = signature;
            command["request_fields"] =
                std::move(request_fields);
            result.push_back(std::move(command));
        }
        return result;
    }

    void pushActionResult(
        const std::string& command_id,
        const std::string& status) {
        const auto found = pending_actions_.find(command_id);
        if (found == pending_actions_.end()) {
            throw std::runtime_error(
                "MA_ActionCommandStatus references unknown "
                "command_id " +
                command_id);
        }
        const auto& command = found->second;
        const auto& capability =
            command.message_data.command.front()
                .capability.value();
        ame::agra::MA_ActionCommandStatusMT result;
        result.security_information =
            command.security_information;
        result.message_header = command.message_header;
        result.message_data.base.command_id =
            capability.base.command_id;
        result.message_data.base.command_processing_state =
            actionStatus(status);
        c2::MA_ActionCommandStatus_Service_Information
            information;
        information.ma_action_command_status =
            std::move(result);
        requireStatus(
            action_port_.publishStatus(information),
            "publish MA_ActionCommandStatus");
        pending_actions_.erase(found);
    }

    void step(unsigned timeout_ms) {
        checkError();
        const pcl_status_t status =
            runtime_->executor().spinOnce(timeout_ms);
        if (status != PCL_OK) {
            requireStatus(status, "spin lacal composite client");
        }
        checkError();
    }

protected:
    pcl_status_t on_configure() override {
        pcl_status_t status = task_port_.bind();
        if (status != PCL_OK) return status;
        status = mission_plan_command_port_.bind();
        if (status != PCL_OK) return status;
        status = activation_port_.bind();
        if (status != PCL_OK) return status;
        status = action_port_.bind();
        if (status != PCL_OK) return status;
        status = mission_plan_port_.bind();
        if (status != PCL_OK) return status;
        status = execution_status_port_.bind();
        if (status != PCL_OK) return status;
        status = contingency_alert_port_.bind();
        if (status != PCL_OK) return status;

        task_subscription_ = task_port_.transitions(
            [this](
                const c2::
                    MA_TaskCommandStatus_Service_Information&
                        value) {
                task_statuses_.push_back(value);
            });
        mission_plan_command_subscription_ =
            mission_plan_command_port_.transitions(
                [this](
                    const c2::
                        MA_MissionPlanCommandStatus_Service_Information&
                            value) {
                    mission_plan_command_statuses_.push_back(
                        value);
                });
        activation_subscription_ =
            activation_port_.transitions(
                [this](
                    const c2::
                        MA_MissionPlanActivationCommandStatus_Service_Information&
                            value) {
                    activation_statuses_.push_back(value);
                });
        mission_plan_subscription_ =
            mission_plan_port_.subscribe(
                [this](
                    const c2::
                        MA_MissionPlan_Service_Information&
                            value) {
                    mission_plans_.push_back(value);
                });
        execution_status_subscription_ =
            execution_status_port_.subscribe(
                [this](
                    const c2::
                        MA_MissionPlanExecutionStatus_Service_Information&
                            value) {
                    execution_statuses_.push_back(value);
                });
        contingency_alert_subscription_ =
            contingency_alert_port_.subscribe(
                [this](
                    const c2::
                        MissionContingencyAlert_Service_Information&
                            value) {
                    contingency_alerts_.push_back(value);
                });
        return task_subscription_ &&
                       mission_plan_command_subscription_ &&
                       activation_subscription_ &&
                       mission_plan_subscription_ &&
                       execution_status_subscription_ &&
                       contingency_alert_subscription_
                   ? PCL_OK
                   : PCL_ERR_STATE;
    }

    void onCommand(
        const c2::MA_ActionCommand_Service_Information&
            information) override {
        action_commands_.push_back(information);
    }

private:
    static std::unique_ptr<pcl::ProcessRuntime> createRuntime(
        const std::string& ports_file) {
        if (ports_file.empty()) {
            throw std::invalid_argument(
                "LacalAgraMaCompositeClient requires a ports file");
        }
        std::string program_name =
            "_ame_py.LacalAgraMaCompositeClient";
        std::string config_path = ports_file;
        char* arguments[] = {
            program_name.data(),
            config_path.data(),
        };
        return std::make_unique<pcl::ProcessRuntime>(
            2,
            arguments,
            std::initializer_list<const char*>{},
            std::initializer_list<pcl::DeploymentPort>{
                {"ma_grounding",
                 grounding_service::ConsumedService::
                     deploymentDescriptor()},
                {"ma_task_command",
                 c2::MaTaskcommandStatusPortClient::
                     deploymentDescriptor()},
                {"ma_mission_plan_command",
                 c2::MaMissionplancommandStatusPortClient::
                     deploymentDescriptor()},
                {"ma_mission_plan_activation_command",
                 c2::
                     MaMissionplanactivationcommandStatusPortClient::
                         deploymentDescriptor()},
                {"ma_action_dispatch",
                 c2::MaActioncommandStatusPortProvider::
                     deploymentDescriptor()},
                {"ma_mission_plan",
                 c2::MaMissionplanInformationPortSink::
                     deploymentDescriptor()},
                {"ma_mission_plan_execution_status",
                 c2::
                     MaMissionplanexecutionstatusInformationPortSink::
                         deploymentDescriptor()},
                {"ma_mission_contingency_alert",
                 c2::
                     MissioncontingencyalertInformationPortSink::
                         deploymentDescriptor()}});
    }

    static void requireStatus(
        pcl_status_t status,
        const std::string& operation) {
        if (status == PCL_OK) {
            return;
        }
        throw std::runtime_error(
            operation + " failed with PCL status " +
            std::to_string(static_cast<int>(status)));
    }

    template <typename Predicate>
    void waitFor(
        Predicate predicate,
        unsigned timeout_ms,
        const char* operation) {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);
        while (!predicate()) {
            checkError();
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error(
                    std::string(operation) +
                    " timed out after " +
                    std::to_string(timeout_ms) + " ms");
            }
            runtime_->executor().spinOnce(10u);
        }
        checkError();
    }

    std::string awaitGrounding(
        std::future<grounding_service::Result<
            grounding_service::Identifier>> future,
        unsigned timeout_ms,
        const char* operation) {
        if (!pcl::await(
                runtime_->executor(),
                future,
                std::chrono::milliseconds(timeout_ms))) {
            throw std::runtime_error(
                std::string(operation) + " timed out after " +
                std::to_string(timeout_ms) + " ms");
        }
        auto result = future.get();
        requireStatus(result.status, operation);
        if (result.value.empty()) {
            throw std::runtime_error(
                std::string(operation) +
                " returned an empty grounding identifier");
        }
        return result.value;
    }

    void spinOnce() {
        step(0u);
    }

    void checkError() {
        if (!error_.empty()) {
            const std::string value = std::move(error_);
            error_.clear();
            throw std::runtime_error(value);
        }
    }

    static ame::agra::CommandProcessingStateEnum actionStatus(
        const std::string& status) {
        std::string normalized;
        normalized.reserve(status.size());
        for (const unsigned char value : status) {
            normalized.push_back(
                static_cast<char>(std::tolower(value)));
        }
        if (normalized == "success" ||
            normalized == "succeeded" ||
            normalized == "complete" ||
            normalized == "completed") {
            return ame::agra::CommandProcessingStateEnum::Accepted;
        }
        if (normalized == "cancelled" ||
            normalized == "canceled") {
            return ame::agra::CommandProcessingStateEnum::Canceled;
        }
        return ame::agra::CommandProcessingStateEnum::Rejected;
    }

    template <typename T>
    static std::vector<T> drain(std::vector<T>& values) {
        std::vector<T> result;
        result.swap(values);
        return result;
    }

    std::unique_ptr<pcl::ProcessRuntime> runtime_;
    grounding_service::ConsumedService grounding_;
    c2::MaTaskcommandStatusPortClient task_port_;
    c2::MaMissionplancommandStatusPortClient
        mission_plan_command_port_;
    c2::MaMissionplanactivationcommandStatusPortClient
        activation_port_;
    c2::MaActioncommandStatusPortProvider action_port_;
    c2::MaMissionplanInformationPortSink mission_plan_port_;
    c2::MaMissionplanexecutionstatusInformationPortSink
        execution_status_port_;
    c2::MissioncontingencyalertInformationPortSink
        contingency_alert_port_;
    c2::SubscriptionHandle task_subscription_;
    c2::SubscriptionHandle mission_plan_command_subscription_;
    c2::SubscriptionHandle activation_subscription_;
    c2::SubscriptionHandle mission_plan_subscription_;
    c2::SubscriptionHandle execution_status_subscription_;
    c2::SubscriptionHandle contingency_alert_subscription_;
    std::vector<
        c2::MA_TaskCommandStatus_Service_Information>
        task_statuses_;
    std::vector<
        c2::MA_MissionPlanCommandStatus_Service_Information>
        mission_plan_command_statuses_;
    std::vector<
        c2::
            MA_MissionPlanActivationCommandStatus_Service_Information>
        activation_statuses_;
    std::vector<c2::MA_MissionPlan_Service_Information>
        mission_plans_;
    std::vector<
        c2::MA_MissionPlanExecutionStatus_Service_Information>
        execution_statuses_;
    std::vector<
        c2::MissionContingencyAlert_Service_Information>
        contingency_alerts_;
    std::vector<c2::MA_ActionCommand_Service_Information>
        action_commands_;
    std::unordered_map<
        std::string, ame::agra::MA_ActionCommandMT>
        pending_actions_;
    std::string error_;
    bool active_ = false;
};

}  // namespace
#endif

class PyIExecutionSink : public ame::IExecutionSink {
public:
    using ame::IExecutionSink::IExecutionSink;

    void reset(const std::string& session_id) override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            void,
            ame::IExecutionSink,
            reset,
            session_id);
    }

    ame::ExecutionSubmission submit(const ame::ActionCommand& command) override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            ame::ExecutionSubmission,
            ame::IExecutionSink,
            submit,
            command);
    }

    std::vector<ame::ActionCommand> pullCommands() override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            std::vector<ame::ActionCommand>,
            ame::IExecutionSink,
            pullCommands);
    }

    void pushResult(const ame::CommandResult& result) override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            void,
            ame::IExecutionSink,
            pushResult,
            result);
    }

    void cancel(const std::string& command_id) override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            void,
            ame::IExecutionSink,
            cancel,
            command_id);
    }

    std::optional<ame::CommandResult> resultFor(
        const std::string& command_id) const override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            std::optional<ame::CommandResult>,
            ame::IExecutionSink,
            resultFor,
            command_id);
    }

    bool isPending(const std::string& command_id) const override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            bool,
            ame::IExecutionSink,
            isPending,
            command_id);
    }

    std::vector<ame::RequirementPlacementRecord> readPlacements()
        const override {
        py::gil_scoped_acquire gil;
        PYBIND11_OVERRIDE_PURE(
            std::vector<ame::RequirementPlacementRecord>,
            ame::IExecutionSink,
            readPlacements);
    }
};


py::bytes agraProtobufToOmsJson(const py::bytes& payload,
                                const std::string& root,
                                const std::string& plugin_path) {
    auto& plugin = omsJsonPlugin(plugin_path);
    if (root == "MA_TaskCommand") {
        auto message = agra_codec::fromJson(
            protobufToNativeJson(payload, "MA_TaskCommandMT"),
            static_cast<ame::agra::MA_TaskCommandMT*>(nullptr));
        decodeHeader(message.message_header);
        headerToOms(message.message_header);
        for (auto& item : message.message_data.command) {
            decodeUuid(item.capability.base.command_id.uuid);
            item.capability.base.command_id.uuid =
                ame::omsUuidFromNative(item.capability.base.command_id.uuid);
            decodeUuid(item.capability.capability_id.uuid);
            item.capability.capability_id.uuid =
                ame::omsUuidFromNative(item.capability.capability_id.uuid);
            decodeUuid(item.capability.task_id.base.uuid);
            item.capability.task_id.base.uuid =
                ame::omsUuidFromNative(item.capability.task_id.base.uuid);
        }
        return encodeOmsJson<ame::agra::MA_TaskCommandMT,
                             pyramid_data_model_agra_MA_TaskCommandMT_c>(
            plugin.codec, "MA_TaskCommand", message,
            pyramid_data_model_agra_MA_TaskCommandMT_c_free);
    }
    if (root == "MA_TaskCommandStatus") {
        auto message = agra_codec::fromJson(
            protobufToNativeJson(payload, "MA_TaskCommandStatusMT"),
            static_cast<ame::agra::MA_TaskCommandStatusMT*>(nullptr));
        decodeHeader(message.message_header);
        headerToOms(message.message_header);
        decodeUuid(message.message_data.base.command_id.uuid);
        message.message_data.base.command_id.uuid =
            ame::omsUuidFromNative(message.message_data.base.command_id.uuid);
        return encodeOmsJson<ame::agra::MA_TaskCommandStatusMT,
                             pyramid_data_model_agra_MA_TaskCommandStatusMT_c>(
            plugin.codec, "MA_TaskCommandStatus", message,
            pyramid_data_model_agra_MA_TaskCommandStatusMT_c_free);
    }
    // Fail closed and name the root. Adding one is a deliberate act -- an
    // unlisted root silently falling through would put an unencodable or
    // mis-encoded message on a conformance-checked wire.
    throw std::runtime_error(
        "no OMS-JSON binding for A-GRA root '" + root +
        "'; add it to agraProtobufToOmsJson rather than routing around it");
}

PYBIND11_MODULE(_ame_py, m) {
    m.def("agra_protobuf_to_oms_json", &agraProtobufToOmsJson,
          py::arg("payload"), py::arg("root"), py::arg("plugin_path"),
          "Encode an agra.* protobuf payload as OMS-JSON using the generated "
          "C++ codec, so a Python host can put it on a LA-CAL transport "
          "without a second encoder implementation.");
    m.doc() = "AME Core Python bindings - direct access to planning components";

#if defined(AME_BUILD_AGRA_MA_BRIDGE)
    installAgraProtobufCodec(m);
    m.attr("agra_ma_bridge_available") = true;
#endif

    // -------------------------------------------------------------------------
    // FactAuthority enum
    // -------------------------------------------------------------------------
    py::enum_<ame::FactAuthority>(m, "FactAuthority")
        .value("BELIEVED", ame::FactAuthority::BELIEVED)
        .value("CONFIRMED", ame::FactAuthority::CONFIRMED)
        .export_values();

    py::enum_<ame::FactAuthorityLevel>(m, "FactAuthorityLevel")
        .value("BELIEVED", ame::FactAuthorityLevel::BELIEVED)
        .value("CONFIRMED", ame::FactAuthorityLevel::CONFIRMED)
        .export_values();

    py::enum_<ame::AutonomyBackendState>(m, "AutonomyBackendState")
        .value("IDLE", ame::AutonomyBackendState::IDLE)
        .value("READY", ame::AutonomyBackendState::READY)
        .value("PENDING_APPROVAL", ame::AutonomyBackendState::PENDING_APPROVAL)
        .value("EXECUTING", ame::AutonomyBackendState::EXECUTING)
        .value("WAITING_FOR_RESULTS", ame::AutonomyBackendState::WAITING_FOR_RESULTS)
        .value("COMPLETE", ame::AutonomyBackendState::COMPLETE)
        .value("FAILED", ame::AutonomyBackendState::FAILED)
        .value("STOPPED", ame::AutonomyBackendState::STOPPED)
        .export_values();

    py::enum_<ame::CommandStatus>(m, "CommandStatus")
        .value("PENDING", ame::CommandStatus::PENDING)
        .value("RUNNING", ame::CommandStatus::RUNNING)
        .value("SUCCEEDED", ame::CommandStatus::SUCCEEDED)
        .value("FAILED_TRANSIENT", ame::CommandStatus::FAILED_TRANSIENT)
        .value("FAILED_PERMANENT", ame::CommandStatus::FAILED_PERMANENT)
        .value("CANCELLED", ame::CommandStatus::CANCELLED)
        .export_values();

    py::enum_<ame::StopMode>(m, "StopMode")
        .value("DRAIN", ame::StopMode::DRAIN)
        .value("IMMEDIATE", ame::StopMode::IMMEDIATE)
        .export_values();

    py::enum_<ame::RequirementPlacementState>(m, "RequirementPlacementState")
        .value("Unspecified", ame::RequirementPlacementState::Unspecified)
        .value("Pending", ame::RequirementPlacementState::Pending)
        .value("Running", ame::RequirementPlacementState::Running)
        .value("Completed", ame::RequirementPlacementState::Completed)
        .value("Failed", ame::RequirementPlacementState::Failed)
        .value("Cancelled", ame::RequirementPlacementState::Cancelled)
        .value("Unsupported", ame::RequirementPlacementState::Unsupported)
        .export_values();

    py::class_<ame::FactUpdate>(m, "FactUpdate")
        .def(py::init<>())
        .def_readwrite("key", &ame::FactUpdate::key)
        .def_readwrite("value", &ame::FactUpdate::value)
        .def_readwrite("source", &ame::FactUpdate::source)
        .def_readwrite("authority", &ame::FactUpdate::authority);

    py::class_<ame::StateUpdate>(m, "StateUpdate")
        .def(py::init<>())
        .def_readwrite("fact_updates", &ame::StateUpdate::fact_updates);

    py::class_<ame::MissionIntent>(m, "MissionIntent")
        .def(py::init<>())
        .def_readwrite("goal_fluents", &ame::MissionIntent::goal_fluents);

    py::class_<ame::AgentState>(m, "AgentState")
        .def(py::init<>())
        .def_readwrite("agent_id", &ame::AgentState::agent_id)
        .def_readwrite("agent_type", &ame::AgentState::agent_type)
        .def_readwrite("available", &ame::AgentState::available);

    py::class_<ame::PolicyEnvelope>(m, "PolicyEnvelope")
        .def(py::init<>())
        .def_readwrite("max_replans", &ame::PolicyEnvelope::max_replans)
        .def_readwrite("enable_goal_dispatch", &ame::PolicyEnvelope::enable_goal_dispatch)
        .def_readwrite("require_plan_approval", &ame::PolicyEnvelope::require_plan_approval);

    py::class_<ame::SessionRequest>(m, "SessionRequest")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::SessionRequest::session_id)
        .def_readwrite("intent", &ame::SessionRequest::intent)
        .def_readwrite("policy", &ame::SessionRequest::policy)
        .def_readwrite("available_agents", &ame::SessionRequest::available_agents);

    py::class_<ame::AutonomyBackendCapabilities>(m, "AutonomyBackendCapabilities")
        .def(py::init<>())
        .def_readwrite("backend_id", &ame::AutonomyBackendCapabilities::backend_id)
        .def_readwrite("supports_batch_planning", &ame::AutonomyBackendCapabilities::supports_batch_planning)
        .def_readwrite("supports_requirement_management", &ame::AutonomyBackendCapabilities::supports_requirement_management)
        .def_readwrite("supports_plan_validation", &ame::AutonomyBackendCapabilities::supports_plan_validation)
        .def_readwrite("supports_external_command_dispatch", &ame::AutonomyBackendCapabilities::supports_external_command_dispatch)
        .def_readwrite("supports_replanning", &ame::AutonomyBackendCapabilities::supports_replanning);

    py::class_<ame::ActionCommand>(m, "ActionCommand")
        .def(py::init<>())
        .def_readwrite("command_id", &ame::ActionCommand::command_id)
        .def_readwrite("action_name", &ame::ActionCommand::action_name)
        .def_readwrite("signature", &ame::ActionCommand::signature)
        .def_readwrite("service_name", &ame::ActionCommand::service_name)
        .def_readwrite("operation", &ame::ActionCommand::operation)
        .def_readwrite("request_fields", &ame::ActionCommand::request_fields);

    py::class_<ame::GoalDispatch>(m, "GoalDispatch")
        .def(py::init<>())
        .def_readwrite("dispatch_id", &ame::GoalDispatch::dispatch_id)
        .def_readwrite("agent_id", &ame::GoalDispatch::agent_id)
        .def_readwrite("goals", &ame::GoalDispatch::goals);

    py::class_<ame::DecisionRecord>(m, "DecisionRecord")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::DecisionRecord::session_id)
        .def_readwrite("plan_id", &ame::DecisionRecord::plan_id)
        .def_readwrite("backend_id", &ame::DecisionRecord::backend_id)
        .def_readwrite("world_version", &ame::DecisionRecord::world_version)
        .def_readwrite("replan_count", &ame::DecisionRecord::replan_count)
        .def_readwrite("plan_success", &ame::DecisionRecord::plan_success)
        .def_readwrite("solve_time_ms", &ame::DecisionRecord::solve_time_ms)
        .def_readwrite("planned_action_signatures", &ame::DecisionRecord::planned_action_signatures)
        .def_readwrite("compiled_bt_xml", &ame::DecisionRecord::compiled_bt_xml);

    py::class_<ame::CommandResult>(m, "CommandResult")
        .def(py::init<>())
        .def_readwrite("command_id", &ame::CommandResult::command_id)
        .def_readwrite("status", &ame::CommandResult::status)
        .def_readwrite("observed_updates", &ame::CommandResult::observed_updates)
        .def_readwrite("source", &ame::CommandResult::source);

    py::class_<ame::RequirementPlacementRecord>(m, "RequirementPlacementRecord")
        .def(py::init<>())
        .def_readwrite("placement_id", &ame::RequirementPlacementRecord::placement_id)
        .def_readwrite("command_id", &ame::RequirementPlacementRecord::command_id)
        .def_readwrite("action_name", &ame::RequirementPlacementRecord::action_name)
        .def_readwrite("signature", &ame::RequirementPlacementRecord::signature)
        .def_readwrite("target_component", &ame::RequirementPlacementRecord::target_component)
        .def_readwrite("target_service", &ame::RequirementPlacementRecord::target_service)
        .def_readwrite("target_type", &ame::RequirementPlacementRecord::target_type)
        .def_readwrite("target_requirement_id", &ame::RequirementPlacementRecord::target_requirement_id)
        .def_readwrite("state", &ame::RequirementPlacementRecord::state);

    py::class_<ame::ExecutionSubmission>(m, "ExecutionSubmission")
        .def(py::init<>())
        .def_readwrite("accepted", &ame::ExecutionSubmission::accepted)
        .def_readwrite("command_egress_visible", &ame::ExecutionSubmission::command_egress_visible)
        .def_readwrite("placement", &ame::ExecutionSubmission::placement)
        .def_readwrite("rejection_reason", &ame::ExecutionSubmission::rejection_reason);

    py::class_<ame::IExecutionSink, PyIExecutionSink>(m, "IExecutionSink")
        .def(py::init<>())
        .def("reset", &ame::IExecutionSink::reset, py::arg("session_id"))
        .def("submit", &ame::IExecutionSink::submit, py::arg("command"))
        .def("pullCommands", &ame::IExecutionSink::pullCommands)
        .def("pushResult", &ame::IExecutionSink::pushResult, py::arg("result"))
        .def("cancel", &ame::IExecutionSink::cancel, py::arg("command_id"))
        .def("resultFor", &ame::IExecutionSink::resultFor, py::arg("command_id"))
        .def("isPending", &ame::IExecutionSink::isPending, py::arg("command_id"))
        .def("readPlacements", &ame::IExecutionSink::readPlacements);

    py::class_<ame::DispatchResult>(m, "DispatchResult")
        .def(py::init<>())
        .def_readwrite("dispatch_id", &ame::DispatchResult::dispatch_id)
        .def_readwrite("status", &ame::DispatchResult::status)
        .def_readwrite("observed_updates", &ame::DispatchResult::observed_updates)
        .def_readwrite("source", &ame::DispatchResult::source);

    py::class_<ame::AutonomyBackendSnapshot>(m, "AutonomyBackendSnapshot")
        .def(py::init<>())
        .def_readwrite("session_id", &ame::AutonomyBackendSnapshot::session_id)
        .def_readwrite("state", &ame::AutonomyBackendSnapshot::state)
        .def_readwrite("world_version", &ame::AutonomyBackendSnapshot::world_version)
        .def_readwrite("replan_count", &ame::AutonomyBackendSnapshot::replan_count)
        .def_readwrite("agent_states", &ame::AutonomyBackendSnapshot::agent_states)
        .def_readwrite("outstanding_commands", &ame::AutonomyBackendSnapshot::outstanding_commands)
        .def_readwrite("outstanding_goal_dispatches", &ame::AutonomyBackendSnapshot::outstanding_goal_dispatches)
        .def_readwrite("decision_history", &ame::AutonomyBackendSnapshot::decision_history)
        .def_readwrite("command_result_history", &ame::AutonomyBackendSnapshot::command_result_history);

    // -------------------------------------------------------------------------
    // AgentInfo struct
    // -------------------------------------------------------------------------
    py::class_<ame::AgentInfo>(m, "AgentInfo")
        .def(py::init<>())
        .def_readwrite("id", &ame::AgentInfo::id)
        .def_readwrite("type", &ame::AgentInfo::type)
        .def_readwrite("available", &ame::AgentInfo::available);

    py::class_<ame::FactMetadata>(m, "FactMetadata")
        .def(py::init<>())
        .def_readwrite("authority", &ame::FactMetadata::authority)
        .def_readwrite("timestamp_us", &ame::FactMetadata::timestamp_us)
        .def_readwrite("source", &ame::FactMetadata::source);

    // -------------------------------------------------------------------------
    // WorldModel
    // -------------------------------------------------------------------------
    py::class_<ame::WorldModel>(m, "WorldModel")
        .def(py::init<>())
        .def("type_system", static_cast<ame::TypeSystem&(ame::WorldModel::*)()>(&ame::WorldModel::typeSystem),
            py::return_value_policy::reference_internal,
            "Access the type system")
        // Fact management - use lambda to avoid overload issues
        .def("set_fact", [](ame::WorldModel& wm, const std::string& key, bool value) {
            wm.setFact(key, value);
        }, py::arg("key"), py::arg("value"),
            "Set a fact by string key")
        .def("set_fact_with_metadata", [](ame::WorldModel& wm,
                                          const std::string& key,
                                          bool value,
                                          const std::string& source,
                                          ame::FactAuthority authority) {
            wm.setFact(key, value, source, authority);
        }, py::arg("key"), py::arg("value"), py::arg("source"), py::arg("authority"),
            "Set a fact by string key with source and authority")
        .def("get_fact", [](const ame::WorldModel& wm, const std::string& key) {
            return wm.getFact(key);
        }, py::arg("key"),
            "Get a fact value by string key")
        .def("get_fact_metadata", [](const ame::WorldModel& wm, const std::string& key) {
            return wm.getFactMetadata(key);
        }, py::arg("key"),
            "Get fact metadata by string key")
        .def("version", &ame::WorldModel::version,
            "Get the current world model version")
        .def("num_fluents", &ame::WorldModel::numFluents,
            "Get the number of fluents")
        .def("fluent_name", &ame::WorldModel::fluentName,
            py::arg("id"),
            "Get fluent name by ID")
        // Goals
        .def("set_goal", &ame::WorldModel::setGoal,
            py::arg("goal_fluents"),
            "Set goal fluents by string keys")
        .def("goal_fluent_ids", &ame::WorldModel::goalFluentIds,
            "Get goal fluent IDs")
        // Types and predicates
        .def("register_predicate", &ame::WorldModel::registerPredicate,
            py::arg("name"), py::arg("param_types"),
            "Register a predicate with parameter types")
        .def("register_confirmed_predicate",
            &ame::WorldModel::registerConfirmedPredicate, py::arg("name"),
            "Declare a predicate whose facts satisfy a precondition only when "
            "observed. Normally comes from the domain's (:confirmed-predicates "
            "...) block rather than this call.")
        .def("is_confirmed_predicate", &ame::WorldModel::isConfirmedPredicate,
            py::arg("name"),
            "Whether a predicate was declared confirmed-only")
        .def("is_confirmed_fact", &ame::WorldModel::isConfirmedFact,
            py::arg("fact_key"),
            "Whether a grounded fact key names a confirmed-only predicate")
        .def("confirmed_predicates",
            [](const ame::WorldModel& wm) {
                const auto& names = wm.confirmedPredicates();
                return std::vector<std::string>(names.begin(), names.end());
            },
            "Names of predicates declared confirmed-only")
        .def("add_object", &ame::WorldModel::addObject,
            py::arg("name"), py::arg("type"),
            "Add an object with a type")
        // Actions
        .def("register_action",
            [](ame::WorldModel& wm,
               const std::string& name,
               const std::vector<std::string>& params,
               const std::vector<std::string>& param_types,
               const std::vector<std::string>& preconditions,
               const std::vector<std::string>& add_effects,
               const std::vector<std::string>& del_effects,
               const std::vector<std::string>& neg_preconditions) {
                wm.registerAction(name, params, param_types, preconditions,
                                  neg_preconditions, add_effects, del_effects);
            },
            py::arg("name"), py::arg("params"), py::arg("param_types"),
            py::arg("preconditions"), py::arg("add_effects"), py::arg("del_effects"),
            py::arg("neg_preconditions") = std::vector<std::string>{},
            "Register an action schema. neg_preconditions are fluent templates "
            "that must be false for the grounded action to apply.")
        .def("num_ground_actions", &ame::WorldModel::numGroundActions,
            "Get number of grounded actions")
        // Agent management
        .def("register_agent", &ame::WorldModel::registerAgent,
            py::arg("id"), py::arg("type"),
            "Register an agent")
        .def("get_agent", static_cast<ame::AgentInfo*(ame::WorldModel::*)(const std::string&)>(&ame::WorldModel::getAgent),
            py::arg("id"),
            py::return_value_policy::reference_internal,
            "Get mutable agent info by id, or None if missing")
        .def("agent_ids", &ame::WorldModel::agentIds,
            "Get all agent IDs")
        .def("available_agent_ids", &ame::WorldModel::availableAgentIds,
            "Get available agent IDs")
        .def("num_agents", &ame::WorldModel::numAgents,
            "Get number of registered agents")
        // Audit callback: fires on every fact state change
        .def("set_audit_callback", [](ame::WorldModel& wm, py::function callback) {
            wm.setAuditCallback([callback](uint64_t version, uint64_t ts_us,
                                           const std::string& fact, bool value,
                                           const std::string& source) {
                py::gil_scoped_acquire gil;
                try {
                    callback(version, ts_us, fact, value, source);
                } catch (...) {}
            });
        }, py::arg("callback"),
            "Set callback fired on every WM fact change: (version, ts_us, fact, value, source)")
        // Snapshot for iteration
        .def("all_true_facts", [](const ame::WorldModel& wm) {
            std::vector<std::string> facts;
            for (unsigned i = 0; i < wm.numFluents(); ++i) {
                if (wm.getFact(i)) {
                    facts.push_back(wm.fluentName(i));
                }
            }
            return facts;
        }, "Get all true facts as strings");

    // -------------------------------------------------------------------------
    // TypeSystem
    // -------------------------------------------------------------------------
    py::class_<ame::TypeSystem>(m, "TypeSystem")
        .def("add_type", &ame::TypeSystem::addType,
            py::arg("name"), py::arg("parent") = "",
            "Add a type with optional parent");

    // -------------------------------------------------------------------------
    // PlanStep
    // -------------------------------------------------------------------------
    py::class_<ame::PlanStep>(m, "PlanStep")
        .def(py::init<>())
        .def_readwrite("action_index", &ame::PlanStep::action_index);

    // -------------------------------------------------------------------------
    // PlanResult
    // -------------------------------------------------------------------------
    py::class_<ame::PlanResult>(m, "PlanResult")
        .def(py::init<>())
        .def_readonly("success", &ame::PlanResult::success)
        .def_readonly("steps", &ame::PlanResult::steps)
        .def_readonly("solve_time_ms", &ame::PlanResult::solve_time_ms)
        .def_readonly("expanded", &ame::PlanResult::expanded)
        .def_readonly("generated", &ame::PlanResult::generated)
        .def_readonly("cost", &ame::PlanResult::cost)
        .def_readonly("error_msg", &ame::PlanResult::error_msg);

    // -------------------------------------------------------------------------
    // Planner
    // -------------------------------------------------------------------------
    py::class_<ame::Planner>(m, "Planner")
        .def(py::init<>())
        .def("solve",
            static_cast<ame::PlanResult (ame::Planner::*)(
                const ame::WorldModel&) const>(&ame::Planner::solve),
            py::arg("wm"),
            "Solve the planning problem defined in WorldModel");

    // -------------------------------------------------------------------------
    // ActionRegistry
    // -------------------------------------------------------------------------
    py::class_<ame::ActionRegistry>(m, "ActionRegistry")
        .def(py::init<>())
        .def("register_action", &ame::ActionRegistry::registerAction,
            py::arg("pddl_name"), py::arg("bt_node_type"), py::arg("reactive") = false)
        .def("register_action_subtree", &ame::ActionRegistry::registerActionSubTree,
            py::arg("pddl_name"), py::arg("subtree_xml_template"), py::arg("reactive") = false)
        .def("register_action_file", &ame::ActionRegistry::registerActionFile,
            py::arg("pddl_name"), py::arg("path"), py::arg("reactive") = false)
        .def("has_action", &ame::ActionRegistry::hasAction, py::arg("pddl_name"))
        .def("registered_names", &ame::ActionRegistry::registeredNames);

    // -------------------------------------------------------------------------
    // PlanCompiler
    // -------------------------------------------------------------------------
    py::class_<ame::PlanCompiler>(m, "PlanCompiler")
        .def(py::init<>())
        .def("set_stub_unregistered_actions",
             &ame::PlanCompiler::setStubUnregisteredActions, py::arg("enabled"),
             "Compile planned actions with no registered BT binding to stub "
             "units (effect predicates only) instead of failing closed. For "
             "authoring/devenv preview; production leaves this off.")
        .def("stub_unregistered_actions",
             &ame::PlanCompiler::stubUnregisteredActions,
             "Whether unregistered actions compile to stub units.")
        .def("compile", [](const ame::PlanCompiler& compiler,
                           const std::vector<ame::PlanStep>& plan,
                           const ame::WorldModel& wm,
                           const ame::ActionRegistry& registry) {
            return compiler.compile(plan, wm, registry);
        }, py::arg("plan"), py::arg("wm"), py::arg("registry"),
            "Compile a plan to BehaviorTree XML")
        .def("compile_with_agent", [](const ame::PlanCompiler& compiler,
                                       const std::vector<ame::PlanStep>& plan,
                                       const ame::WorldModel& wm,
                                       const ame::ActionRegistry& registry,
                                       const std::string& agent_id) {
            return compiler.compile(plan, wm, registry, agent_id);
        }, py::arg("plan"), py::arg("wm"), py::arg("registry"), py::arg("agent_id"),
            "Compile a plan to BehaviorTree XML with agent context");

    py::class_<ame::CurrentAmeBackendAdapter>(m, "CurrentAmeBackendAdapter")
        .def(py::init<ame::WorldModel&, const ame::ActionRegistry&, const ame::Planner&, const ame::PlanCompiler&>(),
            py::arg("world_model"),
            py::arg("action_registry"),
            py::arg("planner"),
            py::arg("plan_compiler"))
        .def("describe_capabilities", &ame::CurrentAmeBackendAdapter::describeCapabilities)
        .def("start", &ame::CurrentAmeBackendAdapter::start, py::arg("request"))
        .def("push_state", &ame::CurrentAmeBackendAdapter::pushState, py::arg("update"))
        .def("push_intent", &ame::CurrentAmeBackendAdapter::pushIntent, py::arg("intent"))
        .def("step", &ame::CurrentAmeBackendAdapter::step)
        .def("pull_commands", &ame::CurrentAmeBackendAdapter::pullCommands)
        .def("pull_goal_dispatches", &ame::CurrentAmeBackendAdapter::pullGoalDispatches)
        .def("pull_decision_records", &ame::CurrentAmeBackendAdapter::pullDecisionRecords)
        .def("approve_plan", &ame::CurrentAmeBackendAdapter::approvePlan, py::arg("plan_id"))
        .def("reject_plan", &ame::CurrentAmeBackendAdapter::rejectPlan,
             py::arg("plan_id"), py::arg("reason"))
        .def("push_command_result", &ame::CurrentAmeBackendAdapter::pushCommandResult, py::arg("result"))
        .def("push_dispatch_result", &ame::CurrentAmeBackendAdapter::pushDispatchResult, py::arg("result"))
        .def("request_stop", &ame::CurrentAmeBackendAdapter::requestStop, py::arg("mode"))
        .def("read_snapshot", &ame::CurrentAmeBackendAdapter::readSnapshot);

#if defined(AME_BUILD_AGRA_MA_BRIDGE)
    py::class_<ame::AgraMaBridge>(m, "AgraMaBridge")
        .def(
            py::init([](
                ame::CurrentAmeBackendAdapter& backend,
                const py::bytes& system_uuid,
                const py::bytes& approval_authority_system_uuid,
                int message_mode,
                unsigned approval_timeout_seconds,
                const ame::PolicyEnvelope& backend_policy) {
                ame::AgraMaBridgeOptions options;
                options.system_uuid =
                    system_uuid.cast<std::string>();
                options.approval_authority_system_uuid =
                    approval_authority_system_uuid.cast<std::string>();
                options.message_mode =
                    static_cast<ame::agra::MessageModeEnum>(
                        message_mode);
                options.approval_timeout_seconds =
                    approval_timeout_seconds;
                options.backend_policy = backend_policy;
                return std::make_unique<ame::AgraMaBridge>(
                    backend, std::move(options));
            }),
            py::arg("backend"),
            py::arg("system_uuid"),
            py::arg("approval_authority_system_uuid"),
            py::arg("message_mode"),
            py::arg("approval_timeout_seconds"),
            py::arg("backend_policy"),
            py::keep_alive<1, 2>())
        .def_static(
            "deterministic_uuid",
            [](const std::string& value) {
                return py::bytes(
                    ame::AgraMaBridge::deterministicUuid(value));
            },
            py::arg("value"))
        .def(
            "register_task_grounding",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& task_payload,
               const std::vector<std::string>& goal_fluents) {
                bridge.registerTaskGrounding(
                    decodeTask(task_payload), goal_fluents);
            },
            py::arg("task_payload"),
            py::arg("goal_fluents"))
        .def(
            "supply_plan_grounding",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& mission_plan_command_uuid,
               const py::iterable& grounding_items) {
                std::vector<ame::AgraPlanElementGrounding>
                    grounding;
                for (const auto item : grounding_items) {
                    const auto data =
                        py::reinterpret_borrow<py::dict>(item);
                    ame::AgraPlanElementGrounding element;
                    element.action_signature =
                        data["action_signature"].cast<std::string>();
                    element.action = decodeAction(
                        data["action_payload"].cast<py::bytes>());
                    element.requires_kinematics =
                        data["requires_kinematics"].cast<bool>();
                    element.route_plan_id =
                        data["route_plan_id"].cast<std::string>();
                    for (const auto waypoint_item :
                         data["waypoints"].cast<py::iterable>()) {
                        const auto waypoint =
                            py::reinterpret_borrow<py::dict>(
                                waypoint_item);
                        element.waypoints.push_back({
                            waypoint["latitude_deg"].cast<double>(),
                            waypoint["longitude_deg"].cast<double>(),
                            waypoint["altitude_m"].cast<double>()});
                    }
                    element.command_parameters =
                        data["command_parameters"].cast<
                            std::unordered_map<
                                std::string, std::string>>();
                    grounding.push_back(std::move(element));
                }
                bridge.supplyPlanGrounding(
                    mission_plan_command_uuid.cast<std::string>(),
                    grounding);
            },
            py::arg("mission_plan_command_uuid"),
            py::arg("grounding"))
        .def(
            "on_mission_plan_command",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& payload) {
                ame::agra_c2_provided::
                    MA_MissionPlanCommand_Service_Information
                        information;
                information.ma_mission_plan_command =
                    decodeMissionPlanCommand(payload);
                bridge.onCommand(information);
            },
            py::arg("payload"))
        .def(
            "on_task_command",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& payload) {
                ame::agra_c2_provided::
                    MA_TaskCommand_Service_Information information;
                information.ma_task_command =
                    decodeTaskCommand(payload);
                bridge.onCommand(information);
            },
            py::arg("payload"))
        .def(
            "on_mission_plan_activation_command",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& payload) {
                ame::agra_c2_provided::
                    MA_MissionPlanActivationCommand_Service_Information
                        information;
                information.ma_mission_plan_activation_command =
                    decodeMissionPlanActivationCommand(payload);
                bridge.onCommand(information);
            },
            py::arg("payload"))
        .def(
            "on_approval_request_status",
            [](ame::AgraMaBridge& bridge,
               const py::bytes& payload) {
                ame::agra_c2_consumed::
                    MA_ApprovalRequestStatus_Service_Information
                        information;
                information.ma_approval_request_status =
                    decodeApprovalStatus(payload);
                bridge.ingestApprovalStatus(information);
            },
            py::arg("payload"))
        .def(
            "read_mission_plan_command_statuses",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MA_MissionPlanCommandStatus_Service_Information;
                return encodeDrain(
                    bridge.handleMaMissionplancommandstatusRead({}),
                    &Information::ma_mission_plan_command_status,
                    "MA_MissionPlanCommandStatusMT",
                    encodeMissionPlanCommandStatus);
            })
        .def(
            "read_task_command_statuses",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MA_TaskCommandStatus_Service_Information;
                return encodeDrain(
                    bridge.handleMaTaskcommandstatusRead({}),
                    &Information::ma_task_command_status,
                    "MA_TaskCommandStatusMT",
                    encodeTaskCommandStatus);
            })
        .def(
            "read_mission_plan_activation_command_statuses",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MA_MissionPlanActivationCommandStatus_Service_Information;
                return encodeDrain(
                    bridge
                        .handleMaMissionplanactivationcommandstatusRead(
                            {}),
                    &Information::
                        ma_mission_plan_activation_command_status,
                    "MA_MissionPlanActivationCommandStatusMT",
                    encodeActivationCommandStatus);
            })
        .def(
            "read_mission_plans",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MA_MissionPlan_Service_Information;
                return encodeDrain(
                    bridge.handleMaMissionplanRead({}),
                    &Information::ma_mission_plan,
                    "MA_MissionPlanMT",
                    encodeMissionPlan);
            })
        .def(
            "read_approval_requests",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_consumed::
                    MA_ApprovalRequest_Service_Information;
                return encodeDrain(
                    bridge.pullApprovalRequests(),
                    &Information::ma_approval_request,
                    "MA_ApprovalRequestMT",
                    encodeApprovalRequest);
            })
        .def(
            "read_mission_plan_execution_statuses",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MA_MissionPlanExecutionStatus_Service_Information;
                return encodeDrain(
                    bridge
                        .handleMaMissionplanexecutionstatusRead({}),
                    &Information::ma_mission_plan_execution_status,
                    "MA_MissionPlanExecutionStatusMT",
                    encodeExecutionStatus);
            })
        .def(
            "read_mission_contingency_alerts",
            [](ame::AgraMaBridge& bridge) {
                using Information = ame::agra_c2_provided::
                    MissionContingencyAlert_Service_Information;
                return encodeDrain(
                    bridge.handleMissioncontingencyalertRead({}),
                    &Information::mission_contingency_alert,
                    "MissionContingencyAlertMT",
                    encodeContingencyAlert);
            })
        .def("pull_commands", &ame::AgraMaBridge::pullCommands)
        .def(
            "push_command_result",
            &ame::AgraMaBridge::pushCommandResult,
            py::arg("result"))
        .def("step", &ame::AgraMaBridge::step);

    py::class_<PyGroundingPortClient>(m, "GroundingPortClient")
        .def(
            py::init<const std::string&, ame::AgraMaBridge*>(),
            py::arg("ports_file") = "",
            py::arg("local_bridge") = nullptr,
            py::keep_alive<1, 3>())
        .def(
            "configure_transport",
            &PyGroundingPortClient::configureTransport,
            py::arg("config_json"))
        .def(
            "create_task_grounding",
            &PyGroundingPortClient::createTaskGrounding,
            py::arg("task_payload"),
            py::arg("goal_fluents"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "create_plan_grounding",
            &PyGroundingPortClient::createPlanGrounding,
            py::arg("mission_plan_command_uuid"),
            py::arg("grounding"),
            py::arg("timeout_ms") = 5000u);

    py::class_<PyLacalAgraMaCompositeClient>(
        m, "LacalAgraMaCompositeClient")
        .def(
            py::init<const std::string&>(),
            py::arg("ports_file"))
        .def(
            "create_task_grounding",
            &PyLacalAgraMaCompositeClient::createTaskGrounding,
            py::arg("task_payload"),
            py::arg("goal_fluents"),
            py::arg("domain_pddl"),
            py::arg("problem_pddl"),
            py::arg("action_bindings"),
            py::arg("confirmed_fluents"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "create_plan_grounding",
            &PyLacalAgraMaCompositeClient::createPlanGrounding,
            py::arg("mission_plan_command_uuid"),
            py::arg("grounding"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "submit_task_command",
            &PyLacalAgraMaCompositeClient::submitTaskCommand,
            py::arg("payload"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "submit_mission_plan_command",
            &PyLacalAgraMaCompositeClient::
                submitMissionPlanCommand,
            py::arg("payload"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "submit_mission_plan_activation_command",
            &PyLacalAgraMaCompositeClient::
                submitActivationCommand,
            py::arg("payload"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "submit_approval_request_status",
            &PyLacalAgraMaCompositeClient::
                submitApprovalStatus,
            py::arg("payload"),
            py::arg("timeout_ms") = 5000u)
        .def(
            "read_task_command_statuses",
            &PyLacalAgraMaCompositeClient::
                readTaskCommandStatuses)
        .def(
            "read_mission_plan_command_statuses",
            &PyLacalAgraMaCompositeClient::
                readMissionPlanCommandStatuses)
        .def(
            "read_mission_plan_activation_command_statuses",
            &PyLacalAgraMaCompositeClient::
                readActivationCommandStatuses)
        .def(
            "read_mission_plans",
            &PyLacalAgraMaCompositeClient::readMissionPlans)
        .def(
            "read_approval_requests",
            &PyLacalAgraMaCompositeClient::
                readApprovalRequests,
            py::arg("timeout_ms") = 5000u)
        .def(
            "read_mission_plan_execution_statuses",
            &PyLacalAgraMaCompositeClient::
                readExecutionStatuses)
        .def(
            "read_mission_contingency_alerts",
            &PyLacalAgraMaCompositeClient::
                readContingencyAlerts)
        .def(
            "pull_action_commands",
            &PyLacalAgraMaCompositeClient::
                pullActionCommands)
        .def(
            "push_action_result",
            &PyLacalAgraMaCompositeClient::pushActionResult,
            py::arg("command_id"),
            py::arg("status"))
        .def(
            "step",
            &PyLacalAgraMaCompositeClient::step,
            py::arg("timeout_ms") = 0u);
#else
    m.attr("agra_ma_bridge_available") = false;
    m.attr("__getattr__") = py::cpp_function(
        [](const std::string& name) -> py::object {
            if (name == "AgraMaBridge") {
                throw py::attribute_error(
                    "AgraMaBridge capability is unavailable; rebuild "
                    "_ame_py with AME_BUILD_AGRA_MA_BRIDGE=ON");
            }
            throw py::attribute_error(
                "module '_ame_py' has no attribute '" + name + "'");
        });
#endif

    // -------------------------------------------------------------------------
    // GoalAllocator
    // -------------------------------------------------------------------------
    py::class_<ame::AgentGoalAssignment>(m, "AgentGoalAssignment")
        .def(py::init<>())
        .def_readwrite("agent_id", &ame::AgentGoalAssignment::agent_id)
        .def_readwrite("goals", &ame::AgentGoalAssignment::goals);

    py::class_<ame::GoalAllocator>(m, "GoalAllocator")
        .def(py::init<>())
        .def("allocate", static_cast<std::vector<ame::AgentGoalAssignment> (ame::GoalAllocator::*)(
                const std::vector<std::string>&,
                const ame::WorldModel&) const>(&ame::GoalAllocator::allocate),
            py::arg("goals"), py::arg("wm"),
            "Allocate goals to available agents from a WorldModel");

    // -------------------------------------------------------------------------
    // PDDL Parser - use PddlParser::parse
    // -------------------------------------------------------------------------
    m.def("parse_pddl", [](ame::WorldModel& wm,
                           const std::string& domain_path,
                           const std::string& problem_path) {
        ame::PddlParser::parse(domain_path, problem_path, wm);
    }, py::arg("wm"), py::arg("domain_path"), py::arg("problem_path"),
        "Parse PDDL domain and problem files into WorldModel");

    // -------------------------------------------------------------------------
    // Utility: get action signature from plan step
    // -------------------------------------------------------------------------
    m.def("get_action_signature", [](const ame::WorldModel& wm, unsigned action_index) {
        if (action_index < wm.numGroundActions()) {
            return wm.groundActions()[action_index].signature;
        }
        return std::string{};
    }, py::arg("wm"), py::arg("action_index"),
    "Get action signature string from action index");

    // -------------------------------------------------------------------------
    // BT NodeStatus enum
    // -------------------------------------------------------------------------
    py::enum_<BT::NodeStatus>(m, "NodeStatus")
        .value("IDLE", BT::NodeStatus::IDLE)
        .value("RUNNING", BT::NodeStatus::RUNNING)
        .value("SUCCESS", BT::NodeStatus::SUCCESS)
        .value("FAILURE", BT::NodeStatus::FAILURE)
        .value("SKIPPED", BT::NodeStatus::SKIPPED)
        .export_values();

    // -------------------------------------------------------------------------
    // ExecutorComponent - BT execution
    // -------------------------------------------------------------------------
    py::class_<ame::ExecutorComponent>(m, "ExecutorComponent")
        .def(py::init<>())
        .def("set_inprocess_world_model", &ame::ExecutorComponent::setInProcessWorldModel,
            py::arg("wm"),
            "Inject world model for in-process execution")
        .def("set_action_sink", &ame::ExecutorComponent::setActionSink,
            py::arg("sink"), py::keep_alive<1, 2>(),
            "Inject the IExecutionSink that AmeDispatchNode leaves dispatch to")
        .def("set_action_registry", &ame::ExecutorComponent::setActionRegistry,
            py::arg("registry"), py::keep_alive<1, 2>(),
            "Inject the ActionRegistry used to register dispatch action verbs")
        .def("set_event_sink", [](ame::ExecutorComponent& exec, py::function callback) {
            exec.setEventSink([callback](const std::string& event) {
                py::gil_scoped_acquire gil;
                try {
                    callback(event);
                } catch (...) {}
            });
        }, py::arg("callback"),
            "Set callback for BT events")
        .def("configure", [](ame::ExecutorComponent& exec) {
            return exec.configure() == PCL_OK;
        }, "Configure the executor")
        .def("activate", [](ame::ExecutorComponent& exec) {
            return exec.activate() == PCL_OK;
        }, "Activate the executor")
        .def("deactivate", [](ame::ExecutorComponent& exec) {
            return exec.deactivate() == PCL_OK;
        }, "Deactivate the executor")
        .def("load_and_execute",
            static_cast<void (ame::ExecutorComponent::*)(const std::string&)>(
                &ame::ExecutorComponent::loadAndExecute),
            py::arg("bt_xml"),
            "Load BT XML and start execution")
        .def("tick_once", &ame::ExecutorComponent::tickOnce,
            "Tick the BT once")
        .def("halt_execution", &ame::ExecutorComponent::haltExecution,
            "Halt execution explicitly")
        .def("is_executing", &ame::ExecutorComponent::isExecuting,
            "Check if BT is currently executing")
        .def("last_status", &ame::ExecutorComponent::lastStatus,
            "Get last BT status");
}
