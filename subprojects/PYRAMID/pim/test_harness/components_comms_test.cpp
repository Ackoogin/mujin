// \file components_comms_test.cpp
// \brief Plugin-system viability example for the new PIM proto set
//        (subprojects/PYRAMID/pim/test/).

#include "pyramid_services_pim_osprey_sensor_products_provided.hpp"

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
}

// The generated per-component JSON codec plugin entry point, statically linked
// into this test. The FlatBuffers plugin is loaded from a shared object/DLL.
extern "C" const pcl_codec_t* pcl_codec_plugin_entry(const char* config_json);

#include <cstdio>
#include <cstdlib>
#include <string>

namespace prov = pyramid::components::pim_osprey::sensor_products::services::provided;
namespace dm   = pyramid::domain_model;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) ++g_failures;
}

struct ProviderHandler : prov::ServiceHandler {
    bool seen = false;

    dm::common::Ack
    handleSprrequirementCreate(const prov::SPRRequirement_Service_Request& req) override {
        seen = true;
        dm::common::Ack a;
        a.success = true;
        a.identifier = req.cancel ? *req.cancel : "osprey.sensor_products";
        return a;
    }
};

ProviderHandler g_handler;

struct RpcProviderState {
    std::string response_storage;
};

pcl_status_t serve_create(pcl_container_t*, const pcl_msg_t* request,
                          pcl_msg_t* response, pcl_svc_context_t*, void* ud) {
    auto* state = static_cast<RpcProviderState*>(ud);
    void* resp = nullptr;
    size_t resp_size = 0;
    prov::dispatch(g_handler,
                   prov::ServiceChannel::SprrequirementCreate,
                   request ? request->data : nullptr,
                   request ? request->size : 0u,
                   request ? request->type_name : prov::kJsonContentType,
                   &resp, &resp_size);
    state->response_storage.assign(static_cast<const char*>(resp ? resp : ""),
                                   resp_size);
    if (resp) std::free(resp);
    response->data = state->response_storage.empty()
        ? nullptr
        : const_cast<char*>(state->response_storage.data());
    response->size = static_cast<uint32_t>(state->response_storage.size());
    response->type_name = request ? request->type_name : prov::kJsonContentType;
    return PCL_OK;
}

pcl_status_t rpc_on_configure(pcl_container_t* c, void* ud) {
    return pcl_container_add_service(
        c, prov::kSvcSprrequirementCreate, prov::kJsonContentType,
        serve_create, ud) ? PCL_OK : PCL_ERR_NOMEM;
}

struct ClientCtx {
    bool decoded = false;
    dm::common::Ack ack;
};

void on_response(const pcl_msg_t* msg, void* ud) {
    auto* ctx = static_cast<ClientCtx*>(ud);
    ctx->decoded = prov::decodeSprrequirementCreateResponse(msg, &ctx->ack);
}

void run_rpc_roundtrip() {
    pcl_executor_t* exec = pcl_executor_create();
    RpcProviderState state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = rpc_on_configure;
    pcl_container_t* c = pcl_container_create("osprey_sensor_products_rpc",
                                              &cbs, &state);
    check(c != nullptr, "create provider container");
    check(pcl_container_configure(c) == PCL_OK, "configure provider");
    check(pcl_container_activate(c) == PCL_OK, "activate provider");
    check(pcl_executor_add(exec, c) == PCL_OK, "add provider to executor");

    prov::SPRRequirement_Service_Request req;
    req.sprrequest.emplace();
    ClientCtx ctx;
    pcl_status_t rc = prov::invokeSprrequirementCreate(
        exec, req, on_response, &ctx);
    check(rc == PCL_OK, "invoke spr_requirement.create");
    check(g_handler.seen, "provider handler ran");
    check(ctx.decoded, "client decoded Ack response");
    check(ctx.ack.success && ctx.ack.identifier == "osprey.sensor_products",
          "round-tripped Ack payload is correct");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

struct TopicProviderState {
    std::string content_type;
    pcl_port_t* information_pub = nullptr;
    pcl_port_t* entity_pub = nullptr;
    bool request_seen = false;
    bool cancel_seen = false;
};

struct TopicConsumerState {
    std::string content_type;
    pcl_port_t* request_pub = nullptr;
    bool information_seen = false;
    bool entity_seen = false;
    unsigned entity_count = 0;
};

void publish_entity_transition(TopicProviderState* state) {
    prov::SPRRequirement_Service_Entity entity;
    entity.sprrequirement.emplace();
    pcl_status_t rc = prov::publishPimOspreySprRequirementEntity(
        state->entity_pub, entity, state->content_type.c_str());
    check(rc == PCL_OK, "provider published entity transition");
}

void on_request_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<TopicProviderState*>(ud);
    prov::SPRRequirement_Service_Request request;
    if (!prov::decodePimOspreySprRequirementRequest(msg, &request)) {
        check(false, "provider decoded request-topic payload");
        return;
    }
    if (request.sprrequest) {
        state->request_seen = true;
    }
    if (request.cancel) {
        state->cancel_seen = true;
    }
    publish_entity_transition(state);
}

void on_information_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<TopicConsumerState*>(ud);
    prov::SPRInformation_Service_Information payload;
    state->information_seen =
        prov::decodePimOspreySprInformationInformation(msg, &payload)
        && static_cast<bool>(payload.sprinformation);
}

void on_entity_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<TopicConsumerState*>(ud);
    prov::SPRRequirement_Service_Entity payload;
    state->entity_seen =
        prov::decodePimOspreySprRequirementEntity(msg, &payload)
        && static_cast<bool>(payload.sprrequirement);
    if (state->entity_seen) {
        ++state->entity_count;
    }
}

pcl_status_t provider_topics_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<TopicProviderState*>(ud);
    state->information_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprInformationInformation,
        state->content_type.c_str());
    state->entity_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprRequirementEntity,
        state->content_type.c_str());
    pcl_port_t* sub = prov::subscribePimOspreySprRequirementRequest(
        c, on_request_topic, state, state->content_type.c_str());
    return (state->information_pub && state->entity_pub && sub)
        ? PCL_OK
        : PCL_ERR_NOMEM;
}

pcl_status_t consumer_topics_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<TopicConsumerState*>(ud);
    state->request_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprRequirementRequest,
        state->content_type.c_str());
    pcl_port_t* info_sub = prov::subscribePimOspreySprInformationInformation(
        c, on_information_topic, state, state->content_type.c_str());
    pcl_port_t* req_sub = prov::subscribePimOspreySprRequirementEntity(
        c, on_entity_topic, state, state->content_type.c_str());
    return (state->request_pub && info_sub && req_sub)
        ? PCL_OK
        : PCL_ERR_NOMEM;
}

void run_topic_roundtrip(const char* content_type) {
    std::printf("\n== topic round-trip: %s ==\n", content_type);
    pcl_executor_t* exec = pcl_executor_create();

    TopicProviderState provider_state;
    provider_state.content_type = content_type;
    TopicConsumerState consumer_state;
    consumer_state.content_type = content_type;

    pcl_callbacks_t provider_cbs{};
    provider_cbs.on_configure = provider_topics_on_configure;
    pcl_callbacks_t consumer_cbs{};
    consumer_cbs.on_configure = consumer_topics_on_configure;

    pcl_container_t* provider = pcl_container_create(
        "osprey_sensor_products_topic_provider",
        &provider_cbs, &provider_state);
    pcl_container_t* consumer = pcl_container_create(
        "osprey_sensor_products_topic_consumer",
        &consumer_cbs, &consumer_state);

    check(provider != nullptr, "create topic provider");
    check(consumer != nullptr, "create topic consumer");
    check(pcl_container_configure(provider) == PCL_OK, "configure topic provider");
    check(pcl_container_configure(consumer) == PCL_OK, "configure topic consumer");
    check(pcl_container_activate(provider) == PCL_OK, "activate topic provider");
    check(pcl_container_activate(consumer) == PCL_OK, "activate topic consumer");
    check(pcl_executor_add(exec, provider) == PCL_OK, "add topic provider");
    check(pcl_executor_add(exec, consumer) == PCL_OK, "add topic consumer");

    prov::SPRInformation_Service_Information info;
    info.sprinformation.emplace();
    check(prov::publishPimOspreySprInformationInformation(
              provider_state.information_pub, info, content_type) == PCL_OK,
          "provider published information topic");
    check(consumer_state.information_seen,
          "consumer decoded information topic");

    prov::SPRRequirement_Service_Request create_request;
    create_request.sprrequest.emplace();
    check(prov::publishPimOspreySprRequirementRequest(
              consumer_state.request_pub, create_request, content_type) == PCL_OK,
          "consumer published request topic");
    check(provider_state.request_seen, "provider observed request topic");
    check(consumer_state.entity_seen,
          "consumer observed entity transition");
    check(consumer_state.entity_count >= 1,
          "conformance: request followed by entity transition");

    consumer_state.entity_seen = false;
    const unsigned before_cancel_entities = consumer_state.entity_count;
    prov::SPRRequirement_Service_Request cancel_request;
    cancel_request.cancel = std::string("cancel-spr-001");
    check(prov::publishPimOspreySprRequirementRequest(
              consumer_state.request_pub, cancel_request, content_type) == PCL_OK,
          "consumer published cancel on request topic");
    check(provider_state.cancel_seen, "provider observed cancel topic payload");
    check(consumer_state.entity_seen,
          "consumer observed post-cancel entity transition");
    check(consumer_state.entity_count > before_cancel_entities,
          "conformance: cancel followed by entity transition");

    pcl_executor_destroy(exec);
    pcl_container_destroy(provider);
    pcl_container_destroy(consumer);
}

}  // namespace

int main(int argc, char** argv) {
    const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
    check(pcl_codec_registry_register(pcl_codec_registry_default(), codec) == PCL_OK,
          "register JSON codec plugin");

    pcl_plugin_handle_t* flatbuffers_handle = nullptr;
    if (argc >= 2) {
        pcl_status_t lr = pcl_plugin_load_codec(
            argv[1], "{\"source\":\"components_comms_test\"}",
            pcl_codec_registry_default(), &flatbuffers_handle);
        check(lr == PCL_OK, "load FlatBuffers codec plugin");
        check(pcl_codec_registry_get(
                  pcl_codec_registry_default(),
                  prov::kFlatBuffersContentType) != nullptr,
              "FlatBuffers codec registered");
    } else {
        check(false, "FlatBuffers codec plugin path supplied");
    }

    run_rpc_roundtrip();
    run_topic_roundtrip(prov::kJsonContentType);
    if (flatbuffers_handle) {
        run_topic_roundtrip(prov::kFlatBuffersContentType);
    }

    pcl_codec_registry_clear(pcl_codec_registry_default());
    if (flatbuffers_handle) {
        pcl_plugin_unload(flatbuffers_handle);
    }

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
