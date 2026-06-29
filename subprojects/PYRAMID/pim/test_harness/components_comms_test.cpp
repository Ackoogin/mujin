// \file components_comms_test.cpp
// \brief Plugin-system viability example for the *new* PIM proto set
//        (subprojects/PYRAMID/pim/test/).
//
// Stands up generated PYRAMID components from the new proto and has them
// communicate request/response through the PCL runtime with the generated JSON
// codec plugin registered — proving the new bindings are wire-viable end to end
// over the plugin system.
//
// Build with ./build_comms_test.sh (generates bindings, compiles the generated
// sources + JSON codec plugin against libpcl_core, runs this).

#include "pyramid_services_pim_osprey_sensor_products_provided.hpp"

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_codec_registry.h>
}

// The generated per-component JSON codec plugin entry point.
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

// Provider component: hosts spr_requirement.create and returns a tagged Ack so
// the client can prove the typed request reached the handler and the response
// round-tripped through the JSON codec.
struct ProviderHandler : prov::ServiceHandler {
    bool seen = false;
    dm::common::Ack
    handleSprrequirementCreate(const prov::SPRRequirement_Service_Request& req) override {
        seen = true;
        dm::common::Ack a;
        a.success = true;
        a.identifier = "osprey.sensor_products";
        (void)req;
        return a;
    }
};

ProviderHandler g_handler;

pcl_status_t serve_create(pcl_container_t*, const pcl_msg_t* request,
                          pcl_msg_t* response, pcl_svc_context_t*, void* ud) {
    auto* buf = static_cast<std::string*>(ud);
    void* resp = nullptr;
    size_t resp_size = 0;
    prov::dispatch(g_handler,
                   prov::ServiceChannel::SprrequirementCreate,
                   request ? request->data : nullptr,
                   request ? request->size : 0u,
                   request ? request->type_name : prov::kJsonContentType,
                   &resp, &resp_size);
    buf->assign(static_cast<const char*>(resp ? resp : ""), resp_size);
    if (resp) std::free(resp);
    response->data = buf->empty() ? nullptr : const_cast<char*>(buf->data());
    response->size = static_cast<uint32_t>(buf->size());
    response->type_name = request ? request->type_name : prov::kJsonContentType;
    return PCL_OK;
}

std::string g_resp_storage;

pcl_status_t on_configure(pcl_container_t* c, void* ud) {
    return pcl_container_add_service(
        c, prov::kSvcSprrequirementCreate, prov::kJsonContentType,
        serve_create, ud) ? PCL_OK : PCL_ERR_NOMEM;
}

struct ClientCtx { bool decoded = false; dm::common::Ack ack; };

void on_response(const pcl_msg_t* msg, void* ud) {
    auto* ctx = static_cast<ClientCtx*>(ud);
    ctx->decoded = prov::decodeSprrequirementCreateResponse(msg, &ctx->ack);
}

}  // namespace

int main() {
    // Register the generated JSON codec plugin into the default registry — this
    // is the "plugin system" the components communicate through.
    const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
    check(pcl_codec_registry_register(pcl_codec_registry_default(), codec) == PCL_OK,
          "register JSON codec plugin");

    pcl_executor_t* exec = pcl_executor_create();
    pcl_callbacks_t cbs{};
    cbs.on_configure = on_configure;
    pcl_container_t* c = pcl_container_create("osprey_sensor_products",
                                              &cbs, &g_resp_storage);
    check(c != nullptr, "create provider container");
    check(pcl_container_configure(c) == PCL_OK, "configure provider");
    check(pcl_container_activate(c) == PCL_OK, "activate provider");
    check(pcl_executor_add(exec, c) == PCL_OK, "add provider to executor");

    // Client invokes spr_requirement.create (req = oneof wrapper, rsp = Ack),
    // both serialised through the registered JSON codec.
    prov::SPRRequirement_Service_Request req;
    req.sprrequest.emplace();  // select the SPRRequest oneof arm
    ClientCtx ctx;
    pcl_status_t rc = prov::invokeSprrequirementCreate(exec, req, on_response, &ctx);
    check(rc == PCL_OK, "invoke spr_requirement.create");
    check(g_handler.seen, "provider handler ran");
    check(ctx.decoded, "client decoded Ack response");
    check(ctx.ack.success && ctx.ack.identifier == "osprey.sensor_products",
          "round-tripped Ack payload is correct");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
