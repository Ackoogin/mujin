// \file plugin_load_test.cpp
// \brief No-relink plugin demonstration for the new PIM proto set
//        (subprojects/PYRAMID/pim/test/).
//
// Unlike components_comms_test.cpp -- which compiles the generated JSON codec
// plugin *into* the test binary and calls pcl_codec_plugin_entry() directly --
// this test links NO wire codec at all. The client/provider binary compiles
// only the component-side pieces (native types, C-ABI marshal, the service
// facade) plus libpcl_core; it obtains the JSON codec exclusively by loading a
// *prebuilt* codec plugin shared library at runtime through the PCL plugin
// loader. This proves the plugin contract holds with no relink: swap or supply
// the codec by pointing at a different .dll/.so, not by recompiling the client.
//
// It also demonstrates that opaque, plugin-specific configuration is threaded
// through the loader to the plugin's entry point: a config_json string is
// passed to pcl_plugin_load_codec() and the loaded codec exposes it via
// codec_ctx (the generated plugin sets codec_ctx to its stored config when a
// non-empty config is supplied, and leaves it NULL otherwise).
//
// Usage:
//   plugin_load_test <codec_plugin_path> [config_json]
//
// Build/run with build_plugin_load_test.bat (Windows) or
// build_plugin_load_test.sh (Linux): both build the codec plugin as a stand-
// alone shared library, build this test WITHOUT the plugin/codec sources, and
// run it against the freshly built plugin.

#include "pyramid_services_pim_osprey_sensor_products_provided.hpp"

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
}

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
// round-tripped through the loaded JSON codec.
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

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
            "usage: %s <codec_plugin_path> [config_json]\n", argv[0]);
        return 2;
    }
    const char* plugin_path = argv[1];
    // Opaque, plugin-specific configuration threaded through the loader to the
    // plugin's entry point. A non-empty string must reach the plugin (observed
    // below via the loaded codec's codec_ctx).
    const char* config_json =
        (argc >= 3) ? argv[2] : "{\"source\":\"plugin_load_test\",\"pretty\":false}";

    std::printf("== loading codec plugin (no relink) ==\n");
    std::printf("   path   : %s\n", plugin_path);
    std::printf("   config : %s\n", config_json);

    pcl_codec_registry_t* reg = pcl_codec_registry_default();

    // The whole point: register the codec by loading a prebuilt shared library
    // at runtime. This binary contains no JSON codec of its own.
    pcl_plugin_handle_t* handle = nullptr;
    pcl_status_t lr =
        pcl_plugin_load_codec(plugin_path, config_json, reg, &handle);
    check(lr == PCL_OK, "pcl_plugin_load_codec succeeded");
    check(handle != nullptr, "plugin handle returned");

    // Config passthrough: the generated plugin stores the config string and
    // points codec_ctx at it when non-empty, so a non-NULL codec_ctx proves the
    // config reached the plugin's entry point through the loader.
    const pcl_codec_t* loaded = pcl_codec_registry_get(reg, prov::kJsonContentType);
    check(loaded != nullptr, "loaded codec registered for application/json");
    check(loaded != nullptr && loaded->codec_ctx != nullptr,
          "config_json threaded through loader to plugin (codec_ctx set)");

    // Drive the full provider/client round-trip entirely through the codec that
    // was just loaded from disk -- no codec is compiled into this binary.
    pcl_executor_t* exec = pcl_executor_create();
    pcl_callbacks_t cbs{};
    cbs.on_configure = on_configure;
    pcl_container_t* c = pcl_container_create("osprey_sensor_products",
                                              &cbs, &g_resp_storage);
    check(c != nullptr, "create provider container");
    check(pcl_container_configure(c) == PCL_OK, "configure provider");
    check(pcl_container_activate(c) == PCL_OK, "activate provider");
    check(pcl_executor_add(exec, c) == PCL_OK, "add provider to executor");

    prov::SPRRequirement_Service_Request req;
    req.sprrequest.emplace();  // select the SPRRequest oneof arm
    ClientCtx ctx;
    pcl_status_t rc = prov::invokeSprrequirementCreate(exec, req, on_response, &ctx);
    check(rc == PCL_OK, "invoke spr_requirement.create via loaded plugin");
    check(g_handler.seen, "provider handler ran");
    check(ctx.decoded, "client decoded Ack response via loaded plugin");
    check(ctx.ack.success && ctx.ack.identifier == "osprey.sensor_products",
          "round-tripped Ack payload is correct");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);

    // The registry borrows the plugin's codec vtable, so unload only after the
    // registry is done with it.
    pcl_codec_registry_clear(reg);
    pcl_plugin_unload(handle);

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
