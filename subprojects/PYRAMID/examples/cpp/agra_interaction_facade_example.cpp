// Showcase: the interaction facade (RequestPortClient / RequestPortProvider)
// against the A-GRA example contract (pim/agra_example/, grammar-conforming
// Request-shape MAAction_Service) -- doc/plans/PYRAMID/
// rpc_pubsub_interchangeability_plan.md, Phases 2/3.
//
// Unlike tobj_shared_memory_example.cpp (which hand-wires ProvidedService/
// ConsumedService and hand-rolls stream tracking), both sides here are built
// entirely from the generated facade: MaactionRequestPortProvider owns the
// RPC service *and* the pub/sub request/requirement legs internally, and
// MaactionRequestPortClient exposes submit()/transitions() as the sole
// client-facing surface. Nothing in this file names an RPC or pub/sub
// primitive directly -- that is exactly the point of the facade.
//
// Demonstrated flow:
//   1. Build one provider component and one client component on a single
//      in-process pcl::Executor (local routing only, no transport plugin).
//   2. Pick the request leg's realization -- RPC or pub/sub -- from a
//      manifest-style JSON string, selectable on the command line
//      (`--binding=rpc` (default) or `--binding=pubsub`). Same component
//      code either way: only Configure_Interaction_Binding's argument
//      changes.
//   3. submit() a Create command and print whether a remote ack came back
//      (only ever true under RPC -- D3: pub/sub never synthesizes one).
//   4. transitions() a Query, then publish a status frame through the
//      provider's transitionWriter() and print the frame the client
//      receives.
//
// Local routing is synchronous (see rpc_pubsub_interchangeability_plan.md
// Phase 2), so -- like the facade's own tests -- this example needs no
// spin loop, no background thread, and no pcl::await: submit(...).get()
// and transitions(...) both resolve inline.

#include "pyramid_services_agra_c2_station_consumed_components.hpp"
#include "pyramid_services_agra_mission_autonomy_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_plugin_loader.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

namespace magra_prov =
    pyramid::components::agra::mission_autonomy::services::provided;
namespace magra_cons =
    pyramid::components::agra::c2_station::services::consumed;

namespace {

void LoadCodecOrDie(const char* path) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_status_t rc = pcl_plugin_load_codec(
      path, nullptr, pcl_codec_registry_default(), &handle);
  if (rc != PCL_OK) {
    std::fprintf(stderr, "FATAL: pcl_plugin_load_codec('%s') failed: rc=%d\n",
                 path, static_cast<int>(rc));
    std::exit(1);
  }
  // Intentionally leaked -- the registry borrows the vtable for the process
  // lifetime, and this example's process lifetime is the whole demo.
}

// Facade-side handler: MaactionRequestPortProvider dispatches Create/Update/
// Cancel here regardless of which realization delivered them. Publishing a
// transition back out is a separate step (see main()): RequestPortProvider
// owns the transitionWriter() that fans a send() out to every open Read
// stream (and, under pub/sub, records + republishes the D6 snapshot) --
// business logic (here, the handler) drives it, the facade does not invoke
// it automatically.
class DemoHandler final : public magra_prov::MaactionRequestPortHandler {
 public:
  magra_prov::Ack onCreate(
      const magra_prov::MAAction_Service_Request& request) override {
    last_id_ =
        request.ma_action.has_value() ? request.ma_action->id : "<none>";
    std::printf("[provider] Create id=%s\n", last_id_.c_str());
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onUpdate(
      const magra_prov::MAAction_Service_Requirement&) override {
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onCancel(const magra_prov::Identifier&) override {
    return magra_prov::Ack{true};
  }

  std::string last_id_;
};

// Provider component: composed entirely from the facade -- no hand-rolled
// ProvidedService/ConsumedService wiring.
class ProviderComponent final : public pcl::Component {
 public:
  ProviderComponent(pcl::Executor& executor, DemoHandler& handler)
      : pcl::Component("agra_provider"), provider_(*this, executor, handler) {}

  magra_prov::MaactionRequestPortProvider& provider() { return provider_; }

  pcl_status_t configureLocal() {
    return provider_.consumedService().configurePubSubTransport(
        R"({"transport":"local"})");
  }

 protected:
  pcl_status_t on_configure() override { return provider_.bind(); }

 private:
  magra_prov::MaactionRequestPortProvider provider_;
};

// Client component: submit()/transitions() are the entire consumer-facing
// surface -- the same code drives either realization.
class ClientComponent final : public pcl::Component {
 public:
  explicit ClientComponent(pcl::Executor& executor)
      : pcl::Component("agra_client"), client_(*this, executor) {}

  magra_cons::MaactionRequestPortClient& client() { return client_; }

 protected:
  pcl_status_t on_configure() override { return client_.bind(); }

 private:
  magra_cons::MaactionRequestPortClient client_;
};

}  // namespace

int main(int argc, char** argv) {
  std::string_view binding = "rpc";
  for (int i = 1; i < argc; ++i) {
    std::string_view arg = argv[i];
    if (arg.rfind("--binding=", 0) == 0) {
      binding = arg.substr(std::strlen("--binding="));
    }
  }
  if (binding != "rpc" && binding != "pubsub") {
    std::fprintf(stderr, "usage: %s [--binding=rpc|pubsub]\n", argv[0]);
    return 1;
  }

  LoadCodecOrDie(AGRA_MISSION_AUTONOMY_JSON_CODEC_PLUGIN_PATH);
  LoadCodecOrDie(AGRA_C2_STATION_JSON_CODEC_PLUGIN_PATH);

  pcl::Executor executor;
  DemoHandler handler;
  ProviderComponent provider{executor, handler};
  ClientComponent client{executor};

  if (provider.configure() != PCL_OK || provider.activate() != PCL_OK ||
      executor.add(provider) != PCL_OK || provider.configureLocal() != PCL_OK) {
    std::fprintf(stderr, "provider bring-up failed\n");
    return 1;
  }

  if (client.configure() != PCL_OK || client.activate() != PCL_OK ||
      executor.add(client) != PCL_OK) {
    std::fprintf(stderr, "client bring-up failed\n");
    return 1;
  }

  // Realization switch by manifest: this JSON string is the only thing that
  // changes between "RPC" and "pub/sub" -- everything above and below is
  // identical component code either way.
  //
  //   {"binding":"rpc"}                                  -- both legs, RPC
  //   {"request_leg":"pubsub","requirement_leg":"pubsub"} -- both legs, pubsub
  std::string config_json;
  if (binding == "rpc") {
    if (client.client().consumedService().routeAllLocal() != PCL_OK) {
      std::fprintf(stderr, "client local RPC routing failed\n");
      return 1;
    }
    config_json = R"({"binding":"rpc"})";
  } else {
    if (client.client().consumedService().configurePubSubTransport(
            R"({"transport":"local"})") != PCL_OK) {
      std::fprintf(stderr, "client local pub/sub routing failed\n");
      return 1;
    }
    config_json = R"({"request_leg":"pubsub","requirement_leg":"pubsub"})";
  }

  if (client.client().configureInteractionBinding(config_json) != PCL_OK ||
      provider.provider().configureInteractionBinding(config_json) != PCL_OK) {
    std::fprintf(stderr, "configureInteractionBinding failed\n");
    return 1;
  }

  std::printf("[client] realization: %.*s\n",
              static_cast<int>(binding.size()), binding.data());

  // transitions() -- subscribe before submitting, so the frame published
  // below (once the Create has been handled) is delivered rather than
  // missed. Under RPC this opens a streaming Read; under pub/sub it
  // subscribes the requirement topic -- same client code either way.
  const std::string kActionId = "example-action-1";
  magra_cons::Query query{};
  query.id.push_back(kActionId);
  query.one_shot = true;
  bool ended = false;
  auto handle = client.client().transitions(
      query,
      [](const magra_cons::MAAction_Service_Requirement& frame) {
        const std::string id = frame.ma_action_status.has_value()
                                    ? frame.ma_action_status->id
                                    : "<none>";
        std::printf("[client] transition frame id=%s\n", id.c_str());
      },
      [&ended](pcl_status_t) { ended = true; });
  if (!handle.valid()) {
    std::fprintf(stderr, "transitions() failed to open\n");
    return 1;
  }

  // submit() -- Create.
  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = kActionId;
  command.ma_action = action;

  auto result = client.client().submit(command).get();
  std::printf("[client] submit accepted=%s status=%d remote_ack=%s\n",
              result.accepted ? "true" : "false",
              static_cast<int>(result.status),
              result.remoteAck().has_value()
                  ? (result.remoteAck()->success ? "true" : "false")
                  : "<none>");
  std::printf("[provider] handler observed create for id=%s\n",
              handler.last_id_.c_str());

  // Publish the transition: business logic (here, main() standing in for
  // whatever drives the mission autonomy state machine) uses the provider's
  // transitionWriter() -- owned internally by RequestPortProvider, fans out
  // to every open Read stream and (under pub/sub) records the D6 snapshot.
  magra_prov::MAAction_Service_Requirement frame{};
  pyramid::domain_model::common::Requirement status{};
  status.id = kActionId;
  frame.ma_action_status = status;
  if (provider.provider().transitionWriter().send(frame) != PCL_OK) {
    std::fprintf(stderr, "transitionWriter().send() failed\n");
    return 1;
  }

  std::printf("[client] transitions stream ended=%s\n",
              ended ? "true" : "false");

  return 0;
}
