/// \file test_pcl_generated_interaction_facade.cpp
/// \brief Tests the Phase 2 interaction facade (RequestPortClient /
///        InformationPortSink) -- doc/plans/PYRAMID/
///        rpc_pubsub_interchangeability_plan.md, "Phase 2 -- C++ consumer
///        facade".
///
/// Follows test_pcl_generated_component_stream_handle.cpp's pattern: real
/// generated headers, one in-process pcl::Executor, no transport plugin
/// (local routing), typed assertions end to end. Exercised against two
/// proto trees generated at CMake-configure time (see tests/CMakeLists.txt):
///   - pim/agra_example/ (MAAction_Service, fully projectable post Phase 0)
///     for the positive rpc/pub-sub submit()/transitions() cases.
///   - tests/interaction_facade_fixture/ (Widget_Service, Update
///     deliberately non-projectable) for D2's negative case.
///
/// The provider side in every test is hand-wired from *existing* primitives
/// only (ProvidedHandler/ProvidedService for RPC; raw ConsumedService
/// subscribe<Topic>()/publish<Topic>() for pub/sub) -- Phase 2 does not
/// touch or add a provider facade (that is Phase 3).
///
/// Port lifecycle note: PCL only allows adding publisher/subscriber/service
/// ports while a container is "configuring" (pcl_container_add_publisher()
/// etc. return NULL otherwise -- see pcl_container.c). Every add*Publisher()/
/// subscribe*() call below therefore happens inside an on_configure()
/// override, never from a test body after activate(). This is also why the
/// generated RequestPortClient/InformationPortSink expose a bind() method
/// components must call from their own on_configure().

#include <gtest/gtest.h>

#include "pyramid_services_agra_mission_autonomy_provided_components.hpp"
#include "pyramid_services_agra_c2_station_consumed_components.hpp"
#include "pyramid_services_interaction_facade_fixture_widget_consumed_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_plugin_loader.h>

#include <cstdio>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

// -----------------------------------------------------------------------
// Codec registration.
//
// Every generated invoke*/publish* helper routes its wire payload through
// `pyramid_try_registry_encode`/`decode`, which consults
// `pcl_codec_registry_default()` -- there is no static-codec fallback (the
// transport/codec plugin system is fail-closed by design: "with no codec
// registered for a content_type, encode/decode return an error rather than
// falling back to a built-in codec", transport_codec_plugin_system.md).
// Statically compiling the generated `*_codec.cpp` (toJson/fromJson free
// functions) into this binary, as done below for the typed
// wrapper/cabi-marshal sources, does NOT register anything -- registration
// only happens when a JSON codec *plugin* .so is loaded via
// pcl_plugin_load_codec(), exactly as every real deployment (and
// agra_shm_comms_test.cpp, the proving plan's Phase C harness) does.
//
// mission_autonomy (provided) and c2_station (consumed) are two distinct
// generated packages -- like the interaction_facade_fixture package --
// each with its own JSON codec plugin exporting the identical
// extern "C" pcl_codec_plugin_entry symbol, so they cannot be statically
// linked into this executable (mirrors agra_shm_comms_test.cpp's finding);
// each is built as its own loadable .so (tests/CMakeLists.txt) and
// dlopen'd here into the shared default registry before any test runs.
// All three packages' plugins are loaded into one registry (unlike
// agra_shm_comms_test.cpp's two separate one-role-each processes) because
// this binary hand-wires both provider and consumer roles in one process.
class CodecLoaderEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    LoadOrDie(AGRA_MISSION_AUTONOMY_JSON_CODEC_PLUGIN_PATH);
    LoadOrDie(AGRA_C2_STATION_JSON_CODEC_PLUGIN_PATH);
    LoadOrDie(INTERACTION_FACADE_FIXTURE_WIDGET_JSON_CODEC_PLUGIN_PATH);
  }

 private:
  static void LoadOrDie(const char* path) {
    pcl_plugin_handle_t* handle = nullptr;
    const pcl_status_t rc = pcl_plugin_load_codec(
        path, nullptr, pcl_codec_registry_default(), &handle);
    if (rc != PCL_OK) {
      fprintf(stderr, "FATAL: pcl_plugin_load_codec('%s') failed: rc=%d\n",
              path, static_cast<int>(rc));
      std::abort();
    }
    // Intentionally leaked: the registry borrows the codec vtable pointer
    // for the process lifetime (pcl_plugin_load_codec's documented
    // contract), and this environment itself lives for the whole test run.
  }
};

namespace {
::testing::Environment* const codec_loader_env =
    ::testing::AddGlobalTestEnvironment(new CodecLoaderEnvironment);
}  // namespace

namespace magra_prov = pyramid::components::agra::mission_autonomy::services::provided;
namespace magra_cons = pyramid::components::agra::c2_station::services::consumed;
namespace widget_cons = pyramid::components::interaction_facade_fixture::widget::services::consumed;
// Domain-model types not re-exported by the service headers' local `using`
// aliases (only Ack/Identifier/Query are -- see e.g.
// pyramid_services_agra_c2_station_consumed.hpp) are referenced via their
// full sub-namespace (pyramid::domain_model::agra::*,
// pyramid::domain_model::common::*) rather than the umbrella
// pyramid::domain_model namespace: generating both the A-GRA example and
// the interaction_facade_fixture trees into one output directory (see
// tests/CMakeLists.txt) means whichever tree's generate_bindings.py run
// happens last regenerates the tree-scoped umbrella header
// (pyramid_data_model_types.hpp) from its own, narrower package set --
// the fixture has no pyramid.data_model.agra, so its umbrella run does not
// re-export MA_Action/MA_ActionPlan. The per-package types headers
// (pyramid_data_model_agra_types.hpp etc.) are unaffected either way.

namespace {

// -- Hand-wired provider side (existing primitives only; no Phase 3) --------

class RecordingHandler final : public magra_prov::ProvidedHandler {
 public:
  magra_prov::Ack onMaactionCreate(
      const magra_prov::MAAction_Service_Request& request) override {
    ++create_count;
    last_create = request;
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onMaactionUpdate(
      const magra_prov::MAAction_Service_Requirement& request) override {
    ++update_count;
    last_update = request;
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onMaactionCancel(const magra_prov::Identifier& id) override {
    ++cancel_count;
    last_cancel = id;
    return magra_prov::Ack{true};
  }

  void onMaactionRead(
      const magra_prov::Query& query,
      magra_prov::StreamWriter<magra_prov::MAAction_Service_Requirement> writer)
      override {
    ++read_count;
    last_query = query;
    open_writer = std::move(writer);
  }

  int create_count = 0;
  int update_count = 0;
  int cancel_count = 0;
  int read_count = 0;
  magra_prov::MAAction_Service_Request last_create;
  magra_prov::MAAction_Service_Requirement last_update;
  magra_prov::Identifier last_cancel;
  magra_prov::Query last_query;
  magra_prov::StreamWriter<magra_prov::MAAction_Service_Requirement> open_writer;
};

// Hand-wired provider component: hosts the real ProvidedService (RPC) and a
// raw pub/sub "probe" -- a plain ConsumedService standing in for the
// not-yet-built Phase 3 provider facade (existing primitives only). Both
// the RPC service and the probe's ports are added from on_configure(),
// per PCL's port lifecycle; the probe's request-subscribe callback is
// fixed at add() time, so tests register their observer via
// onRequestPublished() *before* configure() runs.
class ProviderComponent final : public pcl::Component {
 public:
  ProviderComponent(pcl::Executor& executor, RecordingHandler& handler)
      : pcl::Component("magra_provider"),
        provided_(*this, executor, handler),
        probe_(*this, executor) {}

  /// \brief Route the RPC service and every generated topic port local.
  pcl_status_t configureLocal() {
    if (auto rc = provided_.configureTransport(R"({"transport":"local"})");
        rc != PCL_OK) {
      return rc;
    }
    return probe_.configurePubSubTransport(R"({"transport":"local"})");
  }

  /// \brief Must be called before configure().
  void onRequestPublished(
      std::function<void(const magra_prov::MAAction_Service_Request&)> cb) {
    on_request_ = std::move(cb);
  }

  /// \brief Raw pub/sub probe standing in for the not-yet-built Phase 3
  ///        provider facade: subscribes the request topic (observes
  ///        pub/sub-mode submit()) and publishes the requirement topic
  ///        (simulates provider status broadcasts for pub/sub-mode
  ///        transitions()). Existing ConsumedService primitives only.
  magra_prov::ConsumedService& probe() { return probe_; }

 protected:
  pcl_status_t on_configure() override {
    if (auto rc = provided_.bind(); rc != PCL_OK) return rc;
    if (auto rc = probe_.addAgraMaActionRequirementPublisher(); rc != PCL_OK) {
      return rc;
    }
    auto* port = probe_.subscribeAgraMaActionRequest(
        [this](const magra_prov::MAAction_Service_Request& req) {
          if (on_request_) on_request_(req);
        });
    return port ? PCL_OK : PCL_ERR_NOMEM;
  }

 private:
  magra_prov::ProvidedService provided_;
  magra_prov::ConsumedService probe_;
  std::function<void(const magra_prov::MAAction_Service_Request&)> on_request_;
};

class ConsumerComponent final : public pcl::Component {
 public:
  explicit ConsumerComponent(pcl::Executor& executor)
      : pcl::Component("magra_consumer"), client_(*this, executor) {}

  magra_cons::MaactionRequestPortClient& client() { return client_; }

 protected:
  pcl_status_t on_configure() override { return client_.bind(); }

 private:
  magra_cons::MaactionRequestPortClient client_;
};

// -- Facade-built provider side (Phase 3) -----------------------------------

class FacadeHandler final : public magra_prov::MaactionRequestPortHandler {
 public:
  magra_prov::Ack onCreate(
      const magra_prov::MAAction_Service_Request& request) override {
    ++create_count;
    last_create = request;
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onUpdate(
      const magra_prov::MAAction_Service_Requirement& request) override {
    ++update_count;
    last_update = request;
    return magra_prov::Ack{true};
  }

  magra_prov::Ack onCancel(const magra_prov::Identifier& id) override {
    ++cancel_count;
    last_cancel = id;
    return magra_prov::Ack{true};
  }

  int create_count = 0;
  int update_count = 0;
  int cancel_count = 0;
  magra_prov::MAAction_Service_Request last_create;
  magra_prov::MAAction_Service_Requirement last_update;
  magra_prov::Identifier last_cancel;
};

// Provider component built entirely from the Phase 3 facade -- no raw
// ProvidedService/ConsumedService probe, no hand-rolled Read-stream
// tracking. Proves "Phase 2's consumer tests re-run against a facade-built
// provider (facade<->facade, both bindings, one executor)" (plan Phase 3
// accept criteria) and exercises the capabilities Phase 2's hand-wired
// stand-in couldn't: multi-stream Read fan-out and D4 snapshot
// re-publication, both owned internally by RequestPortProvider.
class FacadeProviderComponent final : public pcl::Component {
 public:
  FacadeProviderComponent(pcl::Executor& executor, FacadeHandler& handler)
      : pcl::Component("magra_facade_provider"),
        provider_(*this, executor, handler) {}

  magra_prov::MaactionRequestPortProvider& provider() { return provider_; }

  pcl_status_t configureLocal() {
    return provider_.consumedService().configurePubSubTransport(
        R"({"transport":"local"})");
  }

  pcl_status_t configureInteractionBinding(std::string_view config_json) {
    return provider_.configureInteractionBinding(config_json);
  }

 protected:
  pcl_status_t on_configure() override { return provider_.bind(); }

 private:
  magra_prov::MaactionRequestPortProvider provider_;
};

class WidgetConsumerComponent final : public pcl::Component {
 public:
  explicit WidgetConsumerComponent(pcl::Executor& executor)
      : pcl::Component("widget_consumer"), client_(*this, executor) {}

  widget_cons::WidgetRequestPortClient& client() { return client_; }

 protected:
  pcl_status_t on_configure() override { return client_.bind(); }

 private:
  widget_cons::WidgetRequestPortClient client_;
};

// PCL's local (no-transport) pub/sub delivery requires an actual listener:
// pcl_executor_publish_port()'s PCL_ROUTE_LOCAL branch reports
// PCL_ERR_NOT_FOUND when nothing local subscribes to the topic (mirroring
// find_service()'s behaviour for unary RPC). The widget fixture has no
// generated "provided"-role package to hand-wire a real provider from (it
// is consumed-only, see the fixture proto's header comment), so this probe
// -- a second widget_cons::ConsumedService instance, reusing the
// role-symmetric subscribe primitive every generated ConsumedService
// carries -- stands in as the fixture's sole listener for the pub/sub
// positive case (SubmitProjectableCreateUnderPubsubStillWorksOnTheSameFixture).
class WidgetProbeComponent final : public pcl::Component {
 public:
  explicit WidgetProbeComponent(pcl::Executor& executor)
      : pcl::Component("widget_probe"), probe_(*this, executor) {}

  pcl_status_t configureLocal() {
    return probe_.configurePubSubTransport(R"({"transport":"local"})");
  }

 protected:
  pcl_status_t on_configure() override {
    auto* port = probe_.subscribeInteractionFacadeFixtureWidgetRequest(
        [](const widget_cons::Widget_Service_Request&) {});
    return port ? PCL_OK : PCL_ERR_NOMEM;
  }

 private:
  widget_cons::ConsumedService probe_;
};

class PlanPublisherComponent final : public pcl::Component {
 public:
  explicit PlanPublisherComponent(pcl::Executor& executor)
      : pcl::Component("maactionplan_publisher"), publisher_(*this, executor) {}

  magra_prov::ConsumedService& publisher() { return publisher_; }

 protected:
  pcl_status_t on_configure() override {
    return publisher_.addAgraMaActionPlanInformationPublisher();
  }

 private:
  magra_prov::ConsumedService publisher_;
};

class PlanSinkComponent final : public pcl::Component {
 public:
  explicit PlanSinkComponent(pcl::Executor& executor)
      : pcl::Component("maactionplan_sink"), sink_(*this, executor) {}

  magra_cons::MaactionplanInformationPortSink& sink() { return sink_; }

 protected:
  pcl_status_t on_configure() override { return sink_.bind(); }

 private:
  magra_cons::MaactionplanInformationPortSink sink_;
};

}  // namespace

// -----------------------------------------------------------------------
// submit() -- RPC realization
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade,
     SubmitCreateUnderRpcDeliversAckAndPopulatesRemoteAck) {
  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = "action-1";
  command.ma_action = action;

  // submit() returns std::future<SubmitResult> (deferred): .get() runs it.
  // Local routing is synchronous, so no polling/wait_for is needed.
  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);
  ASSERT_TRUE(result.remoteAck().has_value());
  EXPECT_TRUE(result.remoteAck()->success);
  EXPECT_EQ(handler.create_count, 1);
  ASSERT_TRUE(handler.last_create.ma_action.has_value());
  EXPECT_EQ(handler.last_create.ma_action->id, "action-1");
}

// -----------------------------------------------------------------------
// submit() -- pub/sub realization
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade,
     SubmitCreateUnderPubsubDeliversToSubscriberWithNoRemoteAck) {
  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  std::vector<magra_prov::MAAction_Service_Request> observed;
  provider.onRequestPublished(
      [&](const magra_prov::MAAction_Service_Request& req) {
        observed.push_back(req);
      });

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(
      consumer.client().configureInteractionBinding(R"({"request_leg":"pubsub"})"),
      PCL_OK);

  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = "action-2";
  command.ma_action = action;

  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);
  EXPECT_FALSE(result.remoteAck().has_value());
  EXPECT_EQ(handler.create_count, 0);  // no RPC call happened

  ASSERT_EQ(observed.size(), 1u);
  ASSERT_TRUE(observed[0].ma_action.has_value());
  EXPECT_EQ(observed[0].ma_action->id, "action-2");
}

// -----------------------------------------------------------------------
// submit() -- D2 negative case: non-projectable command over a
// pub/sub-realized request leg fails at the facade, precisely.
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade,
     SubmitNonProjectableUpdateUnderPubsubFailsClosedWithPclErrState) {
  pcl::Executor executor;
  WidgetConsumerComponent consumer{executor};

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(
      consumer.client().configureInteractionBinding(R"({"request_leg":"pubsub"})"),
      PCL_OK);

  // No publish is even attempted: a correct implementation must fail before
  // touching the wire at all (D2's "fail at the facade").
  widget_cons::Widget_Service_Requirement command{};
  auto result = consumer.client().submit(command).get();
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, PCL_ERR_STATE);
  EXPECT_FALSE(result.remoteAck().has_value());
}

TEST(PclGeneratedInteractionFacade,
     SubmitProjectableCreateUnderPubsubStillWorksOnTheSameFixture) {
  pcl::Executor executor;
  WidgetProbeComponent probe{executor};
  WidgetConsumerComponent consumer{executor};

  ASSERT_EQ(probe.configure(), PCL_OK);
  ASSERT_EQ(probe.activate(), PCL_OK);
  ASSERT_EQ(executor.add(probe), PCL_OK);
  ASSERT_EQ(probe.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(
      consumer.client().configureInteractionBinding(R"({"request_leg":"pubsub"})"),
      PCL_OK);

  // Create's request type IS the wrapper (D2: is_wrapper) -- projectable
  // even though this fixture's Update/Cancel are not. A listener (probe)
  // must be present for PCL's local-routed publish to report delivery.
  widget_cons::Widget_Service_Request command{};
  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);
}

// -----------------------------------------------------------------------
// transitions() -- RPC realization
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade,
     TransitionsUnderRpcDeliversFramesFromProviderStream) {
  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  bool ended = false;
  magra_cons::Query query{};
  query.id.push_back("action-3");
  auto handle = consumer.client().transitions(
      query,
      [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      },
      [&](pcl_status_t) { ended = true; });
  ASSERT_TRUE(handle.valid());
  ASSERT_EQ(handler.read_count, 1);
  ASSERT_EQ(handler.last_query.id.size(), 1u);
  EXPECT_EQ(handler.last_query.id.front(), "action-3");

  magra_prov::MAAction_Service_Requirement frame{};
  pyramid::domain_model::common::Requirement req{};
  req.id = "action-3";
  frame.ma_action_status = req;
  ASSERT_EQ(handler.open_writer.send(frame), PCL_OK);
  ASSERT_EQ(handler.open_writer.end(PCL_OK), PCL_OK);

  ASSERT_EQ(received.size(), 1u);
  ASSERT_TRUE(received[0].ma_action_status.has_value());
  EXPECT_EQ(received[0].ma_action_status->id, "action-3");
  EXPECT_TRUE(ended);
}

// -----------------------------------------------------------------------
// transitions() -- pub/sub realization: client-side query.id filtering
// (D4). Two concurrently-published ids; the subscriber only sees the one
// it queried for -- mirrors the correlation/non-conflation pattern from
// agra_shm_comms_test.cpp, in-process rather than cross-process SHM.
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade, TransitionsUnderPubsubFiltersByQueryId) {
  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(
                R"({"requirement_leg":"pubsub"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  magra_cons::Query query{};
  query.id.push_back("action-A");
  auto handle = consumer.client().transitions(
      query, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      });
  ASSERT_TRUE(handle.valid());

  magra_prov::MAAction_Service_Requirement frame_a{};
  pyramid::domain_model::common::Requirement req_a{};
  req_a.id = "action-A";
  frame_a.ma_action_status = req_a;

  magra_prov::MAAction_Service_Requirement frame_b{};
  pyramid::domain_model::common::Requirement req_b{};
  req_b.id = "action-B";
  frame_b.ma_action_status = req_b;

  ASSERT_EQ(provider.probe().publishAgraMaActionRequirement(frame_a), PCL_OK);
  ASSERT_EQ(provider.probe().publishAgraMaActionRequirement(frame_b), PCL_OK);

  ASSERT_EQ(received.size(), 1u);
  ASSERT_TRUE(received[0].ma_action_status.has_value());
  EXPECT_EQ(received[0].ma_action_status->id, "action-A");

  handle.cancel();
  ASSERT_EQ(provider.probe().publishAgraMaActionRequirement(frame_a), PCL_OK);
  EXPECT_EQ(received.size(), 1u);  // cancel() suppressed further delivery
}

// Codex PR review: PubsubTransition dropped the on_end callback entirely
// (never even stored it), so a Query.one_shot subscriber completing under
// pub/sub never got the completion signal a one_shot RPC Read stream gives
// when the provider ends it -- silently non-interchangeable for any
// caller relying on on_end.
TEST(PclGeneratedInteractionFacade,
     TransitionsUnderPubsubHonorsOneShotAndFiresOnEnd) {
  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(
                R"({"requirement_leg":"pubsub"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  bool ended = false;
  pcl_status_t end_status = PCL_ERR_INVALID;
  magra_cons::Query query{};
  query.id.push_back("action-one-shot-pubsub");
  query.one_shot = true;
  auto handle = consumer.client().transitions(
      query,
      [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      },
      [&](pcl_status_t status) {
        ended = true;
        end_status = status;
      });
  ASSERT_TRUE(handle.valid());

  magra_prov::MAAction_Service_Requirement frame_a{};
  pyramid::domain_model::common::Requirement req_a{};
  req_a.id = "action-one-shot-pubsub";
  frame_a.ma_action_status = req_a;

  ASSERT_EQ(provider.probe().publishAgraMaActionRequirement(frame_a), PCL_OK);

  ASSERT_EQ(received.size(), 1u);
  EXPECT_TRUE(ended);
  EXPECT_EQ(end_status, PCL_OK);

  // one_shot deactivated the subscription: a second publication for the
  // same id must not be delivered.
  ASSERT_EQ(provider.probe().publishAgraMaActionRequirement(frame_a), PCL_OK);
  EXPECT_EQ(received.size(), 1u);
}

// -----------------------------------------------------------------------
// InformationPortSink -- pub/sub realization
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacade,
     InformationPortSinkSubscribeUnderPubsubReceivesPublication) {
  pcl::Executor executor;

  PlanPublisherComponent publisher_host{executor};
  ASSERT_EQ(publisher_host.configure(), PCL_OK);
  ASSERT_EQ(publisher_host.activate(), PCL_OK);
  ASSERT_EQ(executor.add(publisher_host), PCL_OK);
  ASSERT_EQ(publisher_host.publisher().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);

  PlanSinkComponent sink_host{executor};
  ASSERT_EQ(sink_host.configure(), PCL_OK);
  ASSERT_EQ(sink_host.activate(), PCL_OK);
  ASSERT_EQ(executor.add(sink_host), PCL_OK);
  ASSERT_EQ(sink_host.sink().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(sink_host.sink().configureInteractionBinding(R"({"binding":"pubsub"})"),
            PCL_OK);

  std::vector<magra_cons::MAActionPlan_Service_Information> received;
  auto handle = sink_host.sink().subscribe(
      [&](const magra_cons::MAActionPlan_Service_Information& msg) {
        received.push_back(msg);
      });
  ASSERT_TRUE(handle.valid());

  magra_prov::MAActionPlan_Service_Information info{};
  pyramid::domain_model::agra::MA_ActionPlan plan{};
  plan.plan_id = "plan-1";
  info.ma_action_plan = plan;
  ASSERT_EQ(publisher_host.publisher().publishAgraMaActionPlanInformation(info),
            PCL_OK);

  ASSERT_EQ(received.size(), 1u);
  ASSERT_TRUE(received[0].ma_action_plan.has_value());
  EXPECT_EQ(received[0].ma_action_plan->plan_id, "plan-1");
}

// -----------------------------------------------------------------------
// Same scenario, both bindings: the load-bearing "one API, two
// realizations" proof for Phase 2. Same compiled facade code path
// (submit()) exercised end to end under {"binding":"rpc"} and
// {"binding":"pubsub"}.
// -----------------------------------------------------------------------

class InteractionFacadeBindingTest
    : public ::testing::TestWithParam<std::string_view> {};

TEST_P(InteractionFacadeBindingTest, SubmitCreateRoundTrip) {
  const std::string binding_json =
      std::string(R"({"binding":")") + std::string(GetParam()) + "\"}";

  pcl::Executor executor;
  RecordingHandler handler;
  ProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  std::vector<magra_prov::MAAction_Service_Request> observed;
  provider.onRequestPublished(
      [&](const magra_prov::MAAction_Service_Request& req) {
        observed.push_back(req);
      });

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(binding_json), PCL_OK);

  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = "action-param";
  command.ma_action = action;

  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);

  if (GetParam() == "rpc") {
    ASSERT_TRUE(result.remoteAck().has_value());
    EXPECT_EQ(handler.create_count, 1);
    EXPECT_EQ(observed.size(), 0u);
  } else {
    EXPECT_FALSE(result.remoteAck().has_value());
    EXPECT_EQ(handler.create_count, 0);
    ASSERT_EQ(observed.size(), 1u);
    EXPECT_EQ(observed[0].ma_action->id, "action-param");
  }
}

INSTANTIATE_TEST_SUITE_P(RpcAndPubsub, InteractionFacadeBindingTest,
                          ::testing::Values("rpc", "pubsub"));

// -----------------------------------------------------------------------
// Phase 3: RequestPortProvider -- facade-built provider, both bindings,
// one executor (plan Phase 3 accept criteria: "Phase 2's consumer tests
// re-run against a facade-built provider").
// -----------------------------------------------------------------------

TEST(PclGeneratedInteractionFacadeProvider,
     FacadeSubmitCreateRoundTripUnderRpc) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = "facade-action-1";
  command.ma_action = action;

  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);
  ASSERT_TRUE(result.remoteAck().has_value());
  EXPECT_TRUE(result.remoteAck()->success);
  EXPECT_EQ(handler.create_count, 1);
  ASSERT_TRUE(handler.last_create.ma_action.has_value());
  EXPECT_EQ(handler.last_create.ma_action->id, "facade-action-1");
}

TEST(PclGeneratedInteractionFacadeProvider,
     FacadeSubmitCreateRoundTripUnderPubsub) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(
      consumer.client().configureInteractionBinding(R"({"request_leg":"pubsub"})"),
      PCL_OK);

  magra_cons::MAAction_Service_Request command{};
  pyramid::domain_model::agra::MA_Action action{};
  action.id = "facade-action-2";
  command.ma_action = action;

  auto result = consumer.client().submit(command).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, PCL_OK);
  EXPECT_FALSE(result.remoteAck().has_value());  // D3: no synthetic ack

  EXPECT_EQ(handler.create_count, 1);
  ASSERT_TRUE(handler.last_create.ma_action.has_value());
  EXPECT_EQ(handler.last_create.ma_action->id, "facade-action-2");
}

// Plan Phase 3 accept criteria: "two concurrent Read streams with disjoint
// queries each receiving only their transitions (RPC mode)". The provider's
// TransitionWriter (owned by RequestPortProvider, not the test) fans a
// single send() out to every open Read stream, filtered by query.id --
// this test proves two overlapping subscriptions on the same executor never
// cross-deliver.
TEST(PclGeneratedInteractionFacadeProvider,
     TransitionWriterFansOutToConcurrentReadStreamsByQuery) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer_a{executor};
  ConsumerComponent consumer_b{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);
  ASSERT_EQ(provider.configureInteractionBinding(R"({"binding":"rpc"})"), PCL_OK);

  ASSERT_EQ(consumer_a.configure(), PCL_OK);
  ASSERT_EQ(consumer_a.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer_a), PCL_OK);
  ASSERT_EQ(consumer_a.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer_a.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  ASSERT_EQ(consumer_b.configure(), PCL_OK);
  ASSERT_EQ(consumer_b.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer_b), PCL_OK);
  ASSERT_EQ(consumer_b.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer_b.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received_a, received_b;
  magra_cons::Query query_a{};
  query_a.id.push_back("facade-action-A");
  magra_cons::Query query_b{};
  query_b.id.push_back("facade-action-B");

  auto handle_a = consumer_a.client().transitions(
      query_a, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received_a.push_back(frame);
      });
  auto handle_b = consumer_b.client().transitions(
      query_b, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received_b.push_back(frame);
      });
  ASSERT_TRUE(handle_a.valid());
  ASSERT_TRUE(handle_b.valid());

  auto writer = provider.provider().transitionWriter();

  magra_prov::MAAction_Service_Requirement frame_a{};
  pyramid::domain_model::common::Requirement req_a{};
  req_a.id = "facade-action-A";
  frame_a.ma_action_status = req_a;
  ASSERT_EQ(writer.send(frame_a), PCL_OK);

  magra_prov::MAAction_Service_Requirement frame_b{};
  pyramid::domain_model::common::Requirement req_b{};
  req_b.id = "facade-action-B";
  frame_b.ma_action_status = req_b;
  ASSERT_EQ(writer.send(frame_b), PCL_OK);

  ASSERT_EQ(received_a.size(), 1u);
  ASSERT_TRUE(received_a[0].ma_action_status.has_value());
  EXPECT_EQ(received_a[0].ma_action_status->id, "facade-action-A");

  ASSERT_EQ(received_b.size(), 1u);
  ASSERT_TRUE(received_b[0].ma_action_status.has_value());
  EXPECT_EQ(received_b[0].ma_action_status->id, "facade-action-B");
}

// Plan Phase 3 / D4 accept criteria: "late-command snapshot re-publication
// observed (pub/sub mode)". The consumer subscribes transitions() first and
// observes an original send() normally (count 1); a later command for the
// *same* id then triggers dispatchPubsubCommand()'s snapshot
// re-publication -- a second, distinct delivery of the same snapshot the
// consumer would otherwise have to already be watching at exactly the
// right moment to catch (the late-join mitigation D4 calls for).
TEST(PclGeneratedInteractionFacadeProvider,
     LateCommandTriggersSnapshotRepublicationUnderPubsub) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);
  ASSERT_EQ(
      provider.configureInteractionBinding(R"({"requirement_leg":"pubsub"})"),
      PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().configurePubSubTransport(
                R"({"transport":"local"})"),
            PCL_OK);
  ASSERT_EQ(
      consumer.client().configureInteractionBinding(
          R"({"request_leg":"pubsub","requirement_leg":"pubsub"})"),
      PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  magra_cons::Query query{};
  query.id.push_back("facade-action-late");
  auto handle = consumer.client().transitions(
      query, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      });
  ASSERT_TRUE(handle.valid());

  // Original send: recorded into the D6 snapshot store *and* delivered
  // normally to the already-live subscription (baseline, not the
  // mechanism under test).
  magra_prov::MAAction_Service_Requirement original{};
  pyramid::domain_model::common::Requirement original_req{};
  original_req.id = "facade-action-late";
  original.ma_action_status = original_req;
  ASSERT_EQ(provider.provider().transitionWriter().send(original), PCL_OK);
  ASSERT_EQ(received.size(), 1u);

  // A later command for the same id: dispatchPubsubCommand() re-publishes
  // the recorded snapshot after handling the command itself -- a second,
  // distinct delivery the consumer observes purely from the re-publication.
  // Cancel has its own dedicated submit() overload (const Identifier&) --
  // constructing a MAAction_Service_Request wrapper with .cancel set and
  // calling submit() on *that* would resolve to Create's overload instead
  // (matching argument type), not what this test means to exercise.
  magra_cons::Identifier late_command_id{"facade-action-late"};
  auto result = consumer.client().submit(late_command_id).get();
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(handler.cancel_count, 1);

  ASSERT_EQ(received.size(), 2u);
  ASSERT_TRUE(received[1].ma_action_status.has_value());
  EXPECT_EQ(received[1].ma_action_status->id, "facade-action-late");
}

// An RPC Read stream opened after a transition for its id was already
// recorded (registerOpenStream(), the RPC-mode counterpart of the pub/sub
// test above -- reachable only under RPC binding, since pub/sub readers get
// their own late-join mitigation via dispatchPubsubCommand()'s
// republishSnapshotFor()) must replay that stored snapshot immediately --
// otherwise a late RPC reader waits forever for a terminal-state id (e.g.
// COMPLETED/CANCELLED) that will never transition again.
TEST(PclGeneratedInteractionFacadeProvider,
     LateRpcStreamReplaysStoredSnapshot) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);
  ASSERT_EQ(provider.configureInteractionBinding(R"({"binding":"rpc"})"), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  // Transition happens *before* any Read stream is open -- nothing is
  // listening yet, so this can only reach a late reader via the D6
  // snapshot store, not fanOutRpc()'s normal live-delivery path.
  magra_prov::MAAction_Service_Requirement original{};
  pyramid::domain_model::common::Requirement original_req{};
  original_req.id = "facade-action-late-rpc";
  original.ma_action_status = original_req;
  ASSERT_EQ(provider.provider().transitionWriter().send(original), PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  magra_cons::Query query{};
  query.id.push_back("facade-action-late-rpc");
  auto handle = consumer.client().transitions(
      query, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      });
  ASSERT_TRUE(handle.valid());

  ASSERT_EQ(received.size(), 1u);
  ASSERT_TRUE(received[0].ma_action_status.has_value());
  EXPECT_EQ(received[0].ma_action_status->id, "facade-action-late-rpc");
}

// Codex PR review: a client-cancelled RPC Read stream stays live() forever
// (pcl_stream_cancel() only sets the context's cancelled flag, not ctx_),
// so fanOutRpc() must prune it -- and end() it -- before calling send() on
// it, not merely check live(). Otherwise every future transitionWriter()
// send() for that id keeps hitting the cancelled stream and can report
// PCL_ERR_CANCELLED for what is, from the provider's perspective, an
// entirely valid transition with simply no one left listening.
TEST(PclGeneratedInteractionFacadeProvider,
     FanOutRpcDropsCancelledStreamBeforeSending) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);
  ASSERT_EQ(provider.configureInteractionBinding(R"({"binding":"rpc"})"), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  magra_cons::Query query{};
  query.id.push_back("facade-action-cancel");
  auto handle = consumer.client().transitions(
      query, [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      });
  ASSERT_TRUE(handle.valid());

  handle.cancel();

  magra_prov::MAAction_Service_Requirement frame{};
  pyramid::domain_model::common::Requirement req{};
  req.id = "facade-action-cancel";
  frame.ma_action_status = req;
  // The stream's only subscriber cancelled: fanOutRpc() must have pruned
  // and ended it, so this send() sees no matching live stream at all and
  // returns PCL_OK -- not PCL_ERR_CANCELLED, which would incorrectly blame
  // the transition itself for a listener that is simply gone.
  EXPECT_EQ(provider.provider().transitionWriter().send(frame), PCL_OK);
  EXPECT_TRUE(received.empty());
}

// InformationPortSource::publish()'s RPC-mode fan-out loop has the
// identical live()-only bug FanOutRpcDropsCancelledStreamBeforeSending
// covers for RequestPortProvider -- same generator (interaction_facade_
// gen.py), same StreamWriter<T>/open_streams_ pattern, independently
// emitted for the Data-1 leg. A runtime round-trip through this specific
// generated class is blocked by an unrelated, pre-existing gap this PR
// does not touch: every Data-1 Read rpc takes google.protobuf.Empty, and
// no generated JSON codec plugin in this codebase registers a codec for
// Empty (every other RPC-realized streaming test here uses a Query- or
// Identifier-typed request, which *do* have registered codecs), so
// dispatchStream()'s pyramid_try_registry_decode() call fails closed
// before the fan-out loop under test is ever reached. See
// pim/test_cancelled_stream_pruning.py for the generated-output-level
// regression test covering this fix instead.

// Codex PR review: fanOutRpc() sent every matching transition forever,
// never checking Query.one_shot at all -- unlike the pub/sub realization,
// which at least stops future delivery (see
// TransitionsUnderPubsubHonorsOneShotAndFiresOnEnd's sibling fix). D4's
// own design table requires the RPC realization to "honour one_shot" the
// same as pub/sub.
TEST(PclGeneratedInteractionFacadeProvider,
     FanOutRpcEndsStreamAfterOneShotMatch) {
  pcl::Executor executor;
  FacadeHandler handler;
  FacadeProviderComponent provider{executor, handler};
  ConsumerComponent consumer{executor};

  ASSERT_EQ(provider.configure(), PCL_OK);
  ASSERT_EQ(provider.activate(), PCL_OK);
  ASSERT_EQ(executor.add(provider), PCL_OK);
  ASSERT_EQ(provider.configureLocal(), PCL_OK);
  ASSERT_EQ(provider.configureInteractionBinding(R"({"binding":"rpc"})"), PCL_OK);

  ASSERT_EQ(consumer.configure(), PCL_OK);
  ASSERT_EQ(consumer.activate(), PCL_OK);
  ASSERT_EQ(executor.add(consumer), PCL_OK);
  ASSERT_EQ(consumer.client().consumedService().routeAllLocal(), PCL_OK);
  ASSERT_EQ(consumer.client().configureInteractionBinding(R"({"binding":"rpc"})"),
            PCL_OK);

  std::vector<magra_cons::MAAction_Service_Requirement> received;
  bool ended = false;
  pcl_status_t end_status = PCL_ERR_INVALID;
  magra_cons::Query query{};
  query.id.push_back("facade-action-one-shot");
  query.one_shot = true;
  auto handle = consumer.client().transitions(
      query,
      [&](const magra_cons::MAAction_Service_Requirement& frame) {
        received.push_back(frame);
      },
      [&](pcl_status_t status) {
        ended = true;
        end_status = status;
      });
  ASSERT_TRUE(handle.valid());

  magra_prov::MAAction_Service_Requirement frame{};
  pyramid::domain_model::common::Requirement req{};
  req.id = "facade-action-one-shot";
  frame.ma_action_status = req;
  ASSERT_EQ(provider.provider().transitionWriter().send(frame), PCL_OK);

  ASSERT_EQ(received.size(), 1u);
  EXPECT_TRUE(ended);
  EXPECT_EQ(end_status, PCL_OK);

  // fanOutRpc() must have ended and pruned the stream after its one match:
  // a second transition for the same id is not delivered at all.
  magra_prov::MAAction_Service_Requirement frame2{};
  pyramid::domain_model::common::Requirement req2{};
  req2.id = "facade-action-one-shot";
  frame2.ma_action_status = req2;
  provider.provider().transitionWriter().send(frame2);
  EXPECT_EQ(received.size(), 1u);
}
