/// \file test_pcl_executor.cpp
/// \brief Tests for PCL executor — spin, dispatch, multi-container, shutdown.
#include <gtest/gtest.h>

#include <thread>
#include <chrono>
#include <utility>

extern "C" {
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"
}

// -- Helpers -------------------------------------------------------------

struct TickCounter {
  int tick_count = 0;
};

static pcl_status_t counting_tick(pcl_container_t*, double, void* ud) {
  static_cast<TickCounter*>(ud)->tick_count++;
  return PCL_OK;
}

static pcl_callbacks_t counting_callbacks() {
  pcl_callbacks_t cbs = {};
  cbs.on_tick = counting_tick;
  return cbs;
}

// -- Basic Executor Tests ------------------------------------------------

TEST(PclExecutor, CreateDestroy) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_executor_destroy(e);
}

TEST(PclExecutor, SpinOnceTicksActiveContainer) {
  TickCounter counter;
  pcl_callbacks_t cbs = counting_callbacks();
  auto* c = pcl_container_create("ticker", &cbs, &counter);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_container_set_tick_rate_hz(c, 100.0);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // Allow wall-clock time to advance so the tick accumulator crosses the
  // configured period deterministically.
  for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    pcl_executor_spin_once(e, 0);
  }

  // at least one tick should have fired
  EXPECT_GT(counter.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, InactiveContainerNotTicked) {
  TickCounter counter;
  pcl_callbacks_t cbs = counting_callbacks();
  auto* c = pcl_container_create("inactive", &cbs, &counter);
  // leave unconfigured — should not tick

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_executor_spin_once(e, 0);

  EXPECT_EQ(counter.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, MultipleContainers) {
  TickCounter c1_count, c2_count;
  pcl_callbacks_t cbs = counting_callbacks();

  auto* c1 = pcl_container_create("c1", &cbs, &c1_count);
  auto* c2 = pcl_container_create("c2", &cbs, &c2_count);

  pcl_container_configure(c1);
  pcl_container_activate(c1);
  pcl_container_configure(c2);
  pcl_container_activate(c2);

  // set different tick rates
  pcl_container_set_tick_rate_hz(c1, 10.0);
  pcl_container_set_tick_rate_hz(c2, 50.0);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c1);
  pcl_executor_add(e, c2);

  for (int i = 0; i < 15; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pcl_executor_spin_once(e, 0);
  }

  // both should have been ticked
  EXPECT_GT(c1_count.tick_count, 0);
  EXPECT_GT(c2_count.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c1);
  pcl_container_destroy(c2);
}

// -- Shutdown Tests ------------------------------------------------------

TEST(PclExecutor, RequestShutdownStopsSpin) {
  auto* c = pcl_container_create("spin_test", nullptr, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // spin in background, request shutdown after a short delay
  std::thread spinner([e]() {
    pcl_executor_spin(e);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pcl_executor_request_shutdown(e);

  spinner.join(); // should return promptly

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, GracefulShutdownFinalizesContainers) {
  int deactivate_count = 0;
  int shutdown_count   = 0;

  pcl_callbacks_t cbs = {};
  cbs.on_deactivate = [](pcl_container_t*, void* ud) -> pcl_status_t {
    (*static_cast<int*>(ud))++;
    return PCL_OK;
  };

  // slightly hacky but works for this test — we need separate counters
  // so we use two containers
  auto* c = pcl_container_create("grace", &cbs, &deactivate_count);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  auto rc = pcl_executor_shutdown_graceful(e, 5000);
  EXPECT_EQ(rc, PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);
  EXPECT_EQ(deactivate_count, 1);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Intra-process Dispatch Tests ----------------------------------------

struct SubReceived {
  bool received = false;
  uint32_t size = 0;
};

static void sub_callback(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
  auto* r = static_cast<SubReceived*>(ud);
  r->received = true;
  r->size = msg->size;
}

TEST(PclExecutor, IntraProcessPubSub) {
  SubReceived sub_data;
  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "topic_a", "TestMsg",
                                  sub_callback, ud);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("subscriber", &sub_cbs, &sub_data);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, sub_c);

  // dispatch a message manually (simulating a publisher)
  int payload = 42;
  pcl_msg_t msg = {};
  msg.data = &payload;
  msg.size = sizeof(payload);
  msg.type_name = "TestMsg";

  auto rc = pcl_executor_dispatch_incoming(e, "topic_a", &msg);
  EXPECT_EQ(rc, PCL_OK);
  EXPECT_TRUE(sub_data.received);
  EXPECT_EQ(sub_data.size, sizeof(int));

  pcl_executor_destroy(e);
  pcl_container_destroy(sub_c);
}

TEST(PclExecutor, DispatchToUnknownTopicReturnsNotFound) {
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.data = nullptr;
  msg.size = 0;
  msg.type_name = "X";

  EXPECT_EQ(pcl_executor_dispatch_incoming(e, "no_such_topic", &msg),
            PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

struct ThreadInjectedData {
  bool            received = false;
  int             value = 0;
  std::thread::id callback_thread;
};

struct RoutedSubData {
  bool received = false;
};

struct MockTransportState {
  int publish_count = 0;
  int invoke_count = 0;
  std::string last_name;
};

struct FanoutState {
  bool received = false;
  pcl_port_t* pub = nullptr;
};

static void threaded_sub_callback(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
  auto* data = static_cast<ThreadInjectedData*>(ud);
  data->received = true;
  data->value = *static_cast<const int*>(msg->data);
  data->callback_thread = std::this_thread::get_id();
}

TEST(PclExecutor, ExternalThreadPostsCopiedMessageToExecutorThread) {
  ThreadInjectedData sub_data;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "sensor_updates", "SensorMsg",
                                 threaded_sub_callback, ud);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("threaded_subscriber", &sub_cbs, &sub_data);
  ASSERT_NE(sub_c, nullptr);
  ASSERT_EQ(pcl_container_configure(sub_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(sub_c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  ASSERT_EQ(pcl_executor_add(e, sub_c), PCL_OK);

  std::thread::id producer_thread;
  pcl_status_t post_rc = PCL_ERR_INVALID;
  std::thread producer([&]() {
    int payload = 42;
    pcl_msg_t msg = {};
    producer_thread = std::this_thread::get_id();
    msg.data = &payload;
    msg.size = sizeof(payload);
    msg.type_name = "SensorMsg";

    post_rc = pcl_executor_post_incoming(e, "sensor_updates", &msg);

    // Overwrite the source buffer immediately to prove the executor copied it.
    payload = 99;
  });

  producer.join();

  ASSERT_EQ(post_rc, PCL_OK);
  ASSERT_EQ(pcl_executor_spin_once(e, 0), PCL_OK);
  EXPECT_TRUE(sub_data.received);
  EXPECT_EQ(sub_data.value, 42);
  EXPECT_EQ(sub_data.callback_thread, std::this_thread::get_id());

  pcl_executor_destroy(e);
  pcl_container_destroy(sub_c);
}

TEST(PclExecutor, RemoteIngressHonorsSubscriberPeerRoute) {
  RoutedSubData data;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"peer_a"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c,
        "remote/topic",
        "RemoteMsg",
        [](pcl_container_t*, const pcl_msg_t*, void* inner_ud) {
          static_cast<RoutedSubData*>(inner_ud)->received = true;
        },
        ud);
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  auto* c = pcl_container_create("remote_sub", &cbs, &data);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "RemoteMsg";

  ASSERT_EQ(pcl_executor_post_remote_incoming(e, "peer_b", "remote/topic", &msg), PCL_OK);
  EXPECT_EQ(pcl_executor_spin_once(e, 0), PCL_ERR_NOT_FOUND);
  EXPECT_FALSE(data.received);

  ASSERT_EQ(pcl_executor_post_remote_incoming(e, "peer_a", "remote/topic", &msg), PCL_OK);
  EXPECT_EQ(pcl_executor_spin_once(e, 0), PCL_OK);
  EXPECT_TRUE(data.received);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

static pcl_status_t mock_publish(void* ctx, const char* topic, const pcl_msg_t*) {
  auto* state = static_cast<MockTransportState*>(ctx);
  state->publish_count++;
  state->last_name = topic ? topic : "";
  return PCL_OK;
}

static pcl_status_t mock_invoke_async(void*            ctx,
                                      const char*      service_name,
                                      const pcl_msg_t*,
                                      pcl_resp_cb_fn_t callback,
                                      void*            user_data) {
  auto* state = static_cast<MockTransportState*>(ctx);
  pcl_msg_t resp = {};
  state->invoke_count++;
  state->last_name = service_name ? service_name : "";
  resp.data = nullptr;
  resp.size = 0;
  resp.type_name = "Resp";
  callback(&resp, user_data);
  return PCL_OK;
}

TEST(PclExecutor, PublisherRouteCanBeLocalAndRemote) {
  FanoutState state;
  MockTransportState transport_state;
  pcl_transport_t transport = {};
  transport.publish = mock_publish;
  transport.adapter_ctx = &transport_state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* state = static_cast<FanoutState*>(ud);
    const char* peers[] = {"peer_a"};
    pcl_port_t* sub = pcl_container_add_subscriber(
        c,
        "fanout/topic",
        "FanoutMsg",
        [](pcl_container_t*, const pcl_msg_t*, void* inner_ud) {
          static_cast<FanoutState*>(inner_ud)->received = true;
        },
        ud);
    pcl_port_t* pub = pcl_container_add_publisher(c, "fanout/topic", "FanoutMsg");
    if (!sub || !pub) return PCL_ERR_NOMEM;
    state->pub = pub;
    if (pcl_port_set_route(sub, PCL_ROUTE_LOCAL, nullptr, 0) != PCL_OK) return PCL_ERR_INVALID;
    return pcl_port_set_route(pub, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, peers, 1);
  };

  auto* c = pcl_container_create("fanout", &cbs, &state);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);
  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &transport), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "FanoutMsg";

  ASSERT_NE(state.pub, nullptr);
  EXPECT_EQ(pcl_port_publish(state.pub, &msg), PCL_OK);
  EXPECT_TRUE(state.received);
  EXPECT_EQ(transport_state.publish_count, 1);
  EXPECT_EQ(transport_state.last_name, "fanout/topic");

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, ConsumedServiceRouteUsesNamedPeerTransport) {
  MockTransportState transport_state;
  pcl_transport_t transport = {};
  pcl_endpoint_route_t route = {};
  bool callback_fired = false;

  transport.invoke_async = mock_invoke_async;
  transport.adapter_ctx = &transport_state;

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &transport), PCL_OK);

  const char* peers[] = {"peer_a"};
  route.endpoint_name = "remote.echo";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = peers;
  route.peer_count = 1;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  pcl_msg_t req = {};
  req.data = "req";
  req.size = 3;
  req.type_name = "Req";

  EXPECT_EQ(pcl_executor_invoke_async(
                e,
                "remote.echo",
                &req,
                [](const pcl_msg_t*, void* ud) {
                  *static_cast<bool*>(ud) = true;
                },
                &callback_fired),
            PCL_OK);
  EXPECT_TRUE(callback_fired);
  EXPECT_EQ(transport_state.invoke_count, 1);
  EXPECT_EQ(transport_state.last_name, "remote.echo");

  pcl_executor_destroy(e);
}

TEST(PclExecutor, RegisterTransportReplaceAndRemove) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  MockTransportState first_state;
  MockTransportState second_state;
  pcl_transport_t first = {};
  pcl_transport_t second = {};

  first.publish = mock_publish;
  first.adapter_ctx = &first_state;
  second.publish = mock_publish;
  second.adapter_ctx = &second_state;

  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &first), PCL_OK);
  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &second), PCL_OK);

  pcl_endpoint_route_t route = {};
  const char* peers[] = {"peer_a"};
  route.endpoint_name = "remote.topic";
  route.endpoint_kind = PCL_ENDPOINT_PUBLISHER;
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = peers;
  route.peer_count = 1;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  pcl_callbacks_t cbs = {};
  struct PublishCtx {
    pcl_port_t* pub = nullptr;
  } ctx;
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<PublishCtx*>(ud);
    const char* peers[] = {"peer_a"};
    ctx->pub = pcl_container_add_publisher(c, "remote.topic", "Msg");
    return ctx->pub ? pcl_port_set_route(ctx->pub, PCL_ROUTE_REMOTE, peers, 1)
                    : PCL_ERR_NOMEM;
  };

  auto* c = pcl_container_create("replace_transport", &cbs, &ctx);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_OK);
  EXPECT_EQ(first_state.publish_count, 0);
  EXPECT_EQ(second_state.publish_count, 1);

  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", nullptr), PCL_OK);
  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, RegisterTransportDestroyShutsDownNamedTransport) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  bool shutdown_called = false;
  pcl_transport_t transport = {};
  transport.shutdown = [](void* ctx) {
    *static_cast<bool*>(ctx) = true;
  };
  transport.adapter_ctx = &shutdown_called;

  ASSERT_EQ(pcl_executor_register_transport(e, "peer_a", &transport), PCL_OK);
  pcl_executor_destroy(e);
  EXPECT_TRUE(shutdown_called);
}

TEST(PclExecutor, SetEndpointRouteRejectsInvalidConfigurations) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  const char* peers[] = {"peer_a"};
  pcl_endpoint_route_t route = {};
  route.endpoint_name = "svc";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.peer_ids = peers;
  route.peer_count = 1;

  route.route_mode = PCL_ROUTE_LOCAL;
  EXPECT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_ERR_INVALID);

  route.route_mode = PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE;
  EXPECT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_ERR_INVALID);

  const char* bad_peers[] = {nullptr};
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = bad_peers;
  EXPECT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_ERR_INVALID);

  pcl_executor_destroy(e);
}

TEST(PclExecutor, InvokeAsyncConsumedRemoteZeroPeersUsesDefaultTransport) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  MockTransportState transport_state;
  pcl_transport_t transport = {};
  transport.invoke_async = mock_invoke_async;
  transport.adapter_ctx = &transport_state;
  ASSERT_EQ(pcl_executor_set_transport(e, &transport), PCL_OK);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "remote.default";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = nullptr;
  route.peer_count = 0;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  bool callback_fired = false;
  pcl_msg_t req = {};
  req.data = "req";
  req.size = 3;
  req.type_name = "Req";

  EXPECT_EQ(
      pcl_executor_invoke_async(
          e,
          "remote.default",
          &req,
          [](const pcl_msg_t*, void* ud) {
            *static_cast<bool*>(ud) = true;
          },
          &callback_fired),
      PCL_OK);
  EXPECT_TRUE(callback_fired);
  EXPECT_EQ(transport_state.invoke_count, 1);
  EXPECT_EQ(transport_state.last_name, "remote.default");

  pcl_executor_destroy(e);
}

TEST(PclExecutor, InvokeAsyncConsumedRemoteWithoutTransportReturnsNotFound) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "remote.missing";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  route.peer_ids = nullptr;
  route.peer_count = 0;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, void*) {};

  EXPECT_EQ(pcl_executor_invoke_async(e, "remote.missing", &req, cb, nullptr),
            PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

TEST(PclExecutor, InvokeAsyncConsumedInvalidRouteModeReturnsInvalid) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "remote.invalid";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = 4u;
  route.peer_ids = nullptr;
  route.peer_count = 0;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, void*) {};

  EXPECT_EQ(pcl_executor_invoke_async(e, "remote.invalid", &req, cb, nullptr),
            PCL_ERR_INVALID);

  pcl_executor_destroy(e);
}

TEST(PclExecutor, PublisherRemoteWithoutTransportReturnsNotFound) {
  struct PublishCtx {
    pcl_port_t* pub = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<PublishCtx*>(ud);
    ctx->pub = pcl_container_add_publisher(c, "remote/only", "Msg");
    return ctx->pub ? pcl_port_set_route(ctx->pub, PCL_ROUTE_REMOTE, nullptr, 0)
                    : PCL_ERR_NOMEM;
  };

  auto* c = pcl_container_create("remote_only_pub", &cbs, &ctx);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, PublisherRemoteNamedPeerWithoutTransportReturnsNotFound) {
  struct PublishCtx {
    pcl_port_t* pub = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<PublishCtx*>(ud);
    const char* peers[] = {"peer_a"};
    ctx->pub = pcl_container_add_publisher(c, "remote/named", "Msg");
    return ctx->pub ? pcl_port_set_route(ctx->pub, PCL_ROUTE_REMOTE, peers, 1)
                    : PCL_ERR_NOMEM;
  };

  auto* c = pcl_container_create("remote_named_pub", &cbs, &ctx);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  auto* e = pcl_executor_create();
  ASSERT_EQ(pcl_executor_add(e, c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Coverage gap-fill tests ---------------------------------------------

// set_transport with NULL clears the transport (else-branch in pcl_executor_set_transport).
TEST(PclExecutor, SetTransportClearNull) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  bool shutdown_called = false;
  pcl_transport_t t = {};
  t.shutdown = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
  t.adapter_ctx = &shutdown_called;

  ASSERT_EQ(pcl_executor_set_transport(e, &t), PCL_OK);
  // Clear transport — exercises the else branch and return PCL_OK after it.
  ASSERT_EQ(pcl_executor_set_transport(e, nullptr), PCL_OK);
  // Shutdown was NOT called through set_transport; the transport is just cleared.
  EXPECT_FALSE(shutdown_called);

  pcl_executor_destroy(e);
}

// Calling set_endpoint_route twice with the same endpoint name/kind exercises
// the update-existing-entry code path (find_endpoint_route_entry returns non-NULL).
TEST(PclExecutor, UpdateExistingEndpointRoute) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "update.svc";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  route.peer_ids      = nullptr;
  route.peer_count    = 0;

  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  // Second call with the same endpoint — exercises the update (non-NULL entry) path.
  const char* peers[] = {"peer_x"};
  route.peer_ids   = peers;
  route.peer_count = 1;
  ASSERT_EQ(pcl_executor_set_endpoint_route(e, &route), PCL_OK);

  pcl_executor_destroy(e);
}

// invoke_async with no transport and no endpoint route falls back to the
// intra-process service lookup path, allocates a svc_context, fires the
// callback inline (PCL_OK immediate-response path).
TEST(PclExecutor, InvokeAsyncIntraProcessImmediateResponse) {
  static int response_val = 42;
  auto svc_handler = [](pcl_container_t*, const pcl_msg_t*,
                         pcl_msg_t* resp,
                         pcl_svc_context_t*, void*) -> pcl_status_t {
    resp->data = &response_val;
    resp->size = sizeof(response_val);
    return PCL_OK;
  };

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto h = *static_cast<decltype(svc_handler)*>(ud);
    return pcl_container_add_service(c, "inproc.svc", "Msg", h, nullptr)
               ? PCL_OK : PCL_ERR_NOMEM;
  };

  auto* e = pcl_executor_create();
  auto* c = pcl_container_create("svc_node", &cbs, &svc_handler);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_executor_add(e, c);

  // No transport, no endpoint route — goes to intra-process calloc path.
  bool cb_fired = false;
  int  cb_val   = 0;
  pcl_msg_t req = {};
  req.type_name = "Msg";

  auto cb = [](const pcl_msg_t* resp, void* ud) {
    auto* pair = static_cast<std::pair<bool*, int*>*>(ud);
    *pair->first = true;
    if (resp->data && resp->size == sizeof(int))
      *pair->second = *static_cast<const int*>(resp->data);
  };
  std::pair<bool*, int*> cb_ctx{&cb_fired, &cb_val};

  EXPECT_EQ(pcl_executor_invoke_async(e, "inproc.svc", &req, cb, &cb_ctx), PCL_OK);
  EXPECT_TRUE(cb_fired);
  EXPECT_EQ(cb_val, 42);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// invoke_async intra-process with a service that returns PCL_PENDING exercises
// the deferred-response branch (the svc_context_t is saved and not freed here).
TEST(PclExecutor, InvokeAsyncIntraProcessPendingResponse) {
  static pcl_svc_context_t* saved_ctx = nullptr;

  auto svc_handler = [](pcl_container_t*, const pcl_msg_t*,
                         pcl_msg_t*, pcl_svc_context_t* ctx, void*) -> pcl_status_t {
    saved_ctx = ctx;
    return PCL_PENDING;
  };

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto h = *static_cast<decltype(svc_handler)*>(ud);
    return pcl_container_add_service(c, "defer.svc", "Msg", h, nullptr)
               ? PCL_OK : PCL_ERR_NOMEM;
  };

  auto* e = pcl_executor_create();
  auto* c = pcl_container_create("defer_node", &cbs, &svc_handler);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_executor_add(e, c);

  bool cb_fired = false;
  pcl_msg_t req = {};
  req.type_name = "Msg";

  EXPECT_EQ(pcl_executor_invoke_async(e, "defer.svc", &req,
    [](const pcl_msg_t*, void* ud) { *static_cast<bool*>(ud) = true; },
    &cb_fired), PCL_OK);
  EXPECT_FALSE(cb_fired);  // deferred — not yet fired
  ASSERT_NE(saved_ctx, nullptr);

  // Deliver the deferred response and clean up.
  pcl_msg_t resp = {};
  resp.type_name = "Msg";
  pcl_service_respond(saved_ctx, &resp);
  EXPECT_TRUE(cb_fired);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// Publisher port with a transport set and no specific endpoint route causes
// port_route_mode to return PCL_ROUTE_REMOTE (has_transport + PUBLISHER type).
TEST(PclExecutor, PublishPortRouteRemoteWithTransport) {
  struct PubCtx { pcl_port_t* pub = nullptr; } pctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<PubCtx*>(ud);
    ctx->pub = pcl_container_add_publisher(c, "remote.topic", "Msg");
    return ctx->pub ? PCL_OK : PCL_ERR_NOMEM;
  };

  auto* c = pcl_container_create("pub_node", &cbs, &pctx);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // Set a transport with a publish function — now has_transport == 1.
  int pub_count = 0;
  pcl_transport_t t = {};
  t.publish = [](void* ctx, const char*, const pcl_msg_t*) -> pcl_status_t {
    (*static_cast<int*>(ctx))++;
    return PCL_OK;
  };
  t.adapter_ctx = &pub_count;
  ASSERT_EQ(pcl_executor_set_transport(e, &t), PCL_OK);

  // No endpoint route configured — port_route_mode returns PCL_ROUTE_REMOTE
  // because has_transport is true and port type is PUBLISHER.
  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(pcl_port_publish(pctx.pub, &msg), PCL_OK);
  EXPECT_EQ(pub_count, 1);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// -- Null Safety ---------------------------------------------------------

TEST(PclExecutor, NullSafety) {
  EXPECT_EQ(pcl_executor_add(nullptr, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_spin(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_spin_once(nullptr, 0), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_shutdown_graceful(nullptr, 0), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_post_incoming(nullptr, "topic", nullptr), PCL_ERR_INVALID);
}
