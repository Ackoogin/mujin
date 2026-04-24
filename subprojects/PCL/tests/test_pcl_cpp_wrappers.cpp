/// \file test_pcl_cpp_wrappers.cpp
/// \brief Tests for PCL C++ wrappers — pcl::Component and pcl::Executor.
///
/// Covers LLRs REQ_PCL_131–REQ_PCL_157 (tracing to HLRs PCL.048 and PCL.049).

#include <gtest/gtest.h>

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <string>
#include <thread>
#include <vector>

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_131–144 — pcl::Component base class (PCL.048)
// ═══════════════════════════════════════════════════════════════════════════

// -- Minimal subclass for testing ------------------------------------------

class TestComponent : public pcl::Component {
public:
  explicit TestComponent(const char* name = "test_component")
      : Component(name) {}

  int configure_count  = 0;
  int activate_count   = 0;
  int deactivate_count = 0;
  int cleanup_count    = 0;
  int shutdown_count   = 0;
  int tick_count       = 0;
  double last_dt       = 0.0;

  pcl::Port pub_port;
  pcl_status_t pub_route_status = PCL_ERR_STATE;

protected:
  pcl_status_t on_configure() override {
    ++configure_count;
    pub_port = addPublisher("test/output", "TestMsg");
    pub_route_status = pub_port.routeLocal();
    return pub_port ? PCL_OK : PCL_ERR_CALLBACK;
  }

  pcl_status_t on_activate() override {
    ++activate_count;
    return PCL_OK;
  }

  pcl_status_t on_deactivate() override {
    ++deactivate_count;
    return PCL_OK;
  }

  pcl_status_t on_cleanup() override {
    ++cleanup_count;
    return PCL_OK;
  }

  pcl_status_t on_shutdown() override {
    ++shutdown_count;
    return PCL_OK;
  }

  pcl_status_t on_tick(double dt) override {
    ++tick_count;
    last_dt = dt;
    return PCL_OK;
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// Construction and RAII
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_131: Component construction produces valid handle. PCL.048.
TEST(PclCppComponent, ConstructionProducesValidHandle) {
  TestComponent comp("mycomp");
  EXPECT_NE(comp.handle(), nullptr);
  EXPECT_STREQ(comp.name(), "mycomp");
  EXPECT_EQ(comp.state(), PCL_STATE_UNCONFIGURED);
}

///< REQ_PCL_132: Component destructor frees resources. PCL.048.
TEST(PclCppComponent, DestructorFreesResources) {
  {
    TestComponent comp;
    comp.configure();
    comp.activate();
    // destructor fires here
  }
  // If we get here without crash or ASAN error, RAII works.
  SUCCEED();
}

// ═══════════════════════════════════════════════════════════════════════════
// Virtual lifecycle callbacks
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_133: Component lifecycle callbacks invoked. PCL.048.
TEST(PclCppComponent, LifecycleCallbacksInvoked) {
  TestComponent comp;

  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_EQ(comp.configure_count, 1);
  EXPECT_EQ(comp.state(), PCL_STATE_CONFIGURED);

  EXPECT_EQ(comp.activate(), PCL_OK);
  EXPECT_EQ(comp.activate_count, 1);
  EXPECT_EQ(comp.state(), PCL_STATE_ACTIVE);

  EXPECT_EQ(comp.deactivate(), PCL_OK);
  EXPECT_EQ(comp.deactivate_count, 1);
  EXPECT_EQ(comp.state(), PCL_STATE_CONFIGURED);

  EXPECT_EQ(comp.cleanup(), PCL_OK);
  EXPECT_EQ(comp.cleanup_count, 1);
  EXPECT_EQ(comp.state(), PCL_STATE_UNCONFIGURED);

  EXPECT_EQ(comp.shutdown(), PCL_OK);
  EXPECT_EQ(comp.shutdown_count, 1);
  EXPECT_EQ(comp.state(), PCL_STATE_FINALIZED);
}

// ═══════════════════════════════════════════════════════════════════════════
// Port creation helpers
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_134: addPublisher during configure. PCL.048.
TEST(PclCppComponent, AddPublisherDuringConfigure) {
  TestComponent comp;
  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_TRUE(comp.pub_port.valid());
  EXPECT_EQ(comp.pub_route_status, PCL_OK);
}

///< REQ_PCL_135: addSubscriber during configure. PCL.048.
TEST(PclCppComponent, AddSubscriberDuringConfigure) {
  struct SubComp : public pcl::Component {
    pcl::Port sub_port;
    bool msg_received = false;

    SubComp() : Component("sub_comp") {}

  protected:
    pcl_status_t on_configure() override {
      sub_port = addSubscriber("test/input", "TestMsg",
          [](pcl_container_t*, const pcl_msg_t*, void* ud) {
            static_cast<SubComp*>(ud)->msg_received = true;
          }, this);
      return sub_port ? PCL_OK : PCL_ERR_CALLBACK;
    }
  };

  SubComp comp;
  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_NE(comp.sub_port, nullptr);
}

///< REQ_PCL_136: addService during configure. PCL.048.
TEST(PclCppComponent, AddServiceDuringConfigure) {
  struct SvcComp : public pcl::Component {
    pcl::Port svc_port;

    SvcComp() : Component("svc_comp") {}

  protected:
    pcl_status_t on_configure() override {
      svc_port = addService("test/service", "SvcReq",
          [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t*,
             pcl_svc_context_t*, void*) -> pcl_status_t {
            return PCL_OK;
          }, this);
      return svc_port ? PCL_OK : PCL_ERR_CALLBACK;
    }
  };

  SvcComp comp;
  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_TRUE(comp.svc_port.valid());
}

///< REQ_PCL_136A: addStreamService during configure. PCL.048.
TEST(PclCppComponent, AddStreamServiceDuringConfigure) {
  struct StreamComp : public pcl::Component {
    pcl::Port stream_port;

    StreamComp() : Component("stream_comp") {}

  protected:
    pcl_status_t on_configure() override {
      stream_port = addStreamService(
          "test/stream", "StreamMsg",
          [](pcl_container_t*, const pcl_msg_t*, pcl_stream_context_t*,
             void*) -> pcl_status_t {
            return PCL_STREAMING;
          }, this);
      return stream_port ? PCL_OK : PCL_ERR_CALLBACK;
    }
  };

  StreamComp comp;
  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_TRUE(comp.stream_port.valid());
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameter helpers
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_137: Parameter string round-trip. PCL.048.
TEST(PclCppComponent, ParamStringRoundTrip) {
  TestComponent comp;
  comp.setParam("name", "Alice");
  EXPECT_EQ(comp.paramStr("name"), "Alice");
  EXPECT_EQ(comp.paramStr("missing", "default"), "default");
}

///< REQ_PCL_138: Parameter double round-trip. PCL.048.
TEST(PclCppComponent, ParamDoubleRoundTrip) {
  TestComponent comp;
  comp.setParam("rate", 42.5);
  EXPECT_DOUBLE_EQ(comp.paramF64("rate"), 42.5);
  EXPECT_DOUBLE_EQ(comp.paramF64("missing", 99.0), 99.0);
}

///< REQ_PCL_139: Parameter int64 round-trip. PCL.048.
TEST(PclCppComponent, ParamInt64RoundTrip) {
  TestComponent comp;
  comp.setParam("count", static_cast<int64_t>(100));
  EXPECT_EQ(comp.paramI64("count"), 100);
  EXPECT_EQ(comp.paramI64("missing", -1), -1);
}

///< REQ_PCL_140: Parameter bool round-trip. PCL.048.
TEST(PclCppComponent, ParamBoolRoundTrip) {
  TestComponent comp;
  comp.setParam("flag", true);
  EXPECT_EQ(comp.paramBool("flag"), true);
  EXPECT_EQ(comp.paramBool("missing", false), false);
}

// ═══════════════════════════════════════════════════════════════════════════
// Tick rate helpers
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_141: Tick rate round-trip. PCL.048.
TEST(PclCppComponent, TickRateRoundTrip) {
  TestComponent comp;
  EXPECT_DOUBLE_EQ(comp.tickRateHz(), 100.0); // default
  comp.setTickRateHz(25.0);
  EXPECT_DOUBLE_EQ(comp.tickRateHz(), 25.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Logging helpers
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_142: Logging does not crash. PCL.048.
TEST(PclCppComponent, LoggingDoesNotCrash) {
  // Install a handler that captures messages.
  struct LogCapture {
    std::vector<std::string> messages;
    static void handler(pcl_log_level_t, const char*, const char* msg, void* ud) {
      static_cast<LogCapture*>(ud)->messages.emplace_back(msg ? msg : "");
    }
  } capture;

  pcl_log_set_handler(LogCapture::handler, &capture);
  pcl_log_set_level(PCL_LOG_DEBUG);

  TestComponent comp("logger");
  comp.logDebug("debug %d", 1);
  comp.logInfo("info %s", "test");
  comp.logWarn("warn %.1f", 3.14);
  comp.logError("error %s", "oops");

  EXPECT_GE(capture.messages.size(), 4u);

  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

// ═══════════════════════════════════════════════════════════════════════════
// Move semantics
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_143: Component move constructor transfers ownership. PCL.048.
TEST(PclCppComponent, MoveConstructor) {
  TestComponent a("moveable");
  pcl_container_t* h = a.handle();
  EXPECT_NE(h, nullptr);

  TestComponent b(std::move(a));
  EXPECT_EQ(b.handle(), h);
  EXPECT_EQ(a.handle(), nullptr);
}

///< REQ_PCL_144: Component move assignment transfers ownership. PCL.048.
TEST(PclCppComponent, MoveAssignment) {
  TestComponent a("src");
  TestComponent b("dst");

  pcl_container_t* ha = a.handle();
  b = std::move(a);
  EXPECT_EQ(b.handle(), ha);
  EXPECT_EQ(a.handle(), nullptr);
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_145–156 — pcl::Executor wrapper (PCL.049)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_145: Executor construction produces valid handle. PCL.049.
TEST(PclCppExecutor, ConstructionProducesValidHandle) {
  pcl::Executor exec;
  EXPECT_NE(exec.handle(), nullptr);
}

///< REQ_PCL_146: Executor destructor frees resources. PCL.049.
TEST(PclCppExecutor, DestructorFreesResources) {
  {
    pcl::Executor exec;
    // destructor fires here
  }
  SUCCEED();
}

///< REQ_PCL_147: Executor add component. PCL.049.
TEST(PclCppExecutor, AddComponent) {
  pcl::Executor exec;
  TestComponent comp;
  comp.configure();
  comp.activate();

  EXPECT_EQ(exec.add(comp), PCL_OK);
  EXPECT_EQ(exec.remove(comp), PCL_OK);
}

///< REQ_PCL_148: Executor add raw container. PCL.049.
TEST(PclCppExecutor, AddRawContainer) {
  pcl::Executor exec;
  auto* c = pcl_container_create("raw", nullptr, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);

  EXPECT_EQ(exec.add(c), PCL_OK);

  // exec destructor does NOT destroy containers — caller owns them.
  pcl_container_destroy(c);
}

///< REQ_PCL_149: Executor spinOnce ticks component. PCL.049.
TEST(PclCppExecutor, SpinOnceTicksComponent) {
  pcl::Executor exec;
  TestComponent comp;
  comp.configure();
  comp.activate();
  comp.setTickRateHz(100.0);
  exec.add(comp);

  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    exec.spinOnce(0);
  }

  EXPECT_GT(comp.tick_count, 0);
}

///< REQ_PCL_150: Executor requestShutdown stops spin. PCL.049.
TEST(PclCppExecutor, RequestShutdownStopsSpin) {
  pcl::Executor exec;
  TestComponent comp;
  comp.configure();
  comp.activate();
  exec.add(comp);

  std::thread spinner([&exec]() {
    exec.spin();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  exec.requestShutdown();
  spinner.join(); // should return promptly
  SUCCEED();
}

///< REQ_PCL_151: Executor shutdownGraceful finalizes components. PCL.049.
TEST(PclCppExecutor, ShutdownGracefulFinalizesComponents) {
  pcl::Executor exec;
  TestComponent comp;
  comp.configure();
  comp.activate();
  exec.add(comp);

  EXPECT_EQ(exec.shutdownGraceful(5000), PCL_OK);
  EXPECT_EQ(comp.state(), PCL_STATE_FINALIZED);
  EXPECT_EQ(comp.deactivate_count, 1);
}

///< REQ_PCL_152: Executor setTransport wires adapter. PCL.049.
TEST(PclCppExecutor, SetTransport) {
  pcl::Executor exec;

  // Set a dummy transport and verify no crash.
  pcl_transport_t dummy = {};
  dummy.publish = [](void*, const char*, const pcl_msg_t*) -> pcl_status_t {
    return PCL_OK;
  };
  dummy.adapter_ctx = nullptr;

  EXPECT_EQ(exec.setTransport(&dummy), PCL_OK);
  EXPECT_EQ(exec.setTransport(nullptr), PCL_OK); // revert
}

///< REQ_PCL_152A: Executor registerTransport accepts named peers. PCL.049.
TEST(PclCppExecutor, RegisterTransport) {
  pcl::Executor exec;
  pcl_transport_t dummy = {};
  dummy.publish = [](void*, const char*, const pcl_msg_t*) -> pcl_status_t {
    return PCL_OK;
  };

  EXPECT_EQ(exec.registerTransport("peer_a", &dummy), PCL_OK);
  EXPECT_EQ(exec.registerTransport("peer_a", nullptr), PCL_OK);
}

///< REQ_PCL_153: Executor dispatchIncoming routes to subscribers. PCL.049.
TEST(PclCppExecutor, DispatchIncoming) {
  pcl::Executor exec;

  bool received = false;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "test/topic", "Msg",
        [](pcl_container_t*, const pcl_msg_t*, void* u) {
          *static_cast<bool*>(u) = true;
        }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("sub", &cbs, &received);
  pcl_container_configure(c);
  pcl_container_activate(c);
  exec.add(c);

  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(exec.dispatchIncoming("test/topic", &msg), PCL_OK);
  EXPECT_TRUE(received);

  pcl_container_destroy(c);
}

///< REQ_PCL_154: Executor postIncoming queues from external thread. PCL.049.
TEST(PclCppExecutor, PostIncoming) {
  pcl::Executor exec;

  std::atomic<bool> received{false};
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "ext/topic", "Msg",
        [](pcl_container_t*, const pcl_msg_t*, void* u) {
          static_cast<std::atomic<bool>*>(u)->store(true);
        }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("sub", &cbs, &received);
  pcl_container_configure(c);
  pcl_container_activate(c);
  exec.add(c);

  pcl_msg_t msg = {};
  msg.data = "y";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(exec.postIncoming("ext/topic", &msg), PCL_OK);
  exec.spinOnce(0);
  EXPECT_TRUE(received);

  pcl_container_destroy(c);
}

///< REQ_PCL_154A: Executor postRemoteIncoming honours remote routing. PCL.049.
TEST(PclCppExecutor, PostRemoteIncoming) {
  struct RemoteSubComp : public pcl::Component {
    pcl::Port sub_port;
    int received = 0;

    RemoteSubComp() : Component("remote_sub") {}

  protected:
    pcl_status_t on_configure() override {
      sub_port = addSubscriber(
          "remote/topic", "Msg",
          [](pcl_container_t*, const pcl_msg_t*, void* u) {
            static_cast<RemoteSubComp*>(u)->received++;
          }, this);
      if (!sub_port) return PCL_ERR_CALLBACK;
      return sub_port.routeRemote("peer_a");
    }
  };

  pcl::Executor exec;
  RemoteSubComp comp;
  EXPECT_EQ(comp.configure(), PCL_OK);
  EXPECT_EQ(comp.activate(), PCL_OK);
  EXPECT_EQ(exec.add(comp), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "z";
  msg.size = 1;
  msg.type_name = "Msg";

  EXPECT_EQ(exec.postRemoteIncoming("peer_b", "remote/topic", &msg), PCL_OK);
  exec.spinOnce(0);
  EXPECT_EQ(comp.received, 0);

  EXPECT_EQ(exec.postRemoteIncoming("peer_a", "remote/topic", &msg), PCL_OK);
  exec.spinOnce(0);
  EXPECT_EQ(comp.received, 1);
}

///< REQ_PCL_154B: Executor endpoint route helpers mirror C API. PCL.049.
TEST(PclCppExecutor, EndpointRouteHelpers) {
  pcl::Executor exec;
  EXPECT_EQ(exec.routeRemote("svc.remote", "peer_a"), PCL_OK);
  EXPECT_EQ(exec.routeLocal("svc.local", PCL_ENDPOINT_PROVIDED), PCL_OK);
  EXPECT_EQ(exec.routeLocalAndRemote("topic.remote",
                                     "peer_b",
                                     PCL_ENDPOINT_PUBLISHER), PCL_OK);
}

///< REQ_PCL_155: Executor move constructor transfers ownership. PCL.049.
TEST(PclCppExecutor, MoveConstructor) {
  pcl::Executor a;
  pcl_executor_t* h = a.handle();
  EXPECT_NE(h, nullptr);

  pcl::Executor b(std::move(a));
  EXPECT_EQ(b.handle(), h);
  EXPECT_EQ(a.handle(), nullptr);
}

///< REQ_PCL_156: Executor move assignment transfers ownership. PCL.049.
TEST(PclCppExecutor, MoveAssignment) {
  pcl::Executor a;
  pcl::Executor b;
  pcl_executor_t* ha = a.handle();

  b = std::move(a);
  EXPECT_EQ(b.handle(), ha);
  EXPECT_EQ(a.handle(), nullptr);
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration: Component + Executor together
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_157: Component lifecycle via executor integration. PCL.048, PCL.049.
TEST(PclCppIntegration, ComponentLifecycleViaExecutor) {
  pcl::Executor exec;
  TestComponent comp("integrated");

  comp.setParam("threshold", 42.5);
  EXPECT_DOUBLE_EQ(comp.paramF64("threshold"), 42.5);

  comp.configure();
  comp.activate();
  comp.setTickRateHz(200.0);

  exec.add(comp);

  // Run a few spin cycles.
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    exec.spinOnce(0);
  }

  EXPECT_GT(comp.tick_count, 0);

  exec.shutdownGraceful(2000);
  EXPECT_EQ(comp.state(), PCL_STATE_FINALIZED);
}
