#include <pcl/pcl_process_runtime.h>

#include <gtest/gtest.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::string uniqueTempPath() {
#ifdef _WIN32
  char temp_dir[MAX_PATH];
  char file_name[MAX_PATH];
  GetTempPathA(MAX_PATH, temp_dir);
  GetTempFileNameA(temp_dir, "ppr", 0, file_name);
  return file_name;
#else
  char path[] = "/tmp/pcl_process_runtime_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  return path;
#endif
}

std::string writePortsFile(const std::string& body) {
  const std::string path = uniqueTempPath();
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  EXPECT_TRUE(output.good());
  output << body;
  output.close();
  return path;
}

struct Runtime {
  explicit Runtime(uint32_t duration = 0u) {
    EXPECT_EQ(pcl_process_runtime_create(duration, &value), PCL_OK);
  }

  ~Runtime() {
    pcl_process_runtime_destroy(value);
  }

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  pcl_process_runtime_t* value = nullptr;
};

const pcl_process_endpoint_descriptor_t kRpcEndpoints[] = {
    {"fixture.provided", PCL_ENDPOINT_PROVIDED},
    {"fixture.consumed", PCL_ENDPOINT_CONSUMED},
    {"fixture.stream_provided", PCL_ENDPOINT_STREAM_PROVIDED},
    {"fixture.stream_consumed", PCL_ENDPOINT_STREAM_CONSUMED},
};

const pcl_process_endpoint_descriptor_t kPubsubEndpoints[] = {
    {"fixture.publisher", PCL_ENDPOINT_PUBLISHER},
    {"fixture.subscriber", PCL_ENDPOINT_SUBSCRIBER},
};

const pcl_process_port_descriptor_t kPort = {
    "fixture_request",
    kRpcEndpoints,
    sizeof(kRpcEndpoints) / sizeof(kRpcEndpoints[0]),
    kPubsubEndpoints,
    sizeof(kPubsubEndpoints) / sizeof(kPubsubEndpoints[0]),
};

pcl_status_t stopOnTick(
    pcl_container_t*,
    double,
    void* user_data) {
  pcl_process_runtime_request_shutdown(
      static_cast<pcl_process_runtime_t*>(user_data));
  return PCL_OK;
}

pcl_status_t failConfigure(pcl_container_t*, void*) {
  return PCL_ERR_STATE;
}

pcl_status_t failActivate(pcl_container_t*, void*) {
  return PCL_ERR_STATE;
}

pcl_status_t failDeactivate(pcl_container_t*, void*) {
  return PCL_ERR_STATE;
}

pcl_status_t failCleanup(pcl_container_t*, void*) {
  return PCL_ERR_STATE;
}

uint32_t processId() {
#ifdef _WIN32
  return static_cast<uint32_t>(GetCurrentProcessId());
#else
  return static_cast<uint32_t>(getpid());
#endif
}

}  // namespace

///< REQ_PCL_476, REQ_PCL_225: creating a process runtime creates its executor, and null-safe accessors fail closed. PCL.079.
TEST(PclProcessRuntime, CreatesExecutorAndHandlesNullArguments) {
  pcl_process_runtime_t* runtime = nullptr;
  EXPECT_EQ(pcl_process_runtime_create(0u, nullptr), PCL_ERR_INVALID);
  ASSERT_EQ(pcl_process_runtime_create(0u, &runtime), PCL_OK);
  ASSERT_NE(runtime, nullptr);
  EXPECT_NE(pcl_process_runtime_executor(runtime), nullptr);
  EXPECT_EQ(pcl_process_runtime_executor(nullptr), nullptr);
  EXPECT_EQ(std::string(pcl_process_runtime_error(nullptr)),
            "process runtime is null");

  EXPECT_EQ(pcl_process_runtime_load_codec(nullptr, STUB_CODEC_PLUGIN_PATH),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_codec(runtime, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_codec(runtime, ""), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                nullptr, "unused", &kPort, 1u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime, nullptr, &kPort, 1u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime, "unused", nullptr, 1u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime, "unused", &kPort, 0u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime, "unused", &kPort, 129u),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_run(nullptr, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_process_runtime_run(runtime, nullptr), PCL_ERR_INVALID);

  pcl_process_runtime_request_shutdown(nullptr);
  pcl_process_runtime_destroy(nullptr);
  pcl_process_runtime_destroy(runtime);
}

///< REQ_PCL_477, REQ_PCL_226: codec loading rejects missing plugins with a useful diagnostic. PCL.079.
TEST(PclProcessRuntime, LoadsCodecAndReportsPluginFailure) {
  Runtime runtime;
  EXPECT_EQ(pcl_process_runtime_load_codec(
                runtime.value, "/no/such/runtime-codec-plugin.so"),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("load codec plugin"),
            std::string::npos);
  EXPECT_EQ(pcl_process_runtime_load_codec(
                runtime.value, STUB_CODEC_PLUGIN_PATH),
            PCL_OK)
      << pcl_process_runtime_error(runtime.value);
  EXPECT_EQ(std::string(pcl_process_runtime_error(runtime.value)), "");
}

///< REQ_PCL_478: a complete per-port deployment file is translated into a routing manifest. PCL.079.
TEST(PclProcessRuntime, LoadsCompletePortsFileAndRejectsSecondLoad) {
  Runtime runtime;
  const pcl_process_endpoint_descriptor_t event_rpc[] = {
      {"fixture.events.call", PCL_ENDPOINT_CONSUMED},
  };
  const pcl_process_endpoint_descriptor_t event_pubsub[] = {
      {"fixture.events.publisher", PCL_ENDPOINT_PUBLISHER},
      {"fixture.events.subscriber", PCL_ENDPOINT_SUBSCRIBER},
  };
  const pcl_process_port_descriptor_t ports[] = {
      kPort,
      {"fixture_events",
       event_rpc,
       sizeof(event_rpc) / sizeof(event_rpc[0]),
       event_pubsub,
       sizeof(event_pubsub) / sizeof(event_pubsub[0])},
  };
  const std::string plugin_config =
      " {\"bus_name\":\"pcl_runtime_complete_" +
      std::to_string(processId()) +
      "\",\"participant_id\":\"runtime_complete\"}";
  const std::string body =
      std::string("# generated deployment\n\n") +
      "  port fixture_request rpc shared " + SHM_TRANSPORT_PLUGIN_PATH +
      plugin_config + "  \n" +
      "port fixture_events pubsub shared " + SHM_TRANSPORT_PLUGIN_PATH +
      plugin_config + "\n";
  const std::string path = writePortsFile(body);

  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), ports, 2u),
            PCL_OK)
      << pcl_process_runtime_error(runtime.value);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), ports, 2u),
            PCL_ERR_STATE);

  std::remove(path.c_str());
}

///< REQ_PCL_477: a `codec` line in the ports file loads the plugin and selects the wire content type. PCL.079.
TEST(PclProcessRuntime, SelectsCodecFromPortsFile) {
  // Default before any ports file is loaded.
  {
    Runtime runtime;
    EXPECT_EQ(std::string(pcl_process_runtime_content_type(runtime.value)),
              "application/json");
  }

  const std::string plugin_config =
      " {\"bus_name\":\"pcl_runtime_codec_" +
      std::to_string(processId()) +
      "\",\"participant_id\":\"runtime_codec\"}";
  const std::string body =
      std::string("codec application/pcl-ports-test ") +
      PORTS_CODEC_PLUGIN_PATH + "\n" +
      "port fixture_request rpc shared " + SHM_TRANSPORT_PLUGIN_PATH +
      plugin_config + "\n";
  const std::string path = writePortsFile(body);

  // First load registers the selected codec and records its content type.
  {
    Runtime runtime;
    EXPECT_EQ(pcl_process_runtime_load_ports_file(
                  runtime.value, path.c_str(), &kPort, 1u),
              PCL_OK)
        << pcl_process_runtime_error(runtime.value);
    EXPECT_EQ(std::string(pcl_process_runtime_content_type(runtime.value)),
              "application/pcl-ports-test");
  }

  // A later process-lifetime load finds the same codec vtable already
  // registered (as a build-time fallback would leave it) and tolerates it,
  // still recording the content type the `codec` line selected.
  {
    Runtime runtime;
    EXPECT_EQ(pcl_process_runtime_load_ports_file(
                  runtime.value, path.c_str(), &kPort, 1u),
              PCL_OK)
        << pcl_process_runtime_error(runtime.value);
    EXPECT_EQ(std::string(pcl_process_runtime_content_type(runtime.value)),
              "application/pcl-ports-test");
  }

  std::remove(path.c_str());
}

///< REQ_PCL_477: `port_codec` lines give individual ports their own codec, and the first `codec` line is the default. PCL.079.
TEST(PclProcessRuntime, SelectsPerPortCodecFromPortsFile) {
  const std::string plugin_config =
      " {\"bus_name\":\"pcl_runtime_perport_" +
      std::to_string(processId()) +
      "\",\"participant_id\":\"runtime_perport\"}";
  const std::string body =
      std::string("codec application/pcl-ports-test ") +
      PORTS_CODEC_PLUGIN_PATH + "\n" +
      "codec application/pcl-ports-test-2 " + PORTS2_CODEC_PLUGIN_PATH + "\n" +
      "port_codec fixture_request application/pcl-ports-test-2\n" +
      "port fixture_request rpc shared " + SHM_TRANSPORT_PLUGIN_PATH +
      plugin_config + "\n";
  const std::string path = writePortsFile(body);

  Runtime runtime;
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &kPort, 1u),
            PCL_OK)
      << pcl_process_runtime_error(runtime.value);
  // First codec line is the process-wide default.
  EXPECT_EQ(std::string(pcl_process_runtime_content_type(runtime.value)),
            "application/pcl-ports-test");
  // The overridden port uses its own codec.
  EXPECT_EQ(std::string(pcl_process_runtime_port_content_type(
                runtime.value, "fixture_request")),
            "application/pcl-ports-test-2");
  // A port without an override falls back to the default.
  EXPECT_EQ(std::string(pcl_process_runtime_port_content_type(
                runtime.value, "unoverridden_port")),
            "application/pcl-ports-test");

  std::remove(path.c_str());
}

///< REQ_PCL_477: a `port_codec` line is rejected when it names an unknown port or an unloaded codec. PCL.079.
TEST(PclProcessRuntime, RejectsInvalidPortCodecLines) {
  const std::string good_port =
      std::string("port fixture_request rpc peer ") + CAPTURE_PLUGIN_PATH +
      " {}\n";
  const std::string unknown_port =
      std::string("codec application/pcl-ports-test ") +
      PORTS_CODEC_PLUGIN_PATH + "\n" +
      "port_codec no_such_port application/pcl-ports-test\n" + good_port;
  const std::string unloaded_codec =
      std::string("codec application/pcl-ports-test ") +
      PORTS_CODEC_PLUGIN_PATH + "\n" +
      "port_codec fixture_request application/never-loaded\n" + good_port;

  for (const auto& entry : {std::make_pair(unknown_port, "unknown port"),
                            std::make_pair(unloaded_codec,
                                           "no codec line loaded")}) {
    Runtime runtime;
    const std::string path = writePortsFile(entry.first);
    EXPECT_EQ(pcl_process_runtime_load_ports_file(
                  runtime.value, path.c_str(), &kPort, 1u),
              PCL_ERR_INVALID)
        << entry.first;
    EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                  .find(entry.second),
              std::string::npos)
        << pcl_process_runtime_error(runtime.value);
    std::remove(path.c_str());
  }
}

///< REQ_PCL_477: a `codec` line is rejected when its plugin loads but provides a different content type. PCL.079.
TEST(PclProcessRuntime, RejectsCodecLineWhosePluginProvidesAnotherContentType) {
  // The plugin here is a real, loadable codec; it simply registers
  // "application/pcl-ports-test-3" rather than the content type the line asks
  // for. Without a config-time check this would be accepted, and the process
  // would only fail later when a port tried to bind an unregistered codec.
  // No other test loads this plugin, so the load below is always the first
  // one and always takes the mismatch path rather than the duplicate-load one.
  const std::string body =
      std::string("codec application/not-provided-by-this-plugin ") +
      PORTS3_CODEC_PLUGIN_PATH + "\n" +
      "port fixture_request rpc peer " + CAPTURE_PLUGIN_PATH + " {}\n";

  Runtime runtime;
  const std::string path = writePortsFile(body);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &kPort, 1u),
            PCL_ERR_INVALID);
  const std::string error = pcl_process_runtime_error(runtime.value);
  // The diagnostic must name both halves of the mismatch, since either one
  // could be the typo.
  EXPECT_NE(error.find("does not provide content type"), std::string::npos)
      << error;
  EXPECT_NE(error.find("application/not-provided-by-this-plugin"),
            std::string::npos)
      << error;
  EXPECT_NE(error.find(PORTS3_CODEC_PLUGIN_PATH), std::string::npos) << error;
  std::remove(path.c_str());
}

///< REQ_PCL_477: a `codec` line naming a plugin that does not exist fails closed. PCL.079.
TEST(PclProcessRuntime, RejectsCodecLineWithMissingPlugin) {
  const std::string body =
      // The content type must be one nothing has registered, so the
      // already-registered tolerance path cannot mask the missing plugin.
      std::string("codec application/pcl-absent-codec "
                  "/no/such/codec-plugin.so\n") +
      "port fixture_request rpc peer " + CAPTURE_PLUGIN_PATH + " {}\n";

  Runtime runtime;
  const std::string path = writePortsFile(body);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &kPort, 1u),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("load codec plugin"),
            std::string::npos)
      << pcl_process_runtime_error(runtime.value);
  std::remove(path.c_str());
}

///< REQ_PCL_479, REQ_PCL_227: a discovered shared-memory gateway is activated and cleaned up. PCL.079.
TEST(PclProcessRuntime, ActivatesAndCleansUpSharedMemoryGateway) {
  Runtime runtime;
  const pcl_process_endpoint_descriptor_t rpc[] = {
      {"fixture.gateway.call", PCL_ENDPOINT_CONSUMED},
  };
  const pcl_process_endpoint_descriptor_t pubsub[] = {
      {"fixture.gateway.request", PCL_ENDPOINT_PUBLISHER},
  };
  const pcl_process_port_descriptor_t port = {
      "gateway_request", rpc, 1u, pubsub, 1u,
  };
  const std::string bus =
      "pcl_runtime_gateway_" + std::to_string(processId());
  const std::string body =
      std::string("port gateway_request rpc shm_peer ") +
      SHM_TRANSPORT_PLUGIN_PATH + " {\"bus_name\":\"" + bus +
      "\",\"participant_id\":\"runtime_gateway\"}\n";
  const std::string path = writePortsFile(body);

  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &port, 1u),
            PCL_OK)
      << pcl_process_runtime_error(runtime.value);

  std::remove(path.c_str());
}

///< REQ_PCL_480: deployment descriptors reject invalid logical-port definitions. PCL.079.
TEST(PclProcessRuntime, RejectsInvalidDeploymentDescriptors) {
  Runtime runtime;
  const pcl_process_port_descriptor_t empty = {
      "", kRpcEndpoints, 1u, kPubsubEndpoints, 1u,
  };
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, "unused", &empty, 1u),
            PCL_ERR_INVALID);

  const pcl_process_port_descriptor_t duplicate[] = {kPort, kPort};
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, "unused", duplicate, 2u),
            PCL_ERR_INVALID);

  const pcl_process_port_descriptor_t null_array = {
      "null_array", nullptr, 1u, kPubsubEndpoints, 1u,
  };
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, "unused", &null_array, 1u),
            PCL_ERR_INVALID);
}

///< REQ_PCL_481: invalid deployment-file entries fail closed with a diagnostic. PCL.079.
TEST(PclProcessRuntime, RejectsInvalidPortsFileEntries) {
  const std::vector<std::string> bodies = {
      "garbage\n",
      std::string("thing fixture_request rpc peer ") +
          CAPTURE_PLUGIN_PATH + " {}\n",
      std::string("port fixture_request invalid peer ") +
          CAPTURE_PLUGIN_PATH + " {}\n",
      std::string("port fixture_request rpc peer ") +
          CAPTURE_PLUGIN_PATH + "\n",
      std::string("port unknown rpc peer ") +
          CAPTURE_PLUGIN_PATH + " {}\n",
      std::string("port fixture_request rpc peer ") +
          CAPTURE_PLUGIN_PATH + " one\n" +
          "port fixture_request rpc peer " + CAPTURE_PLUGIN_PATH + " one\n",
      "# fixture_request is missing\n",
      "codec application/json\n",
  };

  for (const std::string& body : bodies) {
    Runtime runtime;
    const std::string path = writePortsFile(body);
    EXPECT_EQ(pcl_process_runtime_load_ports_file(
                  runtime.value, path.c_str(), &kPort, 1u),
              PCL_ERR_INVALID)
        << body;
    EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value)), "");
    std::remove(path.c_str());
  }

  const pcl_process_port_descriptor_t two_ports[] = {
      kPort,
      {"fixture_events", kRpcEndpoints, 1u, kPubsubEndpoints, 1u},
  };
  const std::string conflicting =
      std::string("port fixture_request rpc peer ") +
      CAPTURE_PLUGIN_PATH + " first\n" +
      "port fixture_events pubsub peer " + CAPTURE_PLUGIN_PATH + " second\n";
  Runtime runtime;
  const std::string path = writePortsFile(conflicting);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), two_ports, 2u),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("conflicting"),
            std::string::npos);
  std::remove(path.c_str());
}

///< REQ_PCL_482: a deployment is bounded to 64 distinct transport peers. PCL.079.
TEST(PclProcessRuntime, RejectsMoreThanMaximumPeers) {
  std::vector<std::string> names;
  std::vector<pcl_process_port_descriptor_t> ports;
  names.reserve(65u);
  ports.reserve(65u);
  std::string body;
  for (size_t index = 0u; index < 65u; ++index) {
    names.push_back("port_" + std::to_string(index));
    ports.push_back({
        names.back().c_str(), kRpcEndpoints, 1u, kPubsubEndpoints, 1u,
    });
    body += "port " + names.back() + " rpc peer_" +
            std::to_string(index) + " " + CAPTURE_PLUGIN_PATH + " config\n";
  }
  Runtime runtime;
  const std::string path = writePortsFile(body);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), ports.data(), ports.size()),
            PCL_ERR_NOMEM);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("too many configured peers"),
            std::string::npos);
  std::remove(path.c_str());
}

///< REQ_PCL_483: invalid endpoint descriptors fail before manifest output. PCL.079.
TEST(PclProcessRuntime, RejectsUnsupportedEndpointDescriptors) {
  const pcl_process_endpoint_descriptor_t bad_endpoints[] = {
      {nullptr, PCL_ENDPOINT_CONSUMED},
      {"fixture.invalid", static_cast<pcl_endpoint_kind_t>(999)},
  };
  for (const auto& endpoint : bad_endpoints) {
    const pcl_process_port_descriptor_t port = {
        "bad_endpoint", &endpoint, 1u, kPubsubEndpoints, 1u,
    };
    Runtime runtime;
    const std::string body =
        std::string("port bad_endpoint rpc capture ") +
        CAPTURE_PLUGIN_PATH + " config\n";
    const std::string path = writePortsFile(body);
    EXPECT_EQ(pcl_process_runtime_load_ports_file(
                  runtime.value, path.c_str(), &port, 1u),
              PCL_ERR_INVALID);
    EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                  .find("unsupported endpoint"),
              std::string::npos);
    std::remove(path.c_str());
  }

  const pcl_process_endpoint_descriptor_t bad_pubsub = {
      nullptr, PCL_ENDPOINT_PUBLISHER,
  };
  const pcl_process_port_descriptor_t port = {
      "bad_pubsub", kRpcEndpoints, 1u, &bad_pubsub, 1u,
  };
  Runtime runtime;
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, "unused", &port, 1u),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("unsupported endpoint"),
            std::string::npos);
}

///< REQ_PCL_484: generated routing failures remain atomic and diagnostic. PCL.079.
TEST(PclProcessRuntime, ReportsGeneratedRoutingFailure) {
  Runtime runtime;
  const std::string path = writePortsFile(
      "port fixture_request rpc missing /no/such/transport.so config\n");
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &kPort, 1u),
            PCL_ERR_NOT_FOUND);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("load port config"),
            std::string::npos);
  std::remove(path.c_str());
}

#ifndef _WIN32
///< REQ_PCL_485: temporary routing-manifest creation failures are reported. PCL.079.
TEST(PclProcessRuntime, ReportsTemporaryManifestCreationFailure) {
  const char* old_tmpdir = std::getenv("TMPDIR");
  const std::string saved_tmpdir = old_tmpdir ? old_tmpdir : "";
  const bool had_tmpdir = old_tmpdir != nullptr;
  const std::string too_long(1100u, 'x');
  ASSERT_EQ(setenv("TMPDIR", too_long.c_str(), 1), 0);

  Runtime runtime;
  const std::string body =
      std::string("port fixture_request rpc capture ") +
      CAPTURE_PLUGIN_PATH + " config\n";
  const std::string path = writePortsFile(body);
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, path.c_str(), &kPort, 1u),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("temporary routing manifest"),
            std::string::npos);
  std::remove(path.c_str());

  if (had_tmpdir) {
    ASSERT_EQ(setenv("TMPDIR", saved_tmpdir.c_str(), 1), 0);
  } else {
    ASSERT_EQ(unsetenv("TMPDIR"), 0);
  }
}
#endif

///< REQ_PCL_486: the runtime executes and unwinds the normal component lifecycle. PCL.079.
TEST(PclProcessRuntime, RunsComponentUntilShutdownRequest) {
  Runtime runtime;
  pcl_callbacks_t callbacks = {};
  callbacks.on_tick = stopOnTick;
  pcl_container_t* component =
      pcl_container_create("runtime_test", &callbacks, runtime.value);
  ASSERT_NE(component, nullptr);

  EXPECT_EQ(pcl_process_runtime_run(runtime.value, component), PCL_OK)
      << pcl_process_runtime_error(runtime.value);
  EXPECT_EQ(pcl_container_state(component), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(component);
}

///< REQ_PCL_487, REQ_PCL_228: SIGTERM requests graceful shutdown and restores handlers. PCL.079.
TEST(PclProcessRuntime, SignalRequestsGracefulShutdown) {
  Runtime runtime;
  pcl_callbacks_t callbacks = {};
  pcl_container_t* component =
      pcl_container_create("runtime_signal_test", &callbacks, nullptr);
  ASSERT_NE(component, nullptr);

  std::thread signal_thread([] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::raise(SIGTERM);
  });
  EXPECT_EQ(pcl_process_runtime_run(runtime.value, component), PCL_OK)
      << pcl_process_runtime_error(runtime.value);
  signal_thread.join();
  EXPECT_EQ(pcl_container_state(component), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(component);
}

///< REQ_PCL_488: lifecycle failures are reported and reachable cleanup runs. PCL.079.
TEST(PclProcessRuntime, ReportsLifecycleFailures) {
  struct Case {
    const char* name;
    pcl_callbacks_t callbacks;
  };
  std::vector<Case> cases;
  {
    pcl_callbacks_t callbacks = {};
    callbacks.on_configure = failConfigure;
    cases.push_back({"configure_failure", callbacks});
  }
  {
    pcl_callbacks_t callbacks = {};
    callbacks.on_activate = failActivate;
    cases.push_back({"activate_failure", callbacks});
  }
  {
    pcl_callbacks_t callbacks = {};
    callbacks.on_tick = stopOnTick;
    callbacks.on_deactivate = failDeactivate;
    cases.push_back({"deactivate_failure", callbacks});
  }
  {
    pcl_callbacks_t callbacks = {};
    callbacks.on_tick = stopOnTick;
    callbacks.on_cleanup = failCleanup;
    cases.push_back({"cleanup_failure", callbacks});
  }

  for (auto& test_case : cases) {
    Runtime runtime;
    pcl_container_t* component =
        pcl_container_create(test_case.name, &test_case.callbacks,
                             runtime.value);
    ASSERT_NE(component, nullptr);
    EXPECT_EQ(pcl_process_runtime_run(runtime.value, component), PCL_ERR_STATE)
        << test_case.name;
    EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value)), "")
        << test_case.name;
    pcl_container_destroy(component);
  }
}

///< REQ_PCL_489: executor-add failure unwinds activation and configuration. PCL.079.
TEST(PclProcessRuntime, ReportsExecutorAddFailure) {
  Runtime runtime;
  std::vector<pcl_container_t*> fillers;
  fillers.reserve(64u);
  pcl_callbacks_t callbacks = {};
  for (size_t index = 0u; index < 64u; ++index) {
    pcl_container_t* filler =
        pcl_container_create(
            ("runtime_filler_" + std::to_string(index)).c_str(),
            &callbacks, nullptr);
    ASSERT_NE(filler, nullptr);
    ASSERT_EQ(pcl_executor_add(
                  pcl_process_runtime_executor(runtime.value), filler),
              PCL_OK);
    fillers.push_back(filler);
  }

  pcl_container_t* component =
      pcl_container_create("full_executor_component", &callbacks, nullptr);
  ASSERT_NE(component, nullptr);

  EXPECT_EQ(pcl_process_runtime_run(runtime.value, component), PCL_ERR_NOMEM);
  EXPECT_EQ(pcl_container_state(component), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(component);
  for (pcl_container_t* filler : fillers) {
    EXPECT_EQ(pcl_executor_remove(
                  pcl_process_runtime_executor(runtime.value), filler),
              PCL_OK);
    pcl_container_destroy(filler);
  }
}

///< REQ_PCL_490: a missing deployment file fails closed with a diagnostic. PCL.079.
TEST(PclProcessRuntime, RejectsMissingPortsFile) {
  Runtime runtime;
  EXPECT_EQ(
      pcl_process_runtime_load_ports_file(
          runtime.value, "/path/that/does/not/exist.ports", &kPort, 1u),
      PCL_ERR_NOT_FOUND);
  EXPECT_NE(
      std::string(pcl_process_runtime_error(runtime.value)).find("cannot open"),
      std::string::npos);
}

///< REQ_PCL_491: a non-zero duration stops the run and performs normal cleanup. PCL.079.
TEST(PclProcessRuntime, DurationLimitStopsRun) {
  Runtime runtime(1u);
  pcl_callbacks_t callbacks = {};
  pcl_container_t* component =
      pcl_container_create("duration_limited_component", &callbacks, nullptr);
  ASSERT_NE(component, nullptr);

  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(pcl_process_runtime_run(runtime.value, component), PCL_OK);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_GE(elapsed, std::chrono::milliseconds(900));
  EXPECT_LT(elapsed, std::chrono::seconds(3));
  EXPECT_EQ(pcl_container_state(component), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(component);
}

///< REQ_PCL_492: a runtime retains at most 32 codec plugin handles. PCL.079.
TEST(PclProcessRuntime, RejectsMoreThanMaximumCodecs) {
  Runtime runtime;
  for (size_t index = 0u; index < 32u; ++index) {
    ASSERT_EQ(pcl_process_runtime_load_codec(
                  runtime.value, SEQUENCE_CODEC_PLUGIN_PATH),
              PCL_OK)
        << index << ": " << pcl_process_runtime_error(runtime.value);
  }
  EXPECT_EQ(pcl_process_runtime_load_codec(
                runtime.value, SEQUENCE_CODEC_PLUGIN_PATH),
            PCL_ERR_NOMEM);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("too many codec plugins"),
            std::string::npos);
}

#ifndef _WIN32
///< REQ_PCL_493: an opened but unreadable deployment source fails closed. PCL.079.
TEST(PclProcessRuntime, ReportsPortsFileReadFailure) {
  Runtime runtime;
  EXPECT_EQ(pcl_process_runtime_load_ports_file(
                runtime.value, "/proc/self/mem", &kPort, 1u),
            PCL_ERR_INVALID);
  EXPECT_NE(std::string(pcl_process_runtime_error(runtime.value))
                .find("failed reading"),
            std::string::npos);
}
#endif
