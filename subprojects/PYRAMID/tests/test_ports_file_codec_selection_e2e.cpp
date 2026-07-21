/// \file test_ports_file_codec_selection_e2e.cpp
/// \brief End-to-end proof that the codec a `.ports` file selects is the codec
///        the generated port facades actually put on the wire.
///
/// The generated application scaffold does three things in sequence: it builds
/// a pcl::ProcessRuntime from a `.ports` file, asks that runtime for each
/// port's content type, and hands the answer to the generated facade that owns
/// the port. Bringing a scaffold up proves only that the ports bound, which
/// looks the same whichever codec was chosen. These tests close that gap by
/// carrying the runtime-resolved content type all the way to a real socket
/// transport and then inspecting the bytes that arrived.
///
/// Both codecs are covered because the interesting case is the binary one:
/// JSON had been the only content type exercised through this path, and a
/// text-only path can hide assumptions that a FlatBuffers payload breaks.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_process_runtime.h>
#include <pcl/pcl_transport.h>
}

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#include "pyramid_codec_plugin_test_paths.hpp"
#include "pyramid_services_tactical_objects_consumed.hpp"

namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace types = pyramid::domain_model;

namespace {

using DestroyTransportFn = void (*)(const pcl_transport_t*);

/// Logical port name used in the `.ports` fixtures below. The name only has to
/// agree between the file and the deployment descriptor.
constexpr const char* kPortName = "object_evidence";

uint32_t processId() {
#if defined(_WIN32)
  return static_cast<uint32_t>(GetCurrentProcessId());
#else
  return static_cast<uint32_t>(getpid());
#endif
}

std::string uniqueTempPath() {
#if defined(_WIN32)
  char temp_dir[MAX_PATH];
  char file_name[MAX_PATH];
  GetTempPathA(MAX_PATH, temp_dir);
  GetTempFileNameA(temp_dir, "pcs", 0, file_name);
  return file_name;
#else
  char path[] = "/tmp/pyramid_codec_selection_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  return path;
#endif
}

/// \brief Write a `.ports` file selecting `content_type` from `codec_plugin`.
///
/// The port line names the shared-memory transport because this test only
/// needs the runtime to accept and record the codec selection; the message
/// exchange further down uses its own socket transport, exactly as a
/// deployment would.
std::string writePortsFile(const char* content_type,
                           const char* codec_plugin) {
  const std::string path = uniqueTempPath();
  std::ostringstream body;
  body << "codec " << content_type << " " << codec_plugin << "\n"
       << "port " << kPortName << " pubsub peer "
       << kPclSharedMemoryTransportPlugin
       << " {\"bus_name\":\"pyramid_codec_selection_" << processId()
       << "\",\"participant_id\":\"selection_test\"}\n";
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  EXPECT_TRUE(output.good());
  output << body.str();
  output.close();
  return path;
}

const pcl_process_endpoint_descriptor_t kPubsubEndpoints[] = {
    {"tactical_objects.object_evidence.publisher", PCL_ENDPOINT_PUBLISHER},
    {"tactical_objects.object_evidence.subscriber", PCL_ENDPOINT_SUBSCRIBER},
};

const pcl_process_endpoint_descriptor_t kRpcEndpoints[] = {
    {"tactical_objects.object_evidence.call", PCL_ENDPOINT_CONSUMED},
};

const pcl_process_port_descriptor_t kPort = {
    kPortName,
    kRpcEndpoints,
    sizeof(kRpcEndpoints) / sizeof(kRpcEndpoints[0]),
    kPubsubEndpoints,
    sizeof(kPubsubEndpoints) / sizeof(kPubsubEndpoints[0]),
};

/// \brief Resolve a port's content type the way the generated scaffold does.
///
/// Returns the empty string when the ports file was rejected, so a test can
/// tell a fail-closed rejection from a successful selection.
std::string resolvePortContentType(const char* content_type,
                                   const char* codec_plugin,
                                   std::string* out_error) {
  pcl_process_runtime_t* runtime = nullptr;
  if (pcl_process_runtime_create(0u, &runtime) != PCL_OK) return {};
  const std::string path = writePortsFile(content_type, codec_plugin);
  std::string resolved;
  const pcl_status_t status =
      pcl_process_runtime_load_ports_file(runtime, path.c_str(), &kPort, 1u);
  if (status == PCL_OK) {
    const char* value =
        pcl_process_runtime_port_content_type(runtime, kPortName);
    resolved = value ? value : "";
  } else if (out_error) {
    *out_error = pcl_process_runtime_error(runtime);
  }
  std::remove(path.c_str());
  pcl_process_runtime_destroy(runtime);
  return resolved;
}

types::ObjectDetail makeEvidence() {
  types::ObjectDetail ev;
  ev.id = "codec-selection-object";
  ev.identity = types::StandardIdentity::Hostile;
  ev.dimension = types::BattleDimension::SeaSurface;
  ev.position.latitude = 0.8901;
  ev.position.longitude = 0.0012;
  ev.creation_time = 42.0;
  ev.quality = 0.95;
  return ev;
}

uint16_t testPort() {
  return static_cast<uint16_t>(23000u + (processId() % 20000u));
}

std::string makeTransportConfig(const char* role,
                                uint16_t port,
                                pcl_executor_t* executor) {
  std::ostringstream out;
  out << "{\"role\":\"" << role
      << "\",\"host\":\"127.0.0.1\",\"port\":" << port
      << ",\"executor\":"
      << static_cast<unsigned long long>(
             reinterpret_cast<std::uintptr_t>(executor))
      << "}";
  return out.str();
}

/// Captures both the decoded value and the raw bytes, so a test can assert
/// what was actually transmitted rather than only that it round-tripped.
struct SubscriberState {
  pcl_executor_t* executor = nullptr;
  std::atomic<bool> started{false};
  std::atomic<bool> connected{false};
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
  std::atomic<int> received_count{0};
  std::mutex mutex;
  types::ObjectDetail decoded;
  std::string raw_payload;
  std::string raw_type_name;
  std::string content_type;
};

void evidenceCallback(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<SubscriberState*>(user_data);
  if (!msg) {
    state->failed.store(true);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->raw_payload.assign(static_cast<const char*>(msg->data), msg->size);
    state->raw_type_name = msg->type_name ? msg->type_name : "";
  }
  types::ObjectDetail decoded;
  if (!cons::decodeObjectEvidence(msg, &decoded)) {
    state->failed.store(true);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->decoded = decoded;
  }
  state->received_count.fetch_add(1);
}

pcl_status_t configureSubscriber(pcl_container_t* container, void* user_data) {
  auto* state = static_cast<SubscriberState*>(user_data);
  return cons::subscribeObjectEvidence(container, evidenceCallback, user_data,
                                       state->content_type.c_str())
             ? PCL_OK
             : PCL_ERR_NOMEM;
}

struct PublisherState {
  pcl_port_t* publisher = nullptr;
  std::string content_type;
};

pcl_status_t configurePublisher(pcl_container_t* container, void* user_data) {
  auto* state = static_cast<PublisherState*>(user_data);
  state->publisher = pcl_container_add_publisher(
      container, cons::kTopicObjectEvidence, state->content_type.c_str());
  return state->publisher ? PCL_OK : PCL_ERR_NOMEM;
}

void subscriberThread(SubscriberState* state, uint16_t port) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  const std::string config =
      makeTransportConfig("server", port, state->executor);

  state->started.store(true);
  if (pcl_plugin_load_transport(kPclSocketTransportPlugin, config.c_str(),
                                &handle, &transport) != PCL_OK) {
    state->failed.store(true);
    return;
  }
  auto destroy_transport = reinterpret_cast<DestroyTransportFn>(
      pcl_plugin_symbol(handle, "pcl_socket_transport_plugin_destroy"));
  if (!transport || !destroy_transport ||
      pcl_executor_set_transport(state->executor, transport) != PCL_OK) {
    state->failed.store(true);
  } else {
    state->connected.store(true);
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!state->done.load() && std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(state->executor, 0);
    if (state->received_count.load() > 0) {
      state->done.store(true);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  destroy_transport(transport);
  pcl_plugin_unload(handle);
}

void expectEvidenceEqual(const types::ObjectDetail& a,
                         const types::ObjectDetail& b) {
  EXPECT_EQ(a.id, b.id);
  EXPECT_EQ(a.identity, b.identity);
  EXPECT_EQ(a.dimension, b.dimension);
  EXPECT_DOUBLE_EQ(a.position.latitude, b.position.latitude);
  EXPECT_DOUBLE_EQ(a.position.longitude, b.position.longitude);
  EXPECT_DOUBLE_EQ(a.creation_time, b.creation_time);
  ASSERT_TRUE(a.quality.has_value());
  ASSERT_TRUE(b.quality.has_value());
  EXPECT_DOUBLE_EQ(a.quality.value(), b.quality.value());
}

/// \brief Publish one value through the generated facade using `content_type`
///        and return the bytes the subscriber received.
///
/// `content_type` is always the string the process runtime resolved from a
/// `.ports` file, never a literal, which is the property under test.
void exchangeThroughFacade(const std::string& content_type,
                           std::string* out_payload,
                           std::string* out_type_name) {
  const uint16_t port = testPort();
  SubscriberState subscriber_state;
  subscriber_state.content_type = content_type;
  subscriber_state.executor = pcl_executor_create();
  ASSERT_NE(subscriber_state.executor, nullptr);

  pcl_callbacks_t subscriber_callbacks = {};
  subscriber_callbacks.on_configure = configureSubscriber;
  pcl_container_t* subscriber_container = pcl_container_create(
      "codec_selection_subscriber", &subscriber_callbacks, &subscriber_state);
  ASSERT_NE(subscriber_container, nullptr);
  ASSERT_EQ(pcl_container_configure(subscriber_container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(subscriber_container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(subscriber_state.executor, subscriber_container),
            PCL_OK);

  std::thread subscriber(subscriberThread, &subscriber_state, port);
  while (!subscriber_state.started.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  pcl_executor_t* publisher_executor = pcl_executor_create();
  ASSERT_NE(publisher_executor, nullptr);

  pcl_plugin_handle_t* publisher_transport_handle = nullptr;
  const pcl_transport_t* publisher_transport = nullptr;
  const std::string publisher_config =
      makeTransportConfig("client", port, publisher_executor);
  ASSERT_EQ(pcl_plugin_load_transport(kPclSocketTransportPlugin,
                                      publisher_config.c_str(),
                                      &publisher_transport_handle,
                                      &publisher_transport),
            PCL_OK);
  auto destroy_publisher_transport = reinterpret_cast<DestroyTransportFn>(
      pcl_plugin_symbol(publisher_transport_handle,
                        "pcl_socket_transport_plugin_destroy"));
  ASSERT_NE(destroy_publisher_transport, nullptr);
  ASSERT_EQ(
      pcl_executor_set_transport(publisher_executor, publisher_transport),
      PCL_OK);

  const auto connected_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!subscriber_state.connected.load() &&
         std::chrono::steady_clock::now() < connected_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(subscriber_state.connected.load());

  PublisherState publisher_state;
  publisher_state.content_type = content_type;
  pcl_callbacks_t publisher_callbacks = {};
  publisher_callbacks.on_configure = configurePublisher;
  pcl_container_t* publisher_container = pcl_container_create(
      "codec_selection_publisher", &publisher_callbacks, &publisher_state);
  ASSERT_NE(publisher_container, nullptr);
  ASSERT_EQ(pcl_container_configure(publisher_container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(publisher_container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(publisher_executor, publisher_container), PCL_OK);
  ASSERT_NE(publisher_state.publisher, nullptr);

  const types::ObjectDetail expected = makeEvidence();
  ASSERT_EQ(cons::publishObjectEvidence(publisher_state.publisher, expected,
                                        content_type.c_str()),
            PCL_OK);

  const auto receive_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscriber_state.received_count.load() == 0 &&
         std::chrono::steady_clock::now() < receive_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_GE(subscriber_state.received_count.load(), 1);
  EXPECT_FALSE(subscriber_state.failed.load());
  {
    std::lock_guard<std::mutex> lock(subscriber_state.mutex);
    expectEvidenceEqual(subscriber_state.decoded, expected);
    if (out_payload) *out_payload = subscriber_state.raw_payload;
    if (out_type_name) *out_type_name = subscriber_state.raw_type_name;
  }

  subscriber_state.done.store(true);
  destroy_publisher_transport(publisher_transport);
  pcl_plugin_unload(publisher_transport_handle);
  pcl_executor_destroy(publisher_executor);
  pcl_container_destroy(publisher_container);

  subscriber.join();
  pcl_executor_destroy(subscriber_state.executor);
  pcl_container_destroy(subscriber_container);
}

/// \brief Decode `payload` as if it had arrived under `content_type`, and
///        report whether that reproduced `expected`.
///
/// Recovering the original value is the check that matters, and it is stricter
/// than asking whether the decode call succeeded. Two separate things make the
/// weaker checks unreliable here:
///
///  - "Does it look like JSON" is not decisive, because a FlatBuffers buffer
///    frequently starts with a byte that is also a valid JSON token, so a
///    permissive parser accepts the prefix.
///  - The generated JSON codec currently *returns success* when handed
///    FlatBuffers bytes, filling the target with default values rather than
///    rejecting the input. That leniency is a real fail-closed weakness, but
///    it is a defect in the JSON codec rather than in codec selection, so this
///    test is written not to depend on it either way.
bool decodesTo(const std::string& payload,
               const char* content_type,
               const types::ObjectDetail& expected) {
  pcl_msg_t msg{};
  msg.data = payload.data();
  msg.size = static_cast<uint32_t>(payload.size());
  msg.type_name = content_type;
  types::ObjectDetail decoded;
  if (!cons::decodeObjectEvidence(&msg, &decoded)) return false;
  return decoded.id == expected.id &&
         decoded.identity == expected.identity &&
         decoded.dimension == expected.dimension &&
         decoded.position.latitude == expected.position.latitude &&
         decoded.position.longitude == expected.position.longitude &&
         decoded.creation_time == expected.creation_time &&
         decoded.quality.has_value() == expected.quality.has_value();
}

/// \brief Printable prefix of a payload, for assertion messages.
std::string describePayload(const std::string& payload) {
  std::ostringstream out;
  out << payload.size() << " bytes, first 32 hex: ";
  for (std::size_t i = 0; i < payload.size() && i < 32u; ++i) {
    char buffer[4];
    std::snprintf(buffer, sizeof(buffer), "%02x",
                  static_cast<unsigned char>(payload[i]));
    out << buffer;
  }
  return out.str();
}

}  // namespace

///< REQ_PCL_477: a `.ports` file selecting FlatBuffers puts FlatBuffers on the wire. PCL.079.
TEST(PortsFileCodecSelection, FlatBuffersSelectionReachesTheGeneratedFacade) {
  std::string error;
  const std::string content_type = resolvePortContentType(
      "application/flatbuffers", kPyramidFlatbuffersCodecPluginTactical,
      &error);
  ASSERT_EQ(content_type, "application/flatbuffers") << error;

  std::string payload;
  std::string type_name;
  ASSERT_NO_FATAL_FAILURE(
      exchangeThroughFacade(content_type, &payload, &type_name));

  EXPECT_EQ(type_name, "application/flatbuffers");
  ASSERT_FALSE(payload.empty());
  // These two assertions together are the point of the test. A process that
  // quietly kept the JSON compatibility default would still have published,
  // been received, and round-tripped successfully above; only the wire format
  // would differ. So check the format itself.
  const types::ObjectDetail expected = makeEvidence();
  EXPECT_TRUE(decodesTo(payload, "application/flatbuffers", expected))
      << describePayload(payload);
  EXPECT_FALSE(decodesTo(payload, "application/json", expected))
      << "the JSON codec recovered the published value, so these are not "
         "FlatBuffers bytes and the selection did not take effect: "
      << describePayload(payload);
}

///< REQ_PCL_477: the same path still carries the JSON compatibility default. PCL.079.
TEST(PortsFileCodecSelection, JsonSelectionReachesTheGeneratedFacade) {
  std::string error;
  const std::string content_type = resolvePortContentType(
      "application/json", kPyramidJsonCodecPluginTactical, &error);
  ASSERT_EQ(content_type, "application/json") << error;

  std::string payload;
  std::string type_name;
  ASSERT_NO_FATAL_FAILURE(
      exchangeThroughFacade(content_type, &payload, &type_name));

  EXPECT_EQ(type_name, "application/json");
  ASSERT_FALSE(payload.empty());
  const types::ObjectDetail expected = makeEvidence();
  EXPECT_TRUE(decodesTo(payload, "application/json", expected))
      << describePayload(payload);
  EXPECT_EQ(payload.front(), '{')
      << "expected a JSON object on the wire: " << describePayload(payload);
}

// The codec/plugin mismatch case (a `codec` line asking for one content type
// while naming a plugin that provides another) is not tested here. Every test
// in this binary shares one process-wide codec registry, so by the time this
// case ran, an earlier test would already have registered the content type and
// the line would legitimately be accepted. It is covered deterministically in
// subprojects/PCL/tests/test_pcl_process_runtime.cpp, which gives that test its
// own codec plugin that nothing else loads.

///< REQ_PCL_477: a codec line naming a plugin that does not exist fails closed. PCL.079.
TEST(PortsFileCodecSelection, RejectsCodecLineWithMissingPlugin) {
  std::string error;
  const std::string content_type = resolvePortContentType(
      "application/x-pyramid-absent", "/no/such/codec-plugin.so", &error);
  EXPECT_TRUE(content_type.empty());
  EXPECT_NE(error.find("load codec plugin"), std::string::npos) << error;
}
