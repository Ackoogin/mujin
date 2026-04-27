/// \file pcl_transport_conformance.hpp
/// \brief Reusable conformance suite for any PCL transport.
///
/// Drop-in test cases that any transport (template, socket, UDP, shared
/// memory, or a fresh engineer-supplied adapter) can plug into.  The
/// caller assembles a connected sender/receiver \ref TransportPair,
/// hands it to one of the conformance helpers below, and gets the same
/// behavioural assertions every other transport already passes.
///
/// To cover a new transport, write one `.cpp` file that:
///
///   1. Constructs a TransportPair (typically inside a helper so the
///      same setup can run several test cases — see how
///      test_pcl_template_transport.cpp wires loopback hooks).
///   2. Calls the helpers below from inside `TEST(...)` blocks.
///
/// The helpers do not own the executors, transports, or peer ids;
/// they only borrow the pointers in the pair.  Tear-down is the
/// caller's responsibility.
#pragma once

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
}

namespace pcl_conformance {

/// \brief Connected sender/receiver pair under test.
///
/// Both sides are full PCL executors with a transport adapter wired in.
/// `peer_id_*` is the logical identifier each end uses to address the
/// other (matching `pcl_executor_register_transport` / `set_peer_id`).
///
/// Lifetime: the caller creates the pair before invoking a conformance
/// helper and tears it down afterwards.  Helpers are pure observers.
struct TransportPair {
  pcl_executor_t*        sender_exec      = nullptr;
  pcl_executor_t*        receiver_exec    = nullptr;
  const pcl_transport_t* sender_vtable    = nullptr;
  const pcl_transport_t* receiver_vtable  = nullptr;
  std::string            sender_peer_id;    /* "what the receiver sees as the sender" */
  std::string            receiver_peer_id;  /* "what the sender sees as the receiver" */

  /// Spin both executors a few times — useful when one helper needs to
  /// drain an asynchronous response into the executor thread.
  void pumpBoth(std::chrono::milliseconds slice = std::chrono::milliseconds(5)) const {
    pcl_executor_spin_once(sender_exec,   0);
    pcl_executor_spin_once(receiver_exec, 0);
    std::this_thread::sleep_for(slice);
  }
};

/// \brief Round-trip a single PUBLISH from sender → receiver.
///
/// Adds a temporary subscriber container on the receiver, publishes
/// once via the sender's vtable, and asserts the subscriber callback
/// observes the same payload within a generous deadline.  The
/// subscriber is removed and destroyed before returning so the helper
/// composes with other test cases.
///
/// Best-effort transports (UDP) may pass `retries > 1` to retransmit
/// in case of loss; reliable transports leave it at 1.
inline void expectPublishRoundTrip(const TransportPair& pair,
                                   const std::string&   topic,
                                   const std::string&   type_name,
                                   const std::string&   payload,
                                   int                  retries = 1,
                                   std::chrono::milliseconds deadline =
                                       std::chrono::milliseconds(2000)) {
  struct SubState {
    std::atomic<bool> received{false};
    std::string       payload;
  } state;

  // Pack pair + topic + sub_state through a layered context so the
  // configure callback can see all three without globals.
  struct OuterCtx {
    const TransportPair* pair;
    std::string          topic;
    SubState*            state;
  } outer{ &pair, topic, &state };

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* o = static_cast<OuterCtx*>(ud);
    const char* peers[] = { o->pair->sender_peer_id.c_str() };
    pcl_port_t* port = pcl_container_add_subscriber(
        c, o->topic.c_str(), "MsgType",
        [](pcl_container_t*, const pcl_msg_t* msg, void* sub_ud) {
          auto* s = static_cast<SubState*>(sub_ud);
          s->payload.assign(static_cast<const char*>(msg->data), msg->size);
          s->received = true;
        }, o->state);
    pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("conf_sub", &cbs, &outer);
  ASSERT_NE(sub_c, nullptr);
  ASSERT_EQ(pcl_container_configure(sub_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(sub_c),  PCL_OK);
  ASSERT_EQ(pcl_executor_add(pair.receiver_exec, sub_c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data      = payload.data();
  msg.size      = static_cast<uint32_t>(payload.size());
  msg.type_name = type_name.c_str();

  for (int attempt = 0; attempt < retries && !state.received; ++attempt) {
    pair.sender_vtable->publish(pair.sender_vtable->adapter_ctx,
                                topic.c_str(), &msg);
    const auto attempt_start = std::chrono::steady_clock::now();
    while (!state.received &&
           std::chrono::steady_clock::now() - attempt_start < deadline / retries) {
      pair.pumpBoth();
    }
  }

  EXPECT_TRUE(state.received) << "subscriber did not observe the publish";
  EXPECT_EQ(state.payload, payload);

  pcl_executor_remove(pair.receiver_exec, sub_c);
  pcl_container_destroy(sub_c);
}

/// \brief Round-trip a single async service call (sender → receiver → sender).
///
/// Spawns a service-server container on the receiver, invokes
/// `service_name` from the sender via the transport's `invoke_async`
/// vtable hook, and asserts:
///
///   - the server handler runs on the receiver executor's thread,
///   - the response callback fires on the sender executor's thread,
///   - the response payload matches what the handler produced.
///
/// Skips itself with `GTEST_SKIP()` if the transport's vtable does not
/// expose `invoke_async` (e.g. UDP).
inline void expectServiceRoundTrip(const TransportPair& pair,
                                   const std::string&   service_name,
                                   const std::string&   request_payload,
                                   const std::string&   response_payload,
                                   std::chrono::milliseconds deadline =
                                       std::chrono::milliseconds(2000)) {
  if (!pair.sender_vtable->invoke_async) {
    GTEST_SKIP() << "transport does not implement invoke_async";
  }

  struct SrvState {
    std::string         observed_request;
    std::string         response;
    std::atomic<bool>   handled{false};
  } srv{ "", response_payload, false };

  pcl_callbacks_t cbs = {};
  struct OuterCtx {
    SrvState*   srv;
    std::string service_name;
  } outer{ &srv, service_name };
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* o = static_cast<OuterCtx*>(ud);
    pcl_port_t* port = pcl_container_add_service(
        c, o->service_name.c_str(), "SvcType",
        [](pcl_container_t*, const pcl_msg_t* req, pcl_msg_t* resp,
           pcl_svc_context_t*, void* svc_ud) -> pcl_status_t {
          auto* s = static_cast<SrvState*>(svc_ud);
          s->observed_request.assign(static_cast<const char*>(req->data),
                                     req->size);
          resp->data      = s->response.data();
          resp->size      = static_cast<uint32_t>(s->response.size());
          resp->type_name = "SvcType";
          s->handled = true;
          return PCL_OK;
        }, o->srv);
    pcl_port_set_route(port, PCL_ROUTE_REMOTE | PCL_ROUTE_LOCAL, nullptr, 0);
    return PCL_OK;
  };

  auto* svc_c = pcl_container_create("conf_svc", &cbs, &outer);
  ASSERT_NE(svc_c, nullptr);
  ASSERT_EQ(pcl_container_configure(svc_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc_c),  PCL_OK);
  ASSERT_EQ(pcl_executor_add(pair.receiver_exec, svc_c), PCL_OK);

  struct ClientState {
    std::atomic<bool> got_response{false};
    std::string       payload;
  } client;

  pcl_msg_t req = {};
  req.data      = request_payload.data();
  req.size      = static_cast<uint32_t>(request_payload.size());
  req.type_name = "SvcType";

  pcl_status_t rc = pair.sender_vtable->invoke_async(
      pair.sender_vtable->adapter_ctx,
      service_name.c_str(),
      &req,
      [](const pcl_msg_t* resp, void* ud) {
        auto* c = static_cast<ClientState*>(ud);
        if (resp && resp->data && resp->size) {
          c->payload.assign(static_cast<const char*>(resp->data), resp->size);
        }
        c->got_response = true;
      },
      &client);
  ASSERT_EQ(rc, PCL_OK);

  const auto start = std::chrono::steady_clock::now();
  while (!client.got_response &&
         std::chrono::steady_clock::now() - start < deadline) {
    pair.pumpBoth();
  }

  EXPECT_TRUE(srv.handled)            << "service handler never ran";
  EXPECT_EQ(srv.observed_request, request_payload);
  EXPECT_TRUE(client.got_response)    << "client never observed the response";
  EXPECT_EQ(client.payload, response_payload);

  pcl_executor_remove(pair.receiver_exec, svc_c);
  pcl_container_destroy(svc_c);
}

/// \brief Verify the basic vtable shape — non-null hooks, non-null
///        adapter_ctx, NULL-safe getters.
///
/// Quick sanity check that any transport adapter should pass before
/// the heavier round-trip helpers above.
inline void expectVtableShape(const pcl_transport_t* vt,
                              bool require_invoke_async = false) {
  ASSERT_NE(vt, nullptr);
  EXPECT_NE(vt->publish,     nullptr);
  EXPECT_NE(vt->subscribe,   nullptr);
  EXPECT_NE(vt->shutdown,    nullptr);
  EXPECT_NE(vt->adapter_ctx, nullptr);
  if (require_invoke_async) {
    EXPECT_NE(vt->invoke_async, nullptr);
  }
}

}  // namespace pcl_conformance
