/// \file test_pcl_streaming.cpp
/// \brief Tests for PCL streaming services.
///
/// Covers REQ_PCL_167–REQ_PCL_172 (tracing to HLR PCL.011c).
#include <gtest/gtest.h>

#include <vector>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
}

// ═══════════════════════════════════════════════════════════════════════
// Streaming service tests
// ═══════════════════════════════════════════════════════════════════════

///< REQ_PCL_167: Streaming service send and end. PCL.011c.
TEST(PclStreaming, BasicStreamingSendEnd) {
  // Test basic streaming: handler sends 3 messages then ends
  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
    bool handler_called = false;
    std::vector<int> received_values;
    bool stream_ended = false;
    pcl_status_t final_status = PCL_OK;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "test.stream", "StreamType",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        auto* ctx = static_cast<Ctx*>(ud);
        ctx->handler_called = true;
        ctx->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("stream_server", &cbs, &ctx);
  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";

  auto stream_cb = [](const pcl_msg_t* msg, bool end, pcl_status_t status, void* ud) {
    auto* ctx = static_cast<Ctx*>(ud);
    if (!end && msg->data && msg->size == sizeof(int)) {
      ctx->received_values.push_back(*static_cast<const int*>(msg->data));
    }
    if (end) {
      ctx->stream_ended = true;
      ctx->final_status = status;
    }
  };

  pcl_stream_context_t* client_ctx = nullptr;
  EXPECT_EQ(pcl_executor_invoke_stream(e, "test.stream", &req, stream_cb, &ctx, &client_ctx),
            PCL_OK);
  EXPECT_TRUE(ctx.handler_called);
  EXPECT_NE(ctx.stream_ctx, nullptr);

  // Server sends 3 messages
  for (int i = 1; i <= 3; ++i) {
    pcl_msg_t msg = {};
    msg.data = &i;
    msg.size = sizeof(i);
    msg.type_name = "Data";
    EXPECT_EQ(pcl_stream_send(ctx.stream_ctx, &msg), PCL_OK);
  }

  EXPECT_EQ(ctx.received_values.size(), 3u);
  EXPECT_EQ(ctx.received_values[0], 1);
  EXPECT_EQ(ctx.received_values[1], 2);
  EXPECT_EQ(ctx.received_values[2], 3);
  EXPECT_FALSE(ctx.stream_ended);

  // Server ends stream
  EXPECT_EQ(pcl_stream_end(ctx.stream_ctx), PCL_OK);
  EXPECT_TRUE(ctx.stream_ended);
  EXPECT_EQ(ctx.final_status, PCL_OK);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_168: Client stream cancellation. PCL.011c.
TEST(PclStreaming, ClientCancellation) {
  // Test client cancellation mid-stream
  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "cancel.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        static_cast<Ctx*>(ud)->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("cancel_server", &cbs, &ctx);
  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  pcl_stream_context_t* client_ctx = nullptr;
  EXPECT_EQ(pcl_executor_invoke_stream(e, "cancel.stream", &req, cb, nullptr, &client_ctx),
            PCL_OK);
  EXPECT_NE(ctx.stream_ctx, nullptr);

  // Initially not cancelled
  EXPECT_FALSE(pcl_stream_is_cancelled(ctx.stream_ctx));

  // Client cancels
  EXPECT_EQ(pcl_stream_cancel(client_ctx), PCL_OK);

  // Server sees cancellation
  EXPECT_TRUE(pcl_stream_is_cancelled(ctx.stream_ctx));

  // Sending after cancel returns CANCELLED
  pcl_msg_t msg = {};
  msg.type_name = "Data";
  EXPECT_EQ(pcl_stream_send(ctx.stream_ctx, &msg), PCL_ERR_CANCELLED);

  // Clean up - abort the stream
  EXPECT_EQ(pcl_stream_abort(ctx.stream_ctx, PCL_ERR_CANCELLED), PCL_OK);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_169: Server stream abort. PCL.011c.
TEST(PclStreaming, ServerAbort) {
  // Test server aborting stream with error
  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
    bool stream_ended = false;
    pcl_status_t final_status = PCL_OK;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "abort.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        static_cast<Ctx*>(ud)->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("abort_server", &cbs, &ctx);
  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool end, pcl_status_t status, void* ud) {
    auto* ctx = static_cast<Ctx*>(ud);
    if (end) {
      ctx->stream_ended = true;
      ctx->final_status = status;
    }
  };

  EXPECT_EQ(pcl_executor_invoke_stream(e, "abort.stream", &req, cb, &ctx, nullptr), PCL_OK);

  // Server aborts with error
  EXPECT_EQ(pcl_stream_abort(ctx.stream_ctx, PCL_ERR_CALLBACK), PCL_OK);
  EXPECT_TRUE(ctx.stream_ended);
  EXPECT_EQ(ctx.final_status, PCL_ERR_CALLBACK);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_170: Stream service not found. PCL.011c, PCL.045.
TEST(PclStreaming, StreamNotFound) {
  auto* e = pcl_executor_create();

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  EXPECT_EQ(pcl_executor_invoke_stream(e, "no.such.stream", &req, cb, nullptr, nullptr),
            PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

///< REQ_PCL_171: Stream API null safety. PCL.011c.
TEST(PclStreaming, StreamNullSafety) {
  pcl_msg_t msg = {};
  msg.type_name = "Msg";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  // Null executor
  EXPECT_EQ(pcl_executor_invoke_stream(nullptr, "svc", &msg, cb, nullptr, nullptr),
            PCL_ERR_INVALID);

  // Null service name
  auto* e = pcl_executor_create();
  EXPECT_EQ(pcl_executor_invoke_stream(e, nullptr, &msg, cb, nullptr, nullptr),
            PCL_ERR_INVALID);

  // Null request
  EXPECT_EQ(pcl_executor_invoke_stream(e, "svc", nullptr, cb, nullptr, nullptr),
            PCL_ERR_INVALID);

  // Null callback
  EXPECT_EQ(pcl_executor_invoke_stream(e, "svc", &msg, nullptr, nullptr, nullptr),
            PCL_ERR_INVALID);

  // Stream API null safety
  EXPECT_EQ(pcl_stream_send(nullptr, &msg), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_stream_end(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_stream_abort(nullptr, PCL_ERR_CALLBACK), PCL_ERR_INVALID);
  EXPECT_FALSE(pcl_stream_is_cancelled(nullptr));
  EXPECT_EQ(pcl_stream_cancel(nullptr), PCL_ERR_INVALID);

  pcl_executor_destroy(e);
}

///< REQ_PCL_172: Add stream service during configure. PCL.011c.
TEST(PclStreaming, AddStreamServiceDuringConfigure) {
  pcl_callbacks_t cbs = {};
  bool port_added = false;
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto handler = [](pcl_container_t*, const pcl_msg_t*,
                      pcl_stream_context_t*, void*) -> pcl_status_t {
      return PCL_STREAMING;
    };
    auto* port = pcl_container_add_stream_service(c, "my.stream", "Type", handler, nullptr);
    *static_cast<bool*>(ud) = (port != nullptr);
    return port ? PCL_OK : PCL_ERR_CALLBACK;
  };

  auto* c = pcl_container_create("stream_add", &cbs, &port_added);
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_TRUE(port_added);

  pcl_container_destroy(c);
}

// ═══════════════════════════════════════════════════════════════════════
// Transport streaming vtable tests
// ═══════════════════════════════════════════════════════════════════════

///< REQ_PCL_167: Streaming with transport vtable. PCL.011c.
TEST(PclStreaming, TransportStreamSendEnd) {
  struct TransportCtx {
    bool stream_send_called = false;
    bool stream_end_called = false;
    pcl_status_t end_status = PCL_OK;
  } tctx;

  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "transport.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        static_cast<Ctx*>(ud)->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("transport_stream", &cbs, &ctx);
  auto* e = pcl_executor_create();

  // Set up transport with streaming functions
  pcl_transport_t t = {};
  t.stream_send = [](void* adapter_ctx, void*, const pcl_msg_t*) -> pcl_status_t {
    static_cast<TransportCtx*>(adapter_ctx)->stream_send_called = true;
    return PCL_OK;
  };
  t.stream_end = [](void* adapter_ctx, void*, pcl_status_t status) -> pcl_status_t {
    auto* tctx = static_cast<TransportCtx*>(adapter_ctx);
    tctx->stream_end_called = true;
    tctx->end_status = status;
    return PCL_OK;
  };
  t.adapter_ctx = &tctx;
  pcl_executor_set_transport(e, &t);

  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  EXPECT_EQ(pcl_executor_invoke_stream(e, "transport.stream", &req, cb, nullptr, nullptr), PCL_OK);
  EXPECT_NE(ctx.stream_ctx, nullptr);

  // Send via transport
  pcl_msg_t msg = {};
  msg.type_name = "Data";
  EXPECT_EQ(pcl_stream_send(ctx.stream_ctx, &msg), PCL_OK);
  EXPECT_TRUE(tctx.stream_send_called);

  // End via transport
  EXPECT_EQ(pcl_stream_end(ctx.stream_ctx), PCL_OK);
  EXPECT_TRUE(tctx.stream_end_called);
  EXPECT_EQ(tctx.end_status, PCL_OK);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_169: Streaming abort with transport. PCL.011c.
TEST(PclStreaming, TransportStreamAbort) {
  struct TransportCtx {
    bool stream_end_called = false;
    pcl_status_t end_status = PCL_OK;
  } tctx;

  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "abort.transport.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        static_cast<Ctx*>(ud)->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("abort_transport", &cbs, &ctx);
  auto* e = pcl_executor_create();

  pcl_transport_t t = {};
  t.stream_end = [](void* adapter_ctx, void*, pcl_status_t status) -> pcl_status_t {
    auto* tctx = static_cast<TransportCtx*>(adapter_ctx);
    tctx->stream_end_called = true;
    tctx->end_status = status;
    return PCL_OK;
  };
  t.adapter_ctx = &tctx;
  pcl_executor_set_transport(e, &t);

  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  EXPECT_EQ(pcl_executor_invoke_stream(e, "abort.transport.stream", &req, cb, nullptr, nullptr), PCL_OK);

  // Abort via transport
  EXPECT_EQ(pcl_stream_abort(ctx.stream_ctx, PCL_ERR_CALLBACK), PCL_OK);
  EXPECT_TRUE(tctx.stream_end_called);
  EXPECT_EQ(tctx.end_status, PCL_ERR_CALLBACK);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_168: Stream cancel with transport. PCL.011c.
TEST(PclStreaming, TransportStreamCancel) {
  struct TransportCtx {
    bool stream_cancel_called = false;
  } tctx;

  struct Ctx {
    pcl_stream_context_t* stream_ctx = nullptr;
  } ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_stream_service(c, "cancel.transport.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t* stream_ctx, void* ud) -> pcl_status_t {
        static_cast<Ctx*>(ud)->stream_ctx = stream_ctx;
        return PCL_STREAMING;
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("cancel_transport", &cbs, &ctx);
  auto* e = pcl_executor_create();

  pcl_transport_t t = {};
  t.stream_cancel = [](void* adapter_ctx, void*) -> pcl_status_t {
    static_cast<TransportCtx*>(adapter_ctx)->stream_cancel_called = true;
    return PCL_OK;
  };
  t.adapter_ctx = &tctx;
  pcl_executor_set_transport(e, &t);

  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  pcl_stream_context_t* client_ctx = nullptr;
  EXPECT_EQ(pcl_executor_invoke_stream(e, "cancel.transport.stream", &req, cb, nullptr, &client_ctx), PCL_OK);

  // Cancel via transport
  EXPECT_EQ(pcl_stream_cancel(client_ctx), PCL_OK);
  EXPECT_TRUE(tctx.stream_cancel_called);

  // Clean up
  EXPECT_EQ(pcl_stream_abort(ctx.stream_ctx, PCL_ERR_CANCELLED), PCL_OK);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

///< REQ_PCL_167: Transport invoke_stream path. PCL.011c.
TEST(PclStreaming, TransportInvokeStream) {
  struct TransportCtx {
    bool invoke_stream_called = false;
    pcl_stream_msg_fn_t saved_cb = nullptr;
    void* saved_ud = nullptr;
  } tctx;

  auto* e = pcl_executor_create();

  pcl_transport_t t = {};
  t.invoke_stream = [](void* adapter_ctx, const char*, const pcl_msg_t*,
                       pcl_stream_msg_fn_t cb, void* ud, void** handle) -> pcl_status_t {
    auto* tctx = static_cast<TransportCtx*>(adapter_ctx);
    tctx->invoke_stream_called = true;
    tctx->saved_cb = cb;
    tctx->saved_ud = ud;
    *handle = reinterpret_cast<void*>(0x1234);  // fake handle
    return PCL_STREAMING;
  };
  t.adapter_ctx = &tctx;
  pcl_executor_set_transport(e, &t);

  pcl_msg_t req = {};
  req.type_name = "Req";

  struct ClientCtx { bool got_msg = false; } cctx;
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void* ud) {
    static_cast<ClientCtx*>(ud)->got_msg = true;
  };

  pcl_stream_context_t* stream_ctx = nullptr;
  // Transport returns PCL_STREAMING which is passed through
  EXPECT_EQ(pcl_executor_invoke_stream(e, "remote.stream", &req, cb, &cctx, &stream_ctx), PCL_STREAMING);
  EXPECT_TRUE(tctx.invoke_stream_called);
  EXPECT_NE(stream_ctx, nullptr);

  // Simulate transport delivering a message
  pcl_msg_t msg = {};
  msg.type_name = "Data";
  tctx.saved_cb(&msg, false, PCL_OK, tctx.saved_ud);
  EXPECT_TRUE(cctx.got_msg);

  // Clean up - free context manually since it's transport-managed
  free(stream_ctx);

  pcl_executor_set_transport(e, nullptr);
  pcl_executor_destroy(e);
}

///< REQ_PCL_170: Handler returns error. PCL.011c.
TEST(PclStreaming, HandlerReturnsError) {
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void*) -> pcl_status_t {
    pcl_container_add_stream_service(c, "error.stream", "Type",
      [](pcl_container_t*, const pcl_msg_t*,
         pcl_stream_context_t*, void*) -> pcl_status_t {
        return PCL_ERR_CALLBACK;  // Return error instead of PCL_STREAMING
      }, nullptr);
    return PCL_OK;
  };

  auto* c = pcl_container_create("error_handler", &cbs, nullptr);
  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t req = {};
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};

  pcl_stream_context_t* stream_ctx = nullptr;
  EXPECT_EQ(pcl_executor_invoke_stream(e, "error.stream", &req, cb, nullptr, &stream_ctx),
            PCL_ERR_CALLBACK);
  EXPECT_EQ(stream_ctx, nullptr);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}
