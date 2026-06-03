/// \file test_pcl_generated_component_stream_handle.cpp
/// \brief Tests generated C++ component facade stream-handle ownership.

#include <gtest/gtest.h>

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <chrono>
#include <future>
#include <string>

namespace prov = pyramid::components::tactical_objects::services::provided;
namespace types = pyramid::domain_model;

namespace {

struct ServerCtx {
  pcl_stream_context_t* stream_context = nullptr;
  int stream_count = 0;
};

class ServerComponent final : public pcl::Component {
public:
  explicit ServerComponent(ServerCtx& ctx)
      : pcl::Component("stream_handle_raii_server"),
        ctx_(&ctx) {}

protected:
  pcl_status_t on_configure() override {
    auto port = addStreamService(
        prov::kSvcObjectOfInterestReadRequirement,
        prov::kJsonContentType,
        [](pcl_container_t*,
           const pcl_msg_t*,
           pcl_stream_context_t* stream_context,
           void* user_data) -> pcl_status_t {
          auto* ctx = static_cast<ServerCtx*>(user_data);
          ctx->stream_context = stream_context;
          ++ctx->stream_count;
          return PCL_STREAMING;
        },
        ctx_);
    return port ? PCL_OK : PCL_ERR_NOMEM;
  }

private:
  ServerCtx* ctx_ = nullptr;
};

class RequirementHandler final : public prov::ProvidedHandler {
public:
  void onObjectOfInterestReadRequirement(
      const types::Query& query,
      prov::StreamWriter<types::ObjectInterestRequirement> writer) override {
    if (!query.id.empty()) {
      open_id = query.id.front();
    }
    stream = std::move(writer);
    ++stream_count;
  }

  types::Ack onObjectOfInterestDeleteRequirement(
      const types::Identifier& id) override {
    deleted_id = id;
    ++delete_count;
    if (drop_stream_on_delete) {
      stream = prov::StreamWriter<types::ObjectInterestRequirement>{};
    }
    return types::kAckOk;
  }

  pcl_status_t endOpen(pcl_status_t status) {
    return stream.end(status);
  }

  bool streamCancelled() const {
    return stream.cancelled();
  }

  std::string open_id;
  std::string deleted_id;
  int stream_count = 0;
  int delete_count = 0;
  bool drop_stream_on_delete = false;

private:
  prov::StreamWriter<types::ObjectInterestRequirement> stream;
};

class RequirementComponent final : public pcl::Component {
public:
  explicit RequirementComponent(pcl::Executor& executor)
      : pcl::Component("requirement_stream_delete_server"),
        provided_(*this, executor, handler_) {}

  RequirementHandler& handler() { return handler_; }

protected:
  pcl_status_t on_configure() override {
    return provided_.bind();
  }

private:
  RequirementHandler handler_;
  prov::ProvidedService provided_;
};

}  // namespace

TEST(PclGeneratedComponentStreamHandle,
     DestructorCancelsOpenStreamAndSuppressesCallbacks) {
  ServerCtx server_ctx;
  pcl::Executor executor;
  ServerComponent server{server_ctx};
  pcl::Component client{"stream_handle_raii_client"};
  prov::ConsumedService consumed{client, executor};

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(consumed.routeAllLocal(), PCL_OK);

  bool ended = false;
  pcl_status_t final_status = PCL_ERR_INVALID;

  {
    types::Query query;
    auto handle = consumed.objectOfInterestReadRequirementStreaming(
        query,
        [](const types::ObjectInterestRequirement&) {},
        [&](pcl_status_t status) {
          ended = true;
          final_status = status;
        });

    ASSERT_TRUE(handle);
    ASSERT_EQ(server_ctx.stream_count, 1);
    ASSERT_NE(server_ctx.stream_context, nullptr);
    EXPECT_FALSE(pcl_stream_is_cancelled(server_ctx.stream_context));
  }

  ASSERT_NE(server_ctx.stream_context, nullptr);
  EXPECT_TRUE(pcl_stream_is_cancelled(server_ctx.stream_context));
  EXPECT_FALSE(ended);

  EXPECT_EQ(pcl_stream_abort(server_ctx.stream_context, PCL_ERR_CANCELLED),
            PCL_OK);
  EXPECT_FALSE(ended);
  EXPECT_EQ(final_status, PCL_ERR_INVALID);
}

TEST(PclGeneratedComponentStreamHandle,
     ExplicitCancelKeepsEndCallback) {
  ServerCtx server_ctx;
  pcl::Executor executor;
  ServerComponent server{server_ctx};
  pcl::Component client{"stream_handle_explicit_cancel_client"};
  prov::ConsumedService consumed{client, executor};

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(consumed.routeAllLocal(), PCL_OK);

  bool ended = false;
  pcl_status_t final_status = PCL_ERR_INVALID;

  types::Query query;
  auto handle = consumed.objectOfInterestReadRequirementStreaming(
      query,
      [](const types::ObjectInterestRequirement&) {},
      [&](pcl_status_t status) {
        ended = true;
        final_status = status;
      });

  ASSERT_TRUE(handle);
  ASSERT_EQ(server_ctx.stream_count, 1);
  ASSERT_NE(server_ctx.stream_context, nullptr);

  handle.cancel();
  EXPECT_TRUE(pcl_stream_is_cancelled(server_ctx.stream_context));
  EXPECT_FALSE(ended);

  EXPECT_EQ(pcl_stream_abort(server_ctx.stream_context, PCL_ERR_CANCELLED),
            PCL_OK);
  EXPECT_TRUE(ended);
  EXPECT_EQ(final_status, PCL_ERR_CANCELLED);
}

TEST(PclGeneratedComponentStreamHandle,
     ServerEndBeforeHandleDropDisarmsDestructorCancel) {
  ServerCtx server_ctx;
  pcl::Executor executor;
  ServerComponent server{server_ctx};
  pcl::Component client{"stream_handle_server_end_client"};
  prov::ConsumedService consumed{client, executor};

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(consumed.routeAllLocal(), PCL_OK);

  bool ended = false;
  pcl_status_t final_status = PCL_ERR_INVALID;

  {
    types::Query query;
    auto handle = consumed.objectOfInterestReadRequirementStreaming(
        query,
        [](const types::ObjectInterestRequirement&) {},
        [&](pcl_status_t status) {
          ended = true;
          final_status = status;
        });

    ASSERT_TRUE(handle);
    ASSERT_NE(server_ctx.stream_context, nullptr);

    EXPECT_EQ(pcl_stream_abort(server_ctx.stream_context, PCL_ERR_CANCELLED),
              PCL_OK);
    EXPECT_TRUE(ended);
    EXPECT_EQ(final_status, PCL_ERR_CANCELLED);
  }
}

TEST(PclGeneratedComponentStreamHandle,
     DeleteForStreamedIdThenHandleDropCancelsOpenStream) {
  pcl::Executor executor;
  RequirementComponent server{executor};
  pcl::Component client{"stream_delete_client"};
  prov::ConsumedService consumed{client, executor};

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(consumed.routeAllLocal(), PCL_OK);

  bool ended = false;
  pcl_status_t final_status = PCL_ERR_INVALID;
  const std::string id = "interest-delete-case";

  {
    types::Query query;
    query.id.push_back(id);
    auto handle = consumed.objectOfInterestReadRequirementStreaming(
        query,
        [](const types::ObjectInterestRequirement&) {},
        [&](pcl_status_t status) {
          ended = true;
          final_status = status;
        });

    ASSERT_TRUE(handle);
    EXPECT_EQ(server.handler().stream_count, 1);
    EXPECT_EQ(server.handler().open_id, id);
    EXPECT_FALSE(server.handler().streamCancelled());

    auto delete_future = consumed.objectOfInterestDeleteRequirementAsync(id);
    ASSERT_EQ(delete_future.wait_for(std::chrono::milliseconds(0)),
              std::future_status::ready);
    const auto delete_result = delete_future.get();
    EXPECT_TRUE(delete_result.ok());
    EXPECT_TRUE(delete_result.value.success);
    EXPECT_EQ(server.handler().delete_count, 1);
    EXPECT_EQ(server.handler().deleted_id, id);

    EXPECT_FALSE(ended);
  }

  EXPECT_TRUE(server.handler().streamCancelled());
  EXPECT_FALSE(ended);

  EXPECT_EQ(server.handler().endOpen(PCL_ERR_CANCELLED), PCL_OK);
  EXPECT_FALSE(ended);
  EXPECT_EQ(final_status, PCL_ERR_INVALID);
}

TEST(PclGeneratedComponentStreamHandle,
     DeleteDroppingServerWriterWithoutEndReportsStateButDisarmsHandle) {
  pcl::Executor executor;
  RequirementComponent server{executor};
  pcl::Component client{"stream_delete_drop_writer_client"};
  prov::ConsumedService consumed{client, executor};
  server.handler().drop_stream_on_delete = true;

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(consumed.routeAllLocal(), PCL_OK);

  bool ended = false;
  pcl_status_t final_status = PCL_ERR_INVALID;
  const std::string id = "interest-drop-writer-on-delete";

  {
    types::Query query;
    query.id.push_back(id);
    auto handle = consumed.objectOfInterestReadRequirementStreaming(
        query,
        [](const types::ObjectInterestRequirement&) {},
        [&](pcl_status_t status) {
          ended = true;
          final_status = status;
        });

    ASSERT_TRUE(handle);
    EXPECT_EQ(server.handler().stream_count, 1);
    EXPECT_EQ(server.handler().open_id, id);

    auto delete_future = consumed.objectOfInterestDeleteRequirementAsync(id);
    ASSERT_EQ(delete_future.wait_for(std::chrono::milliseconds(0)),
              std::future_status::ready);
    const auto delete_result = delete_future.get();
    EXPECT_TRUE(delete_result.ok());
    EXPECT_TRUE(delete_result.value.success);
    EXPECT_EQ(server.handler().delete_count, 1);
    EXPECT_EQ(server.handler().deleted_id, id);
    EXPECT_TRUE(ended);
    EXPECT_EQ(final_status, PCL_ERR_STATE);
  }
}
