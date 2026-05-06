/// \file test_pcl_proto_bindings.cpp
/// \brief Tests for C++ PCL/proto service bindings (provided and consumed).
///
/// Validates the auto-generated C++ service bindings that mirror the Ada
/// generated files in subprojects/PYRAMID/bindings/ada/generated/:
///   - Wire-name constants match the proto service definitions
///   - Topic constants are correct
///   - msgToString utility works correctly
///   - JSON builder functions produce well-formed output
///   - ServiceHandler default stubs compile and return sensible values
///   - PCL subscribe wrappers register ports during on_configure
///   - Dispatch handles all ServiceChannel values without crashing

#include <gtest/gtest.h>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"
#include "pyramid_data_model_types.hpp"

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>
}

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdlib>
#include <string>
#include <vector>

namespace prov = pyramid::components::tactical_objects::services::provided;
namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
namespace protobuf_codec = pyramid::services::tactical_objects::protobuf_codec;
namespace types = pyramid::domain_model;
namespace tactical_codec = pyramid::domain_model::tactical;

namespace {

std::string makeProvidedFlatbuffersRequest(prov::ServiceChannel ch) {
    switch (ch) {
    case prov::ServiceChannel::MatchingObjectsReadMatch:
    case prov::ServiceChannel::ObjectOfInterestReadRequirement:
    case prov::ServiceChannel::SpecificObjectDetailReadDetail:
        return flatbuffers_codec::toBinary(types::Query{});
    case prov::ServiceChannel::ObjectOfInterestCreateRequirement:
    case prov::ServiceChannel::ObjectOfInterestUpdateRequirement:
        return flatbuffers_codec::toBinary(types::ObjectInterestRequirement{});
    case prov::ServiceChannel::ObjectOfInterestDeleteRequirement:
        return flatbuffers_codec::toBinary(types::Identifier{});
    }
    return {};
}

std::string makeConsumedFlatbuffersRequest(cons::ServiceChannel ch) {
    switch (ch) {
    case cons::ServiceChannel::ObjectEvidenceReadDetail:
    case cons::ServiceChannel::ObjectSolutionEvidenceReadRequirement:
    case cons::ServiceChannel::ObjectSourceCapabilityReadCapability:
        return flatbuffers_codec::toBinary(types::Query{});
    case cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement:
    case cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement:
        return flatbuffers_codec::toBinary(types::ObjectEvidenceRequirement{});
    case cons::ServiceChannel::ObjectSolutionEvidenceDeleteRequirement:
        return flatbuffers_codec::toBinary(types::Identifier{});
    }
    return {};
}

std::string makeProvidedProtobufRequest(prov::ServiceChannel ch) {
    switch (ch) {
    case prov::ServiceChannel::MatchingObjectsReadMatch:
    case prov::ServiceChannel::ObjectOfInterestReadRequirement:
    case prov::ServiceChannel::SpecificObjectDetailReadDetail:
        return protobuf_codec::toBinary(types::Query{});
    case prov::ServiceChannel::ObjectOfInterestCreateRequirement:
    case prov::ServiceChannel::ObjectOfInterestUpdateRequirement:
        return protobuf_codec::toBinary(types::ObjectInterestRequirement{});
    case prov::ServiceChannel::ObjectOfInterestDeleteRequirement:
        return protobuf_codec::toBinary(types::Identifier{});
    }
    return {};
}

std::string makeConsumedProtobufRequest(cons::ServiceChannel ch) {
    switch (ch) {
    case cons::ServiceChannel::ObjectEvidenceReadDetail:
    case cons::ServiceChannel::ObjectSolutionEvidenceReadRequirement:
    case cons::ServiceChannel::ObjectSourceCapabilityReadCapability:
        return protobuf_codec::toBinary(types::Query{});
    case cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement:
    case cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement:
        return protobuf_codec::toBinary(types::ObjectEvidenceRequirement{});
    case cons::ServiceChannel::ObjectSolutionEvidenceDeleteRequirement:
        return protobuf_codec::toBinary(types::Identifier{});
    }
    return {};
}

}  // namespace

// ===========================================================================
// Wire-name constant tests
// ===========================================================================

TEST(ProtoBindingsProvided, WireNames) {
    EXPECT_STREQ(prov::kSvcMatchingObjectsReadMatch, "matching_objects.read_match");
    EXPECT_STREQ(prov::kSvcObjectOfInterestCreateRequirement,
                 "object_of_interest.create_requirement");
    EXPECT_STREQ(prov::kSvcObjectOfInterestReadRequirement,
                 "object_of_interest.read_requirement");
    EXPECT_STREQ(prov::kSvcObjectOfInterestUpdateRequirement,
                 "object_of_interest.update_requirement");
    EXPECT_STREQ(prov::kSvcObjectOfInterestDeleteRequirement,
                 "object_of_interest.delete_requirement");
    EXPECT_STREQ(prov::kSvcSpecificObjectDetailReadDetail,
                 "specific_object_detail.read_detail");
}

TEST(ProtoBindingsProvided, TopicNames) {
    EXPECT_STREQ(prov::kTopicEntityMatches,        "standard.entity_matches");
    EXPECT_STREQ(prov::kTopicEvidenceRequirements, "standard.evidence_requirements");
}

TEST(ProtoBindingsConsumed, WireNames) {
    EXPECT_STREQ(cons::kSvcObjectEvidenceReadDetail, "object_evidence.read_detail");
    EXPECT_STREQ(cons::kSvcObjectSolutionEvidenceCreateRequirement,
                 "object_solution_evidence.create_requirement");
    EXPECT_STREQ(cons::kSvcObjectSolutionEvidenceReadRequirement,
                 "object_solution_evidence.read_requirement");
    EXPECT_STREQ(cons::kSvcObjectSolutionEvidenceUpdateRequirement,
                 "object_solution_evidence.update_requirement");
    EXPECT_STREQ(cons::kSvcObjectSolutionEvidenceDeleteRequirement,
                 "object_solution_evidence.delete_requirement");
    EXPECT_STREQ(cons::kSvcObjectSourceCapabilityReadCapability,
                 "object_source_capability.read_capability");
}

TEST(ProtoBindingsConsumed, TopicNames) {
    EXPECT_STREQ(cons::kTopicObjectEvidence, "standard.object_evidence");
}

TEST(ProtoBindingsProvided, ContentTypeMetadata) {
    EXPECT_TRUE(prov::supportsContentType(nullptr));
    EXPECT_TRUE(prov::supportsContentType(prov::kJsonContentType));
    EXPECT_TRUE(prov::supportsContentType(prov::kFlatBuffersContentType));
    EXPECT_TRUE(prov::supportsContentType(prov::kProtobufContentType));
    EXPECT_FALSE(prov::supportsContentType("text/plain"));
    EXPECT_GE(prov::supportedContentTypes().size(), 1u);
}

TEST(ProtoBindingsConsumed, ContentTypeMetadata) {
    EXPECT_TRUE(cons::supportsContentType(nullptr));
    EXPECT_TRUE(cons::supportsContentType(cons::kJsonContentType));
    EXPECT_TRUE(cons::supportsContentType(cons::kFlatBuffersContentType));
    EXPECT_TRUE(cons::supportsContentType(cons::kProtobufContentType));
    EXPECT_FALSE(cons::supportsContentType("text/plain"));
    EXPECT_GE(cons::supportedContentTypes().size(), 1u);
}

// ===========================================================================
// msgToString utility
// ===========================================================================

TEST(ProtoBindingsProvided, MsgToString) {
    const char payload[] = "hello world";
    std::string result = prov::msgToString(payload, static_cast<unsigned>(sizeof(payload) - 1));
    EXPECT_EQ(result, "hello world");
}

TEST(ProtoBindingsConsumed, MsgToString) {
    const char payload[] = R"({"key":"value"})";
    std::string result = cons::msgToString(payload, static_cast<unsigned>(sizeof(payload) - 1));
    EXPECT_EQ(result, R"({"key":"value"})");
}

TEST(ProtoBindingsProvided, MsgToStringEmpty) {
    std::string result = prov::msgToString(nullptr, 0);
    EXPECT_TRUE(result.empty());
}

TEST(ProtoBindingsProvided, TopicEncodeDecodeEntityMatches) {
    types::ObjectMatch match;
    match.id = "obj-1";
    match.matching_object_id = "match-1";
    match.source = "radar";
    match.update_time = 42.0;
    const std::vector<types::ObjectMatch> input{match};

    const char* content_types[] = {
        prov::kJsonContentType,
        prov::kFlatBuffersContentType,
        prov::kProtobufContentType,
    };
    for (const char* content_type : content_types) {
        std::string payload;
        ASSERT_TRUE(prov::encodeEntityMatches(input, content_type, &payload)) << content_type;

        pcl_msg_t msg{};
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = content_type;

        std::vector<types::ObjectMatch> output;
        ASSERT_TRUE(prov::decodeEntityMatches(&msg, &output)) << content_type;
        ASSERT_EQ(output.size(), 1u);
        EXPECT_EQ(output[0].id, "obj-1");
        EXPECT_EQ(output[0].matching_object_id, "match-1");
        EXPECT_EQ(output[0].source, "radar");
        ASSERT_TRUE(output[0].update_time.has_value());
        EXPECT_DOUBLE_EQ(*output[0].update_time, 42.0);
    }
}

TEST(ProtoBindingsConsumed, TopicEncodeDecodeObjectEvidence) {
    types::ObjectDetail input;
    input.id = "obj-7";
    input.identity = types::StandardIdentity::Friendly;
    input.dimension = types::BattleDimension::Air;
    input.position.latitude = 0.25;
    input.position.longitude = -0.75;
    input.entity_source = "sensor-a";

    const char* content_types[] = {
        cons::kJsonContentType,
        cons::kFlatBuffersContentType,
        cons::kProtobufContentType,
    };
    for (const char* content_type : content_types) {
        std::string payload;
        ASSERT_TRUE(cons::encodeObjectEvidence(input, content_type, &payload)) << content_type;

        pcl_msg_t msg{};
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = content_type;

        types::ObjectDetail output;
        ASSERT_TRUE(cons::decodeObjectEvidence(&msg, &output)) << content_type;
        EXPECT_EQ(output.id, "obj-7");
        EXPECT_EQ(output.identity, types::StandardIdentity::Friendly);
        EXPECT_EQ(output.dimension, types::BattleDimension::Air);
        EXPECT_DOUBLE_EQ(output.position.latitude, 0.25);
        EXPECT_DOUBLE_EQ(output.position.longitude, -0.75);
        EXPECT_EQ(output.entity_source, "sensor-a");
    }
}

TEST(ProtoBindingsProvided, TopicEncodeRejectsUnsupportedContentType) {
    std::vector<types::ObjectMatch> matches;
    std::string payload;
    EXPECT_FALSE(prov::encodeEntityMatches(matches, "text/plain", &payload));
}

// ===========================================================================
// Proto-native JSON codec -- ObjectInterestRequirement
// ===========================================================================

TEST(ProtoBindingsProvided, CreateRequirementJsonKeys) {
    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Query;
    req.source = types::ObjectSource::Local;
    req.dimension.push_back(types::BattleDimension::Air);

    std::string json_str = tactical_codec::toJson(req);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_TRUE(j.contains("policy"));
    EXPECT_TRUE(j.contains("source"));
    EXPECT_TRUE(j.contains("dimension"));
    ASSERT_TRUE(j["dimension"].is_array());
    ASSERT_EQ(j["dimension"].size(), 1u);
}

TEST(ProtoBindingsProvided, CreateRequirementJsonDefaults) {
    types::ObjectInterestRequirement req;
    std::string json_str = tactical_codec::toJson(req);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_FALSE(j.contains("source"));
    ASSERT_TRUE(j.contains("dimension"));
    EXPECT_TRUE(j["dimension"].is_array());
    EXPECT_TRUE(j["dimension"].empty());
}

// ===========================================================================
// Proto-native JSON codec -- ObjectDetail topic payload
// ===========================================================================

TEST(ProtoBindingsProvided, EvidenceJsonKeys) {
    types::ObjectDetail ev;
    ev.id            = "obj-1";
    ev.identity      = types::StandardIdentity::Friendly;
    ev.dimension     = types::BattleDimension::Ground;
    ev.position.latitude  = 0.785;
    ev.position.longitude = -1.571;
    ev.quality       = 0.9;
    ev.creation_time = 0.75;

    std::string json_str = tactical_codec::toJson(ev);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_TRUE(j.contains("id"));
    EXPECT_TRUE(j.contains("identity"));
    EXPECT_TRUE(j.contains("dimension"));
    ASSERT_TRUE(j.contains("position"));
    EXPECT_DOUBLE_EQ(j["position"]["latitude"].get<double>(),  0.785);
    EXPECT_DOUBLE_EQ(j["position"]["longitude"].get<double>(), -1.571);
    EXPECT_DOUBLE_EQ(j["quality"].get<double>(),    0.9);
    EXPECT_DOUBLE_EQ(j["creation_time"].get<double>(),   0.75);
}

TEST(ProtoBindingsProvided, EvidenceJsonDefaultObservedAt) {
    types::ObjectDetail ev;
    std::string json_str = tactical_codec::toJson(ev);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_FALSE(j.contains("quality"));
}

// ===========================================================================
// ServiceHandler -- default stub implementations
// ===========================================================================

// Concrete subclass to exercise the virtual-dispatch path.
class TestProvidedHandler : public prov::ServiceHandler {
public:
    bool read_match_called        = false;
    bool create_req_called        = false;
    bool read_req_called          = false;
    bool update_req_called        = false;
    bool delete_req_called        = false;
    bool read_detail_called       = false;

    std::vector<types::ObjectMatch>
    handleMatchingObjectsReadMatch(const types::Query& q) override {
        read_match_called = true;
        return prov::ServiceHandler::handleMatchingObjectsReadMatch(q);  // call stub
    }

    types::Identifier
    handleObjectOfInterestCreateRequirement(
        const types::ObjectInterestRequirement& r) override {
        create_req_called = true;
        return prov::ServiceHandler::handleObjectOfInterestCreateRequirement(r);
    }

    std::vector<types::ObjectInterestRequirement>
    handleObjectOfInterestReadRequirement(const types::Query& q) override {
        read_req_called = true;
        return prov::ServiceHandler::handleObjectOfInterestReadRequirement(q);
    }

    types::Ack
    handleObjectOfInterestUpdateRequirement(
        const types::ObjectInterestRequirement& r) override {
        update_req_called = true;
        return prov::ServiceHandler::handleObjectOfInterestUpdateRequirement(r);
    }

    types::Ack
    handleObjectOfInterestDeleteRequirement(const types::Identifier& id) override {
        delete_req_called = true;
        return prov::ServiceHandler::handleObjectOfInterestDeleteRequirement(id);
    }

    std::vector<types::ObjectDetail>
    handleSpecificObjectDetailReadDetail(const types::Query& q) override {
        read_detail_called = true;
        return prov::ServiceHandler::handleSpecificObjectDetailReadDetail(q);
    }
};

TEST(ProtoBindingsProvided, HandlerStubsReturnEmpty) {
    TestProvidedHandler handler;

    types::Query q;
    auto matches = handler.handleMatchingObjectsReadMatch(q);
    EXPECT_TRUE(matches.empty());
    EXPECT_TRUE(handler.read_match_called);

    types::ObjectInterestRequirement req;
    auto id = handler.handleObjectOfInterestCreateRequirement(req);
    EXPECT_TRUE(id.empty());
    EXPECT_TRUE(handler.create_req_called);

    auto reqs = handler.handleObjectOfInterestReadRequirement(q);
    EXPECT_TRUE(reqs.empty());
    EXPECT_TRUE(handler.read_req_called);

    auto ack_update = handler.handleObjectOfInterestUpdateRequirement(req);
    EXPECT_TRUE(ack_update.success);
    EXPECT_TRUE(handler.update_req_called);

    auto ack_delete = handler.handleObjectOfInterestDeleteRequirement("some-id");
    EXPECT_TRUE(ack_delete.success);
    EXPECT_TRUE(handler.delete_req_called);

    auto details = handler.handleSpecificObjectDetailReadDetail(q);
    EXPECT_TRUE(details.empty());
    EXPECT_TRUE(handler.read_detail_called);
}

class TestConsumedHandler : public cons::ServiceHandler {
public:
    bool read_detail_called    = false;
    bool create_req_called     = false;
    bool read_req_called       = false;
    bool update_req_called     = false;
    bool delete_req_called     = false;
    bool read_cap_called       = false;

    std::vector<types::ObjectDetail>
    handleObjectEvidenceReadDetail(const types::Query& q) override {
        read_detail_called = true;
        return cons::ServiceHandler::handleObjectEvidenceReadDetail(q);
    }

    types::Identifier
    handleObjectSolutionEvidenceCreateRequirement(
        const types::ObjectEvidenceRequirement& r) override {
        create_req_called = true;
        return cons::ServiceHandler::handleObjectSolutionEvidenceCreateRequirement(r);
    }

    std::vector<types::ObjectEvidenceRequirement>
    handleObjectSolutionEvidenceReadRequirement(const types::Query& q) override {
        read_req_called = true;
        return cons::ServiceHandler::handleObjectSolutionEvidenceReadRequirement(q);
    }

    types::Ack
    handleObjectSolutionEvidenceUpdateRequirement(
        const types::ObjectEvidenceRequirement& r) override {
        update_req_called = true;
        return cons::ServiceHandler::handleObjectSolutionEvidenceUpdateRequirement(r);
    }

    types::Ack
    handleObjectSolutionEvidenceDeleteRequirement(
        const types::Identifier& id) override {
        delete_req_called = true;
        return cons::ServiceHandler::handleObjectSolutionEvidenceDeleteRequirement(id);
    }

    std::vector<types::Capability>
    handleObjectSourceCapabilityReadCapability(const types::Query& q) override {
        read_cap_called = true;
        return cons::ServiceHandler::handleObjectSourceCapabilityReadCapability(q);
    }
};

TEST(ProtoBindingsConsumed, HandlerStubsReturnEmpty) {
    TestConsumedHandler handler;

    types::Query q;
    auto details = handler.handleObjectEvidenceReadDetail(q);
    EXPECT_TRUE(details.empty());
    EXPECT_TRUE(handler.read_detail_called);

    types::ObjectEvidenceRequirement req;
    auto id = handler.handleObjectSolutionEvidenceCreateRequirement(req);
    EXPECT_TRUE(id.empty());
    EXPECT_TRUE(handler.create_req_called);

    auto reqs = handler.handleObjectSolutionEvidenceReadRequirement(q);
    EXPECT_TRUE(reqs.empty());
    EXPECT_TRUE(handler.read_req_called);

    auto ack_update = handler.handleObjectSolutionEvidenceUpdateRequirement(req);
    EXPECT_TRUE(ack_update.success);
    EXPECT_TRUE(handler.update_req_called);

    auto ack_delete = handler.handleObjectSolutionEvidenceDeleteRequirement("ev-id");
    EXPECT_TRUE(ack_delete.success);
    EXPECT_TRUE(handler.delete_req_called);

    auto caps = handler.handleObjectSourceCapabilityReadCapability(q);
    EXPECT_TRUE(caps.empty());
    EXPECT_TRUE(handler.read_cap_called);
}

// ===========================================================================
// PCL subscribe registration -- verify ports are created during on_configure
// ===========================================================================

struct SubscribeCtx {
    std::atomic<int> call_count{0};
};

static void null_sub_cb(pcl_container_t*, const pcl_msg_t*, void* ud) {
    static_cast<SubscribeCtx*>(ud)->call_count++;
}

static pcl_status_t configure_provided_subs(pcl_container_t* c, void* ud) {
    prov::subscribeEntityMatches(c, null_sub_cb, ud);
    prov::subscribeEvidenceRequirements(c, null_sub_cb, ud);
    return PCL_OK;
}

TEST(ProtoBindingsProvided, SubscribeRegistrationSucceeds) {
    SubscribeCtx ctx;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_provided_subs;

    pcl_container_t* c = pcl_container_create("prov_sub_test", &cbs, &ctx);
    ASSERT_NE(c, nullptr);

    pcl_status_t rc = pcl_container_configure(c);
    EXPECT_EQ(rc, PCL_OK);
    EXPECT_EQ(pcl_container_state(c), PCL_STATE_CONFIGURED);

    pcl_container_destroy(c);
}

struct ConsumedSubCtx {
    std::atomic<int> call_count{0};
    pcl_port_t*      pub_port = nullptr;
};

static pcl_status_t configure_consumed_subs(pcl_container_t* c, void* ud) {
    auto* ctx = static_cast<ConsumedSubCtx*>(ud);
    cons::subscribeObjectEvidence(c, null_sub_cb, ud);
    ctx->pub_port = pcl_container_add_publisher(
        c, cons::kTopicObjectEvidence, "application/json");
    return PCL_OK;
}

struct ConsumedInvokeCtx {
    std::atomic<int> service_count{0};
    std::atomic<int> callback_count{0};
    bool decoded_response = false;
    types::ObjectEvidenceRequirement captured_req;
    types::Identifier response_id;
    std::string response_buffer;
};

static pcl_status_t handle_consumed_create_requirement(
    pcl_container_t*, const pcl_msg_t* request, pcl_msg_t* response,
    pcl_svc_context_t*, void* user_data) {
    auto* ctx = static_cast<ConsumedInvokeCtx*>(user_data);

    struct CapturingHandler : public cons::ServiceHandler {
        explicit CapturingHandler(ConsumedInvokeCtx& ctx) : ctx(ctx) {}

        types::Identifier
        handleObjectSolutionEvidenceCreateRequirement(
            const types::ObjectEvidenceRequirement& req) override {
            ctx.service_count++;
            ctx.captured_req = req;
            return "evidence-requirement-typed";
        }

        ConsumedInvokeCtx& ctx;
    };

    CapturingHandler handler(*ctx);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    cons::dispatch(handler,
                   cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement,
                   request ? request->data : nullptr,
                   request ? request->size : 0u,
                   request ? request->type_name : cons::kJsonContentType,
                   &resp_buf, &resp_size);

    ctx->response_buffer.clear();
    if (resp_buf && resp_size > 0) {
        ctx->response_buffer.assign(static_cast<const char*>(resp_buf), resp_size);
        std::free(resp_buf);
    }

    response->data = ctx->response_buffer.empty()
                         ? nullptr
                         : const_cast<char*>(ctx->response_buffer.data());
    response->size = static_cast<uint32_t>(ctx->response_buffer.size());
    response->type_name = request ? request->type_name : cons::kJsonContentType;
    return PCL_OK;
}

static pcl_status_t configure_consumed_create_requirement(
    pcl_container_t* c, void* ud) {
    auto* port = pcl_container_add_service(
        c, cons::kSvcObjectSolutionEvidenceCreateRequirement,
        cons::kJsonContentType,
        handle_consumed_create_requirement, ud);
    return port ? PCL_OK : PCL_ERR_NOMEM;
}

static void consumed_create_requirement_response(
    const pcl_msg_t* response, void* user_data) {
    auto* ctx = static_cast<ConsumedInvokeCtx*>(user_data);
    ctx->decoded_response =
        cons::decodeObjectSolutionEvidenceCreateRequirementResponse(
            response, &ctx->response_id);
    ctx->callback_count++;
}

TEST(ProtoBindingsConsumed, SubscribeRegistrationSucceeds) {
    ConsumedSubCtx ctx;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_consumed_subs;

    pcl_container_t* c = pcl_container_create("cons_sub_test", &cbs, &ctx);
    ASSERT_NE(c, nullptr);

    pcl_status_t rc = pcl_container_configure(c);
    EXPECT_EQ(rc, PCL_OK);
    EXPECT_NE(ctx.pub_port, nullptr);

    pcl_container_destroy(c);
}

// ===========================================================================
// Typed invoke facade -- component code should not construct raw PCL messages
// ===========================================================================

TEST(ProtoBindingsConsumed, InvokeCreateRequirementUsesTypedFacade) {
    ConsumedInvokeCtx ctx;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_consumed_create_requirement;

    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    pcl_container_t* c =
        pcl_container_create("cons_invoke_create_requirement", &cbs, &ctx);
    ASSERT_NE(c, nullptr);
    ASSERT_EQ(pcl_container_configure(c), PCL_OK);
    ASSERT_EQ(pcl_container_activate(c), PCL_OK);
    ASSERT_EQ(pcl_executor_add(exec, c), PCL_OK);

    types::ObjectEvidenceRequirement req;
    req.base.id = "typed-evidence-interest";
    req.policy = types::DataPolicy::Obtain;

    const pcl_status_t rc = cons::invokeObjectSolutionEvidenceCreateRequirement(
        exec, req, consumed_create_requirement_response, &ctx);

    EXPECT_EQ(rc, PCL_OK);
    EXPECT_EQ(ctx.service_count.load(), 1);
    EXPECT_EQ(ctx.callback_count.load(), 1);
    EXPECT_EQ(ctx.captured_req.base.id, "typed-evidence-interest");
    EXPECT_EQ(ctx.captured_req.policy, types::DataPolicy::Obtain);
    EXPECT_TRUE(ctx.decoded_response);
    EXPECT_EQ(ctx.response_id, "evidence-requirement-typed");

    const pcl_status_t fire_and_forget_rc =
        cons::invokeObjectSolutionEvidenceCreateRequirement(
            exec, req, cons::kJsonContentType);

    EXPECT_EQ(fire_and_forget_rc, PCL_OK);
    EXPECT_EQ(ctx.service_count.load(), 2);
    EXPECT_EQ(ctx.callback_count.load(), 1);

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

// ===========================================================================
// publishObjectEvidence -- verify publish returns an error when not active
// (port is closed when container is not ACTIVE)
// ===========================================================================

TEST(ProtoBindingsConsumed, PublishReturnsBadStatusWhenInactive) {
    ConsumedSubCtx ctx;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_consumed_subs;

    pcl_container_t* c = pcl_container_create("cons_pub_inactive", &cbs, &ctx);
    ASSERT_NE(c, nullptr);
    pcl_container_configure(c);
    ASSERT_NE(ctx.pub_port, nullptr);

    // Container is CONFIGURED but not ACTIVE -- port should be closed.
    pcl_status_t rc = cons::publishObjectEvidence(ctx.pub_port, R"({"test":1})");
    EXPECT_NE(rc, PCL_OK);  // expect PCL_ERR_PORT_CLOSED or similar

    pcl_container_destroy(c);
}

TEST(ProtoBindingsConsumed, PublishSucceedsWhenActive) {
    ConsumedSubCtx ctx;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_consumed_subs;

    pcl_container_t* c = pcl_container_create("cons_pub_active", &cbs, &ctx);
    ASSERT_NE(c, nullptr);
    pcl_container_configure(c);
    pcl_container_activate(c);
    ASSERT_NE(ctx.pub_port, nullptr);

    pcl_status_t rc = cons::publishObjectEvidence(ctx.pub_port, R"({"type":"evidence"})");
    EXPECT_EQ(rc, PCL_OK);

    pcl_container_destroy(c);
}

// ===========================================================================
// Dispatch -- verify all ServiceChannel values are handled without crash
// ===========================================================================

TEST(ProtoBindingsProvided, DispatchAllChannelsNoCrash) {
    prov::ServiceHandler handler;  // default stubs
    const prov::ServiceChannel channels[] = {
        prov::ServiceChannel::MatchingObjectsReadMatch,
        prov::ServiceChannel::ObjectOfInterestCreateRequirement,
        prov::ServiceChannel::ObjectOfInterestReadRequirement,
        prov::ServiceChannel::ObjectOfInterestUpdateRequirement,
        prov::ServiceChannel::ObjectOfInterestDeleteRequirement,
        prov::ServiceChannel::SpecificObjectDetailReadDetail,
    };

    // Empty JSON object as minimal valid request for all channels
    std::string empty_req = "{}";
    for (auto ch : channels) {
        void*  resp_buf  = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            prov::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           &resp_buf, &resp_size));
        // All stubs return something ([] for streams, "" for id, json for ack)
        if (resp_buf) std::free(resp_buf);
    }
}

TEST(ProtoBindingsConsumed, DispatchAllChannelsNoCrash) {
    cons::ServiceHandler handler;  // default stubs
    const cons::ServiceChannel channels[] = {
        cons::ServiceChannel::ObjectEvidenceReadDetail,
        cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceReadRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceDeleteRequirement,
        cons::ServiceChannel::ObjectSourceCapabilityReadCapability,
    };

    std::string empty_req = "{}";
    for (auto ch : channels) {
        void*  resp_buf  = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            cons::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           &resp_buf, &resp_size));
        if (resp_buf) std::free(resp_buf);
    }
}

TEST(ProtoBindingsProvided, DispatchAllChannelsFlatBuffersNoCrash) {
    prov::ServiceHandler handler;
    const prov::ServiceChannel channels[] = {
        prov::ServiceChannel::MatchingObjectsReadMatch,
        prov::ServiceChannel::ObjectOfInterestCreateRequirement,
        prov::ServiceChannel::ObjectOfInterestReadRequirement,
        prov::ServiceChannel::ObjectOfInterestUpdateRequirement,
        prov::ServiceChannel::ObjectOfInterestDeleteRequirement,
        prov::ServiceChannel::SpecificObjectDetailReadDetail,
    };

    for (auto ch : channels) {
        const std::string empty_req = makeProvidedFlatbuffersRequest(ch);
        void* resp_buf = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            prov::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           "application/flatbuffers", &resp_buf, &resp_size));
        if (resp_buf) std::free(resp_buf);
    }
}

TEST(ProtoBindingsConsumed, DispatchAllChannelsFlatBuffersNoCrash) {
    cons::ServiceHandler handler;
    const cons::ServiceChannel channels[] = {
        cons::ServiceChannel::ObjectEvidenceReadDetail,
        cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceReadRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceDeleteRequirement,
        cons::ServiceChannel::ObjectSourceCapabilityReadCapability,
    };

    for (auto ch : channels) {
        const std::string empty_req = makeConsumedFlatbuffersRequest(ch);
        void* resp_buf = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            cons::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           "application/flatbuffers", &resp_buf, &resp_size));
        if (resp_buf) std::free(resp_buf);
    }
}

TEST(ProtoBindingsProvided, DispatchAllChannelsProtobufNoCrash) {
    prov::ServiceHandler handler;
    const prov::ServiceChannel channels[] = {
        prov::ServiceChannel::MatchingObjectsReadMatch,
        prov::ServiceChannel::ObjectOfInterestCreateRequirement,
        prov::ServiceChannel::ObjectOfInterestReadRequirement,
        prov::ServiceChannel::ObjectOfInterestUpdateRequirement,
        prov::ServiceChannel::ObjectOfInterestDeleteRequirement,
        prov::ServiceChannel::SpecificObjectDetailReadDetail,
    };

    for (auto ch : channels) {
        const std::string empty_req = makeProvidedProtobufRequest(ch);
        void* resp_buf = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            prov::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           "application/protobuf", &resp_buf, &resp_size));
        if (resp_buf) std::free(resp_buf);
    }
}

TEST(ProtoBindingsConsumed, DispatchAllChannelsProtobufNoCrash) {
    cons::ServiceHandler handler;
    const cons::ServiceChannel channels[] = {
        cons::ServiceChannel::ObjectEvidenceReadDetail,
        cons::ServiceChannel::ObjectSolutionEvidenceCreateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceReadRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
        cons::ServiceChannel::ObjectSolutionEvidenceDeleteRequirement,
        cons::ServiceChannel::ObjectSourceCapabilityReadCapability,
    };

    for (auto ch : channels) {
        const std::string empty_req = makeConsumedProtobufRequest(ch);
        void* resp_buf = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            cons::dispatch(handler, ch, empty_req.data(), empty_req.size(),
                           "application/protobuf", &resp_buf, &resp_size));
        if (resp_buf) std::free(resp_buf);
    }
}

// ===========================================================================
// Typed dispatch -- round-trip through handler with real serialization
// ===========================================================================

TEST(ProtoBindingsProvided, DispatchCreateRequirementRoundTrip) {
    // Custom handler that captures the deserialized request
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier
        handleObjectOfInterestCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-42";
        }
    };

    CapturingHandler handler;

    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Obtain;
    std::string req_json = tactical_codec::toJson(req);

    void*  resp_buf  = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler,
                   prov::ServiceChannel::ObjectOfInterestCreateRequirement,
                   req_json.data(), req_json.size(),
                   &resp_buf, &resp_size);

    // Handler should have received deserialized request
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);

    // Response should be the proto-native Identifier payload.
    ASSERT_NE(resp_buf, nullptr);
    std::string resp_str(static_cast<const char*>(resp_buf), resp_size);
    auto resp = nlohmann::json::parse(resp_str);
    EXPECT_EQ(resp.get<std::string>(), "new-id-42");
    std::free(resp_buf);
}

TEST(ProtoBindingsProvided, DispatchCreateRequirementFlatBuffersRoundTrip) {
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier
        handleObjectOfInterestCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-42";
        }
    };

    CapturingHandler handler;
    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Obtain;
    const std::string req_payload = flatbuffers_codec::toBinary(req);

    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler,
                   prov::ServiceChannel::ObjectOfInterestCreateRequirement,
                   req_payload.data(), req_payload.size(),
                   "application/flatbuffers", &resp_buf, &resp_size);

    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = flatbuffers_codec::fromBinaryIdentifier(resp_buf, resp_size);
    EXPECT_EQ(resp, "new-id-42");
    std::free(resp_buf);
}

TEST(ProtoBindingsProvided, DispatchReadMatchReturnsJsonArray) {
    struct MatchHandler : public prov::ServiceHandler {
        std::vector<types::ObjectMatch>
        handleMatchingObjectsReadMatch(const types::Query& /*req*/) override {
            types::ObjectMatch m;
            m.id = "obj-1";
            m.matching_object_id = "match-1";
            return {m};
        }
    };

    MatchHandler handler;
    std::string req_json = "{}";

    void*  resp_buf  = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::MatchingObjectsReadMatch,
                   req_json.data(), req_json.size(),
                   &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    std::string resp_str(static_cast<const char*>(resp_buf), resp_size);
    auto arr = nlohmann::json::parse(resp_str);
    ASSERT_TRUE(arr.is_array());
    ASSERT_EQ(arr.size(), 1u);
    EXPECT_EQ(arr[0]["matching_object_id"], "match-1");
    std::free(resp_buf);
}

TEST(ProtoBindingsConsumed, DispatchUpdateRequirementRoundTrip) {
    struct AckHandler : public cons::ServiceHandler {
        types::ObjectEvidenceRequirement captured_req;
        types::Ack
        handleObjectSolutionEvidenceUpdateRequirement(
            const types::ObjectEvidenceRequirement& req) override {
            captured_req = req;
            return types::kAckOk;
        }
    };

    AckHandler handler;
    types::ObjectEvidenceRequirement req;
    req.policy = types::DataPolicy::Query;
    std::string req_json = tactical_codec::toJson(req);

    void*  resp_buf  = nullptr;
    size_t resp_size = 0;
    cons::dispatch(handler,
                   cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
                   req_json.data(), req_json.size(),
                   &resp_buf, &resp_size);

    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Query);

    ASSERT_NE(resp_buf, nullptr);
    std::string resp_str(static_cast<const char*>(resp_buf), resp_size);
    auto ack_j = nlohmann::json::parse(resp_str);
    EXPECT_TRUE(ack_j["success"].get<bool>());
    std::free(resp_buf);
}

TEST(ProtoBindingsConsumed, DispatchUpdateRequirementFlatBuffersRoundTrip) {
    struct AckHandler : public cons::ServiceHandler {
        types::ObjectEvidenceRequirement captured_req;
        types::Ack
        handleObjectSolutionEvidenceUpdateRequirement(
            const types::ObjectEvidenceRequirement& req) override {
            captured_req = req;
            return types::kAckOk;
        }
    };

    AckHandler handler;
    types::ObjectEvidenceRequirement req;
    req.policy = types::DataPolicy::Query;
    const std::string req_payload = flatbuffers_codec::toBinary(req);

    void* resp_buf = nullptr;
    size_t resp_size = 0;
    cons::dispatch(handler,
                   cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
                   req_payload.data(), req_payload.size(),
                   "application/flatbuffers", &resp_buf, &resp_size);

    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Query);

    ASSERT_NE(resp_buf, nullptr);
    auto ack = flatbuffers_codec::fromBinaryAck(resp_buf, resp_size);
    EXPECT_TRUE(ack.success);
    std::free(resp_buf);
}

TEST(ProtoBindingsProvided, DispatchCreateRequirementProtobufRoundTrip) {
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier
        handleObjectOfInterestCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-4242";
        }
    };

    CapturingHandler handler;
    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Obtain;
    const std::string req_payload = protobuf_codec::toBinary(req);

    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler,
                   prov::ServiceChannel::ObjectOfInterestCreateRequirement,
                   req_payload.data(), req_payload.size(),
                   "application/protobuf", &resp_buf, &resp_size);

    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);
    ASSERT_NE(resp_buf, nullptr);
    auto resp = protobuf_codec::fromBinaryIdentifier(resp_buf, resp_size);
    EXPECT_EQ(resp, "new-id-4242");
    std::free(resp_buf);
}

TEST(ProtoBindingsConsumed, DispatchUpdateRequirementProtobufRoundTrip) {
    struct AckHandler : public cons::ServiceHandler {
        types::ObjectEvidenceRequirement captured_req;
        types::Ack
        handleObjectSolutionEvidenceUpdateRequirement(
            const types::ObjectEvidenceRequirement& req) override {
            captured_req = req;
            return types::kAckOk;
        }
    };

    AckHandler handler;
    types::ObjectEvidenceRequirement req;
    req.policy = types::DataPolicy::Query;
    const std::string req_payload = protobuf_codec::toBinary(req);

    void* resp_buf = nullptr;
    size_t resp_size = 0;
    cons::dispatch(handler,
                   cons::ServiceChannel::ObjectSolutionEvidenceUpdateRequirement,
                   req_payload.data(), req_payload.size(),
                   "application/protobuf", &resp_buf, &resp_size);

    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Query);
    ASSERT_NE(resp_buf, nullptr);
    auto ack = protobuf_codec::fromBinaryAck(resp_buf, resp_size);
    EXPECT_TRUE(ack.success);
    std::free(resp_buf);
}

// ===========================================================================
// Domain types -- sanity checks on default values
// ===========================================================================

TEST(ProtoBindingsTypes, AckConstants) {
    EXPECT_TRUE(types::kAckOk.success);
    EXPECT_FALSE(types::kAckFail.success);
}

TEST(ProtoBindingsTypes, QueryDefault) {
    types::Query q;
    EXPECT_FALSE(q.one_shot.has_value());  // proto3: unset optional
    EXPECT_TRUE(q.id.empty());
}

TEST(ProtoBindingsTypes, IdentifierIsString) {
    types::Identifier id = "test-uuid-1234";
    EXPECT_EQ(id, "test-uuid-1234");
}

TEST(ProtoBindingsTypes, ObjectDetailDefaults) {
    types::ObjectDetail d;
    EXPECT_EQ(d.id, "");
    EXPECT_FALSE(d.quality.has_value());
    EXPECT_FALSE(d.course.has_value());
    EXPECT_FALSE(d.speed.has_value());
    EXPECT_FALSE(d.length.has_value());
    EXPECT_EQ(d.identity,  types::StandardIdentity::Unspecified);
    EXPECT_EQ(d.dimension, types::BattleDimension::Unspecified);
    EXPECT_TRUE(d.source.empty());  // repeated ObjectSource field
}
