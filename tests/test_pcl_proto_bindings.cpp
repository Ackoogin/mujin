/// \file test_pcl_proto_bindings.cpp
/// \brief Tests for C++ PCL/proto service bindings (provided and consumed).
///
/// Validates the auto-generated C++ service bindings that mirror the Ada
/// generated files in examples/ada/generated/:
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
#include "pyramid_services_tactical_objects_json_codec.hpp"
#include "pyramid_services_tactical_objects_types.hpp"

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>
}

#include <nlohmann/json.hpp>

#include <atomic>
#include <string>
#include <vector>

namespace prov = pyramid::services::tactical_objects::provided;
namespace cons = pyramid::services::tactical_objects::consumed;
namespace codec = pyramid::services::tactical_objects::json_codec;
namespace types = pyramid::services::tactical_objects;

// ===========================================================================
// Wire-name constant tests
// ===========================================================================

TEST(ProtoBindingsProvided, WireNames) {
    EXPECT_STREQ(prov::kSvcReadMatch,         "matching_objects.read_match");
    EXPECT_STREQ(prov::kSvcCreateRequirement, "object_of_interest.create_requirement");
    EXPECT_STREQ(prov::kSvcReadRequirement,   "object_of_interest.read_requirement");
    EXPECT_STREQ(prov::kSvcUpdateRequirement, "object_of_interest.update_requirement");
    EXPECT_STREQ(prov::kSvcDeleteRequirement, "object_of_interest.delete_requirement");
    EXPECT_STREQ(prov::kSvcReadDetail,        "specific_object_detail.read_detail");
}

TEST(ProtoBindingsProvided, TopicNames) {
    EXPECT_STREQ(prov::kTopicEntityMatches,        "standard.entity_matches");
    EXPECT_STREQ(prov::kTopicEvidenceRequirements, "standard.evidence_requirements");
}

TEST(ProtoBindingsConsumed, WireNames) {
    EXPECT_STREQ(cons::kSvcReadDetail,         "object_evidence.read_detail");
    EXPECT_STREQ(cons::kSvcCreateRequirement,  "object_solution_evidence.create_requirement");
    EXPECT_STREQ(cons::kSvcReadRequirement,    "object_solution_evidence.read_requirement");
    EXPECT_STREQ(cons::kSvcUpdateRequirement,  "object_solution_evidence.update_requirement");
    EXPECT_STREQ(cons::kSvcDeleteRequirement,  "object_solution_evidence.delete_requirement");
    EXPECT_STREQ(cons::kSvcReadCapability,     "object_source_capability.read_capability");
}

TEST(ProtoBindingsConsumed, TopicNames) {
    EXPECT_STREQ(cons::kTopicObjectEvidence, "standard.object_evidence");
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

// ===========================================================================
// JSON codec — CreateRequirementRequest (replaces buildStandardRequirementJson)
// ===========================================================================

TEST(ProtoBindingsProvided, CreateRequirementJsonKeys) {
    codec::CreateRequirementRequest req;
    req.policy      = codec::DataPolicy::Query;
    req.identity    = codec::StandardIdentity::Hostile;
    req.dimension   = codec::BattleDimension::Air;
    req.min_lat_rad = 0.1;
    req.max_lat_rad = 0.2;
    req.min_lon_rad = 0.3;
    req.max_lon_rad = 0.4;

    std::string json_str = codec::toJson(req);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_TRUE(j.contains("policy"));
    EXPECT_TRUE(j.contains("identity"));
    EXPECT_TRUE(j.contains("dimension"));
    EXPECT_DOUBLE_EQ(j["min_lat_rad"].get<double>(), 0.1);
    EXPECT_DOUBLE_EQ(j["max_lat_rad"].get<double>(), 0.2);
    EXPECT_DOUBLE_EQ(j["min_lon_rad"].get<double>(), 0.3);
    EXPECT_DOUBLE_EQ(j["max_lon_rad"].get<double>(), 0.4);
}

TEST(ProtoBindingsProvided, CreateRequirementJsonDefaults) {
    // Default-constructed request: zero-valued coords are omitted by codec
    codec::CreateRequirementRequest req;
    std::string json_str = codec::toJson(req);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_FALSE(j.contains("min_lat_rad"));
    EXPECT_FALSE(j.contains("max_lat_rad"));
    EXPECT_FALSE(j.contains("dimension"));
}

// ===========================================================================
// JSON codec — ObjectEvidence (replaces buildStandardEvidenceJson)
// ===========================================================================

TEST(ProtoBindingsProvided, EvidenceJsonKeys) {
    codec::ObjectEvidence ev;
    ev.identity      = codec::StandardIdentity::Friendly;
    ev.dimension     = codec::BattleDimension::Ground;
    ev.latitude_rad  = 0.785;
    ev.longitude_rad = -1.571;
    ev.confidence    = 0.9;
    ev.observed_at   = 0.75;

    std::string json_str = codec::toJson(ev);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_TRUE(j.contains("identity"));
    EXPECT_TRUE(j.contains("dimension"));
    EXPECT_DOUBLE_EQ(j["latitude_rad"].get<double>(),  0.785);
    EXPECT_DOUBLE_EQ(j["longitude_rad"].get<double>(), -1.571);
    EXPECT_DOUBLE_EQ(j["confidence"].get<double>(),    0.9);
    EXPECT_DOUBLE_EQ(j["observed_at"].get<double>(),   0.75);
}

TEST(ProtoBindingsProvided, EvidenceJsonDefaultObservedAt) {
    // Default-constructed evidence: observed_at = 0.0 is omitted by codec
    codec::ObjectEvidence ev;
    std::string json_str = codec::toJson(ev);
    auto j = nlohmann::json::parse(json_str);
    EXPECT_FALSE(j.contains("observed_at"));
}

// ===========================================================================
// ServiceHandler — default stub implementations
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
    handleReadMatch(const types::Query& q) override {
        read_match_called = true;
        return prov::ServiceHandler::handleReadMatch(q);  // call stub
    }

    types::Identifier
    handleCreateRequirement(const types::ObjectInterestRequirement& r) override {
        create_req_called = true;
        return prov::ServiceHandler::handleCreateRequirement(r);
    }

    std::vector<types::ObjectInterestRequirement>
    handleReadRequirement(const types::Query& q) override {
        read_req_called = true;
        return prov::ServiceHandler::handleReadRequirement(q);
    }

    types::Ack
    handleUpdateRequirement(const types::ObjectInterestRequirement& r) override {
        update_req_called = true;
        return prov::ServiceHandler::handleUpdateRequirement(r);
    }

    types::Ack
    handleDeleteRequirement(const types::Identifier& id) override {
        delete_req_called = true;
        return prov::ServiceHandler::handleDeleteRequirement(id);
    }

    std::vector<types::ObjectDetail>
    handleReadDetail(const types::Query& q) override {
        read_detail_called = true;
        return prov::ServiceHandler::handleReadDetail(q);
    }
};

TEST(ProtoBindingsProvided, HandlerStubsReturnEmpty) {
    TestProvidedHandler handler;

    types::Query q;
    auto matches = handler.handleReadMatch(q);
    EXPECT_TRUE(matches.empty());
    EXPECT_TRUE(handler.read_match_called);

    types::ObjectInterestRequirement req;
    auto id = handler.handleCreateRequirement(req);
    EXPECT_TRUE(id.empty());
    EXPECT_TRUE(handler.create_req_called);

    auto reqs = handler.handleReadRequirement(q);
    EXPECT_TRUE(reqs.empty());
    EXPECT_TRUE(handler.read_req_called);

    auto ack_update = handler.handleUpdateRequirement(req);
    EXPECT_TRUE(ack_update.success);
    EXPECT_TRUE(handler.update_req_called);

    auto ack_delete = handler.handleDeleteRequirement("some-id");
    EXPECT_TRUE(ack_delete.success);
    EXPECT_TRUE(handler.delete_req_called);

    auto details = handler.handleReadDetail(q);
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
    handleReadDetail(const types::Query& q) override {
        read_detail_called = true;
        return cons::ServiceHandler::handleReadDetail(q);
    }

    types::Identifier
    handleCreateRequirement(const types::ObjectEvidenceRequirement& r) override {
        create_req_called = true;
        return cons::ServiceHandler::handleCreateRequirement(r);
    }

    std::vector<types::ObjectEvidenceRequirement>
    handleReadRequirement(const types::Query& q) override {
        read_req_called = true;
        return cons::ServiceHandler::handleReadRequirement(q);
    }

    types::Ack
    handleUpdateRequirement(const types::ObjectEvidenceRequirement& r) override {
        update_req_called = true;
        return cons::ServiceHandler::handleUpdateRequirement(r);
    }

    types::Ack
    handleDeleteRequirement(const types::Identifier& id) override {
        delete_req_called = true;
        return cons::ServiceHandler::handleDeleteRequirement(id);
    }

    std::vector<types::Identifier>
    handleReadCapability(const types::Query& q) override {
        read_cap_called = true;
        return cons::ServiceHandler::handleReadCapability(q);
    }
};

TEST(ProtoBindingsConsumed, HandlerStubsReturnEmpty) {
    TestConsumedHandler handler;

    types::Query q;
    auto details = handler.handleReadDetail(q);
    EXPECT_TRUE(details.empty());
    EXPECT_TRUE(handler.read_detail_called);

    types::ObjectEvidenceRequirement req;
    auto id = handler.handleCreateRequirement(req);
    EXPECT_TRUE(id.empty());
    EXPECT_TRUE(handler.create_req_called);

    auto reqs = handler.handleReadRequirement(q);
    EXPECT_TRUE(reqs.empty());
    EXPECT_TRUE(handler.read_req_called);

    auto ack_update = handler.handleUpdateRequirement(req);
    EXPECT_TRUE(ack_update.success);
    EXPECT_TRUE(handler.update_req_called);

    auto ack_delete = handler.handleDeleteRequirement("ev-id");
    EXPECT_TRUE(ack_delete.success);
    EXPECT_TRUE(handler.delete_req_called);

    auto caps = handler.handleReadCapability(q);
    EXPECT_TRUE(caps.empty());
    EXPECT_TRUE(handler.read_cap_called);
}

// ===========================================================================
// PCL subscribe registration — verify ports are created during on_configure
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
// publishObjectEvidence — verify publish returns an error when not active
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

    // Container is CONFIGURED but not ACTIVE — port should be closed.
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
// Dispatch — verify all ServiceChannel values are handled without crash
// ===========================================================================

TEST(ProtoBindingsProvided, DispatchAllChannelsNoCrash) {
    const prov::ServiceChannel channels[] = {
        prov::ServiceChannel::ReadMatch,
        prov::ServiceChannel::CreateRequirement,
        prov::ServiceChannel::ReadRequirement,
        prov::ServiceChannel::UpdateRequirement,
        prov::ServiceChannel::DeleteRequirement,
        prov::ServiceChannel::ReadDetail,
    };

    for (auto ch : channels) {
        void*  resp_buf  = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            prov::dispatch(ch, nullptr, 0, &resp_buf, &resp_size));
        EXPECT_EQ(resp_buf,  nullptr);
        EXPECT_EQ(resp_size, 0u);
    }
}

TEST(ProtoBindingsConsumed, DispatchAllChannelsNoCrash) {
    const cons::ServiceChannel channels[] = {
        cons::ServiceChannel::ReadDetail,
        cons::ServiceChannel::CreateRequirement,
        cons::ServiceChannel::ReadRequirement,
        cons::ServiceChannel::UpdateRequirement,
        cons::ServiceChannel::DeleteRequirement,
        cons::ServiceChannel::ReadCapability,
    };

    for (auto ch : channels) {
        void*  resp_buf  = nullptr;
        size_t resp_size = 0;
        EXPECT_NO_FATAL_FAILURE(
            cons::dispatch(ch, nullptr, 0, &resp_buf, &resp_size));
        EXPECT_EQ(resp_buf,  nullptr);
        EXPECT_EQ(resp_size, 0u);
    }
}

// ===========================================================================
// Domain types — sanity checks on default values
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
