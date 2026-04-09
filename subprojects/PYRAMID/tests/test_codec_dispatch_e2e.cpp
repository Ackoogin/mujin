/// \file test_codec_dispatch_e2e.cpp
/// \brief E2E tests for generated multi-codec PCL communication.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>
}

#include <atomic>
#include <cstring>
#include <string>
#include <vector>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_services_tactical_objects_json_codec.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"

namespace prov = pyramid::services::tactical_objects::provided;
namespace cons = pyramid::services::tactical_objects::consumed;
namespace json_codec = pyramid::services::tactical_objects::json_codec;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
namespace wire_types = pyramid::services::tactical_objects::wire_types;
namespace types = pyramid::data_model;

namespace {

wire_types::ObjectEvidence makeTestEvidence() {
    wire_types::ObjectEvidence ev;
    ev.identity = types::StandardIdentity::Hostile;
    ev.dimension = types::BattleDimension::SeaSurface;
    ev.latitude_rad = 0.8901;
    ev.longitude_rad = 0.0012;
    ev.confidence = 0.95;
    ev.observed_at = 1.5;
    return ev;
}

void expectEvidenceEqual(const wire_types::ObjectEvidence& a,
                         const wire_types::ObjectEvidence& b) {
    EXPECT_EQ(a.identity, b.identity);
    EXPECT_EQ(a.dimension, b.dimension);
    EXPECT_DOUBLE_EQ(a.latitude_rad, b.latitude_rad);
    EXPECT_DOUBLE_EQ(a.longitude_rad, b.longitude_rad);
    EXPECT_DOUBLE_EQ(a.confidence, b.confidence);
    EXPECT_DOUBLE_EQ(a.observed_at, b.observed_at);
}

std::string serializeEvidence(const wire_types::ObjectEvidence& ev,
                              const char* content_type) {
    if (std::strcmp(content_type, "application/json") == 0)
        return json_codec::toJson(ev);
    if (std::strcmp(content_type, "application/flatbuffers") == 0)
        return flatbuffers_codec::toBinary(ev);
    return {};
}

bool deserializeEvidence(const void* data,
                         size_t size,
                         const char* content_type,
                         wire_types::ObjectEvidence& out) {
    try {
        if (std::strcmp(content_type, "application/json") == 0) {
            out = json_codec::objectEvidenceFromJson(
                std::string(static_cast<const char*>(data), size));
            return true;
        }
        if (std::strcmp(content_type, "application/flatbuffers") == 0) {
            out = flatbuffers_codec::fromBinaryObjectEvidence(data, size);
            return true;
        }
    } catch (...) {
        return false;
    }
    return false;
}

struct CodecPubSubState {
    pcl_port_t* pub_port = nullptr;
    std::string content_type = "application/json";

    std::atomic<int> messages_received{0};
    std::string last_type_name;
    wire_types::ObjectEvidence received{};
    bool deserialize_ok = false;
};

void codec_sub_cb(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<CodecPubSubState*>(ud);
    if (!msg || !msg->data || msg->size == 0) return;

    st->last_type_name = msg->type_name ? msg->type_name : "";
    st->deserialize_ok = deserializeEvidence(
        msg->data, msg->size, st->last_type_name.c_str(), st->received);
    st->messages_received.fetch_add(1);
}

pcl_status_t configure_codec_pub(pcl_container_t* c, void* ud) {
    auto* st = static_cast<CodecPubSubState*>(ud);
    st->pub_port = pcl_container_add_publisher(
        c, cons::kTopicObjectEvidence, st->content_type.c_str());
    return PCL_OK;
}

pcl_status_t configure_codec_sub(pcl_container_t* c, void* ud) {
    auto* st = static_cast<CodecPubSubState*>(ud);
    cons::subscribeObjectEvidence(c, codec_sub_cb, ud, st->content_type.c_str());
    return PCL_OK;
}

void runPubSubRoundTrip(const char* content_type,
                        const wire_types::ObjectEvidence& evidence) {
    CodecPubSubState state;
    state.content_type = content_type;

    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = configure_codec_pub;
    pcl_container_t* pub_c =
        pcl_container_create("codec_pub", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = configure_codec_sub;
    pcl_container_t* sub_c =
        pcl_container_create("codec_sub", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    pcl_status_t rc = cons::publishObjectEvidence(
        state.pub_port, evidence, content_type);
    ASSERT_EQ(rc, PCL_OK);

    for (int i = 0; i < 10; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.messages_received.load() > 0) break;
    }

    ASSERT_GE(state.messages_received.load(), 1);
    EXPECT_EQ(state.last_type_name, content_type);
    EXPECT_TRUE(state.deserialize_ok);
    expectEvidenceEqual(state.received, evidence);

    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

struct TypeOnlyState {
    std::string subscribed_type;
};

void type_only_sub_cb(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<TypeOnlyState*>(ud);
    if (msg && msg->type_name)
        st->subscribed_type = msg->type_name;
}

pcl_status_t configure_default_codec(pcl_container_t* c, void* ud) {
    prov::subscribeEntityMatches(c, type_only_sub_cb, ud);
    return PCL_OK;
}

pcl_status_t configure_flatbuffers_codec(pcl_container_t* c, void* ud) {
    prov::subscribeEntityMatches(c, type_only_sub_cb, ud, "application/flatbuffers");
    return PCL_OK;
}

}  // namespace

TEST(CodecDispatchE2E, JsonPubSubRoundTrip) {
    runPubSubRoundTrip("application/json", makeTestEvidence());
}

TEST(CodecDispatchE2E, FlatBuffersPubSubRoundTrip) {
    runPubSubRoundTrip("application/flatbuffers", makeTestEvidence());
}

TEST(CodecDispatchE2E, ContentTypePropagation) {
    CodecPubSubState state;
    state.content_type = "application/flatbuffers";

    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = configure_codec_pub;
    pcl_container_t* pub_c =
        pcl_container_create("fb_pub", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = configure_codec_sub;
    pcl_container_t* sub_c =
        pcl_container_create("fb_sub", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    auto ev = makeTestEvidence();
    auto payload = flatbuffers_codec::toBinary(ev);
    pcl_status_t rc = cons::publishObjectEvidence(
        state.pub_port, payload, "application/flatbuffers");
    ASSERT_EQ(rc, PCL_OK);

    for (int i = 0; i < 10; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.messages_received.load() > 0) break;
    }

    EXPECT_GE(state.messages_received.load(), 1);
    EXPECT_EQ(state.last_type_name, "application/flatbuffers");
    EXPECT_TRUE(state.deserialize_ok);

    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

TEST(CodecDispatchE2E, TwoCodecsTwoPorts) {
    struct State {
        pcl_port_t* json_pub = nullptr;
        pcl_port_t* flat_pub = nullptr;
        std::atomic<int> json_count{0};
        std::atomic<int> flat_count{0};
    } state;

    auto ev = makeTestEvidence();
    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = [](pcl_container_t* c, void* ud) {
        auto* st = static_cast<State*>(ud);
        st->json_pub = pcl_container_add_publisher(c, "test.json", "application/json");
        st->flat_pub = pcl_container_add_publisher(c, "test.flat", "application/flatbuffers");
        return PCL_OK;
    };
    pcl_container_t* pub_c = pcl_container_create("two_pub", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = [](pcl_container_t* c, void* ud) {
        auto* st = static_cast<State*>(ud);
        pcl_container_add_subscriber(
            c, "test.json", "application/json",
            [](pcl_container_t*, const pcl_msg_t* msg, void* u) {
                auto* s = static_cast<State*>(u);
                wire_types::ObjectEvidence parsed{};
                if (deserializeEvidence(msg->data, msg->size, "application/json", parsed))
                    s->json_count.fetch_add(1);
            }, st);
        pcl_container_add_subscriber(
            c, "test.flat", "application/flatbuffers",
            [](pcl_container_t*, const pcl_msg_t* msg, void* u) {
                auto* s = static_cast<State*>(u);
                wire_types::ObjectEvidence parsed{};
                if (deserializeEvidence(msg->data, msg->size, "application/flatbuffers", parsed))
                    s->flat_count.fetch_add(1);
            }, st);
        return PCL_OK;
    };
    pcl_container_t* sub_c = pcl_container_create("two_sub", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    {
        auto payload = json_codec::toJson(ev);
        pcl_msg_t msg{};
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = "application/json";
        pcl_port_publish(state.json_pub, &msg);
    }
    {
        auto payload = flatbuffers_codec::toBinary(ev);
        pcl_msg_t msg{};
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = "application/flatbuffers";
        pcl_port_publish(state.flat_pub, &msg);
    }

    for (int i = 0; i < 20; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.json_count.load() > 0 && state.flat_count.load() > 0) break;
    }

    EXPECT_GE(state.json_count.load(), 1);
    EXPECT_GE(state.flat_count.load(), 1);

    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

TEST(CodecDispatchE2E, BinaryCodecSmallerThanJson) {
    auto ev = makeTestEvidence();
    auto json_payload = json_codec::toJson(ev);
    auto flat_payload = flatbuffers_codec::toBinary(ev);

    EXPECT_LT(flat_payload.size(), json_payload.size());
}

TEST(CodecDispatchE2E, DefaultCodecIsJson) {
    TypeOnlyState state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_default_codec;

    pcl_container_t* c = pcl_container_create("default_codec", &cbs, &state);
    ASSERT_NE(c, nullptr);
    pcl_container_configure(c);
    pcl_container_activate(c);

    pcl_executor_t* exec = pcl_executor_create();
    pcl_executor_add(exec, c);

    wire_types::EntityMatch match{};
    match.object_id = "obj-1";
    auto payload = json_codec::toJson(match);
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";
    pcl_executor_dispatch_incoming(exec, prov::kTopicEntityMatches, &msg);
    pcl_executor_spin_once(exec, 0);

    EXPECT_EQ(state.subscribed_type, "application/json");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

TEST(CodecDispatchE2E, ExplicitCodecFlatBuffers) {
    TypeOnlyState state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_flatbuffers_codec;

    pcl_container_t* c = pcl_container_create("explicit_codec", &cbs, &state);
    ASSERT_NE(c, nullptr);
    pcl_container_configure(c);
    pcl_container_activate(c);

    pcl_executor_t* exec = pcl_executor_create();
    pcl_executor_add(exec, c);

    wire_types::EntityMatch match{};
    match.object_id = "obj-1";
    match.identity = types::StandardIdentity::Hostile;
    wire_types::EntityMatchArray matches{match};
    auto payload = flatbuffers_codec::toBinary(matches);
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/flatbuffers";
    pcl_executor_dispatch_incoming(exec, prov::kTopicEntityMatches, &msg);
    pcl_executor_spin_once(exec, 0);

    EXPECT_EQ(state.subscribed_type, "application/flatbuffers");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

TEST(CodecDispatchE2E, JsonCodecSerDeRoundTrip) {
    wire_types::CreateRequirementRequest req;
    req.policy = types::DataPolicy::Obtain;
    req.identity = types::StandardIdentity::Hostile;
    req.dimension = types::BattleDimension::SeaSurface;
    req.min_lat_rad = 0.873;
    req.max_lat_rad = 0.907;
    req.min_lon_rad = -0.017;
    req.max_lon_rad = 0.017;

    auto json_str = json_codec::toJson(req);
    auto req2 = json_codec::createRequirementRequestFromJson(json_str);

    EXPECT_EQ(req2.policy, types::DataPolicy::Obtain);
    EXPECT_EQ(req2.identity, types::StandardIdentity::Hostile);
    EXPECT_EQ(req2.dimension, types::BattleDimension::SeaSurface);
    EXPECT_DOUBLE_EQ(req2.min_lat_rad, 0.873);
    EXPECT_DOUBLE_EQ(req2.max_lat_rad, 0.907);
    EXPECT_DOUBLE_EQ(req2.min_lon_rad, -0.017);
    EXPECT_DOUBLE_EQ(req2.max_lon_rad, 0.017);
}

TEST(CodecDispatchE2E, FlatBuffersCodecUnitRoundTrip) {
    auto ev = makeTestEvidence();
    auto payload = flatbuffers_codec::toBinary(ev);
    auto roundtripped = flatbuffers_codec::fromBinaryObjectEvidence(payload);
    expectEvidenceEqual(roundtripped, ev);
}

TEST(CodecDispatchE2E, JsonCreateRequirementDispatchRoundTrip) {
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier handleCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-42";
        }
    } handler;

    wire_types::CreateRequirementRequest req;
    req.policy = types::DataPolicy::Obtain;
    req.identity = types::StandardIdentity::Hostile;

    auto payload = json_codec::toJson(req);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::CreateRequirement,
                   payload.data(), payload.size(),
                   "application/json", &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = json_codec::createRequirementResponseFromJson(
        std::string(static_cast<const char*>(resp_buf), resp_size));
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);
    EXPECT_EQ(resp.interest_id, "new-id-42");
    std::free(resp_buf);
}

TEST(CodecDispatchE2E, FlatBuffersCreateRequirementDispatchRoundTrip) {
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier handleCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-84";
        }
    } handler;

    wire_types::CreateRequirementRequest req;
    req.policy = types::DataPolicy::Query;
    req.identity = types::StandardIdentity::Hostile;
    req.dimension = types::BattleDimension::SeaSurface;
    req.min_lat_rad = 0.1;
    req.max_lat_rad = 0.2;
    req.min_lon_rad = 0.3;
    req.max_lon_rad = 0.4;

    auto payload = flatbuffers_codec::toBinary(req);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::CreateRequirement,
                   payload.data(), payload.size(),
                   "application/flatbuffers", &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = flatbuffers_codec::fromBinaryCreateRequirementResponse(
        resp_buf, resp_size);
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Query);
    ASSERT_EQ(handler.captured_req.dimension.size(), 1u);
    EXPECT_EQ(handler.captured_req.dimension[0], types::BattleDimension::SeaSurface);
    EXPECT_TRUE(handler.captured_req.poly_area.has_value());
    EXPECT_EQ(resp.interest_id, "new-id-84");
    std::free(resp_buf);
}
