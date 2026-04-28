/// \file test_codec_dispatch_e2e.cpp
/// \brief E2E tests for generated multi-codec PCL communication.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>
}

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstring>
#include <string>
#include <vector>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

namespace prov = pyramid::components::tactical_objects::services::provided;
namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
namespace protobuf_codec = pyramid::services::tactical_objects::protobuf_codec;
namespace types = pyramid::domain_model;
namespace tactical_codec = pyramid::domain_model::tactical;

namespace {

types::ObjectDetail makeTestEvidence() {
    types::ObjectDetail ev;
    ev.id = "obj-1";
    ev.identity = types::StandardIdentity::Hostile;
    ev.dimension = types::BattleDimension::SeaSurface;
    ev.position.latitude = 0.8901;
    ev.position.longitude = 0.0012;
    ev.creation_time = 1.5;
    ev.quality = 0.95;
    return ev;
}

void expectEvidenceEqual(const types::ObjectDetail& a,
                         const types::ObjectDetail& b) {
    EXPECT_EQ(a.id, b.id);
    EXPECT_EQ(a.identity, b.identity);
    EXPECT_EQ(a.dimension, b.dimension);
    EXPECT_DOUBLE_EQ(a.position.latitude, b.position.latitude);
    EXPECT_DOUBLE_EQ(a.position.longitude, b.position.longitude);
    ASSERT_TRUE(a.quality.has_value());
    ASSERT_TRUE(b.quality.has_value());
    EXPECT_DOUBLE_EQ(a.quality.value(), b.quality.value());
    EXPECT_DOUBLE_EQ(a.creation_time, b.creation_time);
}

std::string serializeEvidence(const types::ObjectDetail& ev,
                              const char* content_type) {
    if (std::strcmp(content_type, "application/json") == 0)
        return tactical_codec::toJson(ev);
    if (std::strcmp(content_type, "application/flatbuffers") == 0)
        return flatbuffers_codec::toBinary(ev);
    if (std::strcmp(content_type, "application/protobuf") == 0)
        return protobuf_codec::toBinary(ev);
    return {};
}

bool deserializeEvidence(const void* data,
                         size_t size,
                         const char* content_type,
                         types::ObjectDetail& out) {
    try {
        if (std::strcmp(content_type, "application/json") == 0) {
            out = tactical_codec::fromJson(
                std::string(static_cast<const char*>(data), size),
                static_cast<types::ObjectDetail*>(nullptr));
            return true;
        }
        if (std::strcmp(content_type, "application/flatbuffers") == 0) {
            out = flatbuffers_codec::fromBinaryObjectDetail(data, size);
            return true;
        }
        if (std::strcmp(content_type, "application/protobuf") == 0) {
            out = protobuf_codec::fromBinaryObjectDetail(data, size);
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
    types::ObjectDetail received{};
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
                        const types::ObjectDetail& evidence) {
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

TEST(CodecDispatchE2E, ProtobufPubSubRoundTrip) {
    runPubSubRoundTrip("application/protobuf", makeTestEvidence());
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
                types::ObjectDetail parsed{};
                if (deserializeEvidence(msg->data, msg->size, "application/json", parsed))
                    s->json_count.fetch_add(1);
            }, st);
        pcl_container_add_subscriber(
            c, "test.flat", "application/flatbuffers",
            [](pcl_container_t*, const pcl_msg_t* msg, void* u) {
                auto* s = static_cast<State*>(u);
                types::ObjectDetail parsed{};
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
        auto payload = tactical_codec::toJson(ev);
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
    auto json_payload = tactical_codec::toJson(ev);
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

    types::ObjectMatch match{};
    match.matching_object_id = "obj-1";
    auto payload = std::string("[") + tactical_codec::toJson(match) + "]";
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

    types::ObjectMatch match{};
    match.matching_object_id = "obj-1";
    std::vector<types::ObjectMatch> matches{match};
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

TEST(CodecDispatchE2E, JsonProtoCodecSerDeRoundTrip) {
    types::ObjectEvidenceRequirement req;
    req.base.id = "interest-7";
    req.policy = types::DataPolicy::Obtain;
    req.dimension.push_back(types::BattleDimension::SeaSurface);
    req.point = types::Point{};
    req.point->position.latitude = 0.873;
    req.point->position.longitude = -0.017;

    auto json_str = tactical_codec::toJson(req);
    auto req2 = tactical_codec::fromJson(
        json_str, static_cast<types::ObjectEvidenceRequirement*>(nullptr));

    EXPECT_EQ(req2.base.id, "interest-7");
    EXPECT_EQ(req2.policy, types::DataPolicy::Obtain);
    ASSERT_EQ(req2.dimension.size(), 1u);
    EXPECT_EQ(req2.dimension.front(), types::BattleDimension::SeaSurface);
    ASSERT_TRUE(req2.point.has_value());
    EXPECT_DOUBLE_EQ(req2.point->position.latitude, 0.873);
    EXPECT_DOUBLE_EQ(req2.point->position.longitude, -0.017);
}

TEST(CodecDispatchE2E, FlatBuffersCodecUnitRoundTrip) {
    auto ev = makeTestEvidence();
    auto payload = flatbuffers_codec::toBinary(ev);
    auto roundtripped = flatbuffers_codec::fromBinaryObjectDetail(payload);
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

    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Obtain;

    auto payload = tactical_codec::toJson(req);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::CreateRequirement,
                   payload.data(), payload.size(),
                   "application/json", &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = nlohmann::json::parse(
        std::string(static_cast<const char*>(resp_buf), resp_size));
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);
    EXPECT_EQ(resp.get<std::string>(), "new-id-42");
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

    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Query;
    req.dimension.push_back(types::BattleDimension::SeaSurface);

    auto payload = flatbuffers_codec::toBinary(req);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::CreateRequirement,
                   payload.data(), payload.size(),
                   "application/flatbuffers", &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = flatbuffers_codec::fromBinaryIdentifier(resp_buf, resp_size);
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Query);
    ASSERT_EQ(handler.captured_req.dimension.size(), 1u);
    EXPECT_EQ(handler.captured_req.dimension[0], types::BattleDimension::SeaSurface);
    EXPECT_EQ(resp, "new-id-84");
    std::free(resp_buf);
}

TEST(CodecDispatchE2E, ProtobufCreateRequirementDispatchRoundTrip) {
    struct CapturingHandler : public prov::ServiceHandler {
        types::ObjectInterestRequirement captured_req;
        types::Identifier handleCreateRequirement(
            const types::ObjectInterestRequirement& req) override {
            captured_req = req;
            return "new-id-168";
        }
    } handler;

    types::ObjectInterestRequirement req;
    req.policy = types::DataPolicy::Obtain;
    req.dimension.push_back(types::BattleDimension::Air);

    auto payload = protobuf_codec::toBinary(req);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(handler, prov::ServiceChannel::CreateRequirement,
                   payload.data(), payload.size(),
                   "application/protobuf", &resp_buf, &resp_size);

    ASSERT_NE(resp_buf, nullptr);
    auto resp = protobuf_codec::fromBinaryIdentifier(resp_buf, resp_size);
    EXPECT_EQ(handler.captured_req.policy, types::DataPolicy::Obtain);
    ASSERT_EQ(handler.captured_req.dimension.size(), 1u);
    EXPECT_EQ(handler.captured_req.dimension[0], types::BattleDimension::Air);
    EXPECT_EQ(resp, "new-id-168");
    std::free(resp_buf);
}
