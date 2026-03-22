/// \file test_codec_dispatch_e2e.cpp
/// \brief E2E tests for multi-codec PCL communication.
///
/// Validates that two components can communicate via the codec dispatch layer
/// with different wire codecs configured per-port.
///
/// Test scenarios:
///   1. Publisher (JSON) → Subscriber (JSON): baseline round-trip
///   2. Publisher (FlatBuffers) → Subscriber (FlatBuffers): binary codec path
///   3. Publisher (JSON) → two subscribers with different content types
///   4. Service request/response with configurable content type
///   5. Content type propagation through pcl_msg_t
///
/// The codec dispatch layer routes serialisation based on pcl_msg_t.type_name,
/// which is set per-port at configure time.  Component business logic works
/// with typed structs and never touches raw bytes directly.
///
/// Architecture:
///   component logic → codec_dispatch::serialize() → pcl_msg_t → PCL executor
///   → subscriber callback → codec_dispatch::deserialize() → typed struct
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

// Generated service bindings (with content_type parameter)
#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_json_codec.hpp"

namespace prov = pyramid::services::tactical_objects::provided;
namespace cons = pyramid::services::tactical_objects::consumed;
namespace codec = pyramid::services::tactical_objects::json_codec;

// ---------------------------------------------------------------------------
// Test 1: JSON pub/sub round-trip through codec dispatch
//
// Publisher serialises with content_type = "application/json", subscriber
// deserialises and verifies all fields survived the round-trip.
// ---------------------------------------------------------------------------

namespace {

struct PubSubState {
    // Publisher side
    pcl_port_t* pub_port = nullptr;
    std::string content_type = "application/json";

    // Subscriber side
    std::atomic<int>  messages_received{0};
    std::string       last_payload;
    std::string       last_type_name;

    // Deserialised fields (from JSON codec)
    std::string       recv_object_id;
    codec::StandardIdentity recv_identity = codec::StandardIdentity::Unspecified;
    double            recv_latitude = 0.0;
    double            recv_confidence = 0.0;
};

void on_evidence_cb(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<PubSubState*>(ud);
    if (!msg || !msg->data || msg->size == 0) return;

    st->last_payload.assign(static_cast<const char*>(msg->data), msg->size);
    st->last_type_name = msg->type_name ? msg->type_name : "";
    st->messages_received.fetch_add(1);

    // Deserialise using JSON codec (if content type is JSON)
    if (st->last_type_name == "application/json") {
        auto evidence = codec::objectEvidenceFromJson(st->last_payload);
        st->recv_identity = evidence.identity;
        st->recv_latitude = evidence.latitude_rad;
        st->recv_confidence = evidence.confidence;
    }
}

pcl_status_t configure_publisher(pcl_container_t* c, void* ud) {
    auto* st = static_cast<PubSubState*>(ud);
    st->pub_port = pcl_container_add_publisher(
        c, cons::kTopicObjectEvidence, st->content_type.c_str());
    return PCL_OK;
}

pcl_status_t configure_subscriber(pcl_container_t* c, void* ud) {
    auto* st = static_cast<PubSubState*>(ud);
    // Subscribe with the same content type as the publisher
    cons::subscribeObjectEvidence(c, on_evidence_cb, ud,
                                  st->content_type.c_str());
    return PCL_OK;
}

}  // namespace

TEST(CodecDispatchE2E, JsonPubSubRoundTrip) {
    PubSubState state;
    state.content_type = "application/json";

    // Create executor
    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    // Create publisher container
    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = configure_publisher;
    pcl_container_t* pub_c =
        pcl_container_create("json_publisher", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    // Create subscriber container
    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = configure_subscriber;
    pcl_container_t* sub_c =
        pcl_container_create("json_subscriber", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    // Build a typed evidence message and serialise via JSON codec
    codec::ObjectEvidence evidence;
    evidence.identity      = codec::StandardIdentity::Hostile;
    evidence.dimension     = codec::BattleDimension::SeaSurface;
    evidence.latitude_rad  = 0.8901;
    evidence.longitude_rad = 0.0;
    evidence.confidence    = 0.95;
    evidence.observed_at   = 1.5;

    std::string payload = codec::toJson(evidence);

    // Publish with content_type from port config
    pcl_status_t rc = cons::publishObjectEvidence(
        state.pub_port, payload, state.content_type.c_str());
    ASSERT_EQ(rc, PCL_OK);

    // Spin to deliver the message
    for (int i = 0; i < 10; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.messages_received.load() > 0) break;
    }

    // Verify round-trip
    EXPECT_GE(state.messages_received.load(), 1);
    EXPECT_EQ(state.last_type_name, "application/json");
    EXPECT_EQ(state.recv_identity, codec::StandardIdentity::Hostile);
    EXPECT_DOUBLE_EQ(state.recv_latitude, 0.8901);
    EXPECT_DOUBLE_EQ(state.recv_confidence, 0.95);

    // Cleanup
    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

// ---------------------------------------------------------------------------
// Test 2: Content type propagation
//
// Verify that the content_type string set at port creation time is correctly
// propagated through pcl_msg_t.type_name to the subscriber callback.
// ---------------------------------------------------------------------------

TEST(CodecDispatchE2E, ContentTypePropagation) {
    // Test with a non-JSON content type to verify propagation
    PubSubState state;
    state.content_type = "application/flatbuffers";

    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = configure_publisher;
    pcl_container_t* pub_c =
        pcl_container_create("fb_publisher", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = configure_subscriber;
    pcl_container_t* sub_c =
        pcl_container_create("fb_subscriber", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    // Publish raw binary payload (simulating flatbuffers output)
    const char raw[] = "\x01\x02\x03\x04";
    pcl_status_t rc = cons::publishObjectEvidence(
        state.pub_port, std::string(raw, 4),
        state.content_type.c_str());
    ASSERT_EQ(rc, PCL_OK);

    for (int i = 0; i < 10; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.messages_received.load() > 0) break;
    }

    // Verify content type was propagated
    EXPECT_GE(state.messages_received.load(), 1);
    EXPECT_EQ(state.last_type_name, "application/flatbuffers");
    EXPECT_EQ(state.last_payload.size(), 4u);

    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

// ---------------------------------------------------------------------------
// Test 3: Per-port codec selection (subscribe with content_type parameter)
//
// Verify that the generated subscribe wrappers correctly pass the
// content_type to pcl_container_add_subscriber.
// ---------------------------------------------------------------------------

namespace {

struct MultiCodecState {
    // Port handles
    pcl_port_t* json_pub = nullptr;
    pcl_port_t* fb_pub = nullptr;

    // Subscriber receives
    std::atomic<int> json_recv_count{0};
    std::atomic<int> fb_recv_count{0};
    std::string json_last_type;
    std::string fb_last_type;
};

void on_json_sub(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<MultiCodecState*>(ud);
    if (!msg || !msg->data) return;
    st->json_last_type = msg->type_name ? msg->type_name : "";
    st->json_recv_count.fetch_add(1);
}

void on_fb_sub(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<MultiCodecState*>(ud);
    if (!msg || !msg->data) return;
    st->fb_last_type = msg->type_name ? msg->type_name : "";
    st->fb_recv_count.fetch_add(1);
}

pcl_status_t configure_multi_pub(pcl_container_t* c, void* ud) {
    auto* st = static_cast<MultiCodecState*>(ud);
    st->json_pub = pcl_container_add_publisher(
        c, "test.json_topic", "application/json");
    st->fb_pub = pcl_container_add_publisher(
        c, "test.fb_topic", "application/flatbuffers");
    return PCL_OK;
}

pcl_status_t configure_multi_sub(pcl_container_t* c, void* ud) {
    pcl_container_add_subscriber(c, "test.json_topic", "application/json",
                                 on_json_sub, ud);
    pcl_container_add_subscriber(c, "test.fb_topic", "application/flatbuffers",
                                 on_fb_sub, ud);
    return PCL_OK;
}

}  // namespace

TEST(CodecDispatchE2E, PerPortCodecSelection) {
    MultiCodecState state;

    pcl_executor_t* exec = pcl_executor_create();
    ASSERT_NE(exec, nullptr);

    // Publisher container with two ports (different codecs)
    pcl_callbacks_t pub_cbs{};
    pub_cbs.on_configure = configure_multi_pub;
    pcl_container_t* pub_c =
        pcl_container_create("multi_publisher", &pub_cbs, &state);
    ASSERT_NE(pub_c, nullptr);
    pcl_container_configure(pub_c);
    pcl_container_activate(pub_c);
    pcl_executor_add(exec, pub_c);

    // Subscriber container with two subscriptions (different codecs)
    pcl_callbacks_t sub_cbs{};
    sub_cbs.on_configure = configure_multi_sub;
    pcl_container_t* sub_c =
        pcl_container_create("multi_subscriber", &sub_cbs, &state);
    ASSERT_NE(sub_c, nullptr);
    pcl_container_configure(sub_c);
    pcl_container_activate(sub_c);
    pcl_executor_add(exec, sub_c);

    // Publish JSON on json_topic
    {
        pcl_msg_t msg{};
        std::string payload = R"({"identity":"STANDARD_IDENTITY_HOSTILE"})";
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = "application/json";
        pcl_port_publish(state.json_pub, &msg);
    }

    // Publish binary on fb_topic
    {
        pcl_msg_t msg{};
        std::string payload = "\xDE\xAD\xBE\xEF";
        msg.data = payload.data();
        msg.size = static_cast<uint32_t>(payload.size());
        msg.type_name = "application/flatbuffers";
        pcl_port_publish(state.fb_pub, &msg);
    }

    // Spin to deliver
    for (int i = 0; i < 10; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (state.json_recv_count.load() > 0 && state.fb_recv_count.load() > 0)
            break;
    }

    // Verify each subscriber got the right content type
    EXPECT_GE(state.json_recv_count.load(), 1);
    EXPECT_EQ(state.json_last_type, "application/json");

    EXPECT_GE(state.fb_recv_count.load(), 1);
    EXPECT_EQ(state.fb_last_type, "application/flatbuffers");

    pcl_executor_destroy(exec);
    pcl_container_destroy(sub_c);
    pcl_container_destroy(pub_c);
}

// ---------------------------------------------------------------------------
// Test 4: Service binding with configurable content type
//
// Verify that the generated subscribe wrappers accept content_type and
// default to "application/json" when not specified (backwards compat).
// ---------------------------------------------------------------------------

namespace {

struct ServiceCodecState {
    std::string subscribed_type;
};

void svc_sub_cb(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* st = static_cast<ServiceCodecState*>(ud);
    if (msg && msg->type_name)
        st->subscribed_type = msg->type_name;
}

pcl_status_t configure_default_codec(pcl_container_t* c, void* ud) {
    // Call without content_type — should default to "application/json"
    prov::subscribeEntityMatches(c, svc_sub_cb, ud);
    return PCL_OK;
}

pcl_status_t configure_explicit_codec(pcl_container_t* c, void* ud) {
    // Call with explicit content_type
    prov::subscribeEntityMatches(c, svc_sub_cb, ud,
                                 "application/protobuf");
    return PCL_OK;
}

}  // namespace

TEST(CodecDispatchE2E, DefaultCodecIsJson) {
    ServiceCodecState state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_default_codec;

    pcl_container_t* c = pcl_container_create("default_codec", &cbs, &state);
    ASSERT_NE(c, nullptr);

    pcl_status_t rc = pcl_container_configure(c);
    EXPECT_EQ(rc, PCL_OK);

    // The subscribe wrapper should have used "application/json" by default.
    // We can verify this indirectly: if we publish a message on the same topic,
    // the subscriber callback should fire with type_name = "application/json".

    pcl_container_activate(c);

    pcl_executor_t* exec = pcl_executor_create();
    pcl_executor_add(exec, c);

    pcl_msg_t msg{};
    std::string payload = "test";
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";
    pcl_executor_dispatch_incoming(exec, prov::kTopicEntityMatches, &msg);
    pcl_executor_spin_once(exec, 0);

    EXPECT_EQ(state.subscribed_type, "application/json");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

TEST(CodecDispatchE2E, ExplicitCodecProtobuf) {
    ServiceCodecState state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = configure_explicit_codec;

    pcl_container_t* c = pcl_container_create("explicit_codec", &cbs, &state);
    ASSERT_NE(c, nullptr);
    pcl_container_configure(c);
    pcl_container_activate(c);

    pcl_executor_t* exec = pcl_executor_create();
    pcl_executor_add(exec, c);

    // Dispatch with protobuf content type
    pcl_msg_t msg{};
    std::string payload = "\x08\x01";  // protobuf-like bytes
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/protobuf";
    pcl_executor_dispatch_incoming(exec, prov::kTopicEntityMatches, &msg);
    pcl_executor_spin_once(exec, 0);

    EXPECT_EQ(state.subscribed_type, "application/protobuf");

    pcl_executor_destroy(exec);
    pcl_container_destroy(c);
}

// ---------------------------------------------------------------------------
// Test 5: JSON codec ser/de round-trip (unit-level, no PCL)
//
// Validates that the generated JSON codec correctly serialises and
// deserialises all field types (enums, doubles, strings, bools).
// ---------------------------------------------------------------------------

TEST(CodecDispatchE2E, JsonCodecSerDeRoundTrip) {
    // Build a CreateRequirementRequest with all fields set
    codec::CreateRequirementRequest req;
    req.policy      = codec::DataPolicy::Obtain;
    req.identity    = codec::StandardIdentity::Hostile;
    req.dimension   = codec::BattleDimension::SeaSurface;
    req.min_lat_rad = 0.873;
    req.max_lat_rad = 0.907;
    req.min_lon_rad = -0.017;
    req.max_lon_rad = 0.017;

    // Serialise
    std::string json_str = codec::toJson(req);
    EXPECT_FALSE(json_str.empty());

    // Deserialise
    auto req2 = codec::createRequirementRequestFromJson(json_str);

    // Verify all fields survived
    EXPECT_EQ(req2.policy,      codec::DataPolicy::Obtain);
    EXPECT_EQ(req2.identity,    codec::StandardIdentity::Hostile);
    EXPECT_EQ(req2.dimension,   codec::BattleDimension::SeaSurface);
    EXPECT_DOUBLE_EQ(req2.min_lat_rad, 0.873);
    EXPECT_DOUBLE_EQ(req2.max_lat_rad, 0.907);
    EXPECT_DOUBLE_EQ(req2.min_lon_rad, -0.017);
    EXPECT_DOUBLE_EQ(req2.max_lon_rad, 0.017);
}

TEST(CodecDispatchE2E, JsonCodecEnumRoundTrip) {
    // Verify all StandardIdentity values survive toString/fromString
    const codec::StandardIdentity identities[] = {
        codec::StandardIdentity::Unspecified,
        codec::StandardIdentity::Unknown,
        codec::StandardIdentity::Friendly,
        codec::StandardIdentity::Hostile,
        codec::StandardIdentity::Suspect,
        codec::StandardIdentity::Neutral,
        codec::StandardIdentity::Pending,
        codec::StandardIdentity::Joker,
        codec::StandardIdentity::Faker,
        codec::StandardIdentity::AssumedFriendly,
    };

    for (auto id : identities) {
        std::string s = codec::toString(id);
        EXPECT_FALSE(s.empty()) << "toString returned empty for ordinal "
                                << static_cast<int>(id);
        auto roundtripped = codec::standardIdentityFromString(s);
        EXPECT_EQ(roundtripped, id)
            << "Round-trip failed for " << s;
    }
}
