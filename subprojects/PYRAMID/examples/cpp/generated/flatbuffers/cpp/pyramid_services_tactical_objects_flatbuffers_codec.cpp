// Auto-generated tactical service FlatBuffers codec
#include "pyramid_services_tactical_objects_flatbuffers_codec.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pyramid::services::tactical_objects::flatbuffers_codec {

namespace {

constexpr std::uint32_t kMagic = 0x42465750u;  // "PWFB"

enum class MessageTag : std::uint32_t {
    PayloadEnvelope = 0,
    CreateRequirementRequest = 1,
    CreateRequirementResponse = 2,
    EntityMatch = 3,
    EntityMatchArray = 4,
    ObjectEvidence = 5,
    EvidenceRequirement = 6,
};

struct Writer {
    std::string data;

    void raw(const void* ptr, size_t size) {
        data.append(static_cast<const char*>(ptr), size);
    }

    void u8(bool value) {
        const std::uint8_t raw_value = value ? 1u : 0u;
        raw(&raw_value, sizeof(raw_value));
    }

    void u32(std::uint32_t value) { raw(&value, sizeof(value)); }
    void i32(std::int32_t value) { raw(&value, sizeof(value)); }
    void f64(double value) { raw(&value, sizeof(value)); }

    void str(const std::string& value) {
        u32(static_cast<std::uint32_t>(value.size()));
        raw(value.data(), value.size());
    }
};

struct Reader {
    const char* data = nullptr;
    size_t size = 0;
    size_t offset = 0;

    bool raw(void* dst, size_t n) {
        if (offset + n > size) return false;
        std::memcpy(dst, data + offset, n);
        offset += n;
        return true;
    }

    bool u8(bool& value) {
        std::uint8_t raw_value = 0;
        if (!raw(&raw_value, sizeof(raw_value))) return false;
        value = raw_value != 0;
        return true;
    }

    bool u32(std::uint32_t& value) { return raw(&value, sizeof(value)); }
    bool i32(std::int32_t& value) { return raw(&value, sizeof(value)); }
    bool f64(double& value) { return raw(&value, sizeof(value)); }

    bool str(std::string& value) {
        std::uint32_t len = 0;
        if (!u32(len)) return false;
        if (offset + len > size) return false;
        value.assign(data + offset, len);
        offset += len;
        return true;
    }
};

void beginMessage(Writer& w, MessageTag tag) {
    w.u32(kMagic);
    w.u32(static_cast<std::uint32_t>(tag));
}

bool beginMessage(Reader& r, MessageTag expected) {
    std::uint32_t magic = 0;
    std::uint32_t tag = 0;
    return r.u32(magic) && r.u32(tag)
        && magic == kMagic
        && tag == static_cast<std::uint32_t>(expected);
}

void encodeCreateRequirementRequest(Writer& w, const wire_types::CreateRequirementRequest& msg) {
    w.i32(static_cast<std::int32_t>(msg.policy));
    w.i32(static_cast<std::int32_t>(msg.identity));
    w.i32(static_cast<std::int32_t>(msg.dimension));
    w.f64(msg.min_lat_rad);
    w.f64(msg.max_lat_rad);
    w.f64(msg.min_lon_rad);
    w.f64(msg.max_lon_rad);
}

bool decodeCreateRequirementRequest(Reader& r, wire_types::CreateRequirementRequest& msg) {
    std::int32_t tmp_i32 = 0;
    if (!r.i32(tmp_i32)) return false;
        msg.policy = static_cast<pyramid::data_model::DataPolicy>(tmp_i32);
    if (!r.i32(tmp_i32)) return false;
        msg.identity = static_cast<pyramid::data_model::StandardIdentity>(tmp_i32);
    if (!r.i32(tmp_i32)) return false;
        msg.dimension = static_cast<pyramid::data_model::BattleDimension>(tmp_i32);
    if (!r.f64(msg.min_lat_rad)) return false;
    if (!r.f64(msg.max_lat_rad)) return false;
    if (!r.f64(msg.min_lon_rad)) return false;
    if (!r.f64(msg.max_lon_rad)) return false;
    return true;
}

void encodeCreateRequirementResponse(Writer& w, const wire_types::CreateRequirementResponse& msg) {
    w.str(msg.interest_id);
}

bool decodeCreateRequirementResponse(Reader& r, wire_types::CreateRequirementResponse& msg) {
    std::int32_t tmp_i32 = 0;
    if (!r.str(msg.interest_id)) return false;
    return true;
}

void encodeEntityMatch(Writer& w, const wire_types::EntityMatch& msg) {
    w.str(msg.object_id);
    w.i32(static_cast<std::int32_t>(msg.identity));
    w.i32(static_cast<std::int32_t>(msg.dimension));
    w.f64(msg.latitude_rad);
    w.f64(msg.longitude_rad);
    w.f64(msg.confidence);
}

bool decodeEntityMatch(Reader& r, wire_types::EntityMatch& msg) {
    std::int32_t tmp_i32 = 0;
    if (!r.str(msg.object_id)) return false;
    if (!r.i32(tmp_i32)) return false;
        msg.identity = static_cast<pyramid::data_model::StandardIdentity>(tmp_i32);
    if (!r.i32(tmp_i32)) return false;
        msg.dimension = static_cast<pyramid::data_model::BattleDimension>(tmp_i32);
    if (!r.f64(msg.latitude_rad)) return false;
    if (!r.f64(msg.longitude_rad)) return false;
    if (!r.f64(msg.confidence)) return false;
    return true;
}

void encodeEntityMatchArray(Writer& w, const wire_types::EntityMatchArray& msg) {
    w.u32(static_cast<std::uint32_t>(msg.size()));
    for (const auto& item : msg) {
        encodeEntityMatch(w, item);
    }
}

bool decodeEntityMatchArray(Reader& r, wire_types::EntityMatchArray& msg) {
    std::uint32_t count = 0;
    if (!r.u32(count)) return false;
    msg.clear();
    msg.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        wire_types::EntityMatch item{};
        if (!decodeEntityMatch(r, item)) return false;
        msg.push_back(std::move(item));
    }
    return true;
}

void encodeObjectEvidence(Writer& w, const wire_types::ObjectEvidence& msg) {
    w.i32(static_cast<std::int32_t>(msg.identity));
    w.i32(static_cast<std::int32_t>(msg.dimension));
    w.f64(msg.latitude_rad);
    w.f64(msg.longitude_rad);
    w.f64(msg.confidence);
    w.f64(msg.observed_at);
}

bool decodeObjectEvidence(Reader& r, wire_types::ObjectEvidence& msg) {
    std::int32_t tmp_i32 = 0;
    if (!r.i32(tmp_i32)) return false;
        msg.identity = static_cast<pyramid::data_model::StandardIdentity>(tmp_i32);
    if (!r.i32(tmp_i32)) return false;
        msg.dimension = static_cast<pyramid::data_model::BattleDimension>(tmp_i32);
    if (!r.f64(msg.latitude_rad)) return false;
    if (!r.f64(msg.longitude_rad)) return false;
    if (!r.f64(msg.confidence)) return false;
    if (!r.f64(msg.observed_at)) return false;
    return true;
}

void encodeEvidenceRequirement(Writer& w, const wire_types::EvidenceRequirement& msg) {
    w.str(msg.id);
    w.i32(static_cast<std::int32_t>(msg.policy));
    w.i32(static_cast<std::int32_t>(msg.dimension));
    w.f64(msg.min_lat_rad);
    w.f64(msg.max_lat_rad);
    w.f64(msg.min_lon_rad);
    w.f64(msg.max_lon_rad);
}

bool decodeEvidenceRequirement(Reader& r, wire_types::EvidenceRequirement& msg) {
    std::int32_t tmp_i32 = 0;
    if (!r.str(msg.id)) return false;
    if (!r.i32(tmp_i32)) return false;
        msg.policy = static_cast<pyramid::data_model::DataPolicy>(tmp_i32);
    if (!r.i32(tmp_i32)) return false;
        msg.dimension = static_cast<pyramid::data_model::BattleDimension>(tmp_i32);
    if (!r.f64(msg.min_lat_rad)) return false;
    if (!r.f64(msg.max_lat_rad)) return false;
    if (!r.f64(msg.min_lon_rad)) return false;
    if (!r.f64(msg.max_lon_rad)) return false;
    return true;
}

template <typename T>
T finishRead(const void* data, size_t size, MessageTag expected,
             bool (*decode_fn)(Reader&, T&)) {
    Reader r{static_cast<const char*>(data), size, 0};
    if (!beginMessage(r, expected))
        throw std::runtime_error("invalid flatbuffers payload header");
    T msg{};
    if (!decode_fn(r, msg) || r.offset != r.size)
        throw std::runtime_error("invalid flatbuffers payload body");
    return msg;
}

} // namespace

std::string wrapPayload(const std::string& payload) {
    return std::string("PWFB", 4) + payload;
}

std::string unwrapPayload(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        return {};
    }
    const auto* bytes = static_cast<const char*>(data);
    if (size < 4 || std::memcmp(bytes, "PWFB", 4) != 0) {
        throw std::runtime_error("invalid flatbuffers payload envelope");
    }
    return std::string(bytes + 4, size - 4);
}

std::string toBinary(const wire_types::CreateRequirementRequest& msg) {
    Writer w;
    beginMessage(w, MessageTag::CreateRequirementRequest);
    encodeCreateRequirementRequest(w, msg);
    return w.data;
}

wire_types::CreateRequirementRequest fromBinaryCreateRequirementRequest(const void* data, size_t size) {
    return finishRead<wire_types::CreateRequirementRequest>(data, size,
        MessageTag::CreateRequirementRequest, decodeCreateRequirementRequest);
}

std::string toBinary(const wire_types::CreateRequirementResponse& msg) {
    Writer w;
    beginMessage(w, MessageTag::CreateRequirementResponse);
    encodeCreateRequirementResponse(w, msg);
    return w.data;
}

wire_types::CreateRequirementResponse fromBinaryCreateRequirementResponse(const void* data, size_t size) {
    return finishRead<wire_types::CreateRequirementResponse>(data, size,
        MessageTag::CreateRequirementResponse, decodeCreateRequirementResponse);
}

std::string toBinary(const wire_types::EntityMatch& msg) {
    Writer w;
    beginMessage(w, MessageTag::EntityMatch);
    encodeEntityMatch(w, msg);
    return w.data;
}

wire_types::EntityMatch fromBinaryEntityMatch(const void* data, size_t size) {
    return finishRead<wire_types::EntityMatch>(data, size,
        MessageTag::EntityMatch, decodeEntityMatch);
}

std::string toBinary(const wire_types::EntityMatchArray& msg) {
    Writer w;
    beginMessage(w, MessageTag::EntityMatchArray);
    encodeEntityMatchArray(w, msg);
    return w.data;
}

wire_types::EntityMatchArray fromBinaryEntityMatchArray(const void* data, size_t size) {
    return finishRead<wire_types::EntityMatchArray>(data, size,
        MessageTag::EntityMatchArray, decodeEntityMatchArray);
}

std::string toBinary(const wire_types::ObjectEvidence& msg) {
    Writer w;
    beginMessage(w, MessageTag::ObjectEvidence);
    encodeObjectEvidence(w, msg);
    return w.data;
}

wire_types::ObjectEvidence fromBinaryObjectEvidence(const void* data, size_t size) {
    return finishRead<wire_types::ObjectEvidence>(data, size,
        MessageTag::ObjectEvidence, decodeObjectEvidence);
}

std::string toBinary(const wire_types::EvidenceRequirement& msg) {
    Writer w;
    beginMessage(w, MessageTag::EvidenceRequirement);
    encodeEvidenceRequirement(w, msg);
    return w.data;
}

wire_types::EvidenceRequirement fromBinaryEvidenceRequirement(const void* data, size_t size) {
    return finishRead<wire_types::EvidenceRequirement>(data, size,
        MessageTag::EvidenceRequirement, decodeEvidenceRequirement);
}

} // namespace pyramid::services::tactical_objects::flatbuffers_codec
