// Auto-generated tactical service FlatBuffers codec
#include "pyramid_services_tactical_objects_flatbuffers_codec.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace pyramid::services::tactical_objects::flatbuffers_codec {

namespace fbs = pyramid::services::tactical_objects;

namespace {

template <typename OffsetT>
std::string finish_buffer(flatbuffers::FlatBufferBuilder& builder, OffsetT root) {
    builder.Finish(root);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

fbs::CreateRequirementRequestT to_fb(const wire_types::CreateRequirementRequest& msg) {
    fbs::CreateRequirementRequestT out{};
    out.policy = static_cast<fbs::DataPolicy>(msg.policy);
    out.identity = static_cast<fbs::StandardIdentity>(msg.identity);
    out.dimension = static_cast<fbs::BattleDimension>(msg.dimension);
    out.min_lat_rad = msg.min_lat_rad;
    out.max_lat_rad = msg.max_lat_rad;
    out.min_lon_rad = msg.min_lon_rad;
    out.max_lon_rad = msg.max_lon_rad;
    return out;
}

wire_types::CreateRequirementRequest from_fb(const fbs::CreateRequirementRequestT& msg) {
    wire_types::CreateRequirementRequest out{};
    out.policy = static_cast<pyramid::data_model::DataPolicy>(msg.policy);
    out.identity = static_cast<pyramid::data_model::StandardIdentity>(msg.identity);
    out.dimension = static_cast<pyramid::data_model::BattleDimension>(msg.dimension);
    out.min_lat_rad = msg.min_lat_rad;
    out.max_lat_rad = msg.max_lat_rad;
    out.min_lon_rad = msg.min_lon_rad;
    out.max_lon_rad = msg.max_lon_rad;
    return out;
}

fbs::CreateRequirementResponseT to_fb(const wire_types::CreateRequirementResponse& msg) {
    fbs::CreateRequirementResponseT out{};
    out.interest_id = msg.interest_id;
    return out;
}

wire_types::CreateRequirementResponse from_fb(const fbs::CreateRequirementResponseT& msg) {
    wire_types::CreateRequirementResponse out{};
    out.interest_id = msg.interest_id;
    return out;
}

fbs::EntityMatchT to_fb(const wire_types::EntityMatch& msg) {
    fbs::EntityMatchT out{};
    out.object_id = msg.object_id;
    out.identity = static_cast<fbs::StandardIdentity>(msg.identity);
    out.dimension = static_cast<fbs::BattleDimension>(msg.dimension);
    out.latitude_rad = msg.latitude_rad;
    out.longitude_rad = msg.longitude_rad;
    out.confidence = msg.confidence;
    return out;
}

wire_types::EntityMatch from_fb(const fbs::EntityMatchT& msg) {
    wire_types::EntityMatch out{};
    out.object_id = msg.object_id;
    out.identity = static_cast<pyramid::data_model::StandardIdentity>(msg.identity);
    out.dimension = static_cast<pyramid::data_model::BattleDimension>(msg.dimension);
    out.latitude_rad = msg.latitude_rad;
    out.longitude_rad = msg.longitude_rad;
    out.confidence = msg.confidence;
    return out;
}

fbs::EntityMatchArrayHolderT to_fb(const wire_types::EntityMatchArray& msg) {
    fbs::EntityMatchArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::EntityMatchT>(to_fb(item)));
    }
    return out;
}

wire_types::EntityMatchArray from_fb(const fbs::EntityMatchArrayHolderT& msg) {
    wire_types::EntityMatchArray out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item));
        }
    }
    return out;
}

fbs::ObjectEvidenceT to_fb(const wire_types::ObjectEvidence& msg) {
    fbs::ObjectEvidenceT out{};
    out.identity = static_cast<fbs::StandardIdentity>(msg.identity);
    out.dimension = static_cast<fbs::BattleDimension>(msg.dimension);
    out.latitude_rad = msg.latitude_rad;
    out.longitude_rad = msg.longitude_rad;
    out.confidence = msg.confidence;
    out.observed_at = msg.observed_at;
    return out;
}

wire_types::ObjectEvidence from_fb(const fbs::ObjectEvidenceT& msg) {
    wire_types::ObjectEvidence out{};
    out.identity = static_cast<pyramid::data_model::StandardIdentity>(msg.identity);
    out.dimension = static_cast<pyramid::data_model::BattleDimension>(msg.dimension);
    out.latitude_rad = msg.latitude_rad;
    out.longitude_rad = msg.longitude_rad;
    out.confidence = msg.confidence;
    out.observed_at = msg.observed_at;
    return out;
}

fbs::EvidenceRequirementT to_fb(const wire_types::EvidenceRequirement& msg) {
    fbs::EvidenceRequirementT out{};
    out.id = msg.id;
    out.policy = static_cast<fbs::DataPolicy>(msg.policy);
    out.dimension = static_cast<fbs::BattleDimension>(msg.dimension);
    out.min_lat_rad = msg.min_lat_rad;
    out.max_lat_rad = msg.max_lat_rad;
    out.min_lon_rad = msg.min_lon_rad;
    out.max_lon_rad = msg.max_lon_rad;
    return out;
}

wire_types::EvidenceRequirement from_fb(const fbs::EvidenceRequirementT& msg) {
    wire_types::EvidenceRequirement out{};
    out.id = msg.id;
    out.policy = static_cast<pyramid::data_model::DataPolicy>(msg.policy);
    out.dimension = static_cast<pyramid::data_model::BattleDimension>(msg.dimension);
    out.min_lat_rad = msg.min_lat_rad;
    out.max_lat_rad = msg.max_lat_rad;
    out.min_lon_rad = msg.min_lon_rad;
    out.max_lon_rad = msg.max_lon_rad;
    return out;
}

} // namespace

std::string wrapPayload(const std::string& payload) {
    flatbuffers::FlatBufferBuilder builder(
        static_cast<uint32_t>(payload.size() + 64u));
    auto payload_offset = builder.CreateString(payload);
    fbs::JsonPayloadBuilder root(builder);
    root.add_payload(payload_offset);
    return finish_buffer(builder, root.Finish());
}

std::string unwrapPayload(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        return {};
    }
    auto* root = flatbuffers::GetRoot<fbs::JsonPayload>(data);
    if (!root || !root->payload()) {
        throw std::runtime_error("invalid JsonPayload flatbuffer");
    }
    return root->payload()->str();
}

std::string toBinary(const wire_types::CreateRequirementRequest& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CreateRequirementRequest::Pack(builder, &object));
}

wire_types::CreateRequirementRequest fromBinaryCreateRequirementRequest(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty CreateRequirementRequest flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::CreateRequirementRequest>(data);
    if (!root) {
        throw std::runtime_error("invalid CreateRequirementRequest flatbuffer");
    }
    fbs::CreateRequirementRequestT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const wire_types::CreateRequirementResponse& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CreateRequirementResponse::Pack(builder, &object));
}

wire_types::CreateRequirementResponse fromBinaryCreateRequirementResponse(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty CreateRequirementResponse flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::CreateRequirementResponse>(data);
    if (!root) {
        throw std::runtime_error("invalid CreateRequirementResponse flatbuffer");
    }
    fbs::CreateRequirementResponseT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const wire_types::EntityMatch& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::EntityMatch::Pack(builder, &object));
}

wire_types::EntityMatch fromBinaryEntityMatch(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty EntityMatch flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::EntityMatch>(data);
    if (!root) {
        throw std::runtime_error("invalid EntityMatch flatbuffer");
    }
    fbs::EntityMatchT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const wire_types::EntityMatchArray& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::EntityMatchArrayHolder::Pack(builder, &object));
}

wire_types::EntityMatchArray fromBinaryEntityMatchArray(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty EntityMatchArray flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::EntityMatchArrayHolder>(data);
    if (!root) {
        throw std::runtime_error("invalid EntityMatchArray flatbuffer");
    }
    fbs::EntityMatchArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const wire_types::ObjectEvidence& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectEvidence::Pack(builder, &object));
}

wire_types::ObjectEvidence fromBinaryObjectEvidence(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty ObjectEvidence flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::ObjectEvidence>(data);
    if (!root) {
        throw std::runtime_error("invalid ObjectEvidence flatbuffer");
    }
    fbs::ObjectEvidenceT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const wire_types::EvidenceRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::EvidenceRequirement::Pack(builder, &object));
}

wire_types::EvidenceRequirement fromBinaryEvidenceRequirement(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error("empty EvidenceRequirement flatbuffer");
    }
    auto* root = flatbuffers::GetRoot<fbs::EvidenceRequirement>(data);
    if (!root) {
        throw std::runtime_error("invalid EvidenceRequirement flatbuffer");
    }
    fbs::EvidenceRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

} // namespace pyramid::services::tactical_objects::flatbuffers_codec
