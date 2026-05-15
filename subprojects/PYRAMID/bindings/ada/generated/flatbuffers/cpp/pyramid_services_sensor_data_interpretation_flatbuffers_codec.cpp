// Auto-generated service FlatBuffers codec
#include "pyramid_services_sensor_data_interpretation_flatbuffers_codec.hpp"

#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_sensors_codec.hpp"
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <utility>

namespace pyramid::services::sensor_data_interpretation::flatbuffers_codec {

namespace fbs = pyramid::services::sensor_data_interpretation;

namespace {

template <typename OffsetT>
std::string finish_buffer(flatbuffers::FlatBufferBuilder& builder, OffsetT root) {
    builder.Finish(root);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

template <typename TableT>
const TableT* verified_root(const void* data, size_t size, const char* type_name) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error(std::string("empty ") + type_name + " flatbuffer");
    }
    auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
    flatbuffers::Verifier verifier(bytes, size);
    if (!verifier.VerifyBuffer<TableT>()) {
        throw std::runtime_error(std::string("invalid ") + type_name + " flatbuffer");
    }
    return flatbuffers::GetRoot<TableT>(bytes);
}

fbs::GeodeticPositionT to_fb(const pyramid::domain_model::GeodeticPosition& msg) {
    fbs::GeodeticPositionT out{};
    out.latitude = msg.latitude;
    out.longitude = msg.longitude;
    return out;
}

pyramid::domain_model::GeodeticPosition from_fb(const fbs::GeodeticPositionT& msg, pyramid::domain_model::GeodeticPosition* /*tag*/) {
    pyramid::domain_model::GeodeticPosition out{};
    out.latitude = msg.latitude;
    out.longitude = msg.longitude;
    return out;
}

fbs::PolyAreaT to_fb(const pyramid::domain_model::PolyArea& msg) {
    fbs::PolyAreaT out{};
    out.points.reserve(msg.points.size());
    for (const auto& item : msg.points) {
        out.points.emplace_back(std::make_unique<fbs::GeodeticPositionT>(to_fb(item)));
    }
    return out;
}

pyramid::domain_model::PolyArea from_fb(const fbs::PolyAreaT& msg, pyramid::domain_model::PolyArea* /*tag*/) {
    pyramid::domain_model::PolyArea out{};
    out.points.reserve(msg.points.size());
    for (const auto& item : msg.points) {
        if (item) {
            out.points.push_back(from_fb(*item, static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr)));
        }
    }
    return out;
}

fbs::AchievementT to_fb(const pyramid::domain_model::Achievement& msg) {
    fbs::AchievementT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.status = static_cast<fbs::Progress>(msg.status);
    out.has_quality = msg.quality.has_value();
    if (msg.quality.has_value()) {
        out.quality = msg.quality.value();
    }
    out.achieveability = static_cast<fbs::Feasibility>(msg.achieveability);
    return out;
}

pyramid::domain_model::Achievement from_fb(const fbs::AchievementT& msg, pyramid::domain_model::Achievement* /*tag*/) {
    pyramid::domain_model::Achievement out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.status = static_cast<pyramid::domain_model::Progress>(msg.status);
    if (msg.has_quality) {
        out.quality = msg.quality;
    }
    out.achieveability = static_cast<pyramid::domain_model::Feasibility>(msg.achieveability);
    return out;
}

fbs::EntityT to_fb(const pyramid::domain_model::Entity& msg) {
    fbs::EntityT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    return out;
}

pyramid::domain_model::Entity from_fb(const fbs::EntityT& msg, pyramid::domain_model::Entity* /*tag*/) {
    pyramid::domain_model::Entity out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    return out;
}

fbs::CircleAreaT to_fb(const pyramid::domain_model::CircleArea& msg) {
    fbs::CircleAreaT out{};
    out.position = std::make_unique<fbs::GeodeticPositionT>(to_fb(msg.position));
    out.radius = msg.radius;
    return out;
}

pyramid::domain_model::CircleArea from_fb(const fbs::CircleAreaT& msg, pyramid::domain_model::CircleArea* /*tag*/) {
    pyramid::domain_model::CircleArea out{};
    if (msg.position) out.position = from_fb(*msg.position, static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
    out.radius = msg.radius;
    return out;
}

fbs::PointT to_fb(const pyramid::domain_model::Point& msg) {
    fbs::PointT out{};
    out.position = std::make_unique<fbs::GeodeticPositionT>(to_fb(msg.position));
    return out;
}

pyramid::domain_model::Point from_fb(const fbs::PointT& msg, pyramid::domain_model::Point* /*tag*/) {
    pyramid::domain_model::Point out{};
    if (msg.position) out.position = from_fb(*msg.position, static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
    return out;
}

fbs::ContraintT to_fb(const pyramid::domain_model::Contraint& msg) {
    fbs::ContraintT out{};
    out.name = msg.name;
    out.value = msg.value;
    return out;
}

pyramid::domain_model::Contraint from_fb(const fbs::ContraintT& msg, pyramid::domain_model::Contraint* /*tag*/) {
    pyramid::domain_model::Contraint out{};
    out.name = msg.name;
    out.value = msg.value;
    return out;
}

fbs::AckT to_fb(const pyramid::domain_model::Ack& msg) {
    fbs::AckT out{};
    out.success = msg.success;
    return out;
}

pyramid::domain_model::Ack from_fb(const fbs::AckT& msg, pyramid::domain_model::Ack* /*tag*/) {
    pyramid::domain_model::Ack out{};
    out.success = msg.success;
    return out;
}

fbs::QueryT to_fb(const pyramid::domain_model::Query& msg) {
    fbs::QueryT out{};
    out.id = msg.id;
    out.has_one_shot = msg.one_shot.has_value();
    if (msg.one_shot.has_value()) {
        out.one_shot = msg.one_shot.value();
    }
    return out;
}

pyramid::domain_model::Query from_fb(const fbs::QueryT& msg, pyramid::domain_model::Query* /*tag*/) {
    pyramid::domain_model::Query out{};
    out.id = msg.id;
    if (msg.has_one_shot) {
        out.one_shot = msg.one_shot;
    }
    return out;
}

fbs::InterpretationRequirementT to_fb(const pyramid::domain_model::InterpretationRequirement& msg) {
    fbs::InterpretationRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    out.policy = static_cast<fbs::InterpretationPolicy>(msg.policy);
    out.type = static_cast<fbs::InterpretationType>(msg.type);
    if (msg.poly_area.has_value()) {
        out.poly_area = std::make_unique<fbs::PolyAreaT>(to_fb(msg.poly_area.value()));
    }
    if (msg.circle_area.has_value()) {
        out.circle_area = std::make_unique<fbs::CircleAreaT>(to_fb(msg.circle_area.value()));
    }
    if (msg.point.has_value()) {
        out.point = std::make_unique<fbs::PointT>(to_fb(msg.point.value()));
    }
    return out;
}

pyramid::domain_model::InterpretationRequirement from_fb(const fbs::InterpretationRequirementT& msg, pyramid::domain_model::InterpretationRequirement* /*tag*/) {
    pyramid::domain_model::InterpretationRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::domain_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::domain_model::Achievement*>(nullptr));
    out.policy = static_cast<pyramid::domain_model::InterpretationPolicy>(msg.policy);
    out.type = static_cast<pyramid::domain_model::InterpretationType>(msg.type);
    if (msg.poly_area) {
        out.poly_area = from_fb(*msg.poly_area, static_cast<pyramid::domain_model::PolyArea*>(nullptr));
    }
    if (msg.circle_area) {
        out.circle_area = from_fb(*msg.circle_area, static_cast<pyramid::domain_model::CircleArea*>(nullptr));
    }
    if (msg.point) {
        out.point = from_fb(*msg.point, static_cast<pyramid::domain_model::Point*>(nullptr));
    }
    return out;
}

fbs::ObjectEvidenceProvisionRequirementT to_fb(const pyramid::domain_model::ObjectEvidenceProvisionRequirement& msg) {
    fbs::ObjectEvidenceProvisionRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    if (msg.poly_area.has_value()) {
        out.poly_area = std::make_unique<fbs::PolyAreaT>(to_fb(msg.poly_area.value()));
    }
    if (msg.circle_area.has_value()) {
        out.circle_area = std::make_unique<fbs::CircleAreaT>(to_fb(msg.circle_area.value()));
    }
    if (msg.point.has_value()) {
        out.point = std::make_unique<fbs::PointT>(to_fb(msg.point.value()));
    }
    return out;
}

pyramid::domain_model::ObjectEvidenceProvisionRequirement from_fb(const fbs::ObjectEvidenceProvisionRequirementT& msg, pyramid::domain_model::ObjectEvidenceProvisionRequirement* /*tag*/) {
    pyramid::domain_model::ObjectEvidenceProvisionRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::domain_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::domain_model::Achievement*>(nullptr));
    if (msg.poly_area) {
        out.poly_area = from_fb(*msg.poly_area, static_cast<pyramid::domain_model::PolyArea*>(nullptr));
    }
    if (msg.circle_area) {
        out.circle_area = from_fb(*msg.circle_area, static_cast<pyramid::domain_model::CircleArea*>(nullptr));
    }
    if (msg.point) {
        out.point = from_fb(*msg.point, static_cast<pyramid::domain_model::Point*>(nullptr));
    }
    return out;
}

fbs::ObjectAquisitionRequirementT to_fb(const pyramid::domain_model::ObjectAquisitionRequirement& msg) {
    fbs::ObjectAquisitionRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    if (msg.poly_area.has_value()) {
        out.poly_area = std::make_unique<fbs::PolyAreaT>(to_fb(msg.poly_area.value()));
    }
    if (msg.circle_area.has_value()) {
        out.circle_area = std::make_unique<fbs::CircleAreaT>(to_fb(msg.circle_area.value()));
    }
    if (msg.point.has_value()) {
        out.point = std::make_unique<fbs::PointT>(to_fb(msg.point.value()));
    }
    return out;
}

pyramid::domain_model::ObjectAquisitionRequirement from_fb(const fbs::ObjectAquisitionRequirementT& msg, pyramid::domain_model::ObjectAquisitionRequirement* /*tag*/) {
    pyramid::domain_model::ObjectAquisitionRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::domain_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::domain_model::Achievement*>(nullptr));
    if (msg.poly_area) {
        out.poly_area = from_fb(*msg.poly_area, static_cast<pyramid::domain_model::PolyArea*>(nullptr));
    }
    if (msg.circle_area) {
        out.circle_area = from_fb(*msg.circle_area, static_cast<pyramid::domain_model::CircleArea*>(nullptr));
    }
    if (msg.point) {
        out.point = from_fb(*msg.point, static_cast<pyramid::domain_model::Point*>(nullptr));
    }
    return out;
}

fbs::CapabilityT to_fb(const pyramid::domain_model::Capability& msg) {
    fbs::CapabilityT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.availability = msg.availability;
    out.name = msg.name;
    out.contraint.reserve(msg.contraint.size());
    for (const auto& item : msg.contraint) {
        out.contraint.emplace_back(std::make_unique<fbs::ContraintT>(to_fb(item)));
    }
    return out;
}

pyramid::domain_model::Capability from_fb(const fbs::CapabilityT& msg, pyramid::domain_model::Capability* /*tag*/) {
    pyramid::domain_model::Capability out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.availability = msg.availability;
    out.name = msg.name;
    out.contraint.reserve(msg.contraint.size());
    for (const auto& item : msg.contraint) {
        if (item) {
            out.contraint.push_back(from_fb(*item, static_cast<pyramid::domain_model::Contraint*>(nullptr)));
        }
    }
    return out;
}

fbs::IdentifierValueT to_fb(const pyramid::domain_model::Identifier& msg) {
    fbs::IdentifierValueT out{};
    out.value = msg;
    return out;
}

pyramid::domain_model::Identifier from_fb(const fbs::IdentifierValueT& msg, pyramid::domain_model::Identifier* /*tag*/) {
    return msg.value;
}

fbs::ObjectEvidenceProvisionRequirementArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement>& msg) {
    fbs::ObjectEvidenceProvisionRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectEvidenceProvisionRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> from_fb(const fbs::ObjectEvidenceProvisionRequirementArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectEvidenceProvisionRequirement*>(nullptr)));
        }
    }
    return out;
}

fbs::ObjectAquisitionRequirementArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectAquisitionRequirement>& msg) {
    fbs::ObjectAquisitionRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectAquisitionRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectAquisitionRequirement> from_fb(const fbs::ObjectAquisitionRequirementArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectAquisitionRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectAquisitionRequirement*>(nullptr)));
        }
    }
    return out;
}

fbs::CapabilityArrayHolderT to_fb(const std::vector<pyramid::domain_model::Capability>& msg) {
    fbs::CapabilityArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::CapabilityT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::Capability> from_fb(const fbs::CapabilityArrayHolderT& msg) {
    std::vector<pyramid::domain_model::Capability> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::Capability*>(nullptr)));
        }
    }
    return out;
}

fbs::InterpretationRequirementArrayHolderT to_fb(const std::vector<pyramid::domain_model::InterpretationRequirement>& msg) {
    fbs::InterpretationRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::InterpretationRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::InterpretationRequirement> from_fb(const fbs::InterpretationRequirementArrayHolderT& msg) {
    std::vector<pyramid::domain_model::InterpretationRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::InterpretationRequirement*>(nullptr)));
        }
    }
    return out;
}

} // namespace

std::string toBinary(const pyramid::domain_model::GeodeticPosition& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::GeodeticPosition::Pack(builder, &object));
}

pyramid::domain_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data, size_t size) {
    auto* root = verified_root<fbs::GeodeticPosition>(data, size, "GeodeticPosition");
    fbs::GeodeticPositionT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::PolyArea& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PolyArea::Pack(builder, &object));
}

pyramid::domain_model::PolyArea fromBinaryPolyArea(const void* data, size_t size) {
    auto* root = verified_root<fbs::PolyArea>(data, size, "PolyArea");
    fbs::PolyAreaT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::PolyArea*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Achievement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Achievement::Pack(builder, &object));
}

pyramid::domain_model::Achievement fromBinaryAchievement(const void* data, size_t size) {
    auto* root = verified_root<fbs::Achievement>(data, size, "Achievement");
    fbs::AchievementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Achievement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Entity& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Entity::Pack(builder, &object));
}

pyramid::domain_model::Entity fromBinaryEntity(const void* data, size_t size) {
    auto* root = verified_root<fbs::Entity>(data, size, "Entity");
    fbs::EntityT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Entity*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::CircleArea& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CircleArea::Pack(builder, &object));
}

pyramid::domain_model::CircleArea fromBinaryCircleArea(const void* data, size_t size) {
    auto* root = verified_root<fbs::CircleArea>(data, size, "CircleArea");
    fbs::CircleAreaT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::CircleArea*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Point& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Point::Pack(builder, &object));
}

pyramid::domain_model::Point fromBinaryPoint(const void* data, size_t size) {
    auto* root = verified_root<fbs::Point>(data, size, "Point");
    fbs::PointT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Point*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Contraint& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Contraint::Pack(builder, &object));
}

pyramid::domain_model::Contraint fromBinaryContraint(const void* data, size_t size) {
    auto* root = verified_root<fbs::Contraint>(data, size, "Contraint");
    fbs::ContraintT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Contraint*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Ack& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Ack::Pack(builder, &object));
}

pyramid::domain_model::Ack fromBinaryAck(const void* data, size_t size) {
    auto* root = verified_root<fbs::Ack>(data, size, "Ack");
    fbs::AckT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Ack*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Query& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Query::Pack(builder, &object));
}

pyramid::domain_model::Query fromBinaryQuery(const void* data, size_t size) {
    auto* root = verified_root<fbs::Query>(data, size, "Query");
    fbs::QueryT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Query*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::InterpretationRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::InterpretationRequirement::Pack(builder, &object));
}

pyramid::domain_model::InterpretationRequirement fromBinaryInterpretationRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::InterpretationRequirement>(data, size, "InterpretationRequirement");
    fbs::InterpretationRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::InterpretationRequirement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::ObjectEvidenceProvisionRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectEvidenceProvisionRequirement::Pack(builder, &object));
}

pyramid::domain_model::ObjectEvidenceProvisionRequirement fromBinaryObjectEvidenceProvisionRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectEvidenceProvisionRequirement>(data, size, "ObjectEvidenceProvisionRequirement");
    fbs::ObjectEvidenceProvisionRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectEvidenceProvisionRequirement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::ObjectAquisitionRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectAquisitionRequirement::Pack(builder, &object));
}

pyramid::domain_model::ObjectAquisitionRequirement fromBinaryObjectAquisitionRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectAquisitionRequirement>(data, size, "ObjectAquisitionRequirement");
    fbs::ObjectAquisitionRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectAquisitionRequirement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Capability& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Capability::Pack(builder, &object));
}

pyramid::domain_model::Capability fromBinaryCapability(const void* data, size_t size) {
    auto* root = verified_root<fbs::Capability>(data, size, "Capability");
    fbs::CapabilityT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Capability*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::Identifier& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::IdentifierValue::Pack(builder, &object));
}

pyramid::domain_model::Identifier fromBinaryIdentifier(const void* data, size_t size) {
    auto* root = verified_root<fbs::IdentifierValue>(data, size, "Identifier");
    fbs::IdentifierValueT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::Identifier*>(nullptr));
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectEvidenceProvisionRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> fromBinaryObjectEvidenceProvisionRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectEvidenceProvisionRequirementArrayHolder>(data, size, "ObjectEvidenceProvisionRequirementArray");
    fbs::ObjectEvidenceProvisionRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectAquisitionRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectAquisitionRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectAquisitionRequirement> fromBinaryObjectAquisitionRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectAquisitionRequirementArrayHolder>(data, size, "ObjectAquisitionRequirementArray");
    fbs::ObjectAquisitionRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::domain_model::Capability>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CapabilityArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::CapabilityArrayHolder>(data, size, "CapabilityArray");
    fbs::CapabilityArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::domain_model::InterpretationRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::InterpretationRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::InterpretationRequirement> fromBinaryInterpretationRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::InterpretationRequirementArrayHolder>(data, size, "InterpretationRequirementArray");
    fbs::InterpretationRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

} // namespace pyramid::services::sensor_data_interpretation::flatbuffers_codec

extern "C" {

void pyramid_services_sensor_data_interpretation_free_buffer(void* data) {
    std::free(data);
}

void* pyramid_services_sensor_data_interpretation_GeodeticPosition_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_GeodeticPosition_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryGeodeticPosition(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_PolyArea_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::PolyArea*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_PolyArea_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryPolyArea(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Achievement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Achievement*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Achievement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryAchievement(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Entity_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Entity*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Entity_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryEntity(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_CircleArea_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::CircleArea*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_CircleArea_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryCircleArea(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Point_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Point*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Point_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryPoint(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Contraint_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Contraint*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Contraint_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryContraint(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Ack_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Ack*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Ack_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryAck(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Query_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Query*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Query_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryQuery(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_InterpretationRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::sensors::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::InterpretationRequirement*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_InterpretationRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryInterpretationRequirement(data, size);
        auto json = pyramid::domain_model::sensors::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::sensors::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectEvidenceProvisionRequirement*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirement(data, size);
        auto json = pyramid::domain_model::sensors::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::sensors::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectAquisitionRequirement*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryObjectAquisitionRequirement(data, size);
        auto json = pyramid::domain_model::sensors::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Capability_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Capability*>(nullptr));
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Capability_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryCapability(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_Identifier_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = nlohmann::json::parse(std::string(json ? json : "")).get<pyramid::domain_model::Identifier>();
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_Identifier_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryIdentifier(data, size);
        auto json = nlohmann::json(value).dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::sensors::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectEvidenceProvisionRequirement*>(nullptr)));
            }
        }
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::sensors::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectAquisitionRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::sensors::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectAquisitionRequirement*>(nullptr)));
            }
        }
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryObjectAquisitionRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::sensors::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_CapabilityArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::Capability> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::common::fromJson(item.dump(), static_cast<pyramid::domain_model::Capability*>(nullptr)));
            }
        }
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_CapabilityArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryCapabilityArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_sensor_data_interpretation_InterpretationRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::InterpretationRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::sensors::fromJson(item.dump(), static_cast<pyramid::domain_model::InterpretationRequirement*>(nullptr)));
            }
        }
        auto payload = pyramid::services::sensor_data_interpretation::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_sensor_data_interpretation_InterpretationRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::sensor_data_interpretation::flatbuffers_codec::fromBinaryInterpretationRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::sensors::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

} // extern "C"
