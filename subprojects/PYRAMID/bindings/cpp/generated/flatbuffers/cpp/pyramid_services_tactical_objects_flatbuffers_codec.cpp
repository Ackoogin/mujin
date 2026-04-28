// Auto-generated service FlatBuffers codec
#include "pyramid_services_tactical_objects_flatbuffers_codec.hpp"

#include "pyramid_data_model_autonomy_codec.hpp"
#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nlohmann/json.hpp>
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

fbs::ObjectDetailT to_fb(const pyramid::domain_model::ObjectDetail& msg) {
    fbs::ObjectDetailT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.source.reserve(msg.source.size());
    for (const auto& item : msg.source) {
        out.source.push_back(static_cast<fbs::ObjectSource>(item));
    }
    out.position = std::make_unique<fbs::GeodeticPositionT>(to_fb(msg.position));
    out.creation_time = msg.creation_time;
    out.has_quality = msg.quality.has_value();
    if (msg.quality.has_value()) {
        out.quality = msg.quality.value();
    }
    out.has_course = msg.course.has_value();
    if (msg.course.has_value()) {
        out.course = msg.course.value();
    }
    out.has_speed = msg.speed.has_value();
    if (msg.speed.has_value()) {
        out.speed = msg.speed.value();
    }
    out.has_length = msg.length.has_value();
    if (msg.length.has_value()) {
        out.length = msg.length.value();
    }
    out.identity = static_cast<fbs::StandardIdentity>(msg.identity);
    out.dimension = static_cast<fbs::BattleDimension>(msg.dimension);
    return out;
}

pyramid::domain_model::ObjectDetail from_fb(const fbs::ObjectDetailT& msg, pyramid::domain_model::ObjectDetail* /*tag*/) {
    pyramid::domain_model::ObjectDetail out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.source.reserve(msg.source.size());
    for (const auto& item : msg.source) {
        out.source.push_back(static_cast<pyramid::domain_model::ObjectSource>(item));
    }
    if (msg.position) out.position = from_fb(*msg.position, static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
    out.creation_time = msg.creation_time;
    if (msg.has_quality) {
        out.quality = msg.quality;
    }
    if (msg.has_course) {
        out.course = msg.course;
    }
    if (msg.has_speed) {
        out.speed = msg.speed;
    }
    if (msg.has_length) {
        out.length = msg.length;
    }
    out.identity = static_cast<pyramid::domain_model::StandardIdentity>(msg.identity);
    out.dimension = static_cast<pyramid::domain_model::BattleDimension>(msg.dimension);
    return out;
}

fbs::ObjectEvidenceRequirementT to_fb(const pyramid::domain_model::ObjectEvidenceRequirement& msg) {
    fbs::ObjectEvidenceRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    out.policy = static_cast<fbs::DataPolicy>(msg.policy);
    out.dimension.reserve(msg.dimension.size());
    for (const auto& item : msg.dimension) {
        out.dimension.push_back(static_cast<fbs::BattleDimension>(item));
    }
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

pyramid::domain_model::ObjectEvidenceRequirement from_fb(const fbs::ObjectEvidenceRequirementT& msg, pyramid::domain_model::ObjectEvidenceRequirement* /*tag*/) {
    pyramid::domain_model::ObjectEvidenceRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::domain_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::domain_model::Achievement*>(nullptr));
    out.policy = static_cast<pyramid::domain_model::DataPolicy>(msg.policy);
    out.dimension.reserve(msg.dimension.size());
    for (const auto& item : msg.dimension) {
        out.dimension.push_back(static_cast<pyramid::domain_model::BattleDimension>(item));
    }
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

fbs::ObjectInterestRequirementT to_fb(const pyramid::domain_model::ObjectInterestRequirement& msg) {
    fbs::ObjectInterestRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    out.has_source = msg.source.has_value();
    if (msg.source.has_value()) {
        out.source = static_cast<fbs::ObjectSource>(msg.source.value());
    }
    out.policy = static_cast<fbs::DataPolicy>(msg.policy);
    out.dimension.reserve(msg.dimension.size());
    for (const auto& item : msg.dimension) {
        out.dimension.push_back(static_cast<fbs::BattleDimension>(item));
    }
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

pyramid::domain_model::ObjectInterestRequirement from_fb(const fbs::ObjectInterestRequirementT& msg, pyramid::domain_model::ObjectInterestRequirement* /*tag*/) {
    pyramid::domain_model::ObjectInterestRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::domain_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::domain_model::Achievement*>(nullptr));
    if (msg.has_source) {
        out.source = static_cast<pyramid::domain_model::ObjectSource>(msg.source);
    }
    out.policy = static_cast<pyramid::domain_model::DataPolicy>(msg.policy);
    out.dimension.reserve(msg.dimension.size());
    for (const auto& item : msg.dimension) {
        out.dimension.push_back(static_cast<pyramid::domain_model::BattleDimension>(item));
    }
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

fbs::ObjectMatchT to_fb(const pyramid::domain_model::ObjectMatch& msg) {
    fbs::ObjectMatchT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.matching_object_id = msg.matching_object_id;
    return out;
}

pyramid::domain_model::ObjectMatch from_fb(const fbs::ObjectMatchT& msg, pyramid::domain_model::ObjectMatch* /*tag*/) {
    pyramid::domain_model::ObjectMatch out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.matching_object_id = msg.matching_object_id;
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

fbs::ObjectDetailArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectDetail>& msg) {
    fbs::ObjectDetailArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectDetailT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectDetail> from_fb(const fbs::ObjectDetailArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectDetail> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectDetail*>(nullptr)));
        }
    }
    return out;
}

fbs::ObjectEvidenceRequirementArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>& msg) {
    fbs::ObjectEvidenceRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectEvidenceRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectEvidenceRequirement> from_fb(const fbs::ObjectEvidenceRequirementArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectEvidenceRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectEvidenceRequirement*>(nullptr)));
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

fbs::ObjectMatchArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectMatch>& msg) {
    fbs::ObjectMatchArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectMatchT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectMatch> from_fb(const fbs::ObjectMatchArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectMatch> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectMatch*>(nullptr)));
        }
    }
    return out;
}

fbs::ObjectInterestRequirementArrayHolderT to_fb(const std::vector<pyramid::domain_model::ObjectInterestRequirement>& msg) {
    fbs::ObjectInterestRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ObjectInterestRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::domain_model::ObjectInterestRequirement> from_fb(const fbs::ObjectInterestRequirementArrayHolderT& msg) {
    std::vector<pyramid::domain_model::ObjectInterestRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::domain_model::ObjectInterestRequirement*>(nullptr)));
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

std::string toBinary(const pyramid::domain_model::ObjectDetail& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectDetail::Pack(builder, &object));
}

pyramid::domain_model::ObjectDetail fromBinaryObjectDetail(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectDetail>(data, size, "ObjectDetail");
    fbs::ObjectDetailT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectDetail*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::ObjectEvidenceRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectEvidenceRequirement::Pack(builder, &object));
}

pyramid::domain_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectEvidenceRequirement>(data, size, "ObjectEvidenceRequirement");
    fbs::ObjectEvidenceRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectEvidenceRequirement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::ObjectInterestRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectInterestRequirement::Pack(builder, &object));
}

pyramid::domain_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectInterestRequirement>(data, size, "ObjectInterestRequirement");
    fbs::ObjectInterestRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectInterestRequirement*>(nullptr));
}

std::string toBinary(const pyramid::domain_model::ObjectMatch& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectMatch::Pack(builder, &object));
}

pyramid::domain_model::ObjectMatch fromBinaryObjectMatch(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectMatch>(data, size, "ObjectMatch");
    fbs::ObjectMatchT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::domain_model::ObjectMatch*>(nullptr));
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

std::string toBinary(const std::vector<pyramid::domain_model::ObjectDetail>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectDetailArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectDetail> fromBinaryObjectDetailArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectDetailArrayHolder>(data, size, "ObjectDetailArray");
    fbs::ObjectDetailArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectEvidenceRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectEvidenceRequirement> fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectEvidenceRequirementArrayHolder>(data, size, "ObjectEvidenceRequirementArray");
    fbs::ObjectEvidenceRequirementArrayHolderT object{};
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

std::string toBinary(const std::vector<pyramid::domain_model::ObjectMatch>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectMatchArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectMatch> fromBinaryObjectMatchArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectMatchArrayHolder>(data, size, "ObjectMatchArray");
    fbs::ObjectMatchArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectInterestRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ObjectInterestRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::domain_model::ObjectInterestRequirement> fromBinaryObjectInterestRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ObjectInterestRequirementArrayHolder>(data, size, "ObjectInterestRequirementArray");
    fbs::ObjectInterestRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

} // namespace pyramid::services::tactical_objects::flatbuffers_codec

extern "C" {

void pyramid_services_tactical_objects_free_buffer(void* data) {
    std::free(data);
}

void* pyramid_services_tactical_objects_GeodeticPosition_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::GeodeticPosition*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_GeodeticPosition_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryGeodeticPosition(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_PolyArea_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::PolyArea*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_PolyArea_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryPolyArea(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Achievement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Achievement*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Achievement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryAchievement(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Entity_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Entity*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Entity_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryEntity(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_CircleArea_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::CircleArea*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_CircleArea_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryCircleArea(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Point_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Point*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Point_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryPoint(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Contraint_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Contraint*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Contraint_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryContraint(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Ack_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Ack*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Ack_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryAck(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Query_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Query*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Query_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryQuery(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_ObjectDetail_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::tactical::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectDetail*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_ObjectDetail_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectDetail(data, size);
        auto json = pyramid::domain_model::tactical::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::tactical::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectEvidenceRequirement*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectEvidenceRequirement(data, size);
        auto json = pyramid::domain_model::tactical::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_ObjectInterestRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::tactical::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectInterestRequirement*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_ObjectInterestRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectInterestRequirement(data, size);
        auto json = pyramid::domain_model::tactical::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_ObjectMatch_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::tactical::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::ObjectMatch*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_ObjectMatch_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectMatch(data, size);
        auto json = pyramid::domain_model::tactical::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Capability_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::domain_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::domain_model::Capability*>(nullptr));
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Capability_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryCapability(data, size);
        auto json = pyramid::domain_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_Identifier_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = [&]() { auto j = nlohmann::json::parse(std::string(json ? json : "")); return j.is_string() ? j.get<pyramid::domain_model::Identifier>() : pyramid::domain_model::Identifier{}; }();
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(value);
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

char* pyramid_services_tactical_objects_Identifier_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryIdentifier(data, size);
        auto json = nlohmann::json(value).dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_tactical_objects_ObjectDetailArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectDetail> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::tactical::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectDetail*>(nullptr)));
            }
        }
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(values);
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

char* pyramid_services_tactical_objects_ObjectDetailArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectDetailArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::tactical::toJson(item)));
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

void* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectEvidenceRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::tactical::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectEvidenceRequirement*>(nullptr)));
            }
        }
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(values);
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

char* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectEvidenceRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::tactical::toJson(item)));
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

void* pyramid_services_tactical_objects_CapabilityArray_to_flatbuffer_json(const char* json, size_t* size_out) {
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
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(values);
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

char* pyramid_services_tactical_objects_CapabilityArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryCapabilityArray(data, size);
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

void* pyramid_services_tactical_objects_ObjectMatchArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectMatch> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::tactical::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectMatch*>(nullptr)));
            }
        }
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(values);
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

char* pyramid_services_tactical_objects_ObjectMatchArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectMatchArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::tactical::toJson(item)));
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

void* pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::domain_model::ObjectInterestRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::domain_model::tactical::fromJson(item.dump(), static_cast<pyramid::domain_model::ObjectInterestRequirement*>(nullptr)));
            }
        }
        auto payload = pyramid::services::tactical_objects::flatbuffers_codec::toBinary(values);
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

char* pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::tactical_objects::flatbuffers_codec::fromBinaryObjectInterestRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::domain_model::tactical::toJson(item)));
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
