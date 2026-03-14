#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectComponents.h>
#include <store/ObjectStore.h>
#include <tl/optional.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace tactical_objects {

/// \brief Wire format message type bytes.
static const uint8_t STREAM_MSG_ENTITY_UPDATE = 0x01;
static const uint8_t STREAM_MSG_ENTITY_DELETE = 0x02;

/// \brief Bit positions in the field_mask for entity update frames.
namespace FieldMaskBit {
  static const uint16_t POSITION        = (1u << 0);  ///< KinematicsComponent::position
  static const uint16_t VELOCITY        = (1u << 1);  ///< KinematicsComponent::velocity
  static const uint16_t AFFILIATION     = (1u << 2);  ///< MilClassComponent::profile::affiliation
  static const uint16_t OBJECT_TYPE     = (1u << 3);  ///< EntityRecord::type
  static const uint16_t CONFIDENCE      = (1u << 4);  ///< QualityComponent::confidence
  static const uint16_t LIFECYCLE_STATUS= (1u << 5);  ///< LifecycleComponent::status
  static const uint16_t MIL_CLASS       = (1u << 6);  ///< Full MilClassProfile
  static const uint16_t BEHAVIOR        = (1u << 7);  ///< BehaviorComponent
  static const uint16_t IDENTITY_NAME   = (1u << 8);  ///< IdentityComponent::name
  static const uint16_t ALL             = 0x01FFu;     ///< All defined bits
}  // namespace FieldMaskBit

/// \brief Decoded representation of a single entity update or delete frame.
struct EntityUpdateFrame {
  uint8_t  message_type = STREAM_MSG_ENTITY_UPDATE;
  UUIDKey  entity_id;
  uint64_t version   = 0;
  double   timestamp = 0.0;
  uint16_t field_mask = 0;

  tl::optional<Position>       position;
  tl::optional<Velocity>       velocity;
  tl::optional<Affiliation>    affiliation;
  tl::optional<ObjectType>     object_type;
  tl::optional<double>         confidence;
  tl::optional<LifecycleStatus> lifecycle_status;
  tl::optional<MilClassProfile> mil_class;
  tl::optional<BehaviorComponent> behavior;
  tl::optional<std::string>    identity_name;
};

/// \brief Compact binary codec for high-rate entity streaming.
///
/// Wire format is little-endian.  No new external dependencies.
class StreamingCodec {
public:
  // -----------------------------------------------------------------------
  // Single-entity encode / decode
  // -----------------------------------------------------------------------

  /// \brief Encode an entity update frame by reading components from the store.
  /// Only fields indicated by \p field_mask are serialised.
  static std::vector<uint8_t> encodeEntityUpdate(
      const UUIDKey& entity_id,
      uint64_t        version,
      double          timestamp,
      uint16_t        field_mask,
      const ObjectStore& store);

  /// \brief Encode a pre-built EntityUpdateFrame.
  static std::vector<uint8_t> encodeEntityUpdateFrame(const EntityUpdateFrame& frame);

  /// \brief Decode a single entity update/delete frame.
  /// Returns a frame with message_type == 0 on error.
  static EntityUpdateFrame decodeEntityUpdate(const uint8_t* data, size_t len);

  // -----------------------------------------------------------------------
  // Batch encode / decode
  // -----------------------------------------------------------------------

  /// \brief Encode multiple frames into one batch message.
  ///
  /// Batch header: frame_type(1)=0x03 + entity_count(4) + tick_timestamp(8)
  /// Followed by concatenated per-entity payloads (each prefixed by its size as uint32_t).
  static std::vector<uint8_t> encodeBatchFrame(
      const std::vector<EntityUpdateFrame>& frames,
      double tick_timestamp = 0.0);

  /// \brief Decode a batch frame into individual EntityUpdateFrames.
  static std::vector<EntityUpdateFrame> decodeBatchFrame(const uint8_t* data, size_t len);

  // -----------------------------------------------------------------------
  // Enum ordinal helpers — O(1), no string allocation
  // -----------------------------------------------------------------------
  static uint8_t affiliationToOrdinal(Affiliation a);
  static Affiliation ordinalToAffiliation(uint8_t v);

  static uint8_t objectTypeToOrdinal(ObjectType t);
  static ObjectType ordinalToObjectType(uint8_t v);

  static uint8_t lifecycleStatusToOrdinal(LifecycleStatus s);
  static LifecycleStatus ordinalToLifecycleStatus(uint8_t v);

  static uint8_t battleDimensionToOrdinal(BattleDimension d);
  static BattleDimension ordinalToBattleDimension(uint8_t v);

  static uint8_t milStatusToOrdinal(MilStatus s);
  static MilStatus ordinalToMilStatus(uint8_t v);

  static uint8_t echelonToOrdinal(Echelon e);
  static Echelon ordinalToEchelon(uint8_t v);

  static uint8_t mobilityToOrdinal(Mobility m);
  static Mobility ordinalToMobility(uint8_t v);
};

}  // namespace tactical_objects
