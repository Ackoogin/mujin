#pragma once

#include <TacticalObjectsTypes.h>
#include <query/QueryEngine.h>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace tactical_objects {

/// \brief JSON serialization/deserialization for tactical_objects messages.
class TacticalObjectsCodec {
public:
  TacticalObjectsCodec() = delete;

  // --- UUID ---
  static nlohmann::json encodeUUID(const UUIDKey& key);
  static UUIDKey decodeUUID(const nlohmann::json& j);

  // --- Position ---
  static nlohmann::json encodePosition(const Position& pos);
  static Position decodePosition(const nlohmann::json& j);

  // --- Observation ---
  static nlohmann::json encodeObservation(const Observation& obs);
  static Observation decodeObservation(const nlohmann::json& j);

  // --- ObservationBatch ---
  static nlohmann::json encodeObservationBatch(const ObservationBatch& batch);
  static ObservationBatch decodeObservationBatch(const nlohmann::json& j);

  // --- ObjectDefinition ---
  static nlohmann::json encodeObjectDefinition(const ObjectDefinition& def);
  static ObjectDefinition decodeObjectDefinition(const nlohmann::json& j);

  // --- ObjectUpdate ---
  static nlohmann::json encodeObjectUpdate(const ObjectUpdate& upd);
  static ObjectUpdate decodeObjectUpdate(const nlohmann::json& j);

  static std::string objectTypeToString(ObjectType t);

  // --- ZoneDefinition ---
  static nlohmann::json encodeZoneDefinition(const ZoneDefinition& def);
  static ZoneDefinition decodeZoneDefinition(const nlohmann::json& j);

  // --- QueryRequest / QueryResponse ---
  static nlohmann::json encodeQueryRequest(const QueryRequest& req);
  static QueryRequest decodeQueryRequest(const nlohmann::json& j);
  static nlohmann::json encodeQueryResponse(const QueryResponse& resp);
  static QueryResponse decodeQueryResponse(const nlohmann::json& j);

private:
  static ObjectType stringToObjectType(const std::string& s);
  static std::string affiliationToString(Affiliation a);
  static Affiliation stringToAffiliation(const std::string& s);
  static std::string zoneTypeToString(ZoneType z);
  static ZoneType stringToZoneType(const std::string& s);
  static std::string battleDimToString(BattleDimension b);
  static BattleDimension stringToBattleDim(const std::string& s);

  static nlohmann::json encodeSourceRef(const SourceRef& sr);
  static SourceRef decodeSourceRef(const nlohmann::json& j);
  static nlohmann::json encodeMilClassProfile(const MilClassProfile& p);
  static MilClassProfile decodeMilClassProfile(const nlohmann::json& j);
};

} // namespace tactical_objects
