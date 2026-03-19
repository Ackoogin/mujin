#pragma once

#include <TacticalObjectsRuntime.h>
#include <StreamingCodec.h>
#include <pcl/component.hpp>

#include <string>

namespace tactical_objects {

/// \brief Bridge component that translates between the standard pyramid proto
///        interface and the internal TacticalObjects wire format.
///
/// External consumers (Ada clients, future gRPC clients) call:
///   - Service  "object_of_interest.create_requirement"  (JSON, standard)
///   - Topic    "standard.object_evidence"               (JSON, standard)
///
/// The bridge translates and delegates to:
///   - Service  "subscribe_interest"       (JSON, internal)
///   - Runtime  processObservationBatch()  (direct in-process call)
///
/// Internal outputs forwarded as standard topics:
///   - "entity_updates"           (binary)  → "standard.entity_matches"  (JSON)
///   - "evidence_requirements"    (JSON)    → "standard.evidence_requirements" (JSON)
///
/// Standard JSON formats:
///
///   create_requirement input:
///     { "policy":    "DATA_POLICY_OBTAIN" | "DATA_POLICY_QUERY",
///       "identity":  "STANDARD_IDENTITY_HOSTILE" | ... ,
///       "dimension": "BATTLE_DIMENSION_SEA_SURFACE" | ... ,
///       "min_lat_rad": <double>,  "max_lat_rad": <double>,
///       "min_lon_rad": <double>,  "max_lon_rad": <double> }
///
///   standard.entity_matches output (array):
///     [ { "object_id":     "<uuid>",
///         "identity":      "STANDARD_IDENTITY_HOSTILE" | ... ,
///         "dimension":     "BATTLE_DIMENSION_SEA_SURFACE" | ... ,
///         "latitude_rad":  <double>,
///         "longitude_rad": <double>,
///         "confidence":    <double> }, ... ]
///
///   standard.object_evidence input:
///     { "identity":      "STANDARD_IDENTITY_HOSTILE" | ... ,
///       "dimension":     "BATTLE_DIMENSION_SEA_SURFACE" | ... ,
///       "latitude_rad":  <double>,
///       "longitude_rad": <double>,
///       "confidence":    <double> }

class StandardBridge : public pcl::Component {
public:
  /// \param runtime  Reference to the TacticalObjectsRuntime (owned by TacticalObjectsComponent).
  /// \param exec     The shared PCL executor (needed to call subscribe_interest locally).
  StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec);

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;

private:
  // ---- PCL service handler ------------------------------------------------

  /// Handle "object_of_interest.create_requirement" → translate → subscribe_interest
  static pcl_status_t handleCreateRequirement(pcl_container_t* c,
                                               const pcl_msg_t* request,
                                               pcl_msg_t* response,
                                               void* user_data);

  // ---- PCL subscriber callbacks -------------------------------------------

  /// entity_updates (binary) → standard.entity_matches (JSON)
  static void onEntityUpdates(pcl_container_t* c, const pcl_msg_t* msg, void* user_data);

  /// evidence_requirements (JSON) → standard.evidence_requirements (JSON)
  static void onEvidenceRequirements(pcl_container_t* c, const pcl_msg_t* msg, void* user_data);

  /// standard.object_evidence (JSON) → processObservationBatch
  static void onStandardObjectEvidence(pcl_container_t* c, const pcl_msg_t* msg, void* user_data);

  // ---- Enum converters (standard ↔ internal strings) ----------------------

  static std::string affiliationToStandardIdentity(Affiliation a);
  static Affiliation  standardIdentityToAffiliation(const std::string& s);
  static std::string  battleDimToStandard(BattleDimension d);
  static BattleDimension standardToBattleDim(const std::string& s);

  // ---- Position conversion -------------------------------------------------
  static double degToRad(double deg) { return deg * 0.017453292519943295; }
  static double radToDeg(double rad) { return rad * 57.29577951308232; }

  // ---- Member state --------------------------------------------------------

  TacticalObjectsRuntime& runtime_;
  pcl_executor_t*         exec_;

  pcl_port_t* pub_entity_matches_   = nullptr;
  pcl_port_t* pub_evidence_reqs_    = nullptr;

  /// Reusable response buffer for handleCreateRequirement.
  std::string resp_buf_;
  /// Reusable publish buffer for onEntityUpdates.
  std::string pub_buf_;
};

} // namespace tactical_objects
