#pragma once

#include <TacticalObjectsRuntime.h>
#include <pcl/component.hpp>

#include <string>
#include <utility>
#include <vector>

namespace tactical_objects {

class BridgeServiceHandler;

/// \brief Proto-facing bridge hosted inside the tactical objects app.
///
/// The runtime still owns the internal business logic and binary entity-update
/// stream. This component exposes the generated PYRAMID tactical-objects
/// provided/consumed interface on top of that runtime using the generated
/// codecs/bindings, with a selectable frontend codec (`application/json`,
/// `application/flatbuffers`, or `application/protobuf`).
class StandardBridge : public pcl::Component {
public:
  StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec,
                 std::string frontend_content_type = "application/json",
                 bool expose_consumed_interface = true);

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_tick(double dt) override;

private:
  static pcl_status_t handleCreateRequirement(pcl_container_t* c,
                                              const pcl_msg_t* request,
                                              pcl_msg_t* response,
                                              pcl_svc_context_t* ctx,
                                              void* user_data);
  static pcl_status_t handleReadRequirement(pcl_container_t* c,
                                            const pcl_msg_t* request,
                                            pcl_msg_t* response,
                                            pcl_svc_context_t* ctx,
                                            void* user_data);
  static pcl_status_t handleUpdateRequirement(pcl_container_t* c,
                                              const pcl_msg_t* request,
                                              pcl_msg_t* response,
                                              pcl_svc_context_t* ctx,
                                              void* user_data);
  static pcl_status_t handleDeleteRequirement(pcl_container_t* c,
                                              const pcl_msg_t* request,
                                              pcl_msg_t* response,
                                              pcl_svc_context_t* ctx,
                                              void* user_data);
  static pcl_status_t handleReadMatch(pcl_container_t* c,
                                      const pcl_msg_t* request,
                                      pcl_msg_t* response,
                                      pcl_svc_context_t* ctx,
                                      void* user_data);
  static pcl_status_t handleReadDetail(pcl_container_t* c,
                                       const pcl_msg_t* request,
                                       pcl_msg_t* response,
                                       pcl_svc_context_t* ctx,
                                       void* user_data);

  static void onStandardObjectEvidence(pcl_container_t* c, const pcl_msg_t* msg,
                                       void* user_data);

  pcl_status_t dispatchProvidedService(int channel, const pcl_msg_t* request,
                                       pcl_msg_t* response);
  void publishEntityMatches(const std::vector<std::string>& entity_ids);
  void publishEvidenceRequirement(const std::string& payload);
  bool supportsContentType(const char* content_type) const;

  friend class BridgeServiceHandler;

  TacticalObjectsRuntime& runtime_;
  pcl_executor_t* exec_ = nullptr;
  std::string frontend_content_type_;
  bool expose_consumed_interface_ = true;

  pcl_port_t* pub_entity_matches_ = nullptr;
  pcl_port_t* pub_evidence_reqs_ = nullptr;

  std::vector<std::pair<UUIDKey, SubscriptionHandle>> interests_;

  std::string response_buffer_;
  std::string publish_buffer_;
};

} // namespace tactical_objects
