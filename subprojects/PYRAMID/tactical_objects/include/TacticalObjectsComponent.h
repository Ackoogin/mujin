#pragma once

#include <TacticalObjectsRuntime.h>
#include <TacticalObjectsCodec.h>
#include <pcl/component.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace tactical_objects {

/// \brief PCL-backed tactical objects component with lifecycle and ports.
class TacticalObjectsComponent : public pcl::Component {
public:
  TacticalObjectsComponent();

  /// \brief Access the runtime for in-process integrations.
  TacticalObjectsRuntime& runtime() { return *runtime_; }
  const TacticalObjectsRuntime& runtime() const { return *runtime_; }

  /// \brief Set streaming tick divisor (publish every N ticks). For testing/config.
  void setStreamingTickDivisor(int v) { streaming_tick_divisor_ = v; }

  /// \brief Set max entities per frame for chunking. For testing/config.
  void setMaxEntitiesPerFrame(int v) { max_entities_per_frame_ = v; }

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;
  pcl_status_t on_tick(double dt) override;

private:
  static void onObservationIngress(pcl_container_t* c, const pcl_msg_t* msg,
                                   void* user_data);

  static pcl_status_t handleCreateObject(pcl_container_t* c,
                                          const pcl_msg_t* request,
                                          pcl_msg_t* response,
                                          pcl_svc_context_t* ctx,
                                          void* user_data);
  static pcl_status_t handleUpdateObject(pcl_container_t* c,
                                          const pcl_msg_t* request,
                                          pcl_msg_t* response,
                                          pcl_svc_context_t* ctx,
                                          void* user_data);
  static pcl_status_t handleDeleteObject(pcl_container_t* c,
                                          const pcl_msg_t* request,
                                          pcl_msg_t* response,
                                          pcl_svc_context_t* ctx,
                                          void* user_data);
  static pcl_status_t handleQuery(pcl_container_t* c,
                                   const pcl_msg_t* request,
                                   pcl_msg_t* response,
                                   pcl_svc_context_t* ctx,
                                   void* user_data);
  static pcl_status_t handleGetObject(pcl_container_t* c,
                                       const pcl_msg_t* request,
                                       pcl_msg_t* response,
                                       pcl_svc_context_t* ctx,
                                       void* user_data);
  static pcl_status_t handleUpsertZone(pcl_container_t* c,
                                       const pcl_msg_t* request,
                                       pcl_msg_t* response,
                                       pcl_svc_context_t* ctx,
                                       void* user_data);
  static pcl_status_t handleRemoveZone(pcl_container_t* c,
                                        const pcl_msg_t* request,
                                        pcl_msg_t* response,
                                        pcl_svc_context_t* ctx,
                                        void* user_data);

  /// \brief Service: register an interest and begin streaming matching entities.
  static pcl_status_t handleSubscribeInterest(pcl_container_t* c,
                                               const pcl_msg_t* request,
                                               pcl_msg_t* response,
                                               pcl_svc_context_t* ctx,
                                               void* user_data);

  /// \brief Service: request a full-snapshot resync for an interest.
  static pcl_status_t handleResync(pcl_container_t* c,
                                    const pcl_msg_t* request,
                                    pcl_msg_t* response,
                                    pcl_svc_context_t* ctx,
                                    void* user_data);

  std::shared_ptr<TacticalObjectsRuntime> runtime_;
  std::string response_buffer_;

  // Streaming state
  uint64_t tick_count_ = 0;
  int max_entities_per_frame_  = 500;  ///< Split frames larger than this
  int streaming_tick_divisor_  = 1;    ///< Publish every N ticks

  // Publisher ports
  pcl_port_t* entity_updates_port_ = nullptr;
  pcl_port_t* evidence_requirements_port_ = nullptr;

  // Per-interest monotonic sequence IDs (for gap detection)
  std::unordered_map<UUIDKey, uint64_t> sequence_ids_;

  // Encode buffer for binary publish (reused each tick)
  std::vector<uint8_t> encode_buf_;
};

} // namespace tactical_objects
