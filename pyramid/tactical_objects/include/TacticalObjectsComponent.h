#pragma once

#include <TacticalObjectsRuntime.h>
#include <TacticalObjectsCodec.h>
#include <pcl/component.hpp>

#include <memory>
#include <string>

namespace tactical_objects {

/// \brief PCL-backed tactical objects component with lifecycle and ports.
class TacticalObjectsComponent : public pcl::Component {
public:
  TacticalObjectsComponent();

  /// \brief Access the runtime for in-process integrations.
  TacticalObjectsRuntime& runtime() { return *runtime_; }
  const TacticalObjectsRuntime& runtime() const { return *runtime_; }

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
                                          void* user_data);
  static pcl_status_t handleUpdateObject(pcl_container_t* c,
                                          const pcl_msg_t* request,
                                          pcl_msg_t* response,
                                          void* user_data);
  static pcl_status_t handleDeleteObject(pcl_container_t* c,
                                          const pcl_msg_t* request,
                                          pcl_msg_t* response,
                                          void* user_data);
  static pcl_status_t handleQuery(pcl_container_t* c,
                                   const pcl_msg_t* request,
                                   pcl_msg_t* response,
                                   void* user_data);
  static pcl_status_t handleGetObject(pcl_container_t* c,
                                       const pcl_msg_t* request,
                                       pcl_msg_t* response,
                                       void* user_data);
  static pcl_status_t handleUpsertZone(pcl_container_t* c,
                                       const pcl_msg_t* request,
                                       pcl_msg_t* response,
                                       void* user_data);
  static pcl_status_t handleRemoveZone(pcl_container_t* c,
                                        const pcl_msg_t* request,
                                        pcl_msg_t* response,
                                        void* user_data);

  std::shared_ptr<TacticalObjectsRuntime> runtime_;
  std::string response_buffer_;
};

} // namespace tactical_objects
