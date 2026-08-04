#pragma once

/// \file agra_action_dispatch_component.hpp
/// \brief Bind AME action dispatch to A-GRA MA_ActionCommand generated Ports.
///
/// Boundary mapping:
/// - AME's string `command_id` becomes a deterministic native UUID in
///   `capability.base.command_id.uuid`; the bridge retains the reverse
///   correlation until a terminal status arrives.
/// - `action_name` and `signature` derive `capability.action_id.uuid`, while
///   the signature (or action name when absent) is its descriptive label.
///   `capability_id` identifies the action-registry capability deterministically
///   from `action_name`; the vehicle-facing registry resolves both identities.
/// - `service_name`, `operation`, and `request_fields` are deliberately not
///   carried. MA_ActionCommand has no free-form dispatch-parameter field, and
///   the receiving host owns service/operation resolution in its action
///   registry. Inventing a side channel here would make Python, PDDL, and the
///   wire compete as planning authorities.
/// - AME result states map to A-GRA command-processing states: pending to
///   received, running/succeeded to accepted, failures to rejected, and
///   cancelled to canceled. A returned accepted state is treated as the
///   terminal successful result on this boundary.
/// - `observed_updates` and `source` have no MA_ActionCommandStatus field and
///   are deliberately absent. A future fact-ingress contract must carry
///   observations rather than hiding them in this command acknowledgement.
///
/// The generated client is the Mission Autonomy role: its deployment
/// descriptor declares MA_ActionCommand as a publisher and
/// MA_ActionCommandStatus as a subscriber. Codec and transport remain
/// deployment choices.

#include "ame/agra_ma_bridge.h"
#include "ame/agra_oms_uuid.h"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <cstdio>
#include <exception>
#include <string>

namespace ame {

namespace action_dispatch_detail {

inline void headerNativeToWire(agra::HeaderType& header) {
  header.system_id.uuid = omsUuidFromNative(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        omsUuidFromNative(header.mission_id->base.uuid);
  }
}

inline void headerWireToNative(agra::HeaderType& header) {
  header.system_id.uuid = nativeUuidFromOms(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        nativeUuidFromOms(header.mission_id->base.uuid);
  }
}

inline agra_c2_provided::MA_ActionCommand_Service_Information
actionCommandNativeToWire(
    agra_c2_provided::MA_ActionCommand_Service_Information information) {
  if (!information.ma_action_command.has_value()) {
    return information;
  }
  auto& message = information.ma_action_command.value();
  headerNativeToWire(message.message_header);
  for (auto& item : message.message_data.command) {
    if (!item.capability.has_value()) {
      continue;
    }
    auto& capability = item.capability.value();
    capability.base.command_id.uuid =
        omsUuidFromNative(capability.base.command_id.uuid);
    capability.capability_id.uuid =
        omsUuidFromNative(capability.capability_id.uuid);
    capability.action_id.base.uuid =
        omsUuidFromNative(capability.action_id.base.uuid);
  }
  return information;
}

inline agra_c2_provided::MA_ActionCommandStatus_Service_Information
actionCommandStatusWireToNative(
    agra_c2_provided::MA_ActionCommandStatus_Service_Information
        information) {
  if (!information.ma_action_command_status.has_value()) {
    return information;
  }
  auto& message = information.ma_action_command_status.value();
  headerWireToNative(message.message_header);
  message.message_data.base.command_id.uuid =
      nativeUuidFromOms(message.message_data.base.command_id.uuid);
  if (message.message_data.base.command_processing_state_reason
          .has_value() &&
      message.message_data.base.command_processing_state_reason
          ->associated_id.has_value()) {
    auto& associated_id =
        message.message_data.base.command_processing_state_reason
            ->associated_id.value();
    associated_id.uuid = nativeUuidFromOms(associated_id.uuid);
  }
  for (auto& activity : message.message_data.activity) {
    activity.activity_id.uuid =
        nativeUuidFromOms(activity.activity_id.uuid);
  }
  return information;
}

}  // namespace action_dispatch_detail

/// \brief Mission Autonomy's generated-Port adapter for outbound actions.
class AgraActionDispatchComponent final : public pcl::Component {
public:
  AgraActionDispatchComponent(
      pcl::Executor& executor, AgraMaBridge& bridge,
      std::string content_type = "application/oms-json",
      double tick_rate_hz = 20.0)
      : pcl::Component("automtk_agra_action_dispatch"),
        bridge_(bridge),
        action_command_port_(
            *this, executor, std::move(content_type)) {
    setTickRateHz(tick_rate_hz);
  }

  std::size_t published_command_count() const {
    return published_commands_;
  }

  std::size_t received_status_count() const {
    return received_statuses_;
  }

protected:
  pcl_status_t on_configure() override {
    const pcl_status_t bound = action_command_port_.bind();
    if (bound != PCL_OK) {
      return bound;
    }
    subscription_ = action_command_port_.transitions(
        [this](
            const agra_c2_provided::
                MA_ActionCommandStatus_Service_Information& status) {
          receiveStatus(status);
        });
    return subscription_ ? PCL_OK : PCL_ERR_STATE;
  }

  pcl_status_t on_tick(double) override {
    if (!status_error_.empty()) {
      std::fprintf(
          stderr,
          "[automtk_agra_action_dispatch] status refused: %s\n",
          status_error_.c_str());
      // Report once, then clear: a single transient or duplicate status from
      // an otherwise-healthy peer must not permanently wedge the outbound
      // dispatch pump. Without this, every later tick would keep returning
      // PCL_ERR_INVALID here, before ever reaching pullActionCommandMessages()
      // below, even after the peer recovered.
      status_error_.clear();
      return PCL_ERR_INVALID;
    }
    try {
      for (auto& command : bridge_.pullActionCommandMessages()) {
        const pcl_status_t published = action_command_port_.submit(
            action_dispatch_detail::actionCommandNativeToWire(
                std::move(command)));
        if (published != PCL_OK) {
          return published;
        }
        ++published_commands_;
      }
    } catch (const std::exception& error) {
      std::fprintf(
          stderr,
          "[automtk_agra_action_dispatch] dispatch failed: %s\n",
          error.what());
      return PCL_ERR_INVALID;
    }
    return PCL_OK;
  }

private:
  void receiveStatus(
      const agra_c2_provided::
          MA_ActionCommandStatus_Service_Information& status) {
    try {
      bridge_.pushActionCommandStatus(
          action_dispatch_detail::actionCommandStatusWireToNative(
              status));
      ++received_statuses_;
    } catch (const std::exception& error) {
      status_error_ = error.what();
    }
  }

  AgraMaBridge& bridge_;
  agra_c2_provided::MaActioncommandStatusPortClient
      action_command_port_;
  agra_c2_provided::SubscriptionHandle subscription_;
  std::string status_error_;
  std::size_t published_commands_ = 0;
  std::size_t received_statuses_ = 0;
};

}  // namespace ame
