#pragma once

/// \file agra_c2_component.hpp
/// \brief The commanding side of the A-GRA task boundary as a component.
///
/// Submits MA_TaskCommand and records the correlated MA_TaskCommandStatus the
/// Mission Autonomy side decides. Correlation is by command id taken from the
/// status itself, so a peer that answers out of order, or answers only some
/// commands, is still read correctly.

#include "pyramid_services_agra_c2_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <cstdio>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

namespace agra_c2_provided =
    pyramid::components::agra::c2::services::provided;

class AgraC2Component final : public pcl::Component {
public:
  using Command = agra_c2_provided::MA_TaskCommand_Service_Information;
  using Status = agra_c2_provided::MA_TaskCommandStatus_Service_Information;

  /// \param commands Submitted once, on the first tick after binding.
  /// \param command_id Reads the correlating id out of a status, so this
  ///        component needs no knowledge of the message layout.
  AgraC2Component(pcl::Executor& executor, std::vector<Command> commands,
                  std::function<std::string(const Status&)> command_id,
                  std::string content_type = "application/oms-json",
                  double tick_rate_hz = 20.0)
      : pcl::Component("automtk_agra_c2"),
        commands_(std::move(commands)),
        command_id_(std::move(command_id)),
        task_command_port_(*this, executor, std::move(content_type)) {
    setTickRateHz(tick_rate_hz);
  }

  const std::unordered_map<std::string, Status>& statuses() const {
    return statuses_;
  }

  /// Order statuses arrived in, which is not necessarily the order commands
  /// were submitted; kept so a caller can assert correlation rather than
  /// sequence.
  const std::vector<std::string>& arrival_order() const {
    return arrival_order_;
  }

protected:
  pcl_status_t on_configure() override {
    const pcl_status_t bound = task_command_port_.bind();
    if (bound != PCL_OK) {
      return bound;
    }
    subscription_ = task_command_port_.transitions(
        [this](const Status& status) { record(status); });
    return subscription_ ? PCL_OK : PCL_ERR_STATE;
  }

  pcl_status_t on_tick(double) override {
    if (submitted_) {
      return PCL_OK;
    }
    for (const auto& command : commands_) {
      const pcl_status_t status = task_command_port_.submit(command);
      std::fprintf(stderr, "[c2] submit -> %d\n", static_cast<int>(status));
      if (status != PCL_OK) {
        return status;
      }
    }
    submitted_ = true;
    return PCL_OK;
  }

private:
  void record(const Status& status) {
    const std::string id = command_id_(status);
    if (statuses_.emplace(id, status).second) {
      arrival_order_.push_back(id);
    }
  }

  std::vector<Command> commands_;
  std::function<std::string(const Status&)> command_id_;
  agra_c2_provided::MaTaskcommandStatusPortClient task_command_port_;
  agra_c2_provided::SubscriptionHandle subscription_;
  std::unordered_map<std::string, Status> statuses_;
  std::vector<std::string> arrival_order_;
  bool submitted_ = false;
};

}  // namespace ame
