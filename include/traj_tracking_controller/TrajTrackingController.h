#pragma once

#include "legged_controllers/ControllerBase.h"

namespace legged {
class TrajTrackingController : public ControllerBase {
 public:
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::vector<std::string> jointNameInPolicy_;
  scalar_t actionScale_{0};

  std::vector<std::vector<double>> trajs;  // trajs[time][joint]
  rclcpp::Time startTime_;
};

}  // namespace legged
