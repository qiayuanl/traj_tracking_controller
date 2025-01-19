#include <memory>

#include "traj_tracking_controller/TrajTrackingController.h"

namespace legged {
vector_t TrajTrackingController::playModel(const vector_t& observations) const {
  std::cerr << "TrajTrackingController::playModel" << std::endl;
  return OnnxController::playModel(observations);
}
}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::TrajTrackingController, controller_interface::ControllerInterface)
