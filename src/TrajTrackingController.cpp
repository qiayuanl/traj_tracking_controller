#include <memory>

#include "traj_tracking_controller/TrajTrackingController.h"

namespace legged {
controller_interface::return_type TrajTrackingController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (ControllerBase::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }
  vector_t desiredPosition = defaultPosition_;
  for (size_t i = 0; i < desiredPosition.size(); ++i) {
    const size_t index = static_cast<size_t>(std::round((time - startTime_).seconds() * 50));
    desiredPosition[i] += 0.3 * trajs[index][i];
  }
  setPositions(desiredPosition);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TrajTrackingController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  if (ControllerBase::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string path;
  get_node()->get_parameter("traj_path", path);
  std::ifstream file(path);

  if (!file.is_open()) {
    std::cerr << "Error opening file: " << path << std::endl;
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> rowValues;

    while (std::getline(lineStream, cell, ',')) {
      try {
        double value = std::stod(cell);
        rowValues.push_back(value);
      } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid number in CSV: " << cell << std::endl;
        return controller_interface::CallbackReturn::ERROR;
      } catch (const std::out_of_range& e) {
        std::cerr << "Number out of range in CSV: " << cell << std::endl;
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    if (!rowValues.empty()) {
      trajs.push_back(rowValues);
    }
  }
  file.close();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajTrackingController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  if (ControllerBase::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  startTime_ = get_node()->now();
  std::cout << "TrajTrackingController activated " << startTime_.seconds() << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::TrajTrackingController, controller_interface::ControllerInterface)
