#ifndef XARM_CONTROLLER_HARDWARE__XARM_HARDWARE_HPP_
#define XARM_CONTROLLER_HARDWARE__XARM_HARDWARE_HPP_

#include "xarm_controller_hardware/xarm_sdk.hpp"

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace xarm_controller_hardware{
class XArmHardware : public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XArmHardware)

  XArmHardware();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;


  private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
  
  std::vector<double> joint_position_min_;
  std::vector<double> joint_position_max_;
  std::vector<double> joint_velocity_max_;

  std::vector<int> servo_ids_;

  std::vector<double> last_commands_;

  XArmSDK sdk_;

  bool mock_hardware_{true};
  bool connected_{false};
};

}  // namespace xarm_controller_hardware

#endif
