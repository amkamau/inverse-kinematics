#include "xarm_controller_hardware/xarm_hardware.hpp"
#include "xarm_controller_hardware/xarm_sdk.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <exception>
#include <algorithm>
#include <string>



namespace xarm_controller_hardware{

XArmHardware::XArmHardware(): sdk_(true){}

hardware_interface::CallbackReturn XArmHardware::on_init(const hardware_interface::HardwareInfo & info){

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware mode (default: mock)
  auto mock_param = info.hardware_parameters.find("mock_hardware");
  if (mock_param != info.hardware_parameters.end()) {
    if (mock_param->second == "true") {
      mock_hardware_ = true;
    } else if (mock_param->second == "false") {
      mock_hardware_ = false;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("XArmHardware"),
        "Invalid mock_hardware value: %s. Use true or false.",
        mock_param->second.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const size_t num_joints = info.joints.size();

  if (num_joints != 6) {
    RCLCPP_ERROR(
      rclcpp::get_logger("XArmHardware"),
      "Expected 6 joints, got %zu",
      num_joints);

    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  last_commands_.resize(num_joints, 0.0);
  // Read and validate joint limits
  joint_position_min_.resize(num_joints);
  joint_position_max_.resize(num_joints);
  joint_velocity_max_.resize(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    const auto & params = info.joints[i].parameters;

    try {
      joint_position_min_[i] = std::stod(params.at("min_position"));
      joint_position_max_[i] = std::stod(params.at("max_position"));
      joint_velocity_max_[i] = std::stod(params.at("max_velocity"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XArmHardware"),
        "Missing or invalid limits for joint %s: %s",
        info.joints[i].name.c_str(), e.what());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint_position_min_[i] >= joint_position_max_[i] ||
        joint_velocity_max_[i] <= 0.0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XArmHardware"),
        "Invalid limits configured for joint %s",
        info.joints[i].name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Physical servo mapping:
  // joint1 -> servo6, joint2 -> servo5,joint3 -> servo4, joint4 -> servo3, joint5 -> servo2, joint6 -> servo1
  servo_ids_ = {6, 5, 4, 3, 2, 1};

  RCLCPP_INFO(
    rclcpp::get_logger("XArmHardware"),
    "XArm hardware initialized with %zu joints",
    num_joints);

  RCLCPP_INFO(
    rclcpp::get_logger("XArmHardware"),
    "Hardware mode: %s",
    mock_hardware_ ? "MOCK" : "REAL");
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>XArmHardware::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>XArmHardware::export_command_interfaces(){  
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }
  return interfaces;
}


hardware_interface::CallbackReturn XArmHardware::on_activate(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Activating XArm hardware in %s mode", mock_hardware_ ? "MOCK" : "REAL");
  if (mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Mock hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  if (!sdk_.connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Failed to connect to Hiwonder xArm");
    return hardware_interface::CallbackReturn::ERROR;
  }

  connected_ = true;

  std::vector<Servo> servos;
  for (int id : servo_ids_) {
    servos.emplace_back(id);
  }

  if (!sdk_.getPositions(servos)) {
    RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Failed to read initial servo positions");
    sdk_.disconnect();
    connected_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < servos.size(); ++i) {
    const double degrees = XArmSDK::positionToAngle(servos[i].position);
    const double radians = degrees * M_PI / 180.0;

    hw_positions_[i] = radians;
    hw_commands_[i] = radians;
  }

  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Connected and initialized from current servo positions");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XArmHardware::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Deactivating XArm hardware");
  if (!mock_hardware_ && connected_) {
    sdk_.disconnect();
    connected_ = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

  hardware_interface::return_type XArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &){
    // Mock hardware: simulated feedback
    if (mock_hardware_) {
      hw_positions_ = hw_commands_;
      return hardware_interface::return_type::OK;
    }
    // Real hardware: read physical servo positions
    if (!connected_ || !sdk_.isConnected()) {
      RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Cannot read: xArm is not connected");
      return hardware_interface::return_type::ERROR;
    }

    std::vector<Servo> servos;

    for (int id : servo_ids_) {
      servos.emplace_back(id);
    }

    if (!sdk_.getPositions(servos)) {
      RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Failed to read servo positions");
      return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < servos.size(); ++i) {
      const double degrees = XArmSDK::positionToAngle(servos[i].position);
      hw_positions_[i] = degrees * M_PI / 180.0;
    }

    return hardware_interface::return_type::OK;
  }


  hardware_interface::return_type XArmHardware::write(const rclcpp::Time &, const rclcpp::Duration & period){

    // Validate and clamp commanded joint positions
    std::vector<double> safe_commands = hw_commands_;

    for (size_t i = 0; i < hw_commands_.size(); ++i) {

      safe_commands[i] = std::clamp(hw_commands_[i], joint_position_min_[i], joint_position_max_[i]);

      // RCLCPP_INFO(
      //   rclcpp::get_logger("XArmHardware"),
      //   "Joint %s command = %.2f rad, limits=[%.2f, %.2f]",
      //   info_.joints[i].name.c_str(),
      //   hw_commands_[i],
      //   joint_position_min_[i],
      //   joint_position_max_[i]);

      // if (safe_commands[i] != hw_commands_[i]) {
      //   RCLCPP_WARN(
      //     rclcpp::get_logger("XArmHardware"),
      //     "Joint %s exceeds position limits! Requested: %.2f rad, clamped to: %.2f rad",
      //     info_.joints[i].name.c_str(),
      //     hw_commands_[i],
      //     safe_commands[i]);
      // }

    }

    // Mock hardware: commands are consumed by read()
    if (mock_hardware_) {
      return hardware_interface::return_type::OK;
    }
    if (!connected_ || !sdk_.isConnected()) {
      RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Cannot write: xArm is not connected");
      return hardware_interface::return_type::ERROR;
    }
    if (safe_commands == last_commands_) {
      return hardware_interface::return_type::OK;
    }

    std::vector<Servo> servos;

    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      const double degrees = safe_commands[i] * 180.0 / M_PI;

      try {
        servos.emplace_back(
          servo_ids_[i],
          XArmSDK::angleToPosition(degrees));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("XArmHardware"),
          "Invalid command for joint %s: %s",
          info_.joints[i].name.c_str(),
          e.what());

        return hardware_interface::return_type::ERROR;
      }
    }

    const int duration_ms = std::max(
      1,
      static_cast<int>(period.seconds() * 1000.0));

    if (!sdk_.setPositions(servos, duration_ms)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XArmHardware"),
        "Failed to send servo commands");

      return hardware_interface::return_type::ERROR;
    }

    last_commands_ = safe_commands;
    
    return hardware_interface::return_type::OK;
  }

}  // namespace xarm_controller_hardware

PLUGINLIB_EXPORT_CLASS(xarm_controller_hardware::XArmHardware, hardware_interface::SystemInterface)
