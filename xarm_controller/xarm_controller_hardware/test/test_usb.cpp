
#include "xarm_controller_hardware/xarm_sdk.hpp"

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

int main()
{
  xarm_controller_hardware::XArmSDK arm(true);

  if (!arm.connect()) {
    std::cerr << "USB connection failed." << std::endl;
    return 1;
  }

  std::cout << "\nReading servo positions...\n" << std::endl;

  // Test read servo poisitions
  std::vector<xarm_controller_hardware::Servo> servos;
  for (int id = 1; id <= 6; ++id) {
    servos.emplace_back(id);
  }
  if (arm.getPositions(servos)) {
    std::cout << "\nMulti-servo read successful:\n";

    for (const auto & servo : servos) {
      std::cout << "Servo " << servo.servo_id
                << ": position = " << servo.position
                << ", angle = " << servo.angle()
                << " degrees" << std::endl;
    }
  } else {
    std::cout << "Multi-servo read failed." << std::endl;
  }

  // Test get battery voltage
  double voltage = 0.0;
  if (arm.getBatteryVoltage(voltage)) {
    std::cout << "\nBattery voltage: "
              << voltage << " V" << std::endl;
  } else {
    std::cout << "\nFailed to read battery voltage."
              << std::endl;
  }


  // Test write position
//   // Test servo movement
// const int servo_id = 1;

// int current_position = 0;

// if (arm.getPosition(1, current_position)) {
//   const int target = current_position + 40;

//   if (target <= 1000 && arm.setPosition(1, target, 2000)) {
//     std::cout << "Target: " << target << std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(2500));

//     int actual = 0;

//     if (arm.getPosition(1, actual)) {
//       std::cout << "Actual: " << actual << std::endl;
//     }
//   }
// }

servos.clear();
// Test multi-servo movement
// std::vector<xarm_controller_hardware::Servo> servos;
for (int id = 1; id <= 6; ++id) {
  int current_position = 0;

  if (!arm.getPosition(id, current_position)) {
    std::cout << "Failed to read servo " << id << std::endl;
    return 1;
  }

  servos.emplace_back(id, current_position + 5);
}

std::cout << "\nSending multi-servo movement command...\n";

if (arm.setPositions(servos, 1000)) {
  std::cout << "Multi-servo command sent successfully.\n";

  std::this_thread::sleep_for(
    std::chrono::milliseconds(1500));

  std::vector<xarm_controller_hardware::Servo> actual;

  for (int id = 1; id <= 6; ++id) {
    actual.emplace_back(id);
  }

  if (arm.getPositions(actual)) {
    std::cout << "\nPosition feedback:\n";

    for (size_t i = 0; i < actual.size(); ++i) {
      std::cout << "Servo " << actual[i].servo_id
                << " | Target: " << servos[i].position
                << " | Actual: " << actual[i].position
                << std::endl;
    }
  } else {
    std::cout << "Failed to read positions after movement.\n";
  }
} else {
  std::cout << "Multi-servo command failed.\n";
}


  arm.disconnect();

  return 0;
}
