
#include "xarm_controller_hardware/xarm_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

namespace xarm_controller_hardware{
// --------------------------------------------------
// Servo
// --------------------------------------------------
Servo::Servo(int id, int pos): servo_id(id), position(pos){
  if (id < 1 || id > 6) {
    throw std::invalid_argument("Invalid servo ID");
  }
  if (pos < 0 || pos > 1000) {
    throw std::invalid_argument("Servo position must be between 0 and 1000");
  }
}

double Servo::angle() const{
  return XArmSDK::positionToAngle(position);
}

void Servo::setAngle(double degrees){
  position = XArmSDK::angleToPosition(degrees);
}

// --------------------------------------------------
// XArmSDK
// --------------------------------------------------
XArmSDK::XArmSDK(bool debug)
: debug_(debug){
}

XArmSDK::~XArmSDK(){
  disconnect();
}

bool XArmSDK::connect(const std::wstring & serial_number){
  if (device_) {
    return true;
  }

  if (hid_init() != 0) {
    return false;
  }

  device_ = hid_open(0x0483, 0x5750, serial_number.empty() ? nullptr : serial_number.c_str());

  if (!device_) {
    if (debug_) {
      std::cerr << "Unable to open Hiwonder USB device: "
                << hid_error(nullptr) << std::endl;
    }
    return false;
  }

  hid_set_nonblocking(device_, 0);

if (debug_) {
  wchar_t serial_number[256] = {};

  hid_get_serial_number_string(
    device_,
    serial_number,
    sizeof(serial_number) / sizeof(wchar_t)
  );

  std::wcerr << L"Connected to Hiwonder xArm. Serial: "
            << serial_number << std::endl;
}
  return true;
}

void XArmSDK::disconnect()
{
  if (device_) {
    hid_close(device_);
    device_ = nullptr;
    hid_exit();
  }
}

bool XArmSDK::isConnected() const
{
  return device_ != nullptr;
}

// --------------------------------------------------
// Position conversion
// --------------------------------------------------

int XArmSDK::angleToPosition(double degrees)
{
  if (!std::isfinite(degrees) ||
      degrees < -125.0 || degrees > 125.0) {
    throw std::invalid_argument("Angle must be between -125 and 125 degrees");
  }

  // Match Python's quarter-degree rounding.
  const double rounded = std::round(degrees * 4.0) / 4.0;

  const double normalized = (rounded + 125.0) / 250.0;

  return static_cast<int>(normalized * 1000.0);
}

double XArmSDK::positionToAngle(int position)
{
  if (position < 0 || position > 1000) {
    throw std::invalid_argument("Position must be between 0 and 1000");
  }

  return -125.0 + (250.0 * position / 1000.0);
}

// --------------------------------------------------
// USB packet transmission
// --------------------------------------------------

bool XArmSDK::send(
  uint8_t command,
  const std::vector<uint8_t> & data)
{
  if (!device_) {
    return false;
  }

  if (data.size() > 59) {
    return false;
  }

  unsigned char report[64] = {};

  report[0] = 0;  // HID report ID
  report[1] = SIGNATURE;
  report[2] = SIGNATURE;
  report[3] = static_cast<uint8_t>(data.size() + 2);
  report[4] = command;

  std::copy(data.begin(), data.end(), report + 5);

  if (debug_) {
    std::cout << "Sending command 0x"
              << std::hex << static_cast<int>(command)
              << std::dec << std::endl;
  }

  const int result = hid_write(device_, report, sizeof(report));
  if (debug_) {
    std::cout << "HID write result: " << result << std::endl;
  }

  return result >= 0;
}

// --------------------------------------------------
// USB packet reception
// --------------------------------------------------
bool XArmSDK::receive(uint8_t command, std::vector<uint8_t> & data){
  if (!device_) {
    return false;
  }

  unsigned char report[64] = {};

  const int result = hid_read_timeout(device_, report, sizeof(report), 50);

  if (debug_) {
    std::cout << "HID read result: " << result << std::endl;
    if (result > 0) {
      std::cout << "Raw response: ";
      for (int i = 0; i < result; ++i) {
        printf("%02X ", report[i]);
      }
      std::cout << std::endl;
    }
  }

  if (result < 4) {
    if (debug_) {
      std::cerr << "Invalid or incomplete HID response. " << "Bytes received: " << result << std::endl;
      if (result < 0) {
        std::cerr << "HID error: " << hid_error(device_) << std::endl;
      }
    }
    return false;
  }

  // Response header: 55 55 LENGTH COMMAND
  if (report[0] != SIGNATURE || report[1] != SIGNATURE || report[3] != command){
    if (debug_) {
      std::cerr << "Invalid HID response header. "
                << "Signature: "
                << std::hex << static_cast<int>(report[0]) << " "
                << static_cast<int>(report[1])
                << ", command: "
                << static_cast<int>(report[3])
                << ", expected: "
                << static_cast<int>(command)
                << std::dec << std::endl;
    }
    return false;
  }

  const size_t length = report[2];

  if (length > static_cast<size_t>(result - 4)) {
    return false;
  }

  // Payload starts immediately after the four-byte header.
  data.assign(report + 4, report + 4 + length);

  if (debug_) {
    std::cout << "Received " << data.size()
              << " payload bytes" << std::endl;
  }

  return true;
}

// --------------------------------------------------
// Set servo position
// --------------------------------------------------

bool XArmSDK::setPosition(
  int servo_id,
  int position,
  int duration)
{
  if (servo_id < 1 || servo_id > 6 ||
      position < 0 || position > 1000 ||
      duration < 0 || duration > 65535) {
    return false;
  }

  std::vector<uint8_t> data = {
    1,
    static_cast<uint8_t>(duration & 0xFF),
    static_cast<uint8_t>((duration >> 8) & 0xFF),
    static_cast<uint8_t>(servo_id),
    static_cast<uint8_t>(position & 0xFF),
    static_cast<uint8_t>((position >> 8) & 0xFF)
  };

  return send(CMD_SERVO_MOVE, data);
}

bool XArmSDK::setPositionDegrees(
  int servo_id,
  double degrees,
  int duration)
{
  try {
    return setPosition(
      servo_id, angleToPosition(degrees), duration);
  } catch (const std::exception &) {
    return false;
  }
}

bool XArmSDK::setPositions(const std::vector<Servo> & servos,int duration){

    if (debug_) {
    std::cout << "\nsetPositions() called\n";
    std::cout << "Servo count: " << servos.size() << '\n';
    std::cout << "Duration: " << duration << " ms\n";

    for (const auto & servo : servos) {
      std::cout << "Servo ID: " << servo.servo_id
                << ", Position: " << servo.position << '\n';
    }
  }
  
  if (servos.empty() || servos.size() > 6 ||
      duration < 0 || duration > 65535) {
    return false;
  }

  std::vector<uint8_t> data = {
    static_cast<uint8_t>(servos.size()),
    static_cast<uint8_t>(duration & 0xFF),
    static_cast<uint8_t>((duration >> 8) & 0xFF)
  };

  for (const auto & servo : servos) {
    if (servo.servo_id < 1 || servo.servo_id > 6 ||
        servo.position < 0 || servo.position > 1000) {
      if (debug_) {
        std::cerr << "Invalid servo data: ID="
                  << servo.servo_id
                  << ", Position=" << servo.position << '\n';
      }
      return false;
    }

    data.push_back(static_cast<uint8_t>(servo.servo_id));
    data.push_back(static_cast<uint8_t>(servo.position & 0xFF));
    data.push_back(static_cast<uint8_t>((servo.position >> 8) & 0xFF));
  }

  return send(CMD_SERVO_MOVE, data);
}

// --------------------------------------------------
// Get servo position
// --------------------------------------------------
bool XArmSDK::getPosition(int servo_id, int & position)
{
  std::vector<uint8_t> request = {
    1,
    static_cast<uint8_t>(servo_id)
  };

  std::vector<uint8_t> response;

  if (!send(CMD_GET_SERVO_POSITION, request)) {
    return false;
  }

  if (!receive(CMD_GET_SERVO_POSITION, response)) {
    return false;
  }

  if (response.size() < 4 ||
      response[0] != 1 ||
      response[1] != servo_id)
  {
    return false;
  }

  position =
    (static_cast<int>(response[3]) << 8) |
     static_cast<int>(response[2]);

  return true;
}

bool XArmSDK::getPositions(std::vector<Servo> & servos)
{
  if (servos.empty() || servos.size() > 6) {
    return false;
  }

  std::vector<uint8_t> request;

  request.push_back(static_cast<uint8_t>(servos.size()));

  for (const auto & servo : servos) {
    request.push_back(static_cast<uint8_t>(servo.servo_id));
  }

  std::vector<uint8_t> response;

  if (!send(CMD_GET_SERVO_POSITION, request)) {
    return false;
  }

  if (!receive(CMD_GET_SERVO_POSITION, response)) {
    return false;
  }

  // One count byte + three bytes per servo.
  if (response.size() < 1 + servos.size() * 3 ||
      response[0] != servos.size()) {
    return false;
  }

  for (size_t i = 0; i < servos.size(); ++i) {
    const size_t offset = 1 + (i * 3);

    if (response[offset] != servos[i].servo_id) {
      return false;
    }

    servos[i].position =
      static_cast<int>(response[offset + 1]) |
      (static_cast<int>(response[offset + 2]) << 8);
  }

  return true;
}


// --------------------------------------------------
// Servo off
// --------------------------------------------------

bool XArmSDK::servoOff(const std::vector<int> & servo_ids)
{
  std::vector<uint8_t> data;

  if (servo_ids.empty()) {
    data = {6, 1, 2, 3, 4, 5, 6};
  } else {
    if (servo_ids.size() > 6) {
      return false;
    }

    data.push_back(static_cast<uint8_t>(servo_ids.size()));

    for (int id : servo_ids) {
      if (id < 1 || id > 6) {
        return false;
      }

      data.push_back(static_cast<uint8_t>(id));
    }
  }

  return send(CMD_SERVO_STOP, data);
}

// --------------------------------------------------
// Battery voltage
// --------------------------------------------------

bool XArmSDK::getBatteryVoltage(double & voltage)
{
  if (!send(CMD_GET_BATTERY_VOLTAGE)) {
    return false;
  }

  std::vector<uint8_t> response;

  if (!receive(CMD_GET_BATTERY_VOLTAGE, response) ||
      response.size() < 2) {
    return false;
  }

  voltage = (response[1] * 256 + response[0]) / 1000.0;

  return true;
}

}  // namespace xarm_controller_hardware
