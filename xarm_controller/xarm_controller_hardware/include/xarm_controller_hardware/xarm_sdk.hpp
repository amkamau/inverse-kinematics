
#ifndef XARM_CONTROLLER_HARDWARE__XARM_SDK_HPP_
#define XARM_CONTROLLER_HARDWARE__XARM_SDK_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hidapi/hidapi.h>

namespace xarm_controller_hardware
{

struct Servo
{
  int servo_id;
  int position;

  Servo(int id, int pos = 500);

  double angle() const;
  void setAngle(double degrees);
};

class XArmSDK
{
public:
  static constexpr uint8_t SIGNATURE = 0x55;
  static constexpr uint8_t CMD_SERVO_MOVE = 0x03;
  static constexpr uint8_t CMD_GET_BATTERY_VOLTAGE = 0x0F;
  static constexpr uint8_t CMD_SERVO_STOP = 0x14;
  static constexpr uint8_t CMD_GET_SERVO_POSITION = 0x15;

  explicit XArmSDK(bool debug = false);
  ~XArmSDK();

  XArmSDK(const XArmSDK &) = delete;
  XArmSDK & operator=(const XArmSDK &) = delete;

  bool connect(const std::wstring & serial_number = L"");
  void disconnect();
  bool isConnected() const;

  bool setPosition(int servo_id, int position,
                   int duration = 1000);

  bool setPositionDegrees(int servo_id, double degrees,
                          int duration = 1000);

  bool setPositions(const std::vector<Servo> & servos,
                    int duration = 1000);

  bool getPosition(int servo_id, int & position);
  bool getPositions(std::vector<Servo> & servos);

  bool servoOff(const std::vector<int> & servo_ids = {});

  bool getBatteryVoltage(double & voltage);

  static int angleToPosition(double degrees);
  static double positionToAngle(int position);

private:
  bool send(uint8_t command, const std::vector<uint8_t> & data = {});
  bool receive(uint8_t command, std::vector<uint8_t> & data);

  hid_device * device_{nullptr};
  bool debug_{false};

};

}  // namespace xarm_controller_hardware

#endif
