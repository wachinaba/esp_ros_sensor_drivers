#ifndef __VL53_ROS_H__
#define __VL53_ROS_H__

#include <Wire.h>
#include <vector>
#include <SparkFun_VL53L5CX_Library.h>

struct
{
  SparkFun_VL53L5CX imager;
  bool ready;
} typedef VL53L5CXSensor;

class VL53Manager
{
public:
  VL53Manager(TwoWire *wire, std::vector<uint8_t> sensor_addresses, std::vector<uint8_t> sensor_reset_pins);
  ~VL53Manager();

  void begin();
  std::vector<VL53L5CXSensor> getSensors() { return sensors_; }

private:
  TwoWire *wire_;
  std::vector<VL53L5CXSensor> sensors_;
  std::vector<uint8_t> sensor_addresses_;
  std::vector<uint8_t> sensor_reset_pins_;
  uint8_t num_sensors_;
};

#endif // __VL53_ROS_H__