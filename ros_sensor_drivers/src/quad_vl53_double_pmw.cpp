#include "VL53Manager.h"
#include <Wire.h>
#include <ros.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

#define BAUDRATE 230400
#define I2C_FREQ 1000000

std::vector<uint8_t> sensor_addresses_wire0{0x40, 0x41};
std::vector<uint8_t> sensor_addresses_wire1{0x42, 0x43};
std::vector<uint8_t> sensor_reset_pins{32, 33};

VL53Manager vl53_manager_wire0(&Wire, sensor_addresses_wire0, sensor_reset_pins);
VL53Manager vl53_manager_wire1(&Wire1, sensor_addresses_wire1, sensor_reset_pins);
VL53L5CX_ResultsData measurement_data = VL53L5CX_ResultsData();

void setup()
{
  Serial.begin(BAUDRATE);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(21, 22, I2C_FREQ);
  Wire1.begin(25, 26, I2C_FREQ);

  vl53_manager_wire0.resetSensorAddresses();
  vl53_manager_wire1.resetSensorAddresses();

  vl53_manager_wire0.beginSensors();
  vl53_manager_wire1.beginSensors();

  Serial.println("VL53L5CX initialized");
}

void loop()
{
  for (auto sensor : vl53_manager_wire0.getSensors())
  {
    SparkFun_VL53L5CX imager = sensor.imager;

    if (!sensor.ready)
    {
      continue;
    }

    if (imager.isDataReady())
    {
      Serial.print("Data ready for sensor address ");
      Serial.println(imager.getAddress(), HEX);
      imager.getRangingData(&measurement_data);
      Serial.println();
    }
  }
  for (auto sensor : vl53_manager_wire1.getSensors())
  {
    SparkFun_VL53L5CX imager = sensor.imager;

    if (!sensor.ready)
    {
      continue;
    }

    if (imager.isDataReady())
    {
      Serial.print("Data ready for sensor address ");
      Serial.println(imager.getAddress(), HEX);
      imager.getRangingData(&measurement_data);
      Serial.println();
    }
  }
  delay(1);
}
