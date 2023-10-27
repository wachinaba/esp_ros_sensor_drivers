#include "VL53Manager.h"
#include <PMW3901.h>
#include <Wire.h>
#include <ros.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

#define BAUDRATE 230400
#define I2C_FREQ 1000000

SPIClass vspi(VSPI);

std::vector<uint8_t> sensor_addresses_wire0{0x40, 0x41};
std::vector<uint8_t> sensor_addresses_wire1{0x42, 0x43};
std::vector<uint8_t> sensor_reset_pins{32, 33};

std::vector<uint8_t> flow_sensor_cs_pins{2, 4};

VL53Manager vl53_manager_wire0(&Wire, sensor_addresses_wire0, sensor_reset_pins);
VL53Manager vl53_manager_wire1(&Wire1, sensor_addresses_wire1, sensor_reset_pins);
VL53L5CX_ResultsData measurement_data = VL53L5CX_ResultsData();

std::vector<PMW3901> flow_sensors{PMW3901(2, &vspi), PMW3901(4, &vspi)};
std::vector<bool> flow_sensor_ready{false, false};

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

  for (uint8_t i = 0; i < flow_sensors.size(); i++)
  {
    for (uint8_t j = 0; j < flow_sensors.size(); j++)
    {
      digitalWrite(flow_sensor_cs_pins[j], HIGH);
    }

    delay(100);
    digitalWrite(flow_sensor_cs_pins[i], LOW);
    delay(1);

    flow_sensor_ready[i] = flow_sensors[i].begin();
    if (flow_sensor_ready[i])
    {
      Serial.println("Flow sensor " + String(i) + " initialized");
      flow_sensors[i].setLed(true);
    }
    else
    {
      Serial.println("Flow sensor " + String(i) + " initialization failed");
    }
  }

  for (uint8_t i = 0; i < flow_sensors.size(); i++)
  {
    digitalWrite(flow_sensor_cs_pins[i], HIGH);
  }

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

  for (uint8_t i = 0; i < flow_sensors.size(); i++)
  {
    if (!flow_sensor_ready[i])
    {
      continue;
    }

    int16_t deltaX, deltaY;
    uint8_t squal;

    digitalWrite(flow_sensor_cs_pins[i], LOW);
    delay(1);

    flow_sensors[i].readMotionCount(&deltaX, &deltaY, &squal);

    digitalWrite(flow_sensor_cs_pins[i], HIGH);
    delay(1);

    Serial.print("Flow sensor " + String(i) + " delta x: ");
    Serial.print(deltaX);
    Serial.print(" delta y: ");
    Serial.print(deltaY);
    Serial.print(" squal: ");
    Serial.println(squal);
  }

  delay(1);
}
