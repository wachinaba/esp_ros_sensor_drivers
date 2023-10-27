#include "VL53Manager.h"

VL53Manager::VL53Manager(TwoWire *wire, std::vector<uint8_t> sensor_addresses, std::vector<uint8_t> sensor_reset_pins) : wire_(wire), sensor_addresses_(sensor_addresses), sensor_reset_pins_(sensor_reset_pins)
{
  num_sensors_ = sensor_addresses_.size();
  sensors_ = std::vector<VL53L5CXSensor>(num_sensors_);
}

VL53Manager::~VL53Manager()
{
}

void VL53Manager::resetSensorAddresses()
{
  // initialize reset pins
  for (auto reset_pin : sensor_reset_pins_)
  {
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
  }

  // search sensors
  for (int i = 0; i < sensor_addresses_.size(); i++)
  {
    digitalWrite(sensor_reset_pins_[i], LOW);
    delay(100);

    // search for sensor addr 8~119
    bool sensor_found = false;
    for (uint8_t address = 8; address <= 119; address++)
    {
      if (sensors_[i].imager.begin(address, *(wire_)) == true)
      {
        Serial.println("Sensor " + String(i) + " found at address " + String(address));
        sensor_found = true;
        break;
      }
    }

    if (!sensor_found)
    {
      Serial.println("Sensor " + String(i) + " not found. skipping...");
      continue;
    }

    // set sensor address
    if (sensors_[i].imager.setAddress(sensor_addresses_[i]) == true)
    {
      Serial.println("Sensor " + String(i) + " address set to " + String(sensor_addresses_[i]));
    }
    else
    {
      Serial.println("Sensor " + String(i) + " address set failed. freezing...");
      while (1)
      {
        delay(1);
      }
      continue;
    }
    delay(100);

    digitalWrite(sensor_reset_pins_[i], HIGH); // reset sensor i2c bus
  }
  delay(100);
}

void VL53Manager::beginSensors()
{
  // begin sensors
  for (uint8_t i = 0; i < sensor_addresses_.size(); i++)
  {
    digitalWrite(sensor_reset_pins_[i], LOW);

    if (sensors_[i].imager.begin(sensor_addresses_[i]) == true)
    {
      Serial.println("Sensor " + String(i) + " ready, address: " + String(sensors_[i].imager.getAddress(), HEX));
      sensors_[i].ready = true;
    }
    else
    {
      Serial.println("Sensor " + String(i) + " begin failed. skipping...");
      continue;
    }

    // set sensor resolution
    if (sensors_[i].imager.setResolution(8 * 8) == true)
    {
      Serial.println("Sensor " + String(i) + " resolution set to " + String(8 * 8));
    }
    else
    {
      Serial.println("Sensor " + String(i) + " resolution set failed. skipping...");
    }

    // set sensor frequency
    if (sensors_[i].imager.setRangingFrequency(15) == true)
    {
      Serial.println("Sensor " + String(i) + " ranging frequency set to 15");
    }
    else
    {
      Serial.println("Sensor " + String(i) + " ranging frequency set failed. skipping...");
    }

    // start ranging
    sensors_[i].imager.startRanging();

    delay(100);
  }
}
