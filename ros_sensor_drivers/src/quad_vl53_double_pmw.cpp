#include "VL53Manager.h"
#include <PMW3901.h>
#include <Wire.h>
#include <ros.h>
#include <multizone_lidar_msgs/MultizoneRange.h>
#include <optical_flow_msgs/OpticalFlowDelta.h>

#define BAUDRATE 230400
#define I2C_FREQ 1000000

SPIClass vspi(VSPI);

int vl53_horizontal_samples = 8;
int vl53_vertical_samples = 8;
int vl53_image_resolution = vl53_horizontal_samples * vl53_vertical_samples;
int vl53_frequency = 15;

int pmw_publish_interval = 1000 / 20; // 20 Hz

std::vector<uint8_t> sensor_addresses_wire0{0x40, 0x41};
std::vector<uint8_t> sensor_addresses_wire1{0x42, 0x43};
std::vector<uint8_t> sensor_reset_pins{32, 33};
int range_sensors_num = sensor_addresses_wire0.size() + sensor_addresses_wire1.size();

std::vector<uint8_t> flow_sensor_cs_pins{2, 4};
int flow_sensors_num = flow_sensor_cs_pins.size();

VL53Manager vl53_manager_wire0(&Wire, sensor_addresses_wire0, sensor_reset_pins);
VL53Manager vl53_manager_wire1(&Wire1, sensor_addresses_wire1, sensor_reset_pins);
VL53L5CX_ResultsData measurement_data = VL53L5CX_ResultsData();

std::vector<PMW3901> flow_sensors{PMW3901(2, &vspi), PMW3901(4, &vspi)};
std::vector<bool> flow_sensor_ready{false, false};

ros::NodeHandle nh;
multizone_lidar_msgs::MultizoneRange range_arrays[] = {multizone_lidar_msgs::MultizoneRange(),
                                                       multizone_lidar_msgs::MultizoneRange(),
                                                       multizone_lidar_msgs::MultizoneRange(),
                                                       multizone_lidar_msgs::MultizoneRange()};
ros::Publisher range_array_pub[] = {
    ros::Publisher("/multizone_range/0", &(range_arrays[0])),
    ros::Publisher("/multizone_range/1", &(range_arrays[1])),
    ros::Publisher("/multizone_range/2", &(range_arrays[2])),
    ros::Publisher("/multizone_range/3", &(range_arrays[3]))};
std::vector<bool> vl53_updated{false, false, false, false};

optical_flow_msgs::OpticalFlowDelta flow_deltas[] =
    {optical_flow_msgs::OpticalFlowDelta(),
     optical_flow_msgs::OpticalFlowDelta()};
ros::Publisher flow_pub[] = {
    ros::Publisher("/optical_flow_delta/0", &(flow_deltas[0])),
    ros::Publisher("/optical_flow_delta/1", &(flow_deltas[1]))};
std::vector<bool> flow_updated{false, false};

void vl53UpdateTask(void *pvParameters)
{
  std::vector<VL53L5CXSensor> sensors(4, VL53L5CXSensor());
  while (1)
  {
    sensors = vl53_manager_wire0.getSensors();
    auto wire1_sensors = vl53_manager_wire1.getSensors();
    sensors.insert(sensors.end(), wire1_sensors.begin(), wire1_sensors.end());

    for (int i = 0; i < sensors.size(); i++)
    {
      auto sensor = sensors[i];
      auto imager = sensor.imager;

      if (!sensor.ready)
      {
        continue;
      }

      if (imager.isDataReady())
      {
        for (int j = 0; j < vl53_image_resolution; j++)
        {
          // float distance = measurement_data.distance_mm[j] / 1000.0;
          // range_arrays[i].ranges[j] = distance;
        }
        range_arrays[i].header.stamp = nh.now();
        vl53_updated[i] = true;
      }
    }

    delay(3);
  }
}

void flowUpdateTask(void *pvParameters)
{
  unsigned long last_published_time = millis();
  int16_t deltaX, deltaY;
  uint8_t squal;
  std::vector<unsigned long> last_updated_us(flow_sensors.size(), micros());

  while (1)
  {
    for (uint8_t i = 0; i < flow_sensors.size(); i++)
    {
      if (!flow_sensor_ready[i])
      {
        continue;
      }

      flow_sensors[i].readMotionCount(&deltaX, &deltaY, &squal);
      flow_deltas[i].delta_px = float(deltaX);
      flow_deltas[i].delta_py = float(deltaY);
      flow_deltas[i].surface_quality = squal;
      flow_deltas[i].integration_time_us = abs((int)micros() - (int)last_updated_us[i]);
      flow_deltas[i].header.stamp = nh.now();
      flow_deltas[i].header.frame_id = "";

      flow_updated[i] = true;
    }

    while (true)
    {
      if (abs((int)millis() - (int)last_published_time) > pmw_publish_interval)
      {
        last_published_time += pmw_publish_interval;
        break;
      }
      else
      {
        delay(1);
      }
    }
  }
}

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

  for (auto range_array : range_arrays)
  {
    range_array.ranges = (float *)malloc(vl53_image_resolution * sizeof(float));
    range_array.ranges_length = vl53_image_resolution;
    range_array.header.frame_id = "";
    range_array.horizontal_fov = 45.0 / 180.0 * M_PI;
    range_array.vertical_fov = 45.0 / 180.0 * M_PI;
    range_array.horizontal_samples = vl53_horizontal_samples;
    range_array.vertical_samples = vl53_vertical_samples;
    range_array.min_range = 0.1;
    range_array.max_range = 4.0;
  }

  for (uint8_t i = 0; i < flow_sensors.size(); i++)
  {
    pinMode(flow_sensor_cs_pins[i], OUTPUT);
    digitalWrite(flow_sensor_cs_pins[i], HIGH);
  }

  delay(100);

  for (uint8_t i = 0; i < flow_sensors.size(); i++)
  {
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

    delay(100);
  }

  Serial.println("PMW3901 initialized");

  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();

  for (int i = 0; i < range_sensors_num; i++)
  {
    nh.advertise(range_array_pub[i]);
  }

  for (int i = 0; i < flow_sensors_num; i++)
  {
    nh.advertise(flow_pub[i]);
  }

  xTaskCreate(vl53UpdateTask, "vl53UpdateTask", 8192, NULL, 1, NULL);
  xTaskCreate(flowUpdateTask, "flowUpdateTask", 8192, NULL, 1, NULL);
}

void loop()
{
  for (uint8_t i = 0; i < range_sensors_num; i++)
  {
    if (vl53_updated[i])
    {
      range_array_pub[i].publish(&(range_arrays[i]));
      vl53_updated[i] = false;
    }
  }

  for (uint8_t i = 0; i < flow_sensors_num; i++)
  {
    if (flow_updated[i])
    {
      flow_pub[i].publish(&(flow_deltas[i]));
      flow_updated[i] = false;
    }
  }

  nh.spinOnce();
  delay(1);
}
