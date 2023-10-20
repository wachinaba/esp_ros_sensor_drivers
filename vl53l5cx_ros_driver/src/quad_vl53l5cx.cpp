#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Wire.h>
#include <ros.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

#define BAUDRATE 115200
#define I2C_FREQ 1000000

SparkFun_VL53L5CX imager[4] = {
    SparkFun_VL53L5CX(),
    SparkFun_VL53L5CX(),
    SparkFun_VL53L5CX(),
    SparkFun_VL53L5CX()};
int sensor_address[4] = {0x40, 0x41, 0x42, 0x43};
bool sensor_ready[4] = {false, false, false, false};
int sensor_reset[4] = {32, 33, 2, 4};
TwoWire *wire[4] = {&Wire, &Wire, &Wire1, &Wire1};

VL53L5CX_ResultsData measurement_data[4] = {
    VL53L5CX_ResultsData(),
    VL53L5CX_ResultsData(),
    VL53L5CX_ResultsData(),
    VL53L5CX_ResultsData()};

ros::NodeHandle nh;
multizone_lidar_msgs::MultizoneRange range_array[4];
ros::Publisher range_array_pub[4] = {
    ros::Publisher("/multizone_range/0", &range_array[0]),
    ros::Publisher("/multizone_range/1", &range_array[1]),
    ros::Publisher("/multizone_range/2", &range_array[2]),
    ros::Publisher("/multizone_range/3", &range_array[3])};

bool updated[4] = {false, false, false, false};
bool wire_ready[2] = {false, false};

constexpr int image_width = 8;
constexpr int image_resolution = image_width * image_width;

constexpr float horizontal_fov = 45.0 / 180.0 * M_PI;
constexpr float vertical_fov = horizontal_fov;

constexpr float horizontal_increment = horizontal_fov / image_width;
constexpr float vertical_increment = vertical_fov / image_width;

std::string frame_id = "lidar_frame";

void wire0_task(void *pvParameters)
{
  while (1)
  {

    delay(5);
  }
}

void setup()
{
  Serial.begin(BAUDRATE);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(21, 22, I2C_FREQ);
  Wire1.begin(25, 26, I2C_FREQ);
  for (int i = 0; i < 4; i++)
  {
    pinMode(sensor_reset[i], OUTPUT);
    digitalWrite(sensor_reset[i], HIGH);
  }
  delay(100);

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(sensor_reset[i], LOW);
    delay(100);
    Serial.println("Initializing sensor " + String(i));
    if (imager[i].begin(0x29, *(wire[i])) == false)
    {
      Serial.println("Sensor " + String(i) + " not found at default address. Trying again at alternate address.");
      for (int j = 0; j < 4; j++)
      {
        if (sensor_ready[j])
        {
          continue;
        }
        if (imager[i].begin(sensor_address[j], *(wire[i])) == true)
        {
          Serial.println("Sensor " + String(i) + " found at alternate address " + String(sensor_address[j]));
          sensor_ready[i] = true;
          break;
        }
      }
    }
    else
    {
      Serial.println("Sensor " + String(i) + " found at default address");
      sensor_ready[i] = true;
    }

    if (!sensor_ready[i])
    {
      Serial.println("Sensor " + String(i) + " not found. Freezing...");
      while (1)
      {
        delay(1);
      }
    }

    if (imager[i].setAddress(sensor_address[i]) == false)
    {
      Serial.println("Sensor " + String(i) + " address change failed. Freezing...");
      while (1)
      {
        delay(1);
      }
    }

    delay(100);

    Serial.print("Sensor " + String(i) + " ready, address: ");
    Serial.println(imager[i].getAddress(), HEX);

    Serial.println("Setting sensor " + String(i) + " resolution to " + String(image_resolution));
    imager[i].setResolution(image_resolution);

    Serial.println("Setting sensor " + String(i) + " ranging frequency to 15");
    imager[i].setRangingFrequency(15);

    delay(100);
  }

  for (int i = 0; i < 4; i++)
  {
    Serial.println("Starting sensor " + String(i) + " ranging");
    imager[i].startRanging();
  }

  delay(1000);

  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  for (int i = 0; i < 4; i++)
  {
    nh.advertise(range_array_pub[i]);
    range_array[i].ranges = (float *)malloc(sizeof(float) * image_resolution);
    range_array[i].ranges_length = image_resolution;
    range_array[i].header.frame_id = frame_id.c_str();
    range_array[i].horizontal_fov = horizontal_fov;
    range_array[i].vertical_fov = vertical_fov;
    range_array[i].horizontal_samples = image_width;
    range_array[i].vertical_samples = image_width;
    range_array[i].min_range = 0.1;
    range_array[i].max_range = 4.0;
  }
}

void loop()
{
  for (int i = 0; i < 4; i++)
  {
    for (int i = 0; i < 4; i++)
    {
      // Serial.println("Checking sensor " + String(i) + " data ready");
      if (imager[i].isDataReady())
      {
        // Serial.println("Sensor " + String(i) + " data ready");
        if (imager[i].getRangingData(&measurement_data[i]))
        {
          // Serial.println("Sensor " + String(i) + " data read");
          for (int j = 0; j < image_resolution; j++)
          {
            float distance = measurement_data[i].distance_mm[j] / 1000.0;
            range_array[i].ranges[j] = distance;
          }
          range_array[i].header.stamp = nh.now();
          updated[i] = true;
        }
      }
    }
    if (updated[i])
    {
      range_array_pub[i].publish(&range_array[i]);
      updated[i] = false;
    }
  }
  nh.spinOnce();
  delay(3);
}