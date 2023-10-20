/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to setup and read two sensors. We will hold one
  sensor in reset while we configure the first. You will need to solder
  a wire to each of the sensor's RST pins and connect them to GPIO 14 and 13
  on your plateform.

  Note: The I2C address for the device is stored in NVM so it will have to be
  set at each power on.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Wire.h>
#include <ros.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

SparkFun_VL53L5CX myImager1;
int sensorAddress1 =
    0x44;              // New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset1 = 32; // GPIO that is connected to the Reset pin on sensor 1
VL53L5CX_ResultsData measurementData1;

SparkFun_VL53L5CX myImager2;
int sensorAddress2 = 0x29; // Default VL53L5CX - this is the unit we'll hold in
                           // reset (has the wire soldered)
int sensorReset2 = 33;     // GPIO that is connected to the Reset pin on sensor 2
VL53L5CX_ResultsData measurementData2;

ros::NodeHandle nh;
multizone_lidar_msgs::MultizoneRange range_array[2];
ros::Publisher range_array_pub[2] = {
    ros::Publisher("/multizone_range/0", &range_array[0]),
    ros::Publisher("/multizone_range/1", &range_array[1])};

int image_width = 8;
int image_resolution = image_width * image_width;

float horizontal_fov = 45.0 / 180.0 * M_PI;
float vertical_fov = horizontal_fov;

float horizontal_increment = horizontal_fov / image_width;
float vertical_increment = vertical_fov / image_width;

std::string frame_id = "lidar_frame";

void set_data(VL53L5CX_ResultsData &measurement_data, multizone_lidar_msgs::MultizoneRange &range_array)
{
  for (int i = 0; i < image_resolution; i++)
  {
    float distance = measurement_data.distance_mm[i] / 1000.0;
    range_array.ranges[i] = distance;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(21, 22, 1000000); // SDA, SCL, I2C bus speed. Default is 100000

  pinMode(sensorReset1, OUTPUT);
  pinMode(sensorReset2, OUTPUT);

  digitalWrite(sensorReset2,
               HIGH);               // Hold sensor 2 in reset while we configure sensor 1
  digitalWrite(sensorReset1, HIGH); // Reset sensor 1
  delay(100);
  digitalWrite(
      sensorReset1,
      LOW); // Sensor 1 should now be available at default address 0x29

  Serial.println(
      F("Initializing sensor 1. This can take up to 10s. Please wait."));

  if (myImager1.begin() == false)
  { // I2C_RST does not reset the I2C address,
    // so we have to try both addresses
    Serial.println(
        F("Sensor 1 not found at default address. Trying again at alternate "
          "address."));

    if (myImager1.begin(sensorAddress1) == false)
    {
      Serial.println(F("Sensor 1 not found at alternate address. Freezing..."));
      while (1)
        ;
    }
  }

  Serial.print(F("Setting sensor 1 address to: 0x"));
  Serial.println(sensorAddress1, HEX);

  if (myImager1.setAddress(sensorAddress1) == false)
  {
    Serial.println(
        F("Sensor 1 failed to set new address. Please try again. Freezing..."));
    while (1)
      ;
  }

  int newAddress = myImager1.getAddress();

  Serial.print(F("New address of sensor 1 is: 0x"));
  Serial.println(newAddress, HEX);

  digitalWrite(sensorReset2, LOW); // Release sensor 2 from reset

  Serial.println(
      F("Initializing sensor 2. This can take up to 10s. Please wait."));
  if (myImager2.begin() == false)
  {
    Serial.println(F("Sensor 2 not found. Check wiring. Freezing..."));
    while (1)
      ;
  }

  // Configure both sensors the same just to keep things clean
  myImager1.setResolution(8 * 8); // Enable all 64 pads
  myImager2.setResolution(8 * 8); // Enable all 64 pads

  imageResolution =
      myImager1.getResolution();      // Query sensor for current resolution -
                                      // either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); // Calculate printing width

  myImager1.setRangingFrequency(15);
  myImager2.setRangingFrequency(15);

  myImager1.startRanging();
  myImager2.startRanging();

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  for (int i = 0; i < 2; i++)
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
  // Poll sensor for new data
  if (myImager1.isDataReady() == true)
  {
    if (myImager1.getRangingData(&measurementData1)) // Read distance data into array
    {
      set_data(measurementData1, range_array[0]);
      range_array[0].header.stamp = nh.now();
      range_array_pub[0].publish(&range_array[0]);
    }
  }
  if (myImager2.isDataReady() == true)
  {
    if (myImager2.getRangingData(&measurementData2)) // Read distance data into array
    {
      set_data(measurementData2, range_array[1]);
      range_array[1].header.stamp = nh.now();
      range_array_pub[1].publish(&range_array[1]);
    }
  }

  nh.spinOnce();

  delay(5); // Small delay between polling
}