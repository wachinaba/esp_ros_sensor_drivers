#include <ros.h>

#include "PMW3901.h"

#include <optical_flow_msgs/OpticalFlowDelta.h>
#include <sensor_msgs/Range.h>

// Using digital pin 26 as chip select, but it can be any pin
PMW3901 flow(26);
int image_width = 4;
int image_resolution = image_width * image_width;

ros::NodeHandle nh;
optical_flow_msgs::OpticalFlowDelta flow_msg;
ros::Publisher flow_pub("optical_flow_delta", &flow_msg);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(flow_pub);

  if (!flow.begin())
  {
    Serial.println("Initialization of the flow sensor failed");
    while (1)
    {
    }
  }

  flow.setLed(true);
}

int16_t deltaX, deltaY;
uint8_t squal;

int last_published_time = millis();
int publish_interval = 1000 / 20; // 20 Hz

void loop()
{
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY, &squal);

  flow_msg.header.stamp = nh.now();
  flow_msg.header.frame_id = "flow_frame";
  flow_msg.integration_time_us = 0;
  flow_msg.delta_px = float(deltaX);
  flow_msg.delta_py = float(deltaY);
  flow_msg.surface_quality = squal;

  while (true)
  {
    if (abs((int)millis() - last_published_time) > publish_interval)
    {
      flow_pub.publish(&flow_msg);
      nh.spinOnce();
      last_published_time += publish_interval;
      break;
    } else {
      delay(1);
    }
  }
  
}