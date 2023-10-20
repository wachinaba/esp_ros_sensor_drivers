#include <optical_flow_msgs/OpticalFlowDelta.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

#include "PMW3901.h"

SPIClass vspi(VSPI);
SPIClass hspi(HSPI);

PMW3901 flow[] = {
    PMW3901(5, &vspi),
    PMW3901(15, &hspi),
};

#define SENSOR_COUNT 2

int image_width = 4;
int image_resolution = image_width * image_width;

ros::NodeHandle nh;

optical_flow_msgs::OpticalFlowDelta flow_msg[2];
ros::Publisher flow_pub[] = {
    ros::Publisher("/optical_flow_delta/0", &flow_msg[0]),
    ros::Publisher("/optical_flow_delta/1", &flow_msg[1]),
};

bool updated = false;

void flow_update_task(void *pvParameters) {
  int16_t deltaX, deltaY;
  uint8_t squal;

  int last_published_time = millis();
  int publish_interval = 1000 / 20;  // 20 Hz

  while (true) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      // Get motion count since last call
      flow[i].readMotionCount(&deltaX, &deltaY, &squal);

      flow_msg[i].header.stamp = nh.now();
      flow_msg[i].header.frame_id = "flow_frame";
      flow_msg[i].integration_time_us = 0;
      flow_msg[i].delta_px = float(deltaX);
      flow_msg[i].delta_py = float(deltaY);
      flow_msg[i].surface_quality = squal;
    }
    updated = true;

    while (true) {
      if (abs((int)millis() - last_published_time) > publish_interval) {
        last_published_time += publish_interval;
        break;
      } else {
        delay(1);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!flow[i].begin()) {
      Serial.println("Initialization of the flow sensor No." + String(i) +
                     " failed!");
      while (1) {
      }
    }
    nh.advertise(flow_pub[i]);
    flow[i].setLed(true);
  }

  xTaskCreatePinnedToCore(flow_update_task, "flow_update_task", 4096, NULL, 1,
                          NULL, 1);
}

void loop() {
  if (updated) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      flow_pub[i].publish(&flow_msg[i]);
    }
    updated = false;
  }
  nh.spinOnce();
  delay(1);
}