#include <Arduino.h>
#include <M5Unified.h>
#include <vector>
#include "LiDAR/LiDAR.hpp"

LiDAR lidar(Serial1);

void setup(void)
{
  auto cfg = M5.config();
  M5.begin(cfg);
  // ディスプレイ設定
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);
  Serial.begin(115200);

  lidar.begin(32, 33);

  delay(10);

  Serial.printf("Start\n");
}

void loop()
{
  M5.update();
  lidar.update();
}
