#include <Arduino.h>
#include "MPU9250.h"
#include <Wire.h>

// Magnetometer Data in µTesla: 
// 1713.12   2946.20   954.26
// Magnetometer Data in µTesla: 
// -3798.05   -880.85   -1954.26

MPU9250 Imu = MPU9250();

void setup() {
  Serial.begin(115200);

  if(!Wire.begin()){
    Serial.println("Wire init error");
  } else {
    if (!Imu.begin()) {
      Serial.println("Imu init error");
    } else {
      if (!Imu.initMagnetometer()) {
        Serial.println("Magnetometer init error");
      }
    }
  }
}

void loop() {
  Imu.readSensor();
  xyzFloat magValue = Imu.getMagValues();

  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);

  delay(1000);
}