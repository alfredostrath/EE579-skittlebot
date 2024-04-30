#include "customLib.h"
#include "CytronMotorDriver.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <ESP32Servo.h>


void mpuSetup(Adafruit_MPU6050 mpu)
{
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("MPU FOUND");
  delay(5000);
}

float mpuSample(Adafruit_MPU6050 mpu)
{
   sensors_event_t a,g,temp;
   mpu.getEvent(&a,&g,&temp);
   Serial.print("Reading from lib:");
   Serial.println(g.gyro.x);
   return g.gyro.x;
}