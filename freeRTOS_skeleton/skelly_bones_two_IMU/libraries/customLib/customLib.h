#ifndef CUSTOMLIB_H
#define CUSTOMLIB_H
#include <Arduino.h>
#include <Adafruit_MPU6050.h>

void mpuSetup(Adafruit_MPU6050 mpu);
float mpuSample(Adafruit_MPU6050 mpu);
#endif