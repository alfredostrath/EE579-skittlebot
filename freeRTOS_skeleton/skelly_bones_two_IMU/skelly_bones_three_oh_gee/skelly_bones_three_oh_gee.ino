//Libraries:
#include "CytronMotorDriver.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <vl53l5cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

// //Pin declarations
 #define I2C_SDA 1
 #define I2C_SCL 2
 #define I2C_RST_PIN 4
// #define PWREN_PIN 5
// #define LPN_PIN 6
 #define DC1_PIN 14
 #define DC2_PIN 15
 #define SERVO_PIN 40

// //Constants:
// #define refelction_threshold 50
// #define distance_threshold 200

//taskHandlers
static TaskHandle_t driveHandler = NULL;
static TaskHandle_t shortTOFHandler = NULL;
static TaskHandle_t longTOFHandler = NULL;
static TaskHandle_t turnHandler = NULL;
static TaskHandle_t sensorHandler = NULL;

uint initialDriveTime = 6000;
uint lengthTime = 10000;
uint turnTime = 1000;
bool validDetection = false;
float IMUerror=0.0;
float heading =0.0;
float headingDEG=0.0;
int turnAngle=30;

bool goingToCan = false;

CytronMD DCmotor(PWM_PWM,DC1_PIN,DC2_PIN);
Adafruit_MPU6050 mpu;
Servo steerServo;


void setup() {                  //STRICTLY SETUP CODE

  Serial.begin(115200);

  Wire.begin(I2C_SDA,I2C_SCL);
  //MPU SETUP

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      Serial.println("Searching for MPU");
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  delay(500);


  //Calibration Step                      
  for (int i=0; i<20; i++){
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp); 
    Serial.print("Calibration reading:");
    Serial.println(g.gyro.x);
    Serial.println(i);
    delay(100);
  
    IMUerror=IMUerror + g.gyro.x;
     Serial.print("IMU ERROR: ");
    Serial.println(IMUerror);
  }
 
  IMUerror=IMUerror/20;

  Serial.print("IMU ERROR: ");
  Serial.println(IMUerror);
  delay(2000);
  //STEER SERVO SETUP

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steerServo.setPeriodHertz(50);    // standard 50 hz servo
  steerServo.attach(SERVO_PIN, 1000, 2000);


  //TaskCreation
  xTaskCreatePinnedToCore(  driveManager,       //Function Name
                "drivingManager",     //Task Name
                20000,         //Stack size
                NULL,         //Task priority 
                2,
                &driveHandler,
                0);      //Task handle

  xTaskCreate(  shortTOF,       //Function Name
                "shortTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                &shortTOFHandler);      //Task handle

                
  xTaskCreate(  longTOF,       //Function Name
                "longTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                &longTOFHandler);      //Task handle


  xTaskCreatePinnedToCore(  turn,       //Function Name
                "turn",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                &turnHandler,
                0);      //Task handle

  // xTaskCreate(  sensorControl,       //Function Name
  //               "sensorControl",     //Task Name
  //               20000,         //Stack size
  //               NULL,         //Task priority 
  //               2,
  //               &sensorHandler);      //Task handle

  xTaskCreatePinnedToCore(  taskManager,       //Function Name
                "task manager",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                NULL,
                0);      //Task handle
}

void taskManager(void * pvParameters){
  // WHILE LOOP TO POLL FOR START SIGNAL 

  //Initial Drive
  xTaskNotifyGive(driveHandler);

  delay(initialDriveTime);

  xTaskNotifyGive(shortTOFHandler);
  xTaskNotifyGive(longTOFHandler);
  delay(lengthTime);


  //turn
  vTaskSuspend(shortTOFHandler);
  vTaskSuspend(longTOFHandler);
  
  xTaskNotifyGive(turnHandler);
  delay(turnTime);

  //Start new length
  vTaskResume(driveHandler);
  vTaskResume(shortTOFHandler);
  vTaskResume(longTOFHandler);


  //Restarts the driving for a search

  // xTaskNotifyGive(turnHandler);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);      //Delay for turning
  delay(10000);
}


void driveManager(void * pvParameters){
  int Kp = 1;
  int servoAngle=45;
  float err =0.0;
  while(1){
    Serial.println("Driving"); 
    DCmotor.setSpeed(255);

    //Sample IMU
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);  
    Serial.print("Measure:");
    Serial.println(g.gyro.x);                       
    heading=(heading+(IMUerror-g.gyro.x)*0.1);
    Serial.print("Heading:");
    headingDEG = heading *(180/3.14159);
    Serial.println(headingDEG);

    if (validDetection==true){
        //PID - detection
    }
    else
    {
      //IMU PID Loop
      err = headingDEG;
      Serial.print("Error: ");
      Serial.println(err);
      servoAngle = 45-err*Kp;
      Serial.println("Servo angle:");
      Serial.println(servoAngle);
      steerServo.write(servoAngle);
    }
    delay(100);
  }
}



void turn(void * pvParameters){
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    if (goingToCan == false){
      steerServo.write(turnAngle);
      Serial.println("Turn"); 
    }
    vTaskSuspend(NULL);

  }
}

void shortTOF(void * pvParameters){
  //Code before here should run on startup?
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while(1){
    //sample sensor
    Serial.println("Short TOF sent"); 
    delay(100);
  }
    
  
}

void longTOF(void * pvParameters){
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    //sample sensor
      Serial.println("Long TOF sent"); 
      delay(500);
  }

  }


void loop() {
}
