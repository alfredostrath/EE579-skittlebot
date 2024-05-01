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
// #define I2C_SDA 1
// #define I2C_SCL 2
// #define I2C_RST_PIN 4
// #define PWREN_PIN 5
// #define LPN_PIN 6
// #define DC1_PIN 14
// #define DC2_PIN 15
// #define SERVO_PIN 40

// //Constants:
// #define refelction_threshold 50
// #define distance_threshold 200

//taskHandlers
static TaskHandle_t driveHandler = NULL;
static TaskHandle_t shortTOFHandler = NULL;
static TaskHandle_t longTOFHandler = NULL;
static TaskHandle_t turnHandler = NULL;
static TaskHandle_t sensorHandler = NULL;

uint initialTime = 6000;
uint lengthTime = 5000;
uint turnTime = 1000;
bool validDetection = False;

void setup() {                  //STRICTLY SETUP CODE
  Serial.begin(115200);


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

  delay(5000);

  vTaskSuspend(driveHandler);


  //loop
    //drive for length
    //turn


  xTaskNotifyGive(shortTOFHandler);
  xTaskNotifyGive(longTOFHandler);
  delay(lengthTime);


  vTaskSuspend(shortTOFHandler);
  vTaskSuspend(longTOFHandler);
  
  xTaskNotifyGive(turnHandler);
  delay(turnTime);

  //Start new length
  vTaskResume(driveHandler);
  vTaskResume(shortTOFHandler);
  vTaskResume(longTOFHandler);
  vTaskResume( )


  
  //Restarts the driving for a search

  // xTaskNotifyGive(turnHandler);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);      //Delay for turning
  delay(10000);
}


void driveManager(void * pvParameters){
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    Serial.println("Driving"); 
    //Set motor PWM

    if (validDetection==True){
        //PID - detection
    }
    else
    {
        //PID - IMU 
    }
    delay(100);
  }
}



void turn(void * pvParameters){
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while(1){
      Serial.println("Turn"); 
      vTaskSuspend(NULL);
  }
}

void shortTOF(void * pvParameters){
  //Code before here should run on startup?
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while(1){
    //sample sensor
    Serial.println("Short TOF sent"); 
    delay(200);
  }
    
  
}

void longTOF(void * pvParameters){
  
  //Code before here should run on startup?
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while(1){
    //sample sensor
      Serial.println("Long TOF sent"); 
      delay(1000);
  }
}

void loop() {
}
