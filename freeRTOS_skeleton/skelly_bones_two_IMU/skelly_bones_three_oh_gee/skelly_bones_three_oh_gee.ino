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

#define SerialPort Serial

// //Pin declarations I2C
 #define I2C_SDA 1
 #define I2C_SCL 2
 #define I2C_RST_PIN 4
// #define PWREN_PIN 5
// #define LPN_PIN 6
 #define DC1_PIN 14
 #define DC2_PIN 15
 #define SERVO_PIN 40

 //srTOF configs
#define vl53l5cx_frequency 1

#define LPN_PIN_1 3
#define I2C_RST_PIN_1 4
#define PWREN_PIN_1 5
#define I2C_Address_1 0x28

#define LPN_PIN_2 7
#define I2C_RST_PIN_2 8
#define PWREN_PIN_2 9
#define I2C_Address_2 0x27

#define refelction_threshold 50
#define distance_threshold 20


//srTof typedef
typedef struct{
    uint16_t target_distance[8][16];
    uint16_t target_angle[8][16];
    uint16_t target_reflectance[8][16];
} Valid_target_Info;

//lrTof typedef
typedef struct{
    uint16_t distance_scan[360];
    uint16_t reflectance_scan[360];
} lrTOFrawData;

//unified Target typedef
typedef struct{
    uint16_t black_angle;
    uint16_t black_distance;
    uint16_t white_angle;
    uint16_t white_distance;
} targets;

//taskHandlers
static TaskHandle_t driveHandler = NULL;
static TaskHandle_t shortTOFHandler = NULL;
static TaskHandle_t longTOFHandler = NULL;
static TaskHandle_t turnHandler = NULL;
static TaskHandle_t sensorHandler = NULL;
static TaskHandle_t shortTOFprocessHandler = NULL;
static TaskHandle_t longTOFprocessHandler = NULL;

QueueHandle_t srTOFrawQ;
QueueHandle_t lrTOFrawQ;
QueueHandle_t srTOFprocQ;
QueueHandle_t lrTOFprocQ;
QueueHandle_t combinedSensorQ;

uint initialDriveTime = 6000;
uint lengthTime = 10000;
uint turnTime = 1000;
bool validDetection = false;
float IMUerror=0.0;
float heading =0.0;
float headingDEG=0.0;
int turnAngle=30;

bool goingToCan = false;

//srTOF output string
char temp_str[64];



CytronMD DCmotor(PWM_PWM,DC1_PIN,DC2_PIN);
Adafruit_MPU6050 mpu;
Servo steerServo;

// srTOFs
VL53L5CX sensor_vl53l5cx_sat_1(&Wire, LPN_PIN_1, I2C_RST_PIN_1);
VL53L5CX sensor_vl53l5cx_sat_2(&Wire, LPN_PIN_2, I2C_RST_PIN_2);


void setup() {                  //STRICTLY SETUP CODE
  //start console serial
  Serial.begin(115200);

  //Start I2C bus
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


  //srTOF setup
  shortTOFsetup();
  

  //create Queues
  srTOFrawQ = xQueueCreate(1, sizeof(Valid_target_Info));
  lrTOFrawQ = xQueueCreate(1, sizeof(lrTOFrawData));
  srTOFprocQ = xQueueCreate(1, sizeof(Valid_target_Info));
  lrTOFprocQ = xQueueCreate(1, sizeof(lrTOFrawData));
  combinedSensorQ = xQueueCreate(1, sizeof(targets));

  //TaskCreation
  xTaskCreatePinnedToCore(  driveManager,       //Function Name
                "drivingManager",     //Task Name
                20000,         //Stack size
                NULL,         //Task input parameter
                1,          //Task priority 
                &driveHandler, //Task handle
                0);      //Core

  xTaskCreatePinnedToCore(  shortTOF,       //Function Name
                "shortTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                0,            //Task priority 
                &shortTOFHandler,  //Task handle
                1);      //Core

                
  xTaskCreatePinnedToCore(  longTOF,       //Function Name
                "longTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                0,            //Task priority
                &longTOFHandler,    //Task handle
                1);         //Core

  xTaskCreatePinnedToCore(  shortTOFprocess,       //Function Name
                "shortTOF processing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                1,            //Task priority 
                &shortTOFprocessHandler,  //Task handle
                1);      //Core

                
  xTaskCreatePinnedToCore(  longTOFprocess,       //Function Name
                "longTOF processing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                1,            //Task priority
                &longTOFprocessHandler,    //Task handle
                1);         //Core


  xTaskCreatePinnedToCore(  turn,       //Function Name
                "turn",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                1,              //Task priority 
                &turnHandler,   //Task handle
                0);             //Core

   xTaskCreatePinnedToCore(  sensorControl,       //Function Name
                 "sensorControl",     //Task Name
                 20000,         //Stack size
                 NULL,         //Task input parameter
                 2,              //Task priority 
                &sensorHandler,   //Task handle
                1);               //core

  xTaskCreatePinnedToCore(  taskManager,       //Function Name
                "task manager",     //Task Name
                10000,         //Stack size
                NULL,         //Task input param
                0,            //Task priority 
                NULL,         //Task handle
                0);      //Core



  //start the machine
  vTaskStartScheduler();

}



void taskManager(void * pvParameters){
  // WHILE LOOP TO POLL FOR START SIGNAL



  //Initial Drive
  xTaskNotifyGive(driveHandler);

  //wait until we are in search area
  vTaskDelay(initialDriveTime / portTICK_PERIOD_MS);

  //start sensing
  xTaskNotifyGive(shortTOFHandler);
  xTaskNotifyGive(longTOFHandler);

  //delay(lengthTime);
  vTaskDelay(lengthTime / portTICK_PERIOD_MS);


  //turn
  //vTaskSuspend(shortTOFHandler);
  //vTaskSuspend(longTOFHandler);
  
  vTaskSuspend(driveHandler);
  xTaskNotifyGive(turnHandler);
  vTaskDelay(turnTime / portTICK_PERIOD_MS);

  //Start new length
  vTaskResume(driveHandler);
  //vTaskResume(shortTOFHandler);
  //vTaskResume(longTOFHandler);


  //Restarts the driving for a search

  // xTaskNotifyGive(turnHandler);
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
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


    targets new_targets;
    if(xQueueReceive(combinedSensorQ, &new_targets, 0) == pdPASS){
      Serial.print("New targets received from sensors");

      //new sensor data should be used to set angle or something
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
    //is this for debouncing?
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



void turn(void * pvParameters){

  //suspend Task indefinitely until called by taskmanager
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    if (goingToCan == false){
      steerServo.write(turnAngle);
      Serial.println("Turn"); 
    }
    vTaskSuspend(NULL);

  }
}


void shortTOFsetup(void){

  //parameters for setup procedure
  int16_t status;
  

  // Enable PWREN's pins if present
  if (PWREN_PIN_1 >= 0){
  
    pinMode(PWREN_PIN_1, OUTPUT);
    digitalWrite(PWREN_PIN_1, HIGH);
    
    pinMode(PWREN_PIN_2, OUTPUT);
    digitalWrite(PWREN_PIN_2, HIGH);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  
  }
  // Configure VL53L5CX satellite component.
  sensor_vl53l5cx_sat_1.begin();
  sensor_vl53l5cx_sat_2.begin();

  sensor_vl53l5cx_sat_1.init_sensor();
  sensor_vl53l5cx_sat_2.init_sensor();
  
  pinMode(LPN_PIN_1, OUTPUT);      // VL53L5CX_1 LPN pin
  pinMode(LPN_PIN_2, OUTPUT);      // VL53L5CX_2 LPN pin
  digitalWrite(LPN_PIN_1, HIGH);    // enable VL53L5CX_1
  digitalWrite(LPN_PIN_2, LOW);   // disable VL53L5CX_2

  //set I2C address 1
  status = sensor_vl53l5cx_sat_1.vl53l5cx_set_i2c_address(I2C_Address_1<<1);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_i2c_address failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }
  
  digitalWrite(LPN_PIN_1, LOW);    // disable VL53L5CX_1
  digitalWrite(LPN_PIN_2, HIGH);   // enable VL53L5CX_2

  //set I2C address 2
  status = sensor_vl53l5cx_sat_2.vl53l5cx_set_i2c_address(I2C_Address_2<<1);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_i2c_address failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }

  digitalWrite(LPN_PIN_1, HIGH);    // enable VL53L5CX_1


  //Enter 8X8 mode 1
  status = sensor_vl53l5cx_sat_1.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_resolution failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }

  //Enter 8X8 mode 2
  status = sensor_vl53l5cx_sat_2.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_resolution failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }
  
  //Set frequency 1
  status = sensor_vl53l5cx_sat_1.vl53l5cx_set_ranging_frequency_hz(vl53l5cx_frequency);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_ranging_frequency_hz failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }
  //Set frequency 2
  status = sensor_vl53l5cx_sat_2.vl53l5cx_set_ranging_frequency_hz(vl53l5cx_frequency);
  if (status) 
  {
    snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_ranging_frequency_hz failed, status %u\r\n", status);
    SerialPort.print(temp_str);
  }
  
  // Start Measurements
  sensor_vl53l5cx_sat_1.vl53l5cx_start_ranging();
  sensor_vl53l5cx_sat_2.vl53l5cx_start_ranging();

}

void shortTOF(void * pvParameters){

  VL53L5CX_ResultsData Results_1;
  VL53L5CX_ResultsData Results_2;
  uint8_t NewDataReady = 0;
  int16_t i, j, k;

  Valid_target_Info target_info;

  uint16_t    closest_white_distance = 9999, closest_black_distance = 9999;
  float       closest_white_angle = 0, closest_black_angle = 0;

  //Code before here should run on startup

  //suspend Task indefinitely until called by taskmanager
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    
    NewDataReady = 0;
    sensor_vl53l5cx_sat_1.vl53l5cx_check_data_ready(&NewDataReady);
    if (NewDataReady) 
    {
      sensor_vl53l5cx_sat_1.vl53l5cx_get_ranging_data(&Results_1);
    }
    NewDataReady = 0;
    sensor_vl53l5cx_sat_2.vl53l5cx_check_data_ready(&NewDataReady);
    if (NewDataReady) 
    {
      sensor_vl53l5cx_sat_2.vl53l5cx_get_ranging_data(&Results_2);
    }
      i = 0;
      for(j = 7; j > 0; j--)
      {
        for(k = 0; k < 8; k++)
        {
            //Check data is valid if there is 1 target and status is 5. If measurment is invalid, set zone to 9999.
          if(Results_1.target_status[i] != 5 && Results_1.target_status[i] != 6 && Results_1.target_status[i] != 9 && Results_1.nb_target_detected[VL53L5CX_RESOLUTION_8X8] != 1)
          {
              target_info.target_distance[j][k] = 9999;
              target_info.target_reflectance[j][k] = 9999;
          }
          else
          {
              target_info.target_distance[j][k] = Results_1.distance_mm[i];
              target_info.target_reflectance[j][k] = Results_1.reflectance[i];
          }
          i++;
        }
      }

      i = 0;
      for(j = 7; j > 0; j--)
      {
        for(k = 8; k < 16; k++)
        {
            //Check data is valid if there is 1 target and status is 5. If measurment is invalid, set zone to 9999.
          if(Results_2.target_status[i] != 5 && Results_2.target_status[i] != 6 && Results_2.target_status[i] != 9 && Results_2.nb_target_detected[VL53L5CX_RESOLUTION_8X8] != 1)
          {
              target_info.target_distance[j][k] = 9999;
              target_info.target_reflectance[j][k] = 9999;
          }
          else
          {
              target_info.target_distance[j][k] = Results_2.distance_mm[i];
              target_info.target_reflectance[j][k] = Results_2.reflectance[i];
          }
          i++;
        }
      }


      //print validated distance array
      for (j = 0; j < 8; j++) 
      {
        for (k = 0; k < 16; k++) 
        {
            snprintf(temp_str, 20, "%04d", target_info.target_distance[j][k]);
            SerialPort.print(temp_str);
            SerialPort.print("\t");
        }
        SerialPort.print("\n");
      } 
    //delay to see something in console while debugging
    //vTaskDelay(5000 / portTICK_PERIOD_MS);

    //check if queue transmission was successful
    if(xQueueSend(srTOFrawQ, &target_info, 0) == pdPASS){
      //start processing task
      xTaskNotifyGive(shortTOFprocessHandler);
    }
    else{
      Serial.println("Short Range TOF Raw Queue TX fault"); //error message in case Q is full
    }
  
  }
  
}

void shortTOFprocess(void * pvParameters){
  
  //Code before here should run on startup

  while(1){

    //suspend Task indefinitely until called by taskmanager
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Valid_target_Info target_info;

    //read new data from Queue once notified by RX
    if(xQueueReceive(srTOFrawQ, &target_info, 0) == pdPASS){
      //processing goes in here

      //pass data to decision function
      if(xQueueSend(srTOFprocQ, &target_info, 0) == pdPASS){
        //start processing task
        xTaskNotifyGive(sensorHandler);
        vTaskSuspend(NULL);
      }
      else{
        Serial.println("Short Range TOF Proc Queue TX fault"); //error message in case Q is full
        vTaskSuspend(NULL);
      }
    }
    else{
      Serial.println("Short Range TOF Raw Queue RX fault"); //error message in case Q is full
      vTaskSuspend(NULL);
    }
  }
  
  

}

void longTOF(void * pvParameters){

  //Code before here should run on startup?
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while(1){
    //get sensor data
    lrTOFrawData measureData;

    //send data on queue
    if(xQueueSend(lrTOFrawQ, &measureData, 0) == pdPASS){
      //start processing task
      xTaskNotifyGive(longTOFprocessHandler);
    }
    else{
      Serial.println("Long Range TOF Raw Queue TX fault"); //error message in case Q is full
    }

  } 

}




void longTOFprocess(void * pvParameters){
  
  //Code before here should run on startup

  while(1){

    //suspend Task indefinitely until called by taskmanager
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    lrTOFrawData inputData;

    //read new data from Queue once notified by RX
    if(xQueueReceive(lrTOFrawQ, &inputData, 0) == pdPASS){
      //processing goes in here

      //pass data to decision function
      if(xQueueSend(lrTOFprocQ, &inputData, 0) == pdPASS){
        //start processing task
        xTaskNotifyGive(sensorHandler);
        vTaskSuspend(NULL);
      }
      else{
        Serial.println("Long Range TOF Proc Queue TX fault"); //error message in case Q is full
        vTaskSuspend(NULL);
      }
    }

    else{
      Serial.println("Long Range TOF Raw Queue RX fault"); //error message in case Q is full
      vTaskSuspend(NULL);
    }
    
  }
  

}


void sensorControl(void * pvParameters){
  
  //Code before here should run on startup?

  while(1){
    //suspend Task indefinitely until called by taskmanager
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Valid_target_Info srInput;
    lrTOFrawData lrInput;

    uint8_t new_lr_data_flag = 0;
    uint8_t new_sr_data_flag = 0;


    //read new data from both Queue once notified by Processing Tasks
    if(xQueueReceive(srTOFprocQ, &srInput, 0) == pdPASS){
      new_sr_data_flag = 1;
    }
    else{
      Serial.println("Short Range Processing Queue RX fault"); //error message in case Q is full
      vTaskSuspend(NULL);
    }
    if(xQueueReceive(lrTOFprocQ, &srInput, 0) == pdPASS){
      new_lr_data_flag = 1;
    }
    else{
      Serial.println("Long Range Processing Queue RX fault"); //error message in case Q is full
      vTaskSuspend(NULL);
    }

    //decisionmaking based on new data
    targets output_data;


    //sending data to Driving side
    if(xQueueSend(combinedSensorQ, &output_data, 0) == pdPASS){
      Serial.println("New Target data successfully sent");
      vTaskSuspend(NULL);
    }
    else{
      Serial.println("Unifed sensor data Queue TX fault"); //error message in case Q is full
      vTaskSuspend(NULL);
    }


    vTaskSuspend(NULL);
  }
  
}


void loop() {
}
