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
#include <math.h>

#define SerialPort Serial

// //Pin declarations I2C
 #define I2C_SDA 1
 #define I2C_SCL 2
 //#define I2C_RST_PIN 4

 #define DC1_PIN 14
 #define DC2_PIN 15
 #define SERVO_PIN 36

 //srTOF configs
#define vl53l5cx_frequency 1

#define LPN_PIN_1 6
#define I2C_RST_PIN_1 4
#define I2C_Address_1 0x28

#define LPN_PIN_2 7
#define I2C_RST_PIN_2 8
#define I2C_Address_2 0x27

#define refelction_threshold 50





//srTof typedef
typedef struct
{
    uint16_t target_distance[8][16];
    int16_t target_angle[8][16];
    uint16_t target_reflectance[8][16];
    
    uint16_t closest_white_distance = 99999, closest_black_distance = 99999;
    float closest_white_angle = 0, closest_black_angle = 0;

} Valid_target_Info;

//lrTof typedef
typedef struct{
    unsigned long TOF_distances[360] = {0};   // length double of scan_angle
    unsigned int TOF_strengths[360] = {0};    // length double of scan_angle
} lrTOFrawData;

//unified Target typedef
typedef struct{
    uint16_t black_angle = 0;
    uint16_t black_distance = 99999;
    uint16_t white_angle = 0;
    uint16_t white_distance = 99999;
} targets;

//taskHandlers
static TaskHandle_t driveHandler = NULL;
static TaskHandle_t shortTOFHandler = NULL;
static TaskHandle_t longTOFHandler = NULL;
static TaskHandle_t turnHandler = NULL;
static TaskHandle_t sensorHandler = NULL;
static TaskHandle_t shortTOFprocessHandler = NULL;
static TaskHandle_t longTOFprocessHandler = NULL;
static TaskHandle_t taskHandler = NULL;

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

//srTOF universal temporary string
char temp_str[64];

//scanning Servo
int servoPin = 35;  // servo pin (number is number on board)
const int scan_angle = 180;
const int avg_window_length = 10;

unsigned char TOF_check = 0;

//skittle width for processing
float skittle_width = 75.0; // mm (should not change)



CytronMD DCmotor(PWM_PWM,DC1_PIN,DC2_PIN);
Adafruit_MPU6050 mpu;
Servo steerServo;
Servo myservo;  // servo object to control servo

// srTOFs
VL53L5CX sensor_vl53l5cx_sat_1(&Wire, LPN_PIN_1, I2C_RST_PIN_1);
VL53L5CX sensor_vl53l5cx_sat_2(&Wire, LPN_PIN_2, I2C_RST_PIN_2);

Valid_target_Info target_info;


void setup() {                  //STRICTLY SETUP CODE
  //start console serial
  Serial.begin(921600);
  Serial.println("Starting Setup");

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
  Serial.println("Calibrating IMU");                     
  for (int i=0; i<20; i++){
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp); 
    //Serial.print("Calibration reading:");
    //Serial.println(g.gyro.x);
    //Serial.println(i);
    delay(100);
  
    IMUerror=IMUerror + g.gyro.x;
    //Serial.print("IMU ERROR: ");
    //Serial.println(IMUerror);
  }
 
  IMUerror=IMUerror/20;

  //Serial.print("IMU ERROR: ");
  //Serial.println(IMUerror);
  //delay(2000);

  Serial.println("Calibrating IMU finished");

  Serial.println("Setting up Servos");
  //STEER SERVO SETUP
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steerServo.setPeriodHertz(50);    // standard 50 hz servo
  steerServo.attach(SERVO_PIN, 1000, 2000);

  //scan servo setup
  myservo.setPeriodHertz(50);           // standard 50 hz servo
	myservo.attach(servoPin, 400, 2420);  // attaches servo on servoPin to servo object

  Serial.println("Setting up Tofs");

  //LR TOF setup
  Serial2.begin(921600, SERIAL_8N1, 16, 17);

  //srTOF setup
  shortTOFsetup();

  Serial.println("Tof Setup done"); 



  //create Queues
  srTOFrawQ = xQueueCreate(1, sizeof(Valid_target_Info));
  lrTOFrawQ = xQueueCreate(1, sizeof(lrTOFrawData));
  srTOFprocQ = xQueueCreate(1, sizeof(Valid_target_Info));
  lrTOFprocQ = xQueueCreate(1, sizeof(lrTOFrawData));
  combinedSensorQ = xQueueCreate(1, sizeof(targets));

  Serial.println("Queues created");

  //TaskCreation
  xTaskCreatePinnedToCore(  driveManager,       //Function Name
                "drivingManager",     //Task Name
                20000,         //Stack size
                NULL,         //Task input parameter
                6,          //Task priority 
                &driveHandler, //Task handle
                0);      //Core

  xTaskCreatePinnedToCore(  shortTOF,       //Function Name
                "shortTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                5,            //Task priority 
                &shortTOFHandler,  //Task handle
                1);      //Core

                
  xTaskCreatePinnedToCore(  longTOF,       //Function Name
                "longTOF sensing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                5,            //Task priority
                &longTOFHandler,    //Task handle
                1);         //Core

  xTaskCreatePinnedToCore(  shortTOFprocess,       //Function Name
                "shortTOF processing",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                6,            //Task priority 
                &shortTOFprocessHandler,  //Task handle
                1);      //Core

                
  xTaskCreatePinnedToCore(  longTOFprocess,       //Function Name
                "longTOF processing",     //Task Name
                100000,         //Stack size
                NULL,         //Task input parameter
                6,            //Task priority
                &longTOFprocessHandler,    //Task handle
                1);         //Core


  xTaskCreatePinnedToCore(  turn,       //Function Name
                "turn",     //Task Name
                10000,         //Stack size
                NULL,         //Task input parameter
                6,              //Task priority 
                &turnHandler,   //Task handle
                0);             //Core

  //  xTaskCreatePinnedToCore(  sensorControl,       //Function Name
  //                "sensorControl",     //Task Name
  //                20000,         //Stack size
  //                NULL,         //Task input parameter
  //                2,              //Task priority 
  //               &sensorHandler,   //Task handle
  //               1);               //core

  xTaskCreatePinnedToCore(  taskManager,       //Function Name
                "task manager",     //Task Name
                10000,         //Stack size
                NULL,         //Task input param
                5,            //Task priority 
                &taskHandler,         //Task handle
                0);      //Core


  //start the machine
  
  Serial.println("All tasks are created");
  xTaskNotifyGive(taskHandler);

}



void taskManager(void * pvParameters){
  //make sure everything is initiallised
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  Serial.println("Task manager is ready for user input");

  //this delay should be replaced by button input
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  // WHILE LOOP TO POLL FOR START SIGNAL
  Serial.println("User has activated driving!");

  //Initial Drive
  xTaskNotifyGive(driveHandler);

  Serial.println(("Driving straight for 5s!"));


  //wait until we are in search area
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  //start sensing SR
  //xTaskNotifyGive(shortTOFHandler);

  //serpentine loop
  while(1){
    

    //do sweep of LR
    vTaskSuspend(driveHandler);
    xTaskNotifyGive(longTOFHandler);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Task Manager Alive again");
    vTaskResume(driveHandler);

    //make sure LR data has reached IMU
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    //start SR again
    xTaskNotifyGive(shortTOFHandler);

    //time until we've gone to hit zone
    Serial.println("Driving straight now");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    //time until we've gone through entire zone
    //vTaskDelay(lengthTime / portTICK_PERIOD_MS);


    //turn
    vTaskSuspend(shortTOFHandler);
    //vTaskSuspend(longTOFHandler);
  
    //vTaskSuspend(driveHandler);
    xTaskNotifyGive(turnHandler);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //Start new length
    //vTaskResume(driveHandler);
    
    //vTaskResume(longTOFHandler);


    //Restarts the driving for a search

    // xTaskNotifyGive(turnHandler);
  }

  
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
}


void driveManager(void * pvParameters){
  int Kp = 1;
  int servoAngle=45;
  float err =0.0;
  bool canApproach=false;
  int approachTimer=0;

  
  Serial.println("Drive task ready");

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  Serial.println("Starting driving task");

  while(1){
    
    Serial.println("New drive control iteration");

    DCmotor.setSpeed(255);
    
    //Sample IMU
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp); 
    
    //Serial.print("Measure:");
    //Serial.println(g.gyro.x);                       
    heading=(heading+(IMUerror-g.gyro.x)*0.1);
    //Serial.print("Heading:");
    headingDEG = heading *(180/3.14159);
    //Serial.println(headingDEG);


    

    //Set heading - either can if target info is available or heading if not 

    //store new targets
    targets new_targets;

    if(xQueueReceive(combinedSensorQ, &new_targets, 0) == pdPASS  && new_targets.black_distance!=99999 ){
      err = new_targets.black_angle;
      canApproach = true;
      approachTimer=0;

      Serial.println("DriveManager received new target");
      Serial.println(new_targets.black_angle);
      Serial.println(new_targets.black_distance);

      //make sure we can read output
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else if (canApproach==false)
    {
      //IMU PID Loop
      err = headingDEG;
    }

    //Maintains the last can detection for three  seconds, then switches heading to global heading if no detection is made.
    if (canApproach==true){
      approachTimer+=1;
    }
    else if (approachTimer>30){   //maintains angle for three seconds (30 * 100 ms)
      canApproach=false;
      approachTimer=0;
      err= headingDEG;
    }

    servoAngle = 45-err*Kp;
    steerServo.write(servoAngle);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



void turn(void * pvParameters){

  Serial.println("Turn task ready");

  //suspend Task indefinitely until called by taskmanager
  

  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("Turn task called");

    steerServo.write(turnAngle);

    //vTaskSuspend(NULL);

  }
}


void shortTOFsetup(void){

  //parameters for setup procedure
  int16_t status, i, j, k;
  float angle, angle_step, fov = 85, num_zones = 16;

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

  //set angles
  angle_step = fov/num_zones;
  
  angle = (-angle_step * 8) + (float(0.5)*angle_step);

  for (k = 0; k < 16 ; k++) 
  {
    for (j = 0; j < 8; j++) 
    {
        target_info.target_angle[j][k] = round(angle);
    }
    angle += angle_step;
  } 
  
}

void shortTOF(void * pvParameters){
  
  VL53L5CX_ResultsData Results_1;
  VL53L5CX_ResultsData Results_2;
  uint8_t NewDataReady = 0;
  int16_t status, i, j, k;


  uint16_t    closest_white_distance = 99999, closest_black_distance = 99999;
  float       closest_white_angle = 0, closest_black_angle = 0;

  //Code before here should run on startup

  Serial.println("SR TOF task ready");
  //suspend Task indefinitely until called by taskmanager
  
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  

  while(1){
    
    Serial.println("New SR TOF scan");

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
      for(j = 7; j > -1; j--)
      {
        for(k = 0; k < 8; k++)
        {
            //Check data is valid if there is 1 target and status is 5. If measurment is invalid, set zone to 99999.
          if(Results_1.target_status[i] != 5 && Results_1.target_status[i] != 6 && Results_1.target_status[i] != 9 && Results_1.nb_target_detected[VL53L5CX_RESOLUTION_8X8] != 1)
          {
              target_info.target_distance[j][k] = 99999;
              target_info.target_reflectance[j][k] = 99999;
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
      for(j = 7; j > -1; j--)
      {
        for(k = 8; k < 16; k++)
        {
            //Check data is valid if there is 1 target and status is 5. If measurment is invalid, set zone to 99999.
          if(Results_2.target_status[i] != 5 && Results_2.target_status[i] != 6 && Results_2.target_status[i] != 9 && Results_2.nb_target_detected[VL53L5CX_RESOLUTION_8X8] != 1)
          {
              target_info.target_distance[j][k] = 99999;
              target_info.target_reflectance[j][k] = 99999;
          }
          else
          {
              target_info.target_distance[j][k] = Results_2.distance_mm[i];
              target_info.target_reflectance[j][k] = Results_2.reflectance[i];
          }
          i++;
        }
      }

    Serial.println("SR_output check");
    //check if queue transmission was successful
    if(xQueueSend(srTOFrawQ, &target_info, 0) == pdPASS){
      //start processing task
      xTaskNotifyGive(shortTOFprocessHandler);
    }
    else{
      Serial.println("Short Range TOF Raw Queue TX fault"); //error message in case Q is full
    }

    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
  
}

void shortTOFprocess(void * pvParameters){
  
  //Code before here should run on startup
   Serial.println("SR TOF processing task ready");

  while(1)
  {
    //suspend Task indefinitely until called by taskmanager
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("SR TOF data processing");

    Valid_target_Info target_info;
    int16_t status, i, j, k;

    //read new data from Queue once notified by RX
    if(xQueueReceive(srTOFrawQ, &target_info, 0) == pdPASS)
    {
      //search array for closest black and white zones
      //reset from last iterration
      target_info.closest_white_distance = 99999;
      target_info.closest_black_distance = 99999;
      for (k = 0; k < 16 ; k++) 
      {
        for (j = 0; j < 8; j++) 
        {
          //find closest white zone
          if(target_info.target_distance[j][k] < target_info.closest_white_distance && target_info.target_reflectance[j][k] > refelction_threshold)
          {
              target_info.closest_white_distance = target_info.target_distance[j][k];
              target_info.closest_white_angle = target_info.target_angle[j][k];
          }
          //find closest black zone
          if(target_info.target_distance[j][k] < target_info.closest_black_distance && target_info.target_reflectance[j][k] < refelction_threshold)
          {
              target_info.closest_black_distance = target_info.target_distance[j][k];
              target_info.closest_black_angle = target_info.target_angle[j][k];
          }
        }
      } 


      // //print validated distance array

      // for (j = 0; j < 8; j++) 
      // {
      //   for (k = 0; k < 16; k++) 
      //   {
      //       snprintf(temp_str, 20, "%04d", target_info.target_distance[j][k]);
      //       SerialPort.print(temp_str);
      //       SerialPort.print("\t");
      //   }
      //   SerialPort.print("\n");
      // }

      // SerialPort.print("\n");
      // //print angle array
      // for (j = 0; j < 8; j++) 
      // {
      //   for (k = 0; k < 16; k++) 
      //   {
      //       snprintf(temp_str, 20, "%04d", target_info.target_angle[j][k]);
      //       SerialPort.print(temp_str);
      //       SerialPort.print("\t");
      //   }
      //   SerialPort.print("\n");
      // }  
      // SerialPort.print("\n");
      // SerialPort.print("\n");
    
      targets outputData;
      outputData.black_angle = target_info.closest_black_angle;
      outputData.black_distance = target_info.closest_black_distance;
      outputData.white_angle = target_info.closest_black_angle;
      outputData.white_distance = target_info.closest_white_distance;

      //pass data to decision function
      if(xQueueOverwrite(combinedSensorQ, &outputData) == pdPASS)
      {
      //start processing task
      //xTaskNotifyGive(sensorHandler);
      Serial.println("Short Range TOF Data passed on successfully");
      }
      else
      {
        Serial.println("Short Range TOF Proc Queue TX fault"); //error message in case Q is full
      }
    }
    else
    {
      Serial.println("Short Range TOF Raw Queue RX fault"); //error message in case Q is full
      
    }
  }
  
  

}

bool verifyWidth(unsigned long dist, float ang_width) {
  float expected_ang_width;
  expected_ang_width = (skittle_width / 2.0 / (float)dist);
  expected_ang_width = 2.0 * asin(expected_ang_width);
  
  // if object width within 1 deg of expected with
  // if (ang_width <= (expected_ang_width + 5.0) && ang_width >= (expected_ang_width - 5.0)) {  <-- THIS IS THE RIGHT ONE
  if (ang_width <= (expected_ang_width + 5.0) | ang_width >= (expected_ang_width - 5.0))
  {
    return true;
  } 
  else
  {
    return false;
  }

}

bool verifyCheckSum(unsigned char data[], unsigned char len){
  TOF_check = 0;

  for(int k=0;k<len-1;k++)
  {
      TOF_check += data[k];
  }

  if(TOF_check == data[len-1])
  {
      // Serial.println("TOF data is ok!");
      return true;    
  }else{
      // Serial.println("TOF data is error!");
      return false;  
  }

}

void longTOF(void * pvParameters){

  // FOR SCANNING
  int pos = 0;        // variable to store the servo position
  int min_us = 400;   // right side
  int max_us = 2420;  // left side

  unsigned char TOF_data[32] = {0};   //store 2 TOF frames
  unsigned char TOF_length = 16;
  unsigned char TOF_header[3] {0x57,0x00,0xFF};
  unsigned long TOF_system_time = 0;
  unsigned long TOF_distance = 0;
  unsigned char TOF_status = 0;
  unsigned int TOF_signal = 0;
 
  unsigned long TOF_distances[360] = {0};   // length double of scan_angle
  unsigned int TOF_strengths[360] = {0};    // length double of scan_angle

  Serial.println("LR TOF task ready");
  //Code before here should run on startup?
  

  while(1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("LR TOF scan called");

    //stop driving
    DCmotor.setSpeed(0);
    
    // SWEEP
    long last_valid_dist = 1500;  // mm
    for (pos = 0; pos < 2*scan_angle; pos += 1)
    { // goes from 0 degrees to 180 degrees
      myservo.write((float)pos / 2.0);   // go to position (half degree increments)
      // delay(1);             // delay to allow reaching position
      //Serial.println((float)pos/2.0);

      // wait for enough ToF data
      while(Serial2.available()<32) {
        /* DO NOTHING */
      }

      if (Serial2.available()>=32) {
        for(int i=0;i<32;i++)
        {
          TOF_data[i] = Serial2.read();
        }
      
        for(int j=0;j<16;j++)
        {
          if( (TOF_data[j]==TOF_header[0] && TOF_data[j+1]==TOF_header[1] && TOF_data[j+2]==TOF_header[2]) && (verifyCheckSum(&TOF_data[j],TOF_length)))
          {
            if(((TOF_data[j+12]) | (TOF_data[j+13]<<8) )==0) {
              Serial.println("Out of range!");
            } else {
            // if (true) {
              // Serial.print("TOF id is: ");
              // Serial.println(TOF_data[j+3],DEC);
        
              TOF_system_time = TOF_data[j+4] | TOF_data[j+5]<<8 | TOF_data[j+6]<<16 | TOF_data[j+7]<<24;
              // Serial.print("TOF system time is: ");
              // Serial.print(TOF_system_time,DEC);
              // Serial.println("ms");
        
              TOF_distance = (TOF_data[j+8]) | (TOF_data[j+9]<<8) | (TOF_data[j+10]<<16);
              //Serial.print("TOF distance is: ");
              //Serial.print(TOF_distance,DEC);
              //Serial.println("mm");
        
              TOF_status = TOF_data[j+11];
              // Serial.print("TOF status is: ");
              // Serial.println(TOF_status,DEC);
        
              TOF_signal = TOF_data[j+12] | TOF_data[j+13]<<8;
              // Serial.print("TOF signal is: ");
              // Serial.println(TOF_signal,DEC);
        
              // Save distances and strengths
              if (TOF_distance == 0) {
                TOF_distances[pos] = TOF_distances[pos-1];
                TOF_strengths[pos] = TOF_signal;
              } else {
                TOF_distances[pos] = TOF_distance;  //(TOF_data[j+8]) | (TOF_data[j+9]<<8) | (TOF_data[j+10]<<16);
                TOF_strengths[pos] = TOF_signal;    //TOF_data[j+12] | TOF_data[j+13]<<8;
              }

              // Serial.print("Compensated distance: ");
              // Serial.print(TOF_distance,DEC);
              // Serial.println("mm");
            }
            break;
          }
        }

      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    
    }

    //make sure sweep is done
    vTaskDelay(10 / portTICK_PERIOD_MS);

    //put servo back in starting position  
    myservo.write(0);   // go to position

    //put data in right output format
    lrTOFrawData outputData;
    for(int i=0; i<360; i++)
    {
      outputData.TOF_distances[i] = TOF_distances[i];
      outputData.TOF_strengths[i] = TOF_strengths[i];
    }
    


    //send data on queue
    if(xQueueSend(lrTOFrawQ, &outputData, 0) == pdPASS){
      Serial.println("LR TOF data sent");
      //start processing task
      xTaskNotifyGive(longTOFprocessHandler);
      //restart driving
      xTaskNotifyGive(driveHandler);
      xTaskNotifyGive(taskHandler);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    }
    else{
      Serial.println("Long Range TOF Raw Queue TX fault"); //error message in case Q is full
      //restart driving
      xTaskNotifyGive(driveHandler);
      xTaskNotifyGive(taskHandler);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

  } 

}


void longTOFprocess(void * pvParameters){

  // FOR PROCESSING
  

  signed long TOF_derivatives[360] = {0}; // length double of scan_angle
  // signed int TOF_stren_derivatives[360] = {0};    // length double of scan_angle
  unsigned int threshold_der = 200;       // derivative minimum threshold (mm)
  unsigned int threshold_dist = 7000;     // maximum distance threshold (mm)

  float edge_alpha1 = 999.0;      // first detected edge angle
  float edge_alpha2 = 999.0;      // second detected edge angle
  signed long edge_der1 = 0;      // first edge dy/dx        
  signed long edge_der2 = 0;      // second edge dy/dx 
  signed int edge_stren_der1 = 0;
  signed int edge_stren_der2 = 0;
  unsigned long edge_dist1 = 0;   // first edge distance
  unsigned long edge_dist2 = 0;   // second edge distance
  float edges_ang_width = 999.0;  // angular width between detected cans
  float edges_ang_pos;
  float edges_dist;

  int avg_window_size = 10; // either side
  int candidate_idxs[10];    // store candidate indexes of original data array
  float avg_stren_offset = 10.0;    // brightness filter offset
  
  Serial.println("LR TOF processing task ready");
  //Code before here should run on startup

  //initialise idxs with zero becaue for some weird reason C doesnt do that
  for (int p=0; p<10; p++){
    candidate_idxs[p]=0;
  }
  

  while(1){

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("LR TOF processing task started");

    //suspend Task indefinitely until called by taskmanager

    lrTOFrawData inputData;

    //read new data from Queue once notified by RX
    if(xQueueReceive(lrTOFrawQ, &inputData, 0) == pdPASS)
    {
        //processing goes in here

      //   // DISPLAY DATA
      // for (int k = 0; k < 2*scan_angle; k++)
      // {
      //   Serial.print(k/2.0);
      //   Serial.print(" deg");
      //   Serial.print("     ");
      //   Serial.print(inputData.TOF_distances[k], DEC);
      //   Serial.println(" mm");
      // }

      

      // --- PROCESSING ---
      int idx = 0;  // index of the candidate in the candidate_idxs array
      int edge_idx1 = 0;
      int edge_idx2 = 0;
      float edge_ang_width;
      float edges_ang_pos;

      for (int j = 0; j < 359; j++)
      {

        // compute derivatives distance
        if (inputData.TOF_distances[j] > threshold_dist)
        {
          TOF_derivatives[j] = 0;
        }
        else
        {
          TOF_derivatives[j] = inputData.TOF_distances[j+1] - inputData.TOF_distances[j]; // d(dist)/d(alpha)
        }
        
        // store latest -ve edge
        if (TOF_derivatives[j] < 0 && abs(TOF_derivatives[j]) > threshold_der)
        {
          edge_der1 = TOF_derivatives[j];
          // edge_dist1 = TOF_distances[j+1];
          edge_alpha1 = (float)j / 2.0;  // scan precision is 0.5 deg
          // edge_stren_der1 = TOF_stren_derivatives[j+1];
          // edge_stren1 = TOF_strengths[j+1];
          edge_idx1 = j+1;

        } 
        // store latest +ve edge
        else if (TOF_derivatives[j] > 0 && abs(TOF_derivatives[j]) > threshold_der)
        {
          edge_der2 = TOF_derivatives[j];
          // edge_dist2 = TOF_distances[j];
          edge_alpha2 = (float)j / 2.0;  // scan precision is 0.5 deg
          // edge_stren_der2 = TOF_stren_derivatives[j];
          // edge_stren2 = TOF_strengths[j];
          edge_idx2 = j;

        }

        // if -ve & +ve edges found, second edge is after first, and second edge derivative is -ve
        if (abs(edge_der1) > threshold_der && abs(edge_der2) > threshold_der && edge_alpha2 > edge_alpha1 && edge_der1 < 0) {
          edges_ang_width = edge_alpha2 - edge_alpha1;
          edges_ang_pos = edge_alpha2 - edges_ang_width/2.0;  // angular positon of object
          edges_dist = (edge_dist1 + edge_dist2) / 2.0;       // distance of object
          // edges_stren = ((float)edge_stren_dist2 + (float)edge_stren_dist1)/2.0;  // reflectivity of object

          // COMPARE TO EXPECTED WIDTH HERE
          // if (edges_ang_width >)
          if (verifyWidth(edges_dist, edges_ang_width))
          {
            Serial.println("Potential candidate");
            Serial.println(edges_ang_pos, DEC);
            Serial.println(edges_dist, DEC);
            Serial.println("");
            Serial.println(edge_dist1, DEC);
            Serial.println(edge_dist2, DEC);

            // HERE APPEND TO CANDIDATES ARRAY
            if (idx<10){
              Serial.print("Candidate_idx old: ");
              Serial.println(candidate_idxs[idx]);
              candidate_idxs[idx] = int((edge_idx2 - edge_idx1) / 2);
              Serial.print("Candidate_idx new: ");
              Serial.println(candidate_idxs[idx]);
              idx += 1;
            }
            
          }

          edge_der1 = 0;
          edge_der2 = 0;
          edge_alpha1 = 0.0;
          edge_alpha2 = 0.0;
        }

        

        // HERE NEED TO FILTER 1 BLACK and 1 WHITE TARGET
        // idea: black done and white done check, with loop

      }
      //create output structure
      targets outputData;

      int start_idx;
      int end_idx;
      int mid_idx;
      int black_dist = 99999;
      int white_dist = 99999;
      float avg = 0.0;
      float mid_idx_float;

      for (int g = 0; g < 10; g++)
      { 
        if ((candidate_idxs[g] - avg_window_length) < 0)
        {
          start_idx = 0;
          end_idx = candidate_idxs[g] + avg_window_length;
        }
        else if (candidate_idxs[g] + avg_window_length > 359)
        {
          start_idx = candidate_idxs[g] + avg_window_length;
          end_idx = 359;
        }

        
        for (int z = start_idx; z <= end_idx; z++)
        {
          avg += float(inputData.TOF_strengths[z]);
        }
        avg = (avg / (end_idx - start_idx)) + avg_stren_offset;
        
        mid_idx = candidate_idxs[g];

        
        // white distances
        if(inputData.TOF_strengths[mid_idx] > avg && inputData.TOF_distances[mid_idx] < white_dist)
        {
          outputData.white_angle = int(mid_idx_float/2.0);
          outputData.white_distance = inputData.TOF_distances[mid_idx];
        }
        // black distances
        else if (inputData.TOF_strengths[mid_idx] <= avg && inputData.TOF_distances[mid_idx] < black_dist)
        {
          outputData.black_angle = int(mid_idx_float/2.0);
          outputData.black_distance = inputData.TOF_distances[mid_idx];
        }

        
      }
      //pass data to decision function
      if(xQueueOverwrite(combinedSensorQ, &outputData) == pdPASS)
      {
        //start processing task
        //xTaskNotifyGive(sensorHandler);
      }
      else
      {
        Serial.println("Long Range TOF Proc Queue TX fault"); //error message in case Q is full
      }
     
    }
    else
    {
      Serial.println("Long Range TOF Raw Queue RX fault"); //error message in case Q is full
    }
    
  }
  

}


// void sensorControl(void * pvParameters){
  
//   //Code before here should run on startup?

//   while(1){
//     //suspend Task indefinitely until called by taskmanager
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     targets srInput;
//     targets lrInput;

//     uint8_t new_lr_data_flag = 0;
//     uint8_t new_sr_data_flag = 0;


//     //read new data from both Queue once notified by Processing Tasks
//     if(xQueueReceive(srTOFprocQ, &srInput, 0) == pdPASS){
//       new_sr_data_flag = 1;
//     }
//     else{
//       Serial.println("Short Range Processing Queue RX fault"); //error message in case Q is full
//       vTaskSuspend(NULL);
//     }
//     if(xQueueReceive(lrTOFprocQ, &srInput, 0) == pdPASS){
//       new_lr_data_flag = 1;
//     }
//     else{
//       Serial.println("Long Range Processing Queue RX fault"); //error message in case Q is full
//       vTaskSuspend(NULL);
//     }

//     //decisionmaking based on new data
//     targets output_data;


//     //sending data to Driving side
//     if(xQueueSend(combinedSensorQ, &output_data, 0) == pdPASS){
//       Serial.println("New Target data successfully sent");
//       vTaskSuspend(NULL);
//     }
//     else{
//       Serial.println("Unifed sensor data Queue TX fault"); //error message in case Q is full
//       vTaskSuspend(NULL);
//     }


//     vTaskSuspend(NULL);
//   }
  
// }




void loop() {
}
