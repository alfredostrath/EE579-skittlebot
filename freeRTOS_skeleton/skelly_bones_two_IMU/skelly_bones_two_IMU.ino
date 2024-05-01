
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


//Pin Definitions
#define I2C_SDA 1
#define I2C_SCL 2
#define I2C_RST_PIN 4
#define PWREN_PIN 5
#define LPN_PIN 6
#define DC1_PIN 14
#define DC2_PIN 15
#define SERVO_PIN 40

#define refelction_threshold 50
#define distance_threshold 200
//#include <customLib.h>

static TaskHandle_t sensorHandler = NULL;
static TaskHandle_t searchHandler = NULL;
static TaskHandle_t PIDHandler = NULL;

int angle =0;
int pos=0;
float offset_error=0.0;
int servoPin=40;

//test
Adafruit_MPU6050 mpu;
CytronMD DCmotor(PWM_PWM,DC1_PIN,DC2_PIN);
Servo steerServo;

float heading=0.0;

char temp_str[20];
typedef struct
{
    int16_t target_distance[(VL53L5CX_RESOLUTION_8X8*VL53L5CX_NB_TARGET_PER_ZONE)];
    float target_angle[(VL53L5CX_RESOLUTION_8X8*VL53L5CX_NB_TARGET_PER_ZONE)];
    int16_t target_reflectance[(VL53L5CX_RESOLUTION_8X8*VL53L5CX_NB_TARGET_PER_ZONE)];
        
    uint16_t   closest_white_distance = 9999, closest_black_distance = 9999;
    float    closest_white_angle = 0, closest_black_angle = 0;

} Valid_target_Info;

Valid_target_Info target_info;

// Components.
VL53L5CX sensor_vl53l5cx_sat(&Wire, LPN_PIN, I2C_RST_PIN);

void setup() { 
  Serial.begin(115200);
  Wire.begin(I2C_SDA,I2C_SCL);

  // //mpuSetup(mpu);
  //   if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //     while (1) {
  //       delay(10);
  //     }
  //   }
  // mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("MPU FOUND");
  delay(1000);
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steerServo.setPeriodHertz(50);    // standard 50 hz servo
	steerServo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  
  delay(10000);

  xTaskCreate(
                initial_drive,       //Function Name
                "drive",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                NULL);      //Task handle
  xTaskCreate(
                sensor_start,       //Function Name
                "sensor",     //Task Name
                100000,         //Stack size
                NULL,         //Task priority 
                1,
                &sensorHandler);      //Task handle

   xTaskCreate(
                search_start,       //Function Name
                "search",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                &searchHandler);      //Task handle
    xTaskCreate(
                PID_loop,       //Function Name
                "PID",     //Task Name
                10000,         //Stack size
                NULL,         //Task priority 
                1,
                &PIDHandler);      //Task handle
  
  Serial.print("Started");
  
}


void initial_drive (void*pvParameters){
  DCmotor.setSpeed(128);
  steerServo.write(0);

  for (int i=0; i<10; i++){
  Serial.print("Drive:");
  Serial.println(i);
  delay(1000);
  }
  xTaskNotifyGive(sensorHandler);
  delay(50);
  xTaskNotifyGive(searchHandler);
  vTaskDelete(NULL);
}

void sensor_start(void * pvParameters){                                                     //Start the TOF sensors

    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY )!=0){
      
      // Enable PWREN pin if present
      if (PWREN_PIN >= 0) {
        pinMode(PWREN_PIN, OUTPUT);
        digitalWrite(PWREN_PIN, HIGH);
        delay(10);
      }

      // Initialize serial for output.
   
      SerialPort.println("Initialize... Please wait, it may take few seconds...");

      // Initialize I2C bus.

      VL53L5CX_ResultsData Results;
      uint8_t NewDataReady = 0;
      char temp_str[64];
      uint8_t status, i;



      // Configure VL53L5CX satellite component.
      sensor_vl53l5cx_sat.begin();
      sensor_vl53l5cx_sat.init_sensor();

      //Enter 8X8 mode
      status = sensor_vl53l5cx_sat.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8);
      if (status) {
        snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_resolution failed, status %u\r\n", status);
        SerialPort.print(temp_str);
      }

      //Enter 15 hz mode
      status = sensor_vl53l5cx_sat.vl53l5cx_set_ranging_frequency_hz(1);
      if (status) {
        snprintf(temp_str, sizeof(temp_str), "vl53l5cx_set_ranging_frequency_hz failed, status %u\r\n", status);
        SerialPort.print(temp_str);
      }

      // Start Measurements
      sensor_vl53l5cx_sat.vl53l5cx_start_ranging();
      
      while(1)
      {
        do 
        {
            status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
        } while (!NewDataReady);

          if ((!status) && (NewDataReady != 0)) 
          {
            status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

            for(i = 0; i < 64; i++)
            {
                //Check data is valid if there is 1 target and status is 5. If measurment is invalid, set zone to 9999.
                if(Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i] != 5 && Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i] != 6 && Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i] != 9 && Results.nb_target_detected[VL53L5CX_RESOLUTION_8X8] != 1)
                {
                    target_info.target_distance[i] = 9999;
                    target_info.target_reflectance[i] = 9999;
                }
                else
                {
                    target_info.target_distance[i] = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i];
                    target_info.target_reflectance[i] = Results.reflectance[VL53L5CX_NB_TARGET_PER_ZONE*i];
                }

                //assign each zone an angle. probably an easier way to do this but i dont know it
                if(i % 8 == 0){target_info.target_angle[i] = -19.6875;}
                else if((i - 1) % 8 == 0){target_info.target_angle[i] = -14.0626;}
                else if((i - 2) % 8 == 0){target_info.target_angle[i] = -8.4375;}
                else if((i - 3) % 8 == 0){target_info.target_angle[i] = -2.8125;}
                else if((i - 4) % 8 == 0){target_info.target_angle[i] = 2.8125;}
                else if((i - 5) % 8 == 0){target_info.target_angle[i] = 8.4375;}
                else if((i - 6) % 8 == 0){target_info.target_angle[i] = 14.0626;}
                else if((i - 7) % 8 == 0){target_info.target_angle[i] = 19.6875;} 
            }

            // //print validated distance for middle 4 rows
            // for(i = 16; i < 48; i++)
            // {
            //     if(i % 8 == 0)
            //     {
            //         SerialPort.print("\n");
            //     }
            //     snprintf(temp_str, 20, "%04d", target_info.target_distance[i]);
            //     SerialPort.print(temp_str);
                
            //     SerialPort.print("\t");
            // }
            // //Print middle angles of zones
            // for(i = 16; i < 48; i++)
            // {
            //     if(i % 8 == 0)
            //     {
            //         SerialPort.print("\n");
            //     }
            //     snprintf(temp_str, 20, "%04e", target_info.target_angle[i]);
            //     SerialPort.print(temp_str);
                
            //     SerialPort.print("\t");
            // }
            // //Print target reflectances
            // for(i = 16; i < 48; i++)
            // {
                
            //     if(i % 8 == 0)
            //     {
            //         SerialPort.print("\n");
            //     }
            //     snprintf(temp_str, 20, "%04d", target_info.target_reflectance[i]);
            //     SerialPort.print(temp_str);
                
            //     SerialPort.print("\t");
            // }

            target_info.closest_white_distance = 9999; //reset closest white distance from last iteration. 
            target_info.closest_black_distance = 9999; //reset closest black distance from last iteration. 

            for(i = 16; i < 48; i++) //find closest white and black zone's distances and angles
            {
                //find closest white zone
                if(target_info.target_distance[i] < target_info.closest_white_distance && target_info.target_reflectance[i] > refelction_threshold)
                {
                    target_info.closest_white_distance = target_info.target_distance[i];
                    target_info.closest_white_angle = target_info.target_angle[i];
                }
                //find closest black zone
                if(target_info.target_distance[i] < target_info.closest_black_distance && target_info.target_reflectance[i] < refelction_threshold)
                {
                    target_info.closest_black_distance = target_info.target_distance[i];
                    target_info.closest_black_angle = target_info.target_angle[i];
                    
                }
            }

        }
        delay(10);
        Serial.println(target_info.closest_black_distance);
        // sensors_event_t a,g,temp;
        // mpu.getEvent(&a,&g,&temp);                         

        // if (i<20)
        // {
        //   offset_error+=g.gyro.x;
        // }
        // i++;
        // offset_error=offset_error/20;
        // Serial.print("Gyro reading:");
        // Serial.println(g.gyro.x);

        // heading=(heading+(offset_error-g.gyro.x)*0.1);
        // Serial.print("Sensing:");
        // Serial.println(i);

        // Serial.print("Heading:");
        // Serial.println(heading*(180/3.1416)+57);
        // delay(100);
        if (i==300){     
          angle=5;                                                                    //Simulate a detection
          vTaskDelete(searchHandler);
          xTaskNotifyGive(PIDHandler);
          
        
        }
      }
  
    }
}

void search_start(void * pvParameters){                                     //Drive for straight section of search zone
  while(1){
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY )!=0){
 
      for (int i=0; i<20;i++){
        
        Serial.print("Searching:");
        Serial.println(i);
        delay(500);
      }
    }
  }
}
  

void PID_loop(void * pvParameters){
  
  if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY )!=0){
    Serial.print("PID enabled:");
    while(1){
      
      Serial.print("Heading (PID):");
      Serial.println(target_info.closest_black_distance);
    }
  }
}


void loop() {
  delay(20000);
}