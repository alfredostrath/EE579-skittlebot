
#include "CytronMotorDriver.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <ESP32Servo.h>

//#include <customLib.h>


static TaskHandle_t sensorHandler = NULL;
static TaskHandle_t searchHandler = NULL;
static TaskHandle_t PIDHandler = NULL;
int angle =0;
int pos=0;
float offset_error=0.0;
int servoPin=5;
//test
Adafruit_MPU6050 mpu;
CytronMD DCmotor(PWM_PWM,2,4);
Servo steerServo;

float heading=0.0;



void setup() { 
  Serial.begin(115200);

  //mpuSetup(mpu);
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
  delay(1000);
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steerServo.setPeriodHertz(50);    // standard 50 hz servo
	steerServo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
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
      int i=0;
      




      while(1){  
        sensors_event_t a,g,temp;
        mpu.getEvent(&a,&g,&temp);                         
      
        if (i<20){
          offset_error+=g.gyro.x;
          
        }
        i++;
        offset_error=offset_error/20;
        Serial.print("Gyro reading:");
        Serial.println(g.gyro.x);

        heading=(heading+(offset_error-g.gyro.x)*0.1);
        Serial.print("Sensing:");
        Serial.println(i);

        Serial.print("Heading:");
        Serial.println(heading*(180/3.1416)+57);
        delay(100);
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
      Serial.println(heading);
    }
  }
}


void loop() {
  delay(20000);
}