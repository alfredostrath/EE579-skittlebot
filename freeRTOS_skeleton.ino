static TaskHandle_t sensorHandler = NULL;
static TaskHandle_t searchHandler = NULL;
static TaskHandle_t PIDHandler = NULL;
int angle =0;

#include "CytronMotorDriver.h"
CytronMD DCmotor(PWM_PWM,2,4);

void setup() { 
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
                10000,         //Stack size
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
  
  Serial.begin(9600);
  Serial.print("Started");
}


void initial_drive (void*pvParameters){
  DCmotor.setSpeed(128);
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
        i++;
        Serial.print("Sensing:");
        Serial.println(i);
        delay(500);
        if (i==10){     
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
  while(1){
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY )!=0){
      Serial.print("PID enabled:");
      Serial.println(angle);
    }
  }
}


void loop() {
  delay(20000);
}