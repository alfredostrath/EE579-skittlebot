



#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 2   //Set GPIO 2 as PWM0A - motor left
#define GPIO_PWM0B_OUT 4   //Set GPIO 4 as PWM0B - motor right
#define GPIO_PWM1A_OUT 5   //Set GPIO 5 as PWM1A - servo

float duty_cycle = 10;
float duty_cycle_servo=5;
float servo_angle=0;
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);


    //servo pin
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
}


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief servo control
 */
static void servo_control(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    duty_cycle_servo=5+5*(servo_angle/180);
    //mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle_servo);
    //mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    printf("Servo control:");
    printf("%f", duty_cycle_servo);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;


    //servo setup
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); 

    pwm_config.frequency = 50;    //frequency = 50Hz,
    pwm_config.cmpr_a = 5;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;


    //servo setup
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); 
  

    while(1){
        servo_angle=40;
        servo_control(MCPWM_UNIT_0, MCPWM_TIMER_1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        servo_angle=60;
        servo_control(MCPWM_UNIT_0, MCPWM_TIMER_1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        servo_angle=80;

        servo_control(MCPWM_UNIT_0, MCPWM_TIMER_1);
        vTaskDelay(1000/portTICK_PERIOD_MS);

    //     vTaskDelay(5000/portTICK_PERIOD_MS);
    //     servo_angle=20;
    //     brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, duty_cycle);
    //     servo_control(MCPWM_UNIT_0, MCPWM_TIMER_1);
    //     vTaskDelay(5000/portTICK_PERIOD_MS);

    //     brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, duty_cycle);
    //     vTaskDelay(5000/portTICK_PERIOD_MS);

    //     brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    //     vTaskDelay(2000/portTICK_PERIOD_MS);

    //     servo_angle=80;
    //     servo_control(MCPWM_UNIT_0, MCPWM_TIMER_1);
        
    //     vTaskDelay(5000/portTICK_PERIOD_MS);
        
    //     duty_cycle = duty_cycle + 10;
    //     if(duty_cycle == 100)
    //     {
    //         duty_cycle = 10; 
    //     }
    //     vTaskDelay(500/portTICK_PERIOD_MS);
     }
}


void app_main(void)
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, NULL, 5, NULL);
}