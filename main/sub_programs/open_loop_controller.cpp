#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

//Motor with encoder parameters
#define IN_1 GPIO_NUM_26
#define IN_2 GPIO_NUM_27
#define EN_PIN GPIO_NUM_14
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19
#define RATIO 34
#define CPR 48
#define DT_MS 25


extern "C" void app_main(void) {
    //Creates motor with encoder object
    Motor_with_Encoder motor(IN_1, IN_2, EN_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0, ENCODER_A, ENCODER_B, RATIO, CPR, DT_MS);
    //motor.set_inverted();

    //Main task loop
    uint8_t power = 0;
    while (1) {
        motor.set_power(power); //Set motor power to potentiometer value in its range (-100, 100)
        printf("Volatge: %fV\tSpeed: %f rpm\n", motor.get_voltage(), motor.get_speed(RPM)); //Print motor speed in rpm
        power+=1;
        if (power>100) power = 0;
        vTaskDelay(pdMS_TO_TICKS(50)); //Delay 50ms
    }
}