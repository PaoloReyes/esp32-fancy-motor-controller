#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"

//Motor with encoder parameters
#define IN_1 GPIO_NUM_26
#define IN_2 GPIO_NUM_27
#define EN_PIN GPIO_NUM_14
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19
#define RATIO 34
#define CPR 48
#define DT_MS 25

//ADC parameters
#define POT_PIN ADC1_CHANNEL_7
#define ADC_RESOLUTION ADC_WIDTH_BIT_12

extern "C" void app_main(void) {
    //Creates motor with encoder object
    Motor_with_Encoder motor(IN_1, IN_2, EN_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0, ENCODER_A, ENCODER_B, RATIO, CPR, DT_MS);
    //motor.set_inverted();

    //Configure ADC Channel
    adc1_config_channel_atten(POT_PIN, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_RESOLUTION);

    //Main task loop
    while (1) {
        motor.set_power(map(adc1_get_raw(POT_PIN), 0, (1<<ADC_RESOLUTION)-1, -100, 100)); //Set motor power to potentiometer value in its range (-100, 100)
        printf("Power: %d%%\tDirection: %d\tSpeed: %f rpm\n", motor.get_power(), motor.get_direction(), motor.get_speed(RPM)); //Print motor speed in rpm
        vTaskDelay(pdMS_TO_TICKS(50)); //Delay 50ms
    }
}