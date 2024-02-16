#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define IN_1 GPIO_NUM_26
#define IN_2 GPIO_NUM_27
#define EN_PIN GPIO_NUM_14
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19
#define RATIO 34
#define CPR 48
#define DT_MS 25

extern "C" void app_main(void) {
    Motor_with_Encoder motor(IN_1, IN_2, EN_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0, ENCODER_A, ENCODER_B, RATIO, CPR, DT_MS);
    
    motor.set_power(38, true);
    while (1) {
        double speed = motor.get_speed();
        printf("Speed: %f\n", speed);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}