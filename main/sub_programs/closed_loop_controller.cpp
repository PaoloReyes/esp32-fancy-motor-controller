#include "motor.h"
#include "pid_controller.h"
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
    PID_Controller* pid_controller = new PID_Controller(1, 10.5, 0.025); //Creates PID controller object
    motor.set_pid_controller(pid_controller); //Sets PID controller reference to motor with encoder object
    //motor.set_inverted();

    //Main task loop
    pid_controller->set_setpoint(100); //Sets PID controller setpoint
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50)); //Delay 50ms
    }
}