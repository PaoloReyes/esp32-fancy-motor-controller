#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "utils.h"

typedef enum {
    RPM,
    RADS
} speed_type_t;

/// @brief Motor class
class Motor {
    public:
        Motor(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer);
        void set_power(int8_t speed);

    private:
        gpio_num_t in_1;
        gpio_num_t in_2;
        gpio_num_t en_pin;
        ledc_channel_config_t motor_pwm_channel;
        ledc_timer_config_t motor_pwm_timer;
};

class Motor_with_Encoder : public Motor {
    public:
        Motor_with_Encoder(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer, gpio_num_t encoder_a, gpio_num_t encoder_b, double ratio, double CPR, int dt);
        int get_encoder_count(void);
        double get_speed(uint8_t type);

    private:
        gpio_num_t encoder_a;
        gpio_num_t encoder_b;
        int32_t encoder_count = 0;
        bool encoder_state[2];
        bool prev_encoder_state[2] = {false, false};
        float ratio;
        double CPR;
        int dt_ms;
        double speed;

        void reset_encoder_count(void);
        void calculate_speed(void);
        static void calculate_speed_wrapper(TimerHandle_t motor_obj);
        void encoder_isr(void);
        static void encoder_isr_wrapper(void* motor_obj);
        uint8_t boolean_encoder_2_uint8_t(bool* arr);
};