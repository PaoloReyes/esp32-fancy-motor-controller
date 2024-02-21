#pragma once
#include "freertos/FreeRTOS.h"
#include "pid_controller.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "utils.h"

class Motor {
    public:
        Motor(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer);
        void set_inverted(void);
        bool get_inverted(void);
        void set_power(int8_t power);
        uint8_t get_power(void);
        void set_voltage(uint8_t voltage);
        double get_voltage(void);
        bool get_direction(void);

    private:
        gpio_num_t in_1;
        gpio_num_t in_2;
        gpio_num_t en_pin;
        uint8_t voltage = 12;
        bool inverted = false;
        int8_t power = 0;
        ledc_channel_config_t motor_pwm_channel;
        ledc_timer_config_t motor_pwm_timer;
};

class Motor_with_Encoder : public Motor {
    public:
        Motor_with_Encoder(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer, gpio_num_t encoder_a, gpio_num_t encoder_b, double ratio, double CPR, int dt);
        double get_speed(uint8_t type);
        double get_speed(void);
        void set_pid_controller(PID_Controller* pid_controller);

    private:
        gpio_num_t encoder_a;
        gpio_num_t encoder_b;
        float ratio;
        double CPR;
        int dt_ms;
        PID_Controller* pid_controller;

        int32_t encoder_count = 0;
        bool encoder_state[2];
        bool prev_encoder_state[2];
        double speed;

        void calculate_speed(void);
        static void calculate_speed_wrapper(TimerHandle_t motor_obj);
        void encoder_isr(void);
        static void encoder_isr_wrapper(void* motor_obj);
        uint8_t boolean_encoder_2_uint8_t(bool* arr);
};