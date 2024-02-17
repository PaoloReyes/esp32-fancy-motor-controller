#include "motor.h"

/// @brief Creates a motor object
/// @param in_1 Input 1 pin
/// @param in_2 Input 2 pin
/// @param en_pin Enable pin
/// @param motor_pwm_channel PWM channel
/// @param motor_pwm_timer PWM timer
Motor::Motor(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer) {
    this->in_1 = in_1;
    this->in_2 = in_2;
    this->en_pin = en_pin;

    this->motor_pwm_channel.gpio_num = en_pin;
    this->motor_pwm_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    this->motor_pwm_channel.channel = motor_pwm_channel;
    this->motor_pwm_channel.intr_type = LEDC_INTR_DISABLE;
    this->motor_pwm_channel.timer_sel = motor_pwm_timer;
    this->motor_pwm_channel.duty = 0;

    this->motor_pwm_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    this->motor_pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
    this->motor_pwm_timer.timer_num = motor_pwm_timer;
    this->motor_pwm_timer.freq_hz = 40000;

    gpio_set_direction(this->in_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(this->in_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(this->en_pin, GPIO_MODE_OUTPUT);
    ledc_channel_config(&this->motor_pwm_channel);
    ledc_timer_config(&this->motor_pwm_timer);
}


/// @brief Sets the speed and direction of the motor
/// @param speed Speed of the motor (-100<-->100)% duty cycle
void Motor::set_power(int8_t speed) {
    speed>100? speed=100 : speed=speed;
    speed<-100? speed=-100 : speed=speed;
    gpio_set_level(this->in_1, speed>0);
    gpio_set_level(this->in_2, speed<0);
    
    speed = abs(speed);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->motor_pwm_channel.channel, map(speed, 0, 100, 0, (1<<this->motor_pwm_timer.duty_resolution)-1));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->motor_pwm_channel.channel);
}