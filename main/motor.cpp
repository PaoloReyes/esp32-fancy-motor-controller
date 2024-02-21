#include "motor.h"

/// @brief Creates a motor object
/// @param in_1 Input 1 pin
/// @param in_2 Input 2 pin
/// @param en_pin Enable pin
/// @param motor_pwm_channel PWM channel
/// @param motor_pwm_timer PWM timer
/// @param inverted Inverted motor direction (true: Inverted, false: Normal)
Motor::Motor(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer) {
    this->in_1 = in_1;
    this->in_2 = in_2;
    this->en_pin = en_pin;

    this->motor_pwm_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    this->motor_pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
    this->motor_pwm_timer.timer_num = motor_pwm_timer;
    this->motor_pwm_timer.freq_hz = 40000;
    this->motor_pwm_timer.clk_cfg = LEDC_AUTO_CLK;
    //this->motor_pwm_timer.deconfigure = false;

    this->motor_pwm_channel.gpio_num = en_pin;
    this->motor_pwm_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    this->motor_pwm_channel.channel = motor_pwm_channel;
    this->motor_pwm_channel.intr_type = LEDC_INTR_DISABLE;
    this->motor_pwm_channel.timer_sel = motor_pwm_timer;
    this->motor_pwm_channel.duty = 0;
    this->motor_pwm_channel.hpoint = 0;

    gpio_set_direction(this->in_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(this->in_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(this->en_pin, GPIO_MODE_OUTPUT);
    ledc_timer_config(&this->motor_pwm_timer);
    ledc_channel_config(&this->motor_pwm_channel);
}

/// @brief Sets the motor direction to inverted
void Motor::set_inverted(void) {
    this->inverted = !this->inverted;
}

/// @brief Gets the motor inverted status
bool Motor::get_inverted(void) {
    return this->inverted;
}

/// @brief Sets the power and direction of the motor
/// @param power power of the motor (-100<-->100)% duty cycle
void Motor::set_power(int8_t power) {
    power>100? power=100 : power=power;
    power<-100? power=-100 : power=power;
    this->inverted? this->power = -power : this->power = power;
    gpio_set_level(this->in_1, this->power>0);
    gpio_set_level(this->in_2, this->power<0);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->motor_pwm_channel.channel, map(abs(this->power), 0, 100, 0, (1<<this->motor_pwm_timer.duty_resolution)-1));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->motor_pwm_channel.channel);
}

/// @brief Gets the current power of the motor
/// @return Power of the motor
uint8_t Motor::get_power(void) {
    return abs(this->power);
}

/// @brief Sets the operative voltage of the motor
/// @param voltage Max voltage of the motor
void Motor::set_voltage(uint8_t voltage) {
    this->voltage = voltage;
}

/// @brief Gets the current voltage of the motor
/// @return Voltage of the motor
double Motor::get_voltage(void) {
    return (double)this->power*this->voltage/100;
}

/// @brief Gets the current direction of the motor
/// @return Direction of the motor (true: Counterclockwise, false: Clockwise)
bool Motor::get_direction(void) {
    return this->inverted? this->power>0 : this->power<0;
}

/// @brief Sets the open loop equation of the motor
/// @param m Slope of the equation
/// @param b Y-intercept of the equation
void Motor::set_open_loop_equation(double m, double b) {
    this->m = m;
    this->b = b;
}

/// @brief Sets the open loop speed of the motor
/// @param speed Speed of the motor
/// @param type Type of speed (RPM, RAD_S)
void Motor::set_open_loop_speed(double speed, uint8_t type) {
    this->set_power(this->m*speed+this->b);
}