#include "motor.h"

/// @brief Creates a motor with encoder object
/// @param in_1 Input 1 pin
/// @param in_2 Input 2 pin
/// @param en_pin PWM pin
/// @param motor_pwm_channel PWM channel
/// @param motor_pwm_timer PWM timer
/// @param encoder_a Encoder A pin
/// @param encoder_b Encoder B pin
/// @param ratio Gear ratio
/// @param CPR Counts per revolution
/// @param dt_ms Time interval for speed calculation in ms
Motor_with_Encoder::Motor_with_Encoder(gpio_num_t in_1, gpio_num_t in_2, gpio_num_t en_pin, ledc_channel_t motor_pwm_channel, ledc_timer_t motor_pwm_timer, gpio_num_t encoder_a, gpio_num_t encoder_b, double ratio, double CPR, int dt_ms) : Motor(in_1, in_2, en_pin, motor_pwm_channel, motor_pwm_timer) {
    //Encoder setup
    this->encoder_a = encoder_a;
    this->encoder_b = encoder_b;
    gpio_config_t encoder_a_conf;
    encoder_a_conf.pin_bit_mask = (1ULL<<encoder_a) | (1ULL<<encoder_b);
    encoder_a_conf.mode = GPIO_MODE_INPUT;
    encoder_a_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    encoder_a_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    encoder_a_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&encoder_a_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(this->encoder_a, this->encoder_isr_wrapper, (void*)this);
    gpio_isr_handler_add(this->encoder_b, this->encoder_isr_wrapper, (void*)this);
    this->reset_encoder_count();
    this->encoder_state[0] = gpio_get_level(this->encoder_a);
    this->encoder_state[1] = gpio_get_level(this->encoder_b);

    //Encoder timer setup
    this->ratio = ratio;
    this->CPR = CPR;
    this->dt_ms = dt_ms;

    TimerHandle_t encoder_timer = xTimerCreate("encoder_timer", pdMS_TO_TICKS(this->dt_ms), pdTRUE, (void*)this, (TimerCallbackFunction_t)calculate_speed_wrapper);
    xTimerStart(encoder_timer, 0);
}

/// @brief Reference the motor with encoder object and calls the privte calculate_speed methods
/// @param motor_obj Motor with encoder object reference
void IRAM_ATTR Motor_with_Encoder::calculate_speed_wrapper(TimerHandle_t motor_obj) {
    Motor_with_Encoder* motor = (Motor_with_Encoder*)pvTimerGetTimerID(motor_obj);
    motor->calculate_speed();
}

/// @brief Updates the speed of the motor and resets the encoder count
void IRAM_ATTR Motor_with_Encoder::calculate_speed(void) {
    this->speed = (this->encoder_count*60000)/this->dt_ms/(this->ratio*this->CPR);
    this->encoder_count = 0;
}

/// @brief Gets the speed of the motor
/// @param type Type of speed to get (RPM, RADS)
/// @return Speed of the motor
double Motor_with_Encoder::get_speed(uint8_t type) {
    if (type == RPM) {
        return this->speed;
    } else if (type == RADS) {
        return this->speed*0.10472;
    }
    return 0;
}

/// @brief Gets the encoder count
/// @return Encoder count
int Motor_with_Encoder::get_encoder_count(void) {
    return this->encoder_count;
}

/// @brief Resets the encoder count
void Motor_with_Encoder::reset_encoder_count(void) {
    this->encoder_count = 0;
}

/// @brief Reference the motor with encoder object and calls the privte isr
/// @param motor_obj Motor with encoder object reference
void IRAM_ATTR Motor_with_Encoder::encoder_isr_wrapper(void* motor_obj) {
    Motor_with_Encoder* motor = (Motor_with_Encoder*)motor_obj;
    motor->encoder_isr();
}

/// @brief ISR for the encoder
void IRAM_ATTR Motor_with_Encoder::encoder_isr(void) {
    this->encoder_state[0] = gpio_get_level(this->encoder_a);
    this->encoder_state[1] = gpio_get_level(this->encoder_b);
    uint8_t uint8_t_encoder_state = boolean_encoder_2_uint8_t(this->encoder_state);
    switch (boolean_encoder_2_uint8_t(this->prev_encoder_state)) {
        case 0b00:
            if (uint8_t_encoder_state == 0b10) {
                this->encoder_count--;
            } else if (uint8_t_encoder_state == 0b01) {
                this->encoder_count++;
            }
            break;
        case 0b01:
            if (uint8_t_encoder_state == 0b00) {
                this->encoder_count--;
            } else if (uint8_t_encoder_state == 0b11) {
                this->encoder_count++;
            }
            break;
        case 0b10:
            if (uint8_t_encoder_state == 0b11) {
                this->encoder_count--;
            } else if (uint8_t_encoder_state == 0b00) {
                this->encoder_count++;
            }
            break;
        case 0b11:
            if (uint8_t_encoder_state == 0b01) {
                this->encoder_count--;
            } else if (uint8_t_encoder_state == 0b10) {
                this->encoder_count++;
            }
            break;
        default:
            break;
    }
    this->prev_encoder_state[0] = this->encoder_state[0];
    this->prev_encoder_state[1] = this->encoder_state[1];
}

/// @brief Translates the encoder state to a string
/// @param arr Encoder state array
uint8_t Motor_with_Encoder::boolean_encoder_2_uint8_t(bool* arr) {
    return (arr[0] ? 2 : 0) + (arr[1] ? 1 : 0);
}