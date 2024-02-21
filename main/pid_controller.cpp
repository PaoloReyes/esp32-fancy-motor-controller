#include "pid_controller.h"

/// @brief Creates a PID controller object
/// @param Kp Proportional gain
/// @param Ki Integral gain
/// @param Kd Derivative gain
/// @param dt Sampling time in seconds
PID_Controller::PID_Controller(double kp, double ki, double kd, double dt) {
    this->k1 = kp+ki*dt+kd/dt;
    this->k2 = -kp-2*kd/dt;
    this->k3 = kd/dt;
    this->dt = dt;
}

/// @brief PID controller object to be used in the motor with encoder object
/// @param kp Proportional gain
/// @param ki Integral gain
/// @param kd Derivative gain
PID_Controller::PID_Controller(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/// @brief Sets the sampling time of the PID controller
/// @param dt Sampling time in seconds
void PID_Controller::set_dt(double dt) {
    this->k1 = this->kp+this->ki*dt+this->kd/dt;
    this->k2 = -this->kp-2*this->kd/dt;
    this->k3 = this->kd/dt;
    this->dt = dt;
}

/// @brief Gets the sampling time of the PID controller
/// @return Sampling time in seconds
double PID_Controller::get_dt(void) {
    return this->dt;
}

/// @brief Sets the setpoint of the PID controller
/// @param setpoint Setpoint value
void PID_Controller::set_setpoint(double setpoint, uint8_t type) {
    this->type = type;
    if (this->type == RPM) {
        this->setpoint = setpoint;
    } else if (this->type == RADS) {
        this->setpoint = setpoint*9.5492965;
    }
}

/// @brief Gets the setpoint of the PID controller
/// @return Setpoint value
double PID_Controller::get_setpoint(void) {
    if (this->type == RPM) {
        return this->setpoint;
    } else if (this->type == RADS) {
        return this->setpoint/9.5492965;
    }
    return 0;
}

/// @brief Gets the output of the PID controller
/// @return Output value
double PID_Controller::get_output(void) {
    return this->output[0];
}

/// @brief Gets the type of units of the PID controller
/// @return Type of the PID controller
bool PID_Controller::get_type(void) {
    return this->type;
}

/// @brief Changes the limits of the PID controller
/// @param min Lower limit
/// @param max Upper limit
void PID_Controller::set_output_limits(double min, double max) {
    this->min = min;
    this->max = max;
}

/// @brief Computes the output of the PID controller
/// @param input Input value
void PID_Controller::compute(double input) {
    this->error[0] = this->setpoint - input;
    this->output[0] = constrain(this->k1 * this->error[0] + this->k2 * this->error[1] + this->k3 * this->error[2] + this->output[1], this->min, this->max);
    this->error[2] = this->error[1];
    this->error[1] = this->error[0];
    this->output[1] = this->output[0];
}

/// @brief Constrains the value between a minimum and maximum value
/// @param value Value to be constrained
/// @param min Lower limit
/// @param max Upper limit
/// @return Constrained value
double PID_Controller::constrain(double value, double min, double max) {
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    } else {
        return value;
    }
}