#pragma once
#include "constants.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PID_Controller {
    public:
        PID_Controller(double Kp, double Ki, double Kd, mode_type_t mode);
        PID_Controller(double Kp, double Ki, double Kd, double dt);
        void set_dt(double dt);
        double get_dt(void);
        void set_setpoint(double setpoint, uint8_t type);
        void set_setpoint(double setpoint);
        double get_setpoint(void);
        double get_output(void);
        bool get_type(void);
        void set_output_limits(double min, double max);
        void compute(double input);
        uint8_t get_mode(void);

    private:
        double kp, ki, kd;
        double k1, k2, k3;
        double dt;
        double setpoint = 0;
        bool type = RPM;
        double error[3] = {0, 0, 0};
        double output[2] = {0, 0};
        double min = -100;
        double max = 100;
        mode_type_t mode = SPEED;

        double constrain(double value, double min, double max);
};