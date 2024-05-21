#include "esc.h"

uint8_t current[4];
double previous_curr = 0.0;


typedef struct {
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain
    double previous_error;  // Previous error value
    double integral;  // Integral value
} PIDController;

void PID_Init(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

double PID_Compute(PIDController *pid, double set_rpm, double actual_rpm, double dt) {
    double error = set_rpm - actual_rpm;
    pid->integral += error * dt;
    double derivative = (error - pid->previous_error) / dt;
    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->previous_error = error;
    return output;
}

void pid_init() {
    PIDController pid;
    double kp = 1.0;  // Proportional gain
    double ki = 0.1;  // Integral gain
    double kd = 0.01; // Derivative gain
    
    PID_Init(&pid, kp, ki, kd);

    return 0;
}

void update_esc(actual_rpm, setpoint_rpm){
    double dt = 1.0;

    double new_curr = previous_curr + PID_Compute(&pid, setpoint_rpm, actual_rpm, dt);
    previous_curr = new_curr;
    new_curr = new_curr*10;
    
    current[0] = (unsigned int)new_curr & 0xFF;
    current[1] = (unsigned int)new_curr>>8 & 0xFF;
    current[2] = (unsigned int)new_curr>>16 & 0xFF;
    current[3] = (unsigned int)new_curr>>24 & 0xFF;
    set_esc_curr(uint8_t currrent[4]);

};