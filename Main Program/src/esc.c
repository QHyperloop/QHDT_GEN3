#include "esc.h"

uint8_t current[4];
double previous_curr = 0.0;
uint8_t dist_prev;

int run[][] = {
{0,100},{1,100},{2,100},{3,100},{4,100},{5,100},{6,100},{7,100},{8,100},{9,100},{10,100},
{11,100},{12,100},{13,100},{14,100},{15,100},{16,100},{17,100},{18,100},{19,100},{20,100},
{21,100},{22,100},{23,100},{24,100},{25,100},{26,100},{27,100},{28,100},{29,100},{30,100},
{31,100},{32,100},{33,100},{34,100},{35,100},{36,100},{37,100},{38,100},{39,100},{40,100},
{41,100},{42,100},{43,100},{44,100},{45,100},{46,100},{47,100},{48,100},{49,100},{50,100},
{51,-999}
}

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

void update_esc(actual_rpm, dist_curr){
    double dt = dist_curr-dist_prev;
    dist_prev = dist_curr;
    int setpoint_rpm = run[dist_curr,1]; 
    double new_curr = previous_curr + PID_Compute(&pid, setpoint_rpm, actual_rpm, dt);
    previous_curr = new_curr;
    new_curr = new_curr*10;
    
    current[0] = (unsigned int)new_curr & 0xFF;
    current[1] = (unsigned int)new_curr>>8 & 0xFF;
    current[2] = (unsigned int)new_curr>>16 & 0xFF;
    current[3] = (unsigned int)new_curr>>24 & 0xFF;
    set_esc_curr(uint8_t currrent[4]);

};