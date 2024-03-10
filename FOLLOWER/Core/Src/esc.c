/*
 * esc.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "esc.h"

uint32_t M_RPM;
uint16_t M_Current;
uint16_t M_Temp_fet;
uint16_t M_Temp_motor;
uint16_t M_Current_in;

// PID Controller parameters
uint32_t kp = 100;  // Proportional gain
uint32_t ki = 1;    // Integral gain
uint32_t kd = 10;   // Derivative gain

// PID variables
uint32_t setpoint = 500;
uint32_t integral = 0;
uint32_t prev_error = 0;

// Function to initialize PID controller
void initializePIDController() {
    integral = 0;
    prev_error = 0;
}

// Function to update PID controller and get control output
uint32_t updatePIDController(uint32_t current_value, uint32_t setpoint) {
    // Calculate error
    uint32_t error = setpoint - current_value;

    // Proportional term
    uint32_t proportional = kp * error;

    // Integral term
    integral += error;
    uint32_t integral_term = ki * integral;

    // Derivative term
    uint32_t derivative = kd * (error - prev_error);

    // PID output
    uint32_t output = proportional + integral_term + derivative;

    // Update previous error for the next iteration
    prev_error = error;

    return output;
}
