/*
 * accelerometer.cpp
 *
 *  Created on: Jan 28, 2024
 *      Author: mihai
 */
#include <stdbool.h>
#include <stdio.h>

#include "accelerometer.h"
#include "bno055.h"
#include "bno_config.h"
#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;

bno055_t bno;
error_bno err;

bno055_vec3_t acc = {0, 0, 0};
bno055_vec3_t lia = {0, 0, 0};
bno055_vec3_t gyr = {0, 0, 0};
bno055_vec3_t mag = {0, 0, 0};
bno055_vec3_t grv = {0, 0, 0};
bno055_euler_t eul = {0, 0, 0};
bno055_vec4_t qua = {0, 0, 0};

void accelerometer_init() {
	bno = (bno055_t){
		.i2c = &hi2c3, .mode = BNO_MODE_IMU, .addr = 0x29, ._temp_unit = BNO_TEMP_UNIT_C,
	};

	err = bno055_init(&bno);
	err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS, BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
}

int read_accelerometer_x() {
	bno.acc(&bno ,&acc);
	return acc.x;
}


