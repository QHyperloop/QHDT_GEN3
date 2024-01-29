/*
 * accelerometer.cpp
 *
 *  Created on: Jan 29, 2024
 *      Author: Patrick
 */

#include "accelerometer.h"
#include "stm32g4xx_hal.h"
#include "bno055.h"
#include "bno_config.h"
#include <stdbool.h>
#include <stdio.h>

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

void accelerometer_init(void){

    bno = (bno055_t){
    	.i2c = &hi2c3, .addr = 0x28, .mode = BNO_MODE_IMU, ._temp_unit = 0,
    };

	 if((err = bno055_init(&bno)) == BNO_OK){
	    printf("[+] BNO055 init success\r\n");
	 }else{
	    printf("[!] BNO055 init failed\r\n");
	    printf("%s\n", bno055_err_str(err));
	    //Error_Handler();
	 }

	 err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
	 if(err != BNO_OK) {
		 printf("[BNO] Failed to set units. Err: %d\r\n", err);
	 }else{
		 printf("[BNO] Unit selection success\r\n");
	 }

}
