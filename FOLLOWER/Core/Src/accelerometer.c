/*
 * accelerometer.cpp
 *
 *  Created on: Jan 29, 2024
 *      Author: Patrick
 */

#include "accelerometer.h"


extern I2C_HandleTypeDef hi2c3;
bno055_t bno;
error_bno err;
s8 temperature = 0;
bno055_vec3_t acc = {0, 0, 0};
bno055_vec3_t lia = {0, 0, 0};
bno055_vec3_t gyr = {0, 0, 0};
bno055_vec3_t mag = {0, 0, 0};
bno055_vec3_t grv = {0, 0, 0};
bno055_euler_t eul = {0, 0, 0};
bno055_vec4_t qua = {0, 0, 0};

uint8_t acc_init(void){

    bno = (bno055_t){
    	.i2c = &hi2c3, .addr = 0x28, .mode = BNO_MODE_IMU, ._temp_unit = 0,
    };

	 if((err = bno055_init(&bno)) == BNO_OK){
	    printf("[+] BNO055 init success\r\n");
	 }else{
		 return 1;
	    //Error_Handler();
	 }

	 err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
	 if(err != BNO_OK) {
		 return 1;
	 }else{
		 printf("[BNO] Unit selection success\r\n");
	 }
	 return 0;

}
/*
 * bno.temperature(&bno, &temperature);
        bno.acc(&bno, &acc);
        bno.linear_acc(&bno, &lia);
        bno.gyro(&bno, &gyr);
        bno.mag(&bno, &mag);
        bno.gravity(&bno, &grv);
        bno.euler(&bno, &eul);
        bno.quaternion(&bno, &qua);
        printf("[+] Temperature: %2d°C\r\n", temperature);
        printf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y, acc.z);
        printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y, lia.z);
        printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y, gyr.z);
        printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y, mag.z);
        printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y, grv.z);
        printf("[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll, eul.pitch, eul.yaw);
        printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", qua.w, qua.x, qua.y, qua.z);
 */
