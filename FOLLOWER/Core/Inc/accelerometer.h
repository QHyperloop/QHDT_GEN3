/*
 * accelerometer.h
 *
 *  Created on: Jan 29, 2024
 *      Author: Patrick
 */

#ifndef SRC_ACCELEROMETER_H_
#define SRC_ACCELEROMETER_H_

#define IMU_ID ((uint8_t)0x28)

#include "stm32g4xx_hal.h"
#include "main.h"
#include "bno055.h"
#include "bno_config.h"
#include <stdbool.h>
#include <stdio.h>

extern error_handler acc_init(void);

#endif /* SRC_ACCELEROMETER_H_ */
