/*
 * can.h
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#ifndef SRC_CAN_H_
#define SRC_CAN_H_

#include "stm32g4xx_hal.h"
#include "main.h"
#include "relay.h"
#include "temperature.h"
#include "esc.h"

#define FOLLOWER_ID ((uint8_t)0x01)
#define ESC_ID ((uint8_t)0x65)
#define IMD_ID ((uint32_t)0xA100101)
#define BMS_ID ((uint32_t)0x99)

extern error_handler CAN_INIT();
extern error_handler IMD_Req_Isolation();


#endif /* SRC_CAN_H_ */
