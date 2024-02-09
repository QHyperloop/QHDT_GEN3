/*
 * temperature.h
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 *
 */

#ifndef SRC_TEMPERATURE_H_
#define SRC_TEMPERATURE_H_

#include "driver_mcp9600_interface.h"
#include "stm32g4xx_hal.h"

extern uint8_t tempsensor_init();
extern uint8_t temp_update();
extern volatile int32_t temps[8];
extern volatile int32_t pressure[1];

#define ADC1_1_MAX_TEMP		250  //25.0 = 250, 25.3 = 253
#define ADC1_1_MIN_TEMP		0
#define ADC1_2_MAX_TEMP		250
#define ADC1_2_MIN_TEMP		0
#define ADC1_3_MAX_TEMP		250
#define ADC1_3_MIN_TEMP		0
#define ADC1_4_MAX_TEMP		250
#define ADC1_4_MIN_TEMP		0

#define MCP_0_MAX_TEMP		250
#define MCP_0_MIN_TEMP		0
#define MCP_1_MAX_TEMP		250
#define MCP_1_MIN_TEMP		0
#define MCP_2_MAX_TEMP		250
#define MCP_2_MIN_TEMP		0
#define MCP_3_MAX_TEMP		250
#define MCP_3_MIN_TEMP		0

#define ADC5_2_MAX_PRESSURE	120.0f
#define ADC5_2_MIN_PRESSURE 60.0f

#endif /* SRC_TEMPERATURE_H_ */
