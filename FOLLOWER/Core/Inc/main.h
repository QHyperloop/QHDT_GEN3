/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef enum{
	INIT,
	FAULT,
	SAFE_TO_APPROACH,
	READY,
	LAUNCH,
	COAST,
	BRAKE,
	CRAWL,
	TRACK
}PodState;

typedef enum _error_handler {
	CAN_INIT_OK,
	CAN_INIT_ERR,
	CAN3_MSG_ERR,
	CAN2_MSG_ERR,
	CAN1_MSG_ERR,
	CAN_OK,
	ACC_INIT_OK,
	ACC_INIT_ERR,
	ACC_UNIT_ERR,
	MCP_ID_FAIL,
	ERROR_MCP_REG,
	ERROR_MCP_THERMO,
	MCP_CONFIG_OK,
	MCP_TEMP_OK,
	MCP_0_READFAIL,
	MCP_1_READFAIL,
	ADC1_1_TEMPFAULT,
	ADC1_2_TEMPFAULT,
	ADC1_3_TEMPFAULT,
	ADC1_4_TEMPFAULT,
	MCP0_TEMPFAULT,
	MCP1_TEMPFAULT,
	TEMP_OK,
	ADC_CONVERT_FAIL,
	TEMP_INIT_SUCCESS,
	MCP_TEMP_FAIL


} error_handler;

extern PodState Curr_State;
extern uint8_t ISO_STATE;
extern volatile uint8_t Fault_Flag;

#include "relay.h"
#include "temperature.h"
#include "accelerometer.h"
#include "can.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
