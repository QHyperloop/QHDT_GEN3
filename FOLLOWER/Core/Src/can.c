/*
 * can.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "can.h"


FDCAN_TxHeaderTypeDef TxHeader_Master_State; //can line 1
FDCAN_TxHeaderTypeDef TxHeader_Master_Data; // can line 2
FDCAN_TxHeaderTypeDef TxHeader_Pod; //can line 3
FDCAN_RxHeaderTypeDef RxHeader_Master_State; //can line 1
FDCAN_RxHeaderTypeDef RxHeader_Master_Data; // can line 2
FDCAN_RxHeaderTypeDef RxHeader_Pod; //can line 3
FDCAN_FilterTypeDef Master_State;
FDCAN_FilterTypeDef Master_Data;
FDCAN_FilterTypeDef Pod;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
uint8_t RxData_Pod[8];
uint8_t ISO_STATE;


uint8_t CAN_INIT(){
	TxHeader_Master_State.Identifier = 0x10000000;
	TxHeader_Master_State.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Master_State.TxFrameType = FDCAN_REMOTE_FRAME;
	TxHeader_Master_State.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_Master_State.DataLength = FDCAN_DLC_BYTES_1;
	TxHeader_Master_State.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader_Master_State.FDFormat = FDCAN_FD_CAN;


	TxHeader_Master_Data.Identifier = 0x10000000;
	TxHeader_Master_Data.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Master_Data.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_Master_Data.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_Master_Data.DataLength = FDCAN_DLC_BYTES_1;
	TxHeader_Master_Data.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader_Master_Data.FDFormat = FDCAN_FD_CAN;

	TxHeader_Pod.Identifier = 0x10000000;
	TxHeader_Pod.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Pod.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_Pod.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_Pod.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_Pod.FDFormat = FDCAN_CLASSIC_CAN;

	if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK){
		return 1;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		return 1;
	}
	if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK){
		return 1;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		return 1;
	}
	if(HAL_FDCAN_Start(&hfdcan3)!= HAL_OK){
		return 1;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
		return 1;
	}
	return 0;

}

uint8_t Set_ESC_CURR(uint8_t esc_id, uint8_t current[4]){
	TxHeader_Pod.Identifier = 0x00000100 | esc_id;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_8;
	uint8_t temp_data[] = {current[3],current[2],current[1],current[0],0x00,0x00,0x00,0x00};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		return 1;
	}
	return 0;
}

uint8_t Set_ESC_RPM(uint8_t esc_id, uint8_t RPM[4]){
	TxHeader_Pod.Identifier = 0x00000300 | esc_id;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_8;
	uint8_t temp_data[] = {RPM[3],RPM[2],RPM[1],RPM[0],0x00,0x00,0x00,0x00};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		return 1;
	}
	return 0;
}

uint8_t IMD_Req_Isolation(){
	TxHeader_Pod.Identifier = 0xA100101;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_1;
	uint8_t temp_data[] = {0xE0};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		return 1;
	}
	return 0;

}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
		Error_Handler();
	}


}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader_Pod, RxData_Pod) != HAL_OK){
		Error_Handler();
	}
// HE NE HU - HV LV IS1 ISO
	if(RxHeader_Pod.Identifier == 0xA100101){
		if(RxData_Pod[1] & 0x03 == 0b10){
			ISO_STATE = 0xF0; //Warning set LED to yellow
			IMD_Req_Isolation();
		}else if(RxData_Pod[1] & 0x03 == 0b01){
			ISO_STATE = 0xFF; //fault set LED to Red and full estop
		}else{
			ISO_STATE = 0x00; //all good

		}
	}
}


