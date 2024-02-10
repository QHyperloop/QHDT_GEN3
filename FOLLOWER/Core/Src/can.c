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







