/*
 * can.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "can.h"


FDCAN_TxHeaderTypeDef TxHeader_Master_State;
FDCAN_TxHeaderTypeDef TxHeader_Master_Data;
FDCAN_TxHeaderTypeDef TxHeader_Pod;
FDCAN_FilterTypeDef Master_State;
FDCAN_FilterTypeDef Master_Data;
FDCAN_FilterTypeDef Pod;


void CAN_INIT(){
	TxHeader_Master_State.Identifier = 0x10000000;
	TxHeader_Master_State.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Master_State.TxFrameType = FDCAN_REMOTE_FRAME;
	TxHeader_Master_State.DataLength = FDCAN_DLC_BYTES_1;
	TxHeader_Master_State.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader_Master_State.FDFormat = FDCAN_FD_CAN;
}
uint8_t Data_1[8];
uint32_t TxMailbox1;




