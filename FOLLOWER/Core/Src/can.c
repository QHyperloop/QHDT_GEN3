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
FDCAN_RxHeaderTypeDef RxHeader_Master; //can line 1,2
FDCAN_RxHeaderTypeDef RxHeader_Pod; //can line 3
FDCAN_FilterTypeDef Filter_STATE_State;
FDCAN_FilterTypeDef Filter_STATE_Data;
FDCAN_FilterTypeDef Filter_ESC;
FDCAN_FilterTypeDef Filter_BATT;
FDCAN_FilterTypeDef Filter_STATE;
FDCAN_FilterTypeDef Filter_DATA;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
uint8_t RxData_Pod[8];
uint8_t RxData_Master[64];




error_handler CAN_INIT(){
	error_handler err;
	TxHeader_Master_State.Identifier = 0x10000000;
	TxHeader_Master_State.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Master_State.TxFrameType = FDCAN_DATA_FRAME;
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

	Filter_ESC.IdType = FDCAN_EXTENDED_ID;
	Filter_ESC.FilterIndex = 0;
	Filter_ESC.FilterType = FDCAN_FILTER_MASK;
	Filter_ESC.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	Filter_ESC.FilterID1 = 0x000000FF;
	Filter_ESC.FilterID2 = ESC_ID;

	Filter_BATT.IdType = FDCAN_EXTENDED_ID;
	Filter_BATT.FilterIndex = 1;
	Filter_BATT.FilterType = FDCAN_FILTER_DUAL;
	Filter_BATT.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	Filter_BATT.FilterID1 = IMD_ID;
	Filter_BATT.FilterID2 = BMS_ID;

	Filter_STATE.IdType = FDCAN_EXTENDED_ID;
	Filter_STATE.FilterIndex = 2;
	Filter_STATE.FilterType = FDCAN_FILTER_MASK;
	Filter_STATE.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	Filter_STATE.FilterID1 = 0x000000FF;
	Filter_STATE.FilterID2 = 0x00;

	Filter_DATA.IdType = FDCAN_EXTENDED_ID;
	Filter_DATA.FilterIndex = 3;
	Filter_DATA.FilterType = FDCAN_FILTER_MASK;
	Filter_DATA.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	Filter_DATA.FilterID1 = 0x000000FF;
	Filter_DATA.FilterID2 = FOLLOWER_ID;

	err = CAN_INIT_OK;
	if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if(HAL_FDCAN_Start(&hfdcan3)!= HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter_STATE) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &Filter_DATA) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &Filter_BATT) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &Filter_ESC) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
		err = CAN_INIT_ERR;
	}
	return err;

}

error_handler Set_ESC_CURR(uint8_t esc_id, uint8_t current[4]){
	error_handler err;
	err = CAN_OK;
	TxHeader_Pod.Identifier = 0x00000100 | esc_id;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_8;
	uint8_t temp_data[] = {current[3],current[2],current[1],current[0],0x00,0x00,0x00,0x00};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		err = CAN3_MSG_ERR;
	}
	return err;
}

error_handler Set_ESC_RPM(uint8_t esc_id, uint8_t RPM[4]){
	error_handler err;
	err = CAN_OK;
	TxHeader_Pod.Identifier = 0x00000300 | esc_id;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_8;
	uint8_t temp_data[] = {RPM[3],RPM[2],RPM[1],RPM[0],0x00,0x00,0x00,0x00};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		err = CAN3_MSG_ERR;
	}
	return err;
}

error_handler IMD_Req_Isolation(){
	error_handler err;
	err = CAN_OK;
	TxHeader_Pod.Identifier = IMD_ID;
	TxHeader_Pod.DataLength = FDCAN_DLC_BYTES_1;
	uint8_t temp_data[] = {0xE0};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Pod, temp_data)!= HAL_OK){
		err = CAN3_MSG_ERR;
	}
	return err;

}

error_handler State_Received(uint32_t state_r){
	error_handler err;
	err = CAN_OK;
	TxHeader_Master_State.Identifier = state_r | FOLLOWER_ID;
	TxHeader_Master_State.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_Master_State.DataLength = FDCAN_DLC_BYTES_1;
	uint8_t temp_data[] = {0x00};

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Master_State, temp_data)!= HAL_OK){
		err = CAN1_MSG_ERR;
	}
	return err;


}

error_handler Sensor_Data(){
	error_handler err;
	err = CAN_OK;
	TxHeader_Master_Data.Identifier = 0x01000100 | FOLLOWER_ID;
	TxHeader_Master_Data.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_Master_Data.DataLength = FDCAN_DLC_BYTES_16;
	uint8_t Temperature_Data[] = {};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Master_Data, Temperature_Data)!= HAL_OK){
		err = CAN1_MSG_ERR;
	}


	TxHeader_Master_Data.Identifier = 0x01000200 | FOLLOWER_ID;
	TxHeader_Master_Data.DataLength = FDCAN_DLC_BYTES_16;
	uint8_t ESC_Data[] = {};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Master_Data, ESC_Data)!= HAL_OK){
		err = CAN2_MSG_ERR;
	}


	TxHeader_Master_Data.Identifier = 0x01000300 | FOLLOWER_ID;
	TxHeader_Master_Data.DataLength = FDCAN_DLC_BYTES_2;
	uint8_t Relay_Data[] = {0x00,0x00};
	Relay_Data[0] = (RelayStates & 0xFF00) >> 8;
	Relay_Data[1] = (RelayStates & 0x00FF);
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Master_Data, Relay_Data)!= HAL_OK){
		err = CAN2_MSG_ERR;
	}


	TxHeader_Master_Data.Identifier = 0x01000400 | FOLLOWER_ID;
	TxHeader_Master_Data.DataLength = FDCAN_DLC_BYTES_16;
	uint8_t Batt_Data[] = {};
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Master_Data, Batt_Data)!= HAL_OK){
		err = CAN2_MSG_ERR;
	}
	return err;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_Master, RxData_Master) != HAL_OK){
		//Error_Handler();
	}
	if((RxHeader_Master.Identifier & 0x000000FF) == 0x00){
		if(RxHeader_Master.Identifier == 0x00001000){
			//init
			Curr_State = INIT;
		}
		if(RxHeader_Master.Identifier == 0x00001100){
			//Fault
			Curr_State = FAULT;
		}
		if(RxHeader_Master.Identifier == 0x00001200){
			//safe to approach
			Curr_State = SAFE_TO_APPROACH;
		}
		if(RxHeader_Master.Identifier == 0x00001300){
			//coast
			Curr_State = COAST;
		}
		if(RxHeader_Master.Identifier == 0x00001400){
			//brake
			Curr_State = BRAKE;
		}
		if(RxHeader_Master.Identifier == 0x00001500){
			//crawl
			Curr_State = CRAWL;
		}
		if(RxHeader_Master.Identifier == 0x00001600){
			//track
			Curr_State = TRACK;
		}
		if(RxHeader_Master.Identifier == 0x0000FF00){
			//launch
			Curr_State = LAUNCH;
		}
		if(RxHeader_Master.Identifier == 0x00001700){
			//ready to launch
			Curr_State = READY;
		}
	}
	if((RxHeader_Master.Identifier & 0x0000FFFF) == (0x0000FF00 | FOLLOWER_ID)){
		Sensor_Data();

	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
			//fault
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
			//fault
	}


}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader_Pod, RxData_Pod) != HAL_OK){
		//Error_Handler();
	}
// HE NE HU - HV LV IS1 ISO
	if(RxHeader_Pod.Identifier == IMD_ID){
		if((RxData_Pod[1] & 0x40) == 0x40){
			if((RxData_Pod[1] & 0x03) == 0b10){
				ISO_STATE = 0xF0; //Warning set LED to yellow
				IMD_Req_Isolation();
			}else if((RxData_Pod[1] & 0x03) == 0b11){
				ISO_STATE = 0xFF; //fault set LED to Red and full estop
				Curr_State = FAULT;
			}else{
				ISO_STATE = 0x00; //all good

			}
		}
	}else{
		//fault
	}
	if((RxHeader_Pod.Identifier & 0x000000FF) == BMS_ID){
		if((RxHeader_Pod.Identifier & 0x0000FF00) == 0x0900){
			//status 1
			M_RPM = (RxData_Pod[7]<<24) | (RxData_Pod[6]<<16) | (RxData_Pod[5]<<8) | (RxData_Pod[4]);
			M_Current = (RxData_Pod[3]<<8) | (RxData_Pod[2]);
		}
		if((RxHeader_Pod.Identifier & 0x0000FF00) == 0x0E00){
			//status 2
		}
		if((RxHeader_Pod.Identifier & 0x0000FF00) == 0x0F00){
			//status 3
		}
		if((RxHeader_Pod.Identifier & 0x0000FF00) == 0x1000){
			//status 4
			M_Temp_fet = (RxData_Pod[7]<<8) | (RxData_Pod[6]);
			M_Temp_motor = (RxData_Pod[5]<<8) | (RxData_Pod[4]);
			M_Current_in = (RxData_Pod[3]<<8) | (RxData_Pod[2]);
		}
		if((RxHeader_Pod.Identifier & 0x0000FF00) == 0x1B00){
			//status 5
		}
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
		//fault
	}

}


