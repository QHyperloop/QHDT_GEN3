/*
 * bms.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "bms.h"
uint8_t lvbms_tx_data[7];
uint8_t lvbms_rx_data[];
#define lv_start 0xDD;
#define lv_read 0xA5;
#define lv_info 0x03;
#define lv_volt 0x04;


unsigned int checksum(unsigned int *data){
	uint16_t checksum = 0x0000;
	uint8_t length = data[3];
	for(int i = 2; i < length + 4; i++){
		checksum += data[i];
	}
	checksum = ~checksum + 1;

	return checksum;
}

void data_in(){

	uint8_t lvrx_length = sizeof(lvbms_rx_data[])/sizeof(lvbms_rx_data[0]);
	uint16_t lvrx_checksum = ((uint16_t)lvbms_rx_data[lvrx_length-3]<<8) |(uint16_t)lvbms_rx_data[lvrx_length-2];
	if(checksum(lvbms_rx_data) != lvrx_checksum){
		//error retry message
	}
}
