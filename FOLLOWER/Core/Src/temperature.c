/*
 * temperature.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "temperature.h"


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc5;
static mcp9600_handle_t MCP_0;
static mcp9600_handle_t MCP_1;
volatile uint16_t adc1_result_dma[4];
volatile uint16_t adc5_result_dma[1];
const int adc1_count = sizeof(adc1_result_dma)/sizeof(adc1_result_dma[0]);
const int adc5_count = sizeof(adc5_result_dma)/sizeof(adc5_result_dma[0]);
volatile int adc1_convert = 0;
volatile int adc5_convert = 0;


uint8_t i2c_temp_init(mcp9600_handle_t *handle, mcp9600_address_t addr_pin, mcp9600_thermocouple_type_t type){
	uint8_t status;

	DRIVER_MCP9600_LINK_INIT(handle, mcp9600_handle_t);
	DRIVER_MCP9600_LINK_IIC_INIT(handle, mcp9600_interface_iic_init);
	DRIVER_MCP9600_LINK_IIC_DEINIT(handle, mcp9600_interface_iic_deinit);
	DRIVER_MCP9600_LINK_IIC_READ_COMMAND(handle, mcp9600_interface_iic_read_cmd);
	DRIVER_MCP9600_LINK_IIC_WRITE_COMMAND(handle, mcp9600_interface_iic_write_cmd);
	DRIVER_MCP9600_LINK_IIC_DELAY_MS(handle, mcp9600_interface_delay_ms);
	DRIVER_MCP9600_LINK_DEBUG_PRINT(handle, mcp9600_interface_debug_print);

	status = mcp9600_set_addr_pin(handle, addr_pin); //set i2c address
	if(status !=0){
		return 1;
	}
	status = mcp9600_init(handle); //chip init
	if(status !=0){
		return 1;
	}
	status = mcp9600_set_mode(handle, MCP9600_MODE_NORMAL); //set normal
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	status = mcp9600_set_filter_coefficient(handle, MCP9600_FILTER_COEFFICIENT_0); //set filter
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	status = mcp9600_set_thermocouple_type(handle, type); //thermocouple type
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	status = mcp9600_set_cold_junction_resolution(handle,MCP9600_COLD_JUNCTION_RESOLUTION_0P0625);
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	status = mcp9600_set_adc_resolution(handle,MCP9600_ADC_RESOLUTION_16_BIT);
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	status = mcp9600_set_adc_resolution(handle, MCP9600_BURST_MODE_SAMPLE_4);
	if(status !=0){
		(void)mcp9600_deinit(handle);
		return 1;
	}
	return 0;
}
uint8_t mcp9600_read(mcp9600_handle_t *handle, int16_t *hot_raw, float *hot_s,int16_t *delta_raw, float *delta_s, int16_t *cold_raw, float *cold_s){
	uint8_t status;
	status = mcp9600_single_read(handle, hot_raw, hot_s, delta_raw, delta_s, cold_raw, cold_s);
	if(status !=0){
		return 1;
	}
	return 0;
}

uint8_t tempsensor_init(){
	uint8_t status;
	status = i2c_temp_init(&MCP_0, MCP9600_ADDRESS_0, MCP9600_THERMOCOUPLE_TYPE_K);
	if(status !=0){
		return 1;
	}
	status = i2c_temp_init(&MCP_1, MCP9600_ADDRESS_1, MCP9600_THERMOCOUPLE_TYPE_K);
	if(status !=0){
		return 1;
	}
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_result_dma,adc1_count);
	while(adc1_convert == 0){
		//will hold until ready used to make sure all temp sensors are connected
	}
	adc1_convert = 0;
	for(int i = 0; i < adc1_count; i++){
		if(adc1_result_dma[i] <= 0){
			return 1;
		}
	}
	HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_result_dma,adc5_count);
	while(adc5_convert == 0){
			//will hold until ready used to make sure all pressure sensors are connected
	}
	adc5_convert = 0;
	if(adc5_result_dma[0] <= 0){
		return 1;
	}
	return 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){
		adc1_convert = 1;
	}
	if(hadc->Instance == ADC5){
		adc5_convert = 1;
	}

}

