/*
 * temperature.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#include "temperature.h"
#include <math.h>

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
volatile int32_t temps[8] = {0};
volatile int32_t pressure[1] = {0};


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
	status = mcp9600_set_adc_resolution(handle,MCP9600_ADC_RESOLUTION_12_BIT);
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

uint8_t Update_Temp(){
	uint8_t status;
	int16_t hot_raw;
	float hot_s;
	int16_t delta_raw;
	float delta_s;
	int16_t cold_raw;
	float cold_s;

	if(adc1_convert == 1){ //when adc is done convert value to degree C and add to global temp variable
		for(int i = 0; i < adc1_count; i++){
			int32_t raw = adc1_result_dma[i];
			int32_t pow2 = raw*raw;
			temps[i] = (((int32_t)((((int64_t)pow2*raw>>16)*76633531) >> 16)+(int32_t)(((int64_t)pow2*-4493757)>>16)+(raw*208775)+ -126577781)>>16);
			//t=c3*adc^3+c2*adc^2+c1*adc+b  c3 = 2.72257E-07 c2 = -0.001046284 c1 = 3.185654692 b = -1931.423651
		}
		adc1_convert = 0;
	}else if(adc1_convert == 0){
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_result_dma,adc1_count); //starts adc conversion sets convert to 2, indicates running prevents restarting while running.
		adc1_convert = 2;
	}
	status = mcp9600_read(&MCP_0,(int16_t *)&hot_raw, (float *)&hot_s, (int16_t *)&delta_raw, (float *)&delta_s, (int16_t *)&cold_raw, (float *)&cold_s);
	temps[4] = (int32_t)roundf(hot_s*10.0f);
	if(status !=0){
		return 1; //read fail
	}

	status = mcp9600_read(&MCP_1,(int16_t *)&hot_raw, (float *)&hot_s, (int16_t *)&delta_raw, (float *)&delta_s, (int16_t *)&cold_raw, (float *)&cold_s);
	temps[5] = (int32_t)roundf(hot_s*10.0f);
	if(status !=0){
		return 1; //read fail
	}

	if(temps[0] > ADC1_1_MAX_TEMP || temps[0] < ADC1_1_MIN_TEMP ){
		return 1; //temp fault
	}
	if(temps[1] > ADC1_2_MAX_TEMP || temps[1] < ADC1_2_MIN_TEMP ){
		return 1; //temp fault
	}
	if(temps[2] > ADC1_3_MAX_TEMP || temps[2] < ADC1_3_MIN_TEMP ){
		return 1; //temp fault
	}
	if(temps[3] > ADC1_4_MAX_TEMP || temps[3] < ADC1_4_MIN_TEMP ){
		return 1; //temp fault
	}
	if((float)(temps[4]/16.0f) > MCP_0_MAX_TEMP || (float)(temps[4]/16.0f) < MCP_0_MIN_TEMP ){
		return 1; //temp fault
	}
	if((float)(temps[5]/16.0f) > MCP_1_MAX_TEMP || (float)(temps[5]/16.0f) < MCP_1_MIN_TEMP ){
		return 1; //temp fault
	}
	return 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){ // inidcates which adc is done running conversion and raises flag
		adc1_convert = 1;
	}
	if(hadc->Instance == ADC5){
		adc5_convert = 1;
	}

}

