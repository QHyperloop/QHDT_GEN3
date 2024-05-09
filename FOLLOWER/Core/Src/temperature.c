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
static uint8_t MCP_0 = 0xC0;
static uint8_t MCP_1 = 0xCC;
volatile uint16_t adc1_result_dma[4];
volatile uint16_t adc5_result_dma[1];
const int adc1_count = sizeof(adc1_result_dma)/sizeof(adc1_result_dma[0]);
const int adc5_count = sizeof(adc5_result_dma)/sizeof(adc5_result_dma[0]);
volatile int adc1_convert = 0;
volatile int adc5_convert = 0;
volatile int32_t temps[8] = {0};
volatile int32_t pressure[1] = {0};






error_handler TEMP_INIT(){

	error_handler status;

	status = MCP_INIT(MCP_0);
	if(status != MCP_CONFIG_OK){
		return status;
	}

	//status = MCP_INIT(MCP_1);
	if(status != MCP_CONFIG_OK){
		return status;
	}

/*
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_result_dma,adc1_count);
	while(adc1_convert == 0){
		//will hold until ready used to make sure all temp sensors are connected
	}
	adc1_convert = 0;
	for(int i = 0; i < adc1_count; i++){
		if(adc1_result_dma[i] <= 0){
			return ADC_CONVERT_FAIL;
		}
	}


	HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_result_dma,adc5_count);
	while(adc5_convert == 0){
			//will hold until ready used to make sure all pressure sensors are connected
	}
	adc5_convert = 0;
	if(adc5_result_dma[0] <= 0){
		return ADC_CONVERT_FAIL;
	}*/
	return TEMP_INIT_SUCCESS;
}

error_handler UPDATE_TEMP(){

	/*
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
	}*/


	if(MCP_TEMP_READ(MCP_0) != MCP_TEMP_OK){
		return MCP_0_READFAIL; //read fail
	}
	temps[4] = (int32_t)roundf((mcp_temp_data[0]*16 + mcp_temp_data[1]/16)*10.0f);

	//if(MCP_TEMP_READ(MCP_1) == MCP_TEMP_OK){
		//return MCP_1_READFAIL; //read fail
	//}
	temps[5] = (int32_t)roundf((mcp_temp_data[0]*16 + mcp_temp_data[1]/16)*10.0f);


	if(temps[0] > ADC1_1_MAX_TEMP || temps[0] < ADC1_1_MIN_TEMP ){
		return ADC1_1_TEMPFAULT; //temp fault
	}
	if(temps[1] > ADC1_2_MAX_TEMP || temps[1] < ADC1_2_MIN_TEMP ){
		return ADC1_2_TEMPFAULT; //temp fault
	}
	if(temps[2] > ADC1_3_MAX_TEMP || temps[2] < ADC1_3_MIN_TEMP ){
		return ADC1_3_TEMPFAULT; //temp fault
	}
	if(temps[3] > ADC1_4_MAX_TEMP || temps[3] < ADC1_4_MIN_TEMP ){
		return ADC1_4_TEMPFAULT; //temp fault
	}
	if((float)(temps[4]/16.0f) > MCP_0_MAX_TEMP || (float)(temps[4]/16.0f) < MCP_0_MIN_TEMP ){
		return MCP0_TEMPFAULT; //temp fault
	}
	if((float)(temps[5]/16.0f) > MCP_1_MAX_TEMP || (float)(temps[5]/16.0f) < MCP_1_MIN_TEMP ){
		return MCP1_TEMPFAULT; //temp fault
	}
	return TEMP_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){ // inidcates which adc is done running conversion and raises flag
		adc1_convert = 1;
	}
	if(hadc->Instance == ADC5){
		adc5_convert = 1;
	}

}

