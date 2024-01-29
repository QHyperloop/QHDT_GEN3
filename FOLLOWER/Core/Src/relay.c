/*
 * relay.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */
#include <stdlib.h>
#include "relay.h"
#include "stm32g4xx_hal.h"



#define RELAY1 atoi("GPIO_PIN_15") //PD15 RED LED
#define RELAY2 atoi("GPIO_PIN_14") //PD14 YELLOW LED
#define RELAY3 atoi("GPIO_PIN_13") //PD13 GREEN LED
#define RELAY4 atoi("GPIO_PIN_12") //PD12 BRAKE RELEASE
#define RELAY5 atoi("GPIO_PIN_11") //PD11 PUMP 1
#define RELAY6 atoi("GPIO_PIN_10") //PD10 PUMP 2
#define RELAY7 atoi("GPIO_PIN_9") //PD9
#define RELAY8 atoi("GPIO_PIN_8") //PD8
#define RELAY9 atoi("GPIO_PIN_7") //PE7
#define RELAY10 atoi("GPIO_PIN_8") //PE8
#define RELAY11 atoi("GPIO_PIN_9") //PE9 MOTOR 0 LOWSIDE
#define RELAY12 atoi("GPIO_PIN_10") //PE10 MOTOR 0 RESISTOR
#define RELAY13 atoi("GPIO_PIN_11") //PE11 MOTOR 0 HIGHSIDE
#define RELAY14 atoi("GPIO_PIN_13") //PE13 MOTOR 1 LOWSIDE
#define RELAY15 atoi("GPIO_PIN_14") //PE14 MOTOR 1 RESISTOR
#define RELAY16 atoi("GPIO_PIN_15") //PE15 MOTOR 1 HIGHSIDE

extern TIM_HandleTypeDef htim2;

//PORT MATCHES LETTER

void precharge(){
 //high side on, low side precharge resitor on wait 500ms then low side main on, resistor off
	HAL_GPIO_WritePin(GPIOE, RELAY14, GPIO_PIN_SET); //LOW SIDE ON
	HAL_GPIO_WritePin(GPIOE, RELAY9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, RELAY15, GPIO_PIN_SET); //RESISTOR ON
	HAL_GPIO_WritePin(GPIOE, RELAY10, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
}
void HV_on(){
	HAL_GPIO_WritePin(GPIOE, RELAY11, GPIO_PIN_SET); //HIGHSIDE ON
	HAL_GPIO_WritePin(GPIOE, RELAY16, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, RELAY15, GPIO_PIN_RESET); //RESISTOR OFF
	HAL_GPIO_WritePin(GPIOE, RELAY10, GPIO_PIN_RESET);

}

void HV_off(){
	HAL_GPIO_WritePin(GPIOE, RELAY11, GPIO_PIN_RESET); //HIGHSIDE ON
	HAL_GPIO_WritePin(GPIOE, RELAY16, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, RELAY14, GPIO_PIN_RESET); //LOW SIDE ON
	HAL_GPIO_WritePin(GPIOE, RELAY9, GPIO_PIN_RESET);
}

void redstatus(int state){
	if(state == 1){
		HAL_GPIO_WritePin(GPIOD, RELAY1, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, RELAY1, GPIO_PIN_RESET);
	}

}

void yellowstatus(int state){
	if(state == 1){
		HAL_GPIO_WritePin(GPIOD, RELAY2, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, RELAY2, GPIO_PIN_RESET);
	}
}

void greenstatus(int state){
	if(state == 1){
		HAL_GPIO_WritePin(GPIOD, RELAY3, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, RELAY3, GPIO_PIN_RESET);
	}
}

void brake_release(int state){
	if(state == 1){
		HAL_GPIO_WritePin(GPIOD, RELAY4, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, RELAY4, GPIO_PIN_RESET);
	}
}

void pump_active(int state){
	if(state == 1){
		HAL_GPIO_WritePin(GPIOD, RELAY5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RELAY6, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOD, RELAY5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RELAY6, GPIO_PIN_RESET);
	}
}

