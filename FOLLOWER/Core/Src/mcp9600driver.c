/*
 * mcp9600driver.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: patri
 */

#include "mcp9600driver.h"

uint8_t mcp_temp_data[2];

error_handler MCP_INIT(uint8_t addr){
	uint16_t reg;
	uint16_t len_reg;
	uint8_t buf[1];
	uint8_t len_buf;


	//MCP ID CHECK

	reg = MCP9600_REG_DEVICE_ID_REVISON;
	len_reg = 1;
	buf[0] = 0;
	len_buf = 1;

	if(HAL_I2C_Mem_Read(&hi2c1,addr, reg, len_reg, buf ,len_buf,20) != HAL_OK){
			return MCP_ID_FAIL;
	}
	if(buf[0] != 0x40){
		return MCP_ID_FAIL;

	}
	//device config

	reg = MCP9600_REG_DEVICE_CONFIGURATION;
	len_reg = 1;
	buf[0] = 0b01100000;
	len_buf = 1;
	if(HAL_I2C_Mem_Write(&hi2c1, addr, reg, len_reg, buf ,len_buf ,20) != HAL_OK){
		return ERROR_MCP_REG;
	}

	if(HAL_I2C_Mem_Read(&hi2c1,addr, reg, len_reg, buf ,len_buf,20) != HAL_OK){
		return ERROR_MCP_REG;
	}

	if(buf[0] != 0b01100000){
		return ERROR_MCP_REG;
	}

	//device thermocouple

	reg = MCP9600_REG_THERMOCOUPLE_SENSOR_CONFIGURATION;
	len_reg = 1;
	buf[0] = 0b00000000;
	len_buf = 1;
	if(HAL_I2C_Mem_Write(&hi2c1, addr, reg, len_reg, buf ,len_buf ,20) != HAL_OK){
		return ERROR_MCP_THERMO;
	}


	if(HAL_I2C_Mem_Read(&hi2c1,addr, reg, len_reg, buf ,len_buf,20) != HAL_OK){
		return ERROR_MCP_THERMO;
	}

	if(buf[0] != 0b00000000){
		return ERROR_MCP_THERMO;
	}

	return MCP_CONFIG_OK;
}

error_handler MCP_TEMP_READ(uint8_t addr){
	uint16_t reg;
	uint16_t len_reg;
	uint8_t buf[2];
	uint8_t len_buf;

	reg = MCP9600_REG_THERMOCOUPLE_HOT_JUNCTION;
	len_reg = 1;
	buf[0] = 0;
	buf[1] = 0;
	len_buf = 2;

	if(HAL_I2C_Mem_Read(&hi2c1,addr, reg, len_reg, buf ,len_buf,20) != HAL_OK){
		return MCP_TEMP_FAIL;
	}
	mcp_temp_data[0] = buf[0];
	mcp_temp_data[1] = buf[1];
	return MCP_TEMP_OK;

}

