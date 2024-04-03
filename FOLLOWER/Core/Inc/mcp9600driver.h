/*
 * mcp9600driver.h
 *
 *  Created on: Apr 2, 2024
 *      Author: patri
 */

#include "main.h"
#include "stm32g4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;

#define CHIP_NAME                 "Microchip MCP9600"        /**< chip name */
#define MANUFACTURER_NAME         "Microchip"                /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.7f                       /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        5.5f                       /**< chip max supply voltage */
#define MAX_CURRENT               2.5f                       /**< chip max current */
#define TEMPERATURE_MIN           -40.0f                     /**< chip min operating temperature */
#define TEMPERATURE_MAX           125.0f                     /**< chip max operating temperature */



#define MCP9600_REG_THERMOCOUPLE_HOT_JUNCTION                0x00        /**< thermocouple hot junction register */
#define MCP9600_REG_JUNCTIONS_TEMPERATURE_DELTA              0x01        /**< junctions temperature delta register */
#define MCP9600_REG_COLD_JUNCTION_TEMPERATURE                0x02        /**< cold junction temperature register */
#define MCP9600_REG_RAW_ADC_DATA                             0x03        /**< raw adc data register */
#define MCP9600_REG_STATUS                                   0x04        /**< status register */
#define MCP9600_REG_THERMOCOUPLE_SENSOR_CONFIGURATION        0x05        /**< thermocouple sensor configuration register */
#define MCP9600_REG_DEVICE_CONFIGURATION                     0x06        /**< device configuration register */
#define MCP9600_REG_ALERT1_CONFIGURATION                     0x08        /**< alert 1 configuration register */
#define MCP9600_REG_ALERT2_CONFIGURATION                     0x09        /**< alert 2 configuration register */
#define MCP9600_REG_ALERT3_CONFIGURATION                     0x0A        /**< alert 3 configuration register */
#define MCP9600_REG_ALERT4_CONFIGURATION                     0x0B        /**< alert 4 configuration register */
#define MCP9600_REG_ALERT1_HYSTERESIS                        0x0C        /**< alert 1 hysteresis register */
#define MCP9600_REG_ALERT2_HYSTERESIS                        0x0D        /**< alert 2 hysteresis register */
#define MCP9600_REG_ALERT3_HYSTERESIS                        0x0E        /**< alert 3 hysteresis register */
#define MCP9600_REG_ALERT4_HYSTERESIS                        0x0F        /**< alert 4 hysteresis register */
#define MCP9600_REG_TEMPERATURE_ALERT1_LIMIT                 0x10        /**< temperature alert 1 limit register */
#define MCP9600_REG_TEMPERATURE_ALERT2_LIMIT                 0x11        /**< temperature alert 2 limit register */
#define MCP9600_REG_TEMPERATURE_ALERT3_LIMIT                 0x12        /**< temperature alert 3 limit register */
#define MCP9600_REG_TEMPERATURE_ALERT4_LIMIT                 0x13        /**< temperature alert 4 limit register */
#define MCP9600_REG_DEVICE_ID_REVISON                        0x20        /**< device id/revision register */

extern error_handler MCP_INIT(uint8_t addr);
extern error_handler MCP_TEMP_READ(uint8_t addr);

extern uint8_t mcp_temp_data[2];


