#include "main.h"

extern error_handler TEMP_INIT();
extern error_handler UPDATE_TEMP();

#define ADC0_1_MAX_TEMP		250  //25.0 = 250, 25.3 = 253
#define ADC0_1_MIN_TEMP		0
#define ADC0_2_MAX_TEMP		250
#define ADC0_2_MIN_TEMP		0
#define ADC0_3_MAX_TEMP		250
#define ADC0_3_MIN_TEMP		0
#define ADC0_4_MAX_TEMP		250
#define ADC0_4_MIN_TEMP		0

#define ADC1_1_MAX_TEMP		250  //25.0 = 250, 25.3 = 253
#define ADC1_1_MIN_TEMP		0
#define ADC1_2_MAX_TEMP		250
#define ADC1_2_MIN_TEMP		0
#define ADC1_3_MAX_TEMP		250
#define ADC1_3_MIN_TEMP		0
#define ADC1_4_MAX_TEMP		250
#define ADC1_4_MIN_TEMP		0

#define MCP_0_MAX_TEMP		250
#define MCP_0_MIN_TEMP		0
#define MCP_1_MAX_TEMP		250
#define MCP_1_MIN_TEMP		0
#define MCP_2_MAX_TEMP		250
#define MCP_2_MIN_TEMP		0
#define MCP_3_MAX_TEMP		250
#define MCP_3_MIN_TEMP		0

#define ADC5_2_MAX_PRESSURE	120.0f
#define ADC5_2_MIN_PRESSURE 60.0f