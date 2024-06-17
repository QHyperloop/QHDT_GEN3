#include <pigpio.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <math.h>
#include <pthread.h>
#include <libwebsockets.h>
#include <json-c/json.h>

extern double sensors_data[6];
extern int Fault_Flag;
extern uint8_t ISO_STATE;
extern int sensor_flag;
extern int response_flag;
extern int success_or_fail;

typedef enum _error_handler {
	CAN_INIT_OK,
	CAN_INIT_ERR,
	CAN3_MSG_ERR,
	CAN2_MSG_ERR,
	CAN1_MSG_ERR,
	CAN_OK,
	ACC_INIT_OK,
	ACC_INIT_ERR,
	ACC_UNIT_ERR,
	MCP_ID_FAIL,
	MCP_CONFIG_FAIL,
	MCP_CONFIG_OK,
	MCP_TEMP_OK,
	MCP_READFAIL,
	TEMP_OK,
	ADS_CONFIG_FAIL,
	ADS_CONFIG_OK,
	ADS_TEMP_OK,
	ADS_READFAIL,
	TEMP_INIT_SUCCESS,
	TEMP_INIT_FAIL,
	OK,
	FAIL


} error_handler;
typedef enum _PodState{
	INIT,
	FAULT,
	SAFE_TO_APPROACH,
	READY,
	LAUNCH,
	COAST,
	BRAKE,
	CRAWL,
	TRACK
}PodState;
extern PodState Curr_State;
extern uint8_t ISO_STATE;


