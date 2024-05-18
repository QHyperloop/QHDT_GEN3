#include "relay.h"

#define MCP23017_ADDRESS 0x20
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_OLATA 0x0A
#define MCP23017_OLATB 0x1A

int mcp[2];
#define I2C_BUS 0
#define I2C_FLAGS 0

#define RELAY1 0x01 //PD15 RED LED
#define RELAY2 0x02 //PD14 YELLOW LED
#define RELAY3 0x04 //PD13 GREEN LED
#define RELAY4 0x08 //PD12 BRAKE RELEASE
#define RELAY5 0x10 //PD11 PUMP 1
#define RELAY6 0x20 //PD10 PUMP 2
#define RELAY7 0x40 //PD9
#define RELAY8 0x80 //PD8
#define RELAY9 0x01 //PE7
#define RELAY10 0x02 //PE8
#define RELAY11 0x04 //PE9 MOTOR 0 LOWSIDE
#define RELAY12 0x08 //PE10 MOTOR 0 RESISTOR
#define RELAY13 0x10 //PE11 MOTOR 0 HIGHSIDE
#define RELAY14 0x20 //PE13 MOTOR 1 LOWSIDE
#define RELAY15 0x40 //PE14 MOTOR 1 RESISTOR
#define RELAY16 0x80 //PE15 MOTOR 1 HIGHSIDE


uint8_t RelayStatesA = 0x00;
uint8_t RelayStatesB = 0x00;

void relay_init(){
    

    mcp[0] = i2cOpen(I2C_BUS, MCP23017_ADDRESS | 0b000, I2C_FLAGS);
    i2cWriteByteData(mcp[0], MCP23017_IODIRA, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_IODIRB, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_OLATA, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_OLATB, 0x00);


}
//PORT MATCHES LETTER

void precharge(void){
 //high side on, low side precharge resitor on wait 500ms then low side main on, resistor off

    i2cWriteByteData(mcp[0], MCP23017_OLATB, RelayStatesB ^ RELAY9 ^ RELAY10 ^ RELAY14 ^ RELAY15);
	RelayStates = RelayStatesB ^ RELAY9 ^ RELAY10 ^ RELAY14 ^ RELAY15;
    clock_t start_time = clock();
    while (clock() < start_time+500);
    HV_on();
	
}
void HV_on(void){

    i2cWriteByteData(mcp[0], MCP23017_OLATB,  RelayStatesB ^ RELAY10 ^ RELAY11 ^ RELAY15 ^ RELAY16);
	RelayStatesB = RelayStatesB ^ RELAY10 ^ RELAY11 ^ RELAY15 ^ RELAY16;


}

void HV_off(void){

    i2cWriteByteData(mcp[0], MCP23017_OLATB, RelayStatesB ^ RELAY9 ^ RELAY11 ^ RELAY14 ^ RELAY16);

	RelayStatesB = RelayStatesB ^ RELAY9 ^ RELAY11 ^ RELAY14 ^ RELAY16;
}

void redstatus(uint8_t state){
	if(state == 1){
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY1);
		RelayStatesA = RelayStatesA ^ RELAY1;
	}else{
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY1);
		RelayStatesA = RelayStatesA ^ RELAY1;
	}

}

void yellowstatus(uint8_t state){
	if(state == 1){
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY2);
		RelayStatesA = RelayStatesA ^ RELAY2;
	}else{
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY2);
		RelayStatesA = RelayStatesA ^ RELAY2;
	}
}

void greenstatus(uint8_t state){
	if(state == 1){
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY3);
		RelayStatesA = RelayStatesA ^ RELAY3;
	}else{
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY3);
		RelayStatesA = RelayStatesA ^ RELAY3;
	}
}

void brake_state(uint8_t state){
	if(state == 1){
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY4);
		RelayStatesA = RelayStatesA ^ RELAY4;
	}else{
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY4);
		RelayStatesA = RelayStatesA ^ RELAY4;
	}
}

void pump_control(uint8_t state){
	if(state == 1){
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY5 ^ RELAY6);
		RelayStatesA = RelayStatesA ^ RELAY5 ^ RELAY6;
	}else{
		i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY5 ^ RELAY6);
		RelayStatesA = RelayStatesA ^ RELAY5 ^ RELAY6;
	}
}