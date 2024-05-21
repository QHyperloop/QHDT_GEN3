#include <pigpio.h>
#include <stdio.h>
#include "main.h"

volatile uint8_t Fault_Flag;
uint8_t ISO_STATE;
PodState Curr_State = INIT;


uint8_t Run_State(PodState state) {
	error_handler status = OK;

    switch (state) {
        case INIT: //Auto state
        	
        	if(TEMP_INIT() != TEMP_INIT_SUCCESS){
        		return 1;
        	}
        	can_init();
			
        	pump_control(1);
        	
        	Curr_State = SAFE_TO_APPROACH;
        	return status;
            break;
        case FAULT: // auto/manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	redstatus(1);
        	pump_control(0);
        	brake_state(0);

        	return status;
            break;
        case SAFE_TO_APPROACH: //manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	brake_state(1);

        	return status;
            break;
        case READY: //manual state
        	precharge();
        	yellowstatus(1);
        	brake_state(0);
        	return status;
            break;
        case LAUNCH: //manual state
        	yellowstatus(0);
        	greenstatus(1);

        	return status;
            break;
        case COAST: //auto state
        	yellowstatus(0);
        	greenstatus(1);

        	return status;
            break;
        case BRAKE: //auto state
        	yellowstatus(0);
        	greenstatus(1);
        	brake_state(1);

        	return status;
            break;
        case CRAWL: //auto state
        	yellowstatus(0);
        	greenstatus(1);
        	brake_state(0);

        	return status;
            break;
        case TRACK: //manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	brake_state(0);

        	return status;
            break;
        default:
        	Curr_State = FAULT;
        	//invalid state
            return 1;
            break;
    }
}






int main(void){
    //i2c init
    if (gpioInitialise() < 0)
    {
        Curr_State = FAULT;
    }

    Fault_Flag = Run_State(Curr_State);
  
  while (1)
  {
	if(msg_wait() < 0){
		printf("CAN Error");
	}
	Fault_Flag = Run_State(Curr_State);
	if(Fault_Flag != 0){
		Curr_State = FAULT;
	}
    
  }
}
