#include <stdint.h>
#include <stdlib.h>
#include <time.h>

extern void precharge(void);
extern void HV_on(void);
extern void HV_off(void);
extern void redstatus(uint8_t state);
extern void yellowstatus(uint8_t state);
extern void greenstatus(uint8_t state);
extern void brake_state(uint8_t state);
extern void pump_control(uint8_t state);

extern uint16_t RelayStates;
