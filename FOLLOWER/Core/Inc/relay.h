/*
 * relay.h
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#ifndef RELAY_H_
#define RELAY_H_

extern void precharge(void);
extern void HV_on(void);
extern void HV_off(void);
extern void redstatus(uint8_t state);
extern void yellowstatus(uint8_t state);
extern void greenstatus(uint8_t state);
extern void brake_state(uint8_t state);
extern void pump_control(uint8_t state);

#endif /* RELAY_H_ */
