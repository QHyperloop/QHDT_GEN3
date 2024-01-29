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
extern void redstatus(int state);
extern void yellowstatus(int state);
extern void greenstatus(int state);
extern void brake_release(int state);
extern void pump_control(int state);

#endif /* RELAY_H_ */
