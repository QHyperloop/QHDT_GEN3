/*
 * relay.h
 *
 *  Created on: Jan 20, 2024
 *      Author: patri
 */

#ifndef RELAY_H_
#define RELAY_H_

extern void precharge();
extern void HV_on();
extern void redstatus(int state);
extern void yellowstatus(int state);
extern void greenstatus(int state);

#endif /* RELAY_H_ */
