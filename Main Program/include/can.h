#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "main.h"

extern uint8_t M_RPM[4];
extern uint8_t M_CURR[2];
extern uint8_t ISO_STATE;

extern int msg_wait();
extern void read_state_responses();
extern void IMD_REQ_ISO();
extern void set_esc_curr(uint8_t curr[4]);
extern void can_init();

