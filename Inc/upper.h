#ifndef __UPPER_H__
#define __UPPER_H__

#include "main.h"
extern uint8_t buf[1];
extern int16_t delta_x_buf;
void upper_send(int steer,int mode);
extern int in_flag;
extern int16_t error_y;
extern uint8_t low_speed_flag;
void back_center_send(void);
#endif

