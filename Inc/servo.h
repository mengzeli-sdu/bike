#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "tim.h"


void servo_init(void);
void servo_set_duty(int duty);

int Steer_Speed_Limit(int now, int last, int limit, int times);
#endif


