#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)  //KEY0°´¼üPE4
uint8_t Key_Scan(void);
void key_init(void);

#endif

