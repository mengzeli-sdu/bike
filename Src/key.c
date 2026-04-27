#include "key.h"

void key_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t Key_Scan(void)
{
	static uint8_t key_up=1;
	
	if(key_up && KEY0==0)
		{
			key_up=0;
			HAL_Delay(20);
			if(KEY0==0) return '1';
		}
	if(!(KEY0==0)) key_up=1;
	return 0;
}
