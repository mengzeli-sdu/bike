#include "upper.h"
#include "usart.h"
#include "task.h"
#include "imu.h"
extern imu_t imu;
#define FHead 0xA5
int16_t delta_x_buf;//图像返回值
int16_t error_y;
uint8_t buf[1];
uint8_t buf_temp[1];
uint8_t state;
int cnttt;
uint8_t buf2[4];
uint8_t ii = 0;
uint8_t low_speed_flag=0;
uint8_t head_buf=0;//记录帧头
uint8_t back_center_data[5] = {0xa5,0x01,0x01,0x00,0x01^0x01^0x00};
extern float distance;
void UART7_IRQHandler(void)
{
}

void back_center_send(void)
{
	static uint8_t back_center_i = 0;
	LL_USART_TransmitData8(UART7,back_center_data[back_center_i]);
	while((UART7->SR&0X40) == 0){};
	back_center_i++;
	back_center_i %= 5;
}
