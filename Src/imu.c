#include "imu.h"
#include "imu_data_decode.h"
#include "packet.h"

uint8_t rxbuf;
imu_t	imu;
float alpha = 0.03f;

float low_pass_filter(float value);
void imu_init(void)
{
	imu_data_decode_init();
}

void imu_get(void)
{
	imu.pit = id0x91.eul[0];
	imu.rol = id0x91.eul[1];
	imu.yaw = id0x91.eul[2];
	if(imu.yaw<0) imu.yaw = id0x91.eul[2]+360;
	imu.vx = id0x91.gyr[0];
	imu.vy = id0x91.gyr[1];
	imu.vz = id0x91.gyr[2];	
	
	// 一阶低通滤波
	imu.vx = low_pass_filter(imu.vx);
}

void UART8_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_RXNE(UART8) && LL_USART_IsEnabledIT_RXNE(UART8))
	{
		rxbuf=LL_USART_ReceiveData8(UART8);
		packet_decode(rxbuf);
		LL_USART_EnableIT_RXNE(UART8);
	}
}

float low_pass_filter(float value)
{
  static float out_last = 0;
  float out;

  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
