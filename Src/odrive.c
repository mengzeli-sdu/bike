#include "odrive.h"
#include "can.h"
OdirveTypeDef odrive;
// can2  250 kbps  250000
HAL_StatusTypeDef status1;
HAL_StatusTypeDef status2;
void odrive_init(void)
{
    odrive.set_speed0 = 0;
    odrive.set_speed1 = 0;
    odrive.now_speed0 = 0;
    odrive.now_speed1 = 0;
    odrive.fliter_speed0[0] = 0;
    odrive.fliter_speed0[1] = 0;
    odrive.fliter_speed0[2] = 0;
    odrive.fliter_speed1[0] = 0;
    odrive.fliter_speed1[1] = 0;
    odrive.fliter_speed1[2] = 0;
    odrive.speed0_i = 0;
    odrive.speed1_i = 0;
		odrive_canFilter_init();
}
void odrive_canFilter_init(void)
{

	  CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan2, &filter);

    status = HAL_CAN_Start(&hcan2);


    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void odrive_speed_ctrl(unsigned char num, float speed)
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
	header.RTR = CAN_RTR_DATA;
	header.IDE = CAN_ID_STD;
	header.DLC = 8;
	header.StdId = ((NODE_ID(num) << 5) | MSG_SET_INPUT_VEL);
	header.ExtId=0;
	uint8_t *ptrSpeed = (uint8_t *)&speed;
	data[0] = ptrSpeed[0];
	data[1] = ptrSpeed[1];
	data[2] = ptrSpeed[2];
	data[3] = ptrSpeed[3];
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	uint32_t ret;
	status1 = HAL_CAN_AddTxMessage(&hcan2, &header, data, &ret);
}
// 0为飞轮 1为后轮
void odrive_vel_callback(unsigned char num)
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
	header.RTR = CAN_RTR_REMOTE;
	header.IDE = CAN_ID_STD;
	header.DLC = 0;
	header.StdId = ((NODE_ID(num) << 5) | MSG_GET_ENCODER_ESTIMATES);
	header.ExtId = 0;
	uint32_t ret;
	status2 = HAL_CAN_AddTxMessage(&hcan2, &header, data, &ret);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t buf[8];
	if(hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &header, buf);
		switch (header.StdId & 0x1F)
		{
			case (MSG_GET_ENCODER_ESTIMATES):
				if((header.StdId >> 5) == AXIS0_CAN_NODE_ID)
				{
					odrive.speed0_i = (++odrive.speed0_i) % 3;
					odrive.fliter_speed0[odrive.speed0_i] = *(float *)(buf + 4);
					odrive.now_speed0 = (odrive.fliter_speed0[0]+odrive.fliter_speed0[1]+odrive.fliter_speed0[2])/3;
				}
				else if((header.StdId >> 5) == AXIS1_CAN_NODE_ID)
				{
					odrive.speed1_i = (++odrive.speed1_i) % 3;
					odrive.fliter_speed1[odrive.speed1_i] = *(float *)(buf + 4);
					odrive.now_speed1 = (odrive.fliter_speed1[0]+odrive.fliter_speed1[1]+odrive.fliter_speed1[2])/3;	
				}
				break;
			default:
				break;
		}
		
	}
}

