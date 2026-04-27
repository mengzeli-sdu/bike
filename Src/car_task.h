#ifndef __CAR_TASK_H
#define __CAR_TASK_H

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef enum
{
    CAR_NORMAL = 0,
    CAR_AVOID,
    CAR_CROSSWALK,
    CAR_FINISH
} CarState;

#define CAM_CMD_LINE        0x01
#define CAM_CMD_AVOID       0x02
#define CAM_CMD_CROSSWALK   0x03
#define CAM_CMD_FINISH      0x04

#define MCU_CMD_AVOID_EXIT  0x81

#define CAR_BASE_SPEED      1.0f      // 正常后轮速度
#define CAR_STOP_SPEED      0.0f

#define CROSSWALK_TIME_MS   30000     // 人行道等待 30s

// 这个值需要你实车标定
// 比如编码器累计 8000 个脉冲后，认为避障距离走完
#define AVOID_EXIT_COUNT    8000      

extern volatile CarState car_state;
extern volatile int16_t camera_error;
extern volatile uint32_t avoid_encoder_count;

void Car_Task_Init(void);
void Car_2ms_Task(void);
void Camera_ProcessFrame(uint8_t cmd, int16_t err);
void Camera_SendCmd(uint8_t cmd);
void Encoder_Reset(void);
void Encoder_UpdateAvoidDistance(void);

#endif