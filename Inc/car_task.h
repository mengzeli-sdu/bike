#ifndef __CAR_TASK_H
#define __CAR_TASK_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "camera_protocol.h"

typedef enum
{
    CAR_NORMAL = 0,
    CAR_AVOID,
    CAR_CROSSWALK,
    CAR_FINISH
} CarState;

#define CAR_BASE_SPEED              1.0f
#define CAR_STOP_SPEED              0.0f

/* 人行道停车 30s */
#define CROSSWALK_TIME_MS           30000U

/* 避障距离阈值：单位近似为米。需要根据轮径和 ODrive 速度单位实车标定 */
#define WHEEL_CIRCUMFERENCE_M       2.05f
#define AVOID_EXIT_DISTANCE_M       2.50f

/* 事件冷却，避免同一个斑马线/锥桶重复触发 */
#define OBSTACLE_COOLDOWN_MS        2000U
#define CROSSWALK_COOLDOWN_MS       3000U

extern volatile CarState car_state;
extern volatile int16_t camera_error;
extern volatile float avoid_distance_m;

void Car_Task_Init(void);
void Car_2ms_Task(void);

/* upper.c 收到视觉帧后调用 */
void Car_ProcessCameraFrame(uint8_t msg_type, uint8_t seq, int16_t err, uint8_t side, uint8_t confidence);

/* 距离估计 */
void Avoid_Distance_Reset(void);
void Avoid_Distance_Update(void);

#endif
