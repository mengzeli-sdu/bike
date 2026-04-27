#include "car_task.h"
#include "task.h"
#include "odrive.h"
#include "servo.h"
#include "usart.h"
#include "tim.h"
#include "math.h"

volatile CarState car_state = CAR_NORMAL;
volatile int16_t camera_error = 0;
volatile uint32_t avoid_encoder_count = 0;

static uint32_t crosswalk_start_tick = 0;
static int16_t last_steer = 0;

extern paramTypeDef param;
extern OdirveTypeDef odrive;
extern TIM_HandleTypeDef htim4;

/************************************************
 * 初始化车辆任务
 ************************************************/
void Car_Task_Init(void)
{
    car_state = CAR_NORMAL;

    camera_error = 0;
    avoid_encoder_count = 0;

    param.scope_flag = 1;     // 开启平衡
    param.run_flag = 1;       // 开启后轮运行

    servo_set_duty(0);        // 舵机回中

    Encoder_Reset();
}

/************************************************
 * 编码器清零
 * 假设 TIM4 被配置为编码器模式
 ************************************************/
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    avoid_encoder_count = 0;
}

/************************************************
 * 避障状态下更新路程
 ************************************************/
void Encoder_UpdateAvoidDistance(void)
{
    static int16_t last_encoder = 0;

    int16_t now_encoder = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = now_encoder - last_encoder;

    last_encoder = now_encoder;

    if(delta < 0)
    {
        delta = -delta;
    }

    avoid_encoder_count += delta;
}

/************************************************
 * 舵机循迹控制
 * 摄像头传来的 camera_error:
 * > 0 表示线在右边，需要右转
 * < 0 表示线在左边，需要左转
 ************************************************/
int Line_Track_Control(int16_t err)
{
    float kp = 0.8f;
    float kd = 0.3f;

    static int16_t last_err = 0;

    int steer;

    steer = (int)(kp * err + kd * (err - last_err));
    last_err = err;

    // 舵机限幅，避免打角太猛
    if(steer > 180) steer = 180;
    if(steer < -180) steer = -180;

    // 舵机变化率限制
    steer = Steer_Speed_Limit(steer, last_steer, 20, 1);
    last_steer = steer;

    return steer;
}

/************************************************
 * 正常循迹动作
 ************************************************/
void Car_Normal_Run(void)
{
    int steer;

    param.scope_flag = 1;             // 平衡保持开启
    param.run_flag = 1;               // 后轮允许运行
    odrive.set_speed1 = CAR_BASE_SPEED;

    steer = Line_Track_Control(camera_error);
    servo_set_duty(steer);
}

/************************************************
 * 避障动作
 * 重点：避障时仍然循迹，同时编码器记录路程
 ************************************************/
void Car_Avoid_Run(void)
{
    int steer;

    param.scope_flag = 1;
    param.run_flag = 1;
    odrive.set_speed1 = CAR_BASE_SPEED;

    // 仍然循迹
    steer = Line_Track_Control(camera_error);
    servo_set_duty(steer);

    // 记录避障距离
    Encoder_UpdateAvoidDistance();

    // 路程达到阈值，退出避障
    if(avoid_encoder_count >= AVOID_EXIT_COUNT)
    {
        Camera_SendCmd(MCU_CMD_AVOID_EXIT);   // 通知摄像头退出避障
        Encoder_Reset();                      // 编码器计数清零
        car_state = CAR_NORMAL;               // 回到正常循迹
    }
}

/************************************************
 * 人行道停车动作
 * 重点：后轮速度为 0，但是平衡仍然打开
 ************************************************/
void Car_Crosswalk_Stop(void)
{
    param.scope_flag = 1;         // 平衡不能关
    param.run_flag = 0;           // 后轮停止
    odrive.set_speed1 = 0;        // 后轮目标速度清零

    servo_set_duty(0);            // 舵机回中，原地平衡

    if(HAL_GetTick() - crosswalk_start_tick >= CROSSWALK_TIME_MS)
    {
        param.run_flag = 1;
        car_state = CAR_NORMAL;
    }
}

/************************************************
 * 终点永久停车
 * 重点：永远不再启动后轮，但平衡继续保持
 ************************************************/
void Car_Finish_Stop(void)
{
    param.scope_flag = 1;         // 平衡继续开
    param.run_flag = 0;           // 后轮禁止运行

    odrive.set_speed1 = 0;        // 后轮速度清零
    servo_set_duty(0);            // 舵机回中
}

/************************************************
 * 2ms 车辆状态机
 * 建议放在 TIM3 中断里面调用
 ************************************************/
void Car_2ms_Task(void)
{
    switch(car_state)
    {
        case CAR_NORMAL:
            Car_Normal_Run();
            break;

        case CAR_AVOID:
            Car_Avoid_Run();
            break;

        case CAR_CROSSWALK:
            Car_Crosswalk_Stop();
            break;

        case CAR_FINISH:
            Car_Finish_Stop();
            break;

        default:
            car_state = CAR_NORMAL;
            break;
    }
}

/************************************************
 * 摄像头数据处理
 ************************************************/
void Camera_ProcessFrame(uint8_t cmd, int16_t err)
{
    // 终点状态最高优先级，进入后不再响应其他指令
    if(car_state == CAR_FINISH)
    {
        return;
    }

    switch(cmd)
    {
        case CAM_CMD_LINE:
            camera_error = err;
            break;

        case CAM_CMD_AVOID:
            camera_error = err;
            car_state = CAR_AVOID;
            Encoder_Reset();
            break;

        case CAM_CMD_CROSSWALK:
            car_state = CAR_CROSSWALK;
            crosswalk_start_tick = HAL_GetTick();

            param.run_flag = 0;
            odrive.set_speed1 = 0;
            servo_set_duty(0);
            break;

        case CAM_CMD_FINISH:
            car_state = CAR_FINISH;

            param.run_flag = 0;
            odrive.set_speed1 = 0;
            servo_set_duty(0);
            break;

        default:
            break;
    }
}

/************************************************
 * 单片机发命令给摄像头
 ************************************************/
void Camera_SendCmd(uint8_t cmd)
{
    uint8_t tx[3];

    tx[0] = 0xA5;
    tx[1] = cmd;
    tx[2] = tx[0] ^ tx[1];

    for(int i = 0; i < 3; i++)
    {
        LL_USART_TransmitData8(UART7, tx[i]);
        while((UART7->SR & 0x40) == 0);
    }
}