#include "car_task.h"
#include "task.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"

volatile CarState car_state = CAR_NORMAL;
volatile int16_t camera_error = 0;
volatile float avoid_distance_m = 0.0f;

static uint32_t crosswalk_start_tick = 0;
static uint32_t obstacle_ignore_until_tick = 0;
static uint32_t crosswalk_ignore_until_tick = 0;
static int16_t last_steer = 0;

static uint8_t last_obstacle_seq = 0;
static uint8_t last_crosswalk_seq = 0;
static uint8_t last_finish_seq = 0;

extern paramTypeDef param;
extern OdirveTypeDef odrive;

void Car_Task_Init(void)
{
    car_state = CAR_NORMAL;
    camera_error = 0;
    avoid_distance_m = 0.0f;

    crosswalk_start_tick = 0;
    obstacle_ignore_until_tick = 0;
    crosswalk_ignore_until_tick = 0;

    last_obstacle_seq = 0;
    last_crosswalk_seq = 0;
    last_finish_seq = 0;

    param.scope_flag = 1;     // 平衡开启
    param.run_flag = 1;       // 后轮允许运行
    odrive.set_speed1 = CAR_BASE_SPEED;

    servo_set_duty(0);
}

void Avoid_Distance_Reset(void)
{
    avoid_distance_m = 0.0f;
}

void Avoid_Distance_Update(void)
{
    /*
     * ODrive now_speed1 通常是 turn/s。
     * 距离 = 轮子转速(turn/s) * 轮周长(m/turn) * 控制周期(s)
     * 如果你的 ODrive 单位不是 turn/s，需要重新标定 WHEEL_CIRCUMFERENCE_M 或阈值。
     */
    avoid_distance_m += fabsf(odrive.now_speed1) * WHEEL_CIRCUMFERENCE_M * 0.002f;
}

int Line_Track_Control(int16_t err)
{
    float kp = 0.8f;
    float kd = 0.3f;
    static int16_t last_err = 0;

    int steer = (int)(kp * err + kd * (err - last_err));
    last_err = err;

    if(steer > 180) steer = 180;
    if(steer < -180) steer = -180;

    steer = Steer_Speed_Limit(steer, last_steer, 20, 1);
    last_steer = steer;

    return steer;
}

static void Car_Normal_Run(void)
{
    int steer;

    param.scope_flag = 1;
    param.run_flag = 1;
    odrive.set_speed1 = CAR_BASE_SPEED;

    steer = Line_Track_Control(camera_error);
    servo_set_duty(steer);
}

static void Car_Avoid_Run(void)
{
    int steer;

    param.scope_flag = 1;
    param.run_flag = 1;
    odrive.set_speed1 = CAR_BASE_SPEED;

    /* 避障期间仍然使用摄像头给出的误差循迹。摄像头端已经根据锥桶方向切换了跟踪线。 */
    steer = Line_Track_Control(camera_error);
    servo_set_duty(steer);

    Avoid_Distance_Update();

    if(avoid_distance_m >= AVOID_EXIT_DISTANCE_M)
    {
        Camera_RequestAvoidExit();       // 通知摄像头退出避障，upper.c 会重发直到摄像头 ACK
        Avoid_Distance_Reset();
        obstacle_ignore_until_tick = HAL_GetTick() + OBSTACLE_COOLDOWN_MS;
        car_state = CAR_NORMAL;
    }
}

static void Car_Crosswalk_Stop(void)
{
    param.scope_flag = 1;       // 平衡不能关
    param.run_flag = 0;
    odrive.set_speed1 = 0;

    servo_set_duty(0);

    if(HAL_GetTick() - crosswalk_start_tick >= CROSSWALK_TIME_MS)
    {
        crosswalk_ignore_until_tick = HAL_GetTick() + CROSSWALK_COOLDOWN_MS;
        param.run_flag = 1;
        car_state = CAR_NORMAL;
    }
}

static void Car_Finish_Stop(void)
{
    param.scope_flag = 1;       // 终点停车仍然保持平衡
    param.run_flag = 0;
    odrive.set_speed1 = 0;
    servo_set_duty(0);
}

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

    /* 处理需要周期重发的 MCU->摄像头命令，例如 AVOID_EXIT */
    Camera_PeriodicTx();
}

void Car_ProcessCameraFrame(uint8_t msg_type, uint8_t seq, int16_t err, uint8_t side, uint8_t confidence)
{
    (void)side;
    (void)confidence;

    if(msg_type == CAM_MSG_LINE)
    {
        camera_error = err;
        return;
    }

    /* 终点最高优先级：进入后不再被其他事件改变状态 */
    if(car_state == CAR_FINISH)
    {
        return;
    }

    if(msg_type == CAM_MSG_FINISH)
    {
        if(seq != last_finish_seq)
        {
            last_finish_seq = seq;
            camera_error = err;
            car_state = CAR_FINISH;
            param.run_flag = 0;
            odrive.set_speed1 = 0;
            servo_set_duty(0);
        }
        return;
    }

    if(msg_type == CAM_MSG_CROSSWALK)
    {
        if(HAL_GetTick() < crosswalk_ignore_until_tick)
        {
            return;
        }
        if(seq != last_crosswalk_seq && car_state != CAR_CROSSWALK)
        {
            last_crosswalk_seq = seq;
            camera_error = err;
            car_state = CAR_CROSSWALK;
            crosswalk_start_tick = HAL_GetTick();
            param.run_flag = 0;
            odrive.set_speed1 = 0;
            servo_set_duty(0);
        }
        return;
    }

    if(msg_type == CAM_MSG_OBSTACLE)
    {
        if(HAL_GetTick() < obstacle_ignore_until_tick)
        {
            return;
        }

        camera_error = err;

        /* 关键：同一个 obstacle seq 不能重复清零距离，否则永远退出不了避障 */
        if(seq != last_obstacle_seq && car_state == CAR_NORMAL)
        {
            last_obstacle_seq = seq;
            car_state = CAR_AVOID;
            Avoid_Distance_Reset();
        }
        return;
    }
}
