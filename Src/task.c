#include "task.h"
#include "tim.h"
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"
#include "car_task.h"

#define FS 3
#if FS==1
    float fast_rate=8.0f,slow_rate=3.0f,mid_rate=5.0f;
#elif FS==2
    float fast_rate=10.0f,slow_rate=4.50f,mid_rate=6.50f;
#else
    float fast_rate=12.0f,slow_rate=5.50f,mid_rate=6.50f;
#endif

#define fly_wheel_rate_limit 15
#define PI 3.1415926f

paramTypeDef param;
float PWM_X, PWM_accel, PWM_Final;

extern int key_times;
extern imu_t imu;
extern OdirveTypeDef odrive;
extern uint8_t low_speed_flag;

int cnt;
int cnt1;
int cnt_vel_send;
int cnt_back_send;
int cnt_rate;
int cnt_odom;
float last_rate = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        imu_get();

        /* 上层车辆状态机：循迹、避障、人行道、终点 */
        Car_2ms_Task();

        cnt_vel_send++;
        cnt_back_send++;
        cnt_rate++;
        cnt_odom++;

        if(cnt_rate >= 50)  // 100ms
        {
            cnt_rate = 0;
            rate_set();
        }

        if(param.scope_flag == 1)
        {
            balance();
        }

        /* 每 20ms 请求一次 ODrive 编码器速度，否则 now_speed0/1 不会实时更新 */
        if(cnt_odom >= 10)
        {
            cnt_odom = 0;
            odrive_vel_callback(0);
            odrive_vel_callback(1);
        }

        /* 0 号轴：飞轮，2ms 发送一次 */
        if(cnt_vel_send >= 1)
        {
            cnt_vel_send = 0;
            odrive_speed_ctrl(0, odrive.set_speed0);
        }

        /* 1 号轴：后轮，40ms 发送一次 */
        if(cnt_back_send >= 20)
        {
            cnt_back_send = 0;
            odrive_speed_ctrl(1, -odrive.set_speed1);
        }
    }
}

void rate_set(void)
{
    if(car_state == CAR_FINISH || car_state == CAR_CROSSWALK)
    {
        odrive.set_speed1 = 0;
        param.run_flag = 0;
        return;
    }

    if(car_state == CAR_NORMAL || car_state == CAR_AVOID)
    {
        if(param.run_flag == 1)
        {
            odrive.set_speed1 = CAR_BASE_SPEED;
        }
        else
        {
            odrive.set_speed1 = 0;
        }
    }
}

void param_init(void)
{
    param.angular_kp = -7.6f;
    param.angular_ki = 0;
    param.angular_kd = -1.4f;

    param.angular_v_kp = 1.4f;
    param.angular_v_ki = 0;
    param.angular_v_kd = 1.1f;

    param.fly_wheel_speed_kp = -1.2f;
    param.fly_wheel_speed_ki = -0.6f;
    param.fly_wheel_speed_kd = 0;

    param.zero_speed_kp = 0;
    param.zero_speed_kd = 0;
    param.zero_speed_ki = 0;

    param.angular_zero = 2.1f;

    param.scope_flag = 0;
    param.run_flag = 0;

    param.Steer_Kp = 1.5f;
    param.Steer_Ki = 0.2f;
    param.Steer_Kd = 0;
}

float Angle_Velocity(float Gyro, float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    static float Angle_Velocity_Last_Bias, Angle_Velocity_Integral;

    Angle_Velocity_Bias = Gyro - Gyro_Target;
    Angle_Velocity_Integral += Angle_Velocity_Bias;

    if(Angle_Velocity_Integral > 10000) Angle_Velocity_Integral = 10000;
    if(Angle_Velocity_Integral < -10000) Angle_Velocity_Integral = -10000;

    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias
            + param.angular_v_ki * Angle_Velocity_Integral
            + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);

    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;
    return PWM_Out;
}

float X_balance_Control(float Angle, float Angle_Zero, float gyro)
{
    float PWM, Bias;
    static float error;

    Bias = Angle - Angle_Zero;
    error += Bias;

    if(error > +30) error = +30;
    if(error < -30) error = -30;

    PWM = param.angular_kp * Bias + param.angular_ki * error + gyro * param.angular_kd;
    return PWM;
}

float Velocity_Control(int encoder, int target_encoder)
{
    float encoder_bias, Velocity;
    static float encoder_bias_integral;

    encoder_bias = encoder - target_encoder;
    encoder_bias_integral += encoder_bias;

    if(encoder_bias_integral > +200) encoder_bias_integral = +200;
    if(encoder_bias_integral < -200) encoder_bias_integral = -200;

    Velocity = encoder_bias * param.fly_wheel_speed_kp / 10.0f
             + encoder_bias_integral * param.fly_wheel_speed_ki / 1000.0f;
    return Velocity;
}

void balance(void)
{
    cnt++;
    cnt1++;

    if(cnt1 >= 80)
    {
        PWM_accel = Velocity_Control((int)odrive.now_speed0, 0);
        cnt1 = 0;
    }

    if(cnt >= 15)
    {
        PWM_X = X_balance_Control(imu.rol, param.angular_zero + PWM_accel, imu.vx);
        cnt = 0;
    }

    PWM_Final = Angle_Velocity(imu.vx, PWM_X);
    odrive.set_speed0 = PWM_Final;

    if(odrive.set_speed0 > fly_wheel_rate_limit)
        odrive.set_speed0 = fly_wheel_rate_limit;
    else if(odrive.set_speed0 < -fly_wheel_rate_limit)
        odrive.set_speed0 = -fly_wheel_rate_limit;

    /* 调试初期建议 6~8 度，调稳后可缩小 */
    if((imu.rol - param.angular_zero) > 8.0f || (imu.rol - param.angular_zero) < -8.0f)
    {
        param.scope_flag = 0;
        odrive.set_speed0 = 0;
        odrive.set_speed1 = 0;
        param.run_flag = 0;
        key_times = 0;
        last_rate = 0;
        low_speed_flag = 0;
    }

    if(param.scope_flag == 0)
    {
        PWM_Final = 0;
    }
}

int my_abs(int x)
{
    return x >= 0 ? x : -x;
}

float my_fabs(float x)
{
    return x >= 0 ? x : -x;
}
