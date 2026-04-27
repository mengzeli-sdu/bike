#include "servo.h"
#define PWM_RESOLUTION 10000
HAL_StatusTypeDef servo_status;

#define Servo_Center_Mid 780                     //舵机直行中值
#define Servo_Left_Max (Servo_Center_Mid + 200)  //舵机左转极限值
#define Servo_Right_Min (Servo_Center_Mid - 200) //舵机右转极限值
inline static void set_pwm_duty(float duty);

void servo_init(void)
{
	MX_TIM2_Init(); //PWM OUTPUT
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //PA0
	//servo_set_duty(0);
}

void servo_set_duty(int duty)
{
    int target = Servo_Center_Mid + duty;
    target = target > Servo_Left_Max ? Servo_Left_Max : target;
    target = target < Servo_Right_Min ? Servo_Right_Min : target;
    float res = (float)target / 10000;
    set_pwm_duty(res);
}

inline static void set_pwm_duty(float duty){
	duty > 1 ? duty = PWM_RESOLUTION : duty;
	duty < 0 ? duty = 0 : duty;
//	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,duty);
//	PWM_SetDuty(&htim2,TIM_CHANNEL_1,duty); //PA 0
//	PWM_SetDuty(&htim2,TIM_CHANNEL_2,duty); //PA 1
	PWM_SetDuty(&htim2,TIM_CHANNEL_3,duty); //PA 2
//	PWM_SetDuty(&htim2,TIM_CHANNEL_4,duty); //PA 3
}
//速度限幅
int Steer_Speed_Limit(int now, int last, int limit, int times)
{
    static int cnt = 0;
    cnt++;
    if (cnt >= times)
    {
        cnt = 0;
        if ((now - last) >= limit)
            return (last + limit);
        else if ((now - last) <= -limit)
            return (last - limit);
        else
            return now;
    }
    return last;
}
