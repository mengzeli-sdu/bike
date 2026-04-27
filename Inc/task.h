#ifndef __TASK_H__
#define __TASK_H__


#include "main.h"
//pid参数结构体
typedef struct 
{
		//平衡飞轮角速度环
    float angular_v_kp;
    float angular_v_ki;
    float angular_v_kd;
		//平衡飞轮角度环
    float angular_kp;
    float angular_ki;
    float angular_kd;
		//平衡飞轮速度环
    float fly_wheel_speed_kp;
    float fly_wheel_speed_ki;
    float fly_wheel_speed_kd;
	  //零点飞轮速度环
	  float zero_speed_kp;
    float zero_speed_ki;
    float zero_speed_kd;
	
    float angular_zero;             //角度零点
		float zer0;
    float angular_target;           //目标角度
		
		int run_flag;
    int scope_flag;
		
    float Steer_Kp;                 //舵机kp
    float Steer_Ki;                 //舵机ki
    float Steer_Kd;        					//舵机kd
		
}paramTypeDef;
extern paramTypeDef param;
extern float distance;//积分距离

int my_abs(int x);
float my_fabs(float x);
void rate_set(void);
int Distance_integral(void);
int SBB_Get_BalancePID(float Angle,float Gyro,float Pitch_Calculate);
int Steer_Engine_control(float image_bias);
void test_zero_pid(void);
void test_zero(void);
void param_init(void);
void balance(void);
#endif

