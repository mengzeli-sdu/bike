#include "task.h"
#include "tim.h" 
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"
#define  FS 3
#if FS==1  
	float fast_rate=8.0f,slow_rate=3.0f,mid_rate=5.0f;
#elif FS==2
	float fast_rate=10.0f,slow_rate=4.50f,mid_rate=6.50f;
	#else 
		float fast_rate=12.0f,slow_rate=5.50f,mid_rate=6.50f;
#endif
//0.9
#define fly_wheel_rate_limit 15 //动量轮速度限幅
#define dt 0.100f
#define PI 3.1415926f
paramTypeDef param;
float PWM_X,PWM_accel,PWM_Final;// PWM中间量
extern int key_times;
int cnt;//角度环计数
int cnt1;//速度环计数
int cnt_vel_callback1;//飞轮速度反馈计数
int cnt_vel_set1;//飞轮速度发送计数
int cnt_balance;//自行车平衡控制周期计数
int cnt_rate;//速度设置计数
float rate;//死区外飞轮速度
float start_yaw0;//开始积分时的偏航角
float last_rate=0;//记录上一时刻的速度
//定时器 2ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		imu_get();//陀螺仪读取
		
		cnt_vel_set1++;
    cnt_balance++;	
		cnt_rate++;
		if(cnt_rate>=50)
		{
			cnt_rate=0;
			rate_set();//速度设置
		}
		if(param.scope_flag == 1)//飞轮平衡控制周期 2ms
		{
				balance();
				cnt_balance=0;
		}
		if(cnt_vel_set1 >= 1)//odrive can通信周期 2ms   
		{				
				cnt_vel_callback1++;
		  	odrive_speed_ctrl(0,odrive.set_speed0);
				
				cnt_vel_set1 = 0;
				if(cnt_vel_callback1 == 20) 
				{
						cnt_vel_callback1 = 0;
				    odrive_speed_ctrl(1,-odrive.set_speed1);
				}
		}
	}
}
/*
函数名称：rate_set
函数功能：分段设置速度
*/
void rate_set()
{
	if(param.run_flag==1)//运行后轮
	{
		odrive.set_speed1 = 1;
	}
	else
	{
		odrive.set_speed1=0;
	}

}


//pid参数初始化
void param_init(){
    param.angular_kp = -7.6;//-5.45;//-9;//4;//-10.6;
    param.angular_ki = 0;
    param.angular_kd = -1.4;//-4.7;//-4.8;
	
    param.angular_v_kp = 1.4;//-2.1;
    param.angular_v_ki = 0; 
    param.angular_v_kd = 1.1;//;//-0.99;
	
    param.fly_wheel_speed_kp = -1.2;//0.8;//0.8;//0.81;
    param.fly_wheel_speed_ki = -0.6;
    param.fly_wheel_speed_kd = 0;
	
	  param.zero_speed_kp=0;
	  param.zero_speed_kd=0;
	  param.zero_speed_ki=0;//0.0002;
	
    param.angular_zero = 2.1;

    param.scope_flag = 0;
		param.run_flag=0;
		
    param.Steer_Kp = 1.5;
    param.Steer_Ki = 0.2;//预防死区
    param.Steer_Kd = 0;

}
//角速度环pid
float Angle_Velocity(float Gyro,float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    static float Angle_Velocity_Last_Bias,Angle_Velocity_Integral;
    Angle_Velocity_Bias = Gyro - Gyro_Target;
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000)
			Angle_Velocity_Integral =10000;                   
    if(Angle_Velocity_Integral < -10000) 
			Angle_Velocity_Integral = -10000;        
		
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;                             //保留上次误差
    return PWM_Out;
}
//角度环pid
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     static float error;
     Bias=Angle-Angle_Zero;                                            //获取偏差
     error+=Bias;                                                      //偏差累积
     if(error>+30) error=+30;                                          //积分限幅
     if(error<-30) error=-30;                                          //积分限幅
     PWM=param.angular_kp*Bias + param.angular_ki*error + (gyro)*param.angular_kd;   //获取最终数值
     return PWM;
}
//速度环pid
float Velocity_Control(int encoder,int target_encoder)
{
    float encoder_bias,Velocity;
    static float encoder_bias_integral;
    encoder_bias = encoder - target_encoder;
    encoder_bias_integral += encoder_bias;
    if(encoder_bias_integral > +200) 
			encoder_bias_integral = +200;                    //积分限幅
    if(encoder_bias_integral < -200) 
			encoder_bias_integral = -200;                    //积分限幅是500
    Velocity = encoder_bias * param.fly_wheel_speed_kp/10 + encoder_bias_integral * param.fly_wheel_speed_ki/1000;
    return Velocity;
}

void balance(void)
{
    cnt++;																																																
    cnt1++;																																												
														 																																				
		if(cnt1>=80){PWM_accel = Velocity_Control(odrive.now_speed0 , 0);cnt1=0;}                              
    if(cnt>=15){PWM_X = X_balance_Control(imu.rol,param.angular_zero+PWM_accel,imu.vx);cnt=0;}	          
    PWM_Final = Angle_Velocity(imu.vx,PWM_X);       																									
    odrive.set_speed0 = PWM_Final;																																					
			  																																														
    if(odrive.set_speed0>fly_wheel_rate_limit) odrive.set_speed0=fly_wheel_rate_limit;      					
    else if(odrive.set_speed0<-fly_wheel_rate_limit) odrive.set_speed0=-fly_wheel_rate_limit; 							 
																																																						 
    if((imu.rol-(param.angular_zero))>3 || (imu.rol-(param.angular_zero))<-3)
		{
			param.scope_flag=0;
			odrive.set_speed0=odrive.set_speed1=0;
			param.run_flag=0;
			key_times=0;
			last_rate=0;
			low_speed_flag=0;
		}    
		if(param.scope_flag==0)
			PWM_Final=0;
   // odrive.set_speed0 =1;角度左正右负 电机左正右负  
		
}

int my_abs(int x)
{
	  float m;
    if(x>=0)
        m= x;
    else if(x<0)
        m= -x;
		return m;
}
float my_fabs(float x)
{
	  float m;
    if(x>=0)
        m= x;
    else if(x<0)
        m= -x;
		return m;
}

