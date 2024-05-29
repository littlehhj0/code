#include "Config.h"

#define PWMA1   TIM4->CCR2  
#define PWMA2   TIM4->CCR1 

#define PWMB1   TIM4->CCR4  
#define PWMB2   TIM4->CCR3

#define SERVO   TIM1->CCR4  //舵机引脚

typedef struct{
	int Tag_Left;   //左轮目标速度
	int Tag_Right;  //右轮目标速度
	int PWM_Servo;  //舵机PWM
}Car_Tag_Value;

typedef struct{
	int PWM_Left;   //左右轮PWM
	int PWM_Right;
	int PWM_Servo;   //舵机PWM
}Car_PWM_Value;  

Car_Tag_Value Kinematic_Analysis(float velocity,float angle);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
Car_PWM_Value Get_PWM(int Encoder_left,int Encoder_right,Car_Tag_Value Tag_v);
void Set_Pwm(Car_PWM_Value PWM);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 TIMX);








