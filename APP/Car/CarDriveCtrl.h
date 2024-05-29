#include "Config.h"

#define PWMA1   TIM4->CCR2  
#define PWMA2   TIM4->CCR1 

#define PWMB1   TIM4->CCR4  
#define PWMB2   TIM4->CCR3

#define SERVO   TIM1->CCR4  //�������

typedef struct{
	int Tag_Left;   //����Ŀ���ٶ�
	int Tag_Right;  //����Ŀ���ٶ�
	int PWM_Servo;  //���PWM
}Car_Tag_Value;

typedef struct{
	int PWM_Left;   //������PWM
	int PWM_Right;
	int PWM_Servo;   //���PWM
}Car_PWM_Value;  

Car_Tag_Value Kinematic_Analysis(float velocity,float angle);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
Car_PWM_Value Get_PWM(int Encoder_left,int Encoder_right,Car_Tag_Value Tag_v);
void Set_Pwm(Car_PWM_Value PWM);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 TIMX);








