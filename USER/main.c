#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "Motor.h"
#include "CarDriveCtrl.h"
#include "CCD.h"
#include "wave.h"
#include "stdio.h"
float Velocity = -6, Angle = 0;
extern int v_left, v_right;
float distance          = 0;
int Identification_mark = 0; // 识别标志 0：未识别 1：识�?
int Identification_num  = 0; // 识别次数
int main(void)
{
    Car_Tag_Value a;
    Car_PWM_Value b;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置系统中断优先级分�?2
    delay_init(168);                                // 初始化延时函�?
    LED_Init();                                     // 初始化LED端口
    uart_init(115200);
    TIM5_Int_Init(50 - 1, 8400 - 1);
    Motor_PWM_Init(8400 - 1, 0);
    Servo_PWM_Init(9999, 336);
    Encoder_Init_TIM2(); // 左轮编码
    Encoder_Init_TIM3(); // 右轮编码
    CtrlIO_Init();
    Adc_Init();
    Drv_Hcsr04_Init();

    while (1) {
        distance = Med_Hcsr04_GetLength();
        // printf("distance = %f\r\n", distance);
        if (distance < 20) {
            Identification_num++;
            if (Identification_num > 5) {
                Identification_mark = 1;
                Identification_num  = 0;
                a                   = Kinematic_Analysis(10, 0); // 后退
                b                   = Get_PWM(v_left, v_right, a);
                b.PWM_Left          = 1500;
                b.PWM_Right         = 1500;
                Set_Pwm(b);
                delay_ms(3000);
                a = Kinematic_Analysis(-10, -1); // 转向
                b = Get_PWM(v_left, v_right, a);
                Set_Pwm(b);
                delay_ms(2000);
            }
        } else {
            Identification_mark = 0;
            Identification_num  = 0;
        }
        if (Identification_mark == 0) {
            a = Kinematic_Analysis(Velocity, Angle);
            b = Get_PWM(v_left, v_right, a);
            Set_Pwm(b);
        }
    }
}
