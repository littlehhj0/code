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
float Velocity = 0, Angle = 0;
static float Bias;
int Sensor_Left, Sensor_Right;
int Sensor, sum;
extern int v_left, v_right;
float distance = 0;
int main(void)
{
    Car_Tag_Value a;
    Car_PWM_Value b;
    // b.PWM_Left  = 1000;
    // b.PWM_Right = -1000;
    // b.PWM_Servo = 1;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置系统中断优先级分组2
    delay_init(168);                                // 初始化延时函数
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
        // delay_ms(500);
        // if (flag == 1) {
        //     flag = 0;

        //            Sensor_Left  = Get_Adc(15);
        //            Sensor_Right = Get_Adc(13);
        //            sum          = Sensor_Left + 100 * Sensor_Right;
        //            Sensor       = sum / (Sensor_Left + Sensor_Right);
        //            Bias         = Sensor - 50; // 提取偏差
        //            Angle        = 0.1f * Bias; //
        //
        Velocity = -(distance - 30) * 0.5;
        // printf("Velocity = %f\r\n", Velocity);
        a = Kinematic_Analysis(Velocity, Angle);
        b = Get_PWM(v_left, v_right, a);
        //printf("PWM_Left = %d, PWM_Right = %d, PWM_Servo = %d\r\n", b.PWM_Left, b.PWM_Right, b.PWM_Servo);
        //b.PWM_Right = b.PWM_Left;
//        b.PWM_Right = -4000;
//        b.PWM_Left = -4000;
        // b.PWM_Left *= 1.3;
        Set_Pwm(b);
        //}
        // Set_Pwm(b);
    }
}
