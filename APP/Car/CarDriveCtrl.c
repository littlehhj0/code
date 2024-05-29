#include "CarDriveCtrl.h"
#include "Config.h"
#include "usart.h"

float Velocity_KP = 12, Velocity_KI = 12;

/**************************************************************************
函数功能：小车运动数学模型
入口参数：平均速度和转角
返回  值：无
**************************************************************************/
Car_Tag_Value Kinematic_Analysis(float velocity, float angle)
{
    Car_Tag_Value PWM;
    PWM.Tag_Left = velocity ;//* (1 + T * tan(angle) / 2 / L);
    PWM.Tag_Right = velocity ;//* (1 - T * tan(angle) / 2 / L); // 后轮差速
    PWM.PWM_Servo = SERVO_INIT_VALUE + angle * K;            // 舵机转向
    return PWM;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(int Encoder, int Target)
{
    static int Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // 计算偏差
    //	printf("BiasA:%d\r\n",Bias);
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; // 增量式PI控制器
    //	printf("PwmA:%d\r\n",Pwm);
    Last_bias = Bias; // 保存上一次偏差
    return Pwm;       // 增量输出
}
int Incremental_PI_B(int Encoder, int Target)
{
    static int Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // 计算偏差
    //	printf("BiasB:%d\r\n",Bias);
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; // 增量式PI控制器
    //	printf("PwmB:%d\r\n",Pwm);
    Last_bias = Bias; // 保存上一次偏差
    return Pwm;       // 增量输出
}

Car_PWM_Value Get_PWM(int Encoder_left, int Encoder_right, Car_Tag_Value Tag_v)
{
    Car_PWM_Value PWM;
    int Amplitude = 8000; //===PWM满幅是7200 限制在6900

    PWM.PWM_Left = Incremental_PI_A(Encoder_left, Tag_v.Tag_Left);
    //	printf("%d\r\n",PWM.PWM_Left);
    PWM.PWM_Right = Incremental_PI_B(Encoder_right, Tag_v.Tag_Right);
    PWM.PWM_Servo = Tag_v.PWM_Servo;

    if (PWM.PWM_Left < -Amplitude)
        PWM.PWM_Left = -Amplitude;
    if (PWM.PWM_Left > Amplitude)
        PWM.PWM_Left = Amplitude;
    if (PWM.PWM_Right < -Amplitude)
        PWM.PWM_Right = -Amplitude;
    if (PWM.PWM_Right > Amplitude)
        PWM.PWM_Right = Amplitude;
    if (PWM.PWM_Servo < (SERVO_INIT_VALUE - 200))
        PWM.PWM_Servo = SERVO_INIT_VALUE - 200; // 舵机限幅
    if (PWM.PWM_Servo > (SERVO_INIT_VALUE + 200))
        PWM.PWM_Servo = SERVO_INIT_VALUE + 200; // 舵机限幅
    return PWM;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(Car_PWM_Value PWM)
{
    //	    if(Flag_Way>=2)//巡线模式下，只允许电机正转
    //			{
//    if (PWM.PWM_Left < 0)
//        PWM.PWM_Left = 0;
//    if (PWM.PWM_Left < 0)
//        PWM.PWM_Left = 0;
    //			}
    if (PWM.PWM_Left < 0)
        PWMA1 = 8400, PWMA2 = 8400 + PWM.PWM_Left;
    else
        PWMA2 = 8400, PWMA1 = 8400 - PWM.PWM_Left;

    if (PWM.PWM_Right < 0)
        PWMB1 = 8400, PWMB2 = 8400 + PWM.PWM_Right;
    else
        PWMB2 = 8400, PWMB1 = 8400 - PWM.PWM_Right;
    SERVO = PWM.PWM_Servo;
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;
    switch (TIMX)
    {
    case 2:
        Encoder_TIM = (short)TIM2->CNT;
        TIM2->CNT = 0;
        break; //
    case 3:
        Encoder_TIM = (short)TIM3->CNT;
        TIM3->CNT = 0;
        break;
    case 4:
        Encoder_TIM = (short)TIM4->CNT;
        TIM4->CNT = 0;
        break;
    default:
        Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //????
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /*
            NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X01 ;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0X02;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
    */

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 8;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //??PB????
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //?????3???

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //?????????GPIOB

    /*??*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

    TIM_DeInit(TIM3);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                  //????
    TIM_TimeBaseStructure.TIM_Period = 0xffff;                  // ENCODER_TIM_PERIOD;//??????????
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //??????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM????
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    /*
            NVIC_InitStructure.NVIC_IRQChannel                                                 = TIM3_IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority        =0X01 ;//?????3
            NVIC_InitStructure.NVIC_IRQChannelSubPriority                         = 0X02;                //????3
            NVIC_InitStructure.NVIC_IRQChannelCmd                                         = ENABLE;                 //IRQ????
            NVIC_Init(&NVIC_InitStructure); //??????????VIC???
    */
    //???????3
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 8;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update); //??TIM??????
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // Reset counter
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}
