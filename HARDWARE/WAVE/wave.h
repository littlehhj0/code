#ifndef __WAVE_H
#define __WAVE_H
#include "sys.h"



#define HCSR04_PORT GPIOC               // 定义IO口
#define HCSR04_CLK RCC_AHB1Periph_GPIOC // 开启GPIO时钟
#define HCSR04_TRIG GPIO_Pin_8          // 定义Trig对应引脚
#define HCSR04_ECHO GPIO_Pin_9          // 定义Echo对应引脚

#define TRIG_Send PCout(8) // 将TRIG_Send映射到PC8
#define ECHO_Reci PCin(9)  // 将ECHO_Reci映射到PC9

void Drv_Hcsr04_Init(void);                      // Hc-sr04初始化
void Drv_Hcsr04_OpenTimerForHc(void);            // 打开定时器
void Drv_Hcsr04_CloseTimerForHc(void);           // 关闭定时器
unsigned long int Drv_Hcsr04_GetEchoTimer(void); // 获取定时器时间				    
float Med_Hcsr04_GetLength(void);
#endif
