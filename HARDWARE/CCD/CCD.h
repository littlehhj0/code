#ifndef __ADC_H
#define __ADC_H	
#include "sys.h" 
extern u16 ADV[128];	
extern u8 CCD_Zhongzhi;
 							  
void CtrlIO_Init(void);								
void Adc_Init(void); 				//ADC通道初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  
void Dly_us(void);
 void RD_TSL(void);
 void  Find_CCD_Zhongzhi(void);
 int myabs(int a);
#endif 















