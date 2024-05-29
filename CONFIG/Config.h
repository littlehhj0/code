//#include "sys.h"
#include "Motor.h"
#include <math.h>
#define SERVO_INIT_VALUE   750  //舵机初始化PWM值
#define T 0.156f
#define L 0.1445f        //阿克曼转角相关参数
#define K 311.4f         //舵机转角映射为PWM系数

