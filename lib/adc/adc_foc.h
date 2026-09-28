#ifndef ADC_FOC_H
#define ADC_FOC_H

#include "stm32g4xx.h"

// 电流采样相关全局变量
extern float i_u, i_v, i_w;

// 函数声明
void ADC_Init_Registers(void);
void Calibrate_Current_Offset(void);


#endif


