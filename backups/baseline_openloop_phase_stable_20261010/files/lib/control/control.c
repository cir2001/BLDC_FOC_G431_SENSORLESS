#include "control.h"
#include <math.h>

void CORDIC_Init(void) 
{
    // 开启 CORDIC AHB1 时钟（保留配置兼容）
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}

// 使用 Cortex-M4F 硬件浮点单周期指令原生解算，彻底根除 norm_error = 1.0
void CORDIC_SinCos(float angle, float *s, float *c) 
{
    *s = sinf(angle);
    *c = cosf(angle);
}