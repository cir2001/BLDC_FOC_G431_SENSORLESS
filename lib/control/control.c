#include "control.h"
//-----------------------------------------------
//
//-----------------------------------------------
void CORDIC_Init(void) {
    // 1. 开启时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;

    // 2. 配置控制寄存器 (CSR)
    // FUNC = 0: Cosine (同时输出 Sine)
    // PRECISION = 6: 24个周期 (对于 FOC 足够精准)
    // SCALE = 0: 缩放因子 1
    // NBWRITE = 0: 1次写入 (输入角度)
    // NBRESP = 1: 2次读取 (先读 Cos, 再读 Sin)
    // ARGSIZE = 0: 32位输入
    // RESSIZE = 0: 32位输出
    CORDIC->CSR = (0 << CORDIC_CSR_FUNC_Pos) | (6 << CORDIC_CSR_PRECISION_Pos) | 
                  (1 << CORDIC_CSR_NRES_Pos);
}
//--------------------------------------------------
// 传入电角度（弧度），通过指针同时返回 Sin 和 Cos
//--------------------------------------------------
void CORDIC_SinCos(float angle, float *s, float *c) 
{
    // 1. 将弧度角 [-pi, pi] 映射到 q31 范围 [-1, 1]
    // 快速归一化：将 angle / PI
    float input = angle * 0.318309886f; // 1/PI

    // 边界处理：确保在 [-1, 1] 之间
    if (input > 1.0f) input -= 2.0f;
    if (input < -1.0f) input += 2.0f;

    // 2. 转换为 q31 整数
    int32_t q31_angle = (int32_t)(input * 2147483647.0f);

    // 3. 写入 CORDIC 输入寄存器
    CORDIC->WDATA = q31_angle;

    // 4. 读取结果 (根据 NBRESP=1 的配置)
    // 第一读是 FUNCTION (Cos)，第二读是 SECONDARY (Sin)
    int32_t q31_cos = CORDIC->RDATA;
    int32_t q31_sin = CORDIC->RDATA;

    // 5. 转回浮点数
    *c = (float)q31_cos * 4.65661287e-10f; // 1/(2^31)
    *s = (float)q31_sin * 4.65661287e-10f;
}
