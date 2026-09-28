#include "mt6826s.h"
#include "delay.h"
#include <math.h>
#include "timer.h"
#include <stdio.h>
#include "align.h"
#include "control.h"
//-----------------------------------------------------
//-----------------------------------------------------
// --- 变量声明 ---
float test_v = 0.2f;           // 测试电压 (根据母线电压调节，12V下建议0.8-1.5V)
int test_cycles = 10;          // 电角度转动的圈数
float theta_e_step = 0.005f;    // 步进速度

//-----------------------------------------------------
/**
 * @brief 电角度对齐寻零
 * @return zero_offset: 机械零位偏移量 (0-16383)
 */
uint16_t FOC_Align_Sensor(void) 
{
    printf("Starting Smooth Two-Stage Alignment...\r\n");

    float max_v = 1.0f;       // 最终对齐电压占空比 (25%)
    float stage1_v = 0.15f;    // 阶段1预定位电压 (15%)
    int ramp_steps = 1000;     // 增加步数，让滑动更丝滑

    // 确保主输出开启
    TIM1->BDTR |= TIM_BDTR_MOE;

    // --- 阶段 1: 电流爬坡 (确保转子先锁定在 V 相轴线，即 120 度) ---
    // 电角度 120 度 (2.0944rad) 对应 V 相
    float start_angle = 2.094395f; 
    // printf("Stage 1: Locking to Phase V...\r\n");
    
    for(int i = 0; i <= ramp_steps; i++) {
        float ramp_v = ((float)i / ramp_steps) * max_v;
        
        // 使用你的 SVPWM 函数：Vd=ramp_v, Vq=0, angle=120deg
        float s, c;
        // 注意：这里由于是初始化，建议用 sinf/cosf 避开 CORDIC 潜在风险
        s = sinf(start_angle);
        c = cosf(start_angle);
        SVPWM_Output_Standard(ramp_v * c, ramp_v * s);
        
        delay_ms(2);
    }
    delay_ms(500); // 锁定后静止 0.5s

    // --- 阶段 2: 矢量平滑旋转 (120度 -> 0度) ---
    // 目的：拖动转子平滑移动到 U 相轴线 (0度)
    // printf("Stage 2: Rotating Vector to Phase U...\r\n");
    for(int i = 0; i <= ramp_steps; i++) {
        float alpha = (float)i / ramp_steps;
        float current_angle = start_angle * (1.0f - alpha); // 从 120 降到 0
        
        float s = sinf(current_angle);
        float c = cosf(current_angle);
        SVPWM_Output_Standard(max_v * c, max_v * s);
        
        delay_ms(2);
    }

    // --- 阶段 3: 最终静止与采样 ---
    // printf("Waiting for stabilization...\r\n");
    delay_ms(1500); // 彻底消除机械余震

    printf("Sampling zero offset...\r\n");
    // 高精度采样平均 (256次)
    uint32_t sum = 0;
    for(int i = 0; i < 512; i++) {
        sum += (TIM4->CNT & ENCODER_MASK);
        delay_ms(1);
    }
    uint16_t zero_offset = (uint16_t)(sum / 512);

    // --- 阶段 4: 结果应用 ---
    // manual_adjust 补偿：如果你的 Clarke 变换以 U 轴为 0 度，
    // 而电流环运行时发现输出异常，可在此微调。 485
    int16_t manual_adjust = -3;//60; 
    uint16_t final_offset = (uint16_t)((zero_offset + manual_adjust) & ENCODER_MASK);

    // 保持轻微电压防止滑走，直到 main 函数开启中断
    SVPWM_Output_Standard(0.3f, 0);

    printf("Raw Offset: %d, Final: %d\r\n", zero_offset, final_offset);
    delay_ms(10);
    
    return final_offset;
}


void Detect_Motor_Pole_Pairs(void){
    // 1. 屏蔽 TIM1 中断
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);

    printf("--- Pole Pair Detection start ---\r\n");
    delay_ms(10); 
    // 2. 预对齐：快速但充分
    // 设定 1 秒对齐时间，足够让转子稳定在 0 度
    for(int i = 0; i < 100; i++) {
        SVPWM_Output_Standard(test_v, 0); 
        delay_ms(5);
    }
    // 3. 记录初始位置
    uint16_t last_raw = (uint16_t)(TIM4->CNT) & ENCODER_MASK;
    int32_t total_accumulated = 0;

    // 4. 开始强拖旋转
    float current_theta_e = 0.0f;
    float target_theta_limit = (float)test_cycles * 2.0f * M_PI;
    printf("Limit: %.2f\r\n", target_theta_limit); // 看看这里是不是 62.83
    delay_ms(10);
    printf("Rotating...\r\n");
    delay_ms(10); 

    while(current_theta_e < target_theta_limit) 
    {
        current_theta_e += theta_e_step;
        
        // float s, c;
        // CORDIC_SinCos(current_theta_e, &s, &c);
        float s = sinf(current_theta_e);
        float c = cosf(current_theta_e);
        
        SVPWM_Output_Standard(test_v * c, test_v * s);

        uint16_t current_raw = (uint16_t)(TIM4->CNT) & ENCODER_MASK;
        int16_t diff = (int16_t)(current_raw - last_raw);
        if (diff > 8192)  diff -= 16384;
        else if (diff < -8192) diff += 16384;
        total_accumulated += diff;
        last_raw = current_raw;

        printf("cur: %.2f\r\n", current_theta_e); // 看看这里是不是 62.83
        delay_ms(1);
    }

    // 5. 停止输出
    SVPWM_Output_Standard(0, 0);

    // 6. 计算
    float mech_rotations = (float)total_accumulated / 16384.0f; 
    float measured_pn = (float)test_cycles / mech_rotations;
    if(measured_pn < 0) measured_pn = -measured_pn;

    printf("End. Mech Rot: %.4f, Pole Pairs: %.2f\r\n", mech_rotations, measured_pn);

    // 7. 恢复中断
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}


