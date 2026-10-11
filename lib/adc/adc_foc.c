#include "adc_foc.h"
#include "delay.h"
#include <stdio.h>
#include "timer.h"

// 全局变量定义
float offset_u = 0.0f, offset_v = 0.0f, offset_w = 0.0f;
float i_u = 0, i_v = 0, i_w = 0;

/**
 * @brief ADC1/2 寄存器初始化 (注入组 + TIM1触发)
 */
void ADC_Init_Registers(void) 
{
    // 1. 开启时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    
    // 强制复位 ADC，确保所有内部标志位清零
    RCC->AHB2RSTR |= (1U << 13);
    delay_ms(10);
    RCC->AHB2RSTR &= ~(1U << 13);

    // 2. --- 切换到同步时钟模式 (HCLK/4 = 42.5MHz) ---
    // 设置 CKMODE = 11 (Sync HCLK/4)，此时 PRESC 位域无效保持为 0
    ADC12_COMMON->CCR = 0;
    ADC12_COMMON->CCR |= (3U << 16); 
    
    // 3. 开启调节器
    ADC1->CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1->CR |= (1U << 28);  // ADVREGEN = 1
    ADC2->CR &= ~(1U << 29);
    ADC2->CR |= (1U << 28);
    delay_ms(10); 

    // 4. ADC1 校准 (增加手动复位确认)
    ADC1->CR &= ~ADC_CR_ADEN; // 确保 ADEN 为 0，否则校准必卡死
    ADC1->CR |= ADC_CR_ADCAL;
    
    uint32_t timeout = 2000000; // 足够长的手动超时
    while((ADC1->CR & ADC_CR_ADCAL) && timeout--); 

    // 5. ADC2 校准
    ADC2->CR &= ~ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADCAL;
    timeout = 2000000;
    while((ADC2->CR & ADC_CR_ADCAL) && timeout--); 

    // 6. 开启 ADC 并等待就绪 (ADRDY)
    ADC1->ISR |= ADC_ISR_ADRDY; // 写 1 清除标志
    ADC1->CR  |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC2->CR  |= ADC_CR_ADEN;
    while(!(ADC2->ISR & ADC_ISR_ADRDY));

    ADC1->CFGR2 &= ~ADC_CFGR2_JOVSE; // 关闭注入组过采样
    ADC2->CFGR2 &= ~ADC_CFGR2_JOVSE;

    // --- 采样时间优化配置 ---
    // 目标：将采样时间压缩到 0.2us ~ 0.5us 左右 (SMP=3: 24.5周期)
    uint32_t smp_time = 2U; // 12.5 周期，转换耗时缩减约 40%
    ADC1->SMPR2 &= ~(7U << 9); 
    ADC1->SMPR2 |= (smp_time << 9); 
    ADC2->SMPR2 &= ~((7U << 18) | (7U << 24));
    ADC2->SMPR2 |= (smp_time << 18);
    ADC2->SMPR2 |= (smp_time << 24);

    // ---- CFGR 配置 ----
    // 禁用注入队列模式
    ADC1->CFGR |= ADC_CFGR_JQDIS;
    ADC2->CFGR |= ADC_CFGR_JQDIS;

    // 关闭连续转换模式（注入组是硬件触发式）
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    ADC2->CFGR &= ~ADC_CFGR_CONT;

    // --- 注入序列配置 ---
    // ADC1 负责 Phase U (通道 13 = OPAMP1 输出)
    ADC1->JSQR = (0U << ADC_JSQR_JL_Pos)          // JL = 0 (1个转换)
            | (8U << ADC_JSQR_JEXTSEL_Pos)        // TIM1_TRGO2 硬件触发
            | (1U << ADC_JSQR_JEXTEN_Pos)         // JEXTEN = 01 -> 上升沿触发
            | (13U << ADC_JSQR_JSQ1_Pos);         // JSQ1 = 通道 13 (Phase U)

    // ADC2 负责 Phase V + W (通道 16=OPAMP2 输出, 通道 18=OPAMP3 输出)
    ADC2->JSQR = (1U << ADC_JSQR_JL_Pos)          // JL = 1 (2个转换)
            | (8U << ADC_JSQR_JEXTSEL_Pos)        // TIM1_TRGO2 硬件触发
            | (1U << ADC_JSQR_JEXTEN_Pos)         // 上升沿触发
            | (16U << ADC_JSQR_JSQ1_Pos)          // JSQ1 = 通道 16 (Phase V)
            | (18U << ADC_JSQR_JSQ2_Pos);         // JSQ2 = 通道 18 (Phase W)

    // --- 清除所有初始标志位 ---
    ADC1->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);
    ADC2->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);
}

// 成功返回 1，失败返回 0；失败时保留旧偏置，调用者必须检查结果。
// 保持 MOE=0，验证硬件触发采样。
uint8_t Calibrate_Current_Offset(void)
{
    Motor_Stop();

    if (!(ADC1->CR & ADC_CR_ADEN) ||
        !(ADC2->CR & ADC_CR_ADEN)) {
        printf("[Calib] FAILED: ADC not enabled.\r\n");
        return 0;
    }

    printf("[System] Starting Automatic Synchronous Calibration...\r\n");
    delay_ms(10);

    SVPWM_Output_Standard(0.0f, 0.0f);
    TIM1->BDTR &= ~TIM_BDTR_MOE;  // 测试期间禁止功率输出
    TIM1->CR1 |= TIM_CR1_CEN;
    delay_ms(100);

    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    delay_ms(100);

    const uint32_t sample_count = 2000U;
    uint32_t sum_u = 0U, sum_v = 0U, sum_w = 0U;

    for (uint32_t i = 0U; i < sample_count; ++i) {
        ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;

        uint32_t timeout = 200000U;
        while (((ADC1->ISR & ADC_ISR_JEOS) == 0U ||
                (ADC2->ISR & ADC_ISR_JEOS) == 0U) &&
               timeout > 0U) {
            --timeout;
        }

        if (timeout == 0U) {
            uint32_t adc1_isr = ADC1->ISR;
            uint32_t adc2_isr = ADC2->ISR;
            uint32_t adc1_cr  = ADC1->CR;
            uint32_t adc2_cr  = ADC2->CR;
            uint32_t tim_sr   = TIM1->SR;
            uint32_t tim_cr1  = TIM1->CR1;
            uint32_t tim_cnt  = TIM1->CNT;

            Motor_Stop();

            printf("[Calib] FAILED: ADC timeout at sample %lu.\r\n",
                   (unsigned long)i);
            delay_ms(10);

            printf("ADC1 ISR=%08lX CR=%08lX JSQR=%08lX\r\n",
                   (unsigned long)adc1_isr,
                   (unsigned long)adc1_cr,
                   (unsigned long)ADC1->JSQR);
            delay_ms(10);

            printf("ADC2 ISR=%08lX CR=%08lX JSQR=%08lX\r\n",
                   (unsigned long)adc2_isr,
                   (unsigned long)adc2_cr,
                   (unsigned long)ADC2->JSQR);
            delay_ms(10);

            printf("TIM1 SR=%08lX CR1=%08lX CNT=%lu\r\n",
                   (unsigned long)tim_sr,
                   (unsigned long)tim_cr1,
                   (unsigned long)tim_cnt);
            delay_ms(10);

            printf("TIM1 ARR=%lu CCR4=%lu CCER=%08lX\r\n",
                   (unsigned long)TIM1->ARR,
                   (unsigned long)TIM1->CCR4,
                   (unsigned long)TIM1->CCER);
            delay_ms(10);

            return 0;
        }

        sum_u += ADC1->JDR1 & 0x0FFFU;
        sum_v += ADC2->JDR1 & 0x0FFFU;
        sum_w += ADC2->JDR2 & 0x0FFFU;
    }

    Motor_Stop();
    offset_u = (float)sum_u / (float)sample_count;
    offset_v = (float)sum_v / (float)sample_count;
    offset_w = (float)sum_w / (float)sample_count;

    printf("[Calib] OK: U=%.1f, V=%.1f, W=%.1f\r\n",
           offset_u, offset_v, offset_w);
    delay_ms(20);
    return 1;
}

// 仅在偏置校准成功后调用；返回 1 只表示采集完成。
uint8_t Check_Current_Zero(void)
{
    Motor_Stop();
    if (!(ADC1->CR & ADC_CR_ADEN) ||
        !(ADC2->CR & ADC_CR_ADEN)) {
        printf("[Zero] FAILED: ADC not enabled.\r\n");
        delay_ms(10);
        return 0;
    }

    const uint32_t sample_count = 2000U;
    const float offsets[3] = {offset_u, offset_v, offset_w};
    uint32_t sums[3] = {0U, 0U, 0U};
    uint32_t minima[3] = {4095U, 4095U, 4095U};
    uint32_t maxima[3] = {0U, 0U, 0U};
    const char phases[3] = {'U', 'V', 'W'};

    for (uint32_t i = 0U; i < sample_count; ++i) {
        ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        uint32_t timeout = 200000U;
        while (((ADC1->ISR & ADC_ISR_JEOS) == 0U ||
                (ADC2->ISR & ADC_ISR_JEOS) == 0U) &&
               timeout > 0U) {
            --timeout;
        }
        if (timeout == 0U) {
            Motor_Stop();
            printf("[Zero] FAILED: ADC timeout at sample %lu.\r\n",
                   (unsigned long)i);
            delay_ms(10);
            return 0;
        }

        const uint32_t raw[3] = {
            ADC1->JDR1 & 0x0FFFU,
            ADC2->JDR1 & 0x0FFFU,
            ADC2->JDR2 & 0x0FFFU
        };
        for (uint32_t phase = 0U; phase < 3U; ++phase) {
            sums[phase] += raw[phase];
            if (raw[phase] < minima[phase]) minima[phase] = raw[phase];
            if (raw[phase] > maxima[phase]) maxima[phase] = raw[phase];
        }
    }

    Motor_Stop();
    printf("[Zero] Collected %lu fresh samples; unit=ADC counts.\r\n",
           (unsigned long)sample_count);
    delay_ms(10);
    for (uint32_t phase = 0U; phase < 3U; ++phase) {
        printf("[Zero] %c: mean=%.2f min=%.2f max=%.2f p-p=%lu\r\n",
               phases[phase],
               (float)sums[phase] / (float)sample_count - offsets[phase],
               (float)minima[phase] - offsets[phase],
               (float)maxima[phase] - offsets[phase],
               (unsigned long)(maxima[phase] - minima[phase]));
        delay_ms(10);
    }
    return 1;
}