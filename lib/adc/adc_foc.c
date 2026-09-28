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

    // 2. --- 切换到同步时钟模式 ---
    // 放弃 RCC_CCIPR 的异步配置，直接使用总线同步时钟
    // 设置 CKMODE = 01 (HCLK/1)，即 ADC 运行在 170MHz
    // 同时设置 PRESC = 0010 (即 4 分频，170/4 = 42.5MHz，非常理想的频率)
    ADC12_COMMON->CCR = 0; // 先清空
    ADC12_COMMON->CCR |= (3U << 16); // CKMODE = 01 (Sync HCLK/1)
    ADC12_COMMON->CCR |= (2U << 18); // PRESC = 0010 (4分频)
    
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
    // 目标：将采样时间压缩到 0.2us ~ 0.5us 左右
    uint32_t smp_time = 3U; 
    ADC1->SMPR2 &= ~(7U << 9); 
    ADC1->SMPR2 |= (smp_time << 9); 
    ADC2->SMPR2 &= ~((7U << 18) | (7U << 24));
    ADC2->SMPR2 |= (smp_time << 18);
    ADC2->SMPR2 |= (smp_time << 24);

    //----FGR 配置（最合适的位置）----
    // 禁用注入队列模式（我们只用单次注入，不需要队列）
    ADC1->CFGR |= ADC_CFGR_JQDIS;
    ADC2->CFGR |= ADC_CFGR_JQDIS;

    // 关闭连续转换模式（注入组是触发式，不是连续）
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    ADC2->CFGR &= ~ADC_CFGR_CONT;

    // --- 注入序列配置 ---
    // ADC1 负责 Phase U (通道 13 = OPAMP1)
    ADC1->JSQR = (0U << ADC_JSQR_JL_Pos)          // JL = 0 (1个转换)
            | (1U << ADC_JSQR_JEXTSEL_Pos)    // TIM1_CH4 硬件触发
            | (1U << ADC_JSQR_JEXTEN_Pos)      // JEXTEN = 01 → 上升沿触发（最稳）
            | (13U << ADC_JSQR_JSQ1_Pos);      // JSQ1 = 通道 13 (Phase U)

    // ADC2 负责 Phase V + W (通道 16=OPAMP2, 18=OPAMP3)
    ADC2->JSQR = (1U << ADC_JSQR_JL_Pos)          // JL = 1 (2个转换)
            | (1U << ADC_JSQR_JEXTSEL_Pos)    //  TIM1_CH4 硬件触发
            | (1U << ADC_JSQR_JEXTEN_Pos)      // 上升沿触发
            | (16U << ADC_JSQR_JSQ1_Pos)       // JSQ1 = 通道 16 (Phase V)
            | (18U << ADC_JSQR_JSQ2_Pos);      // JSQ2 = 通道 18 (Phase W)

    // --- 清除所有初始标志位 ---
    ADC1->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);
    ADC2->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);

}

void Calibrate_Current_Offset(void) 
{
    printf("[System] Starting Automatic Synchronous Calibration...\r\n");
    delay_ms(10);

    // 确保 FOC 中断关闭，防止干扰校准过程
    TIM1->DIER &= ~TIM_DIER_UIE;

    // 使用 SVPWM 产生 50% 占空比 (零矢量)
    // 输入 (0,0) 会让 Ta, Tb, Tc 都等于 0.5，从而产生 PWM_ARR/2 的占空比
    SVPWM_Output_Standard(0.0f, 0.0f);

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_CEN;
    delay_ms(100);

    // 确保 ADC 处于 Ready 状态且已使能
    // 如果 ADC 没使能，JADSTART 是无效的
    if (!(ADC1->CR & ADC_CR_ADEN) || !(ADC2->CR & ADC_CR_ADEN)) {
        printf("[Error] ADC not enabled!\r\n");
        return;
    }

    // 清除旧的标志位，确保开始时是干净的
    ADC1->ISR |= (ADC_ISR_JEOS | ADC_ISR_JEOC);
    ADC2->ISR |= (ADC_ISR_JEOS | ADC_ISR_JEOC);

    // 启动 ADC 的硬件触发监听模式
    // 只有执行了这一步，ADC 才会开始等待 CCR4 的上升沿
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    delay_ms(100);

    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    int success_count = 0;

    for(int i = 0; i < 2000; i++) 
    {
        uint32_t timeout = 200000;
        // 等待硬件触发完成 (JEOS 代表整个序列转换完成)
        while(!(ADC2->ISR & ADC_ISR_JEOS) && timeout--);

        if(timeout > 0) {
            sum_u += ADC1->JDR1;
            sum_v += ADC2->JDR1;
            sum_w += ADC2->JDR2;
            success_count++;
            
            // 清除标志位，ADC 会自动等待下一个 PWM 周期的 CCR4 信号
            ADC1->ISR |= ADC_ISR_JEOS;
            ADC2->ISR |= ADC_ISR_JEOS;
        }
    }

    // 停止监听（正式跑 FOC 时再开启）
    // 注意：在正式 FOC 循环开始前，需再次执行 JADSTART 开启监听
    if(success_count > 100) {
        offset_u = (float)sum_u / success_count;
        offset_v = (float)sum_v / success_count;
        offset_w = (float)sum_w / success_count;
        printf("[Calib] CCR Success! U:%.1f, V:%.1f, W:%.1f\r\n", offset_u, offset_v, offset_w);
        delay_ms(50);
    } else {
        printf("[Calib] FAILED: No Hardware Triggers detected.\r\n");
        delay_ms(50);
    }

    // 校准完成，根据需要保持输出或关闭
    // 建议：暂时关闭输出，直到正式进入 FOC 或对齐逻辑
    TIM1->BDTR &= ~TIM_BDTR_MOE; 
    delay_ms(20);
}

