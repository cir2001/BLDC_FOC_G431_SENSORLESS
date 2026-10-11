#include "opamp.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

/**
 * @brief 配置 B-G431B-ESC1 的 3 路内部运放为 PGA 模式 (增益 16)
 *  Phase U (OPAMP1): 正相输入 PA1  (VP_SEL = 00)
 *  Phase V (OPAMP2): 正相输入 PA7  (VP_SEL = 00)
 *  Phase W (OPAMP3): 正相输入 PB0  (VP_SEL = 01，关键修正！)
 */
void OPAMP_Init_Registers(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // GPIO 配置为模拟模式 (Analog Mode)
    // Phase U (PA1), Phase V (PA7)
    GPIOA->MODER |= (3U << (1 * 2)) | (3U << (7 * 2)); 
    // Phase W (PB0)
    GPIOB->MODER |= (3U << (0 * 2));

    // 基础配置掩码:
    // EN=1, VM_SEL=10(PGA), OPAINTOEN=1(输出到内部ADC), PGA_GAIN=0011(x16)
    uint32_t base_config = 
          (1U << 0)          // EN
        | (2U << 5)          // VM_SEL = PGA mode
        | (1U << 8)          // OPAINTOEN = Output to ADC
        | (3U << 14);        // PGA_GAIN = 16

    // 清理低 20 位配置区
    uint32_t clear_mask = 0x000FFFFF;

    // OPAMP1 (U相, PA1): VP_SEL = 00 -> (0U << 11)
    OPAMP1->CSR = (OPAMP1->CSR & ~clear_mask) | base_config | (0U << 11);

    // OPAMP2 (V相, PA7): VP_SEL = 00 -> (0U << 11)
    OPAMP2->CSR = (OPAMP2->CSR & ~clear_mask) | base_config | (0U << 11);

    // OPAMP3 (W相): 恢复为 VP_SEL = 00
    OPAMP3->CSR = (OPAMP3->CSR & ~clear_mask) | base_config | (0U << 11);

    // 等待电路完全稳定
    delay_ms(10);
}