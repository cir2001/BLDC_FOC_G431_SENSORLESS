#include "opamp.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

/**
 * @brief 配置 B-G431B-ESC1 的 3 路内部运放为 PGA 模式 (增益 16)
 *  Phase U (OPAMP1): 正相输入 PA1
    Phase V (OPAMP2): 正相输入 PA7
    Phase W (OPAMP3): 正相输入 PB0
 */
void OPAMP_Init_Registers(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // GPIO 配置为模拟模式 (Analog Mode)
    // Phase U (PA1), Phase V (PA7)
    GPIOA->MODER |= (3U << (1 * 2)) | (3U << (7 * 2)); 
    // Phase W (PB0)
    GPIOB->MODER |= (3U << (0 * 2));

    // 3. 定义配置掩码
    // EN=1, VM_SEL=10(PGA), OPAINTOEN=1, VP_SEL=000, PGA_GAIN=0011(x16)
    uint32_t config_mask = 
          (1U << 0)          // EN
        | (2U << 5)          // VM_SEL = PGA mode
        | (1U << 8)          // OPAINTOEN = Output to ADC
        | (0U << 11)         // VP_SEL = 000 (关键：解决 W 相偏置错误)
        | (3U << 14);        // PGA_GAIN = 16

    // 4. 定义清理掩码 (清除低 20 位配置区)
    uint32_t clear_mask = 0x000FFFFF;

    // 5. 分别配置 OPAMP 1, 2, 3 (保留高位校准值)
    OPAMP1->CSR = (OPAMP1->CSR & ~clear_mask) | config_mask;
    OPAMP2->CSR = (OPAMP2->CSR & ~clear_mask) | config_mask;
    OPAMP3->CSR = (OPAMP3->CSR & ~clear_mask) | config_mask;

    // 6. 等待稳定
    for(volatile int i=0; i<10000; i++); // 简易延迟，确保电路稳定

    // 7. 打印验证
    // printf("OP1 CSR: 0x%08lX\r\n", OPAMP1->CSR);
    // printf("OP2 CSR: 0x%08lX\r\n", OPAMP2->CSR);
    // printf("OP3 CSR: 0x%08lX\r\n", OPAMP3->CSR);
}
