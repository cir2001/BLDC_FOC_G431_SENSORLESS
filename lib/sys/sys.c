#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//
//////////////////////////////////////////////////////////////////////////////////  
// 设置NVIC分组
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group) {
    u32 temp, temp1;
    temp1 = (~NVIC_Group) & 0x07; // 取后三位
    temp1 <<= 8;
    temp = SCB->AIRCR;  
    temp &= 0X0000F8FF; 
    temp |= 0X05FA0000; 
    temp |= temp1;
    SCB->AIRCR = temp;        
}

// 设置NVIC中断
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group) {
    u32 temp;
    MY_NVIC_PriorityGroupConfig(NVIC_Group);
    temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
    temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
    temp &= 0xf;
    NVIC->ISER[NVIC_Channel / 32] |= 1 << (NVIC_Channel % 32);
    NVIC->IP[NVIC_Channel] |= temp << 4;
}

void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx) {
    // 1. 设置 MODER 寄存器为 "复用模式" (10)
    // 每个引脚占用 2 位，所以左移 BITx * 2
    GPIOx->MODER &= ~(3U << (BITx * 2));
    GPIOx->MODER |= (2U << (BITx * 2));

    // 2. 设置 AFR 寄存器 (你原来的逻辑是正确的)
    GPIOx->AFR[BITx >> 3] &= ~(0X0F << ((BITx & 0X07) * 4));
    GPIOx->AFR[BITx >> 3] |= (u32)AFx << ((BITx & 0X07) * 4);
}

// GPIO通用设置
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD) {
    u32 pinpos = 0, pos = 0, curpin = 0;
    for (pinpos = 0; pinpos < 16; pinpos++) {
        pos = 1 << pinpos;
        curpin = BITx & pos;
        if (curpin == pos) {
            GPIOx->MODER &= ~(3 << (pinpos * 2));
            GPIOx->MODER |= MODE << (pinpos * 2);
            if ((MODE == 1) || (MODE == 2)) { // 输出或复用
                GPIOx->OSPEEDR &= ~(3 << (pinpos * 2));
                GPIOx->OSPEEDR |= OSPEED << (pinpos * 2);
                GPIOx->OTYPER &= ~(1 << pinpos);
                GPIOx->OTYPER |= OTYPE << pinpos;
            }
            GPIOx->PUPDR &= ~(3 << (pinpos * 2));
            GPIOx->PUPDR |= PUPD << (pinpos * 2);
        }
    }
}

// 常用内核函数
void WFI_SET(void) { __WFI(); }
void INTX_DISABLE(void) { __disable_irq(); }
void INTX_ENABLE(void) { __enable_irq(); }
void Sys_Soft_Reset(void) { SCB->AIRCR = 0X05FA0000 | (u32)0x04; }

/**
 * @brief  系统时钟初始化至 170MHz
 * @note   使用 HSI (16MHz) 作为时钟源
 * 计算公式: 16MHz / M(4) * N(85) / R(2) = 170MHz
 */
void SystemClock_Config(void) {
    // 1. 使能电源接口时钟并配置电压范围 (Range 1)
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    
    // 配置为 Range 1 Boost 模式以支持 170MHz
    // 先清除位，再设置为 Range 1 (01)
    PWR->CR1 &= ~PWR_CR1_VOS;
    PWR->CR1 |= (1 << PWR_CR1_VOS_Pos);
    
    // 等待电压稳定
    while (PWR->SR2 & PWR_SR2_VOSF);

    // 2. 开启 HSI 并等待稳定
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // 3. 配置 Flash 等待周期 (170MHz 需要 4WS)
    // 同时开启预取指 (Prefetch) 提升指令执行效率
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN;

    // 4. 配置 PLL: 源选择 HSI, M=4, N=85, R=2
    // 清除现有的 PLL 配置
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | 
                      RCC_PLLCFGR_PLLN   | RCC_PLLCFGR_PLLR);
    
    // 设置参数
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI); // 选择 HSI (16MHz)
    RCC->PLLCFGR |= (3 << RCC_PLLCFGR_PLLM_Pos);   // M = 4 (写入值 = 分频数-1)
    RCC->PLLCFGR |= (85 << RCC_PLLCFGR_PLLN_Pos);  // N = 85
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLR_Pos);   // R = 2 (00 = /2)
    
    // 必须使能 PLLR 输出
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    // 5. 开启 PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 6. 切换系统时钟源为 PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 7. 配置总线分频 (HCLK=170MHz, PCLK1=170MHz, PCLK2=170MHz)
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); // 使能FPU
}




