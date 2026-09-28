#include "button.h"

void Button_Init(void) {
    // 1. 开启 GPIOC 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // 2. 配置 PC10 为输入模式 (MODER[21:20] = 00)
    GPIOC->MODER &= ~(GPIO_MODER_MODE10);

    // 3. 配置 PC10 为上拉电阻 (PUPDR[21:20] = 01)
    // 必须上拉，否则引脚电平不确定，导致检测不到按下
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOC->PUPDR |= (1U << GPIO_PUPDR_PUPD10_Pos);
}
// 返回 1 表示按下，0 表示未按下
uint8_t Is_Button_Pressed(void) {
    if (!(GPIOC->IDR & (1U << 10))) { // 检测到低电平
        delay_ms(20); // 软件消抖
        if (!(GPIOC->IDR & (1U << 10))) {
            return 1;
        }
    }
    return 0;
}

