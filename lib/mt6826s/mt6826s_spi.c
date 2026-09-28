#include "mt6826s_spi.h"
#include <stdio.h>
#include "delay.h"
//===============================================
//
//===============================================
/**
 * @brief 简单的微秒级延迟 (针对 170MHz 频率优化)
 */
static void SPI_Delay(void) {
    for (volatile int i = 0; i < 300; i++); 
}
//--------------------------------------------------
static inline void DelayShort(void) {
    __NOP(); __NOP(); __NOP(); __NOP();
}
/**
 * @brief 初始化模拟 SPI 引脚
 */
void MT6826S_SPI_Init(void) {
    // 1. 开启 GPIOA, GPIOB, GPIOC 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // 2. 配置 NSS (PC10 输出推挽高速)
    GPIOC->MODER   &= ~(GPIO_MODER_MODE10_Msk);
    GPIOC->MODER   |=  (GPIO_MODER_MODE10_0);  // 输出模式
    GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT10);     // 推挽
    GPIOC->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0);  // 高速

    // 3. 配置 SCK (PA11 输出推挽高速)
    GPIOA->MODER   &= ~(GPIO_MODER_MODE11_Msk);
    GPIOA->MODER   |=  (GPIO_MODER_MODE11_0);
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT11);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED11_0);

    // 4. 配置 MOSI (PB3 输出推挽高速)
    GPIOB->MODER   &= ~(GPIO_MODER_MODE3_Msk);
    GPIOB->MODER   |=  (GPIO_MODER_MODE3_0);
    GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT3);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0);

    
    // 5. 配置 MISO (PA15 输入 + 上拉)
    GPIOA->MODER   &= ~(GPIO_MODER_MODE15_Msk);               // 输入模式 (00)
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD15_Msk);
    // GPIOA->PUPDR   |=  (GPIO_PUPDR_PUPD15_0);                 // 上拉 (01)

    // 初始状态
     AS_NSS_H;
    AS_SCK_H;
    AS_MOSI_H;

    delay_ms(50);  // 上电稳定时间
}

/**
 * @brief 模拟 SPI 16位数据传输
 */
uint8_t SPI_Transfer(uint8_t data_out){
    uint8_t data_in = 0;

    for (int i = 7; i >= 0; i--) {
        if (data_out & (1 << i)) {
            AS_MOSI_H;
        } else {
            AS_MOSI_L;
        }
        DelayShort();           // MOSI 稳定
        AS_SCK_L;               // 下降沿：从机采样 MOSI
        SPI_Delay();            // 低电平宽度

        AS_SCK_H;               // 上升沿：MISO 有效
        if (AS_MISO_IN) {
            data_in |= (1 << i);
        }
        SPI_Delay();            // 高电平宽度
    }
    return data_in;
}
//===============================================
// 写寄存器（24位帧）
//===============================================
void MT6826S_WriteReg(uint16_t addr, uint8_t data) {
    AS_NSS_L;
    DelayShort();

    // 命令 0110 (0x6) + 地址高4位 
    SPI_Transfer(0x60 | ((addr >> 8) & 0x0F));
    SPI_Transfer(addr & 0xFF);
    SPI_Transfer(data); // 写入数据

    DelayShort();
    AS_NSS_H;
}
//===============================================
// 读寄存器（24位帧）
// 返回低 8 位数据（MT6826 大部分寄存器是 8 位）
//===============================================
uint8_t MT6826S_ReadReg(uint16_t addr) {
    uint8_t rx[3];
    AS_NSS_L;
    DelayShort();

    // 命令 0011 (0x3) + 地址高4位 
    SPI_Transfer(0x30 | ((addr >> 8) & 0x0F));
    SPI_Transfer(addr & 0xFF);         // 地址低8位
    rx[2] = SPI_Transfer(0x00);        // 第3字节才是数据 

    DelayShort();
    AS_NSS_H;
    return rx[2];
}
//===============================================
// 推荐：Burst Read Angle（最常用、最快）
// 返回 15位 原始角度值 (0~32767)
//===============================================
uint16_t MT6826S_ReadAngle(void) {
    uint8_t rx[4];
    AS_NSS_L; // 下降沿锁存当前角度 [cite: 1986]
    DelayShort();

    // 连读命令 1010 (0xA) + 起始地址 0x003 [cite: 1985]
    SPI_Transfer(0xA0); 
    SPI_Transfer(0x03); 
    rx[2] = SPI_Transfer(0x00); // 0x003 的数据 (ANGLE[14:7]) [cite: 1966]
    rx[3] = SPI_Transfer(0x00); // 0x004 的数据 (ANGLE[6:0]) [cite: 1966]

    AS_NSS_H;

    // 根据手册：0x003 是高 8 位，0x004 是低 7 位 [cite: 1966]
    uint16_t angle = ((uint16_t)rx[2] << 7) | (rx[3] >> 1);
    return angle;
}
/**
 * @brief 烧录 EEPROM 命令
 * 将当前所有 RAM 寄存器的值永久保存到 EEPROM [cite: 1872]
 */
uint8_t MT6826S_BurnEEPROM(void) {
    uint8_t ack = 0;
    AS_NSS_L;
    DelayShort();

    // 烧录命令 1100 (0xC) + 地址字段全 0 [cite: 1815, 1872]
    SPI_Transfer(0xC0); 
    SPI_Transfer(0x00);
    ack = SPI_Transfer(0x00); // 正常应返回 0x55 [cite: 1872]

    DelayShort();
    AS_NSS_H;
    
    // 烧录需要时间，建议延时并等待确认码
    delay_ms(100); 
    return ack;
}


