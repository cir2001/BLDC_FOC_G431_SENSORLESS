#ifndef __MT6826S_SPI_H
#define __MT6826S_SPI_H

#include <stm32g4xx.h>
#include "sys.h"
//================================================================
/**
 * @brief 引脚操作宏定义 (寄存器方式)
 */
#define AS_NSS_H    (GPIOC->BSRR = GPIO_BSRR_BS10)
#define AS_NSS_L    (GPIOC->BSRR = GPIO_BSRR_BR10)
#define AS_SCK_H    (GPIOA->BSRR = GPIO_BSRR_BS11)
#define AS_SCK_L    (GPIOA->BSRR = GPIO_BSRR_BR11)

#define AS_MOSI_H   (GPIOB->BSRR = GPIO_BSRR_BS3)
#define AS_MOSI_L   (GPIOB->BSRR = GPIO_BSRR_BR3)

#define AS_MISO_IN  (GPIOA->IDR & GPIO_IDR_ID15)
//=================================================================
// 函数原型
void MT6826S_SPI_Init(void);
void MT6826S_WriteReg(uint16_t addr, uint8_t data);
uint8_t MT6826S_ReadReg(uint16_t addr);

uint16_t MT6826S_ReadAngle(void);
uint8_t MT6826S_ReadStatus(void);

uint8_t MT6826S_BurnEEPROM(void);

#endif /* __MT6826S_SPI_H */



