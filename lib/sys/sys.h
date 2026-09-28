#ifndef __SYS_H
#define __SYS_H	 
#include <stm32g4xx.h>
#include <stdint.h>     // 引入标准整数类型
//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 
// 重新定义旧代码习惯使用的简写类型
typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

typedef int32_t   s32;
typedef int16_t   s16;
typedef int8_t    s8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;															    
	 
// GPIO口操作宏定义 (G431不建议使用位带，直接操作寄存器更快)
#define PAout(n)   (GPIOA->ODR ^= (1U << (n))) 
#define PAin(n)    ((GPIOA->IDR >> (n)) & 1U)
#define PBout(n)   (GPIOB->ODR ^= (1U << (n)))
#define PBin(n)    ((GPIOB->IDR >> (n)) & 1U)
#define PCout(n)   (GPIOC->ODR ^= (1U << (n)))
#define PCin(n)    ((GPIOC->IDR >> (n)) & 1U)
////////////////////////////////////////////////////////////////////////////////// 
//Ex_NVIC_Config专用定义
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6 
#define GPIO_H 				7 
#define GPIO_I 				8 

#define FTIR   				1  		//下降沿触发
#define RTIR   				2  		//上升沿触发

//GPIO设置专用宏定义
#define GPIO_MODE_IN    	0		//普通输入模式
#define GPIO_MODE_OUT		1		//普通输出模式
#define GPIO_MODE_AF		2		//AF功能模式
#define GPIO_MODE_AIN		3		//模拟输入模式

#define GPIO_SPEED_2M		0		//GPIO速度2Mhz
#define GPIO_SPEED_25M		1		//GPIO速度25Mhz
#define GPIO_SPEED_50M		2		//GPIO速度50Mhz
#define GPIO_SPEED_170M		3		//GPIO速度100Mhz

#define GPIO_PUPD_NONE		0		//不带上下拉
#define GPIO_PUPD_PU		1		//上拉
#define GPIO_PUPD_PD		2		//下拉
#define GPIO_PUPD_RES		3		//保留 

#define GPIO_OTYPE_PP		0		//推挽输出
#define GPIO_OTYPE_OD		1		//开漏输出 

//GPIO引脚编号定义
#define PIN0				1<<0
#define PIN1				1<<1
#define PIN2				1<<2
#define PIN3				1<<3
#define PIN4				1<<4
#define PIN5				1<<5
#define PIN6				1<<6
#define PIN7				1<<7
#define PIN8				1<<8
#define PIN9				1<<9
#define PIN10				1<<10
#define PIN11				1<<11
#define PIN12				1<<12
#define PIN13				1<<13
#define PIN14				1<<14
#define PIN15				1<<15 
////////////////////////////////////////////////////////////////////////////////// 

// --- 系统时钟相关宏定义 ---
#define SYS_CLK_FREQ    170000000UL  // 170MHz
#define AHB_CLK_FREQ    170000000UL
#define APB1_CLK_FREQ   170000000UL
#define APB2_CLK_FREQ   170000000UL

// --- 函数声明 ---

/**
 * @brief 初始化系统时钟到 170MHz (使用 HSI)
 */
void SystemClock_Config(void);// G431通常固定170M，不再需要传4个参数

// 函数声明 
void Sys_Soft_Reset(void);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group);
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD);
void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx);
void INTX_DISABLE(void);
void INTX_ENABLE(void);
void WFI_SET(void);


#endif


