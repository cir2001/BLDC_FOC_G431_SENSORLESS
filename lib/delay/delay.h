#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  
////////////////////////////////////////////////////////////////////////////////// 

/**
 * @brief  初始化延时函数
 * @param  SYSCLK: 系统时钟频率 (单位 MHz, 如 170)
 */
void delay_init(u8 SYSCLK);

/**
 * @brief  微秒级延时
 * @param  nus: 延时时长 (0 ~ 788,000us)
 */
void delay_us(u32 nus);

/**
 * @brief  毫秒级延时
 * @param  nms: 延时时长 (0 ~ 65535ms)
 */
void delay_ms(u16 nms);


#endif

